// SPDX-License-Identifier: Apache-2.0
/* Copyright 2018-2019 IBM Corp. */

#include <hiomap.h>
#include <inttypes.h>
#include <ipmi.h>
#include <lpc.h>
#include <mem_region-malloc.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <lock.h>
#include <debug_descriptor.h>
#include <timebase.h>
#include <cpu.h>

#include <ccan/container_of/container_of.h>

#include "errors.h"
#include "ipmi-hiomap.h"

#define CMD_OP_HIOMAP_EVENT	0x0f

#define hiomap_trace(fmt, a...)    prlog(PR_TRACE,"HIOMAP: %s " fmt, __func__, ## a)
#define hiomap_info(fmt, a...)     prlog(PR_INFO,"HIOMAP: %s " fmt, __func__, ## a)
#define hiomap_notice(fmt, a...)   prlog(PR_NOTICE,"HIOMAP: %s " fmt, __func__, ## a)
#define hiomap_warn(fmt, a...)     prlog(PR_WARNING,"HIOMAP: %s " fmt, __func__, ## a)
#define hiomap_error(fmt, a...)    prlog(PR_ERR,"HIOMAP: %s " fmt, __func__, ## a)

struct ipmi_hiomap_result {
	struct ipmi_hiomap *ctx;
	int16_t cc;
};

static struct hiomap_v2_create_window *window_parms;

static inline uint32_t blocks_to_bytes(struct ipmi_hiomap *ctx, uint16_t blocks)
{
	return blocks << ctx->block_size_shift;
}

static inline uint16_t bytes_to_blocks(struct ipmi_hiomap *ctx, uint32_t bytes)
{
	return bytes >> ctx->block_size_shift;
}

static inline uint16_t bytes_to_blocks_align_up(struct ipmi_hiomap *ctx,
						uint32_t pos, uint32_t len)
{
	uint32_t block_size = 1 << ctx->block_size_shift;
	uint32_t delta = pos & (block_size - 1);
	uint32_t aligned = ALIGN_UP((len + delta), block_size);
	uint32_t blocks = aligned >> ctx->block_size_shift;
	/* Our protocol can handle block count < sizeof(u16) */
	uint32_t mask = ((1 << 16) - 1);

	assert(!(blocks & ~mask));

	return blocks & mask;
}

/* Call under ctx->lock */
static int hiomap_protocol_ready(struct ipmi_hiomap *ctx)
{
	hiomap_trace("ctx->bmc_state=%i\n", ctx->bmc_state);
	if (!(ctx->bmc_state & HIOMAP_E_DAEMON_READY)) {
		hiomap_notice("FLASH_ERR_DEVICE_GONE\n");
		return FLASH_ERR_DEVICE_GONE;
	}
	if (ctx->bmc_state & HIOMAP_E_FLASH_LOST) {
		hiomap_notice("HIOMAP_E_FLASH_LOST\n");
		return FLASH_ERR_AGAIN;
	}

	return 0;
}

static int hiomap_queue_msg(struct ipmi_hiomap *ctx, struct ipmi_msg *msg)
{
	int rc;
	int bl_flags;

	lock(&ctx->lock);
	bl_flags = ctx->bl.flags;
	unlock(&ctx->lock);

	/*
	 * during boot caution to stay duration within skiboot
	 * no exit re-entry due to poller conflicts with synchronous window moves
	 * asynchronous usage intended for opal_flash_op and flash_poll paths
	 * this comment mostly for awareness that during boot any future
	 * modifications for transition from booting (sync) to async ops
	 * available need to make sure to take proper care
	 */

	/*
	 * There's an unavoidable TOCTOU race here with the BMC sending an
	 * event saying it's no-longer available right after we test but before
	 * we call into the IPMI stack to send the message.
	 * hiomap_queue_msg_sync() exists to capture the race in a single
	 * location.
	 */

	if ((opal_booting()) || (!(bl_flags & ASYNC_REQUIRED))) {
		lock(&ctx->lock);
		rc = hiomap_protocol_ready(ctx);
		unlock(&ctx->lock);
		if (rc) {
			ipmi_free_msg(msg);
			return rc;
		}
		hiomap_trace("SENDING SYNC\n");
		ipmi_queue_msg_sync(msg);
	} else {
		hiomap_trace("SENDING ASYNC\n");
		rc = ipmi_queue_msg(msg);
	}

	return rc;
}

/* Call under ctx->lock */
static int hiomap_window_valid(struct ipmi_hiomap *ctx, uint64_t pos,
			        uint64_t len)
{
	if (ctx->bmc_state & HIOMAP_E_FLASH_LOST) {
		hiomap_notice("HIOMAP_E_FLASH_LOST\n");
		return FLASH_ERR_AGAIN;
	}
	if (ctx->bmc_state & HIOMAP_E_PROTOCOL_RESET) {
		hiomap_notice("HIOMAP_E_PROTOCOL_RESET\n");
		return FLASH_ERR_AGAIN;
	}
	if (ctx->bmc_state & HIOMAP_E_WINDOW_RESET) {
		hiomap_notice("HIOMAP_E_WINDOW_RESET\n");
		return FLASH_ERR_AGAIN;
	}
	if (ctx->window_state == closed_window) {
		hiomap_trace("window_state=closed_window\n");
		return FLASH_ERR_PARM_ERROR;
	}
	if (pos < ctx->current.cur_pos) {
		hiomap_trace("we need to move the window pos=%llu < ctx->current.cur_pos=0x%x\n",
			pos, ctx->current.cur_pos);
		return FLASH_ERR_PARM_ERROR;
	}
	if ((pos + len) > (ctx->current.cur_pos + ctx->current.size)) {
		/*
		 * we will compensate the proper values in the caller
		 * which manages the transaction due to chunk size that
		 * may be straddling the current window so store values
		 */
		if ((pos + ctx->current.size) <= (ctx->current.cur_pos + ctx->current.size)) {
			hiomap_trace("OK pos=%llu "
				"ctx->current.size=0x%x "
				"ctx->current.cur_pos=0x%x\n",
				pos,
				ctx->current.size,
				ctx->current.cur_pos);
		} else {
			hiomap_trace("CHECKING further pos=%llu "
				"for len=%llu ctx->current.size=0x%x "
				"ctx->current.cur_pos=0x%x\n",
				pos,
				len,
				ctx->current.size,
				ctx->current.cur_pos);
			if ((pos + ctx->current.adjusted_window_size) <= (ctx->current.cur_pos + ctx->current.size)) {
				hiomap_trace("OK use ADJUSTED pos=%llu "
					"adjusted_len=%i for len=%llu "
					"ctx->current.size=0x%x "
					"ctx->current.cur_pos=0x%x\n",
					pos,
					ctx->current.adjusted_window_size,
					len,
					ctx->current.size,
					ctx->current.cur_pos);
			} else {
				hiomap_trace("we need to MOVE the window\n");
				return FLASH_ERR_PARM_ERROR;
			}
		}
	}

	hiomap_trace("ALL GOOD, no move needed\n");
	return 0;
}

static void move_cb(struct ipmi_msg *msg)
{
	/*
	 * we leave the move_cb outside of the ipmi_hiomap_cmd_cb
	 * based on the command we need to special close the window
	 * async request may take minutes 
	 */

	struct ipmi_hiomap_result *res = msg->user_data;
	struct ipmi_hiomap *ctx = res->ctx;
	/*
	 * only a few iterations to try for lock
	 * contention is probably hiomap_window_move trying to setup again
	 */
	int lock_try_counter = 10;

	if ((msg->resp_size != 8) || (msg->cc != IPMI_CC_NO_ERROR) || (msg->data[1] != ctx->inflight_seq)) {
		hiomap_trace("Command %u (4=READ 6=WRITE): "
			"Unexpected results to check: response size we "
			"expect 8 but received %u, ipmi cc=%d "
			"(should be zero), expected ipmi seq %i but got "
			"wrong ipmi seq %i\n",
			msg->data[0],
			msg->resp_size,
			msg->cc,
			ctx->inflight_seq,
			msg->data[1]);
		lock(&ctx->lock);
		ctx->cc = OPAL_HARDWARE;
		ctx->window_state = closed_window;
		goto out;
	} else {
		hiomap_trace("Entered for %s window from "
			"OLD block pos 0x%x for 0x%x bytes at "
			"lpc_addr 0x%x ipmi seq=%i\n",
			(msg->data[0] == HIOMAP_C_CREATE_READ_WINDOW) ? "read" : "write",
			ctx->current.cur_pos,
			ctx->current.size,
			ctx->current.lpc_addr,
			ctx->inflight_seq);
	}

	window_parms = (struct hiomap_v2_create_window *)&msg->data[2];

	/*
	 * due to optimization issues the following logic
	 * allows the execution of the logic "x" times
	 */
	while (!try_lock(&ctx->lock)) {
		--lock_try_counter;
		if (lock_try_counter == 0) {
			break;
		}
	}
	if (lock_try_counter == 0) {
		/*
		 * we cannot get the lock, but update anyway
		 * because we cannot communicate this completion
		 * and someone will need to retry
		 * contention usually with handle_events or window_move
		 * this code path is the critical path that will open the window
		 */
		ctx->window_state = closed_window;
		ctx->cc = OPAL_PARAMETER;
		goto out2;
	}

	/* If here, we got the lock, cc consumed higher up so need in ctx */

	ctx->cc = IPMI_CC_NO_ERROR;
	ctx->current.lpc_addr =
		blocks_to_bytes(ctx, le16_to_cpu(window_parms->lpc_addr));
	ctx->current.size =
		blocks_to_bytes(ctx, le16_to_cpu(window_parms->size));
	ctx->current.cur_pos =
		blocks_to_bytes(ctx, le16_to_cpu(window_parms->offset));
	/* refresh to current */
	ctx->current.adjusted_window_size = ctx->current.size;

	/*
	 * now that we have moved stuff the values
	 * we may need to compensate the proper values in the caller
	 * which manages the transaction due to chunk size that
	 * may be straddling the current window so store values
	 */
	*ctx->active_size = ctx->requested_len;

	/*
	 * Is length past the end of the window?
	 * if this condition happens it can cause the async.retry_counter to fail
	 */
	if ((ctx->requested_pos + ctx->requested_len) > (ctx->current.cur_pos + ctx->current.size)) {
		/*
		 * Adjust size to meet current window
		 * active_size goes back to caller,
		 * but caller may expire and we need to store for future use
		 */
		*ctx->active_size = (ctx->current.cur_pos + ctx->current.size) - ctx->requested_pos;
		ctx->current.adjusted_window_size = (ctx->current.cur_pos + ctx->current.size) - ctx->requested_pos;
		hiomap_trace("VALID MOVE ADJUSTMENT "
			"*ctx->active_size=%llu "
			"ctx->requested_pos=%llu "
			"ctx->current.adjusted_window_size=%i\n",
			*ctx->active_size,
			ctx->requested_pos,
			ctx->current.adjusted_window_size);
	}

	if (ctx->requested_len != 0 && *ctx->active_size == 0) {
		hiomap_notice("Invalid window properties: len: %llu, size: %llu\n",
			ctx->requested_len, *ctx->active_size);
		ctx->cc = OPAL_PARAMETER;
		ctx->window_state = closed_window;
		goto out;
	}

	if (msg->data[0] == HIOMAP_C_CREATE_READ_WINDOW)
		ctx->window_state = read_window;
	else
		ctx->window_state = write_window;

	hiomap_trace("Opened %s window to NEW block pos 0x%x for 0x%x bytes "
		"at lpc_addr 0x%x ipmi seq=%i active size=%llu "
		"adjusted_window_size=%i\n",
		(msg->data[0] == HIOMAP_C_CREATE_READ_WINDOW) ? "read" : "write",
		ctx->current.cur_pos,
		ctx->current.size,
		ctx->current.lpc_addr,
		ctx->inflight_seq,
		*ctx->active_size,
		ctx->current.adjusted_window_size);

out:	hiomap_trace("Exiting the move window callback "
		"transaction ipmi seq=%i\n",
		ctx->inflight_seq);
	unlock(&ctx->lock);
out2:	ipmi_free_msg(msg);
}

static void ipmi_hiomap_cmd_cb(struct ipmi_msg *msg)
{
	struct ipmi_hiomap_result *res = msg->user_data;
	struct ipmi_hiomap *ctx = res->ctx;

	res->cc = msg->cc;
	if (msg->cc != IPMI_CC_NO_ERROR) {
		ipmi_free_msg(msg);
		return;
	}

	/* We at least need the command and sequence */
	if (msg->resp_size < 2) {
		hiomap_error("Illegal response size: %u\n", msg->resp_size);
		res->cc = IPMI_ERR_UNSPECIFIED;
		ipmi_free_msg(msg);
		return;
	}

	if (msg->data[1] != ctx->inflight_seq) {
		hiomap_trace("Unmatched ipmi sequence number: wanted %u got %u\n",
			ctx->inflight_seq,
			msg->data[1]);
		res->cc = IPMI_ERR_UNSPECIFIED;
		ipmi_free_msg(msg);
		return;
	}

	switch (msg->data[0]) {
	case HIOMAP_C_GET_INFO:
	{
		struct hiomap_v2_info *parms;

		ctx->cc = IPMI_CC_NO_ERROR;
		if (msg->resp_size != 6) {
			hiomap_error("%u: Unexpected response size: %u\n", msg->data[0],
				msg->resp_size);
			res->cc = IPMI_ERR_UNSPECIFIED;
			break;
		}

		ctx->version = msg->data[2];
		if (ctx->version < 2) {
			hiomap_error("Failed to negotiate protocol v2 or higher: %d\n",
				ctx->version);
			res->cc = IPMI_ERR_UNSPECIFIED;
			break;
		}

		parms = (struct hiomap_v2_info *)&msg->data[3];
		ctx->block_size_shift = parms->block_size_shift;
		ctx->timeout = le16_to_cpu(parms->timeout);
		break;
	}
	case HIOMAP_C_GET_FLASH_INFO:
	{
		struct hiomap_v2_flash_info *parms;

		ctx->cc = IPMI_CC_NO_ERROR;
		if (msg->resp_size != 6) {
			hiomap_error("%u: Unexpected response size: %u\n", msg->data[0],
				msg->resp_size);
			res->cc = IPMI_ERR_UNSPECIFIED;
			break;
		}

		parms = (struct hiomap_v2_flash_info *)&msg->data[2];
		ctx->total_size =
			blocks_to_bytes(ctx, le16_to_cpu(parms->total_size));
		ctx->erase_granule =
			blocks_to_bytes(ctx, le16_to_cpu(parms->erase_granule));
		break;
	}
	case HIOMAP_C_MARK_DIRTY:
	case HIOMAP_C_FLUSH:
	case HIOMAP_C_ACK:
	case HIOMAP_C_ERASE:
	case HIOMAP_C_RESET:
		if (msg->resp_size != 2) {
			hiomap_error("%u: Unexpected response size: %u\n",
				msg->data[0],
				msg->resp_size);
			res->cc = IPMI_ERR_UNSPECIFIED;
			ctx->cc = OPAL_HARDWARE;
			break;
		} else {
			hiomap_trace("Command=%u 1=RESET 7=DIRTY 8=FLUSH 9=ACK 10=ERASE ipmi seq=%u ctx->inflight_seq=%u\n",
				msg->data[0],
				msg->data[1],
				ctx->inflight_seq);
			ctx->cc = IPMI_CC_NO_ERROR;
		}
		break;
	default:
		hiomap_warn("Unimplemented command handler: %u\n",
		      msg->data[0]);
		break;
	};
	ipmi_free_msg(msg);
}

static void hiomap_init(struct ipmi_hiomap *ctx)
{
	/*
	 * Speculatively mark the daemon as available so we attempt to perform
	 * the handshake without immediately bailing out.
	 */
	lock(&ctx->lock);
	ctx->bmc_state = HIOMAP_E_DAEMON_READY;
	unlock(&ctx->lock);
}

static int hiomap_wait_for_cc(struct ipmi_hiomap *ctx, int *cc, uint8_t *seq, uint64_t ticks)
{
	uint64_t now;
	uint64_t start_time;
	uint64_t wait_time;
	uint64_t ipmi_hiomap_ticks;
	uint64_t timeout_counter;
	int rc;

	hiomap_trace("Start wait for cc ipmi seq=%i *cc=%i ticks=%llu\n", *seq, *cc, ticks);
	rc = 0;
	if (this_cpu()->tb_invalid) {
		/*
		 * SYNC paths already have *cc success
		 * ASYNC will RE-QUEUE and retry
		 * we just need to skip the tb logic handling
		 * we need to close the window to have the logic try the move again
		 */
		if (*cc != IPMI_CC_NO_ERROR) {
			lock(&ctx->lock);
			ctx->window_state = closed_window;
			++ctx->missed_cc_count;
			hiomap_notice("tb_invalid, CLOSING WINDOW for cc "
				"ipmi seq=%i ctx->missed_cc_count=%i\n",
				*seq, ctx->missed_cc_count);
			unlock(&ctx->lock);
			rc = FLASH_ERR_ASYNC_WORK;
		}
		hiomap_notice("tb_invalid, hopefully this will "
			"retry/recover rc=%i\n",
			rc);
		return rc;
	}
	start_time = mftb();
	now = mftb();
	wait_time = tb_to_msecs(now - start_time);
	timeout_counter = 0;

	if (ticks != 0) {
		ipmi_hiomap_ticks = ticks;
	} else {
		ipmi_hiomap_ticks = IPMI_HIOMAP_TICKS;
	}

	hiomap_trace("wait_time=%llu ipmi_hiomap_ticks=%llu ipmi seq=%i "
			"ctx->missed_cc_count=%i\n",
		wait_time, ticks, *seq, ctx->missed_cc_count);
	/*
	 * due to optimization issues the following logic
	 * allows the execution of the logic "x" times
	 */
	while (wait_time < ipmi_hiomap_ticks) {
		++timeout_counter;
		if (timeout_counter % IPMI_SKIP_INC == 0) {
			now = mftb();
			wait_time = tb_to_msecs(now - start_time);
		}
		if (*cc == IPMI_CC_NO_ERROR) {
			hiomap_trace("Break cc ipmi seq=%i "
				"ctx->missed_cc_count=%i\n",
				*seq, ctx->missed_cc_count);
			break;
		}
	}
	hiomap_trace("Status CHECK wait_time=%llu *cc=%i "
		"ipmi seq=%i ctx->missed_cc_count=%i\n",
		wait_time, *cc, *seq, ctx->missed_cc_count);
	if (*cc != IPMI_CC_NO_ERROR) {
		lock(&ctx->lock);
		ctx->window_state = closed_window;
		++ctx->missed_cc_count;
		hiomap_trace("CLOSING WINDOW for cc ipmi seq=%i "
			"ctx->missed_cc_count=%i\n",
			*seq, ctx->missed_cc_count);
		unlock(&ctx->lock);
		rc = FLASH_ERR_ASYNC_WORK;
	}

	hiomap_trace("Stop wait for *cc=%i ipmi seq=%i "
		"ctx->missed_cc_count=%i\n",
		*cc, *seq, ctx->missed_cc_count);
	return rc;

}

static int hiomap_get_info(struct ipmi_hiomap *ctx)
{
	static struct ipmi_hiomap_result info_res;
	unsigned char req[3];
	struct ipmi_msg *msg;
	uint8_t info_seq;
	int orig_flags;
	int tmp_sync_flags;
	int rc;

	info_res.ctx = ctx;
	info_res.cc = -1;

	lock(&ctx->lock);
	orig_flags = ctx->bl.flags;
	/*
	 * clear out async to always do sync
	 * we may be doing this under ASYNC_REQUIRED
	 * so we need to temporarily undo
	 */
	tmp_sync_flags = ctx->bl.flags &= ~ASYNC_REQUIRED;
	ctx->bl.flags = tmp_sync_flags;
	ctx->cc = -1;
	info_seq = ++ctx->seq;
	ctx->inflight_seq = info_seq;
	unlock(&ctx->lock);

	/* Negotiate protocol version 2 */
	req[0] = HIOMAP_C_GET_INFO;
	req[1] = info_seq;
	req[2] = HIOMAP_V2;

	msg = ipmi_mkmsg(IPMI_DEFAULT_INTERFACE,
		         bmc_platform->sw->ipmi_oem_hiomap_cmd,
			 ipmi_hiomap_cmd_cb, &info_res, req, sizeof(req), 6);

	rc = hiomap_queue_msg(ctx, msg);
	lock(&ctx->lock);
	ctx->bl.flags = orig_flags;
	unlock(&ctx->lock);
	if (rc)
		return rc;

	rc = hiomap_wait_for_cc(ctx, &ctx->cc, &ctx->inflight_seq, IPMI_ACK_DEFAULT);

	if (rc) {
		hiomap_trace("hiomap_wait_for_cc failed: rc=%d ticks=%i\n",
			rc, IPMI_ACK_DEFAULT);
	}

	return rc;
}

static int hiomap_get_flash_info(struct ipmi_hiomap *ctx)
{
	static struct ipmi_hiomap_result flash_info_res;
	unsigned char req[2];
	struct ipmi_msg *msg;
	uint8_t flash_info_seq;
	int orig_flags;
	int tmp_sync_flags;
	int rc;

	flash_info_res.ctx = ctx;
	flash_info_res.cc = -1;

	lock(&ctx->lock);
	orig_flags = ctx->bl.flags;
	/*
	 * clear out async to always do sync
	 * we may be doing this under ASYNC_REQUIRED
	 * so we need to temporarily undo
	 */
	tmp_sync_flags = ctx->bl.flags &= ~ASYNC_REQUIRED;
	ctx->bl.flags = tmp_sync_flags;
	ctx->cc = -1;
	flash_info_seq = ++ctx->seq;
	ctx->inflight_seq = flash_info_seq;
	unlock(&ctx->lock);

	req[0] = HIOMAP_C_GET_FLASH_INFO;
	req[1] = flash_info_seq;
	msg = ipmi_mkmsg(IPMI_DEFAULT_INTERFACE,
		         bmc_platform->sw->ipmi_oem_hiomap_cmd,
			 ipmi_hiomap_cmd_cb, &flash_info_res, req, sizeof(req), 2 + 2 + 2);

	rc = hiomap_queue_msg(ctx, msg);
	lock(&ctx->lock);
	ctx->bl.flags = orig_flags;
	unlock(&ctx->lock);
	if (rc)
		return rc;

	rc = hiomap_wait_for_cc(ctx, &ctx->cc, &ctx->inflight_seq, IPMI_ACK_DEFAULT);
	if (rc) {
		hiomap_trace("hiomap_wait_for_cc failed: rc=%d ticks=%i\n",
			rc, IPMI_ACK_DEFAULT);
	}

	return rc;
}

static int hiomap_window_move(struct ipmi_hiomap *ctx, uint8_t command,
			      uint64_t pos, uint64_t len, uint64_t *size)
{
	enum lpc_window_state want_state;
	struct hiomap_v2_range *range;
	static struct ipmi_hiomap_result move_res;
	unsigned char req[6];
	struct ipmi_msg *msg;
	uint8_t move_seq;
	bool valid_state;
	bool is_read;
	int rc;

	move_res.ctx = ctx;
	move_res.cc = -1;
	is_read = (command == HIOMAP_C_CREATE_READ_WINDOW);
	want_state = is_read ? read_window : write_window;

	/* there will be lock contention between hiomap_window_move and move_cb */

	lock(&ctx->lock);
	ctx->cc = -1;

	if (ctx->bl.flags & IN_PROGRESS) {
		pos = ctx->tracking_pos;
		len = ctx->tracking_len;
	} else {
		ctx->tracking_pos = pos;
		ctx->tracking_len = len;
	}

	valid_state = want_state == ctx->window_state;
	rc = hiomap_window_valid(ctx, pos, len);

	if (valid_state && !rc) {
		/* if its valid stuff the proper maybe modified size */
		if ((pos + len) > (ctx->current.cur_pos + ctx->current.size)) {
			/* if we had bumped the adjusted_window_size down in move_cb */
			if ((ctx->current.adjusted_window_size != ctx->current.size)) {
				*size = ctx->current.adjusted_window_size;
			} else {
				*size = (ctx->current.cur_pos + ctx->current.size) - pos;
			}
		} else {
			*size = len;
		}
		ctx->cc = IPMI_CC_NO_ERROR;
		unlock(&ctx->lock);
		return 0;
	}

	ctx->window_state = moving_window;

	ctx->active_size = size;
	ctx->requested_pos = pos;
	ctx->requested_len = len;

	move_seq = ++ctx->seq;
	ctx->inflight_seq = move_seq;

	req[0] = command;
	req[1] = move_seq;

	unlock(&ctx->lock);

	range = (struct hiomap_v2_range *)&req[2];
	range->offset = cpu_to_le16(bytes_to_blocks(ctx, pos));
	range->size = cpu_to_le16(bytes_to_blocks_align_up(ctx, pos, len));

	msg = ipmi_mkmsg(IPMI_DEFAULT_INTERFACE,
		         bmc_platform->sw->ipmi_oem_hiomap_cmd,
			 move_cb, &move_res, req, sizeof(req),
			 2 + 2 + 2 + 2);

	rc = hiomap_queue_msg(ctx, msg);

	if (rc) {
		hiomap_notice("move queue msg failed: rc=%d\n", rc);
		return rc;
	}

	return 0;
}

static int hiomap_mark_dirty(struct ipmi_hiomap *ctx, uint64_t offset,
			      uint64_t size)
{
	struct hiomap_v2_range *range;
	enum lpc_window_state state;
	static struct ipmi_hiomap_result dirty_res;
	unsigned char req[6];
	struct ipmi_msg *msg;
	uint8_t dirty_seq;
	uint32_t pos;
	int rc;

	dirty_res.ctx = ctx;
	dirty_res.cc = -1;
	lock(&ctx->lock);
	state = ctx->window_state;
	dirty_seq = ++ctx->seq;
	ctx->inflight_seq = dirty_seq;
	ctx->cc = -1;
	unlock(&ctx->lock);

	if (state != write_window) {
		hiomap_notice("failed: state=%i\n", state);
		return FLASH_ERR_PARM_ERROR;
	}

	req[0] = HIOMAP_C_MARK_DIRTY;
	req[1] = dirty_seq;

	pos = offset - ctx->current.cur_pos;
	range = (struct hiomap_v2_range *)&req[2];
	range->offset = cpu_to_le16(bytes_to_blocks(ctx, pos));
	range->size = cpu_to_le16(bytes_to_blocks_align_up(ctx, pos, size));

	msg = ipmi_mkmsg(IPMI_DEFAULT_INTERFACE,
		         bmc_platform->sw->ipmi_oem_hiomap_cmd,
			 ipmi_hiomap_cmd_cb, &dirty_res, req, sizeof(req), 2);

	rc = hiomap_queue_msg(ctx, msg);

	if (rc) {
		hiomap_notice("dirty queue msg failed: rc=%d\n", rc);
		return rc;
	}

	hiomap_trace("Start to mark flash dirty at pos %llu size %llu bytes ipmi seq=%i\n",
		offset, size, dirty_seq);

	return 0;
}

static int hiomap_flush(struct ipmi_hiomap *ctx)
{
	enum lpc_window_state state;
	static struct ipmi_hiomap_result flush_res;
	unsigned char req[2];
	struct ipmi_msg *msg;
	uint8_t flush_seq;
	int rc;

	flush_res.ctx = ctx;
	flush_res.cc = -1;
	lock(&ctx->lock);
	state = ctx->window_state;
	flush_seq = ++ctx->seq;
	ctx->inflight_seq = flush_seq;
	ctx->cc = -1;
	unlock(&ctx->lock);

	if (state != write_window) {
		hiomap_notice("failed: state=%i\n", state);
		return FLASH_ERR_PARM_ERROR;
	}

	req[0] = HIOMAP_C_FLUSH;
	req[1] = flush_seq;

	msg = ipmi_mkmsg(IPMI_DEFAULT_INTERFACE,
		         bmc_platform->sw->ipmi_oem_hiomap_cmd,
			 ipmi_hiomap_cmd_cb, &flush_res, req, sizeof(req), 2);

	rc = hiomap_queue_msg(ctx, msg);

	if (rc) {
		hiomap_notice("flush queue msg failed: rc=%d\n", rc);
		return rc;
	}

	hiomap_trace("Start to flush writes ipmi seq=%i\n", flush_seq);

	return 0;
}

static int hiomap_ack(struct ipmi_hiomap *ctx, uint8_t ack)
{
	static struct ipmi_hiomap_result ack_res;
	unsigned char req[3];
	struct ipmi_msg *msg;
	uint8_t ack_seq;
	int orig_flags;
	int tmp_sync_flags;
	int rc;

	ack_res.ctx = ctx;
	ack_res.cc = -1;

	lock(&ctx->lock);
	orig_flags = ctx->bl.flags;
	/*
	 * clear out async to always do sync
	 * we may be doing this under ASYNC_REQUIRED
	 * so we need to temporarily undo
	 */
	tmp_sync_flags = ctx->bl.flags &= ~ASYNC_REQUIRED;
	ctx->bl.flags = tmp_sync_flags;
	ctx->cc = -1;
	ack_seq = ++ctx->seq;
	ctx->inflight_seq = ack_seq;
	unlock(&ctx->lock);

	req[0] = HIOMAP_C_ACK;
	req[1] = ack_seq;
	req[2] = ack;

	msg = ipmi_mkmsg(IPMI_DEFAULT_INTERFACE,
		         bmc_platform->sw->ipmi_oem_hiomap_cmd,
			 ipmi_hiomap_cmd_cb, &ack_res, req, sizeof(req), 2);

	hiomap_trace("SENDING req[1]=%i\n", req[1]);
	rc = hiomap_queue_msg(ctx, msg);
	lock(&ctx->lock);
	ctx->bl.flags = orig_flags;
	unlock(&ctx->lock);
	if (rc) {
		hiomap_notice("queue msg failed: rc=%d\n", rc);
		return rc;
	}

	rc = hiomap_wait_for_cc(ctx, &ctx->cc, &ctx->inflight_seq, IPMI_ACK_DEFAULT);
	if (rc) {
		hiomap_trace("hiomap_wait_for_cc failed: rc=%d ticks=%i\n",
			rc, IPMI_ACK_DEFAULT);
		return rc;
	}

	hiomap_notice("Acked events: 0x%x\n", ack);

	return 0;
}

static int hiomap_erase(struct ipmi_hiomap *ctx, uint64_t offset,
			 uint64_t size)
{
	struct hiomap_v2_range *range;
	enum lpc_window_state state;
	static struct ipmi_hiomap_result erase_res;
	unsigned char req[6];
	struct ipmi_msg *msg;
	uint8_t erase_seq;
	uint32_t pos;
	int rc;

	erase_res.ctx = ctx;
	erase_res.cc = -1;
	lock(&ctx->lock);
	state = ctx->window_state;
	erase_seq = ++ctx->seq;
	ctx->inflight_seq = erase_seq;
	ctx->cc = -1;
	unlock(&ctx->lock);

	if (state != write_window) {
		hiomap_notice("failed: state=%i\n", state);
		return FLASH_ERR_PARM_ERROR;
	}

	req[0] = HIOMAP_C_ERASE;
	req[1] = erase_seq;

	pos = offset - ctx->current.cur_pos;
	range = (struct hiomap_v2_range *)&req[2];
	range->offset = cpu_to_le16(bytes_to_blocks(ctx, pos));
	range->size = cpu_to_le16(bytes_to_blocks_align_up(ctx, pos, size));

	msg = ipmi_mkmsg(IPMI_DEFAULT_INTERFACE,
		         bmc_platform->sw->ipmi_oem_hiomap_cmd,
			 ipmi_hiomap_cmd_cb, &erase_res, req, sizeof(req), 2);

	rc = hiomap_queue_msg(ctx, msg);

	if (rc) {
		hiomap_notice("erase queue msg failed: rc=%d\n", rc);
		return rc;
	}

	hiomap_trace("Erasing flash at pos %llu for size %llu\n",
		offset, size);

	return 0;
}

static bool hiomap_reset(struct ipmi_hiomap *ctx)
{
	static struct ipmi_hiomap_result reset_res;
	unsigned char req[2];
	struct ipmi_msg *msg;
	uint8_t reset_seq;
	int orig_flags;
	int tmp_sync_flags;
	int rc;

	hiomap_notice("Reset ENTRY\n");
	reset_res.ctx = ctx;
	reset_res.cc = -1;

	lock(&ctx->lock);
	orig_flags = ctx->bl.flags;
	/*
	 * clear out async to always do sync
	 * we may be doing this under ASYNC_REQUIRED
	 * so we need to temporarily undo
	 */
	tmp_sync_flags = ctx->bl.flags &= ~ASYNC_REQUIRED;
	ctx->bl.flags = tmp_sync_flags;
	reset_seq = ++ctx->seq;
	ctx->cc = -1;
	ctx->inflight_seq = reset_seq;
	unlock(&ctx->lock);

	req[0] = HIOMAP_C_RESET;
	req[1] = reset_seq;
	msg = ipmi_mkmsg(IPMI_DEFAULT_INTERFACE,
		         bmc_platform->sw->ipmi_oem_hiomap_cmd,
			 ipmi_hiomap_cmd_cb, &reset_res, req, sizeof(req), 2);

	rc = hiomap_queue_msg(ctx, msg);
	lock(&ctx->lock);
	ctx->bl.flags = orig_flags;
	unlock(&ctx->lock);

	if (rc) {
		hiomap_notice("reset queue msg failed: rc=%d\n", rc);
		return false;
	}

	rc = hiomap_wait_for_cc(ctx, &ctx->cc, &ctx->inflight_seq, IPMI_ACK_DEFAULT);

	if (rc) {
		hiomap_notice("hiomap_wait_for_cc failed: rc=%d ticks=%i\n",
			rc, IPMI_ACK_DEFAULT);
		return false;
	}

	hiomap_notice("Reset EXIT\n");
	return true;
}

static void hiomap_event(uint8_t events, void *context)
{
	struct ipmi_hiomap *ctx = context;

	hiomap_notice("Received events: 0x%x ctx->bmc_state=%i\n",
		events,
		ctx->bmc_state);

	lock(&ctx->lock);
	ctx->bmc_state = events | (ctx->bmc_state & HIOMAP_E_ACK_MASK);
	hiomap_notice("Updated bmc_state Received events: 0x%x ctx->bmc_state=%i\n",
		events,
		ctx->bmc_state);
	unlock(&ctx->lock);
}

static int lpc_window_read(struct ipmi_hiomap *ctx, uint32_t pos,
			   void *buf, uint32_t len)
{
	uint32_t off = ctx->current.lpc_addr + (pos - ctx->current.cur_pos);
	int rc;

	if ((ctx->current.lpc_addr + ctx->current.size) < (off + len))
		return FLASH_ERR_PARM_ERROR;

	hiomap_trace("LPC Reading at 0x%08x for 0x%08x offset: 0x%08x\n",
	      pos, len, off);

	while(len) {
		uint32_t chunk;
		uint32_t dat;

		/* XXX: make this read until it's aligned */
		if (len > 3 && !(off & 3)) {
			rc = lpc_read(OPAL_LPC_FW, off, &dat, 4);
			if (!rc)
				*(uint32_t *)buf = dat;
			chunk = 4;
		} else {
			rc = lpc_read(OPAL_LPC_FW, off, &dat, 1);
			if (!rc)
				*(uint8_t *)buf = dat;
			chunk = 1;
		}
		if (rc) {
			hiomap_error("lpc_read failure %d to FW 0x%08x\n", rc, off);
			return rc;
		}
		len -= chunk;
		off += chunk;
		buf += chunk;
	}

	return 0;
}

static int lpc_window_write(struct ipmi_hiomap *ctx, uint32_t pos,
			    const void *buf, uint32_t len)
{
	uint32_t off = ctx->current.lpc_addr + (pos - ctx->current.cur_pos);
	enum lpc_window_state state;
	int rc;

	lock(&ctx->lock);
	state = ctx->window_state;
	unlock(&ctx->lock);

	if (state != write_window)
		return FLASH_ERR_PARM_ERROR;

	if ((ctx->current.lpc_addr + ctx->current.size) < (off + len))
		return FLASH_ERR_PARM_ERROR;

	hiomap_trace("LPC Writing at 0x%08x for 0x%08x offset: 0x%08x\n",
	      pos, len, off);

	while(len) {
		uint32_t chunk;

		if (len > 3 && !(off & 3)) {
			rc = lpc_write(OPAL_LPC_FW, off,
				       *(uint32_t *)buf, 4);
			chunk = 4;
		} else {
			rc = lpc_write(OPAL_LPC_FW, off,
				       *(uint8_t *)buf, 1);
			chunk = 1;
		}
		if (rc) {
			hiomap_error("failure %d to FW 0x%08x\n", rc, off);
			return rc;
		}
		len -= chunk;
		off += chunk;
		buf += chunk;
	}

	return 0;
}

/* Best-effort asynchronous event handling by blocklevel callbacks */
static int ipmi_hiomap_handle_events(struct ipmi_hiomap *ctx)
{
	uint8_t status;
	int rc;

	lock(&ctx->lock);

	status = ctx->bmc_state;
	hiomap_trace("status=%i\n", status);

	/*
	 * Immediately clear the ackable events to make sure we don't race to
	 * clear them after dropping the lock, as we may lose protocol or
	 * window state if a race materialises. In the event of a failure where
	 * we haven't completed the recovery, the state we mask out below gets
	 * OR'ed back in to avoid losing it.
	 */
	ctx->bmc_state &= ~HIOMAP_E_ACK_MASK;

	/*
	 * We won't be attempting to restore window state -
	 * ipmi_hiomap_handle_events() is followed by hiomap_window_move() in
	 * all cases. Attempting restoration after HIOMAP_E_PROTOCOL_RESET or
	 * HIOMAP_E_WINDOW_RESET can be wasteful if we immediately shift the
	 * window elsewhere, and if it does not need to be shifted with respect
	 * to the subsequent request then hiomap_window_move() will handle
	 * re-opening it from the closed state.
	 *
	 * Therefore it is enough to mark the window as closed to consider it
	 * recovered.
	 */
	if (status & HIOMAP_E_PROTOCOL_RESET) {
		hiomap_trace("status=HIOMAP_E_PROTOCOL_RESET\n");
	}

	if (status & HIOMAP_E_WINDOW_RESET) {
		hiomap_trace("status=HIOMAP_E_WINDOW_RESET\n");
	}

	if (status & (HIOMAP_E_PROTOCOL_RESET | HIOMAP_E_WINDOW_RESET)) {
		ctx->window_state = closed_window;
		hiomap_trace("closed_window\n");
	}

	unlock(&ctx->lock);

	/*
	 * If there's anything to acknowledge, do so in the one request to
	 * minimise overhead. By sending the ACK prior to performing the
	 * protocol recovery we ensure that even with coalesced resets we still
	 * end up in the recovered state and not unknowingly stuck in a reset
	 * state. We may receive reset events after the ACK but prior to the
	 * recovery procedures being run, but this just means that we will
	 * needlessly perform recovery on the following invocation of
	 * ipmi_hiomap_handle_events(). If the reset event is a
	 * HIOMAP_E_WINDOW_RESET it is enough that the window is already marked
	 * as closed above - future accesses will force it to be re-opened and
	 * the BMC's cache must be valid if opening the window is successful.
	 */
	if (status & HIOMAP_E_ACK_MASK) {
		hiomap_trace("status=%i HIOMAP_E_ACK_MASK so TRY to ACK\n", status);
		/* ACK is unversioned, can send it if the daemon is ready */
		rc = hiomap_ack(ctx, status & HIOMAP_E_ACK_MASK);
		if (rc) {
			hiomap_notice("Failed to ack events rc=%i: status & HIOMAP_E_ACK_MASK=0x%x status=%i\n",
			      rc, (status & HIOMAP_E_ACK_MASK), status);
			goto restore;
		}
	}

	if (status & HIOMAP_E_PROTOCOL_RESET) {
		hiomap_info("Protocol was reset\n");

		rc = hiomap_get_info(ctx);
		if (rc) {
			hiomap_error("Failure to renegotiate after protocol reset\n");
			goto restore;
		}

		rc = hiomap_get_flash_info(ctx);
		if (rc) {
			hiomap_error("Failure to fetch flash info after protocol reset\n");
			goto restore;
		}

		hiomap_info("Restored state after protocol reset\n");
	}

	/*
	 * As there's no change to the protocol on HIOMAP_E_WINDOW_RESET we
	 * simply need to open a window to recover, which as mentioned above is
	 * handled by hiomap_window_move() after our cleanup here.
	 */

	return 0;

restore:
	/*
	 * Conservatively restore the events to the un-acked state to avoid
	 * losing events due to races. It might cause us to restore state more
	 * than necessary, but never less than necessary.
	 */
	lock(&ctx->lock);
	hiomap_trace("PRE restore status=%i PRE ctx->bmc_state=%i rc=%i\n", status, ctx->bmc_state, rc);
	ctx->bmc_state |= (status & HIOMAP_E_ACK_MASK);
	hiomap_trace("POST restored status=%i POST ctx->bmc_state=%i rc=%i\n", status, ctx->bmc_state, rc);
	unlock(&ctx->lock);

	return rc;
}

static int ipmi_hiomap_read(struct blocklevel_device *bl, uint64_t pos,
			    void *buf, uint64_t len)
{
	struct ipmi_hiomap *ctx;
	enum lpc_window_state state;
	static uint64_t size;
	int rc;

	/* LPC is only 32bit */
	if (pos > UINT_MAX || len > UINT_MAX)
		return OPAL_PARAMETER;

	ctx = container_of(bl, struct ipmi_hiomap, bl);

	lock(&ctx->transaction_lock);

	rc = ipmi_hiomap_handle_events(ctx);
	if (rc) {
		hiomap_notice("ipmi_hiomap_handle_events failed: rc=%d\n", rc);
		goto out;
	}

	lock(&ctx->lock);
	if (ctx->bl.flags & IN_PROGRESS) {
		buf = ctx->tracking_buf;
		pos = ctx->tracking_pos;
		len = ctx->tracking_len;
	} else {
		ctx->tracking_buf = buf;
		ctx->tracking_pos = 0;
		ctx->tracking_len = 0;
	}
	unlock(&ctx->lock);

	hiomap_trace("Flash READ at pos %llu for %llu bytes\n", pos, len);
	while (len > 0) {
		lock(&ctx->lock);
		state = ctx->window_state;
		unlock(&ctx->lock);
		if (state != moving_window) {
			/* Move window and get a new size to read */
			rc = hiomap_window_move(ctx, HIOMAP_C_CREATE_READ_WINDOW, pos,
				len, &size);
			if (rc) {
				hiomap_notice("hiomap_window_move failed: rc=%d\n",
					rc);
				goto out;
			}
		} else {
			rc = hiomap_wait_for_cc(ctx, &ctx->cc, &ctx->inflight_seq, IPMI_HIOMAP_TICKS_DEFAULT);
			if (rc) {
				hiomap_trace("move hiomap_wait_for_cc failed: rc=%d ticks=%i\n",
					rc, IPMI_HIOMAP_TICKS_DEFAULT);
				goto out;
			}
		}

		lock(&ctx->lock);
		state = ctx->window_state;
		unlock(&ctx->lock);
		if (state == read_window) {
			/*
			 * don't lock in case move_cb in progress
			 * if we get here the state is good
			 * just double-checking
			 */
			if (ctx->cc != IPMI_CC_NO_ERROR) {
				hiomap_notice("failed: cc=%d\n", ctx->cc);
				rc = OPAL_HARDWARE;
				goto out;
			}
			/* Perform the read for this window */
			rc = lpc_window_read(ctx, pos, buf, size);
			if (rc) {
				hiomap_notice("lpc_window_read failed: rc=%d\n", rc);
				goto out;
			}

			/* Check we can trust what we read */
			lock(&ctx->lock);
			rc = hiomap_window_valid(ctx, pos, size);
			unlock(&ctx->lock);
			if (rc) {
				hiomap_notice("hiomap_window_valid failed: rc=%d\n", rc);
				goto out;
			}

			len -= size;
			pos += size;
			buf += size;
			lock(&ctx->lock);
			ctx->tracking_len = len;
			ctx->tracking_pos = pos;
			ctx->tracking_buf = buf;
			unlock(&ctx->lock);
		}
	}

out:	unlock(&ctx->transaction_lock);
	return rc;
}

static int ipmi_hiomap_write(struct blocklevel_device *bl, uint64_t pos,
			     const void *buf, uint64_t len)
{
	struct ipmi_hiomap *ctx;
	enum lpc_window_state state;
	static uint64_t size;
	int rc;

	/* LPC is only 32bit */
	if (pos > UINT_MAX || len > UINT_MAX)
		return FLASH_ERR_PARM_ERROR;

	ctx = container_of(bl, struct ipmi_hiomap, bl);
	lock(&ctx->transaction_lock);

	rc = ipmi_hiomap_handle_events(ctx);
	if (rc) {
		hiomap_notice("ipmi_hiomap_handle_events failed: rc=%d\n", rc);
		goto out;
	}

	lock(&ctx->lock);
	if (ctx->bl.flags & IN_PROGRESS) {
		buf = ctx->tracking_buf;
		pos = ctx->tracking_pos;
		len = ctx->tracking_len;
	} else {
		ctx->tracking_buf = (void *) buf;
		ctx->tracking_pos = 0;
		ctx->tracking_len = 0;
	}
	unlock(&ctx->lock);

	hiomap_trace("Flash WRITE at pos %llu for %llu bytes\n", pos, len);
	while (len > 0) {
		lock(&ctx->lock);
		state = ctx->window_state;
		unlock(&ctx->lock);
		if (state != moving_window) {
			/* Move window and get a new size to read */
			rc = hiomap_window_move(ctx, HIOMAP_C_CREATE_WRITE_WINDOW, pos,
					        len, &size);
			if (rc) {
				hiomap_notice("hiomap_window_move failed: rc=%d\n",
					rc);
				goto out;
			}
		} else {
			rc = hiomap_wait_for_cc(ctx, &ctx->cc, &ctx->inflight_seq, IPMI_LONG_TICKS);
			if (rc) {
				hiomap_trace("move hiomap_wait_for_cc failed: rc=%d ticks=%i\n",
					rc, IPMI_LONG_TICKS);
				goto out;
			}
		}

		lock(&ctx->lock);
		state = ctx->window_state;
		unlock(&ctx->lock);

		if (state == write_window) {
			if (ctx->cc != IPMI_CC_NO_ERROR) {
				hiomap_notice("failed: cc=%d\n", ctx->cc);
				rc = OPAL_HARDWARE;
				goto out;
			}

			/* Perform the write for this window */
			rc = lpc_window_write(ctx, pos, buf, size);
			if (rc) {
				hiomap_notice("lpc_window_write failed: rc=%d\n", rc);
				goto out;
			}

			/*
			 * Unlike ipmi_hiomap_read() we don't explicitly test if the
			 * window is still valid after completing the LPC accesses as
			 * the following hiomap_mark_dirty() will implicitly check for
			 * us. In the case of a read operation there's no requirement
			 * that a command that validates window state follows, so the
			 * read implementation explicitly performs a check.
			 */

			rc = hiomap_mark_dirty(ctx, pos, size);
			if (rc) {
				hiomap_notice("hiomap_mark_dirty failed: rc=%d\n", rc);
				goto out;
			}
			rc = hiomap_wait_for_cc(ctx, &ctx->cc, &ctx->inflight_seq, IPMI_LONG_TICKS);
			if (rc) {
				hiomap_trace("dirty hiomap_wait_for_cc failed: rc=%d ticks=%i\n",
					rc, IPMI_LONG_TICKS);
				goto out;
			}

			/*
			 * The BMC *should* flush if the window is implicitly closed,
			 * but do an explicit flush here to be sure.
			 *
			 * XXX: Removing this could improve performance
			 */
			rc = hiomap_flush(ctx);
			if (rc) {
				hiomap_notice("hiomap_flush failed: rc=%d\n", rc);
				goto out;
			}
			rc = hiomap_wait_for_cc(ctx, &ctx->cc, &ctx->inflight_seq, IPMI_LONG_TICKS);
			if (rc) {
				hiomap_trace("flush hiomap_wait_for_cc failed: rc=%d ticks=%i\n",
					rc, IPMI_LONG_TICKS);
				goto out;
			}

			len -= size;
			pos += size;
			buf += size;
			lock(&ctx->lock);
			ctx->tracking_len = len;
			ctx->tracking_pos = pos;
			ctx->tracking_buf = (void *) buf;
			unlock(&ctx->lock);
		}
	}

out:	unlock(&ctx->transaction_lock);
	return rc;
}

static int ipmi_hiomap_erase(struct blocklevel_device *bl, uint64_t pos,
			     uint64_t len)
{
	struct ipmi_hiomap *ctx;
	enum lpc_window_state state;
	static uint64_t size;
	int rc;

	/* LPC is only 32bit */
	if (pos > UINT_MAX || len > UINT_MAX)
		return OPAL_PARAMETER;

	ctx = container_of(bl, struct ipmi_hiomap, bl);
	lock(&ctx->transaction_lock);

	rc = ipmi_hiomap_handle_events(ctx);
	if (rc) {
		hiomap_notice("ipmi_hiomap_handle_events failed: rc=%d\n", rc);
		goto out;
	}


	lock(&ctx->lock);
	if (ctx->bl.flags & IN_PROGRESS) {
		pos = ctx->tracking_pos;
		len = ctx->tracking_len;
	} else {
		ctx->tracking_pos = 0;
		ctx->tracking_len = 0;
	}
	unlock(&ctx->lock);

	hiomap_trace("Flash ERASE at pos %llu for %llu bytes\n", pos, len);

	while (len > 0) {
		lock(&ctx->lock);
		state = ctx->window_state;
		unlock(&ctx->lock);
		if (state != moving_window) {
			/* Move window and get a new size to erase */
			rc = hiomap_window_move(ctx, HIOMAP_C_CREATE_WRITE_WINDOW, pos,
					        len, &size);
			if (rc) {
				hiomap_notice("hiomap_window_move failed: rc=%d\n",
					rc);
				goto out;
			}
		} else {
			rc = hiomap_wait_for_cc(ctx, &ctx->cc, &ctx->inflight_seq, IPMI_LONG_TICKS);
			if (rc) {
				hiomap_trace("move hiomap_wait_for_cc failed: rc=%d ticks=%i\n",
					rc, IPMI_LONG_TICKS);
				goto out;
			}
		}

		lock(&ctx->lock);
		state = ctx->window_state;
		unlock(&ctx->lock);
		if (state == write_window) {
			if (ctx->cc != IPMI_CC_NO_ERROR) {
				hiomap_notice("failed: cc=%d\n", ctx->cc);
				rc = OPAL_HARDWARE;
				goto out;
			}
			rc = hiomap_erase(ctx, pos, size);
			if (rc) {
				hiomap_notice("hiomap_erase failed: rc=%d\n", rc);
				goto out;
			}
			rc = hiomap_wait_for_cc(ctx, &ctx->cc, &ctx->inflight_seq, IPMI_LONG_TICKS);
			if (rc) {
				hiomap_trace("move hiomap_wait_for_cc failed: rc=%d ticks=%i\n",
					rc, IPMI_LONG_TICKS);
				goto out;
			}

			/*
			 * Flush directly, don't mark that region dirty otherwise it
			 * isn't clear if a write happened there or not
			 */
			rc = hiomap_flush(ctx);
			if (rc) {
				hiomap_notice("hiomap_flush failed: rc=%d\n", rc);
				goto out;
			}
			rc = hiomap_wait_for_cc(ctx, &ctx->cc, &ctx->inflight_seq, IPMI_LONG_TICKS);
			if (rc) {
				hiomap_trace("move hiomap_wait_for_cc failed: rc=%d ticks=%i\n",
					rc, IPMI_LONG_TICKS);
				goto out;
			}

			len -= size;
			pos += size;
			lock(&ctx->lock);
			ctx->tracking_len = len;
			ctx->tracking_pos = pos;
			unlock(&ctx->lock);
		}
	}

out:	unlock(&ctx->transaction_lock);
	return rc;
}

static int ipmi_hiomap_get_flash_info(struct blocklevel_device *bl,
				      const char **name, uint64_t *total_size,
				      uint32_t *erase_granule)
{
	struct ipmi_hiomap *ctx;
	int rc;

	ctx = container_of(bl, struct ipmi_hiomap, bl);
	lock(&ctx->transaction_lock);

	rc = ipmi_hiomap_handle_events(ctx);
	if (rc) {
		hiomap_notice("ipmi_hiomap_handle_events failed: rc=%d\n", rc);
		return rc;
	}

	rc = hiomap_get_flash_info(ctx);
	if (rc) {
		hiomap_notice("hiomap_get_flash_info failed: rc=%d\n", rc);
		return rc;
	}

	ctx->bl.erase_mask = ctx->erase_granule - 1;

	if (name)
		*name = NULL;
	if (total_size)
		*total_size = ctx->total_size;
	if (erase_granule)
		*erase_granule = ctx->erase_granule;

	unlock(&ctx->transaction_lock);
	return 0;
}

int ipmi_hiomap_init(struct blocklevel_device **bl)
{
	struct ipmi_hiomap *ctx;
	int rc;

	if (!bmc_platform->sw->ipmi_oem_hiomap_cmd)
		/* FIXME: Find a better error code */
		return FLASH_ERR_DEVICE_GONE;

	if (!bl)
		return FLASH_ERR_PARM_ERROR;

	*bl = NULL;

	ctx = zalloc(sizeof(struct ipmi_hiomap));
	if (!ctx)
		return FLASH_ERR_MALLOC_FAILED;

	init_lock(&ctx->lock);
	init_lock(&ctx->transaction_lock);

	ctx->bl.read = &ipmi_hiomap_read;
	ctx->bl.write = &ipmi_hiomap_write;
	ctx->bl.erase = &ipmi_hiomap_erase;
	ctx->bl.get_info = &ipmi_hiomap_get_flash_info;
	ctx->bl.exit = &ipmi_hiomap_exit;

	hiomap_init(ctx);

	/* Ack all pending ack-able events to avoid spurious failures */
	rc = hiomap_ack(ctx, HIOMAP_E_ACK_MASK);
	if (rc) {
		hiomap_notice("Failed to ack events: 0x%x\n",
		      HIOMAP_E_ACK_MASK);
		goto err;
	}

	rc = ipmi_sel_register(CMD_OP_HIOMAP_EVENT, hiomap_event, ctx);
	if (rc < 0) {
		hiomap_error("Failed ipmi_sel_register: %d\n", rc);
		goto err;
	}

	/* Negotiate protocol behaviour */
	rc = hiomap_get_info(ctx);
	if (rc) {
		hiomap_error("Failed to get hiomap parameters: %d\n", rc);
		goto err;
	}

	/* Grab the flash parameters */
	rc = hiomap_get_flash_info(ctx);
	if (rc) {
		hiomap_error("Failed to get flash parameters: %d\n", rc);
		goto err;
	}

	hiomap_notice("Negotiated hiomap protocol v%u\n", ctx->version);
	hiomap_notice("Block size is %uKiB\n",
	      1 << (ctx->block_size_shift - 10));
	hiomap_notice("BMC suggested flash timeout of %us\n", ctx->timeout);
	hiomap_notice("Flash size is %uMiB\n", ctx->total_size >> 20);
	hiomap_notice("Erase granule size is %uKiB\n",
	      ctx->erase_granule >> 10);

	ctx->bl.keep_alive = 0;

	*bl = &(ctx->bl);

	return 0;

err:
	free(ctx);

	return rc;
}

bool ipmi_hiomap_exit(struct blocklevel_device *bl)
{
	bool status = true;

	struct ipmi_hiomap *ctx;
	if (bl) {
		ctx = container_of(bl, struct ipmi_hiomap, bl);
		status = hiomap_reset(ctx);
		free(ctx);
	}

	return status;
}
