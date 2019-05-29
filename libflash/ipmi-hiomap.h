// SPDX-License-Identifier: Apache-2.0
/* Copyright 2018-2019 IBM Corp. */

#ifndef __LIBFLASH_IPMI_HIOMAP_H
#define __LIBFLASH_IPMI_HIOMAP_H

#include <lock.h>
#include <stdbool.h>
#include <stdint.h>

#include "blocklevel.h"

/*
 * we basically check for a quick response
 * otherwise we catch the updated window in the next cycle
 */
#define IPMI_HIOMAP_TICKS 5
#define IPMI_HIOMAP_TICKS_DEFAULT 0

/* time to wait for write/erase/dirty ops */
#define IPMI_LONG_TICKS 500

/*
 * default for ack'ing typically 1-10 wait_time's
 * allow upper bounds because if we cannot ack
 * we make no forward progress post protocol reset
 * async paths will retry
 * sync paths always hit with zero wait_time elapsed
 * with ASYNC_REQUIRED masked out, this is not used
 */
#define IPMI_ACK_DEFAULT 500

/* increment to skip the waiting loop */
#define IPMI_SKIP_INC 2

enum lpc_window_state { closed_window, read_window, write_window, moving_window };

struct lpc_window {
	uint32_t lpc_addr; /* Offset into LPC space */
	uint32_t cur_pos;  /* Current position of the window in the flash */
	uint32_t size;     /* Size of the window into the flash */
	uint32_t adjusted_window_size; /* store adjusted window size */
};

struct ipmi_hiomap {
	/* Members protected by the blocklevel lock */
	uint8_t seq;
	uint8_t version;
	uint8_t block_size_shift;
	uint16_t timeout;
	struct blocklevel_device bl;
	uint32_t total_size;
	uint32_t erase_granule;
	struct lpc_window current;

	/*
	 * update, bmc_state and window_state can be accessed by both calls
	 * through read/write/erase functions and the IPMI SEL handler. All
	 * three variables are protected by lock to avoid conflict.
	 */
	struct lock lock;
	struct lock transaction_lock;

	/* callers transaction info */
	uint64_t *active_size;
	uint64_t requested_len;
	uint64_t requested_pos;
	uint64_t tracking_len;
	uint64_t tracking_pos;
	void *tracking_buf;

	int missed_cc_count;
	int cc;
	/* inflight_seq used to aide debug */
	/* with other OPAL ipmi msg's      */
	uint8_t inflight_seq;
	uint8_t bmc_state;
	enum lpc_window_state window_state;
};

int ipmi_hiomap_init(struct blocklevel_device **bl);
bool ipmi_hiomap_exit(struct blocklevel_device *bl);

#endif /* __LIBFLASH_IPMI_HIOMAP_H */
