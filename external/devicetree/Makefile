# SPDX-License-Identifier: Apache-2.0
#
# Copyright 2019 IBM Corp.

DTC=dtc
OUT=p9-simics.dtb

all: $(OUT)

%.dtb: %.dts
	$(DTC) -I dts -O dtb $< -o $@

clean:
	$(RM) $(OUT)
