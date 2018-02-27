# (C) Copyright 2015 Technologic Systems
#
# SPDX-License-Identifier:	GPL-2.0+
#

obj-y  := ts4100.o led.o fpga.o silabs.o post.o parse_strap.o powercheck.o bb.o

extra-$(CONFIG_USE_PLUGIN) :=  plugin.bin
$(obj)/plugin.bin: $(obj)/plugin.o
	$(OBJCOPY) -O binary --gap-fill 0xff $< $@
