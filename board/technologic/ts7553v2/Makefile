# (C) Copyright 2015 Technologic Systems
#
# SPDX-License-Identifier:	GPL-2.0+
#

obj-y  := ts7553v2.o silabs.o c2.o led.o post.o parse_strap.o

extra-$(CONFIG_USE_PLUGIN) :=  plugin.bin
$(obj)/plugin.bin: $(obj)/plugin.o
	$(OBJCOPY) -O binary --gap-fill 0xff $< $@
