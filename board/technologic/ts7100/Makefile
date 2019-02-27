# (C) Copyright 2015 Technologic Systems
#
# SPDX-License-Identifier:	GPL-2.0+
#

obj-y  := ts7100.o led.o post.o c2.o tsmicroctl.o fram.o

extra-$(CONFIG_USE_PLUGIN) :=  plugin.bin
$(obj)/plugin.bin: $(obj)/plugin.o
	$(OBJCOPY) -O binary --gap-fill 0xff $< $@
