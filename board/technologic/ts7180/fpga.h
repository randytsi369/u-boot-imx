/*
 * Copyright (C) 2016 Technologic Systems
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#ifndef __FPGA_H__
#define __FPGA_H__

/* FPGA IO */
#define EN_YEL_LEDN		2
#define EN_GREEN_LEDN		3
#define EN_RED_LEDN		4
#define EN_BLUE_LED		5
#define EN_SD_POWER 		14
#define EN_USB_HOST_5V		15
#define EN_OFF_BD_5V		16
#define SD_BOOT_JMPN		36

void fpga_gpio_output(int io, int value);
int fpga_gpio_input(int io);
int fpga_get_rev(void);

#endif // __FPGA_H__