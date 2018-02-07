/*
 * Copyright (C) 2018 Technologic Systems
 *
 * Author: Kris Bahnsen <kris@embeddedarm.com>
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#include <common.h>
#include <console.h>

#include <asm/arch/mx6-pins.h>
#include <asm/gpio.h>
#include <asm/imx-common/gpio.h>
#include <asm/imx-common/iomux-v3.h>
#include <cli.h>
#include <command.h>

#define POWERFAIL	IMX_GPIO_NR(5, 0)

static int do_powercheck(cmd_tbl_t *cmdtp, int flag, int argc, char * const argv[])
{
	gpio_direction_input(POWERFAIL);

	if (gpio_get_value(POWERFAIL)) {
		printf("Waiting for valid power input\n");
		while(gpio_get_value(POWERFAIL)) {
			if(ctrlc()) return 1;
		}
	}

	return 0;
}

U_BOOT_CMD(powercheck, 2, 1, do_powercheck,
	"Wait until valid power input",
	""
);
