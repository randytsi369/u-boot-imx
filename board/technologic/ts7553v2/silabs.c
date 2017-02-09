/*
 * Copyright (C) 2016 Technologic Systems
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#include <common.h>
#include <stdlib.h>
#include <asm/io.h>
#include <asm/arch/gpio.h>
#include <asm/gpio.h>
#include <asm/imx-common/iomux-v3.h>
#include <asm/arch/imx-regs.h>
#include <asm/arch/iomux.h>
#include <status_led.h>
#include <i2c.h>

void board_sleep(int seconds)
{
	uint8_t dat[4] = {0};

	dat[0] = 0x0;
	dat[1] = ((seconds >> 16) & 0xff);
	dat[2] = ((seconds >> 8) & 0xff);
	dat[3] = (seconds & 0xff);
	
	i2c_set_bus_num(0);
	i2c_write(0x4a, 0, 0, dat, 4);
}

static int do_microctl(cmd_tbl_t *cmdtp, int flag, 
	int argc, char * const argv[])
{
	int i;

	for (i = 1; i < argc; i++)
	{
		int micros;
		int pct;
		char *p;
		uint16_t data[10];
		if(argv[i][0] == '-')
			p = &argv[i][1];
		else
			p = &argv[i][0];

		switch(p[0]) {
			case 's':
				if(i+1 == argc) {
					printf("Missing option for microseconds to sleep\n");
					return 1;
				}
				micros = simple_strtoul(argv[++i], NULL, 10);
				printf("Sleep for %d seconds\n", micros);
				board_sleep(micros);
				break;
			default:
				printf("Unknown option '%s'\n", argv[i]);
				return 1;
		}
	}

	return 0;
}

U_BOOT_CMD(tsmicroctl, 3, 0, do_microctl,
	"TS supervisory microcontroller access",
	"  Usage: tsmicroctl <options>\n"
	"    -s <seconds> sleep for x seconds\n"
	"  If the seconds argument is supplied the board will sleep.\n"
);
