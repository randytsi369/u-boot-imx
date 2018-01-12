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

#include "silabs.h"

int silab_rev(void)
{
	uint8_t val[17];
	i2c_set_bus_num(0);
	i2c_read(0x4a, 0, 0, val, 17);
	return (val[16] & 0xF);
}

void board_sleep(int seconds)
{
	uint8_t dat[4] = {0};

	dat[0] = 0x0;
	dat[1] = ((seconds >> 16) & 0xff);
	dat[2] = ((seconds >> 8) & 0xff);
	dat[3] = (seconds & 0xff);
	
	i2c_write(0x4a, 0, 0, dat, 4);
}

// Scale voltage to silabs 0-2.5V
static uint16_t inline sscale(uint16_t data){
	return data * (2.5/1023) * 1000;
}

// Scale voltage for resistor dividers
static uint16_t inline rscale(uint16_t data, uint16_t r1, uint16_t r2)
{
	uint16_t ret = (data * (r1 + r2)/r2);
	return sscale(ret);
}

void read_adcs(void)
{
	uint16_t data[14];
	uint8_t tmp[28];
	int i, ret;

	ret = i2c_read(0x4a, 0, 0, tmp, 28);

	if(ret){
		printf("I2C Read failed with %d\n", ret);
		return;
	}
	for (i = 0; i < 14; i++){
		data[i] = (tmp[i*2] << 8) | tmp[(i*2)+1];
	}

	/* Byte order is P1.2-P1.4, P2.0-P2.7, temp sensor */
	printf("AN_SUP_CAP_1=%d\n", sscale(data[0]));
	printf("AN_SUP_CAP_2=%d\n", rscale(data[1], 20, 20));
	printf("AN_MAIN_4P7V=%d\n", rscale(data[2], 20, 20));
	printf("MAIN_5V=%d\n", rscale(data[3], 536, 422));
	printf("USB_OTG_5V=%d\n", rscale(data[4], 536, 422));
	printf("V3P3=%d\n", rscale(data[5], 422, 422));
	printf("RAM_1P35V=%d\n", sscale(data[6]));
	printf("VDD_6UL_CORE=%d\n", sscale(data[9]));
	printf("AN_CHRG=%d\n", rscale(data[10], 422, 422));
	printf("VDD_SOC_CAP=%d\n", sscale(data[11]));
	printf("VDD_ARM_CAP=%d\n", sscale(data[12]));
	printf("SILAB_REV=0x%X\n", data[8] >> 12);
	}

static int do_microctl(cmd_tbl_t *cmdtp, int flag, 
	int argc, char * const argv[])
{
	int i;
	i2c_set_bus_num(0);

	for (i = 1; i < argc; i++)
	{
		int micros;
		char *p;
		if(argv[i][0] == '-')
			p = &argv[i][1];
		else
			p = &argv[i][0];

		switch(p[0]) {
			case 'i':
				read_adcs();
				break;
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
