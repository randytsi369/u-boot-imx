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

// Scale voltage to silabs 0-2.5V
uint16_t inline sscale(uint16_t data){
	return data * (2.5/1023) * 1000;
}

// Scale voltage for resistor dividers
uint16_t inline rscale(uint16_t data, uint16_t r1, uint16_t r2)
{
	uint16_t ret = (data * (r1 + r2)/r2);
	return sscale(ret);
}

void read_adcs(void)
{
	uint16_t data[16];
	uint8_t tmp[32];
	int i, ret;

	ret = i2c_read(0x10, 0, 0, tmp, 32);

	if(ret){
		printf("I2C Read failed with %d\n", ret);
		return;
	}
    for (i = 0; i < 15; i++)
    	data[i] = (tmp[i*2] << 8) | tmp[(i*2)+1];

	printf("VDD_ARM_CAP=%d\n", sscale(data[0]));
	printf("VDD_HIGH_CAP=%d\n", sscale(data[1]));
	printf("VDD_SOC_CAP=%d\n",sscale(data[2]));
	printf("VDD_ARM=%d\n", sscale(data[3]));
	printf("SILAB_P10=0x%X\n", data[4]);
	printf("SILAB_P11=0x%X\n", data[5]);
	printf("SILAB_P12=0x%X\n", data[6]);
	printf("VIN=%d\n", rscale(data[7], 2870, 147));
	printf("V5_A=%d\n", rscale(data[8], 147, 107));
	printf("V3P1=%d\n", rscale(data[9], 499, 499));
	printf("DDR_1P5V=%d\n", sscale(data[10]));
	printf("V1P8=%d\n", sscale(data[11]));
	printf("V1P2=%d\n", sscale(data[12]));
	printf("RAM_VREF=%d\n", sscale(data[13]));
	printf("V3P3=%d\n", rscale(data[14], 499, 499));
}

void board_sleep(int deciseconds)
{
	uint8_t dat[4] = {0};
	uint8_t opt_resetswitchwkup = 1;
	uint8_t opt_sleepmode = 1;

	dat[0]=(0x1 | (opt_resetswitchwkup << 1) |
	  ((opt_sleepmode-1) << 4) | 1 << 6);
	dat[3] = (deciseconds & 0xff);
	dat[2] = ((deciseconds >> 8) & 0xff);
	dat[1] = ((deciseconds >> 16) & 0xff);
	i2c_write(0x10, 0, 0, dat, 4);
}

static int do_microctl(cmd_tbl_t *cmdtp, int flag, 
	int argc, char * const argv[])
{
	if(argc == 1)
		read_adcs();
	else if(argc == 2)
		board_sleep(simple_strtoul(argv[1], NULL, 10));
	else
		return 1;

	return 0;
}

U_BOOT_CMD(tsmicroctl, 3, 1, do_microctl,
	"TS supervisory microcontroller access",
	"  Usage: tsmicroctl [seconds]\n"
	"  If the seconds argument is supplied the board will sleep.\n"
	"  if not specified, it will print out the ADC values\n"
);
