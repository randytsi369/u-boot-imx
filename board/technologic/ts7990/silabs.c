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

#include "tsfpga.h"

// Scale voltage to silabs 0-2.5V
uint16_t inline sscale(uint16_t data) {
	return data * (2.5/1023) * 1000;
}

// Scale voltage for resistor dividers
uint16_t inline rscale(uint16_t data, uint16_t r1, uint16_t r2)
{
	uint16_t ret = (data * (r1 + r2)/r2);
	return sscale(ret);
}

void read_adcs(uint16_t *data)
{
	uint8_t tmp[20];
	int i, ret;
	tmp[19] = 0;

	ret = i2c_read(0x4a, 0, 0, tmp, 19);

	if(ret){
		printf("I2C Read failed with %d\n", ret);
		return;
	}
	for (i = 0; i < 10; i++)
		data[i] = (tmp[i*2] << 8) | tmp[(i*2)+1];
}

void enable_supercaps(void)
{
	uint8_t val = 0x1;
	/* Set all pins to GPIO and inputs */
	fpga_gpio_input(FPGA_DIO_1);
	fpga_gpio_input(FPGA_DIO_2);
	fpga_gpio_input(FPGA_DIO_4);
	fpga_gpio_input(FPGA_DIO_5);
	fpga_gpio_input(FPGA_DIO_6);

	/*Tell supercaps to charge */
	i2c_write(0x4a, 0, 0, &val, 1);
}

void disable_supercaps(void)
{
	uint8_t val = 0x0;
	i2c_write(0x4a, 0, 0, &val, 1);
}

int tssilo_is_detected(void)
{
	/* Silabs does not reset on reboot, so this
	 * must be reset manually */
	disable_supercaps();
	if(fpga_gpio_input(FPGA_DIO_1) == 0) {
		return true;
	}
	return false;
}

void board_sleep(int seconds)
{
	uint8_t dat[4] = {0};

	/* Only relevant on cap touch variants */
	if(i2c_probe(0x5c)) {
		/* Put touch controller into sleep mode (scan slower) */
		dat[0] = 0x1;
		i2c_write(0x5c, 51, 1, dat, 1);

		/* Change touch controller to assert int only on touch*/
		dat[0] = 0xa;
		i2c_write(0x5c, 52, 1, dat, 1);
	}

	dat[0] = 0x0;
	dat[1] = ((seconds >> 16) & 0xff);
	dat[2] = ((seconds >> 8) & 0xff);
	dat[3] = (seconds & 0xff);
	i2c_write(0x4a, 0, 0, dat, 4);
}

void block_charge(int blkpct)
{
	int i = 0;
	int chgpct = 0;

	enable_supercaps();

	while(chgpct < blkpct) {
		int chgmv;
		uint16_t data[10];
		read_adcs(data);

		if(ctrlc())
			return;

		if(fpga_gpio_input(FPGA_POWER_FAIL) == 1) {
			if(i % 250)
				puts("Cannot boot with POWER_FAIL asserted\n");
		} else if(fpga_gpio_input(FPGA_DIO_6) == 1) {
			if(i % 250)
				printf("Need 24VDC(18VDC min) to charge, at %dmV\r",
					rscale(data[0], 2870, 147));
		} else {
			chgmv = rscale(data[5], 100, 22);
			chgpct = chgmv*100/12000;
			if(i % 250)
				printf("Charging to %d%%... %03d/100%% (%dmV)\r",
					blkpct,
					chgpct, 
					chgmv);
		}

		i++;
		udelay(1000);
	}
	puts("\n");
	puts("Fully charged caps\n");
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
			case 'o':
				disable_supercaps();
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
			case 'i':
				read_adcs(data);
				printf("VIN=%dmV\n", rscale(data[0], 2870, 147));
				printf("SUPERCAP=%dmV\n", rscale(data[5], 100, 22));
				break;
			case 'b':
				if(i+1 == argc) {
					printf("Missing option for percentage of charge to wait for\n");
					return 1;
				}
				pct = simple_strtoul(argv[++i], NULL, 10);
				block_charge(pct);
				break;
			case 'e':
				enable_supercaps();
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
	"    -i print ADC values in millivolts\n"
	"    -b <percent> Turn on supercaps and block until charged\n"
	"    -e Turn on supercaps\n"
	"    -o Turn off supercaps\n"
	"  If the seconds argument is supplied the board will sleep.\n"
	"  if not specified, it will print out the ADC values\n"
);
