/*
 * Copyright (C) 2016 Technologic Systems
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#include <common.h>
#include <console.h>
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

void enable_tssilo(void)
{
	uint8_t dat = 0x1;
	i2c_write(0x4a, 0x0, 0, &dat, 1);
}

void disable_tssilo(void)
{
	uint8_t dat = 0x0;
	i2c_write(0x4a, 0x0, 0, &dat, 1);
}

/*
	0,1   P1.2, AN_SUP_CAP_1
	2,3   P1.3, AN_SUP_CAP_2
	4,5   P1.4, AN_MAIN_4.7V
	6,7   P2.0, VIN (scaled 5.3%)
	8,9   P2.1  5.2V_A (scaled 44%)
	10,11 P2.2  3.3V (scaled 50%)
	12,13 P2.3  RAM_1.35V
	14,17 reserved
	18,19 P2.4  VDD_6UL_CORE
	20,21 P2.5  AN_CHRG (scaled 50%)
	22,23 P2.6  VDD_SOC_CAP
	24,25 P2.7  VDD_ARM_CAP
*/

#define ADC_AN_SUP_CAP_1   (data[0]<<8|data[1])
#define ADC_AN_SUP_CAP_2   ((data[2]<<8|data[3]))
#define ADC_AN_MAIN_4V7    ((data[4]<<8|data[5]))
#define ADC_VIN            ((data[6]<<8|data[7]))
#define ADC_5V2_A          ((data[8]<<8|data[9]))
#define ADC_3V3            ((data[10]<<8|data[11]))
#define ADC_RAM_1V35       ((data[12]<<8|data[13]))
#define ADC_VDD_6UL_CORE   ((data[18]<<8|data[19]))
#define ADC_AN_CHRG        ((data[20]<<8|data[21]))
#define ADC_VDD_SOC_CAP    ((data[22]<<8|data[23]))
#define ADC_VDD_ARM_CAP    ((data[24]<<8|data[25]))


// Scale voltage to silabs 0-2.5V
static uint16_t inline sscale(uint16_t data){
	return (data * 10000) / 4092;
	//	return data * (2.5/1023) * 1000;

}

// Scale voltage for resistor dividers
static uint16_t inline rscale(uint16_t data, uint16_t r1, uint16_t r2)
{
	uint16_t ret = (data * (r1 + r2)/r2);
	return sscale(ret);
}

int wait_for_supercaps(int pct, int verbose)
{
	unsigned char data[4] = {0};
	unsigned int check;

	enable_tssilo();

	if(pct == 0) {
		printf("Not waiting for SuperCaps to charge\n");
		return 0;
	} else {
		printf("Waiting until SuperCaps are charged to %d%%\n", pct);
	}
	if(pct > 100) pct = 100;

	while(1) {
		i2c_read(0x4a, 0x0, 0, data, 4);
		check = rscale(ADC_AN_SUP_CAP_2, 20, 20);

		// 0% is 3800mV, 100% is 4800mV
		if(check > 3800 ) {
			check = (check-3800) / 10;
			if(check > 100) check = 100;
			if(verbose) printf("%d%%\n", check);
			if(check >= pct) return 0;
		} else {
			if(verbose) printf("0%%\n");
		}
		if(ctrlc()) return 1;
		udelay(1000000);
	}
}

static void do_info(void)
{
	uint8_t data[18];
	i2c_read(0x4a, 0, 0, data, 18);
	printf("revision=0x%x\n", data[16]);
}


static int do_microctl(cmd_tbl_t *cmdtp, int flag,
	int argc, char * const argv[])
{
	int i;
	unsigned int verbose = 0;
	unsigned int pct = 0;
	char *p;

	i2c_set_bus_num(0);
	for (i = 1; i < argc; i++)
	{
		int micros;

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
			case '0':
				break;
			case '1':
				verbose = 1;
				break;
			case 'w':
				if(i+1 == argc) {
					printf("Missing argument for percent to charge\n");
					return 1;
				}
				pct = simple_strtoul(argv[++i], NULL, 10);
				break;
			case 'e':
				enable_tssilo();
				break;
			case 'd':
				disable_tssilo();
				break;
			case 'i':
				do_info();
				break;
			default:
				printf("Unknown option '%s'\n", argv[i]);
				return 1;
		}
	}

	if (pct) wait_for_supercaps(pct, verbose);

	i2c_set_bus_num(2);
	return 0;
}

U_BOOT_CMD(tsmicroctl, 4, 0, do_microctl,
	"TS supervisory microcontroller access",
	"  Usage: tsmicroctl <options>\n"
	"    -i           Get version\n"
	"    -s <seconds> sleep for x seconds\n"
	"    -w <pct>     Wait until Supercaps are charged to <pct>%\n"
	"    -1           Verbose output when -w is supplied\n"
	"    -e           Enable charging of TS-SILO supercaps\n"
	"    -d           Disable charging of TS-SILO supercaps\n"
);
