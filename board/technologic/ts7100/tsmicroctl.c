#include <common.h>
#include <command.h>
#include <asm/io.h>
#include <asm/arch/imx-regs.h>
#include <asm/arch/clock.h>
#include <asm/arch/gpio.h>
#include <asm/gpio.h>
#include <status_led.h>
#include "tsfpga.h"
#include "c2.h"

unsigned int c2_len;
unsigned char *c2_data;

int do_silabs_info(void)
{
	/* Not implemented */
	return 1;
}

int do_silabs_supercaps(uint pct)
{
	/* Not implemented */
	return 1;
}

int do_silabs_sleep(uint seconds)
{
	/* Not implemented */
	return 1;
}

int do_silabs_program(unsigned char *data, unsigned int len)
{
	c2_len = len;
	c2_data = data;

	/* Force 5V to stay on during programming */
	fpga_dio2_dat_set(BANK2_EN_PROG_SILAB);
	fpga_dio2_oe_set(BANK2_EN_PROG_SILAB);
	udelay(1000*1000);

	/* CLK is inverted, drive low to keep actual signal high */
	fpga_dio1_dat_clr(BANK1_SILAB_CLK);
	fpga_dio1_oe_set(BANK1_SILAB_CLK);

	fpga_dio1_dat_set(BANK1_SILAB_DATA);
	fpga_dio1_oe_set(BANK1_SILAB_DATA);

	blast_silabs();

	fpga_dio2_oe_clr(BANK2_EN_PROG_SILAB);

	return 1;
}

int c2d_get(void)
{
	fpga_dio1_data_get(BANK1_SILAB_DATA);
	fpga_dio1_data_get(BANK1_SILAB_DATA);
	fpga_dio1_data_get(BANK1_SILAB_DATA);
	return fpga_dio1_data_get(BANK1_SILAB_DATA);
}

 /* 1, 0, or ‘z’ */
void c2d_set(unsigned char state)
{
	c2d_get();
	if (state == 'z') {
		fpga_dio1_oe_clr(BANK1_SILAB_CLK);
	} else if (state == 1) {
		fpga_dio1_dat_set(BANK1_SILAB_CLK);
		fpga_dio1_oe_set(BANK1_SILAB_CLK);
	} else {
		fpga_dio1_dat_clr(BANK1_SILAB_CLK);
		fpga_dio1_oe_set(BANK1_SILAB_CLK);
	}
	c2d_get();
}

unsigned int c2_fopen(void) {
	return c2_len;
}

unsigned char c2_getc(void) {
	unsigned char ret;
	ret = (*c2_data++);
	return ret;
}

void c2_reset(void) {
	fpga_dio1_dat_set(BANK1_SILAB_CLK);
	udelay(25*1000);
	fpga_dio1_dat_clr(BANK1_SILAB_CLK);
	udelay(1);
}

void c2ck_set(unsigned char state)
{
	c2d_get();
	if(state == 'z') {

	} else if (state == 1) {
		fpga_dio1_dat_clr(BANK1_SILAB_CLK);
	} else {
		fpga_dio1_dat_set(BANK1_SILAB_CLK);
	}
	c2d_get();
}

void c2ck_strobe(void)
{
	fpga_dio1_dat_set(BANK1_SILAB_CLK);
	c2d_get();
	fpga_dio1_dat_clr(BANK1_SILAB_CLK);
	c2d_get();
}

static int do_tsmicroctl(cmd_tbl_t *cmdtp, int flag, int argc, char * const argv[])
{
	if(argc == 1)
		return do_silabs_info();

	if(argc != 3 && argc != 4)
		return 1;

	if (strcmp(argv[1], "-p") == 0) {
		return do_silabs_program((unsigned char *)simple_strtoul(argv[2], NULL, 16),
					 simple_strtoul(argv[3], NULL, 16));
	} else if (strcmp(argv[1], "-s") == 0) {
		return do_silabs_sleep(simple_strtoul(argv[2], NULL, 16));
	} else if (strcmp(argv[1], "-b") == 0) {
		return do_silabs_supercaps(simple_strtoul(argv[2], NULL, 16));
	} else {
		return 1;
	}
	return 0;
}

U_BOOT_CMD(tsmicroctl, 4, 0, do_tsmicroctl,
	"Utility for managing onboard microcontroller",
	"-p [image address] [filesize]\n"
	"tsmicroctl -s [seconds to sleep]\n"
	"tsmicroctl -b [Enable supercaps, charge to pct]\n"
);
