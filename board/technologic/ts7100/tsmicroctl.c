#include <common.h>
#include <command.h>
#include <console.h>
#include <i2c.h>
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

#define I2C_ADR 0x54

/*******************************************
 * Supervisory microcontroller interface
 ******************************************/

int silab_rev(void)
{
	uint8_t val;
	i2c_set_bus_num(0);
	i2c_read(I2C_ADR, 0x800, 2, &val, 1);
	return val;
}

uint16_t get_silo_pct(void)
{
	uint8_t val[2];
	int16_t pct;
	uint16_t mv;

	i2c_set_bus_num(0);
	i2c_read(I2C_ADR, 16, 2, val, 2);

	mv = (uint16_t)(val[0] << 8) | val[1];
	pct = ((int32_t)((mv - 3430) * 10) / 117);
	if (pct > 100) pct = 100;
	if (pct < 0) pct = 0;

	return (uint16_t)pct;
}

/* This pulled almost directly from the userspace tsmicroctl.
 * More information about all of the steps is available there.
 */
int do_silabs_info(void)
{
	int i, ret;
	uint8_t buf_8[28];
	uint32_t wdt_ms;
	uint16_t *buf_16 = (uint16_t *)buf_8;

	const char *an_names[11] = {
	  "MV5", "MV_SILO_CHARGE", "MV3P3", "MVIN", "INIT_TEMP", "", "",
	  "MV_SILO", "MV_SILO_TOT", "", "CUR_TEMP"};


        ret = i2c_read(I2C_ADR, 0, 2, buf_8, 28);
        if(ret){
                printf("I2C Read failed with %d\n", ret);
                return 1;
        }

        for (i = 0; i < 11; i++) {
                buf_16[i] =
                  ((uint16_t)(buf_8[(i << 1)] << 8) | buf_8[((i << 1)|1)]);
        }

        printf("REVISION=0x%X\n", silab_rev());
        for (i = 0; i < 11; i++) {
                if (*an_names[i]) {
                        printf("%s=%d\n", an_names[i], buf_16[i]);
                }
        }
        printf("TEMPERATURE=%d\n", buf_16[10]);
        printf("USB_CONN_CONNECTED=%d\n", !!(buf_8[22] & (1 << 4)));

        printf("SILO=");
        if      ((buf_8[22]) & 1) printf("discharging\n");
        else if ((buf_8[22]) & (1 << 2)) printf("charged\n");
        else if ((buf_8[22]) & (1 << 3)) printf("charging\n");
        else printf("disabled\n");
        printf("SILO_CHARGE_PCT=%d\n", get_silo_pct());
        printf("SILO_DEFAULT=");
        if (buf_8[23] & 1) printf("enabled\n");
        else printf("disabled\n");
        printf("SILO_CHARGE_CUR=%d\n", ((uint16_t)(buf_8[26] << 8)|buf_8[27]));
        printf("SILO_DEF_CHARGE_CUR=%d\n",
	  ((uint16_t)(buf_8[24] << 8) | buf_8[25]));
        printf("WDT_STATUS=%s\n",(buf_8[22] & (1 << 6) ? "armed" : "disabled"));
        printf("WDT_DEF_STATUS=%s\n",(buf_8[23] & (1 << 1) ?
	  "disabled" : "armed"));

        ret = i2c_read(I2C_ADR, 1024, 2, buf_8, 5);
        if(ret){
                printf("I2C Read failed with %d\n", ret);
                return 1;
        }
        wdt_ms = 0;
        for (i = 0; i < 4; i++) {
                wdt_ms |= (uint32_t)(buf_8[i] << (8 * i));
        }
        wdt_ms *= 10;
        printf("WDT_TIMEOUT_LEN=%d\n", wdt_ms);
        printf("REBOOT_SOURCE=%s\n", (buf_8[4] & (1 << 7)) ? "WDT" : "poweron");

	return 0;
}

int do_tssilo_charge(uint8_t val)
{
	uint8_t buf;

	i2c_set_bus_num(0);
	i2c_read(I2C_ADR, 22, 2, &buf, 1);

	if (val) buf |= (1 << 1);
	else buf &= ~(1 << 1);

	i2c_write(I2C_ADR, 22, 2, &buf, 1);

	return 0;
}

int set_timeout(uint32_t sec)
{
	uint8_t buf[4];

	/* uC sleep/WDT is in centiseconds
	 * Lowest I2C address contains the LSB.
	 */
	sec *= 100;
	buf[0] = sec & 0xFF;
	buf[1] = (sec >> 8) & 0xFF;
	buf[2] = (sec >> 16) & 0xFF;
	buf[3] = (sec >> 24) & 0xFF;

	i2c_set_bus_num(0);
	i2c_write(I2C_ADR, 1024, 2, buf, 4);

	return 0;
}

int do_sleep(uint32_t sec)
{
	uint8_t buf = 0x02; /* Sleep command */

	set_timeout(sec);
	i2c_set_bus_num(0);
	i2c_write(I2C_ADR, 1028, 2, &buf, 1);

	/* Spin forever until we go to sleep */
	while(1);

	return 0;
}

int set_wdt_default(int val)
{
	uint8_t buf;

	i2c_set_bus_num(0);
	/* Read from flag register */
	i2c_read(I2C_ADR, 23, 2, &buf, 1);

	/* The flag meaning is inverted */
	if (val) {
		buf &= ~(0x2);
	} else {
		buf |= 0x2;
	}

	i2c_write(I2C_ADR, 23, 2, &buf, 1);

	return 0;
}

int do_silabs_waitcharge(uint32_t pct, uint32_t verbose)
{
        uint16_t check;

	do_tssilo_charge(1);

        if (pct > 100) pct = 100;

        if(pct == 0) {
                printf("Not waiting for SuperCaps to charge\n");
                return 0;
        } else {
                printf("Waiting until SuperCaps are charged to %d%%\n", pct);
        }

        while(1) {
		check = get_silo_pct();
		if (verbose) printf("%d%%\n", check);
		if (check >= pct) return 0;

		if (ctrlc()) return 1;
		udelay(1000000);
	}

	return 0;
}

static int do_tsmicroctl(cmd_tbl_t *cmdtp, int flag, int argc, char * const argv[])
{
	int i;
	uint32_t pct = 0;
	uint32_t verbose = 0;
	uint8_t buf;
	char *p;

	i2c_set_bus_num(0);

	if (argc == 1) {
		printf("%s - %s\n\nUsage:\n%s %s\n",
		  cmdtp->name, cmdtp->usage, cmdtp->name, cmdtp->help);
		return 1;
	}


	for (i = 1; i < argc; i++) {
		if(argv[i][0] == '-') {
			p = &argv[i][1];
		} else {
			p = &argv[i][0];
		}

		switch(p[0]) {
		  case 'i':
			return do_silabs_info();
			break;
		  case 'w':
			pct = simple_strtoul(argv[(++i)], NULL, 0);
			break;
		  case 'v':
			verbose = 1;
			break;
		  case 'e':
			return do_tssilo_charge(1);
			break;
		  case 'd':
			return do_tssilo_charge(0);
			break;
		  case 's':
			do_sleep(simple_strtoul(argv[(++i)], NULL, 0));
			break;
		  case 'a':
			return set_wdt_default(1);
			break;
		  case 'A':
			return set_wdt_default(0);
			break;
		  case 't':
			return set_timeout(simple_strtoul(argv[(++i)], NULL,
			  0));
			break;
		  case 'f':
			buf = 1; /* WDT feed command */
			i2c_set_bus_num(0);
			return i2c_write(I2C_ADR, 1028, 2, &buf, 1);
			break;
		  default:
			printf("Unknown option '%s'\n", argv[i]);
			return 1;
			break;
		}
	}

	if (pct) return do_silabs_waitcharge(pct, verbose);
	return 0;
}

U_BOOT_CMD(tsmicroctl, 4, 0, do_tsmicroctl,
	"Utility for managing onboard microcontroller",
	"-i		Information\n"
	"tsmicroctl -s <sec>	Sleep system for <sec>\n"
	"tsmicroctl -e		Enable TS-SILO charging\n"
	"tsmicroctl -d		Disable TS-SILO charging\n"
	"tsmicroctl -w <p> [-v]	Delay until TS-SILO is charged to <p>%.\n"
	"				-v to en. verbose output\n"
	"tsmicroctl -a		Arm WDT for 600 s at poweron\n"
	"tsmicroctl -A		Disarm WDT at poweron\n"
	"tsmicroctl -t <sec>	Set WDT timeout to <sec>. Set to 0 and\n"
	"				feed to disable WDT.\n"
	"tsmicroctl -f		Feed WDT for prev. set <sec> timeout\n"

);

/*************************************
 *  C8051 C2 reprogramming interface
 *************************************/

int c2d_get(void)
{
	return fpga_dio1_data_get(BANK1_SILAB_DATA);
}

 /* 1, 0, or ‘z’ */
void c2d_set(unsigned char state)
{
	switch (state) {
	  default:
	  case 'z':
		fpga_dio1_oe_clr(BANK1_SILAB_DATA);
		fpga_dio1_dat_clr(BANK1_SILAB_DATA);
		break;
	  case 1:
		fpga_dio1_dat_set(BANK1_SILAB_DATA);
		fpga_dio1_oe_set(BANK1_SILAB_DATA);
		break;
	  case 0:
		fpga_dio1_dat_clr(BANK1_SILAB_DATA);
		fpga_dio1_oe_set(BANK1_SILAB_DATA);
		break;
	}
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
	udelay(25*100000);
	fpga_dio1_dat_clr(BANK1_SILAB_CLK);
	udelay(1);
}

void c2ck_set(unsigned char state)
{
	switch (state) {
	  case 1:
		/* TS-7100 SILAB_CLK from the FPGA is inverted. This bit drives
		 * a MOSFET gate which pulls the clock pin low. This is to
		 * prevent conflict when TS-9471 is installed which does direct
		 * drive of the SILAB_CLK to the uC itself.
		 */
		fpga_dio1_dat_clr(BANK1_SILAB_CLK);
		break;
	  case 0:
		/* This FPGA has a bit specifically for strobing SILAB_CLK.
		 * Technically, this case should never be reached. All of the
		 * code that calls c2ck_set() as a part of c2.c only ever calls
		 * it to set as a 1. It will then use the c2ck_strobe() function
		 * to issue a clock. For sake of completeness, we will set the
		 * clock here.
		 */
		fpga_dio1_dat_set(BANK1_SILAB_CLK);
		break;
	  case 'z':
	  default:
		break;
	}
}

void c2ck_strobe(void)
{
	/* SILAB_CLK_STB is auto-clearing after one wb_clk of being asserted. */
	fpga_dio3_dat_set(BANK3_SILAB_CLK_STB);
}

int do_silabs_program(cmd_tbl_t *cmdtp, int flag, int argc, char * const argv[])
{
	if (argc == 1) {
		printf("%s - %s\n\nUsage:\n%s %s\n",
		  cmdtp->name, cmdtp->usage, cmdtp->name, cmdtp->help);
		return 1;
	}

        c2_data = (unsigned char *)simple_strtoul(argv[1], NULL, 16);
        c2_len = simple_strtoul(argv[2], NULL, 16);

	/* Force 5V to stay on during programming */
	fpga_dio2_dat_set(BANK2_EN_PROG_SILAB);
	fpga_dio2_oe_set(BANK2_EN_PROG_SILAB);
	udelay(1000*1000);

	/* CLK is inverted, drive low to keep actual signal high
	 * This FPGA is set up to have a separate GPIO be a clock one-shot
	 * with auto clear. This reduces the possibility of some CPU delay
	 * during programming causing the uC to reset. Holding the C2 clk low
	 * in the uC for a long enough period will hardware reset the uC.
	 */
	fpga_dio1_dat_clr(BANK1_SILAB_CLK);
	fpga_dio1_oe_set(BANK1_SILAB_CLK);

	fpga_dio1_dat_set(BANK1_SILAB_DATA);
	fpga_dio1_oe_set(BANK1_SILAB_DATA);

	blast_silabs();

	fpga_dio2_oe_clr(BANK2_EN_PROG_SILAB);

	return 0;
}

U_BOOT_CMD(silabs, 3, 0, do_silabs_program,
        "TS supervisory microcontroller re-flashing tool",
        "<image address> <length>\n"
	"\nMISUSE MAY DAMAGE DEVICE!\n"
);
