/*
 * Copyright (C) 2019 Technologic Systems
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#include <common.h>

#include <asm/arch/mx6-pins.h>
#include <asm/gpio.h>
#include <asm/imx-common/gpio.h>
#include <asm/imx-common/iomux-v3.h>
#include <cli.h>
#include <command.h>
#include <console.h>
#include <i2c.h>
#include <spi.h>
#include <status_led.h>
#include <usb.h>
#include <div64.h>
#include <asm/io.h>
#include <asm/arch/imx-regs.h>
#include <asm/arch/clock.h>

#include "tsfpga.h"
#include "fram.h"

#include <miiphy.h>

static void leds_test(void)
{
	int i;
	red_led_on();
	green_led_on();

	for(i = 0; i < 24; i++){
		if(i % 4 == 0) green_led_on();
		else green_led_off();
		if(i % 4 == 1) red_led_on();
		else red_led_off();
		mdelay(100);
	}

	red_led_on();
	green_led_on();
}

#define UART_TEST_SIZE (32768)

/* Test scratch register & 16550 loopback */
static int uart_16550_test(uint32_t addr)
{
	unsigned long long start, end;
	int ret = 0;
	int i;
	int rd = 0;
	int sz = 0;
	uint8_t reg;
	uint8_t rbuf[UART_TEST_SIZE], wbuf[UART_TEST_SIZE];

	/* Test scratch register */
	for (i = 0; i < 0xff; i++) {
		writeb(i, addr + TS16550_SR);
		if(i != readb(addr + TS16550_SR)) {
			printf("16550 0x%X scratch reg did not retain value\n",
			  addr);
			return 1;
		}
	}

	/* Enable loopback */
	writeb(0x10, addr + TS16550_MCR);

	/* Enable DLAB, select 8n1 */
	writeb(0x83, addr + TS16550_LCR);

	/* Set divisor to 1 */
	writeb(0x1, addr + TS16550_DLL);
	writeb(0x0, addr + TS16550_DLH);

	/* Disable DLAB */
	writeb(0x3, addr + TS16550_LCR);

	/* Enable FIFO */
	writeb(0x1, addr + TS16550_FCR);

	/* clear tx/rx fifos, enable 8-byte trigger level */
	writeb(0x83, addr + TS16550_FCR);

	/* Enable RX/TX IRQs */
	writeb(0x3, addr + TS16550_IER);

	/* Write back 0-UART_TEST_SIZE and read it back */
	for (i = 0; i < UART_TEST_SIZE; i++)
		wbuf[i] = i;

	start = get_ticks();

	while(rd != UART_TEST_SIZE) {
		reg = readb(addr + TS16550_IIR);

		if(ctrlc()) return 1;

		switch((reg >> 1) & 0x7){
		case 0x0:
			readb(addr + TS16550_MSR);
			break;
		case 0x1:
			if(sz < UART_TEST_SIZE) {
				for (i = 0; i < 16; i++) {
					if(sz != UART_TEST_SIZE) {
						writeb(wbuf[sz],
						  addr + TS16550_THR);
						sz++;
						/* Dummy rd following a wr */
						readb(addr + TS16550_SR);
					}
				}
			}
			break;
		case 0x6:
		case 0x2:
			rbuf[rd] = readb(addr + TS16550_RBR);
			rd++;
			break;
		case 0x3:
			readb(addr + TS16550_LSR);
			break;
		default:
			printf("Read back an invalid 16550 IIR value!\n");
			return 1;
		}

		end = get_ticks();

		/* Wait for 10 s max (averages 2.8 seconds). In terms of 8MHz
		 * ticks, this is 80000000 */
		if((end - start) > 80000000) {
			printf("16550 timeout\n");
			return 1;
		}
	}

	/* Turn off IRQs */
	writeb(0x0, addr + TS16550_IER);

	/* Turn off loopback */
	writeb(0x10, addr + TS16550_MCR);

	for (i = 0; i < UART_TEST_SIZE; i++) {
		if(wbuf[i] != rbuf[i]) {
			printf("IDX %d: Expected 0x%X, got 0x%X\n",
			  i, wbuf[i], rbuf[i]);
			ret++;
		}
	}
	if(ret) {
		printf("%d errors\n", ret);
	} else {
		printf("Took %llu 8MHz ticks (%dms)\n",
		  end - start, (uint32_t)(end - start)/8000);
	}

	return ret;
}

/* Tx only without loopback on */
/* XXX: This is very closely matching uart_16550_test, this could be eliminated
 * or just integrated in to uart_16550_test as a function argument */
static int uart_16550_tx_test(uint32_t addr)
{
	unsigned long long start, end;
	int ret = 0;
	int i;
	int sz = 0;
	uint8_t reg;
	uint8_t wbuf[UART_TEST_SIZE];

	/* Test scratch register */
	for (i = 0; i < 0xff; i++) {
		writeb(i, addr + TS16550_SR);
		if(i != readb(addr + TS16550_SR)) {
			printf("16550 0x%X scratch reg did not retain value\n",
			  addr);
			return 1;
		}
	}

	/* Enable DLAB, select 8n1 */
	writeb(0x83, addr + TS16550_LCR);

	/* Set divisor to 1 */
	writeb(0x1, addr + TS16550_DLL);
	writeb(0x0, addr + TS16550_DLH);

	/* Disable DLAB */
	writeb(0x3, addr + TS16550_LCR);

	/* Enable FIFO */
	writeb(0x1, addr + TS16550_FCR);

	/* clear tx/rx fifos, enable 8-byte trigger level */
	writeb(0x83, addr + TS16550_FCR);

	/* Enable TX IRQs */
	writeb(0x2, addr + TS16550_IER);

	/* Write back 0-UART_TEST_SIZE and read it back */
	for (i = 0; i < UART_TEST_SIZE; i++)
		wbuf[i] = i;

	start = get_ticks();

	while(sz != UART_TEST_SIZE) {
		reg = readb(addr + TS16550_IIR);

		if(ctrlc()) return 1;

		switch((reg >> 1) & 0x7){
		case 0x0:
			readb(addr + TS16550_MSR);
			break;
		case 0x1:
			if(sz < UART_TEST_SIZE) {
				for (i = 0; i < 16; i++){
					if(sz != UART_TEST_SIZE){
						writeb(wbuf[sz],
						  addr + TS16550_THR);
						sz++;
						/* Dummy rd following a wr */
						readb(addr + TS16550_SR);
					}
				}
			}
			break;
		case 0x6:
		case 0x2:
			readb(addr + TS16550_RBR);
			break;
		case 0x3:
			readb(addr + TS16550_LSR);
			break;
		default:
			printf("Read back an invalid 16550 IIR value!\n");
			return 1;
		}

		end = get_ticks();
	}

	/* Turn off IRQs */
	writeb(0x0, addr + TS16550_IER);

	printf("TX Took %llu 8MHz ticks (%dms)\n",
	  end - start, (uint32_t)(end - start)/8000);

	return ret;
}

static int micrel_phy_test(void)
{
	int ret = 0;
	unsigned int oui;
	unsigned char model;
	unsigned char rev;

	if (miiphy_info ("FEC1", 0x1, &oui, &model, &rev) != 0) {
		printf("Failed to find PHY\n");
		return 1;
	}

	if(oui != 0x0885) {
		printf("Wrong PHY? Bad OUI 0x%X 0x0885\n", oui);
		ret |= 1;
	}

	if(model != 0x16) {
		printf("Wrong PHY? Bad model 0x%X not 0x16\n", model);
		ret |= 1;
	}

	if (ret == 0) printf("PHY test passed\n");
	else printf("PHY test failed\n");

	return ret;
}

/* Check for M41T00S */
static int rtc_test(void)
{
	int ret;

	i2c_set_bus_num(0);
	ret = i2c_probe(0x68);

	if (ret == 0) printf("RTC test passed\n");
	else printf("RTC test failed\n");
	return ret;
}

/* Short mem test compatible with both 512 and 1 G RAM */
static int mem_test(void)
{
        int ret = 0;
        cmd_tbl_t *cmd;

        /* Arguments to mtest are start, end, pattern, and iterations */
        char *argv[5] = {"mtest", "0x80800000", "0x80810000", "1", "20" };

        cmd = find_cmd("mtest");

        ret |= cmd->cmd(cmd, 0, 5, argv);

        if (ret == 0) printf("RAM test passed\n");
        else printf("RAM test failed\n");
        return ret;
}

/* Verify the functionality of the eMMC
 * This functions sets the current mmc dev to eMMC bus and runs tests.
 * The destructive test will write a pattern of 0xAA that is 4 MiB long starting
 * at eMMC address 0x8080000, or roughtly 2.1 GiB, and then reads it back and
 * verifies. Followed by a pattern of 0x55, also 4 MiB long, also at the same
 * start, this is also read back and verified.
 * If either of the writes or read-backs fails, the test fails.
 * The non-destructive test simple does an enumeration with the mmc command.
 */
static int emmc_test(int destructive)
{
	int ret = 0, i;
	uint32_t *loadaddr = (uint32_t *)0x80800000;
	cmd_tbl_t *cmd;

	/* NOTE: When porting, the last element of query_argv[] needs to be
	 * updated to be the correct mmc bus which the eMMC is on */
	char *query_argv[3] = { "mmc", "dev", "0" };
	char *write_argv[5] = { "mmc", "write", "0x80800000", "0x0", "0x800" };
	char *read_argv[5] = { "mmc", "read", "0x80800000", "0x0", "0x800" };

	cmd = find_cmd("mmc");

	/* This tests simple enumeration */
	ret |= cmd->cmd(cmd, 0, 3, query_argv);

	if(destructive) {
		memset(loadaddr, 0xAAAAAAAA, 1024*1024*4);
		ret |= cmd->cmd(cmd, 0, 5, write_argv);
		memset(loadaddr, 0x00000000, 1024*1024*4);
		ret |= cmd->cmd(cmd, 0, 5, read_argv);

		for (i = 0; i < (1024*1024)/4; i++) {
			if (loadaddr[i] != 0xAAAAAAAA) {
				ret = 1;
			}
		}

		memset(loadaddr, 0x55555555, 1024*1024*4);
		ret |= cmd->cmd(cmd, 0, 5, write_argv);
		memset(loadaddr, 0x00000000, 1024*1024*4);
		ret |= cmd->cmd(cmd, 0, 5, read_argv);

		for (i = 0; i < (1024*1024)/4; i++) {
			if (loadaddr[i] != 0x55555555) {
				ret = 1;
			}
		}

		if (ret == 0) printf("eMMC test passed\n");
		else printf("eMMC test failed\n");
	} else {
		printf("Not running eMMC test!\n");
	}

	return ret;
}

static int do_post_test(cmd_tbl_t *cmdtp, int flag, int argc, char * const argv[])
{
	int ret = 0;
	char *p;
	int destructive = 0;

	if (argv[1][0] == '-') p = &argv[1][1];
	else p = &argv[1][0];

	if (*p == 'd') destructive = 1;

	/* TODO: IO board may or may not have LEDs */
	leds_test();

	//ret |= atmel_wifi_test();
	ret |= rtc_test();
	ret |= mem_test();
	ret |= emmc_test(destructive);
	ret |= micrel_phy_test();
	/* XXX: Add Splash Flash test? */
	/* XXX: Add uC voltage rail check */
	ret |= uart_16550_tx_test(0x50000000);
	ret |= uart_16550_test(0x50000000);

	return ret;
}

U_BOOT_CMD(post, 2, 1,	do_post_test,
	"Runs a POST test",
	"[-d]\n"
	"If -d is supplied, the test is destructive to data on eMMC\n"
);
