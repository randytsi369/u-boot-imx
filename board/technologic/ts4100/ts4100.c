/*
 * Copyright (C) 2016 Technologic Systems
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#include <asm/arch/clock.h>
#include <asm/arch/crm_regs.h>
#include <asm/arch/iomux.h>
#include <asm/arch/imx-regs.h>
#include <asm/arch/mx6-pins.h>
#include <asm/arch/sys_proto.h>
#include <asm/gpio.h>
#include <asm/imx-common/iomux-v3.h>
#include <asm/imx-common/boot_mode.h>
#include <asm/imx-common/mxc_i2c.h>
#include <asm/io.h>
#include <common.h>
#include <fpga.h>
#include <fsl_esdhc.h>
#include <fuse.h>
#include <i2c.h>
#include <lattice.h>
#include <linux/sizes.h>
#include <linux/fb.h>
#include <miiphy.h>
#include <mmc.h>
#include <mxsfb.h>
#include <netdev.h>
#include <usb.h>
#include <usb/ehci-fsl.h>
#include "fpga.h"
#include "silabs.h"

DECLARE_GLOBAL_DATA_PTR;

#define UART_PAD_CTRL  (PAD_CTL_PKE | PAD_CTL_PUE |		\
	PAD_CTL_PUS_100K_UP | PAD_CTL_SPEED_MED |		\
	PAD_CTL_DSE_40ohm   | PAD_CTL_SRE_FAST  | PAD_CTL_HYS)

#define USDHC_PAD_CTRL (PAD_CTL_PKE | PAD_CTL_PUE |		\
	PAD_CTL_PUS_22K_UP  | PAD_CTL_SPEED_LOW |		\
	PAD_CTL_DSE_80ohm   | PAD_CTL_SRE_FAST  | PAD_CTL_HYS)

#define USDHC_PAD_CTRL_WP (PAD_CTL_PKE | PAD_CTL_PUE |		\
	PAD_CTL_PUS_100K_DOWN | PAD_CTL_SPEED_LOW |		\
	PAD_CTL_DSE_80ohm   | PAD_CTL_SRE_FAST  | PAD_CTL_HYS)

#define ENET_PAD_CTRL  (PAD_CTL_PUS_100K_UP | PAD_CTL_PUE |     \
	PAD_CTL_SPEED_HIGH   |                                   \
	PAD_CTL_DSE_48ohm   | PAD_CTL_SRE_FAST)

#define ENET_CLK_PAD_CTRL  (PAD_CTL_SPEED_MED | \
	PAD_CTL_DSE_120ohm   | PAD_CTL_SRE_FAST)

#define ENET_RX_PAD_CTRL  (PAD_CTL_PKE | PAD_CTL_PUE |          \
	PAD_CTL_SPEED_HIGH   | PAD_CTL_SRE_FAST)

#define I2C_PAD_CTRL    (PAD_CTL_PKE | PAD_CTL_PUE |            \
	PAD_CTL_PUS_100K_UP | PAD_CTL_SPEED_MED |               \
	PAD_CTL_DSE_40ohm | PAD_CTL_HYS |			\
	PAD_CTL_ODE)

#define LCD_PAD_CTRL    (PAD_CTL_HYS | PAD_CTL_PUS_100K_UP | PAD_CTL_PUE | \
	PAD_CTL_PKE | PAD_CTL_SPEED_MED | PAD_CTL_DSE_40ohm)

#define OTG_ID_PAD_CTRL (PAD_CTL_PKE | PAD_CTL_PUE |		\
	PAD_CTL_PUS_47K_UP  | PAD_CTL_SPEED_LOW |		\
	PAD_CTL_DSE_80ohm   | PAD_CTL_SRE_FAST  | PAD_CTL_HYS)

#define PC MUX_PAD_CTRL(I2C_PAD_CTRL)

#define EN_FPGA_PWR             IMX_GPIO_NR(5, 2)
#define FPGA_RESETN             IMX_GPIO_NR(4, 11)
#define JTAG_FPGA_TDO           IMX_GPIO_NR(5, 4)
#define JTAG_FPGA_TDI           IMX_GPIO_NR(5, 5)
#define JTAG_FPGA_TMS           IMX_GPIO_NR(5, 6)
#define JTAG_FPGA_TCK           IMX_GPIO_NR(5, 7)
#define EN_ETH_PHY_PWR 		IMX_GPIO_NR(1, 10)
#define PHY1_DUPLEX 		IMX_GPIO_NR(2, 0)
#define PHY2_DUPLEX 		IMX_GPIO_NR(2, 8)
#define PHY1_PHYADDR2 		IMX_GPIO_NR(2, 1)
#define PHY2_PHYADDR2		IMX_GPIO_NR(2, 9)
#define PHY1_CONFIG_2 		IMX_GPIO_NR(2, 2)
#define PHY2_CONFIG_2		IMX_GPIO_NR(2, 10)
#define PHY1_ISOLATE		IMX_GPIO_NR(2, 7)
#define PHY2_ISOLATE		IMX_GPIO_NR(2, 15)

/* I2C1 for Silabs */
struct i2c_pads_info i2c_pad_info1 = {
	.scl = {
		.i2c_mode = MX6_PAD_GPIO1_IO02__I2C1_SCL | PC,
		.gpio_mode = MX6_PAD_GPIO1_IO02__GPIO1_IO02 | PC,
		.gp = IMX_GPIO_NR(1, 2),
	},
	.sda = {
		.i2c_mode = MX6_PAD_GPIO1_IO03__I2C1_SDA | PC,
		.gpio_mode = MX6_PAD_GPIO1_IO03__GPIO1_IO03 | PC,
		.gp = IMX_GPIO_NR(1, 3),
	},
};

/* I2C3 for FPGA/offbd */
struct i2c_pads_info i2c_pad_info3 = {
	.scl = {
		.i2c_mode = MX6_PAD_LCD_DATA01__I2C3_SCL | PC,
		.gpio_mode = MX6_PAD_LCD_DATA01__GPIO3_IO06 | PC,
		.gp = IMX_GPIO_NR(3, 6),
	},
	.sda = {
		.i2c_mode = MX6_PAD_LCD_DATA00__I2C3_SDA | PC,
		.gpio_mode = MX6_PAD_LCD_DATA00__GPIO3_IO05 | PC,
		.gp = IMX_GPIO_NR(3, 5),
	},
};

int dram_init(void)
{
	gd->ram_size = PHYS_SDRAM_SIZE;

	return 0;
}

iomux_v3_cfg_t const fpga_jtag_pads[] = {
	/* JTAG_FPGA_TDO */
	MX6_PAD_SNVS_TAMPER4__GPIO5_IO04 | MUX_PAD_CTRL(NO_PAD_CTRL), 
	/* JTAG_FPGA_TDI */
	MX6_PAD_SNVS_TAMPER5__GPIO5_IO05 | MUX_PAD_CTRL(NO_PAD_CTRL),
	/* JTAG_FPGA_TMS */
	MX6_PAD_SNVS_TAMPER6__GPIO5_IO06 | MUX_PAD_CTRL(NO_PAD_CTRL),
	/* JTAG_FPGA_TCK */
	MX6_PAD_SNVS_TAMPER7__GPIO5_IO07 | MUX_PAD_CTRL(NO_PAD_CTRL),
	/* EN_FPGA_PWR */
	MX6_PAD_SNVS_TAMPER2__GPIO5_IO02 | MUX_PAD_CTRL(NO_PAD_CTRL),
	/* FPGA_RESET# */
	MX6_PAD_NAND_WP_B__GPIO4_IO11 | MUX_PAD_CTRL(NO_PAD_CTRL), 
};

void fpga_mmc_init(void)
{
	/* Rev B has SD power connected to TAMPER9, already biased high */
}

void fpga_late_init(void)
{
	int sdboot;
	int uboot;

	/* Onboard jumpers to boot to SD or break in u-boot */
	fpga_gpio_output(OFF_BD_RESET_PADN, 1);
	fpga_gpio_output(OFF_BD_RESET_PADN, 0);
	sdboot = fpga_gpio_input(DIO_20);
	uboot = fpga_gpio_input(DIO_43);

	/* While OFF_BD_RESET_PADN is low read CN1_98 which 
	 * will have a pulldown to OFF_BD_RESET_PADN if the sd
	 * boot jumper is on */
	if(sdboot)
		setenv("jpsdboot", "off");
	else
		setenv("jpsdboot", "on");

	if(uboot)
		setenv("jpuboot", "off");
	else
		setenv("jpuboot", "on");

	mdelay(10);
	fpga_gpio_output(OFF_BD_RESET_PADN, 1);
	fpga_gpio_output(EN_USB_HOST_5V_PAD, 1);
}

#if defined(CONFIG_FPGA)

static void ts4100_fpga_jtag_init(void)
{
	gpio_direction_output(JTAG_FPGA_TDI, 1);
	gpio_direction_output(JTAG_FPGA_TCK, 1);
	gpio_direction_output(JTAG_FPGA_TMS, 1);
	gpio_direction_input(JTAG_FPGA_TDO);
}

static void ts4100_fpga_done(void)
{
	gpio_direction_input(JTAG_FPGA_TDI);
	gpio_direction_input(JTAG_FPGA_TCK);
	gpio_direction_input(JTAG_FPGA_TMS);
	gpio_direction_input(JTAG_FPGA_TDO);

	/* During FPGA programming several important pins will
	 * have been tristated.  Put it back to normal */
	fpga_mmc_init();
	fpga_late_init();
	red_led_on();
	green_led_off();
}

static void ts4100_fpga_tdi(int value)
{
	gpio_set_value(JTAG_FPGA_TDI, value);
}

static void ts4100_fpga_tms(int value)
{
	gpio_set_value(JTAG_FPGA_TMS, value);
}

static void ts4100_fpga_tck(int value)
{
	gpio_set_value(JTAG_FPGA_TCK, value);
}

static int ts4100_fpga_tdo(void)
{
	return gpio_get_value(JTAG_FPGA_TDO);
}

lattice_board_specific_func ts4100_fpga_fns = {
	ts4100_fpga_jtag_init,
	ts4100_fpga_tdi,
	ts4100_fpga_tms,
	ts4100_fpga_tck,
	ts4100_fpga_tdo,
	ts4100_fpga_done
};

Lattice_desc ts4100_fpga = {
	Lattice_XP2,
	lattice_jtag_mode,
	589012,
	(void *) &ts4100_fpga_fns,
	NULL,
	0,
	"machxo_2_cb132"
};

int ts4100_fpga_init(void)
{
	fpga_init();
	fpga_add(fpga_lattice, &ts4100_fpga);

	return 0;
}

#endif

static iomux_v3_cfg_t const uart1_pads[] = {
	MX6_PAD_UART1_TX_DATA__UART1_DCE_TX | MUX_PAD_CTRL(UART_PAD_CTRL),
	MX6_PAD_UART1_RX_DATA__UART1_DCE_RX | MUX_PAD_CTRL(UART_PAD_CTRL),
	MX6_PAD_JTAG_TMS__CCM_CLKO1 | MUX_PAD_CTRL(NO_PAD_CTRL),
};

static iomux_v3_cfg_t const usdhc1_sd_pads[] = {
	MX6_PAD_SD1_CLK__USDHC1_CLK | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD1_CMD__USDHC1_CMD | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD1_DATA0__USDHC1_DATA0 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD1_DATA1__USDHC1_DATA1 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD1_DATA2__USDHC1_DATA2 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD1_DATA3__USDHC1_DATA3 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_GPIO1_IO05__USDHC1_VSELECT | MUX_PAD_CTRL(NO_PAD_CTRL),
};

static iomux_v3_cfg_t const usdhc2_emmc_pads[] = {
	MX6_PAD_NAND_RE_B__USDHC2_CLK | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_NAND_WE_B__USDHC2_CMD | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_NAND_DATA00__USDHC2_DATA0 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_NAND_DATA01__USDHC2_DATA1 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_NAND_DATA02__USDHC2_DATA2 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_NAND_DATA03__USDHC2_DATA3 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
};

static void setup_iomux_uart(void)
{
	imx_iomux_v3_setup_multiple_pads(uart1_pads, ARRAY_SIZE(uart1_pads));
}

static struct fsl_esdhc_cfg usdhc_cfg[2] = {
	{USDHC1_BASE_ADDR, 0, 4},
	{USDHC2_BASE_ADDR, 0, 4},
};

#define USDHC1_VSELECT IMX_GPIO_NR(1, 5)

int board_mmc_getcd(struct mmc *mmc)
{
	return 1;
}

int board_mmc_init(bd_t *bis)
{
	/*
	 * According to the board_mmc_init() the following map is done:
	 * (U-boot device node)    (Physical Port)
	 * mmc0                    USDHC1 (SD/WIFI)
	 * mmc1                    USDHC2 (eMMC)
	 */
	fpga_mmc_init();

	/* For the SD Select 3.3V instead of 1.8V */
	gpio_direction_output(USDHC1_VSELECT, 1);

	imx_iomux_v3_setup_multiple_pads(
		usdhc1_sd_pads, ARRAY_SIZE(usdhc1_sd_pads));
	usdhc_cfg[0].sdhc_clk = mxc_get_clock(MXC_ESDHC_CLK);

	imx_iomux_v3_setup_multiple_pads(
		usdhc2_emmc_pads, ARRAY_SIZE(usdhc2_emmc_pads));
	usdhc_cfg[1].sdhc_clk = mxc_get_clock(MXC_ESDHC2_CLK);

	if (fsl_esdhc_initialize(bis, &usdhc_cfg[0]))
		printf("Warning: failed to initialize sd dev\n");

	if (fsl_esdhc_initialize(bis, &usdhc_cfg[1]))
		printf("Warning: failed to initialize emmc dev\n");

	return 0;
}

#ifdef CONFIG_FEC_MXC

static iomux_v3_cfg_t const fec_enet_pads[] = {
	MX6_PAD_GPIO1_IO06__ENET1_MDIO | MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_GPIO1_IO07__ENET1_MDC | MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_ENET1_TX_DATA0__ENET1_TDATA00 | MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_ENET1_TX_DATA1__ENET1_TDATA01 | MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_ENET1_TX_EN__ENET1_TX_EN | MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_ENET1_TX_CLK__ENET1_REF_CLK1 | MUX_PAD_CTRL(ENET_CLK_PAD_CTRL),
	MX6_PAD_ENET1_RX_DATA0__ENET1_RDATA00 | MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_ENET1_RX_DATA1__ENET1_RDATA01 | MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_ENET1_RX_ER__ENET1_RX_ER | MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_ENET1_RX_EN__ENET1_RX_EN | MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_ENET2_TX_DATA0__ENET2_TDATA00 | MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_ENET2_TX_DATA1__ENET2_TDATA01 | MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_ENET2_RX_DATA0__ENET2_RDATA00 | MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_ENET2_RX_DATA1__ENET2_RDATA01 | MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_ENET2_RX_EN__ENET2_RX_EN | MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_ENET2_RX_ER__ENET2_RX_ER | MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_ENET2_TX_DATA0__ENET2_TDATA00 | MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_ENET2_TX_DATA1__ENET2_TDATA01 | MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_ENET2_TX_EN__ENET2_TX_EN | MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_ENET2_TX_CLK__ENET2_REF_CLK2 | MUX_PAD_CTRL(ENET_PAD_CTRL),
};

static iomux_v3_cfg_t const fec_enet_pads1[] = {
	MX6_PAD_GPIO1_IO06__ENET1_MDIO | MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_GPIO1_IO07__ENET1_MDC | MUX_PAD_CTRL(ENET_PAD_CTRL),
	/* DUPLEX */
	MX6_PAD_ENET1_RX_DATA0__GPIO2_IO00 | MUX_PAD_CTRL(ENET_PAD_CTRL),
	/* DUPLEX */
	MX6_PAD_ENET2_RX_DATA0__GPIO2_IO08 | MUX_PAD_CTRL(ENET_PAD_CTRL),
	/* PHYADDR2 */
	MX6_PAD_ENET1_RX_DATA1__GPIO2_IO01 | MUX_PAD_CTRL(ENET_PAD_CTRL),
	/* PHYADDR2 */
	MX6_PAD_ENET2_RX_DATA1__GPIO2_IO09 | MUX_PAD_CTRL(ENET_PAD_CTRL),
	/* CONFIG_2 */
	MX6_PAD_ENET1_RX_EN__GPIO2_IO02 | MUX_PAD_CTRL(ENET_PAD_CTRL),
	/* CONFIG_2 */
	MX6_PAD_ENET2_RX_EN__GPIO2_IO10 | MUX_PAD_CTRL(ENET_PAD_CTRL),
	/* Isolate */
	MX6_PAD_ENET1_RX_ER__GPIO2_IO07 | MUX_PAD_CTRL(ENET_PAD_CTRL),
	/* Isolate */
	MX6_PAD_ENET2_RX_ER__GPIO2_IO15 | MUX_PAD_CTRL(ENET_PAD_CTRL),
	/* EN_ETH_PHY_PWR */
	MX6_PAD_JTAG_MOD__GPIO1_IO10 | MUX_PAD_CTRL(ENET_PAD_CTRL)
};

int board_eth_init(bd_t *bis)
{
	int ret, i;
	uint32_t uniq1, uniq2;
	uchar enetaddr[6];

	/* Set pins to strapping GPIO modes */
	imx_iomux_v3_setup_multiple_pads(fec_enet_pads1,
					 ARRAY_SIZE(fec_enet_pads1));

	/* Reset */
	gpio_direction_output(EN_ETH_PHY_PWR, 0);
	fpga_gpio_output(ETH_PHY_RESET, 1);
	mdelay(5); // falls in ~2ms
	gpio_direction_output(EN_ETH_PHY_PWR, 1);

	gpio_direction_output(PHY1_DUPLEX, 0);
	gpio_direction_output(PHY2_DUPLEX, 0);
	gpio_direction_output(PHY1_PHYADDR2, 0);
	gpio_direction_output(PHY2_PHYADDR2, 0);
	gpio_direction_output(PHY1_CONFIG_2, 0);
	gpio_direction_output(PHY2_CONFIG_2, 0);
	gpio_direction_output(PHY1_ISOLATE, 0);
	gpio_direction_output(PHY2_ISOLATE, 0);

	/* PHYs require minimum 10 ms from power valid, to unreset
	 * Strap values are read ~at unreset time.
	 * We wait 15 ms just to provide some additional margin
	 */
	mdelay(15);
	fpga_gpio_output(ETH_PHY_RESET, 0);

	/* Set pins to enet modes */
	imx_iomux_v3_setup_multiple_pads(fec_enet_pads,
					 ARRAY_SIZE(fec_enet_pads));

	ret = fecmxc_initialize_multi(bis, CONFIG_FEC_ENET_DEV,
				       CONFIG_FEC_MXC_PHYADDR, IMX_FEC_BASE);

	/* If env var ethaddr is not set, then the fuse'ed MAC is not a
	 * proper MAC (generally just not set).  In this case, a random one
	 * needs to be generated.  Stock generation uses time as the srand()
	 * seed, and this is a bad idea when multiple units are done on the
	 * rack at the same time.  This pulls two unique IDs from fuses;
	 * correspond to lot number and part in lot number, and sets srand()
	 * from the XOR of those two values.
	 */
	if (!eth_getenv_enetaddr("ethaddr", enetaddr)) {
		printf("Mo MAC address set in fuses. Using random MAC\n");

		/* Read two unique IDs stored in OTP fuses */
		fuse_read(0, 1, &uniq1);
		fuse_read(0, 2, &uniq2);

		srand(uniq1 ^ uniq2);
		for (i = 0; i < 6; i++) {
			enetaddr[i] = rand();
		}
		enetaddr[0] &= 0xfe;	/* Clear multicast bit */
		enetaddr[0] |= 0x02;	/* Set local assignment bit (IEEE802) */

		if (eth_setenv_enetaddr("ethaddr", enetaddr)) {
			printf("Failed to set a random MAC address\n");
		}
	}

	return ret;
}

static int setup_fec(int fec_id)
{
	struct iomuxc *const iomuxc_regs = (struct iomuxc *)IOMUXC_BASE_ADDR;
	int ret;

	if (check_module_fused(MX6_MODULE_ENET1))
		return -1;

	if (check_module_fused(MX6_MODULE_ENET2))
		return -1;

	/*
	 * Use 50M anatop loopback REF_CLK1 for ENET1,
	 * clear gpr1[13], set gpr1[17].
	 */
	clrsetbits_le32(&iomuxc_regs->gpr[1], IOMUX_GPR1_FEC1_MASK,
			IOMUX_GPR1_FEC1_CLOCK_MUX1_SEL_MASK);

	clrsetbits_le32(&iomuxc_regs->gpr[1], IOMUX_GPR1_FEC2_MASK,
			IOMUX_GPR1_FEC2_CLOCK_MUX1_SEL_MASK);

	ret = enable_fec_anatop_clock(0, ENET_50MHZ);
	ret |= enable_fec_anatop_clock(1, ENET_50MHZ);
	if (ret)
		return ret;

	enable_enet_clk(1);

	return 0;
}

int board_phy_config(struct phy_device *phydev)
{
	/* Reset phy 1, phy 2 is reset by default */
	phydev->bus->write(phydev->bus, 0x1, MDIO_DEVAD_NONE, 0x0, 0x8000);
	/* Disable BCAST, select RMII */
	phydev->bus->write(phydev->bus, 0x1, MDIO_DEVAD_NONE, 0x16, 0x202);
	phydev->bus->write(phydev->bus, 0x2, MDIO_DEVAD_NONE, 0x16, 0x202);
	/* Enable 50MHZ Clock */
	phydev->bus->write(phydev->bus, 0x1, MDIO_DEVAD_NONE, 0x1f, 0x8180);
	phydev->bus->write(phydev->bus, 0x2, MDIO_DEVAD_NONE, 0x1f, 0x8180);

	return 0;
}
#endif

int board_early_init_f(void)
{
	setup_iomux_uart();

	imx_iomux_v3_setup_multiple_pads(fpga_jtag_pads,
					 ARRAY_SIZE(fpga_jtag_pads));

	/* Keep as inputs to allow offboard programming */
	gpio_direction_input(JTAG_FPGA_TDI);
	gpio_direction_input(JTAG_FPGA_TCK);
	gpio_direction_input(JTAG_FPGA_TMS);
	gpio_direction_input(JTAG_FPGA_TDO);

	/* Enable LVDS clock output.
	 * Writing CCM_ANALOG_MISC1 to use output from 24M OSC */
	setbits_le32(0x020C8160, 0x412);

	return 0;
}

int board_init(void)
{
	/* Address of boot parameters */
	gd->bd->bi_boot_params = PHYS_SDRAM + 0x100;

	#ifdef CONFIG_FEC_MXC
	setup_fec(CONFIG_FEC_ENET_DEV);
	#endif

	#ifdef CONFIG_FPGA
	ts4100_fpga_init();
	#endif

	return 0;
}

#ifdef CONFIG_CMD_BMODE
static const struct boot_mode board_boot_modes[] = {
	{"sd", MAKE_CFGVAL(0x42, 0x20, 0x00, 0x00)},
	{"emmc1", MAKE_CFGVAL(0x74, 0xa8, 0x00, 0x00)},
	{NULL,	 0},
};
#endif

static void do_bbdetect(void)
{
	int id = 0;
	int i;

	for(i = 0; i < 8; i++) {
		if(i & 1)fpga_gpio_output(RED_LED_PADN, 1);
		else fpga_gpio_output(RED_LED_PADN, 0);

		if(i & 2)fpga_gpio_output(GREEN_LED_PADN, 1);
		else fpga_gpio_output(GREEN_LED_PADN, 0);
		
		if(i & 4)fpga_gpio_output(DIO_20, 1);
		else fpga_gpio_output(DIO_20, 0);

		id = (id >> 1);
		if(fpga_gpio_input(DIO_05)) id |= 0x80;
	}
	printf("Baseboard ID: 0x%X\n", id & ~0xc0);
	printf("Baseboard Rev: %d\n", ((id & 0xc0) >> 6));
	setenv_hex("baseboardid", id & ~0xc0);
	setenv_hex("baseboardrev", ((id & 0xc0) >> 6));
}

int board_late_init(void)
{
#ifdef CONFIG_CMD_BMODE
	add_board_boot_modes(board_boot_modes);
#endif
	set_wdog_reset((struct wdog_regs *)WDOG1_BASE_ADDR);

	fpga_late_init();

	/* Detect the carrier board id to pick the right 
	 * device tree */
	do_bbdetect();

	red_led_on();
	green_led_off();

	return 0;
}

u32 get_board_rev(void)
{
	return get_cpu_rev();
}

int checkboard(void)
{
	int fpgarev;
	/* reset the FGPA */
	gpio_direction_output(FPGA_RESETN, 0);
	gpio_direction_output(EN_FPGA_PWR, 0);
	// off is 70us max
	mdelay(1);
	gpio_direction_output(EN_FPGA_PWR, 1);
	// on is typical ~5ms
	mdelay(10);
	gpio_direction_output(FPGA_RESETN, 1);

	setup_i2c(0, CONFIG_SYS_I2C_SPEED, 0x4a, &i2c_pad_info1);
	setup_i2c(2, CONFIG_SYS_I2C_SPEED, 0x28, &i2c_pad_info3);
	fpgarev = fpga_get_rev();
	puts("Board: Technologic Systems TS-4100\n");
	if(fpgarev < 0)
		printf("FPGA I2C communication failed: %d\n", fpgarev);
	else
		printf("FPGA:  Rev %d\n", fpgarev);
	printf("Silab: Rev %d\n", silab_rev());

	return 0;
}

#ifdef CONFIG_USB_EHCI_MX6
#define USB_OTHERREGS_OFFSET	0x800
#define UCTRL_PWR_POL		(1 << 9)
iomux_v3_cfg_t const usb_otg1_pads[] = {
	MX6_PAD_GPIO1_IO04__USB_OTG1_PWR | MUX_PAD_CTRL(NO_PAD_CTRL),
	MX6_PAD_GPIO1_IO00__ANATOP_OTG1_ID | MUX_PAD_CTRL(OTG_ID_PAD_CTRL),
};

int board_usb_phy_mode(int port)
{
	if (port == 1)
		return USB_INIT_HOST;
	else
		return usb_phy_mode(port);
}

int board_ehci_hcd_init(int port)
{
	u32 *usbnc_usb_ctrl;

	imx_iomux_v3_setup_multiple_pads(usb_otg1_pads,
					 ARRAY_SIZE(usb_otg1_pads));

	usbnc_usb_ctrl = (u32 *)(USB_BASE_ADDR + USB_OTHERREGS_OFFSET +
				 port * 4);

	/* Set Power polarity */
	setbits_le32(usbnc_usb_ctrl, UCTRL_PWR_POL);
	return 0;
}
#endif
