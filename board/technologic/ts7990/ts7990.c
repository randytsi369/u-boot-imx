/*
 * Copyright (C) 2016 Technologic Systems
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#include <asm/arch/clock.h>
#include <asm/arch/imx-regs.h>
#include <asm/arch/iomux.h>
#include <asm/imx-common/mxc_i2c.h>
#include <asm/arch/mx6-pins.h>
#include <asm/errno.h>
#include <asm/gpio.h>
#include <asm/imx-common/iomux-v3.h>
#include <asm/imx-common/boot_mode.h>
#include <asm/imx-common/video.h>
#include <asm/imx-common/sata.h>
#include <linux/ctype.h>
#include <mmc.h>
#include <malloc.h>
#include <spi.h>
#include <net.h>
#include <image.h>
#include <fsl_esdhc.h>
#include <command.h>
#include <miiphy.h>
#include <netdev.h>
#include <asm/arch/mxc_hdmi.h>
#include <asm/arch/crm_regs.h>
#include <i2c.h>
#include <micrel.h>
#include <linux/fb.h>
#include <ipu_pixfmt.h>
#include <asm/io.h>
#include <asm/arch/sys_proto.h>
#include <fpga.h>
#include <lattice.h>

#include "tsfpga.h"
#include "silabs.h"

#define TS7990_HUB_RESETN	IMX_GPIO_NR(2, 11)
#define TS7990_ENUSB_5V		IMX_GPIO_NR(2, 22)
#define TS7990_SDBOOT		IMX_GPIO_NR(2, 26)
#define TS7990_JPUBOOT		IMX_GPIO_NR(2, 25)
#define TS7990_SPI_CS		IMX_GPIO_NR(3, 19)
#define TS7990_PHY_RST		IMX_GPIO_NR(4, 20)
#define TS7990_RGMII_RXC	IMX_GPIO_NR(6, 30)
#define TS7990_RGMII_RD0	IMX_GPIO_NR(6, 25)
#define TS7990_RGMII_RD1	IMX_GPIO_NR(6, 27)
#define TS7990_RGMII_RD2	IMX_GPIO_NR(6, 28)
#define TS7990_RGMII_RD3	IMX_GPIO_NR(6, 29)
#define TS7990_RGMII_RX_CTL	IMX_GPIO_NR(6, 24)
#define TS7990_EN_RTC		IMX_GPIO_NR(3, 23)
#define TS7990_SCL			IMX_GPIO_NR(3, 21)
#define TS7990_SDA			IMX_GPIO_NR(3, 28)
#define TS7990_BKL			IMX_GPIO_NR(2, 9)
#define TS7990_BKL_EN		IMX_GPIO_NR(3, 0)
#define TS7990_FPGA_RESET	IMX_GPIO_NR(2, 28)
#define TS7990_REVB			IMX_GPIO_NR(3, 2)
#define TS7990_SATASEL		IMX_GPIO_NR(7, 8)
#define TS7990_NOCHRG		IMX_GPIO_NR(5, 28)
#define TS7990_FPGA_SPI_CS	IMX_GPIO_NR(5, 31)

DECLARE_GLOBAL_DATA_PTR;
int random_mac = 0;

#define UART_PAD_CTRL (PAD_CTL_PUS_100K_UP |			\
	PAD_CTL_SPEED_MED | PAD_CTL_DSE_40ohm |			\
	PAD_CTL_SRE_FAST  | PAD_CTL_HYS)

#define USDHC_PAD_CTRL (PAD_CTL_PUS_47K_UP |			\
	PAD_CTL_SPEED_LOW | PAD_CTL_DSE_80ohm |			\
	PAD_CTL_SRE_FAST  | PAD_CTL_HYS)

#define ENET_PAD_CTRL (PAD_CTL_SPEED_MED | PAD_CTL_DSE_80ohm)

#define LCD_PAD_CTRL (PAD_CTL_PUS_100K_UP |			\
	PAD_CTL_SPEED_MED | PAD_CTL_DSE_40ohm | PAD_CTL_HYS)

#define SPI_PAD_CTRL (PAD_CTL_HYS | PAD_CTL_SPEED_MED |		\
	PAD_CTL_DSE_40ohm     | PAD_CTL_SRE_FAST)

#define I2C_PAD_CTRL (PAD_CTL_PUS_100K_UP |                  \
	PAD_CTL_SPEED_MED | PAD_CTL_DSE_40ohm | PAD_CTL_HYS |   \
	PAD_CTL_ODE | PAD_CTL_SRE_FAST)

iomux_v3_cfg_t const ecspi1_pads[] = {
	MX6_PAD_EIM_D19__GPIO3_IO19 | MUX_PAD_CTRL(SPI_PAD_CTRL),
	MX6_PAD_EIM_D17__ECSPI1_MISO | MUX_PAD_CTRL(SPI_PAD_CTRL),
	MX6_PAD_EIM_D18__ECSPI1_MOSI | MUX_PAD_CTRL(SPI_PAD_CTRL),
	MX6_PAD_EIM_D16__ECSPI1_SCLK | MUX_PAD_CTRL(SPI_PAD_CTRL),
};

iomux_v3_cfg_t const misc_pads[] = {
	MX6_PAD_CSI0_DAT13__GPIO5_IO31 | MUX_PAD_CTRL(NO_PAD_CTRL), // FPGA_SPI_CS0#
	MX6_PAD_SD4_DAT3__GPIO2_IO11 | MUX_PAD_CTRL(NO_PAD_CTRL),   // USB_HUB_RESET#
	MX6_PAD_EIM_A16__GPIO2_IO22 | MUX_PAD_CTRL(NO_PAD_CTRL),    // EN_USB_5V
	MX6_PAD_EIM_RW__GPIO2_IO26 | MUX_PAD_CTRL(NO_PAD_CTRL),     // JP_SD_BOOT#
	MX6_PAD_EIM_OE__GPIO2_IO25 | MUX_PAD_CTRL(NO_PAD_CTRL),     // JP_OPTION
	MX6_PAD_EIM_D23__GPIO3_IO23 | MUX_PAD_CTRL(NO_PAD_CTRL),    // EN_RTC_PWR#
	MX6_PAD_GPIO_3__XTALOSC_REF_CLK_24M | MUX_PAD_CTRL(NO_PAD_CTRL),  // FPGA CLK
	MX6_PAD_EIM_EB0__GPIO2_IO28 | MUX_PAD_CTRL(NO_PAD_CTRL),    // FPGA_RESET
	MX6_PAD_SD4_DAT1__GPIO2_IO09 | MUX_PAD_CTRL(LCD_PAD_CTRL),  // PWM_LOCAL_LCD
	MX6_PAD_EIM_DA0__GPIO3_IO00 | MUX_PAD_CTRL(LCD_PAD_CTRL),   // EN_BKLT
	MX6_PAD_EIM_DA9__GPIO3_IO09 | MUX_PAD_CTRL(NO_PAD_CTRL),    // PUSH_SW_1 (Home)
	MX6_PAD_EIM_DA10__GPIO3_IO10 | MUX_PAD_CTRL(NO_PAD_CTRL),   // PUSH_SW_2 (Back)
	MX6_PAD_EIM_DA2__GPIO3_IO02 | MUX_PAD_CTRL(NO_PAD_CTRL),    // TS7990_REVB strap
	MX6_PAD_SD3_RST__GPIO7_IO08 | MUX_PAD_CTRL(NO_PAD_CTRL),
	MX6_PAD_CSI0_DAT10__GPIO5_IO28 | MUX_PAD_CTRL(NO_PAD_CTRL), // NO_CHRG jumper
};

/* SD card */
iomux_v3_cfg_t const usdhc2_pads[] = {
	MX6_PAD_SD2_CLK__SD2_CLK	| MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD2_CMD__SD2_CMD	| MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD2_DAT0__SD2_DATA0	| MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD2_DAT1__SD2_DATA1	| MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD2_DAT2__SD2_DATA2	| MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD2_DAT3__SD2_DATA3	| MUX_PAD_CTRL(USDHC_PAD_CTRL),
};

/* MMC */
iomux_v3_cfg_t const usdhc3_pads[] = {
	MX6_PAD_SD3_CLK__SD3_CLK   | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD3_CMD__SD3_CMD   | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD3_DAT0__SD3_DATA0 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD3_DAT1__SD3_DATA1 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD3_DAT2__SD3_DATA2 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD3_DAT3__SD3_DATA3 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
};

iomux_v3_cfg_t const uart1_pads[] = {
	MX6_PAD_SD3_DAT7__UART1_TX_DATA | MUX_PAD_CTRL(UART_PAD_CTRL),
	MX6_PAD_SD3_DAT6__UART1_RX_DATA | MUX_PAD_CTRL(UART_PAD_CTRL),
};

iomux_v3_cfg_t const enet_pads1[] = {
	MX6_PAD_ENET_MDIO__ENET_MDIO		| MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_ENET_MDC__ENET_MDC		| MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_RGMII_TXC__RGMII_TXC	| MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_RGMII_TD0__RGMII_TD0	| MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_RGMII_TD1__RGMII_TD1	| MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_RGMII_TD2__RGMII_TD2	| MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_RGMII_TD3__RGMII_TD3	| MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_RGMII_TX_CTL__RGMII_TX_CTL	| MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_ENET_REF_CLK__ENET_TX_CLK	| MUX_PAD_CTRL(ENET_PAD_CTRL),
	/* pin 35 - 1 (PHY_AD2) on reset */
	MX6_PAD_RGMII_RXC__GPIO6_IO30		| MUX_PAD_CTRL(NO_PAD_CTRL),
	/* pin 32 - 1 - (MODE0) all */
	MX6_PAD_RGMII_RD0__GPIO6_IO25		| MUX_PAD_CTRL(NO_PAD_CTRL),
	/* pin 31 - 1 - (MODE1) all */
	MX6_PAD_RGMII_RD1__GPIO6_IO27		| MUX_PAD_CTRL(NO_PAD_CTRL),
	/* pin 28 - 1 - (MODE2) all */
	MX6_PAD_RGMII_RD2__GPIO6_IO28		| MUX_PAD_CTRL(NO_PAD_CTRL),
	/* pin 27 - 1 - (MODE3) all */
	MX6_PAD_RGMII_RD3__GPIO6_IO29		| MUX_PAD_CTRL(NO_PAD_CTRL),
	/* pin 33 - 1 - (CLK125_EN) 125Mhz clockout enabled */
	MX6_PAD_RGMII_RX_CTL__GPIO6_IO24	| MUX_PAD_CTRL(NO_PAD_CTRL),
	// PHY RESET
	MX6_PAD_DI0_PIN4__GPIO4_IO20		| MUX_PAD_CTRL(NO_PAD_CTRL),
};

iomux_v3_cfg_t const enet_pads2[] = {
	MX6_PAD_RGMII_RXC__RGMII_RXC		| MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_RGMII_RD0__RGMII_RD0		| MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_RGMII_RD1__RGMII_RD1		| MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_RGMII_RD2__RGMII_RD2		| MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_RGMII_RD3__RGMII_RD3		| MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_RGMII_RX_CTL__RGMII_RX_CTL	| MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_ENET_MDIO__ENET_MDIO		| MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_ENET_MDC__ENET_MDC		| MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_RGMII_TXC__RGMII_TXC		| MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_RGMII_TD0__RGMII_TD0		| MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_RGMII_TD1__RGMII_TD1		| MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_RGMII_TD2__RGMII_TD2		| MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_RGMII_TD3__RGMII_TD3		| MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_RGMII_TX_CTL__RGMII_TX_CTL	| MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_ENET_REF_CLK__ENET_TX_CLK	| MUX_PAD_CTRL(ENET_PAD_CTRL),
	// PHY RESET
	MX6_PAD_DI0_PIN4__GPIO4_IO20		| MUX_PAD_CTRL(NO_PAD_CTRL),
};

iomux_v3_cfg_t const lcd_pads[] = {
	MX6_PAD_DI0_DISP_CLK__IPU1_DI0_DISP_CLK | MUX_PAD_CTRL(LCD_PAD_CTRL),
	// DE
	MX6_PAD_DI0_PIN15__IPU1_DI0_PIN15 | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX6_PAD_DISP0_DAT2__IPU1_DISP0_DATA02	| MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX6_PAD_DISP0_DAT3__IPU1_DISP0_DATA03	| MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX6_PAD_DISP0_DAT4__IPU1_DISP0_DATA04	| MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX6_PAD_DISP0_DAT5__IPU1_DISP0_DATA05	| MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX6_PAD_DISP0_DAT6__IPU1_DISP0_DATA06	| MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX6_PAD_DISP0_DAT7__IPU1_DISP0_DATA07	| MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX6_PAD_DISP0_DAT10__IPU1_DISP0_DATA10	| MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX6_PAD_DISP0_DAT11__IPU1_DISP0_DATA11	| MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX6_PAD_DISP0_DAT12__IPU1_DISP0_DATA12	| MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX6_PAD_DISP0_DAT13__IPU1_DISP0_DATA13	| MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX6_PAD_DISP0_DAT14__IPU1_DISP0_DATA14	| MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX6_PAD_DISP0_DAT15__IPU1_DISP0_DATA15	| MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX6_PAD_DISP0_DAT18__IPU1_DISP0_DATA18	| MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX6_PAD_DISP0_DAT19__IPU1_DISP0_DATA19	| MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX6_PAD_DISP0_DAT20__IPU1_DISP0_DATA20	| MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX6_PAD_DISP0_DAT21__IPU1_DISP0_DATA21	| MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX6_PAD_DISP0_DAT22__IPU1_DISP0_DATA22	| MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX6_PAD_DISP0_DAT23__IPU1_DISP0_DATA23	| MUX_PAD_CTRL(LCD_PAD_CTRL),
};

struct i2c_pads_info i2c_pad_info0 = {
	.scl = {
		.i2c_mode  = MX6_PAD_EIM_D21__I2C1_SCL | MUX_PAD_CTRL(I2C_PAD_CTRL),
		.gpio_mode = MX6_PAD_EIM_D21__GPIO3_IO21 | MUX_PAD_CTRL(I2C_PAD_CTRL),
		.gp = TS7990_SCL
	},
	.sda = {
		.i2c_mode = MX6_PAD_EIM_D28__I2C1_SDA | MUX_PAD_CTRL(I2C_PAD_CTRL),
		.gpio_mode = MX6_PAD_EIM_D28__GPIO3_IO28 | MUX_PAD_CTRL(I2C_PAD_CTRL),
		.gp = TS7990_SDA
	}
};

/* We need to know if it is rev A before we set up i2c.  This is done 
* differently on rev a/b, but is the same on >= 'B' */
bool board_is_reva()
{
	gpio_direction_input(TS7990_REVB);
	/* Low means it is REV B */
	if(gpio_get_value(TS7990_REVB))
		return 1;
	return 0;
}

char board_rev(void)
{
	static int rev = -1;

	if(rev == -1) {
		if(board_is_reva()){
			rev = 'A';
		} else {
			uint8_t val;
			uint8_t fpgarev;
			i2c_read(0x28, FPGA_REV_OPS, 2, &val, 1);
			fpgarev = val >> 4;
			if((val & FPGA_OPS_G12) &&
				fpgarev >= 10) {
				rev = 'C';
			} else {
				rev = 'B';
			}
		}
	}

	return (char)rev;
}

#if defined(CONFIG_FPGA)

iomux_v3_cfg_t const fpga_jtag_pads[] = {
	MX6_PAD_GPIO_5__GPIO1_IO05 | MUX_PAD_CTRL(NO_PAD_CTRL), 	// JTAG_FPGA_TMS
	MX6_PAD_GPIO_16__GPIO7_IO11 | MUX_PAD_CTRL(NO_PAD_CTRL), 	// JTAG_FPGA_TCK
	MX6_PAD_GPIO_17__GPIO7_IO12 | MUX_PAD_CTRL(NO_PAD_CTRL), 	// JTAG_FPGA_TDI
	MX6_PAD_CSI0_MCLK__GPIO5_IO19 | MUX_PAD_CTRL(NO_PAD_CTRL), 	// JTAG_FPGA_TDO
};

static void ts7990_jtag_init(void)
{
	imx_iomux_v3_setup_multiple_pads(fpga_jtag_pads,
					 ARRAY_SIZE(fpga_jtag_pads));
	gpio_direction_output(CONFIG_FPGA_TDI, 1);
	gpio_direction_output(CONFIG_FPGA_TCK, 1);
	gpio_direction_output(CONFIG_FPGA_TMS, 1);
	gpio_direction_input(CONFIG_FPGA_TDO);

	return;
}

static void ts7990_fpga_jtag_set_tdi(int value)
{
	gpio_set_value(CONFIG_FPGA_TDI, value);
}

static void ts7990_fpga_jtag_set_tms(int value)
{
	gpio_set_value(CONFIG_FPGA_TMS, value);
}

static void ts7990_fpga_jtag_set_tck(int value)
{
	gpio_set_value(CONFIG_FPGA_TCK, value);
}

static int ts7990_fpga_jtag_get_tdo(void)
{
	return gpio_get_value(CONFIG_FPGA_TDO);
}

lattice_board_specific_func ts7990_fpga_fns = {
	ts7990_jtag_init,
	ts7990_fpga_jtag_set_tdi,
	ts7990_fpga_jtag_set_tms,
	ts7990_fpga_jtag_set_tck,
	ts7990_fpga_jtag_get_tdo
};

Lattice_desc ts7990_fpga = {
	Lattice_XP2,
	lattice_jtag_mode,
	540212,
	(void *) &ts7990_fpga_fns,
	NULL,
	0,
	"machxo_2_cb132"
};

int ts7990_fpga_init(void)
{
	fpga_init();
	fpga_add(fpga_lattice, &ts7990_fpga);

	return 0;
}

#endif

static void setup_display(void)
{
	struct mxc_ccm_reg *mxc_ccm = (struct mxc_ccm_reg *)CCM_BASE_ADDR;
	struct iomuxc *iomux = (struct iomuxc *)IOMUXC_BASE_ADDR;
	int reg;

	enable_ipu_clock();

	/* Turn on LDB0,IPU,IPU DI0 clocks */
	reg = __raw_readl(&mxc_ccm->CCGR3);
	reg |=  MXC_CCM_CCGR3_LDB_DI0_MASK;
	writel(reg, &mxc_ccm->CCGR3);

	/* set LDB0, LDB1 clk select to 011/011 */
	reg = readl(&mxc_ccm->cs2cdr);
	reg &= ~(MXC_CCM_CS2CDR_LDB_DI0_CLK_SEL_MASK
		 |MXC_CCM_CS2CDR_LDB_DI1_CLK_SEL_MASK);
	reg |= (3<<MXC_CCM_CS2CDR_LDB_DI0_CLK_SEL_OFFSET);
	writel(reg, &mxc_ccm->cs2cdr);

	reg = readl(&mxc_ccm->cscmr2);
	reg |= MXC_CCM_CSCMR2_LDB_DI0_IPU_DIV;
	writel(reg, &mxc_ccm->cscmr2);

	reg = readl(&mxc_ccm->chsccdr);
	reg |= (CHSCCDR_CLK_SEL_LDB_DI0
		<<MXC_CCM_CHSCCDR_IPU1_DI0_CLK_SEL_OFFSET);
	writel(reg, &mxc_ccm->chsccdr);

	reg = IOMUXC_GPR2_BGREF_RRMODE_EXTERNAL_RES
	     |IOMUXC_GPR2_DI0_VS_POLARITY_ACTIVE_LOW
	     |IOMUXC_GPR2_BIT_MAPPING_CH1_SPWG
	     |IOMUXC_GPR2_DATA_WIDTH_CH1_18BIT
	     |IOMUXC_GPR2_BIT_MAPPING_CH0_SPWG
	     |IOMUXC_GPR2_DATA_WIDTH_CH0_18BIT
	     |IOMUXC_GPR2_LVDS_CH1_MODE_DISABLED
	     |IOMUXC_GPR2_LVDS_CH0_MODE_ENABLED_DI0;
	writel(reg, &iomux->gpr[2]);

	reg = readl(&iomux->gpr[3]);
	reg = (reg & ~(IOMUXC_GPR3_LVDS0_MUX_CTL_MASK
			|IOMUXC_GPR3_HDMI_MUX_CTL_MASK))
	    | (IOMUXC_GPR3_MUX_SRC_IPU1_DI0
	       <<IOMUXC_GPR3_LVDS0_MUX_CTL_OFFSET);
	writel(reg, &iomux->gpr[3]);
}

#define LCD_LXD 0
#define LCD_OKAYA 1
#define LCD_MICROTIPS 2

static int detect_lcd(void)
{
	static int lcd = -1;
	if(lcd == -1) {
		/* Rev B and on added current sense circuits
		 * rather than individual strap resistors for
		 * differeing LCDs */
		if(board_rev() == 'A') {
			uint8_t val = 0;

			i2c_read(0x28, FPGA_REV_OPS, 2, &val, 1);
			if(val & FPGA_OPS_P13) {
				i2c_read(0x28, FPGA_OPS2, 2, &val, 1);
				if(val & FPGA_OPS_OKAYA) {
					lcd = LCD_OKAYA; // Okaya
				} else {
					lcd = LCD_MICROTIPS; // Microtips
				}
			} else { // LXD
				lcd = LCD_LXD;
			}
		} else {
			uint8_t val = 0;
			i2c_read(0x28, FPGA_OPS2, 2, &val, 1);
			if(val & FPGA_OPS_OKAYA) {
				lcd = LCD_OKAYA;
			} else if(val & FPGA_OPS_LXD) {
				lcd = LCD_LXD;
			} else {
				lcd = LCD_MICROTIPS;
			}
		}
	}
	return lcd;
}

static int is_lxd(struct display_info_t const *dev)
{
	if(detect_lcd() == LCD_LXD)
		return 1;
	return 0;
}
static int is_okaya(struct display_info_t const *dev)
{
	if(detect_lcd() == LCD_OKAYA)
		return 1;
	return 0;
}

static int is_microtips(struct display_info_t const *dev)
{
	if(detect_lcd() == LCD_MICROTIPS)
		return 1;
	return 0;
}

static void setup_lxd(struct display_info_t const *dev)
{
	uint8_t val;
	u32 reg;
	struct iomuxc *iomux = (struct iomuxc *)
				IOMUXC_BASE_ADDR;

	// EN_LCD_POWER (VDD)
	val = 0x14;
	i2c_write(0x28, 59, 2, &val, 1);

	udelay(65000);
	// 50ms after (VDD) power stable:
	// deassert LCD_RESET#
	val = 0x1c;
	i2c_write(0x28, 59, 2, &val, 1);
	// Enable LCD_11V (AVDD), and wait until stable
	val = 0x18;
	i2c_write(0x28, 59, 2, &val, 1);
	udelay(10000);
	// Enable LCD_NEG_7V and LCD_20V (VGL + VGH) (no delay)
	val = 0x1B;
	i2c_write(0x28, 59, 2, &val, 1);
	// Wait 70ms more before enabling the backlight
	// TODO, trim this down.  It might already be 70ms before data/backlight happen
	udelay(20000);
	udelay(50000);

	val = 0x14;
	i2c_write(0x28, 59, 2, &val, 1);
	udelay(50000);
	val = 0x1b;
	i2c_write(0x28, 59, 2, &val, 1);

	setup_display();

	reg = readl(&iomux->gpr[2]);
	reg |= IOMUXC_GPR2_DATA_WIDTH_CH0_24BIT;
	writel(reg, &iomux->gpr[2]);
}

static void setup_microtips(struct display_info_t const *dev)
{
	uint8_t val;
	// EN_LCD_POWER
	val = 0x14;
	i2c_write(0x28, 59, 2, &val, 1);
	udelay(50000);
	// Deassert LCD_RESET#
	val = 0x1c;
	i2c_write(0x28, 59, 2, &val, 1);
	udelay(20000);
	// enable LCD_11V
	val = 0x18;
	i2c_write(0x28, 59, 2, &val, 1);
	udelay(5000);
	// enable LCD_NEG_7V
	val = 0x1a;
	i2c_write(0x28, 59, 2, &val, 1);
	udelay(5000);
	// enable LCD_20V
	val = 0x1b;
	i2c_write(0x28, 59, 2, &val, 1);
	// Wait 20ms more before driving lcd pins
	udelay(20000);

	imx_iomux_v3_setup_multiple_pads(lcd_pads, ARRAY_SIZE(lcd_pads));
	// Drive MT_LCD_PRESENT to limit the backlight current for 400 nits
	val = 0x2;
	i2c_write(0x28, 60, 2, &val, 1);
	setup_display();
}

static void setup_okaya(struct display_info_t const *dev)
{
	uint8_t val;

	// EN_LCD_POWER
	val = 0x14;
	i2c_write(0x28, 59, 2, &val, 1);
	udelay(50000);

	// Drive MT_LCD_PRESENT low for the 800 nit display
	val = 0x0;
	i2c_write(0x28, 60, 2, &val, 1);

	imx_iomux_v3_setup_multiple_pads(lcd_pads, ARRAY_SIZE(lcd_pads));
	setup_display();
}

struct display_info_t const displays[] = { {
	.pixfmt	= IPU_PIX_FMT_LVDS666,
	.detect	= is_lxd,
	.enable	= setup_lxd,
	.mode	= {
		.name           = "LXD-WSVGA",
		.refresh        = 60,
		.xres           = 1024,
		.yres           = 600,
		.pixclock       = 19530,
		.left_margin    = 46,
		.right_margin   = 210,
		.upper_margin   = 23,
		.lower_margin   = 12,
		.hsync_len      = 20,
		.vsync_len      = 10,
		.sync           = FB_SYNC_EXT,
		.vmode          = FB_VMODE_NONINTERLACED
} }, {
	.pixfmt	= IPU_PIX_FMT_RGB24,
	.detect	= is_okaya,
	.enable	= setup_okaya,
	.mode	= {
		.name           = "OKAYA-WVGA",
		.refresh        = 60,
		.xres           = 800,
		.yres           = 480,
		.pixclock       = 33330,
		.left_margin    = 40,
		.right_margin   = 40,
		.upper_margin   = 29,
		.lower_margin   = 13,
		.hsync_len      = 48,
		.vsync_len      = 3,
		.sync           = FB_SYNC_EXT,
		.vmode          = FB_VMODE_NONINTERLACED
} }, {
	.pixfmt	= IPU_PIX_FMT_RGB24,
	.detect	= is_microtips,
	.enable	= setup_microtips,
	.mode	= {
		.name           = "Microtips-WVGA",
		.refresh        = 60,
		.xres           = 800,
		.yres           = 480,
		.pixclock       = 30030,
		.left_margin    = 46,
		.right_margin   = 210,
		.upper_margin   = 23,
		.lower_margin   = 22,
		.hsync_len      = 1,
		.vsync_len      = 1,
		.sync           = FB_SYNC_EXT,
		.vmode          = FB_VMODE_NONINTERLACED
} } };
size_t display_count = ARRAY_SIZE(displays);

int board_spi_cs_gpio(unsigned bus, unsigned cs)
{
	return (bus == 0 && cs == 0) ? (TS7990_SPI_CS) : -1;
}

void setup_spi(void)
{
	imx_iomux_v3_setup_multiple_pads(ecspi1_pads,
					 ARRAY_SIZE(ecspi1_pads));

	// Enable clock
	setbits_le32(CCM_CCGR1, MXC_CCM_CCGR1_ECSPI2S_MASK);
}

int dram_init(void)
{	
	gd->ram_size = (phys_size_t)CONFIG_DDR_MB * 1024 * 1024;
	return 0;
}

static void setup_iomux_enet(void)
{
	// Assert reset
	gpio_direction_output(TS7990_PHY_RST, 1);
	if(board_rev() == 'A') {
		imx_iomux_v3_setup_multiple_pads(enet_pads1, ARRAY_SIZE(enet_pads1));

		gpio_direction_output(TS7990_RGMII_RXC, 1);
		gpio_direction_output(TS7990_RGMII_RD0, 1);
		gpio_direction_output(TS7990_RGMII_RD1, 1);
		gpio_direction_output(TS7990_RGMII_RD2, 1);
		gpio_direction_output(TS7990_RGMII_RD3, 1);
		gpio_direction_output(TS7990_RGMII_RX_CTL, 1);
	}

	/* Need delay at least 10ms according to KSZ9031 spec, and
	 * per the 88E1512 on REV B and higher*/
	mdelay(10);

	// De-assert reset
	gpio_direction_output(TS7990_PHY_RST, 0);

	imx_iomux_v3_setup_multiple_pads(enet_pads2, ARRAY_SIZE(enet_pads2));
	// Need up to 1ms for the phy to come out of reset
	mdelay(1);
}

struct fsl_esdhc_cfg usdhc_cfg[2] = {
	{USDHC2_BASE_ADDR},
	{USDHC3_BASE_ADDR},
};

int board_mmc_getcd(struct mmc *mmc)
{
	return 1;
}

int board_mmc_init(bd_t *bis)
{
	s32 status = 0;
	int i;

	imx_iomux_v3_setup_multiple_pads(usdhc2_pads, 
		ARRAY_SIZE(usdhc2_pads));
	imx_iomux_v3_setup_multiple_pads(usdhc3_pads, 
		ARRAY_SIZE(usdhc3_pads));

	/*
	 * According to the board_mmc_init() the following map is done:
	 * (U-boot device node)    (Physical Port)
	 * mmc0                    SD
	 * mmc1                    eMMC
	 */
	for (i = 0; i < CONFIG_SYS_FSL_USDHC_NUM; i++) {
		switch (i) {
		case 0: // SD
			usdhc_cfg[0].max_bus_width = 4;
			usdhc_cfg[0].sdhc_clk = mxc_get_clock(MXC_ESDHC2_CLK);

			break;
		case 1: // MMC
			usdhc_cfg[1].max_bus_width = 4;
			usdhc_cfg[1].sdhc_clk = mxc_get_clock(MXC_ESDHC3_CLK);
			break;
		default:
			printf("Warning: you configured more USDHC controllers"
			       "(%d) then supported by the board (%d)\n",
			       i + 1, CONFIG_SYS_FSL_USDHC_NUM);
			return status;
		}

		status |= fsl_esdhc_initialize(bis, &usdhc_cfg[i]);
	}

	return status;
}

int board_phy_config(struct phy_device *phydev)
{
	/* Microchip KSZ9031RNX */
	if(board_rev() == 'A') {
		ksz9031_phy_extended_write(phydev, 0x2, 0x8, 0x8000, 0x3EF);
		ksz9031_phy_extended_write(phydev, 0x0, 0x3, 0x8000, 0x1A80);
		ksz9031_phy_extended_write(phydev, 0x0, 0x4, 0x8000, 0x0006);
	} else { /* Marvell 88E1512 */
		/* reg page 2 */
		phy_write(phydev, MDIO_DEVAD_NONE, 22, 0x0002);
		/* Delay RGMII TX and RX */
		phy_write(phydev, MDIO_DEVAD_NONE, 0x15, 0x1070);
		/* reg page 0 */
		phy_write(phydev, MDIO_DEVAD_NONE, 22, 0x0000);
		/* Enable downshift after 1 try */
		phy_write(phydev, MDIO_DEVAD_NONE, 16, 0x1860);
		/* reg page 3 */
		phy_write(phydev, MDIO_DEVAD_NONE, 22, 0x0003);
		/* Change LED */
		phy_write(phydev, MDIO_DEVAD_NONE, 16, 0x1017);
		/* reset to reg page 0 */
		phy_write(phydev, MDIO_DEVAD_NONE, 22, 0x0000);
	}

	if (phydev->drv->config)
		phydev->drv->config(phydev); 

	return 0;
}

/*
 * Do not overwrite the console
 * Use always serial for U-Boot console
 */
int overwrite_console(void)
{
	return 1;
}

int board_eth_init(bd_t *bis)
{
	uint32_t base = IMX_FEC_BASE;
	struct mii_dev *bus = NULL;
	struct phy_device *phydev = NULL;
	uchar enetaddr[6];
	char enet1addr[18];
	int ret;

	setup_iomux_enet();
	bus = fec_get_miibus(base, -1);
	if (!bus)
		return 0;

	if(board_rev() == 'A')
		phydev = phy_find_by_mask(bus, (0xf << 4), PHY_INTERFACE_MODE_RGMII);
	else
		phydev = phy_find_by_mask(bus, 0xf, PHY_INTERFACE_MODE_RGMII);	

	if (!phydev) {
		free(bus);
		return 0;
	}
	printf("using phy at %d\n", phydev->addr);
	ret  = fec_probe(bis, -1, base, bus, phydev);
	if (ret) {
		printf("FEC MXC: %s:failed\n", __func__);
		free(phydev);
		free(bus);
	}

#ifdef CONFIG_RANDOM_MACADDR
	if (!eth_getenv_enetaddr("ethaddr", enetaddr)) {
		printf("No MAC address set in fuses.  Using random mac address.\n");
		eth_random_addr(enetaddr);
		random_mac = 1;
		if (eth_setenv_enetaddr("ethaddr", enetaddr)) {
			printf("Failed to set ethernet address\n");
		}
	} else {
		/* Each board is allocated two sequential mac addresses.
		 * This is used for USB ethernets or the I210 on a carrier board */
		if (enetaddr[5] == 0xff) {
			if (enetaddr[4] == 0xff) {
				enetaddr[3]++;
			}
			enetaddr[4]++;
		}
		enetaddr[5]++;

		snprintf(enet1addr, 18, "00:d0:69:%02x:%02x:%02x",
				 enetaddr[3],
				 enetaddr[4],
				 enetaddr[5]);
		setenv("eth1addr", enet1addr);
		setenv("usbethaddr", enet1addr);
	}
#endif
	
	return ret;
}

#ifdef CONFIG_LDO_BYPASS_CHECK
/* no external pmic, always ldo_enable */
void ldo_mode_set(int ldo_bypass)
{
	return;
}
#endif

int board_early_init_f(void)
{
	imx_iomux_v3_setup_multiple_pads(misc_pads, ARRAY_SIZE(misc_pads));
	imx_iomux_v3_setup_multiple_pads(uart1_pads, ARRAY_SIZE(uart1_pads));

	gpio_direction_output(TS7990_BKL, 0);
	gpio_direction_output(TS7990_BKL_EN, 0);
	gpio_direction_output(TS7990_FPGA_RESET, 1);
	gpio_direction_output(TS7990_FPGA_RESET, 0);

	return 0;
}

int misc_init_r(void)
{
	int sdboot, jpuboot;
	char rev[2] = {0, 0};
	uint8_t val[33];
	struct iomuxc *iomuxc_regs = (struct iomuxc *)IOMUXC_BASE_ADDR;

	// Turn off USB hub until hub is reset
	// Set DC_SEL_USB to use usb on the standard header
	gpio_direction_input(TS7990_SDBOOT);
	gpio_direction_output(TS7990_ENUSB_5V, 0);
	gpio_direction_output(TS7990_HUB_RESETN, 0); // hub only needs 1us
	udelay(1);
	gpio_set_value(TS7990_HUB_RESETN, 1);

	i2c_read(0x28, FPGA_ENS, 2, val, 1);
	val[0] |= FPGA_ENS_TOUCHRST;
	i2c_write(0x28, FPGA_ENS, 2, val, 1);

	// Magic number for usb power on reset
	// bad things happen with most devices if the hub
	// gets a reset and power doesn't.  This *might* not be
	// enough for some devices
	udelay(10000); 

	gpio_set_value(TS7990_ENUSB_5V, 1);
	sdboot = gpio_get_value(TS7990_SDBOOT);

	rev[0] = tolower(board_rev());
	setenv("pcbrev", rev);

	/* Take touch out of reset */
	val[0] &= ~(FPGA_ENS_TOUCHRST);
	i2c_write(0x28, FPGA_ENS, 2, val, 1);

	if(sdboot) setenv("jpsdboot", "off");
	else setenv("jpsdboot", "on");

	jpuboot = gpio_get_value(TS7990_JPUBOOT);
	if(jpuboot) setenv("jpuboot", "off");
	else setenv("jpuboot", "on");

	setenv("imx_type", CONFIG_IMX_TYPE);

	#ifdef CONFIG_MX6Q
	setenv("cpu", "q");
	#else
	setenv("cpu", "dl");
	#endif

	setenv("model", "7990");
	setenv_hex("reset_cause", get_imx_reset_cause());

	if(board_rev() != 'A') {
		if(tssilo_is_detected()) {
			setenv("silopresent", "1");
		} else {
			setenv("silopresent", "0");
		}
	} else {
		setenv("silopresent", "0");
	}

	/* Drive chip select high so FPGA does not interfere
	 * with FPGA_SPI_MISO, which is the no chrg jumper */
	gpio_direction_output(TS7990_FPGA_SPI_CS, 1);

	gpio_direction_input(TS7990_NOCHRG);
	if (gpio_get_value(TS7990_NOCHRG) == 0) {
		setenv("jpnochrg", "1");
	} else {
		setenv("jpnochrg", "0");
	}

	/* PCIE does not get properly disabled from a watchdog reset.  This prevents 
	 * a hang in the kernel if pcie was enabled in a previous boot. */
	setbits_le32(&iomuxc_regs->gpr[1], IOMUXC_GPR1_TEST_POWERDOWN);
	clrbits_le32(&iomuxc_regs->gpr[1], IOMUXC_GPR1_REF_SSP_EN);

	printf("PCB:   %c\n", board_rev());
	i2c_read(0x28, FPGA_REV_OPS, 2, val, 1);
	printf("FPGA:  %d\n", val[0] >> 4);
	i2c_read(0x4a, 0, 0, val, 32);
	printf("Silab: %d\n", val[31]);

	if(is_lxd(NULL)) setenv("lcd", "lxd");
	else if (is_okaya(NULL)) setenv("lcd", "okaya");
	else if (is_microtips(NULL)) setenv("lcd", "microtips");
	return 0;
}

#ifdef CONFIG_CMD_BMODE
static const struct boot_mode board_boot_modes[] = {
	{"emmc", MAKE_CFGVAL(0x60, 0x30, 0x00, 0x00)},
	{NULL,   0},
};
#endif

void bmp_display_post(void)
{
	uint8_t val;

	gpio_direction_output(TS7990_BKL, 1);
	gpio_direction_output(TS7990_BKL_EN, 1);
	/* Enable backlight late in boot */
	// (REV A only) backlight enable
	val = 0x1;
	i2c_write(0x28, 58, 2, &val, 1);
}

int board_init(void)
{
	int i;

	if(board_is_reva()) {
		imx_iomux_v3_setup_pad(i2c_pad_info0.sda.gpio_mode);
		imx_iomux_v3_setup_pad(i2c_pad_info0.scl.gpio_mode);
		/* The Intersil RTC does not behave correctly on every boot.  When it
		 * fails it locks up the I2C bus by driving it to ~1.5V.  It should only
		 * be open drain, but the theory is that memory is corrupted on startup
		 * on the RTC itself causing it to have a seemingly random behavior. If
		 * we catch the force_idle_bus failing, then then off the fet, drive the pins 
		 * low, wait, turn it back on, and this seems to fix the RTC issues. */
		gpio_direction_output(TS7990_EN_RTC, 0);
		udelay(1000*2); // 2ms to turn on

		/* 5 is an arbitrary magic number.  2 should be enough, but 5 is 
		 * including overkill and doesn't take very long if it were to fail up to 5 */
		for (i = 0; i < 5; i++)
		{
			if (gpio_get_value(TS7990_SCL) == 1 &&
				gpio_get_value(TS7990_SDA) == 1)
				break;

			// Drive I2C pins low
			imx_iomux_v3_setup_pad(i2c_pad_info0.sda.gpio_mode);
			imx_iomux_v3_setup_pad(i2c_pad_info0.scl.gpio_mode);
			gpio_direction_output(i2c_pad_info0.sda.gp, 0);
			gpio_direction_output(i2c_pad_info0.scl.gp, 0);

			// Enable RTC FET
			gpio_direction_output(TS7990_EN_RTC, 1);
			udelay(1000*140); // 140ms to discharge
			gpio_direction_output(TS7990_EN_RTC, 0);
			udelay(1000*2); // 2ms to turn on
			imx_iomux_v3_setup_pad(i2c_pad_info0.sda.i2c_mode);
			imx_iomux_v3_setup_pad(i2c_pad_info0.scl.i2c_mode);

			if(i == 4) puts ("Not able to force bus idle.  Giving up.\n");
		}
	}
	
	setup_i2c(0, CONFIG_SYS_I2C_SPEED, 0x7f, &i2c_pad_info0);

	/* Reset pcap touch */
	if(detect_lcd() == LCD_LXD) {
		uint8_t val;

		i2c_read(0x28, 59, 2, &val, 1);
		val |= (1 << 5);
		i2c_write(0x28, 59, 2, &val, 1);
		val &= ~(1 << 5);
		i2c_write(0x28, 59, 2, &val, 1);
	}

	/* address of boot parameters */
	gd->bd->bi_boot_params = PHYS_SDRAM + 0x100;
	setup_spi();

	#ifdef CONFIG_CMD_SATA
	setup_sata();
	#endif

	#ifdef CONFIG_CMD_BMODE
	add_board_boot_modes(board_boot_modes);
	#endif

	#ifdef CONFIG_FPGA
	ts7990_fpga_init();
	#endif

	return 0;
}

int board_late_init(void)
{
	return 0;
}

int checkboard(void)
{
	printf("Board: TS-TPC-7990\n");
	return 0;
}
