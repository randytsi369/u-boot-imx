/*
 * Copyright (C) 2016 Technologic Systems
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */
#ifndef __TS7553V2_CONFIG_H
#define __TS7553V2_CONFIG_H


#include <asm/arch/imx-regs.h>
#include <linux/sizes.h>
#include <asm/imx-common/gpio.h>
#include "mx6_common.h"

/* Env is at the 1MB boundary in emmc boot partition 0 */
#define CONFIG_ENV_IS_IN_MMC
#define CONFIG_SYS_MMC_ENV_DEV  	1 /* mmcblk0 */
#define CONFIG_SYS_MMC_ENV_PART 	1 /* boot0 */
/* Erase block on our Micron emmc is 4MiB.  Separate redund by this */

#define CONFIG_ENV_OFFSET		0x400000 /* 8MiB */
#define CONFIG_ENV_SIZE			SZ_128K
#define CONFIG_ENV_OFFSET_REDUND 	0x800000 /* 12MiB */

#define CONFIG_ENV_VARS_UBOOT_RUNTIME_CONFIG
#define CONFIG_MODULE_FUSE

#define CONFIG_FSL_USDHC
#define CONFIG_MX6
#define CONFIG_SYS_CONSOLE_INFO_QUIET

#define CONFIG_BOARD_SPECIFIC_LED
#define CONFIG_STATUS_LED
#define CONFIG_CMD_LED

#define CONFIG_RED_LED                  115
#define CONFIG_GREEN_LED                116
#define STATUS_LED_RED                  0
#define STATUS_LED_GREEN                1

#define STATUS_LED_BIT                  STATUS_LED_RED
#define STATUS_LED_STATE                STATUS_LED_ON
#define STATUS_LED_PERIOD               (CONFIG_SYS_HZ / 2)

/* #define CONFIG_USE_PLUGIN */

/* No PMIC */
#undef CONFIG_LDO_BYPASS_CHECK

/* Size of malloc() pool */
#define CONFIG_SYS_MALLOC_LEN		(16 * SZ_1M)

#define CONFIG_BOARD_EARLY_INIT_F
#define CONFIG_MISC_INIT_R
#define CONFIG_BOARD_LATE_INIT

#define CONFIG_MXC_UART
#define CONFIG_MXC_UART_BASE		UART1_BASE

#define CONFIG_FPGA
#define CONFIG_FPGA_LATTICE

#undef CONFIG_BOOTM_NETBSD
#undef CONFIG_BOOTM_PLAN9
#undef CONFIG_BOOTM_RTEMS

#define CONFIG_CMD_SPI
#define CONFIG_MXC_SPI
#define CONFIG_ATMEL_WIFI_BUS		3 /* ECSPI4 */
#define CONFIG_ATMEL_WIFI_CS		0 /* ECSPI4 */
#define CONFIG_FRAM_BUS			2 /* ECSPI3 */
#define CONFIG_FRAM_CS			0 /* ECSPI3 */

/* I2C configs */
#define CONFIG_CMD_I2C
#define CONFIG_SYS_I2C
#define CONFIG_SYS_I2C_MXC
#define CONFIG_SYS_I2C_MXC_I2C1		/* enable I2C bus 1 */
#define CONFIG_SYS_I2C_SPEED		100000

#undef CONFIG_BOOTDELAY
#define CONFIG_BOOTDELAY               0
#define CONFIG_AUTOBOOT_KEYED          1

#define CONFIG_EXTRA_ENV_SETTINGS \
        "chrg_pct=0\0" \
        "chrg_verb=0\0" \
	"rstuboot=1\0" \
	"fdt_high=0xffffffff\0" \
	"initrd_high=0xffffffff\0" \
	"fdtaddr=0x83000000\0" \
	"autoload=no\0" \
	"nfsroot=192.168.0.1:/usr/local/ts7553v2-nfsroot/\0" \
	"clearenv=mmc dev 1 1; mmc erase 2000 2000; mmc erase 4000 2000;\0" \
	"cmdline_append=rw rootwait console=ttymxc0,115200 loglevel=3\0" \
	"usbprod=usb start;" \
		"if usb storage;" \
			"then echo Checking USB storage for updates;" \
			"if load usb 0:1 ${loadaddr} /tsinit.ub;" \
				"then led green on;" \
				"source ${loadaddr};" \
				"led red off;" \
				"exit;" \
			"fi;" \
		"fi;\0" \
	"sdboot=echo Booting from the SD card ...;" \
		"if load mmc 0:1 ${loadaddr} /boot/boot.ub;" \
			"then echo Booting from custom /boot/boot.ub;" \
			"source ${loadaddr};" \
		"fi;" \
		"load mmc 0:1 ${fdtaddr} /boot/imx6ul-ts7553v2.dtb;" \
		"load mmc 0:1 ${loadaddr} /boot/zImage;" \
		"setenv bootargs root=/dev/mmcblk0p1 ${cmdline_append};" \
		"bootz ${loadaddr} - ${fdtaddr};\0" \
	"emmcboot=echo Booting from the eMMC ...;" \
		"if load mmc 1:1 ${loadaddr} /boot/boot.ub;" \
			"then echo Booting from custom /boot/boot.ub;" \
			"source ${loadaddr};" \
		"fi;" \
		"load mmc 1:1 ${fdtaddr} /boot/imx6ul-ts7553v2.dtb;" \
		"load mmc 1:1 ${loadaddr} /boot/zImage;" \
		"setenv bootargs root=/dev/mmcblk1p1 ${cmdline_append};" \
		"bootz ${loadaddr} - ${fdtaddr};\0" \
	"nfsboot=echo Booting from NFS ...;" \
		"dhcp;" \
		"nfs ${fdtaddr} ${nfsroot}/boot/imx6ul-ts7553v2.dtb;" \
		"nfs ${loadaddr} ${nfsroot}/boot/zImage;" \
		"setenv bootargs root=/dev/nfs ip=dhcp nfsroot=${nfsroot} " \
			"${cmdline_append};" \
		"bootz ${loadaddr} - ${fdtaddr}; \0" \
	"bootcmd_mfg=echo MFG boot;" \
		"if mmc dev 0;" \
			"then load mmc 0:1 ${loadaddr} /prime-ts7553v2.ub;" \
			"source ${loadaddr};" \
			"exit;" \
		"fi;" \
		"dhcp;" \
		"nfs ${loadaddr} 192.168.0.11:/u/x/jessie-armel/boot-imx6ul/prime-ts7553v2.ub;" \
		"source ${loadaddr};\0" \
	"update-uboot=echo Updating U-Boot image;" \
		"load mmc 0:1 ${loadaddr} /boot/u-boot.imx;"\
		"setexpr filesize ${filesize} / 200;" \
		"setexpr filesize ${filesize} + 1;" \
		"mmc dev 1 1;" \
		"mmc write ${loadaddr} 2 ${filesize};\0" \
	"emmc-ums=gpio set 71; ums 0 mmc 1;\0" \

#define CONFIG_BOOTCOMMAND \
	"if test \"${jpuboot}\" = \"on\"; then " \
		"run usbprod;" \
	"else;"\
		"tsmicroctl w ${chrg_pct} ${chrg_verb};"\
		"if test ${jpsdboot} = 'on';" \
			"then run sdboot;" \
			"else run emmcboot;" \
		"fi;" \
	"fi;"

#define CONFIG_CMD_MEMTEST
#define CONFIG_SYS_MEMTEST_START	0x80800000
#define CONFIG_SYS_MEMTEST_END		(CONFIG_SYS_MEMTEST_START + 0x10000)
#define CONFIG_SYS_ALT_MEMTEST
#define CONFIG_CMD_TIME

#define CONFIG_SYS_HZ			1000

#define CONFIG_STACKSIZE		SZ_128K

/* Physical Memory Map */
#define CONFIG_NR_DRAM_BANKS		1
#define PHYS_SDRAM			MMDC0_ARB_BASE_ADDR
#define PHYS_SDRAM_SIZE			SZ_512M

#define CONFIG_SYS_SDRAM_BASE		PHYS_SDRAM
#define CONFIG_SYS_INIT_RAM_ADDR	IRAM_BASE_ADDR
#define CONFIG_SYS_INIT_RAM_SIZE	IRAM_SIZE

#define CONFIG_SYS_INIT_SP_OFFSET \
	(CONFIG_SYS_INIT_RAM_SIZE - GENERATED_GBL_DATA_SIZE)
#define CONFIG_SYS_INIT_SP_ADDR \
	(CONFIG_SYS_INIT_RAM_ADDR + CONFIG_SYS_INIT_SP_OFFSET)

#define CONFIG_SYS_NO_FLASH

/* MMC Configs */
#define CONFIG_FSL_ESDHC
#define CONFIG_SYS_FSL_ESDHC_ADDR	0
#define CONFIG_SYS_FSL_USDHC_NUM	2

#define CONFIG_CMD_EXT2_WRITE
#define CONFIG_CMD_FAT
#define CONFIG_FAT_WRITE

/* USB Configs */
#define CONFIG_CI_UDC
#define CONFIG_CMD_USB
#define CONFIG_CMD_USB_MASS_STORAGE
#define CONFIG_EHCI_HCD_INIT_AFTER_RESET
#define CONFIG_G_DNL_MANUFACTURER	"FSL"
#define CONFIG_G_DNL_PRODUCT_NUM	0x7553
#define CONFIG_G_DNL_VENDOR_NUM		0x15a2
#define CONFIG_MXC_USB_FLAGS   0
#define CONFIG_MXC_USB_PORTSC  (PORT_PTS_UTMI | PORT_PTS_PTW)
#define CONFIG_USB_STORAGE
#define CONFIG_USB_EHCI
#define CONFIG_USB_EHCI_MX6
#define CONFIG_USB_FUNCTION_MASS_STORAGE
#define CONFIG_USB_GADGET
#define CONFIG_USB_GADGET_DOWNLOAD
#define CONFIG_USB_GADGET_DUALSPEED
#define CONFIG_USB_GADGET_VBUS_DRAW	2
#define CONFIG_USB_MAX_CONTROLLER_COUNT 2

#ifdef CONFIG_CMD_NET
#define CONFIG_CMD_MII
#define CONFIG_FEC_MXC
#define CONFIG_MII
#define CONFIG_NFS_TIMEOUT 100UL
#define CONFIG_FEC_ENET_DEV 0

#if (CONFIG_FEC_ENET_DEV == 0)
#define IMX_FEC_BASE			ENET_BASE_ADDR
#define CONFIG_FEC_MXC_PHYADDR          0x2
#define CONFIG_FEC_XCV_TYPE             RMII
#elif (CONFIG_FEC_ENET_DEV == 1)
#define IMX_FEC_BASE			ENET2_BASE_ADDR
#define CONFIG_FEC_MXC_PHYADDR          0x1 
#define CONFIG_FEC_XCV_TYPE             RMII
#endif
#define CONFIG_ETHPRIME                 "FEC0"

#define CONFIG_PHYLIB
#define CONFIG_PHY_MICREL
#endif

#endif /* __TS7553V2_CONFIG_H */
