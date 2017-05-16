/*
 * Copyright (C) 2015 Freescale Semiconductor, Inc.
 *
 * Configuration settings for the Freescale i.MX6UL 14x14 DDR3 ARM2.
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */
#ifndef __TS4100_CONFIG_H
#define __TS4100_CONFIG_H

#error "U-Boot 2015.04 for the TS-4100 is deprecated. Remove the #error from the TS-4100 config file to continue"


#include <asm/arch/imx-regs.h>
#include <linux/sizes.h>
#include "mx6_common.h"
#include <asm/imx-common/gpio.h>

/* Env is at the 1MB boundary in emmc boot partition 1 */
#define CONFIG_ENV_IS_IN_MMC
#define CONFIG_SYS_MMC_ENV_DEV  	1
#define CONFIG_SYS_MMC_ENV_PART 	1
#define CONFIG_ENV_OFFSET		0x100000     /* 1MB */
#define CONFIG_ENV_SIZE			SZ_16K
#define CONFIG_ENV_OFFSET_REDUND 0x200000

#define CONFIG_FSL_USDHC
#define CONFIG_MX6
#define CONFIG_ROM_UNIFIED_SECTIONS
#define CONFIG_SYS_GENERIC_BOARD
#define CONFIG_DISPLAY_CPUINFO
#define CONFIG_DISPLAY_BOARDINFO
#define CONFIG_SYS_CONSOLE_INFO_QUIET

#define CONFIG_BOARD_SPECIFIC_LED
#define CONFIG_STATUS_LED
#define CONFIG_CMD_LED

#define STATUS_LED_RED                  0
#define STATUS_LED_GREEN                1

#define STATUS_LED_BIT                  STATUS_LED_RED
#define STATUS_LED_STATE                STATUS_LED_ON
#define STATUS_LED_PERIOD               (CONFIG_SYS_HZ / 2)

#define STATUS_LED_BIT1                 STATUS_LED_GREEN
#define STATUS_LED_STATE1               STATUS_LED_ON
#define STATUS_LED_PERIOD1              (CONFIG_SYS_HZ / 2)

/* No PMIC */
#undef CONFIG_LDO_BYPASS_CHECK

#define CONFIG_CMDLINE_TAG
#define CONFIG_SETUP_MEMORY_TAGS
#define CONFIG_INITRD_TAG
#define CONFIG_REVISION_TAG

/* Size of malloc() pool */
#define CONFIG_SYS_MALLOC_LEN		(16 * SZ_1M)

#define CONFIG_BOARD_EARLY_INIT_F
#define CONFIG_BOARD_LATE_INIT
#define CONFIG_MXC_GPIO
#define CONFIG_CMD_GPIO

#define CONFIG_MXC_UART
#define CONFIG_MXC_UART_BASE		UART1_BASE

/* allow to overwrite serial and ethaddr */
#define CONFIG_ENV_OVERWRITE
#define CONFIG_CONS_INDEX		1
#define CONFIG_BAUDRATE			115200

#define CONFIG_CMD_FUSE
#ifdef CONFIG_CMD_FUSE
#define CONFIG_MXC_OCOTP
#endif

#define CONFIG_FPGA
#define CONFIG_FPGA_LATTICE

#undef CONFIG_BOOTM_NETBSD
#undef CONFIG_BOOTM_PLAN9
#undef CONFIG_BOOTM_RTEMS

/* I2C configs */
#define CONFIG_CMD_I2C
#define CONFIG_SYS_I2C
#define CONFIG_SYS_I2C_MXC
#define CONFIG_SYS_I2C_SPEED		100000

/* Command definition */
#include <config_cmd_default.h>

#undef CONFIG_CMD_IMLS

#define CONFIG_BOOTDELAY	       	1
#define CONFIG_AUTOBOOT_KEYED 		1
#define CONFIG_AUTOBOOT_PROMPT "Press Ctrl+C to abort autoboot in %d second(s)\n", bootdelay
#define CTRL(c) ((c)&0x1F)     
#define CONFIG_AUTOBOOT_STOP_STR  (char []){CTRL('C'), 0}

#define CONFIG_LOADADDR			0x80800000
#define CONFIG_SYS_TEXT_BASE		0x87800000

#define CONFIG_PREBOOT ""

#define CONFIG_EXTRA_ENV_SETTINGS \
	"fdt_high=0xffffffff\0" \
	"initrd_high=0xffffffff\0" \
	"fdtaddr=0x83000000\0" \
	"model=4100\0" \
	"autoload=no\0" \
	"nfsip=192.168.1.139\0" \
	"nfsroot=/mnt/storage/imx6\0" \
	"clearenv=mmc dev 1 1; mmc erase 800 20; mmc erase 1000 20;\0" \
	"cmdline_append=console=ttymxc0,115200 init=/sbin/init\0" \
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
		"if load mmc 0:1 ${fdtaddr} /boot/imx6ul-ts4100-${baseboardid}.dtb;" \
			"then echo $baseboardid detected;" \
		"else " \
			"echo Booting default device tree;" \
			"load mmc 0:1 ${fdtaddr} /boot/imx6ul-ts4100.dtb;" \
		"fi;" \
		"load mmc 0:1 ${loadaddr} /boot/uImage;" \
		"setenv bootargs root=/dev/mmcblk0p1 rootwait rw ${cmdline_append};" \
		"bootm ${loadaddr} - ${fdtaddr};\0" \
	"emmcboot=echo Booting from the eMMC ...;" \
		"if load mmc 1:1 ${loadaddr} /boot/boot.ub;" \
			"then echo Booting from custom /boot/boot.ub;" \
			"source ${loadaddr};" \
		"fi;" \
		"if load mmc 1:1 ${fdtaddr} /boot/imx6ul-ts4100-${baseboardid}.dtb;" \
			"then echo $baseboardid detected;" \
		"else " \
			"echo Booting default device tree;" \
			"load mmc 1:1 ${fdtaddr} /boot/imx6ul-ts4100.dtb;" \
		"fi;" \
		"load mmc 1:1 ${loadaddr} /boot/uImage;" \
		"setenv bootargs root=/dev/mmcblk1p1 rootwait rw ${cmdline_append};" \
		"bootm ${loadaddr} - ${fdtaddr};\0" \
	"nfsboot=echo Booting from NFS ...;" \
		"dhcp;" \
		"mw.l ${fdtaddr} 0 1000;" \
		"mw.l ${loadaddr} 0 1000;" \
		"nfs ${fdtaddr} ${nfsip}:${nfsroot}/boot/imx6ul-ts4100-${baseboardid}.dtb;" \
		"if fdt addr ${fdtaddr};" \
			"then echo Baseboard $baseboardid detected;" \
		"else " \
			"echo Booting default device tree;" \
			"nfs ${fdtaddr} ${nfsip}:${nfsroot}/boot/imx6ul-ts4100.dtb;" \
		"fi;" \
		"nfs ${loadaddr} ${nfsip}:${nfsroot}/boot/uImage;" \
		"setenv bootargs root=/dev/nfs ip=dhcp nfsroot=${nfsip}:${nfsroot} " \
			"rootwait rw ${cmdline_append};" \
		"bootm ${loadaddr} - ${fdtaddr};\0" \
	"bootcmd_mfg=echo MFG boot;" \
		"if mmc dev 0;" \
			"then load mmc 0:1 ${loadaddr} /prime-ts4100.ub;" \
			"source ${loadaddr};" \
			"exit;" \
		"fi;" \
		"dhcp;" \
		"nfs ${loadaddr} 192.168.0.11:/u/x/jessie-armel/boot-imx6ul/prime-ts4100.ub;" \
		"source ${loadaddr};\0"

#define CONFIG_BOOTCOMMAND \
	"run usbprod;" \
	"if test ${jpsdboot} = 'on' ;" \
		"then run sdboot;" \
		"else run emmcboot;" \
	"fi;"

/* Miscellaneous configurable options */
#define CONFIG_SYS_LONGHELP
#define CONFIG_SYS_HUSH_PARSER
#define CONFIG_SYS_PROMPT		"U-Boot > "
#define CONFIG_AUTO_COMPLETE
#define CONFIG_SYS_CBSIZE		1024
#define CONFIG_CMD_SETEXPR

#define CONFIG_CMD_BMODE

/* Print Buffer Size */
#define CONFIG_SYS_PBSIZE (CONFIG_SYS_CBSIZE + sizeof(CONFIG_SYS_PROMPT) + 16)
#define CONFIG_SYS_MAXARGS		256
#define CONFIG_SYS_BARGSIZE CONFIG_SYS_CBSIZE

#define CONFIG_CMD_MEMTEST
#define CONFIG_SYS_MEMTEST_START	0x80000000
#define CONFIG_SYS_MEMTEST_END		(CONFIG_SYS_MEMTEST_START + 0x20000000)
#define CONFIG_CMD_TIME

#define CONFIG_SYS_LOAD_ADDR		CONFIG_LOADADDR
#define CONFIG_SYS_HZ			1000

#define CONFIG_CMDLINE_EDITING
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

#define CONFIG_MMC
#define CONFIG_CMD_MMC
#define CONFIG_GENERIC_MMC
#define CONFIG_CMD_EXT2
#define CONFIG_CMD_EXT2_WRITE
#define CONFIG_CMD_EXT4
#define CONFIG_CMD_EXT4_WRITE
#define CONFIG_CMD_FS_GENERIC
#define CONFIG_CMD_FAT
#define CONFIG_FAT_WRITE
#define CONFIG_DOS_PARTITION
#define CONFIG_SUPPORT_EMMC_BOOT /* eMMC specific */

#define CONFIG_OF_LIBFDT
#define CONFIG_CMD_BOOTZ
#define CONFIG_SUPPORT_RAW_INITRD

#ifndef CONFIG_SYS_DCACHE_OFF
#define CONFIG_CMD_CACHE
#endif

/* USB Configs */
#define CONFIG_CMD_USB
#define CONFIG_USB_EHCI
#define CONFIG_USB_EHCI_MX6
#define CONFIG_USB_STORAGE
#define CONFIG_EHCI_HCD_INIT_AFTER_RESET
#define CONFIG_USB_GADGET
#define CONFIG_CI_UDC
#define CONFIG_MXC_USB_PORTSC  (PORT_PTS_UTMI | PORT_PTS_PTW)
#define CONFIG_MXC_USB_FLAGS   0
#define CONFIG_USB_MAX_CONTROLLER_COUNT 2
#define CONFIG_USB_GADGET_MASS_STORAGE
#define CONFIG_CMD_USB_MASS_STORAGE
#define CONFIG_USB_GADGET_DUALSPEED
#define CONFIG_USBDOWNLOAD_GADGET
#define CONFIG_USB_GADGET_VBUS_DRAW	2
#define CONFIG_G_DNL_VENDOR_NUM		0x15a2
#define CONFIG_G_DNL_PRODUCT_NUM	0x4100
#define CONFIG_G_DNL_MANUFACTURER	"FSL"

#define CONFIG_CMD_NET
#ifdef CONFIG_CMD_NET
#define CONFIG_CMD_PING
#define CONFIG_CMD_DHCP
#define CONFIG_CMD_MII
#define CONFIG_FEC_MXC
#define CONFIG_USB_HOST_ETHER
#define CONFIG_USB_ETHER_ASIX
#define CONFIG_MII
#define CONFIG_LIB_RAND
#define CONFIG_FEC_ENET_DEV 0
#define CONFIG_NFS_TIMEOUT 100UL

#if (CONFIG_FEC_ENET_DEV == 0)
#define IMX_FEC_BASE			ENET_BASE_ADDR
#define CONFIG_FEC_MXC_PHYADDR          0x2
#define CONFIG_FEC_XCV_TYPE             RMII
#elif (CONFIG_FEC_ENET_DEV == 1)
#define IMX_FEC_BASE			ENET2_BASE_ADDR
#define CONFIG_FEC_MXC_PHYADDR          0x1 
#define CONFIG_FEC_XCV_TYPE             RMII
#endif
#define CONFIG_ETHPRIME                 "FEC"

#define CONFIG_PHYLIB
#define CONFIG_PHY_MICREL
#define CONFIG_FEC_DMA_MINALIGN		64
#endif

#endif /* __TS4100_CONFIG_H */
