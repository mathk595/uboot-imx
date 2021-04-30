/*
 * Copyright 2018 NXP
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#if 0
#define DEBUG 1
#define CONFIG_SYS_DCACHE_OFF
#endif

#ifndef __IMX8MM_TRIZEPS8MINI_H
#define __IMX8MM_TRIZEPS8MINI_H

#include <linux/sizes.h>
#include <asm/arch/imx-regs.h>

#include "imx_env.h"

#ifdef CONFIG_SECURE_BOOT
#define CONFIG_CSF_SIZE			0x2000 /* 8K region */
#endif

/*
#define BOOTLOADER_RBIDX_OFFSET        0x1E000
#define CONFIG_IMX_TRUSTY_OS           1
#define CONFIG_FSL_CAAM_KB             1
*/

/* ENABLE_UNUSED_CLOCKS_IMX8MM:  Enables clocks which are not used by Bootloader:
   make sure that it will be passed to arch/arm/mach-imx/imx8mm/clocks_imx8mm.c    */
//      #define ENABLE_UNUSED_CLOCKS_IMX8MM    1  

/* Init i2c (Camera) during U-Boot init:  Disabled because i2c commands might not work later */
//      #define ENABLE_I2C_TRIZEPS8MINI        0


#define CONF_SYS_BL_PART              0  // use user Partition 1=boot partition
#define LOG_DDR4_TRAINING_LEVEL         0

#define CONFIG_SPL_MAX_SIZE		(148 * 1024)
#define CONFIG_SYS_MONITOR_LEN		(512 * 1024)
#define CONFIG_SYS_MMCSD_RAW_MODE_U_BOOT_USE_SECTOR
#define CONFIG_SYS_MMCSD_RAW_MODE_U_BOOT_SECTOR	0x300
#define CONFIG_SYS_MMCSD_FS_BOOT_PARTITION	1
#define CONFIG_SYS_UBOOT_BASE		(QSPI0_AMBA_BASE + CONFIG_SYS_MMCSD_RAW_MODE_U_BOOT_SECTOR * 512)

#ifdef CONFIG_SPL_BUILD
/*#define CONFIG_ENABLE_DDR_TRAINING_DEBUG*/
#define CONFIG_SPL_WATCHDOG_SUPPORT
#define CONFIG_SPL_POWER_SUPPORT
#define CONFIG_SPL_DRIVERS_MISC_SUPPORT
#define CONFIG_SPL_I2C_SUPPORT
#define CONFIG_SPL_LDSCRIPT		"arch/arm/cpu/armv8/u-boot-spl.lds"
#define CONFIG_SPL_STACK		0x91fff0
#define CONFIG_SPL_LIBCOMMON_SUPPORT
#define CONFIG_SPL_LIBGENERIC_SUPPORT
#define CONFIG_SPL_GPIO_SUPPORT
#define CONFIG_SPL_BSS_START_ADDR      0x00910000
#define CONFIG_SPL_BSS_MAX_SIZE        0x2000	/* 8 KB */
#define CONFIG_SYS_SPL_MALLOC_START    0x42200000
#define CONFIG_SYS_SPL_MALLOC_SIZE     0x80000	/* 512 KB */
#define CONFIG_SYS_ICACHE_OFF
#define CONFIG_SYS_DCACHE_OFF

#define CONFIG_MALLOC_F_ADDR		0x912000 /* malloc f used before GD_FLG_FULL_MALLOC_INIT set */

#define CONFIG_SPL_ABORT_ON_RAW_IMAGE /* For RAW image gives a error info not panic */

/* #undef CONFIG_DM_MMC */
#undef CONFIG_DM_PMIC
#undef CONFIG_DM_PMIC_PFUZE100

#define CONFIG_POWER
#define CONFIG_POWER_I2C
#define CONFIG_POWER_BD71837

#define CONFIG_SYS_I2C
#define CONFIG_SYS_I2C_MXC_I2C1		/* enable I2C bus 1 */
#define CONFIG_SYS_I2C_MXC_I2C2		/* enable I2C bus 2 */
#define CONFIG_SYS_I2C_MXC_I2C3		/* enable I2C bus 3 */

#define CONFIG_ENV_VARS_UBOOT_RUNTIME_CONFIG

#if defined(CONFIG_NAND_BOOT)
#define CONFIG_SPL_NAND_SUPPORT
#define CONFIG_SPL_DMA_SUPPORT
#define CONFIG_SPL_NAND_MXS
#define CONFIG_SYS_NAND_U_BOOT_OFFS 	0x4000000 /* Put the FIT out of first 64MB boot area */

/* Set a redundant offset in nand FIT mtdpart. The new uuu will burn full boot image (not only FIT part) to the mtdpart, so we check both two offsets */
#define CONFIG_SYS_NAND_U_BOOT_OFFS_REDUND \
	(CONFIG_SYS_NAND_U_BOOT_OFFS + CONFIG_SYS_MMCSD_RAW_MODE_U_BOOT_SECTOR * 512 - 0x8400)
#endif

#endif

#define CONFIG_CMD_READ
#define CONFIG_SERIAL_TAG
#define CONFIG_FASTBOOT_USB_DEV 0

#define CONFIG_REMAKE_ELF

#define CONFIG_BOARD_EARLY_INIT_F
#define CONFIG_BOARD_POSTCLK_INIT
#define CONFIG_BOARD_LATE_INIT

/* Flat Device Tree Definitions */
#define CONFIG_OF_BOARD_SETUP

#undef CONFIG_CMD_EXPORTENV
#undef CONFIG_CMD_IMPORTENV
#undef CONFIG_CMD_IMLS

#undef CONFIG_CMD_CRC32
#undef CONFIG_BOOTM_NETBSD

/* ENET Config */
/* ENET1 */
#if defined(CONFIG_CMD_NET)
#define CONFIG_CMD_PING
#define CONFIG_CMD_DHCP
#define CONFIG_CMD_MII
#define CONFIG_MII
#define CONFIG_ETHPRIME                 "FEC"

#define CONFIG_FEC_MXC
#define CONFIG_FEC_XCV_TYPE             RGMII
#define CONFIG_FEC_MXC_PHYADDR          4
#define FEC_QUIRK_ENET_MAC

#define CONFIG_PHY_GIGE
#define IMX_FEC_BASE			0x30BE0000

/*
 #define CONFIG_PHYLIB
 #define CONFIG_PHY_ATHEROS
*/
#endif


#define CONFIG_LOADADDR					0x40480000
#define CONFIG_SYS_LOAD_ADDR            CONFIG_LOADADDR
#define SCRIPT_ADDR                     0x40400000

#ifndef  CONFIG_APPEND_BOOTARGS
#define CONFIG_APPEND_BOOTARGS          1
#endif

/*
 * Another approach is add the clocks for inmates into clks_init_on
 * in clk-imx8mm.c, then clk_ingore_unused could be removed.
 */
#define JAILHOUSE_ENV \
	"jh_clk= \0 " \
	"jh_mmcboot=mw 0x303d0518 0xff; setenv fdt_file kuk-trizeps8mini-root.dtb;" \
		"setenv jh_clk clk_ignore_unused; " \
			   "if run loadimage; then " \
				   "run mmcboot; " \
			   "else run jh_netboot; fi; \0" \
	"jh_netboot=mw 0x303d0518 0xff; setenv fdt_file kuk-trizeps8mini-root.dtb; setenv jh_clk clk_ignore_unused; run netboot; \0 "

#ifdef CONFIG_NAND_BOOT
#define MFG_NAND_PARTITION "mtdparts=gpmi-nand:64m(nandboot),16m(nandfit),32m(nandkernel),16m(nanddtb),8m(nandtee),-(nandrootfs) "
#endif

#define CONFIG_MFG_ENV_SETTINGS \
	CONFIG_MFG_ENV_SETTINGS_DEFAULT \
	"initrd_addr=0x43800000\0" \
	"initrd_high=0xffffffffffffffff\0" \
	"emmc_dev=0\0"\
	"sd_dev=1\0" \

#define KUK_FUSE_SETTINGS \
	"fuse_emmc=fuse prog 1 3 0x10002012 \0" \
	"fuse_sd=fuse prog 1 3 0x10001010 \0" \

#ifdef CONFIG_MYON2_V1R1_1GB
#define KUK_FUSE_PRODUCTION \
	KUK_FUSE_SETTINGS \
	"fuse_config=fuse prog 14 0 0x21000000; run fuse_emmc \0" 
#else
#ifdef CONFIG_MYON2_V1R1_2GB
#define KUK_FUSE_PRODUCTION \
	KUK_FUSE_SETTINGS \
	"fuse_config=fuse prog 14 0 0x22000000; run fuse_emmc \0" 
#else
#ifdef CONFIG_MYON2_V1R1_4GB
#define KUK_FUSE_PRODUCTION \
	KUK_FUSE_SETTINGS \
	"fuse_config=fuse prog 14 0 0x24000000; run fuse_emmc \0" 
#else
#ifdef CONFIG_TRIZEPS8MINI_V1R2_1GB
#ifdef CONFIG_MOUNTOPTION_EMMC
#define KUK_FUSE_PRODUCTION \
	KUK_FUSE_SETTINGS \
	"fuse_config=fuse prog 14 0 0x11100000; run fuse_emmc \0" 
#else
#ifdef CONFIG_MOUNTOPTION_SD
#define KUK_FUSE_PRODUCTION \
	KUK_FUSE_SETTINGS \
	"fuse_config=fuse prog 14 0 0x11100000; run fuse_sd \0" 
#else
#define KUK_FUSE_PRODUCTION
#endif
#endif
#else
#ifdef CONFIG_TRIZEPS8MINI_V1R2_2GB
#ifdef CONFIG_MOUNTOPTION_EMMC
#define KUK_FUSE_PRODUCTION \
	KUK_FUSE_SETTINGS \
	"fuse_config=fuse prog 14 0 0x12100000; run fuse_emmc \0" 
#else
#ifdef CONFIG_MOUNTOPTION_SD
#define KUK_FUSE_PRODUCTION \
	KUK_FUSE_SETTINGS \
	"fuse_config=fuse prog 14 0 0x12100000; run fuse_sd \0" 
#else
#define KUK_FUSE_PRODUCTION
#endif
#endif
#else
#ifdef CONFIG_TRIZEPS8MINI_V1R2_4GB
#ifdef CONFIG_MOUNTOPTION_EMMC
#define KUK_FUSE_PRODUCTION \
	KUK_FUSE_SETTINGS \
	"fuse_config=fuse prog 14 0 0x14100000; run fuse_emmc \0" 
#else
#ifdef CONFIG_MOUNTOPTION_SD
#define KUK_FUSE_PRODUCTION \
	KUK_FUSE_SETTINGS \
	"fuse_config=fuse prog 14 0 0x14100000; run fuse_sd \0" 
#else
#define KUK_FUSE_PRODUCTION
#endif
#endif
#else
#ifdef CONFIG_SBCSOM8MINI_V1R1_1GB
#ifdef CONFIG_MOUNTOPTION_EMMC
#define KUK_FUSE_PRODUCTION \
	KUK_FUSE_SETTINGS \
	"fuse_config=fuse prog 14 0 0x41000000; run fuse_emmc \0" 
#else
#ifdef CONFIG_MOUNTOPTION_SD
#define KUK_FUSE_PRODUCTION \
	KUK_FUSE_SETTINGS \
	"fuse_config=fuse prog 14 0 0x41000000; run fuse_sd \0" 
#else
#define KUK_FUSE_PRODUCTION
#endif
#endif
#else
#ifdef CONFIG_SBCSOM8MINI_V1R1_2GB
#ifdef CONFIG_MOUNTOPTION_EMMC
#define KUK_FUSE_PRODUCTION \
	KUK_FUSE_SETTINGS \
	"fuse_config=fuse prog 14 0 0x42000000; run fuse_emmc \0" 
#else
#ifdef CONFIG_MOUNTOPTION_SD
#define KUK_FUSE_PRODUCTION \
	KUK_FUSE_SETTINGS \
	"fuse_config=fuse prog 14 0 0x42000000; run fuse_sd \0" 
#else
#define KUK_FUSE_PRODUCTION
#endif
#endif
#else
#ifdef CONFIG_SBCSOM8MINI_V1R1_4GB
#ifdef CONFIG_MOUNTOPTION_EMMC
#define KUK_FUSE_PRODUCTION \
	KUK_FUSE_SETTINGS \
	"fuse_config=fuse prog 14 0 0x44000000; run fuse_emmc \0" 
#else
#ifdef CONFIG_MOUNTOPTION_SD
#define KUK_FUSE_PRODUCTION \
	KUK_FUSE_SETTINGS \
	"fuse_config=fuse prog 14 0 0x44000000; run fuse_sd \0" 
#else
#define KUK_FUSE_PRODUCTION
#endif
#endif
#else
#define KUK_FUSE_PRODUCTION
#endif
#endif
#endif
#endif
#endif  
#endif
#endif 
#endif 
#endif 

/* Initial environment variables */
#if defined(CONFIG_NAND_BOOT)
#define CONFIG_EXTRA_ENV_SETTINGS \
	CONFIG_MFG_ENV_SETTINGS \
	"fdt_addr=0x43000000\0"			\
	"fdt_high=0xffffffffffffffff\0" \
	"mtdparts=" MFG_NAND_PARTITION "\0" \
	"console=ttymxc0,115200 earlycon=ec_imx6q,0x30860000,115200\0" \
	"bootargs=console=ttymxc0,115200 earlycon=ec_imx6q,0x30860000,115200 ubi.mtd=5 "  \
		"root=ubi0:nandrootfs rootfstype=ubifs "		     \
		MFG_NAND_PARTITION \
		"\0" \
	"bootcmd=nand read ${loadaddr} 0x5000000 0x2000000;"\
		"nand read ${fdt_addr} 0x7000000 0x100000;"\
		"booti ${loadaddr} - ${fdt_addr}"

#else
#define CONFIG_EXTRA_ENV_SETTINGS		\
	CONFIG_MFG_ENV_SETTINGS \
	JAILHOUSE_ENV \
	KUK_FUSE_PRODUCTION \
	"display=auto\0"                              \
	"pcie=none\0"                              \
	"append_bootargs=androidboot.selinux=permissive\0"\
	"script=boot.scr\0" \
	"splashpos=m,m\0"			   \
	"image=Image\0"                            \
	"console=ttymxc0,115200 earlycon=ec_imx6q,0x30860000,115200\0" \
	"fdt_addr=0x43000000\0"			    \
    "script_addr="__stringify(SCRIPT_ADDR)"\0"  \
	"fdt_high=0xffffffffffffffff\0"	            \
	"boot_fdt=try\0"                            \
	"fdt_file=" CONFIG_DEFAULT_FDT_FILE "\0"    \
	"initrd_addr=0x43800000\0"		    \
	"initrd_high=0xffffffffffffffff\0"          \
	"uart4-access=both\0"                       \
	"uart2=acess=both\0"                        \
	"emmc_ack=1\0"                              \
	"emmc_dev="__stringify(CONFIG_SYS_MMC_ENV_DEV)"\0"       \
	"mmcdev="__stringify(CONFIG_SYS_MMC_ENV_DEV)"\0"         \
	"mmcpart="__stringify(CONFIG_SYS_MMC_IMG_LOAD_PART) "\0" \
	"target_ubootdev="__stringify(CONFIG_SYS_MMC_ENV_DEV)"\0"\
	"mmc_bootloader_partition="__stringify(CONF_SYS_BL_PART)"\0" \
	"partfdtandroid=8\0"			    \
	"ethspeed=100M\0"			    \
	"mmcroot=" CONFIG_MMCROOT " rootwait rw\0"  \
	"mmcpartext4=1\0"                           \
	"mmcautodetect=yes\0"                       \
    "bootsector=66\0"                           \
	"brickme_sd=mmc erase $bootsector 10\0"     \
    "brickme_mmc=mmc partconf 0 1 0 1 ; mmc erase $bootsector 10 ;mmc partconf 0 1 1 1 ; mmc erase $bootsector 10 ;mmc partconf 0 0 0 0 ; mmc erase $bootsector 10 \0" \
	"updateusb=1\0" \
	"updatesd1=1\0" \
	"loadupdatescriptfatusb=if test -n ${updateusb} = 1; then if usb start; then fatload usb 0 ${script_addr} autoboot.bat; fi; fi;\0 "            \
	"loadupdatescriptfatsd=if test -n ${updatesd1} = 1; then fatload mmc 1 ${script_addr} autoboot.bat; fi;\0"            \
	"update_bl=mmc partconf 0 0 0 0 ; ums 0 mmc 0 \0"	                                    \
    "serial_download=i2c dev 2; i2c mw 0x10 2.1 2 \0"                                           \
    "serial_download2=mw 0x30390098 0x40000000; reset \0"                                           \
	"mmcargs=setenv bootargs ${jh_clk} console=${console} root=${mmcroot}\0 "                   \
	"loadbootscriptext4=ext4load mmc ${mmcdev}:${mmcpartext4} ${script_addr} ${script};\0"      \
	"loadbootscriptfat=fatload mmc ${mmcdev}:${mmcpart} ${script_addr} ${script};\0"            \
	"loadbootscript   =fatload mmc ${mmcdev}:${mmcpart} ${script_addr} ${script};\0"            \
	"loadbootscriptfatandroid=fatload mmc ${mmcdev}:${partfdtandroid} ${script_addr} ${script};\0" \
	"loadbootscriptfatusb=fatload usb ${usbdev}:${usbpart} ${script_addr} ${script};\0"         \
	"bootscript=echo Running bootscript...; source ${script_addr}\0"                            \
	"loadimageext4=ext4load mmc ${mmcdev}:${mmcpartext4} ${loadaddr} ${image}\0"                \
	"loadimagefat=fatload mmc ${mmcdev}:${mmcpart} ${loadaddr} ${image}\0"                      \
	"loadandroid=boota  mmc${mmcdev}\0"                                                         \
	"loadfdtfat=fatload mmc ${mmcdev}:${mmcpart} ${fdt_addr} ${fdt_file}\0"                     \
	"loadfdtext4=ext4load mmc ${mmcdev}:${mmcpartext4} ${fdt_addr} ${fdt_file}\0"               \
	"loadfdtandroid=fatload mmc ${mmcdev}:${partfdtandroid} ${fdt_addr} ${fdt_file}\0"          \
	"loadfdt=echo loading fdt..;"                                                               \
    "if run loadfdtext4; then echo loadtdext4 ok; else echo trying from fat....; "          \
    "if run loadfdtfat;  then echo loadftdfat ok; fi; fi;\0"                                \
	"loadbootiot=echo Try Booting IoT Core ...;  	part list mmc ${mmcdev} devplist; for bootpar in ${devplist}; do part type mmc ${mmcdev}:${bootpar} part_type; if test -n ${part_type} && test ${part_type} = 1d30adf8-0aef-4d83-b78c-ac719086c709; then if read mmc ${mmcdev}:${bootpar} 0x40800000 0 1000; then usb start; bootm 0x40800000; fi; fi; done;\0"  \
	"loadbootenterprise=echo Try Booting IoT...;  	part list mmc ${mmcdev} devplist; for bootpar in ${devplist}; do part type mmc ${mmcdev}:${bootpar} part_type; if test -n ${part_type} && test ${part_type} = c12a7328-f81f-11d2-ba4b-00a0c93ec93b; then if mmc dev ${mmcdev} ${emmcbootpart}; mmc read 0x40800000 0xBFA 1000; then usb start; mmc dev ${mmcdev} 1; bootm 0x40800000; fi; fi; done;\0"  \
	"loadinstallenterprise=echo Try Install IoT...;if mmc dev 1; then part list mmc 1 devplist; for bootpar in ${devplist}; do part type mmc 1:${bootpar} part_type; if test -n ${part_type} && test ${part_type} = c12a7328-f81f-11d2-ba4b-00a0c93ec93b; then if mmc dev ${mmcdev} ${emmcbootpart}; mmc read 0x40800000 0xBFA 1000; then usb start; mmc dev ${mmcdev} 1; bootm 0x40800000; fi; fi; done; else echo no sd card found; fi;\0" \
	"mmcboot=echo Booting from mmc ...; " \
		"run mmcargs; " \
		"if test ${boot_fdt} = yes || test ${boot_fdt} = try; then " \
			"if run loadfdt; then " \
				"booti ${loadaddr} - ${fdt_addr}; " \
			"else " \
				"echo WARN: Cannot load the DT; " \
			"fi; " \
		"else " \
			"echo wait for boot; " \
		"fi;\0" \
	"netargs=setenv bootargs ${jh_clk} console=${console} " \
		"root=/dev/nfs " \
		"ip=dhcp nfsroot=${serverip}:${nfsroot},v3,tcp\0" \
	"netboot=echo Booting from net ...; " \
		"run netargs;  " \
		"if test ${ip_dyn} = yes; then " \
			"setenv get_cmd dhcp; " \
		"else " \
			"setenv get_cmd tftp; " \
		"fi; " \
		"${get_cmd} ${loadaddr} ${image}; " \
		"if test ${boot_fdt} = yes || test ${boot_fdt} = try; then " \
			"if ${get_cmd} ${fdt_addr} ${fdt_file}; then " \
				"booti ${loadaddr} - ${fdt_addr}; " \
			"else " \
				"echo WARN: Cannot load the DT; " \
			"fi; " \
		"else " \
			"booti; " \
		"fi;\0"

#ifndef CONFIG_BOOTCOMMAND
#define CONFIG_BOOTCOMMAND \
		"mmc dev ${mmcdev};mmc partconf ${mmcdev}; "\
		"if run loadupdatescriptfatusb; then run bootscript; " \
		"else if run loadupdatescriptfatsd; then run bootscript; " \
		"fi; " \
		"fi; " \
		"if mmc rescan; "\
        "then "\
		"run loadbootenterprise; " \
		"run loadinstallenterprise; " \
		"run loadbootiot; " \
		"usb stop; " \
	     "if              run loadbootscriptext4;       then run bootscript; "    \
	     "else if         run loadbootscriptfat;        then run bootscript; "    \
	       "else if       run loadbootscriptfatandroid; then run bootscript; "    \
	         "else if     run loadimageext4;            then run mmcboot; "       \
                   "else if   run loadimagefat;             then run mmcboot; "       \
                     "else    mw.b $fdt_addr 0 0x40;        run loadfdtandroid; "     \
	                  "if  run loadandroid; then  ;     else run loadandroid; "  \
                          "fi; "\
		     "fi; "    \
		   "fi; "      \
		 "fi; "        \
	       "fi; "          \
	     "fi; "	       \
	"else booti ${loadaddr} - ${fdt_addr}; fi "
#endif
#endif

/* Link Definitions */
#define CONFIG_LOADADDR			0x40480000

#define CONFIG_SYS_LOAD_ADDR           CONFIG_LOADADDR

#define CONFIG_SYS_INIT_RAM_ADDR        0x40000000
#define CONFIG_SYS_INIT_RAM_SIZE        0x00080000
#define CONFIG_SYS_INIT_SP_OFFSET \
        (CONFIG_SYS_INIT_RAM_SIZE - GENERATED_GBL_DATA_SIZE)
#define CONFIG_SYS_INIT_SP_ADDR \
        (CONFIG_SYS_INIT_RAM_ADDR + CONFIG_SYS_INIT_SP_OFFSET)

#define CONFIG_ENV_OVERWRITE
#if defined(CONFIG_ENV_IS_IN_MMC)
#define CONFIG_ENV_OFFSET               (64 * SZ_64K)
#elif defined(CONFIG_ENV_IS_IN_SPI_FLASH)
#define CONFIG_ENV_OFFSET		(4 * 1024 * 1024)
#define CONFIG_ENV_SECT_SIZE		(64 * 1024)
#define CONFIG_ENV_SPI_BUS		CONFIG_SF_DEFAULT_BUS
#define CONFIG_ENV_SPI_CS		CONFIG_SF_DEFAULT_CS
#define CONFIG_ENV_SPI_MODE		CONFIG_SF_DEFAULT_MODE
#define CONFIG_ENV_SPI_MAX_HZ		CONFIG_SF_DEFAULT_SPEED
#elif defined(CONFIG_ENV_IS_IN_NAND)
#define CONFIG_ENV_OFFSET       (60 << 20)
#endif
#define CONFIG_ENV_SIZE			0x2000
#ifdef CONFIG_BOOT_EXTSDCARD
#define CONFIG_SYS_MMC_ENV_DEV		1                 
#define CONFIG_MMCROOT			"/dev/mmcblk1p2"  
#else
#ifdef BOOT_SDCARD3
#define CONFIG_SYS_MMC_ENV_DEV		2                 
#define CONFIG_MMCROOT			"/dev/mmcblk1p3"  
#else
#define CONFIG_SYS_MMC_ENV_DEV		0                 /* USDHC2 */
#define CONFIG_MMCROOT			"/dev/mmcblk0p2"  /* USDHC2 */
#endif
#endif

/* Size of malloc() pool */
#define CONFIG_SYS_MALLOC_LEN		((CONFIG_ENV_SIZE + (2*1024) + (16*1024)) * 1024)

#define CONFIG_SYS_SDRAM_BASE           0x40000000
#define PHYS_SDRAM                      0x40000000
#define PHYS_SDRAM_SIZE				0x100000000 /* 4GB DDR */

#define CONFIG_SYS_MEMTEST_START    PHYS_SDRAM
#define CONFIG_SYS_MEMTEST_END      (CONFIG_SYS_MEMTEST_START + (PHYS_SDRAM_SIZE >> 1))

#define CONFIG_BAUDRATE			115200

#define CONFIG_MXC_UART
#define CONFIG_MXC_UART_BASE		UART1_BASE_ADDR

/* Monitor Command Prompt */
#undef CONFIG_SYS_PROMPT
#define CONFIG_SYS_PROMPT		"u-boot=> "
#define CONFIG_SYS_PROMPT_HUSH_PS2     "> "
#define CONFIG_SYS_CBSIZE              2048
#define CONFIG_SYS_MAXARGS             64
#define CONFIG_SYS_BARGSIZE CONFIG_SYS_CBSIZE
#define CONFIG_SYS_PBSIZE		(CONFIG_SYS_CBSIZE + \
					sizeof(CONFIG_SYS_PROMPT) + 16)

#define CONFIG_IMX_BOOTAUX

/* USDHC */
#define CONFIG_CMD_MMC
#define CONFIG_FSL_ESDHC
#define CONFIG_FSL_USDHC

#ifdef CONFIG_SBCSOM
#define CONFIG_SYS_FSL_USDHC_NUM	3
#else
#ifdef CONFIG_TANARO
#define CONFIG_SYS_FSL_USDHC_NUM	3
#else
#define CONFIG_SYS_FSL_USDHC_NUM	2
#endif
#endif

#define CONFIG_SYS_FSL_ESDHC_ADDR       0

#define CONFIG_SUPPORT_EMMC_BOOT	/* eMMC specific */
#define CONFIG_SYS_MMC_IMG_LOAD_PART	1

#ifdef CONFIG_FSL_FSPI
#define FSL_FSPI_FLASH_SIZE		SZ_32M
#define FSL_FSPI_FLASH_NUM		1
#define FSPI0_BASE_ADDR			0x30bb0000
#define FSPI0_AMBA_BASE			0x0
#define CONFIG_FSPI_QUAD_SUPPORT

#define CONFIG_SYS_FSL_FSPI_AHB
#endif

#ifdef CONFIG_CMD_NAND
#define CONFIG_NAND_MXS
#define CONFIG_CMD_NAND_TRIMFFS

/* NAND stuff */
#define CONFIG_SYS_MAX_NAND_DEVICE     1
#define CONFIG_SYS_NAND_BASE           0x20000000
#define CONFIG_SYS_NAND_5_ADDR_CYCLE
#define CONFIG_SYS_NAND_ONFI_DETECTION

/* DMA stuff, needed for GPMI/MXS NAND support */
#define CONFIG_APBH_DMA
#define CONFIG_APBH_DMA_BURST
#define CONFIG_APBH_DMA_BURST8

#ifdef CONFIG_CMD_UBI
#define CONFIG_MTD_PARTITIONS
#define CONFIG_MTD_DEVICE
#endif
#endif /* CONFIG_CMD_NAND */


#define CONFIG_MXC_GPIO

#define CONFIG_MXC_OCOTP
#define CONFIG_CMD_FUSE

#ifndef CONFIG_DM_I2C
#define CONFIG_SYS_I2C
#endif
#define CONFIG_SYS_I2C_MXC_I2C1		/* enable I2C bus 1 */
#define CONFIG_SYS_I2C_MXC_I2C2		/* enable I2C bus 2 */
#define CONFIG_SYS_I2C_MXC_I2C3		/* enable I2C bus 3 */
#define CONFIG_SYS_I2C_SPEED		100000

/* USB configs */
#ifndef CONFIG_SPL_BUILD
#define CONFIG_CMD_USB
#define CONFIG_USB_STORAGE

#define CONFIG_CMD_USB_MASS_STORAGE
#define CONFIG_USB_GADGET_MASS_STORAGE
#define CONFIG_USB_FUNCTION_MASS_STORAGE

#endif

#define CONFIG_CI_UDC
#define CONFIG_USB_GADGET_DUALSPEED
#define CONFIG_USBD_HS
#define CONFIG_USB_GADGET_VBUS_DRAW 2
#define CONFIG_USB_MAX_CONTROLLER_COUNT         2


#define CONFIG_MXC_USB_PORTSC  (PORT_PTS_UTMI | PORT_PTS_PTW)

#ifdef CONFIG_VIDEO
#define CONFIG_VIDEO_MXS
#define CONFIG_VIDEO_LOGO
#define CONFIG_SPLASH_SCREEN
#define CONFIG_SPLASH_SCREEN_ALIGN
#define CONFIG_CMD_BMP
#define CONFIG_BMP_16BPP
#define CONFIG_VIDEO_BMP_RLE8
#define CONFIG_VIDEO_BMP_LOGO
#define CONFIG_IMX_VIDEO_SKIP
#endif

#define CONFIG_OF_SYSTEM_SETUP
#define IS_TRIZEPS8_MINI

/* Trizeps8Mini Custom Firmware Version Commands */
#define CONFIG_CMD_FPGA_REV
#define CONFIG_CMD_MCU_REV
/*************************************************************************************************************/
/* Modify config for Android                                                                                 */
/*************************************************************************************************************/
#if defined(CONFIG_ANDROID_SUPPORT)
#define IMX8MM_EVK_ANDROID_H

#define CONFIG_BCB_SUPPORT

/*#define CONFIG_FASTBOOT*/
#define CONFIG_ANDROID_AB_SUPPORT
#define CONFIG_AVB_SUPPORT
#define CONFIG_SUPPORT_EMMC_RPMB
#define CONFIG_SYSTEM_RAMDISK_SUPPORT
#define CONFIG_AVB_FUSE_BANK_SIZEW 0
#define CONFIG_AVB_FUSE_BANK_START 0
#define CONFIG_AVB_FUSE_BANK_END 0
#define CONFIG_FASTBOOT_LOCK
#define FSL_FASTBOOT_FB_DEV "mmc"

#ifdef CONFIG_SYS_MALLOC_LEN
#undef CONFIG_SYS_MALLOC_LEN
#define CONFIG_SYS_MALLOC_LEN           (96 * SZ_1M)
#endif

#ifndef CONFIG_USB_FUNCTION_FASTBOOT
#define CONFIG_USB_FUNCTION_FASTBOOT
#endif

#ifndef CONFIG_CMD_FASTBOOT
#define CONFIG_CMD_FASTBOOT
#endif

#ifndef CONFIG_ANDROID_BOOT_IMAGE
#define CONFIG_ANDROID_BOOT_IMAGE
#endif

#ifndef CONFIG_FASTBOOT_FLASH
#define CONFIG_FASTBOOT_FLASH
#endif

#ifndef CONFIG_FSL_FASTBOOT
#define CONFIG_FSL_FASTBOOT
#endif

#define CONFIG_ANDROID_RECOVERY

#define CONFIG_CMD_BOOTA
#ifndef CONFIG_SUPPORT_RAW_INITRD
#define CONFIG_SUPPORT_RAW_INITRD
#endif
#define CONFIG_SERIAL_TAG


/* Enable mcu firmware flash */
#ifdef CONFIG_FLASH_MCUFIRMWARE_SUPPORT
#define ANDROID_MCU_FRIMWARE_DEV_TYPE DEV_MMC
#define ANDROID_MCU_FIRMWARE_START 0x500000
#define ANDROID_MCU_FIRMWARE_SIZE  0x40000
#define ANDROID_MCU_FIRMWARE_HEADER_STACK 0x20020000
#endif

#ifdef CONFIG_FSL_CAAM_KB
#undef CONFIG_FSL_CAAM_KB
#endif
#define AVB_AB_I_UNDERSTAND_LIBAVB_AB_IS_DEPRECATED

#ifdef CONFIG_IMX_TRUSTY_OS
#define AVB_RPMB
#define KEYSLOT_HWPARTITION_ID 2
#define KEYSLOT_BLKS             0x1FFF
#define NS_ARCH_ARM64 1

#ifdef CONFIG_SPL_BUILD
#undef CONFIG_BLK
#endif
#endif

#endif
#endif
