/*
 * Copyright 2018-2019 NXP
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#include <common.h>
#include <spl.h>
#include <asm/io.h>
#include <errno.h>
#include <asm/io.h>
#include <asm/mach-imx/iomux-v3.h>
#include <asm/arch/imx8mm_pins.h>
#include <asm/arch/sys_proto.h>
#include <power/pmic.h>
#include <power/bd71837.h>
#include <asm/arch/clock.h>
#include <asm/mach-imx/gpio.h>
#include <asm/mach-imx/mxc_i2c.h>
#include <fsl_esdhc.h>
#include <mmc.h>
#include <asm/arch/ddr.h>

DECLARE_GLOBAL_DATA_PTR;

#include "../common/kuk_boards.h"

extern struct dram_timing_info dram_timing_v1r1;
extern struct dram_timing_info dram_timing_v1r2_2GB_K4F6E304HB;
extern struct dram_timing_info dram_timing_v1r2_2GB_K4F6E3S4HM;
extern struct dram_timing_info dram_timing_v1r2_4GB_K4FBE3D4HM;

#ifdef LP_RAM_SETTING
extern struct dram_timing_info dram_timing_v1r2_2GB_K4F6E304HB_LP;
extern struct dram_timing_info dram_timing_v1r2_2GB_K4F6E3S4HM_LP;
#endif

void spl_dram_init(void)
{
	int module;
	int version;
	int ramsize;
	int ramskew;

	module	= kuk_GetModule();
	version	= kuk_GetPCBrevision();
	ramsize = kuk_GetRAMSize();
	ramskew = kuk_GetRAMSkew();

	if (( module == KUK_MODULE_TRIZEPS8MINI)&&( version == KUK_PCBREV_V1R1))
	{
		ddr_init(&dram_timing_v1r1);	// 2GB RAM, 32bit LPDDR4, CH A/B <=> CH B/A
	}else
	{
		switch( ramsize)
		{
			case KUK_RAMSIZE_2GB:
				if ( ramskew == 0)
				{	// Dual-Die 2ch with 2cs
#ifdef LP_RAM_SETTING
					printf("Choose dram_timing_v1r2_2GB_K4F6E304HB_LP \r\n");
					ddr_init(&dram_timing_v1r2_2GB_K4F6E304HB_LP);
#else
					printf("Choose dram_timing_v1r2_2GB_K4F6E304HB \r\n");
					ddr_init(&dram_timing_v1r2_2GB_K4F6E304HB);	// 2GB RAM, 32bit LPDDR4, CH A/B <=> CH A/B
#endif
				}else
				{	// Mono-Die 2ch with 1cs
#ifdef LP_RAM_SETTING
					printf("Choose dram_timing_v1r2_2GB_K4F6E3S4HM_LP\r\n");
					ddr_init(&dram_timing_v1r2_2GB_K4F6E3S4HM_LP);	
					
#else
					printf("Choose dram_timing_v1r2_2GB_K4F6E3S4HM\r\n");
					ddr_init(&dram_timing_v1r2_2GB_K4F6E3S4HM);	// 2GB RAM, 32bit LPDDR4, CH A/B <=> CH A/B
#endif
								
				}												
				break;
			case KUK_RAMSIZE_4GB:	
				printf("Choose dram_timing_v1r2_4GB_K4FBE3D4HM\r\n");			
				ddr_init(&dram_timing_v1r2_4GB_K4FBE3D4HM);	// 4GB RAM, 32bit LPDDR4, CH A/B <=> CH A/B
				break;
			default:
				printf("Choose dram_timing_v1r2_2GB_K4F6E304HB\r\n");
				ddr_init(&dram_timing_v1r2_2GB_K4F6E304HB);	// 2GB RAM, 32bit LPDDR4, CH A/B <=> CH A/B
				break;
		}
	}	
}

#define I2C_PAD_CTRL	(PAD_CTL_DSE6 | PAD_CTL_HYS | PAD_CTL_PUE | PAD_CTL_PE)
#define PC MUX_PAD_CTRL(I2C_PAD_CTRL)
struct i2c_pads_info i2c_pad_info3 = {
	.scl = {
		.i2c_mode = IMX8MM_PAD_I2C3_SCL_I2C3_SCL | PC,
		.gpio_mode = IMX8MM_PAD_I2C3_SCL_GPIO5_IO18 | PC,
		.gp = IMX_GPIO_NR(5, 18),
	},
	.sda = {
		.i2c_mode = IMX8MM_PAD_I2C3_SDA_I2C3_SDA | PC,
		.gpio_mode = IMX8MM_PAD_I2C3_SDA_GPIO5_IO19 | PC,
		.gp = IMX_GPIO_NR(5, 19),
	},
};

#define USDHC2_CD_GPIO	IMX_GPIO_NR(2, 18)
#define USDHC2_PWR_GPIO IMX_GPIO_NR(2, 19)

#define USDHC_PAD_CTRL	(PAD_CTL_DSE6 | PAD_CTL_HYS | PAD_CTL_PUE |PAD_CTL_PE | \
			 PAD_CTL_FSEL2)
#define USDHC_GPIO_PAD_CTRL (PAD_CTL_HYS | PAD_CTL_DSE1)

static iomux_v3_cfg_t const usdhc1_pads[] = {
	IMX8MM_PAD_SD1_CLK_USDHC1_CLK | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	IMX8MM_PAD_SD1_CMD_USDHC1_CMD | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	IMX8MM_PAD_SD1_DATA0_USDHC1_DATA0 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	IMX8MM_PAD_SD1_DATA1_USDHC1_DATA1 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	IMX8MM_PAD_SD1_DATA2_USDHC1_DATA2 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	IMX8MM_PAD_SD1_DATA3_USDHC1_DATA3 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	IMX8MM_PAD_SD1_DATA4_USDHC1_DATA4 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	IMX8MM_PAD_SD1_DATA5_USDHC1_DATA5 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	IMX8MM_PAD_SD1_DATA6_USDHC1_DATA6 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	IMX8MM_PAD_SD1_DATA7_USDHC1_DATA7 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
};

static iomux_v3_cfg_t const usdhc2_pads[] = {
	IMX8MM_PAD_SD2_CLK_USDHC2_CLK | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	IMX8MM_PAD_SD2_CMD_USDHC2_CMD | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	IMX8MM_PAD_SD2_DATA0_USDHC2_DATA0 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	IMX8MM_PAD_SD2_DATA1_USDHC2_DATA1 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	IMX8MM_PAD_SD2_DATA2_USDHC2_DATA2 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	IMX8MM_PAD_SD2_DATA3_USDHC2_DATA3 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
};

#define RST_PAD IMX_GPIO_NR(3, 14)
static iomux_v3_cfg_t const reset_out_pads[] = {
	IMX8MM_PAD_NAND_DQS_GPIO3_IO14 | MUX_PAD_CTRL((PAD_CTL_HYS | PAD_CTL_DSE1)) 
};

/*
 * The evk board uses DAT3 to detect CD card plugin,
 * in u-boot we mux the pin to GPIO when doing board_mmc_getcd.
 */


static struct fsl_esdhc_cfg usdhc_cfg[2] = {
	{USDHC1_BASE_ADDR, 0, 8},
	{USDHC2_BASE_ADDR, 0, 4},
};

int board_mmc_init(bd_t *bis)
{
	int i, ret;
	/*
	 * According to the board_mmc_init() the following map is done:
	 * (U-Boot device node)    (Physical Port)
	 * mmc0                    USDHC1
	 * mmc1                    USDHC2
	 */
	for (i = 0; i < CONFIG_SYS_FSL_USDHC_NUM; i++) {
		switch (i) {
		case 0:
			init_clk_usdhc(0);
			usdhc_cfg[0].sdhc_clk = mxc_get_clock(MXC_ESDHC_CLK);
			imx_iomux_v3_setup_multiple_pads(
				usdhc1_pads, ARRAY_SIZE(usdhc1_pads));
			break;
		case 1:
			init_clk_usdhc(1);
			usdhc_cfg[1].sdhc_clk = mxc_get_clock(MXC_ESDHC2_CLK);
			imx_iomux_v3_setup_multiple_pads(
				usdhc2_pads, ARRAY_SIZE(usdhc2_pads));
			break;
		default:
			printf("Warning: you configured more USDHC controllers"
				"(%d) than supported by the board\n", i + 1);
			return -EINVAL;
		}

		ret = fsl_esdhc_initialize(bis, &usdhc_cfg[i]);
		if (ret)
			return ret;
	}

	return 0;
}

int board_mmc_getcd(struct mmc *mmc)
{
	struct fsl_esdhc_cfg *cfg = (struct fsl_esdhc_cfg *)mmc->priv;
	int ret = 0;

	switch (cfg->esdhc_base) {
	case USDHC1_BASE_ADDR:
		ret = 1;
		break;
	case USDHC2_BASE_ADDR:
		ret = 1;
		return ret;
	}

	return 1;
}

#ifdef CONFIG_POWER
#define I2C_PMIC	2	/* I2C3 */
int power_init_board(void)
{
	struct pmic *p;
	int ret;
	int module;
	//int version;
	int iovolt;
	//unsigned int temp;
	module	= kuk_GetModule();
	//version = kuk_GetPCBrevision(); 
	iovolt	= kuk_GetPeripheral( KUK_OTP_IOVOLTAGE);

	ret = power_bd71837_init(I2C_PMIC);
	if (ret)
		printf("power init failed");

	p = pmic_get("BD71837");
	pmic_probe(p);

	/* decrease RESET key long push time from the default 10s to 10ms */
	pmic_reg_write(p, BD71837_PWRONCONFIG1, 0x0);

	/* unlock the PMIC regs */
	pmic_reg_write(p, BD71837_REGLOCK, 0x1);

	/* increase VDD_SOC to typical value 0.85v before first DRAM access */
	pmic_reg_write(p, BD71837_BUCK1_VOLT_RUN, 0x0f);

#ifdef LP_RAM_SETTING
	pmic_reg_write(p, BD71837_BUCK5_VOLT, 0x12);
#else
	/* increase VDD_DRAM to 0.975v for 3Ghz DDR */
	pmic_reg_write(p, BD71837_BUCK5_VOLT, 0x83);
#endif
	


#ifndef CONFIG_IMX8M_LPDDR4
	/* increase NVCC_DRAM_1V2 to 1.2v for DDR4 */
	pmic_reg_write(p, BD71837_BUCK8_VOLT, 0x28);
#endif


	if ( module == KUK_MODULE_TRIZEPS8MINI)
	{	// Trizeps
		/* Set VDD_FPGA_MIPI to 2.5V */
		pmic_reg_write(p, BD71837_LDO5_VOLT, 0xC7);

		//if ( version == KUK_PCBREV_V1R1) need to check if still needed for V1R2; keep this line uncommented until verified
		{	
			/* Set FPGA-Core-Voltage to 1.3V (default 1.2V) */
			pmic_reg_write(p, BD71837_LDO6_VOLT, 0xC4);
		}
	}else
	{	// Myon
		switch( iovolt)
		{			
			case KUK_IOVOLTAGE_3V3:
				pmic_reg_write(p, BD71837_LDO5_VOLT, 0xCF);	// 3.3V
				break;
			case KUK_IOVOLTAGE_CUSTOM:
				pmic_reg_write(p, BD71837_LDO5_VOLT, 0xC0);	// 1.8V: may be modified by customer
				break;
			case KUK_IOVOLTAGE_1V8:
			default:
				pmic_reg_write(p, BD71837_LDO5_VOLT, 0xC0);	// 1.8V
				break;
		}
	}

	/* lock the PMIC regs */
	pmic_reg_write(p, BD71837_REGLOCK, 0x11);

	if (( module == KUK_MODULE_MYON2)||( module == KUK_MODULE_MYON2NANO))
	{
		/* Set Reset-out */
		imx_iomux_v3_setup_multiple_pads(
					reset_out_pads, ARRAY_SIZE(reset_out_pads));
		gpio_request(RST_PAD, "reset_out");
		gpio_direction_output(RST_PAD, 1);
	}
	return 0;
}
#endif

void spl_board_init(void)
{
#ifndef CONFIG_SPL_USB_SDP_SUPPORT
	/* Serial download mode */
	if (is_usb_boot()) {
		puts("Back to ROM, SDP\n");
		restore_boot_params();
	}
#endif
	puts("Normal Boot\n");
}

#ifdef CONFIG_SPL_LOAD_FIT
int board_fit_config_name_match(const char *name)
{
	/* Just empty function now - can't decide what to choose */
	debug("%s: %s\n", __func__, name);

	return 0;
}
#endif

void board_init_f(ulong dummy)
{
	int ret;

	/* Clear the BSS. */
	memset(__bss_start, 0, __bss_end - __bss_start);

	arch_cpu_init();

	board_early_init_f();

	timer_init();

	preloader_console_init();

	ret = spl_init();
	if (ret) {
		debug("spl_init() failed: %d\n", ret);
		hang();
	}

	enable_tzc380();

	/* Adjust pmic voltage to 1.0V for 800M */
	setup_i2c(2, CONFIG_SYS_I2C_SPEED, 0x7f, &i2c_pad_info3);

	power_init_board();

	/* DDR initialization */
	spl_dram_init();

	board_init_r(NULL, 0);
}
