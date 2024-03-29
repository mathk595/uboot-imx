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
#ifdef CONFIG_FSL_ESDHC_IMX
#include <fsl_esdhc_imx.h>
#else
#include <fsl_esdhc.h>
#endif
#include <mmc.h>
#include <asm/arch/ddr.h>

DECLARE_GLOBAL_DATA_PTR;

#include "../common/kuk_boards.h"

extern struct dram_timing_info dram_timing_v1r1;
extern struct dram_timing_info dram_timing_v1r2_1GB_K4F8E304HB;
extern struct dram_timing_info dram_timing_v1r2_1GB_K4F8E3S4HD;
extern struct dram_timing_info dram_timing_v1r2_2GB_K4F6E304HB;
extern struct dram_timing_info dram_timing_v1r2_2GB_K4F6E3S4HM;
extern struct dram_timing_info dram_timing_v1r2_4GB_K4FBE3D4HM;

//#define LP_RAM_SETTING 1

#ifdef LP_RAM_SETTING
extern struct dram_timing_info dram_timing_v1r2_2GB_K4F6E304HB_LP;
extern struct dram_timing_info dram_timing_v1r2_2GB_K4F6E3S4HM_LP;
extern struct dram_timing_info dram_timing_MX8M_Mini_LPDDR4_RPA_v18_800MHz_4GByte_822r16c10_v1;
#endif

void spl_dram_init(void)
{
	int module;
	int version;
	int ramsize;
	int ramskew;
	int versno=1;	

	module	= kuk_GetModule();
	version	= kuk_GetPCBrevision();
	ramsize = kuk_GetRAMSize();
	ramskew = kuk_GetRAMSkew();

	if (( module == KUK_MODULE_TRIZEPS8MINI)&&( version == KUK_PCBREV_V1R1))
	{
		ddr_init(&dram_timing_v1r1);	// 2GB RAM, 32bit LPDDR4, CH A/B <=> CH B/A
	}else
	{
	        versno=version+1;	  
		switch( ramsize)
		{
		  
		case KUK_RAMSIZE_1GB:
		     if ( ramskew == 1 || module == GUF_MODULE_TANARO)
		     {
		       printf("Choose dram_timing_v1r%d_1GB_K4F8E3S4HD\r\n", versno);			
		       ddr_init(&dram_timing_v1r2_1GB_K4F8E3S4HD);	// 1GB RAM, 32bit LPDDR4, CH A/B <=> CH A/B                    
		     }else
		     {   // Need to check if K4F8E3S4HD from Tanaro may be used for Trizeps VIII Mini
		       printf("Choose dram_timing_v1r%d_1GB_K4F8E304HB\r\n", versno);			
		       ddr_init(&dram_timing_v1r2_1GB_K4F8E304HB);	// 1GB RAM, 32bit LPDDR4, CH A/B <=> CH A/B
		     }
		     break;
		     
		case KUK_RAMSIZE_2GB:
		  if ( ramskew == 0)
		    {	// Dual-Die 2ch with 2cs
#ifdef LP_RAM_SETTING
		      printf("Choose dram_timing_v1r%d_2GB_K4F6E304HB_LP \r\n", versno);
		      ddr_init(&dram_timing_v1r2_2GB_K4F6E304HB_LP);
#else	
		      printf("Choose dram_timing_v1r%d_2GB_K4F6E304HB \r\n", versno);
		      ddr_init(&dram_timing_v1r2_2GB_K4F6E304HB);	// 2GB RAM, 32bit LPDDR4, CH A/B <=> CH A/B
#endif
		    }else
		    {	// Mono-Die 2ch with 1cs
#ifdef LP_RAM_SETTING
		      printf("Choose dram_timing_v1r%d_2GB_K4F6E3S4HM_LP\r\n", versno);
		      ddr_init(&dram_timing_v1r2_2GB_K4F6E3S4HM_LP);	
		      
#else
		      printf("Choose dram_timing_v1r%d_2GB_K4F6E3S4HM\r\n", versno);
		      ddr_init(&dram_timing_v1r2_2GB_K4F6E3S4HM);	// 2GB RAM, 32bit LPDDR4, CH A/B <=> CH A/B
#endif
								
		    }												
		    break;
		case KUK_RAMSIZE_4GB:	
		    printf("Choose dram_timing_v1r%d_4GB_K4FBE3D4HM\r\n", versno);			
		    ddr_init(&dram_timing_v1r2_4GB_K4FBE3D4HM);	// 4GB RAM, 32bit LPDDR4, CH A/B <=> CH A/B
		    break;
		default:
		    printf("Choose dram_timing_v1r%d_2GB_K4F6E304HB\r\n", versno);
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

static iomux_v3_cfg_t const usdhc3_pads[] = {
	IMX8MM_PAD_NAND_WE_B_USDHC3_CLK | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	IMX8MM_PAD_NAND_WP_B_USDHC3_CMD | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	IMX8MM_PAD_NAND_DATA04_USDHC3_DATA0 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	IMX8MM_PAD_NAND_DATA05_USDHC3_DATA1 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	IMX8MM_PAD_NAND_DATA06_USDHC3_DATA2 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	IMX8MM_PAD_NAND_DATA07_USDHC3_DATA3 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
};


#define RST_PAD IMX_GPIO_NR(3, 14)
static iomux_v3_cfg_t const reset_out_pads[] = {
	IMX8MM_PAD_NAND_DQS_GPIO3_IO14 | MUX_PAD_CTRL((PAD_CTL_HYS | PAD_CTL_DSE1)) 
};

#define SDHC1_VOLTAGE_SELECT_GPIO   IMX_GPIO_NR(3,17)
#define GPIO_PAD_CTRL    (PAD_CTL_DSE6 | PAD_CTL_FSEL1 )
#define GPIO_PAD_PU_CTRL (PAD_CTL_DSE6 | PAD_CTL_FSEL1 | PAD_CTL_PUE | PAD_CTL_PE)
#define GPIO_PAD_PD_CTRL (PAD_CTL_DSE6 | PAD_CTL_FSEL1 |               PAD_CTL_PE)

static iomux_v3_cfg_t const sdhc1_vselect_pads[] = {
	IMX8MM_PAD_NAND_WE_B_SION_GPIO3_IO17 | MUX_PAD_CTRL( GPIO_PAD_PD_CTRL ), // SELECT SDHC1 VOLTAGE
};


/*
 * The evk board uses DAT3 to detect CD card plugin,
 * in u-boot we mux the pin to GPIO when doing board_mmc_getcd.
 */


static struct fsl_esdhc_cfg usdhc_cfg[3] = {
	{USDHC1_BASE_ADDR, 0, 8},
	{USDHC2_BASE_ADDR, 0, 4},
	{USDHC3_BASE_ADDR, 0, 4},
};


int board_mmc_init(bd_t *bis)
{
	int i, ret;
	int module, version;
	/*
	 * According to the board_mmc_init() the following map is done:
	 * (U-Boot device node)    (Physical Port)
	 * mmc0                    USDHC1
	 * mmc1                    USDHC2
	 * mmc2                    USDHC3   (only on SBCSOM)
	 */
	for (i = 0; i < CONFIG_SYS_FSL_USDHC_NUM; i++)
	{
		switch (i) {
		case 0:
			init_clk_usdhc(0);
			usdhc_cfg[0].sdhc_clk = mxc_get_clock(MXC_ESDHC_CLK);
			imx_iomux_v3_setup_multiple_pads(usdhc1_pads, ARRAY_SIZE(usdhc1_pads));
			module	= kuk_GetModule();
			if( (module == KUK_MODULE_TRIZEPS8MINI) || (module == KUK_MODULE_TRIZEPS8NANO) )
			{
			  if( (version=kuk_GetPCBrevision()) >= KUK_PCBREV_V1R3 )
			  {
			    printf("board_mmc_init USDHC@0x%lx init voltsel\n\r",(unsigned long)usdhc_cfg[i].esdhc_base);			    
			    imx_iomux_v3_setup_multiple_pads(sdhc1_vselect_pads, 1);	
			    gpio_request(SDHC1_VOLTAGE_SELECT_GPIO, "SDHC1_VOLTAGE_SELECT");	
			    gpio_direction_output(SDHC1_VOLTAGE_SELECT_GPIO, 0);
			    gpio_free(SDHC1_VOLTAGE_SELECT_GPIO);			    			    
			  }
			}			
			break;
		case 1:
			init_clk_usdhc(1);
			usdhc_cfg[1].sdhc_clk = mxc_get_clock(MXC_ESDHC2_CLK);
			imx_iomux_v3_setup_multiple_pads(
				usdhc2_pads, ARRAY_SIZE(usdhc2_pads));
			break;
		case 2:
			init_clk_usdhc(2);
			usdhc_cfg[2].sdhc_clk = mxc_get_clock(MXC_ESDHC3_CLK);
			imx_iomux_v3_setup_multiple_pads(
				usdhc3_pads, ARRAY_SIZE(usdhc3_pads));
			break;
		default:
			printf("Warning: you configured more USDHC controllers"
				"(%d) than supported by the board\n", i + 1);
			return -EINVAL;
		}
		pr_debug("board_mmc_init USDHC%d@0x%lx\n\r",i+1,(unsigned long)usdhc_cfg[i].esdhc_base);
		ret = fsl_esdhc_initialize(bis, &usdhc_cfg[i]);
		if (ret)
			return ret;
	}

	return 0;
}

#ifdef CONFIG_POWER
#define I2C_PMIC	2	/* I2C3 */
int power_init_board(void)
{
	struct pmic *p;
	int ret;
	int module;
	int iovolt;
	//unsigned int temp;
	module	= kuk_GetModule();
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

	/* increase VDD_ARM to typical value 1.0v to support up to 1.8Ghz*/
	pmic_reg_write(p, BD71837_BUCK2_VOLT_RUN, 0x1E);

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


	if (( module == KUK_MODULE_TRIZEPS8MINI)||( module == KUK_MODULE_TRIZEPS8NANO))
	{	// Trizeps
		/* Set VDD_FPGA_MIPI to 2.5V */
		pmic_reg_write(p, BD71837_LDO5_VOLT, 0xC7);

		//if ( version == KUK_PCBREV_V1R1) need to check if still needed for V1R2; keep this line uncommented until verified
		{	
			/* Set FPGA-Core-Voltage to 1.3V (default 1.2V) */
			pmic_reg_write(p, BD71837_LDO6_VOLT, 0xC4);
		}
	}else
    if (( module == KUK_MODULE_SBCSOM8MINI)||( module == KUK_MODULE_SBCSOM8NANO))
    {   // SBCSOM
        // Nothing additional todo				
    }else
    if (( module == GUF_MODULE_TANARO))
    {   // Tanaro platform
        // Add additional init here:
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

	if (( module == KUK_MODULE_MYON2)||( module == KUK_MODULE_MYON2NANO)||
        ( module == KUK_MODULE_SBCSOM8MINI)||( module == KUK_MODULE_SBCSOM8NANO))
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

extern int intpll_configure(enum pll_clocks pll, ulong freq);

void board_init_f(ulong dummy)
{
	int ret;
	u32 max_freq;

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
	
	max_freq = get_cpu_speed_grade_hz();
	if(max_freq)
	{
		//update cpu clock
		clock_set_target_val(ARM_A53_CLK_ROOT, CLK_ROOT_ON | \
					CLK_ROOT_SOURCE_SEL(2));

		intpll_configure(ANATOP_ARM_PLL, max_freq);

		clock_set_target_val(ARM_A53_CLK_ROOT, CLK_ROOT_ON | \
					CLK_ROOT_SOURCE_SEL(1) | \
					CLK_ROOT_POST_DIV(CLK_ROOT_POST_DIV1));
	}
	board_init_r(NULL, 0);
}

#if 0
int do_reset(cmd_tbl_t *cmdtp, int flag, int argc, char * const argv[])
{
	puts ("resetting ...\n");

	reset_cpu(WDOG1_BASE_ADDR);

	return 0;
}
#endif
