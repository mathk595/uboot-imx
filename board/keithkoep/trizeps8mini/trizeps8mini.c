/*
 * Copyright 2018 NXP
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#include <common.h>
#include <malloc.h>
#include <errno.h>
#include <asm/io.h>
#include <miiphy.h>
#include <netdev.h>
#include <asm/mach-imx/iomux-v3.h>
#include <asm-generic/gpio.h>
#include <fsl_esdhc.h>
#include <mmc.h>
#include <asm/arch/imx8mm_pins.h>
#include <asm/arch/sys_proto.h>
#include <asm/mach-imx/gpio.h>
#include <asm/mach-imx/mxc_i2c.h>
#include <asm/arch/clock.h>
#include <spl.h>
#include <asm/mach-imx/dma.h>
#include <power/pmic.h>
#include <power/bd71837.h>
#include "../common/tcpc.h"
#include <usb.h>
#if defined(ENABLE_I2C_TRIZEPS8MINI) && ENABLE_I2C_TRIZEPS8MINI
#include <sec_mipi_dsim.h>
#include <imx_mipi_dsi_bridge.h>
#include <mipi_dsi_panel.h>
#include <asm/mach-imx/video.h>
#endif
#include "../common/kuk_boards.h"

#define TRIZEPS8_USE_RESET_OUT_AS_WATCHDOG_OUT	0	/* NXP i.MX8MQ EVK uses RESET_OUT as Watchdog Out */


DECLARE_GLOBAL_DATA_PTR;

#define UART_PAD_CTRL	 (PAD_CTL_DSE6 | PAD_CTL_FSEL1)
#define WDOG_PAD_CTRL	 (PAD_CTL_DSE6 | PAD_CTL_ODE | PAD_CTL_PUE | PAD_CTL_PE)
#define GPIO_PAD_CTRL    (PAD_CTL_DSE6 | PAD_CTL_FSEL1)

#define GPIO_PAD_PU_CTRL (PAD_CTL_DSE6 | PAD_CTL_PUE | PAD_CTL_PE)
#define GPIO_PAD_PD_CTRL (PAD_CTL_DSE6 |               PAD_CTL_PE)

#if defined(ENABLE_I2C_TRIZEPS8MINI) && ENABLE_I2C_TRIZEPS8MINI
#ifdef CONFIG_SYS_I2C_MXC

#define I2C_PAD_CTRL	(PAD_CTL_DSE6 | PAD_CTL_HYS | PAD_CTL_PUE | PAD_CTL_PE)
#define PC MUX_PAD_CTRL(I2C_PAD_CTRL)

struct i2c_pads_info i2c_pad_info1 = {
	.scl = {
		.i2c_mode = IMX8MM_PAD_I2C1_SCL_I2C1_SCL | PC,
		.gpio_mode = IMX8MM_PAD_I2C1_SCL_GPIO5_IO14 | PC,
		.gp = IMX_GPIO_NR(5, 14),
	},
	.sda = {
		.i2c_mode = IMX8MM_PAD_I2C1_SDA_I2C1_SDA | PC,
		.gpio_mode = IMX8MM_PAD_I2C1_SDA_GPIO5_IO15 | PC,
		.gp = IMX_GPIO_NR(5, 15),
	},
};
#endif
#endif

static iomux_v3_cfg_t const uart_pads[] = {
	IMX8MM_PAD_UART1_RXD_UART1_RX | MUX_PAD_CTRL(UART_PAD_CTRL),
	IMX8MM_PAD_UART1_TXD_UART1_TX | MUX_PAD_CTRL(UART_PAD_CTRL),
};

#define PCIE_CLKREQ             IMX_GPIO_NR(5,20)
#define PCIE_WL_POWERDOWN       IMX_GPIO_NR(3, 5)
#define PCIE_W_DISABLE_GPIO     IMX_GPIO_NR(3,17)
#define PCIE_RESET              IMX_GPIO_NR(2,19)
#define PCIE_WAKE               IMX_GPIO_NR(3, 2)

#define PCIE_W_DISABLE_GPIO_EXT IMX_GPIO_NR(4,15)  
#define PCIE_RESET_EXT          IMX_GPIO_NR(4,17)
#define PCIE_WAKE_EXT           IMX_GPIO_NR(3,13)  

  
static iomux_v3_cfg_t const pcie_wifi_pads[] = {
        IMX8MM_PAD_I2C4_SCL_PCIE1_CLKREQ_B  | MUX_PAD_CTRL(0x61),          // CLKREQ_B  
//      IMX8MM_PAD_I2C4_SCL_GPIO5_IO20      | MUX_PAD_CTRL(GPIO_PAD_CTRL), // CLKREQ_B
	IMX8MM_PAD_SD2_RESET_B_GPIO2_IO19   | MUX_PAD_CTRL(GPIO_PAD_CTRL), // PCIE_RESET
	IMX8MM_PAD_NAND_CLE_GPIO3_IO5       | MUX_PAD_CTRL(GPIO_PAD_CTRL), // PCIE_WL_POWERDOWN
	IMX8MM_PAD_NAND_CE1_B_GPIO3_IO2     | MUX_PAD_CTRL(GPIO_PAD_CTRL), // PCIE_WAKE
	IMX8MM_PAD_NAND_WE_B_GPIO3_IO17     | MUX_PAD_CTRL(GPIO_PAD_CTRL), // PCIE_W_DISABLE_GPIO
};

static iomux_v3_cfg_t const pcie_ext_pads[] = {
	IMX8MM_PAD_SAI1_TXD5_GPIO4_IO17     | MUX_PAD_CTRL(GPIO_PAD_PU_CTRL), // PCIE_RESET
        IMX8MM_PAD_I2C4_SCL_PCIE1_CLKREQ_B  | MUX_PAD_CTRL(0x61),             // CLKREQ_B
	IMX8MM_PAD_SAI1_TXD3_GPIO4_IO15     | MUX_PAD_CTRL(GPIO_PAD_CTRL),    // PCIE_W_DISABLE_GPIO
	IMX8MM_PAD_NAND_CLE_GPIO3_IO5       | MUX_PAD_CTRL(GPIO_PAD_CTRL),    // PCIE_WL_POWERDOWN
	IMX8MM_PAD_NAND_DATA07_GPIO3_IO13   | MUX_PAD_CTRL(GPIO_PAD_CTRL),    // PCIE_WAKE
};

#define USBH_PWR_GPIO IMX_GPIO_NR(1,14)  
#define OTG_PWR_GPIO  IMX_GPIO_NR(1,12)  
  
static iomux_v3_cfg_t const usb_pads[] = {
	IMX8MM_PAD_GPIO1_IO14_GPIO1_IO14 | MUX_PAD_CTRL(GPIO_PAD_CTRL),
	IMX8MM_PAD_GPIO1_IO12_GPIO1_IO12 | MUX_PAD_CTRL(GPIO_PAD_CTRL),	
};

#if defined(ENABLE_I2C_TRIZEPS8MINI) && ENABLE_I2C_TRIZEPS8MINI
#define CAMERA_PWDN IMX_GPIO_NR(1,3)
#define CAMERA_nRST IMX_GPIO_NR(1,6)

static iomux_v3_cfg_t const camera_pads[] = {
        IMX8MM_PAD_GPIO1_IO03_GPIO1_IO3  | MUX_PAD_CTRL(GPIO_PAD_PU_CTRL),  // Pin 123 Cam Pwdn
	IMX8MM_PAD_GPIO1_IO06_GPIO1_IO6  | MUX_PAD_CTRL(GPIO_PAD_PU_CTRL),  // Pin 123 Cam nRST
};
#endif

#if TRIZEPS8_USE_RESET_OUT_AS_WATCHDOG_OUT
static iomux_v3_cfg_t const wdog_pads[] = {
	IMX8MM_PAD_GPIO1_IO02_WDOG1_WDOG_B  | MUX_PAD_CTRL(WDOG_PAD_CTRL),
};
#endif

static iomux_v3_cfg_t const reset_out_pads_v1r1[] = { IMX8MM_PAD_GPIO1_IO02_GPIO1_IO2 | MUX_PAD_CTRL(GPIO_PAD_PU_CTRL),};
static iomux_v3_cfg_t const reset_out_pads_v1r2[] = { IMX8MM_PAD_NAND_DQS_GPIO3_IO14  | MUX_PAD_CTRL(GPIO_PAD_PU_CTRL),};

#define GPIO_RESET_OUT_V1R1         IMX_GPIO_NR(1, 2)
#define GPIO_RESET_OUT_V1R2         IMX_GPIO_NR(3,14)

#define SDHC1_VOLTAGE_SELECT_GPIO   IMX_GPIO_NR(3,17)
// extern iomux_v3_cfg_t const sdhc1_vselect_pads[];


#define GPIO4_DATA *(unsigned int *)0x30230000
#define GPIO4_DIR  *(unsigned int *)0x30230004
#define GPIO4_MUX  *(unsigned int *)0x30330178
#define GPIO4_PAD  *(unsigned int *)0x303303E0

int iomux_sai1_txd5, iopad_sai1_txd5;
int board_init_has_run=0;

#if defined(ENABLE_I2C_TRIZEPS8MINI) && ENABLE_I2C_TRIZEPS8MINI
#ifdef CONFIG_VIDEO_MXS
void   GetDisplayEnvironment(void);
#endif
#endif

#if defined(ENABLE_I2C_TRIZEPS8MINI) && ENABLE_I2C_TRIZEPS8MINI
static void init_camera_ov5640(void)
{
        printf("Init Camera Pins SODIMM 123,125 as GPIO \n");  
	imx_iomux_v3_setup_multiple_pads(camera_pads, ARRAY_SIZE(camera_pads));
	gpio_request(CAMERA_PWDN, "camera_pwdn");
	gpio_direction_output(CAMERA_PWDN, 1);
	gpio_request(CAMERA_nRST, "camera_nrst");
	gpio_direction_output(CAMERA_nRST, 1);
	gpio_direction_output(CAMERA_nRST, 0);
	gpio_direction_output(CAMERA_nRST, 1);
	gpio_direction_output(CAMERA_PWDN, 0);	
	gpio_direction_output(CAMERA_nRST, 1);
	gpio_direction_output(CAMERA_nRST, 0);
	udelay(5000);
	gpio_direction_output(CAMERA_nRST, 1);
	
}
#endif

int board_early_init_f(void)
{
  //    init_gpio4_17(0);  
#if TRIZEPS8_USE_RESET_OUT_AS_WATCHDOG_OUT
	struct wdog_regs *wdog = (struct wdog_regs *)WDOG1_BASE_ADDR;
	imx_iomux_v3_setup_multiple_pads(wdog_pads, ARRAY_SIZE(wdog_pads));
	set_wdog_reset(wdog);
#endif
	imx_iomux_v3_setup_multiple_pads(usb_pads, ARRAY_SIZE(usb_pads));
	imx_iomux_v3_setup_multiple_pads(uart_pads, ARRAY_SIZE(uart_pads));
	
	init_uart_clk(0);
	return 0;
}

#ifdef CONFIG_BOARD_POSTCLK_INIT
int board_postclk_init(void)
{
	/* TODO */
	return 0;
}
#endif


int board_phys_sdram_size(phys_size_t *size)
{
	u64 ram_size;

	switch( kuk_GetRAMSize())
	{
		case KUK_RAMSIZE_512MB:	ram_size = 	(512*1024*1024UL);	break;
		case KUK_RAMSIZE_1GB:	ram_size = 	(1024*1024*1024UL);	break;
		case KUK_RAMSIZE_2GB:	ram_size = 	(2048*1024*1024UL);	break;
#ifdef CONFIG_KUK_CUTRAMSIZE_3GB		
		case KUK_RAMSIZE_4GB:	ram_size = 	(3072*1024*1024UL); break; //(4096*1024*1024UL);	break;
#else
		case KUK_RAMSIZE_4GB:	ram_size = 	(4096*1024*1024UL); break; //(4096*1024*1024UL);	break;
#endif		
		case KUK_RAMSIZE_8GB:	ram_size = 	(8192*1024*1024UL);	break;
		default:				ram_size = PHYS_SDRAM_SIZE;		break;
	}

	*size = ram_size;


	return 0;
}

ulong board_get_usable_ram_top(ulong total_size)
{
	if(gd->ram_top > 0x100000000)
    	gd->ram_top = 0x100000000;

	return gd->ram_top;
}

#ifdef CONFIG_OF_BOARD_SETUP
int ft_board_setup(void *blob, bd_t *bd)
{
	return 0;
}
#endif

#ifdef CONFIG_FEC_MXC

static iomux_v3_cfg_t const fec1_nrst_pads[] = {
	IMX8MM_PAD_ENET_RX_CTL_ENET1_RGMII_RX_CTL | MUX_PAD_CTRL(0x91),
	IMX8MM_PAD_ENET_RD2_ENET1_RGMII_RD2       | MUX_PAD_CTRL(0x91),
	IMX8MM_PAD_ENET_RXC_ENET1_RGMII_RXC       | MUX_PAD_CTRL(0x91),
	IMX8MM_PAD_ENET_RD3_ENET1_RGMII_RD3       | MUX_PAD_CTRL(0x91),
	IMX8MM_PAD_ENET_RD0_ENET1_RGMII_RD0       | MUX_PAD_CTRL(0x91),	
	IMX8MM_PAD_ENET_RD1_ENET1_RGMII_RD1       | MUX_PAD_CTRL(0x91),
};

#define FEC_RST_PAD IMX_GPIO_NR(1, 9)
#define FEC_MODE0   IMX_GPIO_NR(1,24)
#define FEC_MODE1   IMX_GPIO_NR(1,28)
#define FEC_MODE2   IMX_GPIO_NR(1,25)
#define FEC_MODE3   IMX_GPIO_NR(1,29)

static iomux_v3_cfg_t const fec1_rst_pads[] = {
    IMX8MM_PAD_GPIO1_IO09_GPIO1_IO9           | MUX_PAD_CTRL(GPIO_PAD_PD_CTRL),
	IMX8MM_PAD_ENET_RX_CTL_GPIO1_IO24         | MUX_PAD_CTRL(GPIO_PAD_PD_CTRL),
	IMX8MM_PAD_ENET_RD2_GPIO1_IO28            | MUX_PAD_CTRL(GPIO_PAD_PD_CTRL),
	IMX8MM_PAD_ENET_RXC_GPIO1_IO25            | MUX_PAD_CTRL(GPIO_PAD_PD_CTRL),
	IMX8MM_PAD_ENET_RD3_GPIO1_IO29            | MUX_PAD_CTRL(GPIO_PAD_PD_CTRL),
	IMX8MM_PAD_ENET_RD0_GPIO1_IO26            | MUX_PAD_CTRL(GPIO_PAD_PD_CTRL),
	IMX8MM_PAD_ENET_RD1_GPIO1_IO27            | MUX_PAD_CTRL(GPIO_PAD_PD_CTRL),
};

static void setup_iomux_fec(void)
{

	// printf("%s: reset fec\n", __func__);
	imx_iomux_v3_setup_multiple_pads(fec1_rst_pads, ARRAY_SIZE(fec1_rst_pads));
	gpio_request(FEC_RST_PAD, "fec1_rst");
	gpio_direction_output(FEC_RST_PAD, 0);
	udelay(1000);		
#if 1
	gpio_request(FEC_MODE0,"fec_mode0");	gpio_direction_output(FEC_MODE0, 0);
	gpio_request(FEC_MODE1,"fec_mode1");	gpio_direction_output(FEC_MODE1, 0);
	gpio_request(FEC_MODE2,"fec_mode2");	gpio_direction_output(FEC_MODE2, 0);
	gpio_request(FEC_MODE3,"fec_mode3");	gpio_direction_output(FEC_MODE3, 0);
#endif
	gpio_direction_output(FEC_RST_PAD, 1);
	udelay(50);
	gpio_direction_output(FEC_RST_PAD, 0);
	udelay(1000);
	gpio_direction_output(FEC_RST_PAD, 1);
	imx_iomux_v3_setup_multiple_pads(fec1_nrst_pads, ARRAY_SIZE(fec1_nrst_pads));
	udelay(1000);

}

static int setup_fec(void)
{
	int module;

	struct iomuxc_gpr_base_regs *const iomuxc_gpr_regs
		= (struct iomuxc_gpr_base_regs *) IOMUXC_GPR_BASE_ADDR;

    module = kuk_GetModule();
	if(module == KUK_MODULE_SBCSOM8MINI)
	{
		//Enable SBCSOM ENET
		gpio_request(IMX_GPIO_NR(1, 6), "ENET EN");
		gpio_direction_output(IMX_GPIO_NR(1, 6), 1);
	}

	/* Use 125M anatop REF_CLK1 for ENET1, not from external */
	// printf("%s: Enable FEC CLK\n", __func__);
	clrsetbits_le32(&iomuxc_gpr_regs->gpr[1],IOMUXC_GPR_GPR1_GPR_ENET1_TX_CLK_SEL_SHIFT, 0);
	set_clk_enet(ENET_125MHZ);
	setup_iomux_fec();
	return 0;
}
int ethernet_1GB(void)
{
  char *s;
  int i;  
  s = env_get("ethspeed");
  i = strncmp(s,"1G", 2);
  if( i == 0 )
  {
      printf("1Gb (ethspeed)\n");
      return(1);
  }else
  {
      printf("100Mb (ethspeed)\n");
      return(0);
  }  
}
  

void board_phy_dump(struct phy_device *phydev)
{
  int i;
  printf("------------------%s-----------------------\n", __func__);
  
  for(i=0; i < 0x1F; i++ )
    printf("Phy Adr: 0x%02x -> 0x%04x\n", i, phy_read(phydev, CONFIG_FEC_MXC_PHYADDR, i));
  
}

#define AR803x_PHY_DEBUG_ADDR_REG	0x1d
#define AR803x_PHY_DEBUG_DATA_REG	0x1e
#define AR803x_PHY_MMD_ADDR_REG	        0x0d
#define AR803x_PHY_MMD_OFFSET_REG	0x0e

void ar8031_write_debug_reg(struct phy_device *phydev, int reg, int value)
{
  phy_write(phydev, CONFIG_FEC_MXC_PHYADDR, AR803x_PHY_DEBUG_ADDR_REG, reg);
  phy_write(phydev, CONFIG_FEC_MXC_PHYADDR, AR803x_PHY_DEBUG_DATA_REG, value);  
}

int ar8031_write_mmd_reg(struct phy_device *phydev, int mmd, int reg, int value)
{
  int regval;
  phy_write(phydev, CONFIG_FEC_MXC_PHYADDR, AR803x_PHY_MMD_ADDR_REG,         mmd);
  phy_write(phydev, CONFIG_FEC_MXC_PHYADDR, AR803x_PHY_MMD_OFFSET_REG,       reg);
  phy_write(phydev, CONFIG_FEC_MXC_PHYADDR, AR803x_PHY_MMD_ADDR_REG,(0x4000|mmd));
  regval=phy_read(phydev, CONFIG_FEC_MXC_PHYADDR,      AR803x_PHY_MMD_OFFSET_REG);
  phy_write(phydev, CONFIG_FEC_MXC_PHYADDR, AR803x_PHY_MMD_OFFSET_REG,     value);  
  return(regval);  
}

int ar8031_read_mmd_reg(struct phy_device *phydev, int mmd, int reg)
{
  int regval;
  phy_write(phydev, CONFIG_FEC_MXC_PHYADDR, AR803x_PHY_MMD_ADDR_REG,         mmd);
  phy_write(phydev, CONFIG_FEC_MXC_PHYADDR, AR803x_PHY_MMD_OFFSET_REG,       reg);
  phy_write(phydev, CONFIG_FEC_MXC_PHYADDR, AR803x_PHY_MMD_ADDR_REG,(0x4000|mmd));
  regval=phy_read(phydev, CONFIG_FEC_MXC_PHYADDR,      AR803x_PHY_MMD_OFFSET_REG);
  return(regval&0xffff);  
}

#define MMD7                   0x07
#define MMD3                   0x03
#define SGMII_Control_Register 0x8011
#define EEE_Advertisement      0x3C
#define EEE_Control            0x805D
#define VDIFF_900mV            0x8000
#define VDIFF_800mV            0x6000
#define VDIFF_700mV            0x4000
#define VDIFF_600mV            0x2000


int board_phy_config(struct phy_device *phydev)
{
  int j,i=100;

  if ( kuk_GetPeripheral( KUK_PERIPHERAL_ETHERNET) == KUK_ETHERNET_NONE)
  {
	printf("none (disabled)\n");
	return 0;
  }
  /* enable rgmii rxc skew and phy mode select to RGMII copper */
#if 0
  // Reset Phy....
  phy_write(phydev, CONFIG_FEC_MXC_PHYADDR, 0x0, 0x8000);
  while( i-- && ((phy_read(phydev, CONFIG_FEC_MXC_PHYADDR, 0x00) & 0x8000)!=0))
    udelay(1000);
#endif
  if (phydev->drv->config)
    phydev->drv->config(phydev);

  //  phy_write(phydev, CONFIG_FEC_MXC_PHYADDR, 0x00, 0x1140);
  phy_write(phydev, CONFIG_FEC_MXC_PHYADDR, 0x1F, 0xF500); // 0x8500); // 0xF400); // 0x8400
#if 0
  //  ar8031_write_debug_reg(phydev,  0x1f, 0x0008);  
  ar8031_write_debug_reg(phydev,  0x00, 0x82ee);
  ar8031_write_debug_reg(phydev,  0x05, 0x0100);
#endif  
  i= ar8031_read_mmd_reg(phydev,MMD7,EEE_Advertisement);    
  if( ethernet_1GB() == 0 )
  {
    phy_write(phydev, CONFIG_FEC_MXC_PHYADDR, 0x09, 0x0);    // Do not use 1GBit
    j  =  i & ~4;
    j &= ~2;    
    //printf("%s: Set Phy to 100MBit EEE=0x%04x->0x%04x\n", __func__, i, j);
    ar8031_write_mmd_reg(phydev,MMD7,EEE_Advertisement,i);// Write MMD7 EEE no EEE mode
  }else
  {
    //printf("%s: Set Phy to 1GBit\n", __func__);        
    phy_write(phydev, CONFIG_FEC_MXC_PHYADDR, 0x09, 0x300);    // use 1GBit 
  }

  i= ar8031_read_mmd_reg( phydev,MMD3,EEE_Control);
  j= i & ~0x100;
  //printf("%s: Disable EEE Control    0x%04x->0x%04x\n", __func__, i, j);  
  i= ar8031_write_mmd_reg(phydev,MMD3,EEE_Control, j);  

  i  = phy_read(phydev, CONFIG_FEC_MXC_PHYADDR, 0x14);
  j  = 0x2C;  
  //printf("%s: Smart Speed Register   0x%04x->0x%04x\n", __func__, i, j);  
  phy_write(phydev, CONFIG_FEC_MXC_PHYADDR, 0x14, j);

  i= ar8031_read_mmd_reg(phydev,MMD7,SGMII_Control_Register);
  j= ( i & 0xfff) | VDIFF_900mV;

  //printf("%s: SGMII_Control_Register 0x%04x->0x%04x\n", __func__, i, j);  
  ar8031_write_mmd_reg(phydev,MMD7,SGMII_Control_Register, j);// Write MMD7 8011 900mV    

  phy_write(phydev, CONFIG_FEC_MXC_PHYADDR, 0x00,  0x3100); // 100MBit Enable AutoNeg. DuplexMode
  phy_write(phydev, CONFIG_FEC_MXC_PHYADDR, 0x00,  0x3300); // " restart AN
  // board_phy_dump(phydev); 
  return(0);  
}
#endif

#define GPC_IPS_BASE_ADDR 0x303A0000
void pci_clk_enable(void);
void mx8qxp_pcie_init(void);
void mx8qm_pcie_init(void);

static void setup_pcie(void)
{
        char *s;
	int  internal_wifi=0, pcie_ext=0, pcbrev=1;

	s = env_get("pcie");
	if( s )
	{	  
		internal_wifi = (strncmp(s,"wifionboard", 2) == 0) ? 1:0;
		pcie_ext      = (strncmp(s,"extern",      2) == 0) ? 1:0;
		//printk("%s: Environment pcie=%s ", __func__, s);
	}else
	{	       
		printk("%s: Environment variable pcie not set. Skip Init PCIe gpios\n", __func__);
	}
	if ( kuk_GetPeripheral( KUK_PERIPHERAL_WIRELESS) == KUK_WIRELESS_NONE)
 	{
		internal_wifi = 0;
 	}

	 if( internal_wifi )
	 {
	   pcbrev=kuk_GetPCBrevision();		   
	   printk("PCIE:  Configure for onboard Wifi Card\n");
	   // For V1R3 do not init IMX8MM_PAD_NAND_WE_B_GPIO3_IO17 --> Arraysize decrement
	   imx_iomux_v3_setup_multiple_pads(pcie_wifi_pads,
					    (pcbrev >= KUK_PCBREV_V1R3) ? (ARRAY_SIZE(pcie_wifi_pads)-1) : (ARRAY_SIZE(pcie_wifi_pads)) );
	   gpio_request(PCIE_CLKREQ,                  "PCIE_CLKREQ");
	   gpio_request(PCIE_WL_POWERDOWN,            "PCIE_WL_POWERDOWN");
	   gpio_request(PCIE_WAKE,                    "PCIE_WAKE");
	   gpio_request(PCIE_RESET,                   "PCIE_RESET");
	   if( pcbrev < KUK_PCBREV_V1R3 )
	     gpio_request(PCIE_W_DISABLE_GPIO,          "PCIE_W_DISABLE_GPIO");
	   
	   gpio_direction_output(PCIE_CLKREQ,          0); // Switch on PCIE CLKREQ
	   gpio_direction_output(PCIE_WL_POWERDOWN,    0); // Switch on Wifi Module
	   if( pcbrev < KUK_PCBREV_V1R3 )
	     gpio_direction_output(PCIE_W_DISABLE_GPIO,0); // Do not disable PCIe
	   gpio_direction_input( PCIE_WAKE);               // Take Wake as input
	   gpio_direction_output(PCIE_RESET,           0); // Activate Reset
	   udelay(1000);
	   gpio_direction_output(PCIE_WL_POWERDOWN,    1); // Switch on Wifi Module	   
	   udelay(1000);
	   if( pcbrev < KUK_PCBREV_V1R3 )	   
	     gpio_direction_output(PCIE_W_DISABLE_GPIO,1); // Do not disable PCIe
	   gpio_direction_output(PCIE_RESET,           1);
	   udelay(500);	
	   gpio_direction_output(PCIE_RESET,           0); // Activate Reset again
	   udelay(500);	
	   gpio_direction_output(PCIE_RESET,           1);
	   // mx8qm_pcie_init();	   
	 }else
	 if( pcie_ext )	  
	 {
	   //printk("> Setup PCIe Gpios for external PCIe Slot\n");	   	   
	   printk("PCIE:  Configure for external PCIe Slot\n");
	   imx_iomux_v3_setup_multiple_pads(pcie_ext_pads, ARRAY_SIZE(pcie_ext_pads));
	   gpio_request(PCIE_WAKE_EXT,                      "PCIE_WAKE_EXT");
	   gpio_request(PCIE_RESET_EXT,                     "PCIE_RESET_EXT");
	   gpio_request(PCIE_W_DISABLE_GPIO_EXT,            "PCIE_W_DISABLE_GPIO_EXT");
	   gpio_direction_output(PCIE_W_DISABLE_GPIO_EXT,1); // Do not disable PCIe
	   gpio_direction_input(PCIE_WAKE_EXT);              // Take Wake as input
	   gpio_direction_output(PCIE_RESET_EXT,         0); // Activate Reset
	   udelay(1000);	
	   gpio_direction_output(PCIE_RESET_EXT,        1);
	   udelay(500);	
	   gpio_direction_output(PCIE_RESET_EXT,        0); // Activate Reset again
	   udelay(500);	
	   gpio_direction_output(PCIE_RESET_EXT,        1);
	   // mx8qm_pcie_init();	   
	 }
}

/* Enable Uart4/Uart2 pre-set in imx8mm_bl31_setup.c bl31_early_platform_setup2() */

void setup_periph2mcu(void)
{
        char *s;  
        volatile unsigned long uart4m4 = 0xff;
        volatile unsigned long uart2m4 = 0xff;	

	s = env_get("uart4-access");
	if( s )
	{	     
	  if( strncmp(s,"both", 2) == 0) uart4m4=0xff;
	  if( strncmp(s,"a53",  2) == 0) uart4m4=0xf3;
	  if( strncmp(s,"m4",   2) == 0) uart4m4=0xfc;	  	  
	  *(volatile unsigned long *)0x303D0518=uart4m4;	
	}else
	{	       
	  *(volatile unsigned long *)0x303D0518=uart4m4;
	}
	
	s = env_get("uart2-access");
	if( s )
	{	     
	  if( strncmp(s,"both", 2) == 0) uart2m4=0xff;
	  if( strncmp(s,"a53",  2) == 0) uart2m4=0xf3;
	  if( strncmp(s,"m4",   2) == 0) uart2m4=0xfc;	  	  
	  *(volatile unsigned long *)0x303D05A4=uart2m4;	
	}else
	{	       
	  *(volatile unsigned long *)0x303D05A4=uart2m4;
	}

  
}


void pci_init_board(void)
{
  /* test the 1 lane mode of the PCIe A controller */
  printf("%s:***************************************************\n", __func__);
}
static int resout=0;

void reset_out(int val)
{
  if( kuk_GetPCBrevision() > KUK_PCBREV_V1R1 )
  {      
    if( resout == 0 )
    {
      imx_iomux_v3_setup_multiple_pads( reset_out_pads_v1r2, 1);      
      gpio_request(GPIO_RESET_OUT_V1R2, "RESET_OUT");
      resout=1;        
    }  
    gpio_direction_output(GPIO_RESET_OUT_V1R2,val);
  }else
  {
    if( resout == 0 )
    {
      imx_iomux_v3_setup_multiple_pads( reset_out_pads_v1r1, 1);            
      gpio_request(GPIO_RESET_OUT_V1R1, "RESET_OUT");
      resout=1;        
    }  
    gpio_direction_output(GPIO_RESET_OUT_V1R1,val);
  }
}

char ArticleText[256];

void kuk_set_article_bootstorage( char *pArticle, int gigabyte, int isemmc);
int board_set_bootstorage_size(struct mmc *mmc)
{
  unsigned long bootstore_size= mmc->capacity_user/(1024*1024*1024);
  kuk_set_article_bootstorage( &ArticleText[0],  bootstore_size, (mmc->version&MMC_VERSION_MMC)?1:0);  
  if( mmc->version & MMC_VERSION_MMC )
  {      
    printf("EMMC BootStorage User: %lld GB\n", mmc->capacity_user/(1024*1024*1024));
    if( bootstore_size > 7 ) 
    printf("EMMC BootStorage Boot: %lld MB\n", mmc->capacity_boot/(1024*1024));
    printf("EMMC BootStorage Rpmb: %lld MB\n", mmc->capacity_rpmb/(1024*1024));
  }else
    printf("SD   BootStorage User: %llx\n", mmc->capacity_user);

  printf("Module Assumed Art-No:<%s>\n", ArticleText);
  env_set("kuk_article", ArticleText);  
  return 0;
}

int board_init(void)
{
	char text[256];
	kuk_GetArticleNo( &ArticleText[0], 24);
	printf("Modul: %s\n", ArticleText);  
	kuk_GetDescription( &text[0], sizeof( text));
	printf("%s\n", text);  
	reset_out(1);
#ifdef CONFIG_FEC_MXC
	if ( kuk_GetPeripheral( KUK_PERIPHERAL_ETHERNET) != KUK_ETHERNET_NONE)
	{		
		setup_fec();
	}
#endif
#if defined(ENABLE_I2C_TRIZEPS8MINI) && ENABLE_I2C_TRIZEPS8MINI
	init_camera_ov5640();	
	setup_pcie(); /* environment not read here */
#ifdef CONFIG_SYS_I2C_MXC
	setup_i2c(0, CONFIG_SYS_I2C_SPEED, 0x7f, &i2c_pad_info1);	
	//init_camera_ov5640();	
#endif
#endif	
	/* Re-Configure nRESET_OUT on MCU as input */
	uint8_t mcu_reset_out_data[2] = { 0x57, 0x01 };
	struct udevice *bus, *mcudev;
	int ret = uclass_get_device_by_seq(UCLASS_I2C, 2, &bus);
	if(!ret)
		ret = dm_i2c_probe(bus, 0x10, 0, &mcudev);
	if(!ret)
		dm_i2c_write(mcudev, 0x04, mcu_reset_out_data, 2);

	/* Enable Uart4/Uart2 pre-set in imx8mm_bl31_setup.c bl31_early_platform_setup2() */
	setup_periph2mcu();
	board_init_has_run=1;
	return 0;
}

int board_mmc_get_env_dev(int devno)
{
	return devno;
}

int mmc_map_to_kernel_blk(int devno)
{
	return devno;
}


int board_late_init(void)
{
#ifdef CONFIG_ENV_IS_IN_MMC
	board_late_mmc_env_init();
#endif
  	setup_pcie();
	return 0;
}

int board_ehci_usb_phy_mode(struct udevice *dev)
{
	if (dev->seq == 0)
	{
	  printf("%s: USB_INIT_DEVICE\n", __func__);
	  return USB_INIT_DEVICE;
	}else
	{	
	  printf("%s: USB_INIT_HOST\n", __func__);
	  return USB_INIT_HOST;
	}
}

int board_usb_init(int index, enum usb_init_type init)
{
	imx8m_usb_power(index, true);
	gpio_request( OTG_PWR_GPIO, "otgpwr");
	gpio_direction_output(  OTG_PWR_GPIO, 0);
	gpio_free( OTG_PWR_GPIO);

	gpio_request(USBH_PWR_GPIO, "usbhpwr");
	gpio_direction_output( USBH_PWR_GPIO, 0);
	gpio_free( USBH_PWR_GPIO);
	return(0);
}

int board_usb_cleanup(int index, enum usb_init_type init)
{
	imx8m_usb_power(index, false);
	gpio_request( OTG_PWR_GPIO, "otgpwr");
	gpio_direction_output(  OTG_PWR_GPIO, 1);
	gpio_free( OTG_PWR_GPIO);

	gpio_request(USBH_PWR_GPIO, "usbhpwr");
	gpio_direction_output( USBH_PWR_GPIO, 1);
	gpio_free( USBH_PWR_GPIO);
	return(0);
}

static iomux_v3_cfg_t const sdhc1_vselect_pads[] = {
	IMX8MM_PAD_NAND_WE_B_SION_GPIO3_IO17 | MUX_PAD_CTRL( GPIO_PAD_PD_CTRL ), // SELECT SDHC1 VOLTAGE
};

struct fsl_esdhc ;
int sd1_was_already_1v8=0;

int board_mmc_signal_voltage_eshdc( struct fsl_esdhc *regs, int volt)
{
        int version, module, ret = 0;
	module = kuk_GetModule();	

	// For Myon no switch needed just return 0
	if( module == KUK_MODULE_MYON2 || module == KUK_MODULE_MYON2NANO )
	{
	  if( (volt < 3000000) )
	    sd1_was_already_1v8=1;	  
	  return 0;
	}
	
	if( (module == KUK_MODULE_TRIZEPS8MINI || module == KUK_MODULE_TRIZEPS8NANO) )
	{
	    
	  if(((version=kuk_GetPCBrevision()) <= KUK_PCBREV_V1R2) && volt < 3300000)
	  {
	      printf("Trizeps8mini V1R%d does not support VMMC Voltage < 3.3V\n\r", version+1);	  
	      return(1);
	  }
	  ret=1;	  
	  switch ((unsigned long) regs)
	  {
	        case USDHC1_BASE_ADDR:
		  if( (volt >= 3000000) || (volt==0) )
		  {
		    if( sd1_was_already_1v8 )
		    {
		      printf("Trizeps8mini V1R%d USDHC1 do not set VMMC Voltage back to 3.3V\n\r", version+1);		      
		    }else
		    {			
		      printf("Trizeps8mini V1R%d USDHC1 set VMMC Voltage 3.3V\n\r", version+1);
		      imx_iomux_v3_setup_multiple_pads(sdhc1_vselect_pads, 1);
		      gpio_request(SDHC1_VOLTAGE_SELECT_GPIO, "SDHC1_VOLTAGE_SELECT");
		      gpio_direction_output(SDHC1_VOLTAGE_SELECT_GPIO, 0);
		      gpio_free(SDHC1_VOLTAGE_SELECT_GPIO);
		    }		    
		  }else
		  {
		    gpio_request(SDHC1_VOLTAGE_SELECT_GPIO, "SDHC1_VOLTAGE_SELECT");
		    imx_iomux_v3_setup_multiple_pads(sdhc1_vselect_pads, 1);		    
		    if( sd1_was_already_1v8==0)
		      printf("Trizeps8mini V1R%d USDHC1 set VMMC Voltage 1.8V\n\r", version+1);    
		    gpio_direction_output(SDHC1_VOLTAGE_SELECT_GPIO, 1);
		    gpio_free(SDHC1_VOLTAGE_SELECT_GPIO);
		    sd1_was_already_1v8=1;	    
		  }		  
		  ret = 0;		  
		  break;
		  
		case USDHC2_BASE_ADDR:
		  if( (volt < 3000000) && (volt!=0) )
		  {
		      printf("Trizeps8mini V1R%d USDHC2 set VMMC Voltage %d impossible \n\r", version+1, volt);
		  }else
		  {
    		      printf("Trizeps8mini V1R%d USDHC2 set VMMC Voltage %d\n\r", version+1, volt);
		      ret = 0;		  		    
		  }		  		  
		  break;
		    
	        case USDHC3_BASE_ADDR:
		  if( (volt < 3000000) && (volt!=0) )
		  {
		      printf("Trizeps8mini V1R%d USDHC3 set VMMC Voltage %d impossible \n\r", version+1, volt);
		  }else
		  {
    		      printf("Trizeps8mini V1R%d USDHC3 set VMMC Voltage %d\n\r", version+1, volt);
		      ret = 0;		  		    
		  }		  		  
		  break;
		  
	        default:
		  if( volt == 0 )
		  {
		      printf("Trizeps8mini V1R%d ESDHC@0x%lx set VMMC Voltage %d, leave (3.3V)\n\r", version+1,
			     (unsigned long) regs, volt);
		      ret=0;		  
		  }else	  
		    printf("Trizeps8mini V1R%d ESDHC@0x%lx set VMMC Voltage %d impossible \n\r", version+1,
			(unsigned long) regs, volt);
		  break;		  
	  }
	}	
	return ret;
}

int board_mmc_signal_voltage(struct mmc *mmc, int volt)
{
  printf("Trizeps8mini board_mmc_signal_voltage...\n\r");  
  struct fsl_esdhc_cfg *cfg = (struct fsl_esdhc_cfg *) mmc->priv;
  struct fsl_esdhc *regs    = (struct fsl_esdhc     *) cfg->esdhc_base;	  
  return( board_mmc_signal_voltage_eshdc(regs, volt) );
}


int board_mmc_getcd(struct mmc *mmc, unsigned long esdhc_base)
{
	int ret = 0;
	switch (esdhc_base) {
	case USDHC1_BASE_ADDR:
	        pr_debug("board_mmc_getcd@0x%lx->1\n\r", (unsigned long) esdhc_base);		
		return(1);
		break;
	case USDHC2_BASE_ADDR:
		ret = (mmc_get_op_cond(mmc) < 0) ? 0 : 1;
	        pr_debug("board_mmc_getcd@0x%lx->%d\n\r",(unsigned long) esdhc_base, ret);			  
		return ret;
		break;		
	case USDHC3_BASE_ADDR:
		ret = (mmc_get_op_cond(mmc) < 0) ? 0 : 1;
	        pr_debug("board_mmc_getcd@0x%lx->%d\n\r",(unsigned long) esdhc_base, ret);			  	  
		return ret;
		break;
	default:
	        printf("board_mmc_getcd@0x%lx->-1\n\r",(unsigned long) esdhc_base);			  	  
		return -1;
	}
	return(-1);	
}

int board_supports_uhs(unsigned long esdhc_base)
{
  int version,module;
  static int printed=0;
  
  module =kuk_GetModule();
  version=kuk_GetPCBrevision();
  pr_debug("board_supports_uhs@0x%lx=?  module:%d, version:%d esdhc_base=0 \n",
	 esdhc_base, module, version);        

  if(    (USDHC1_BASE_ADDR==esdhc_base)
      && ((module == KUK_MODULE_TRIZEPS8MINI) || (module == KUK_MODULE_TRIZEPS8NANO)))
  {
    if( version >= KUK_PCBREV_V1R3 )
    {
      if( !printed)
	printf("board_supports_uhs=1  module:%d, version:%d \n", module, version);
      printed=1;      
      return(1);
    }
    if( kuk_GetBootStorage() != KUK_BOOTSTORAGE_SDCARD )
      return(1);    
  }else
  {
    if( !printed)    
      printf("board_supports_uhs=0  module:%d, version:%d \n", module, version);          
    printed=1;            
  }
  return(0);  
}

#ifdef CONFIG_FSL_FASTBOOT
#ifdef CONFIG_ANDROID_RECOVERY
int is_recovery_key_pressing(void)
{
	return 0; /*TODO*/
}
#endif /*CONFIG_ANDROID_RECOVERY*/
#endif /*CONFIG_FSL_FASTBOOT*/

static int do_toggle_mmc_vsel(cmd_tbl_t *cmdtp, int flag, int argc, char * const argv[])
{
  int i;  
  for( i=0; i < 10; i++ )
  {      
    board_mmc_signal_voltage_eshdc((struct fsl_esdhc *) USDHC1_BASE_ADDR, 1800000);
    board_mmc_signal_voltage_eshdc((struct fsl_esdhc *) USDHC1_BASE_ADDR, 3300000);    
  }
  return(0);  
}

U_BOOT_CMD(
	toggle_mmc_vsel, 2,1, do_toggle_mmc_vsel,
	"toggle vsel pin",
	"-"
);
