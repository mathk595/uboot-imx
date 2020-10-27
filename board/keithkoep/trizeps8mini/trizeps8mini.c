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
#include <sec_mipi_dsim.h>
#include <imx_mipi_dsi_bridge.h>
#include <mipi_dsi_panel.h>
#include <asm/mach-imx/video.h>

#include "../common/kuk_boards.h"

#define TRIZEPS8_USE_RESET_OUT_AS_WATCHDOG_OUT	0	/* NXP i.MX8MQ EVK uses RESET_OUT as Watchdog Out */


DECLARE_GLOBAL_DATA_PTR;

#define UART_PAD_CTRL	 (PAD_CTL_DSE6 | PAD_CTL_FSEL1)
#define WDOG_PAD_CTRL	 (PAD_CTL_DSE6 | PAD_CTL_ODE | PAD_CTL_PUE | PAD_CTL_PE)
#define GPIO_PAD_CTRL    (PAD_CTL_DSE6 | PAD_CTL_FSEL1)

#define GPIO_PAD_PU_CTRL (PAD_CTL_DSE6 | PAD_CTL_PUE | PAD_CTL_PE)
#define GPIO_PAD_PD_CTRL (PAD_CTL_DSE6 |               PAD_CTL_PE)

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
#if 0
struct i2c_pads_info i2c_pad_info2 = {
	.scl = {
		.i2c_mode = IMX8MM_PAD_I2C2_SCL_I2C2_SCL | PC,
		.gpio_mode = IMX8MM_PAD_I2C2_SCL_GPIO5_IO16 | PC,
		.gp = IMX_GPIO_NR(5, 16),
	},
	.sda = {
		.i2c_mode = IMX8MM_PAD_I2C2_SDA_I2C2_SDA | PC,
		.gpio_mode = IMX8MM_PAD_I2C2_SDA_GPIO5_IO17 | PC,
		.gp = IMX_GPIO_NR(5, 17),
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
	IMX8MM_PAD_NAND_WE_B_GPIO3_IO17     | MUX_PAD_CTRL(GPIO_PAD_CTRL), // PCIE_W_DISABLE_GPIO
	IMX8MM_PAD_SD2_RESET_B_GPIO2_IO19   | MUX_PAD_CTRL(GPIO_PAD_CTRL), // PCIE_RESET
	IMX8MM_PAD_NAND_CLE_GPIO3_IO5       | MUX_PAD_CTRL(GPIO_PAD_CTRL), // PCIE_WL_POWERDOWN
	IMX8MM_PAD_NAND_CE1_B_GPIO3_IO2     | MUX_PAD_CTRL(GPIO_PAD_CTRL), // PCIE_WAKE
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

#define CAMERA_PWDN IMX_GPIO_NR(1,3)
#define CAMERA_nRST IMX_GPIO_NR(1,6)

static iomux_v3_cfg_t const camera_pads[] = {
        IMX8MM_PAD_GPIO1_IO03_GPIO1_IO3  | MUX_PAD_CTRL(GPIO_PAD_PU_CTRL),  // Pin 123 Cam Pwdn
	IMX8MM_PAD_GPIO1_IO06_GPIO1_IO6  | MUX_PAD_CTRL(GPIO_PAD_PU_CTRL),  // Pin 123 Cam nRST
};


#if TRIZEPS8_USE_RESET_OUT_AS_WATCHDOG_OUT
static iomux_v3_cfg_t const wdog_pads[] = {
	IMX8MM_PAD_GPIO1_IO02_WDOG1_WDOG_B  | MUX_PAD_CTRL(WDOG_PAD_CTRL),
};
#endif

#define GPIO4_DATA *(unsigned int *)0x30230000
#define GPIO4_DIR  *(unsigned int *)0x30230004
#define GPIO4_MUX  *(unsigned int *)0x30330178
#define GPIO4_PAD  *(unsigned int *)0x303303E0

int iomux_sai1_txd5, iopad_sai1_txd5;

#if 0
static void init_gpio4_17(int level)
{
       unsigned int dir,reg;
       iomux_sai1_txd5=GPIO4_MUX;
       iopad_sai1_txd5=GPIO4_PAD;       
       imx_iomux_v3_setup_multiple_pads(pcie_ext_pads, 1); // Init expernal GPIO 4.17

       reg= level ? (GPIO4_DATA | (1<<17)) : (GPIO4_DATA & ~(1<<17));
       GPIO4_DATA = reg;
       dir =  GPIO4_DIR | 1<<17;
       GPIO4_DIR=dir;

}
#endif

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
	int  internal_wifi=0, pcie_ext=0;

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
	   //printk("> Setup PCIe Gpios for onboard Wifi Card\n");	   
	   printk("PCIE:  Configure for onboard Wifi Card\n");
	   imx_iomux_v3_setup_multiple_pads(pcie_wifi_pads, ARRAY_SIZE(pcie_wifi_pads));
	   gpio_request(PCIE_CLKREQ,                  "PCIE_CLKREQ");
	   gpio_request(PCIE_WL_POWERDOWN,            "PCIE_WL_POWERDOWN");
	   gpio_request(PCIE_WAKE,                    "PCIE_WAKE");
	   gpio_request(PCIE_RESET,                   "PCIE_RESET");
	   gpio_request(PCIE_W_DISABLE_GPIO,          "PCIE_W_DISABLE_GPIO");
	   
	   gpio_direction_output(PCIE_CLKREQ,          0); // Switch on PCIE CLKREQ
	   gpio_direction_output(PCIE_WL_POWERDOWN,    0); // Switch on Wifi Module
	   gpio_direction_output(PCIE_W_DISABLE_GPIO,  0); // Do not disable PCIe
	   gpio_direction_input( PCIE_WAKE);               // Take Wake as input
	   gpio_direction_output(PCIE_RESET,           0); // Activate Reset
	   udelay(1000);
	   gpio_direction_output(PCIE_WL_POWERDOWN,    1); // Switch on Wifi Module
	   udelay(1000);	
	   gpio_direction_output(PCIE_W_DISABLE_GPIO,  1); // Do not disable PCIe
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
  
int board_init(void)
{
	char text[256];
  //    init_gpio4_17(0);  
  //    printf("GPIO4.17 mux 0x%x, GPIO4.17 pad 0x%x \n", iomux_sai1_txd5, iopad_sai1_txd5);
	kuk_GetArticleNo( &text[0], 24);
	printf("Modul: %s\n", text);  
	kuk_GetDescription( &text[0], sizeof( text));
	printf("%s\n", text);  

#ifdef CONFIG_FEC_MXC
	if ( kuk_GetPeripheral( KUK_PERIPHERAL_ETHERNET) != KUK_ETHERNET_NONE)
	{		
		setup_fec();
	}
#endif
	//init_camera_ov5640();	
	//setup_pcie(); /* environment not read here */

#ifdef CONFIG_SYS_I2C_MXC
	setup_i2c(0, CONFIG_SYS_I2C_SPEED, 0x7f, &i2c_pad_info1);	
	//setup_i2c(1, CONFIG_SYS_I2C_SPEED, 0x7f, &i2c_pad_info2);
#endif
	/* Enable Uart4/Uart2 pre-set in imx8mm_bl31_setup.c bl31_early_platform_setup2() */
	setup_periph2mcu();
	
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

#ifdef CONFIG_FSL_FASTBOOT
#ifdef CONFIG_ANDROID_RECOVERY
int is_recovery_key_pressing(void)
{
	return 0; /*TODO*/
}
#endif /*CONFIG_ANDROID_RECOVERY*/
#endif /*CONFIG_FSL_FASTBOOT*/

#ifdef CONFIG_VIDEO_MXS

#define FPGA_ID 0x41
#define LVDS_ID 0x2C

#define ADV7535_MAIN 0x39
#define ADV7535_DSI_CEC 0x3C

static const struct sec_mipi_dsim_plat_data imx8mm_mipi_dsim_plat_data = {
	.version	= 0x1060200,
	.max_data_lanes = 4,
	.max_data_rate  = 1500000000ULL,
	.reg_base = MIPI_DSI_BASE_ADDR,
	.gpr_base = CSI_BASE_ADDR + 0x8000,
};

static void fpga_init(uint8_t val)
{
	struct udevice *bus, *main_dev;
	int i2c_bus = 2;
	int ret;

	ret = uclass_get_device_by_seq(UCLASS_I2C, i2c_bus, &bus);
	if (ret) {
		printf("%s: No bus %d\n", __func__, i2c_bus);
		return;
	}
	
	ret = dm_i2c_probe(bus, FPGA_ID, 0, &main_dev);
	if (ret) {
		printf("%s: Can't find device id=0x%x, on bus %d\n",
			__func__, FPGA_ID, i2c_bus);
		return;
	}

	ret = dm_i2c_write(main_dev, 0x10, &val, 1);
}

static void lvds_init(struct display_info_t const *dev)
{
	struct udevice *bus, *main_dev;
	int i2c_bus = 2;
	int ret;
	uint8_t i = 0;
	uint8_t addr[] = { 0x09, 0x0A, 0x0A, 0x0B, 0x0D, 0x10, 0x11, 0x12, 0x13, 0x18, 0x19, 0x1A, 0x1B, 0x20, 0x21, 0x22, 0x23,
					   0x24, 0x25, 0x26, 0x27, 0x28, 0x29 ,0x2A, 0x2B, 0x2C, 0x2D, 0x2E, 0x2F, 0x30, 0x31, 0x32, 0x33,
					   0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 0x3A, 0x3B, 0x3C, 0x3D, 0x3E, 0x0D, 0xFF};
	uint8_t valb[] = { 0x01, 0x00, 0x05, 0x10, 0x00, 0x26, 0x00, 0x27, 0x00, 0x78, 0x00, 0x03, 0x00, 0x00, 0x04, 0x00, 0x00,
					   0x00, 0x03, 0x00, 0x00, 0x21, 0x00 ,0x00, 0x00, 0x08, 0x00, 0x00, 0x00, 0x15, 0x00, 0x00, 0x00,
					   0x9c, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0xFF};
					   
	if(strncmp(dev->mode.name, "IPANT10", 7) == 0)
	{
		//DataImage
		valb[7]  = 0x2A;	
		valb[14] = 0x05;
		valb[18] = 0x00;
		valb[25] = 0x3C;	
		valb[29] = 0x03;
		valb[33] = 0x30;
    }
	else if(strncmp(dev->mode.name, "PCONXS", 6) == 0)
	{
		//printf("%s: -> PCONXS\n", __func__);
		valb[2]  = 0x03;	
		valb[3] = 0x14;
		valb[5] = 0x20;
		valb[7] = 0x3D;	
		valb[17] = 0x58;
		valb[18] = 0x02;
		valb[21] = 0x20;
		valb[25] = 0x04;
		valb[29] = 0x01;
		valb[33] = 0xA0;
		valb[37] = 0xA0;
		valb[39] = 0x0C;
    }

	ret = uclass_get_device_by_seq(UCLASS_I2C, i2c_bus, &bus);
	if (ret) {
		printf("%s: No bus %d\n", __func__, i2c_bus);
		return;
	}
	
	ret = dm_i2c_probe(bus, LVDS_ID, 0, &main_dev);
	if (ret) {
		printf("%s: Can't find device id=0x%x, on bus %d\n",
			__func__, LVDS_ID, i2c_bus);
		return;
	}
	
	do{
		//printf("[0x%x] = 0x%x\n", addr[i], valb[i]);
		dm_i2c_write(main_dev, addr[i], &valb[i], 1);
		i++;
	}while(addr[i] != 0xFF);

	
	return;
}

static int adv7535_i2c_reg_write(struct udevice *dev, uint addr, uint mask, uint data)
{
	uint8_t valb;
	int err;

	if (mask != 0xff) {
		err = dm_i2c_read(dev, addr, &valb, 1);
		if (err)
			return err;

		valb &= ~mask;
		valb |= data;
	} else {
		valb = data;
	}

	err = dm_i2c_write(dev, addr, &valb, 1);
	return err;
}

static int adv7535_i2c_reg_read(struct udevice *dev, uint8_t addr, uint8_t *data)
{
	uint8_t valb;
	int err;

	err = dm_i2c_read(dev, addr, &valb, 1);
	if (err)
		return err;

	*data = (int)valb;
	return 0;
}

typedef struct stADV753XReg {
	uint8_t addr;
	uint8_t reg;
	uint8_t val;
} ADV753XREG, *PADV753XREG;

static ADV753XREG advinit[] = {
{0x39, 0x41, 0x10},
{0xff, 0x05, 0x00},
{0x39, 0xd6, 0x48},
{0xff, 0x05, 0x00},
{0x3c, 0x03, 0x89},
{0x39, 0x16, 0x20},
{0x39, 0x9a, 0xe0},
{0x39, 0xba, 0x70},
{0x39, 0xde, 0x82},
{0x39, 0xe4, 0xc0},
{0x39, 0xe5, 0x80},
{0x3c, 0x15, 0xd0},
{0x3c, 0x17, 0xd0},
{0x3c, 0x24, 0x20},
{0x3c, 0x57, 0x11},
{0x39, 0xaf, 0x06},
{0x39, 0x40, 0x80},
{0x39, 0x4c, 0x04},
{0x39, 0x49, 0x02},
{0x39, 0x0d, 0x40},
{0x3c, 0x1c, 0x40},
{0x39, 0x17, 0x02},
{0x3c, 0x16, 0x00},
{0x3c, 0x27, 0xCB},
{0x3c, 0x28, 0x89},
{0x3c, 0x29, 0x80},
{0x3c, 0x2a, 0x02},
{0x3c, 0x2b, 0xc0},
{0x3c, 0x2c, 0x05},
{0x3c, 0x2d, 0x80},
{0x3c, 0x2e, 0x09},
{0x3c, 0x2f, 0x40},
{0x3c, 0x30, 0x46},
{0x3c, 0x31, 0x50},
{0x3c, 0x32, 0x00},
{0x3c, 0x33, 0x50},
{0x3c, 0x34, 0x00},
{0x3c, 0x35, 0x40},
{0x3c, 0x36, 0x02},
{0x3c, 0x37, 0x40},
{0x3c, 0x27, 0xCB},
{0x3c, 0x27, 0x8B},
{0xff, 0x05, 0x00},
{0x3c, 0x27, 0xCB},
{0xff, 0x64, 0x00},
{0x3c, 0x55, 0x00},
{0x3c, 0x03, 0x09},
{0xff, 0x05, 0x00},
{0x3c, 0x03, 0x89},
{0x39, 0x12, 0x20},
{0x39, 0x13, 0x00},
{0x39, 0x14, 0x02},
{0x39, 0x15, 0x20},
{0x39, 0x0a, 0x41},
{0x39, 0x0c, 0xbc},
{0x39, 0x0d, 0x18},
{0x39, 0x03, 0x00},
{0x39, 0x02, 0x18},
{0x39, 0x01, 0x00},
{0x39, 0x09, 0x70},
{0x39, 0x08, 0x62},
{0x39, 0x07, 0x00},
{0x39, 0x73, 0x01},
{0x39, 0x76, 0x00},
{0x3c, 0x05, 0xC8},
{0x00, 0x00, 0x00}
};

static int adv7535_init(struct display_info_t const *dev)
{
	struct udevice *bus, *main_dev, *cec_dev;
	int i2c_bus = 1;
	int ret;
	uint8_t val;
	PADV753XREG padvseq = &advinit[0];

	for(i2c_bus = 1; i2c_bus <= 2; i2c_bus++)
	{ 
		ret = uclass_get_device_by_seq(UCLASS_I2C, i2c_bus, &bus);
		if (ret) {
			//printf("%s: No bus %d\n", __func__, i2c_bus);
			continue;
		}

		ret = dm_i2c_probe(bus, ADV7535_MAIN, 0, &main_dev);
		if (ret) {
			//printf("%s: Can't find device id=0x%x, on bus %d\n",
			//	__func__, ADV7535_MAIN, i2c_bus);
			continue;
		}

		ret = dm_i2c_probe(bus, ADV7535_DSI_CEC, 0, &cec_dev);
		if (ret) {
			//printf("%s: Can't find device id=0x%x, on bus %d\n",
			//	__func__, ADV7535_MAIN, i2c_bus);
			continue;
		}

		if(!ret)
		{
			break;
		}
	}
	if (ret)
	{
		//printf("%s: Can't find ADV7535\n", __func__);
		return 0;
	}

	//printf("%s: ADV7535 found\n", __func__);
	adv7535_i2c_reg_read(main_dev, 0x00, &val);
	if ( val != 0x14) {
		printf("Chip revision: 0x%x (expected: 0x14)\n", val);
		return 0;
	}
	adv7535_i2c_reg_read(cec_dev, 0x00, &val);
	if ( val != 0x75) {
		printf("Chip ID MSB: 0x%x (expected: 0x75)\n", val);
		return 0;
	}
	adv7535_i2c_reg_read(cec_dev, 0x01, &val);
	if ( val != 0x33) {
		printf("Chip ID LSB: 0x%x (expected: 0x33)\n", val);
		return 0;
	}
	
	while( padvseq->addr != 0)
	{
		if ( padvseq->addr == 0xFF )
		{
			mdelay( padvseq->reg);
		}else
		if ( padvseq->addr == ADV7535_MAIN)
		{
			adv7535_i2c_reg_write(main_dev, padvseq->reg, 0xff, padvseq->val);
		}else
		if ( padvseq->addr == ADV7535_DSI_CEC)
		{
			adv7535_i2c_reg_write(cec_dev, padvseq->reg, 0xff, padvseq->val);
		}else
		{
			printf("Unknown ADV7535 address\n");
		}
		padvseq++;
	}

	return 1;
}

#define DISPLAY_MIX_SFT_RSTN_CSR		0x00
#define DISPLAY_MIX_CLK_EN_CSR		0x04

   /* 'DISP_MIX_SFT_RSTN_CSR' bit fields */
#define BUS_RSTN_BLK_SYNC_SFT_EN	BIT(6)

   /* 'DISP_MIX_CLK_EN_CSR' bit fields */
#define LCDIF_PIXEL_CLK_SFT_EN		BIT(7)
#define LCDIF_APB_CLK_SFT_EN		BIT(6)

void disp_mix_bus_rstn_reset(ulong gpr_base, bool reset)
{
	if (!reset)
		/* release reset */
		setbits_le32(gpr_base + DISPLAY_MIX_SFT_RSTN_CSR, BUS_RSTN_BLK_SYNC_SFT_EN);
	else
		/* hold reset */
		clrbits_le32(gpr_base + DISPLAY_MIX_SFT_RSTN_CSR, BUS_RSTN_BLK_SYNC_SFT_EN);
}

void disp_mix_lcdif_clks_enable(ulong gpr_base, bool enable)
{
	if (enable)
		/* enable lcdif clks */
		setbits_le32(gpr_base + DISPLAY_MIX_CLK_EN_CSR, LCDIF_PIXEL_CLK_SFT_EN | LCDIF_APB_CLK_SFT_EN);
	else
		/* disable lcdif clks */
		clrbits_le32(gpr_base + DISPLAY_MIX_CLK_EN_CSR, LCDIF_PIXEL_CLK_SFT_EN | LCDIF_APB_CLK_SFT_EN);
}

struct mipi_dsi_client_dev kuk_panel_drv = {
	.channel = 0,
	.lanes   = 2,
	.format  = MIPI_DSI_FMT_RGB888,
	.mode_flags = MIPI_DSI_MODE_VIDEO | MIPI_DSI_MODE_VIDEO_SYNC_PULSE ,
	.name = "default",
};

struct mipi_dsi_client_dev adv7535_dev = {
	.channel	= 0,
	.lanes = 4,
	.format  = MIPI_DSI_FMT_RGB888,
	.mode_flags = MIPI_DSI_MODE_VIDEO | MIPI_DSI_MODE_VIDEO_SYNC_PULSE |
			  MIPI_DSI_MODE_EOT_PACKET | MIPI_DSI_MODE_VIDEO_HSE,
	.name = "ADV7535",
};


#define FSL_SIP_GPC			0xC2000000
#define FSL_SIP_CONFIG_GPC_PM_DOMAIN	0x3
#define DISPMIX				9
#define MIPI				10

void do_enable_mipi2rgb(struct display_info_t const *dev)
{	
	printf("%s: Enable %s \n", __func__, dev->mode.name);

	gpio_request(IMX_GPIO_NR(1, 5), "DISPLAY_EN");
	gpio_direction_output(IMX_GPIO_NR(1, 5), 1);
	
	gpio_request(IMX_GPIO_NR(1, 1), "BACKLIGHT_PWM");
	gpio_direction_output(IMX_GPIO_NR(1, 1), 1);
	
	gpio_request(IMX_GPIO_NR(3, 22), "BACKLIGHT_EN");
	gpio_direction_output(IMX_GPIO_NR(3, 22), 1);

	fpga_init(0x8D);

	/* enable the dispmix & mipi phy power domain */
	call_imx_sip(FSL_SIP_GPC, FSL_SIP_CONFIG_GPC_PM_DOMAIN, DISPMIX, true, 0);
	call_imx_sip(FSL_SIP_GPC, FSL_SIP_CONFIG_GPC_PM_DOMAIN, MIPI, true, 0);

	/* Put lcdif out of reset */
	disp_mix_bus_rstn_reset(imx8mm_mipi_dsim_plat_data.gpr_base, false);
	disp_mix_lcdif_clks_enable(imx8mm_mipi_dsim_plat_data.gpr_base, true);

	/* Setup mipi dsim */
	sec_mipi_dsim_setup(&imx8mm_mipi_dsim_plat_data);

	imx_mipi_dsi_bridge_attach(&kuk_panel_drv);
}

void do_display_default(struct display_info_t const *dev)
{
    int module;
    module = kuk_GetModule();

    switch( module) {
        case KUK_MODULE_SBCSOM8MINI:
        case KUK_MODULE_SBCSOM8NANO:
           	gpio_request(IMX_GPIO_NR(3, 22), "BACKLIGHT_ENABLE");
   	        gpio_request(IMX_GPIO_NR(1, 1), "BACKLIGHT_PWM");
           	gpio_request(IMX_GPIO_NR(1, 5), "DISPLAY_ENABLE");

	        gpio_direction_output(IMX_GPIO_NR(3, 22), 0);
	        gpio_direction_output(IMX_GPIO_NR(1, 1), 0);
	        gpio_direction_output(IMX_GPIO_NR(1, 5), 0);
            break;
        default:
            break;
    }
}

void do_enable_mipi2lvds(struct display_info_t const *dev)
{
	printf("%s: Enable %s \n", __func__, dev->mode.name);
		
	gpio_request(IMX_GPIO_NR(1, 5), "DISPLAY_EN");
	gpio_direction_output(IMX_GPIO_NR(1, 5), 1);
	
	gpio_request(IMX_GPIO_NR(1, 4), "LVDS EN");
	gpio_direction_output(IMX_GPIO_NR(1, 4), 1);
	
	gpio_request(IMX_GPIO_NR(1, 1), "BACKLIGHT PWM");
	gpio_direction_output(IMX_GPIO_NR(1, 1), 1);
	//gpio_direction_output(IMX_GPIO_NR(1, 1), 0);
	
	gpio_request(IMX_GPIO_NR(3, 22), "BACKLIGHT EN");
	gpio_direction_output(IMX_GPIO_NR(3, 22), 1);

	lvds_init(dev);
	
	/* enable the dispmix & mipi phy power domain */
	call_imx_sip(FSL_SIP_GPC, FSL_SIP_CONFIG_GPC_PM_DOMAIN, DISPMIX, true, 0);
	call_imx_sip(FSL_SIP_GPC, FSL_SIP_CONFIG_GPC_PM_DOMAIN, MIPI, true, 0);

	/* Put lcdif out of reset */
	disp_mix_bus_rstn_reset(imx8mm_mipi_dsim_plat_data.gpr_base, false);
	disp_mix_lcdif_clks_enable(imx8mm_mipi_dsim_plat_data.gpr_base, true);

	/* Setup mipi dsim */
	sec_mipi_dsim_setup(&imx8mm_mipi_dsim_plat_data);
	
	imx_mipi_dsi_bridge_attach(&kuk_panel_drv);
	
}

void do_enable_mipi2hdmi(struct display_info_t const *dev)
{
	/* ADV7353 initialization */
	adv7535_init(dev);

	/* enable the dispmix & mipi phy power domain */
	call_imx_sip(FSL_SIP_GPC, FSL_SIP_CONFIG_GPC_PM_DOMAIN, DISPMIX, true, 0);
	call_imx_sip(FSL_SIP_GPC, FSL_SIP_CONFIG_GPC_PM_DOMAIN, MIPI, true, 0);

	/* Put lcdif out of reset */
	disp_mix_bus_rstn_reset(imx8mm_mipi_dsim_plat_data.gpr_base, false);
	disp_mix_lcdif_clks_enable(imx8mm_mipi_dsim_plat_data.gpr_base, true);

	/* Setup mipi dsim */
	sec_mipi_dsim_setup(&imx8mm_mipi_dsim_plat_data);
	imx_mipi_dsi_bridge_attach(&adv7535_dev); /* attach adv7535 device */
}

#define FOCALTECH_ID 0x38
#define DATAIMAGE_ID 0x5C
#define GOODIX_ID    0x5D //0x14

static int detect_ipant7(struct display_info_t const *dev)
{
	struct udevice *bus, *main_dev;
	int i2c_bus = 1;
	int ret;

	//printf("%s:  %s \n", __func__, dev->mode.name);
	
	gpio_request(IMX_GPIO_NR(3, 23), "TOUCH EN");
	gpio_direction_output(IMX_GPIO_NR(3, 23), 1);
	
	mdelay(10);

	ret = uclass_get_device_by_seq(UCLASS_I2C, i2c_bus, &bus);
	if (ret) {
		//printf("%s: No bus %d\n", __func__, i2c_bus);
		return 0;
	}
	
	ret = dm_i2c_probe(bus, FOCALTECH_ID, 0, &main_dev);
	if (ret) {
		//printf("%s: Can't find device id=0x%x, on bus %d\n",
		//	__func__, FOCALTECH_ID, i2c_bus);
		return 0;
	}
	
	kuk_panel_drv.lanes = 2;
	kuk_panel_drv.name = "IPANT7";
	
	return 1;
}

static int detect_ipant10(struct display_info_t const *dev)
{
	
	struct udevice *bus, *main_dev;
	int i2c_bus = 1;
	int ret;

	//printf("%s:  %s \n", __func__, dev->mode.name);

	gpio_request(IMX_GPIO_NR(3, 23), "TOUCH EN");
	gpio_direction_output(IMX_GPIO_NR(3, 23), 0);

	mdelay(10);

	ret = uclass_get_device_by_seq(UCLASS_I2C, i2c_bus, &bus);
	if (ret) {
		//printf("%s: No bus %d\n", __func__, i2c_bus);
		return 0;
	}
	
	ret = dm_i2c_probe(bus, DATAIMAGE_ID, 0, &main_dev);
	if (ret) {
		//printf("%s: Can't find device id=0x%x, on bus %d\n",
		//	__func__, DATAIMAGE_ID, i2c_bus);
		return 0;
	}
		
	kuk_panel_drv.lanes = 4;
	kuk_panel_drv.name = "IPANT10";

	return 1;
}

static int detect_pconxs(struct display_info_t const *dev)
{
	struct udevice *bus, *main_dev;
	int i2c_bus = 1;
	int ret;
	
	gpio_request(IMX_GPIO_NR(3, 23), "TOUCH_EN");
	gpio_direction_output(IMX_GPIO_NR(3, 23), 0);

	gpio_request(IMX_GPIO_NR(1, 5), "DISPLAY_EN");
	gpio_direction_output(IMX_GPIO_NR(1, 5), 1);
	
	gpio_request(IMX_GPIO_NR(4, 14), "BL_EN");
	gpio_direction_output(IMX_GPIO_NR(4, 14), 1);

	gpio_request(IMX_GPIO_NR(1, 1), "BACKLIGHT_PWM");
	gpio_direction_output(IMX_GPIO_NR(1, 1), 1);
	//gpio_direction_output(IMX_GPIO_NR(1, 1), 0);

	mdelay(10);
	

	ret = uclass_get_device_by_seq(UCLASS_I2C, i2c_bus, &bus);
	if (ret) {
		//printf("%s: No bus %d\n", __func__, i2c_bus);
		return 0;
	}
	
	ret = dm_i2c_probe(bus, GOODIX_ID, 0, &main_dev);
	if (ret) {
		//printf("%s: Can't find device id=0x%x, on bus %d\n",
		//	__func__, GOODIX_ID, i2c_bus);
		return 0;
	}
	
	kuk_panel_drv.lanes = 4;
	kuk_panel_drv.name = "PCONXS";
	
	return 1;
}

/*
void board_quiesce_devices(void)
{
	gpio_request(IMX_GPIO_NR(1, 8), "DSI EN");
	gpio_direction_output(IMX_GPIO_NR(1, 8), 0);
}
*/

static int detect_display(struct display_info_t const *dev)
{
	struct udevice *bus;
	int ret;
	int i2c_bus = 1;
	char *s;

	s = env_get("display");
	if( s )
	{
		//printk("%s: Environment display=%s \n", __func__, s);
		if(!(strncmp(s, dev->mode.name, sizeof(s))))
		{
			//printf("%s: Choose %s \n", __func__, dev->mode.name);

			ret = uclass_get_device_by_seq(UCLASS_I2C, i2c_bus, &bus);
			if (ret) {
				//printf("%s: No bus %d\n", __func__, i2c_bus);
				return 0;
			}

			if(!(strncmp(s, "IPANT10", sizeof(s))))
			{
				gpio_request(IMX_GPIO_NR(3, 23), "TOUCH EN");
				gpio_direction_output(IMX_GPIO_NR(3, 23), 0);

				kuk_panel_drv.lanes = 4;
				kuk_panel_drv.name = "IPANT10";
			}
			else if(!(strncmp(s, "IPANT7", sizeof(s))))
			{
				gpio_request(IMX_GPIO_NR(3, 23), "TOUCH EN");
				gpio_direction_output(IMX_GPIO_NR(3, 23), 1);

				kuk_panel_drv.lanes = 2;
				kuk_panel_drv.name = "IPANT7";
			}
			else if(!(strncmp(s, "PCONXS", sizeof(s))))
			{
				gpio_request(IMX_GPIO_NR(3, 23), "TOUCH EN");
				gpio_direction_output(IMX_GPIO_NR(3, 23), 0);

				kuk_panel_drv.lanes = 4;
				kuk_panel_drv.name = "PCONXS";
			}

			return 1;
		}
		if(strncmp(s, "auto", sizeof(s)))
		{
			return 0;
		}
	}
	//printk("%s: Try autodetect... %s \n", __func__, dev->mode.name);

	if(!(strncmp(dev->mode.name, "IPANT7", sizeof(dev->mode.name))))
	{
		return detect_ipant7(dev);
	}
	else if(!(strncmp(dev->mode.name, "IPANT10", sizeof(dev->mode.name))))
	{
		return detect_ipant10(dev);
	}
	else if(!(strncmp(dev->mode.name, "PCONXS", sizeof(dev->mode.name))))
	{
		return detect_pconxs(dev);
	}

	return 0;
}

struct display_info_t const displays[] = {{
	.bus = LCDIF_BASE_ADDR,
	.addr = 0,
	.pixfmt = 24,
	.detect = detect_display,
	.enable	= do_display_default,
	.mode	= {
		.name			= "MIPI2KUK",
		.refresh		= 60,
		.xres			= 1024,
		.yres			= 768,
		.pixclock		= 16835, /* 59400000 // 65000000 */
		.left_margin	= 156,
		.right_margin	= 156,
		.upper_margin	= 21,
		.lower_margin	= 7,
		.hsync_len		= 8,
		.vsync_len		= 10,
		.sync			= FB_SYNC_EXT,
		.vmode			= FB_VMODE_NONINTERLACED
} }, {
	.bus = LCDIF_BASE_ADDR,
	.addr = 0,
	.pixfmt = 24,
	.detect = detect_display,
	.enable	= do_enable_mipi2lvds,
	.mode	= {
		.name			= "PCONXS",
		.refresh		= 60,
		.xres			= 1024,
		.yres			= 600,
		.pixclock		= 16835, /* 59400000 // 65000000 */
		.left_margin	= 156,
		.right_margin	= 156,
		.upper_margin	= 21,
		.lower_margin	= 7,
		.hsync_len		= 8,
		.vsync_len		= 10,
		.sync			= FB_SYNC_EXT,
		.vmode			= FB_VMODE_NONINTERLACED
} }, {
	.bus = LCDIF_BASE_ADDR,
	.addr = 0,
	.pixfmt = 24,
	.detect = detect_display,
	.enable	= do_enable_mipi2rgb,
	.mode	= {
		.name			= "IPANT7",
		.refresh		= 60,
		.xres			= 800,
		.yres			= 480,
		.pixclock		= 33300, /* 33300000 */
		.left_margin	= 40,
		.right_margin	= 210,
		.upper_margin	= 20,
		.lower_margin	= 22,
		.hsync_len		= 6,
		.vsync_len		= 3,
		.sync			= FB_SYNC_EXT,
		.vmode			= FB_VMODE_NONINTERLACED
} }, {
	.bus = LCDIF_BASE_ADDR,
	.addr = 0,
	.pixfmt = 24,
	.detect = detect_display,
	.enable	= do_enable_mipi2lvds,
	.mode	= {
		.name			= "IPANT10",
		.refresh		= 60,
		.xres			= 1280,
		.yres			= 800,
		.pixclock		= 24390, /* 41000000 */
		.left_margin	= 48,
		.right_margin	= 52,
		.upper_margin	= 10,
		.lower_margin	= 10,
		.hsync_len		= 60,
		.vsync_len		= 3,
		.sync			= FB_SYNC_EXT,
		.vmode			= FB_VMODE_NONINTERLACED
} }, {
	
	.bus = LCDIF_BASE_ADDR,
	.addr = 0,
	.pixfmt = 24,
	.detect = detect_display,
	.enable	= do_enable_mipi2hdmi,
	.mode	= {
		.name			= "MIPI2HDMI",
		.refresh		= 60,
		.xres			= 1920,
		.yres			= 1080,
		.pixclock		= 6734, /* 148500000 */
		.left_margin	= 148,
		.right_margin	= 88,
		.upper_margin	= 36,
		.lower_margin	= 4,
		.hsync_len		= 44,
		.vsync_len		= 5,
		.sync			= FB_SYNC_EXT,
		.vmode			= FB_VMODE_NONINTERLACED
} }
};
size_t display_count = ARRAY_SIZE(displays);
#endif


