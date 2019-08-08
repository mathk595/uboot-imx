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

#define TRIZEPS8_USE_RESET_OUT_AS_WATCHDOG_OUT	0	/* NXP i.MX8MQ EVK uses RESET_OUT as Watchdog Out */
#define TRIZEPS8_CONTROL_RESET_OUT	0				/* 0: RESET_OUT is controlled by Kinetis MCU; 1: use when no Kinetis MCU */


DECLARE_GLOBAL_DATA_PTR;

#define UART_PAD_CTRL	(PAD_CTL_DSE6 | PAD_CTL_FSEL1)
#define WDOG_PAD_CTRL	(PAD_CTL_DSE6 | PAD_CTL_ODE | PAD_CTL_PUE | PAD_CTL_PE)
#define GPIO_PAD_CTRL   (PAD_CTL_DSE6 )

static iomux_v3_cfg_t const uart_pads[] = {
	IMX8MM_PAD_UART1_RXD_UART1_RX | MUX_PAD_CTRL(UART_PAD_CTRL),
	IMX8MM_PAD_UART1_TXD_UART1_TX | MUX_PAD_CTRL(UART_PAD_CTRL),
};

#define PCIE_WL_POWERDOWN       IMX_GPIO_NR(3, 5)
#define PCIE_W_DISABLE_GPIO     IMX_GPIO_NR(3,17)
#define PCIE_RESET              IMX_GPIO_NR(2,19)
#define PCIE_WAKE               IMX_GPIO_NR(3, 2)

#define PCIE_W_DISABLE_GPIO_EXT IMX_GPIO_NR(4,15)  
#define PCIE_RESET_EXT          IMX_GPIO_NR(4,17)
#define PCIE_WAKE_EXT           IMX_GPIO_NR(3,13)  

  
static iomux_v3_cfg_t const pcie_wifi_pads[] = {
        IMX8MM_PAD_I2C4_SCL_PCIE1_CLKREQ_B  | MUX_PAD_CTRL(0x61),          // CLKREQ_B
	IMX8MM_PAD_NAND_WE_B_GPIO3_IO17     | MUX_PAD_CTRL(GPIO_PAD_CTRL), // PCIE_W_DISABLE_GPIO
	IMX8MM_PAD_SD2_RESET_B_GPIO2_IO19   | MUX_PAD_CTRL(GPIO_PAD_CTRL), // PCIE_RESET
	IMX8MM_PAD_NAND_CLE_GPIO3_IO5       | MUX_PAD_CTRL(GPIO_PAD_CTRL), // PCIE_WL_POWERDOWN
	IMX8MM_PAD_NAND_CE1_B_GPIO3_IO2     | MUX_PAD_CTRL(GPIO_PAD_CTRL), // PCIE_WAKE
};

static iomux_v3_cfg_t const pcie_ext_pads[] = {
        IMX8MM_PAD_I2C4_SCL_PCIE1_CLKREQ_B  | MUX_PAD_CTRL(0x61),          // CLKREQ_B
	IMX8MM_PAD_SAI1_TXD3_GPIO4_IO15     | MUX_PAD_CTRL(GPIO_PAD_CTRL), // PCIE_W_DISABLE_GPIO
	IMX8MM_PAD_SAI1_TXD5_GPIO4_IO17     | MUX_PAD_CTRL(GPIO_PAD_CTRL), // PCIE_RESET
	IMX8MM_PAD_NAND_CLE_GPIO3_IO5       | MUX_PAD_CTRL(GPIO_PAD_CTRL), // PCIE_WL_POWERDOWN
	IMX8MM_PAD_NAND_DATA07_GPIO3_IO13   | MUX_PAD_CTRL(GPIO_PAD_CTRL), // PCIE_WAKE
};

#define USBH_PWR_GPIO IMX_GPIO_NR(1,14)  
#define OTG_PWR_GPIO  IMX_GPIO_NR(1,12)  
  
static iomux_v3_cfg_t const usb_pads[] = {
	IMX8MM_PAD_GPIO1_IO14_GPIO1_IO14 | MUX_PAD_CTRL(GPIO_PAD_CTRL),
	IMX8MM_PAD_GPIO1_IO12_GPIO1_IO12 | MUX_PAD_CTRL(GPIO_PAD_CTRL),	
};

#if TRIZEPS8_USE_RESET_OUT_AS_WATCHDOG_OUT
static iomux_v3_cfg_t const wdog_pads[] = {
	IMX8MM_PAD_GPIO1_IO02_WDOG1_WDOG_B  | MUX_PAD_CTRL(WDOG_PAD_CTRL),
};
#endif

int board_early_init_f(void)
{
#if TRIZEPS8_USE_RESET_OUT_AS_WATCHDOG_OUT
	struct wdog_regs *wdog = (struct wdog_regs *)WDOG1_BASE_ADDR;

	imx_iomux_v3_setup_multiple_pads(wdog_pads, ARRAY_SIZE(wdog_pads));

	set_wdog_reset(wdog);
#endif

	imx_iomux_v3_setup_multiple_pads(usb_pads, ARRAY_SIZE(usb_pads));
	imx_iomux_v3_setup_multiple_pads(uart_pads, ARRAY_SIZE(uart_pads));
	return 0;
}

#ifdef CONFIG_BOARD_POSTCLK_INIT
int board_postclk_init(void)
{
	/* TODO */
	return 0;
}
#endif

int dram_init(void)
{
	/* rom_pointer[1] contains the size of TEE occupies */
	if (rom_pointer[1])
		gd->ram_size = PHYS_SDRAM_SIZE - rom_pointer[1];
	else
		gd->ram_size = PHYS_SDRAM_SIZE;

	return 0;
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
};

#define FEC_RST_PAD IMX_GPIO_NR(1, 9)
#define FEC_MODE0   IMX_GPIO_NR(1,24)
#define FEC_MODE1   IMX_GPIO_NR(1,28)
#define FEC_MODE2   IMX_GPIO_NR(1,25)
#define FEC_MODE3   IMX_GPIO_NR(1,29)

static iomux_v3_cfg_t const fec1_rst_pads[] = {
        IMX8MM_PAD_GPIO1_IO09_GPIO1_IO9           | MUX_PAD_CTRL(NO_PAD_CTRL),
	IMX8MM_PAD_ENET_RX_CTL_GPIO1_IO24         | MUX_PAD_CTRL(NO_PAD_CTRL),
	IMX8MM_PAD_ENET_RD2_GPIO1_IO28            | MUX_PAD_CTRL(NO_PAD_CTRL),
	IMX8MM_PAD_ENET_RXC_GPIO1_IO25            | MUX_PAD_CTRL(NO_PAD_CTRL),
	IMX8MM_PAD_ENET_RD3_GPIO1_IO29            | MUX_PAD_CTRL(NO_PAD_CTRL),
	/*

	IMX8MM_PAD_ENET_MDC_ENET1_MDC	          | MUX_PAD_CTRL(	0x03),
	IMX8MM_PAD_ENET_MDIO_ENET1_MDIO           | MUX_PAD_CTRL(	0x03),
	IMX8MM_PAD_ENET_TD3_ENET1_RGMII_TD3       | MUX_PAD_CTRL(	0x1f),
	IMX8MM_PAD_ENET_TD2_ENET1_RGMII_TD2       | MUX_PAD_CTRL(	0x1f),
	IMX8MM_PAD_ENET_TD1_ENET1_RGMII_TD1       | MUX_PAD_CTRL(	0x1f),
	IMX8MM_PAD_ENET_TD0_ENET1_RGMII_TD0       | MUX_PAD_CTRL(	0x1f),
	IMX8MM_PAD_ENET_RD3_ENET1_RGMII_RD3       | MUX_PAD_CTRL(	0x91),
	IMX8MM_PAD_ENET_RD2_ENET1_RGMII_RD2       | MUX_PAD_CTRL(	0x91),
	IMX8MM_PAD_ENET_RD1_ENET1_RGMII_RD1       | MUX_PAD_CTRL(	0x91),
	IMX8MM_PAD_ENET_RD0_ENET1_RGMII_RD0       | MUX_PAD_CTRL(	0x91),
	IMX8MM_PAD_ENET_TXC_ENET1_RGMII_TXC       | MUX_PAD_CTRL(	0x1f),
	IMX8MM_PAD_ENET_RXC_ENET1_RGMII_RXC       | MUX_PAD_CTRL(	0x91),
	IMX8MM_PAD_ENET_RX_CTL_ENET1_RGMII_RX_CTL | MUX_PAD_CTRL(	0x91),
	IMX8MM_PAD_ENET_TX_CTL_ENET1_RGMII_TX_CTL | MUX_PAD_CTRL(	0x1f),
	*/
};

static void setup_iomux_fec(void)
{

	printf("%s: reset fec\n", __func__);
	imx_iomux_v3_setup_multiple_pads(fec1_rst_pads, ARRAY_SIZE(fec1_rst_pads));
#if 1
	gpio_request(FEC_MODE0,"fec_mode0");	gpio_direction_output(FEC_MODE0, 0);
	gpio_request(FEC_MODE1,"fec_mode1");	gpio_direction_output(FEC_MODE1, 0);
	gpio_request(FEC_MODE2,"fec_mode2");	gpio_direction_output(FEC_MODE2, 0);
	gpio_request(FEC_MODE3,"fec_mode3");	gpio_direction_output(FEC_MODE3, 0);
#endif
	gpio_request(FEC_RST_PAD, "fec1_rst");
	gpio_direction_output(FEC_RST_PAD, 0);
	udelay(500);
	gpio_direction_output(FEC_RST_PAD, 1);
#if 1
	udelay(1000);
	gpio_direction_output(FEC_RST_PAD, 0);
	udelay(1000);
	gpio_direction_output(FEC_RST_PAD, 1);
#endif
	imx_iomux_v3_setup_multiple_pads(fec1_nrst_pads, ARRAY_SIZE(fec1_nrst_pads));


}

static int setup_fec(void)
{
	struct iomuxc_gpr_base_regs *const iomuxc_gpr_regs
		= (struct iomuxc_gpr_base_regs *) IOMUXC_GPR_BASE_ADDR;
	/* Use 125M anatop REF_CLK1 for ENET1, not from external */
	setup_iomux_fec();
	printf("%s: Enable FEC CLK\n", __func__);
	clrsetbits_le32(&iomuxc_gpr_regs->gpr[1],IOMUXC_GPR_GPR1_GPR_ENET1_TX_CLK_SEL_SHIFT, 0);
	set_clk_enet(ENET_125MHZ);
	return 0;
}

#undef  MDIO_DEVAD_NONE
#define MDIO_DEVAD_NONE 4

int ethernet_1GB(void)
{
  char *s;
  int i;  
  s = env_get("ethspeed");
  i = strncmp(s,"1G", 2);
  if( i == 0 )
  {
      printf("ethspeed=1Gb\n");
      return(1);
  }else
  {
      printf("ethspeed=100Mb\n");
      return(0);
  }  
}
  

void board_phy_dump(struct phy_device *phydev)
{
  int i;
  printf("------------------%s-----------------------\n", __func__);
  
  for(i=0; i < 0x1F; i++ )
    printf("Phy Adr: 0x%02x -> 0x%02x\n", i, phy_read(phydev, MDIO_DEVAD_NONE, i));
  
}
 

int board_phy_config(struct phy_device *phydev)
{
  int i=100;
  
  /* enable rgmii rxc skew and phy mode select to RGMII copper */
#if 0
  // Reset Phy....
  phy_write(phydev, MDIO_DEVAD_NONE, 0x0, 0x8000);
  while( i-- && ((phy_read(phydev, MDIO_DEVAD_NONE, 0x00) & 0x8000)!=0))
    udelay(1000);
#endif

  if (phydev->drv->config)
    phydev->drv->config(phydev);

  phy_write(phydev, MDIO_DEVAD_NONE, 0x1F, 0x8040);
  phy_write(phydev, MDIO_DEVAD_NONE, 0x1d, 0x1f);
  phy_write(phydev, MDIO_DEVAD_NONE, 0x1e, 0x02);  
  phy_write(phydev, MDIO_DEVAD_NONE, 0x1d, 0x00);
  phy_write(phydev, MDIO_DEVAD_NONE, 0x1e, 0x82ee);
  phy_write(phydev, MDIO_DEVAD_NONE, 0x1d, 0x05);
  phy_write(phydev, MDIO_DEVAD_NONE, 0x1e, 0x100);

  if( ethernet_1GB() == 0 )
    phy_write(phydev, MDIO_DEVAD_NONE, 0x09, 0x000);    // Do not use 1GBit 
  else
    phy_write(phydev, MDIO_DEVAD_NONE, 0x09, 0x300);    // use 1GBit 
 
  i = phy_read(phydev, MDIO_DEVAD_NONE, 0x14);
  i &= ~0x1C;
  i |=  0x04;
  i &= ~0x02;
  printf("%s: Phy Write 0x14 0x%02x \n", __func__, i);
  phy_write(phydev, MDIO_DEVAD_NONE, 0x14, i);  
  
  phy_write(phydev, MDIO_DEVAD_NONE, 0x0,  0x3200);   // 100MBit restart AN
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
	   printk("%s: Environment pcie=%s ", __func__, s);
	 }else
	   printk("%s: Environment variable pcie not set. Skip Init PCIe gpios\n", __func__);	   

	 if( internal_wifi )
	 {
	   printk("> Setup PCIe Gpios for onboard Wifi Card\n");	   
	   imx_iomux_v3_setup_multiple_pads(pcie_wifi_pads, ARRAY_SIZE(pcie_wifi_pads));
	   gpio_request(PCIE_WL_POWERDOWN,            "PCIE_WL_POWERDOWN");
	   gpio_request(PCIE_WAKE,                    "PCIE_WAKE");
	   gpio_request(PCIE_RESET,                   "PCIE_RESET");
	   gpio_request(PCIE_W_DISABLE_GPIO,          "PCIE_W_DISABLE_GPIO");
	   gpio_direction_output(PCIE_WL_POWERDOWN,    1); // Switch on Wifi Module
	   gpio_direction_output(PCIE_W_DISABLE_GPIO,  1); // Do not disable PCIe
	   gpio_direction_input(PCIE_WAKE);                // Take Wake as input
	   gpio_direction_output(PCIE_RESET,           0); // Activate Reset
	   udelay(1000);	
	   gpio_direction_output(PCIE_RESET,           1);
	   udelay(500);	
	   gpio_direction_output(PCIE_RESET,           0); // Activate Reset again
	   udelay(500);	
	   gpio_direction_output(PCIE_RESET,           1);
	   // mx8qm_pcie_init();	   
	 }else
	 if( pcie_ext )	  
	 {
	   printk("> Setup PCIe Gpios for external PCIe Slot\n");	   	   
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

void pci_init_board(void)
{
  /* test the 1 lane mode of the PCIe A controller */
  printf("%s:***************************************************\n", __func__);
}
  
int board_init(void)
{

#ifdef CONFIG_FEC_MXC
	setup_fec();
#endif
	//setup_pcie(); /* environment not read here */
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

#define ADV7535_MAIN 0x3d
#define ADV7535_DSI_CEC 0x3c

static const struct sec_mipi_dsim_plat_data imx8mm_mipi_dsim_plat_data = {
	.version	= 0x1060200,
	.max_data_lanes = 4,
	.max_data_rate  = 1500000000ULL,
	.reg_base = MIPI_DSI_BASE_ADDR,
	.gpr_base = CSI_BASE_ADDR + 0x8000,
};

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

static void adv7535_init(void)
{
	struct udevice *bus, *main_dev, *cec_dev;
	int i2c_bus = 1;
	int ret;
	uint8_t val;

	ret = uclass_get_device_by_seq(UCLASS_I2C, i2c_bus, &bus);
	if (ret) {
		printf("%s: No bus %d\n", __func__, i2c_bus);
		return;
	}

	ret = dm_i2c_probe(bus, ADV7535_MAIN, 0, &main_dev);
	if (ret) {
		printf("%s: Can't find device id=0x%x, on bus %d\n",
			__func__, ADV7535_MAIN, i2c_bus);
		return;
	}

	ret = dm_i2c_probe(bus, ADV7535_DSI_CEC, 0, &cec_dev);
	if (ret) {
		printf("%s: Can't find device id=0x%x, on bus %d\n",
			__func__, ADV7535_MAIN, i2c_bus);
		return;
	}

	adv7535_i2c_reg_read(main_dev, 0x00, &val);
	debug("Chip revision: 0x%x (expected: 0x14)\n", val);
	adv7535_i2c_reg_read(cec_dev, 0x00, &val);
	debug("Chip ID MSB: 0x%x (expected: 0x75)\n", val);
	adv7535_i2c_reg_read(cec_dev, 0x01, &val);
	debug("Chip ID LSB: 0x%x (expected: 0x33)\n", val);

	/* Power */
	adv7535_i2c_reg_write(main_dev, 0x41, 0xff, 0x10);
	/* Initialisation (Fixed) Registers */
	adv7535_i2c_reg_write(main_dev, 0x16, 0xff, 0x20);
	adv7535_i2c_reg_write(main_dev, 0x9A, 0xff, 0xE0);
	adv7535_i2c_reg_write(main_dev, 0xBA, 0xff, 0x70);
	adv7535_i2c_reg_write(main_dev, 0xDE, 0xff, 0x82);
	adv7535_i2c_reg_write(main_dev, 0xE4, 0xff, 0x40);
	adv7535_i2c_reg_write(main_dev, 0xE5, 0xff, 0x80);
	adv7535_i2c_reg_write(cec_dev, 0x15, 0xff, 0xD0);
	adv7535_i2c_reg_write(cec_dev, 0x17, 0xff, 0xD0);
	adv7535_i2c_reg_write(cec_dev, 0x24, 0xff, 0x20);
	adv7535_i2c_reg_write(cec_dev, 0x57, 0xff, 0x11);
	/* 4 x DSI Lanes */
	adv7535_i2c_reg_write(cec_dev, 0x1C, 0xff, 0x40);

	/* DSI Pixel Clock Divider */
	adv7535_i2c_reg_write(cec_dev, 0x16, 0xff, 0x18);

	/* Enable Internal Timing Generator */
	adv7535_i2c_reg_write(cec_dev, 0x27, 0xff, 0xCB);
	/* 1920 x 1080p 60Hz */
	adv7535_i2c_reg_write(cec_dev, 0x28, 0xff, 0x89); /* total width */
	adv7535_i2c_reg_write(cec_dev, 0x29, 0xff, 0x80); /* total width */
	adv7535_i2c_reg_write(cec_dev, 0x2A, 0xff, 0x02); /* hsync */
	adv7535_i2c_reg_write(cec_dev, 0x2B, 0xff, 0xC0); /* hsync */
	adv7535_i2c_reg_write(cec_dev, 0x2C, 0xff, 0x05); /* hfp */
	adv7535_i2c_reg_write(cec_dev, 0x2D, 0xff, 0x80); /* hfp */
	adv7535_i2c_reg_write(cec_dev, 0x2E, 0xff, 0x09); /* hbp */
	adv7535_i2c_reg_write(cec_dev, 0x2F, 0xff, 0x40); /* hbp */

	adv7535_i2c_reg_write(cec_dev, 0x30, 0xff, 0x46); /* total height */
	adv7535_i2c_reg_write(cec_dev, 0x31, 0xff, 0x50); /* total height */
	adv7535_i2c_reg_write(cec_dev, 0x32, 0xff, 0x00); /* vsync */
	adv7535_i2c_reg_write(cec_dev, 0x33, 0xff, 0x50); /* vsync */
	adv7535_i2c_reg_write(cec_dev, 0x34, 0xff, 0x00); /* vfp */
	adv7535_i2c_reg_write(cec_dev, 0x35, 0xff, 0x40); /* vfp */
	adv7535_i2c_reg_write(cec_dev, 0x36, 0xff, 0x02); /* vbp */
	adv7535_i2c_reg_write(cec_dev, 0x37, 0xff, 0x40); /* vbp */

	/* Reset Internal Timing Generator */
	adv7535_i2c_reg_write(cec_dev, 0x27, 0xff, 0xCB);
	adv7535_i2c_reg_write(cec_dev, 0x27, 0xff, 0x8B);
	adv7535_i2c_reg_write(cec_dev, 0x27, 0xff, 0xCB);

	/* HDMI Output */
	adv7535_i2c_reg_write(main_dev, 0xAF, 0xff, 0x16);
	/* AVI Infoframe - RGB - 16-9 Aspect Ratio */
	adv7535_i2c_reg_write(main_dev, 0x55, 0xff, 0x02);
	adv7535_i2c_reg_write(main_dev, 0x56, 0xff, 0x0);

	/*  GC Packet Enable */
	adv7535_i2c_reg_write(main_dev, 0x40, 0xff, 0x0);
	/*  GC Colour Depth - 24 Bit */
	adv7535_i2c_reg_write(main_dev, 0x4C, 0xff, 0x0);
	/*  Down Dither Output Colour Depth - 8 Bit (default) */
	adv7535_i2c_reg_write(main_dev, 0x49, 0xff, 0x00);

	/* set low refresh 1080p30 */
	adv7535_i2c_reg_write(main_dev, 0x4A, 0xff, 0x80); /*should be 0x80 for 1080p60 and 0x8c for 1080p30*/

	/* HDMI Output Enable */
	adv7535_i2c_reg_write(cec_dev, 0xbe, 0xff, 0x3c);
	adv7535_i2c_reg_write(cec_dev, 0x03, 0xff, 0x89);
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

struct mipi_dsi_client_dev adv7535_dev = {
	.channel	= 0,
	.lanes = 4,
	.format  = MIPI_DSI_FMT_RGB888,
	.mode_flags = MIPI_DSI_MODE_VIDEO | MIPI_DSI_MODE_VIDEO_SYNC_PULSE |
			  MIPI_DSI_MODE_EOT_PACKET | MIPI_DSI_MODE_VIDEO_HSE,
	.name = "ADV7535",
};

struct mipi_dsi_client_dev rm67191_dev = {
	.channel	= 0,
	.lanes = 4,
	.format  = MIPI_DSI_FMT_RGB888,
	.mode_flags = MIPI_DSI_MODE_VIDEO | MIPI_DSI_MODE_VIDEO_SYNC_PULSE |
			  MIPI_DSI_MODE_EOT_PACKET | MIPI_DSI_MODE_VIDEO_HSE,
};

#define FSL_SIP_GPC			0xC2000000
#define FSL_SIP_CONFIG_GPC_PM_DOMAIN	0x3
#define DISPMIX				9
#define MIPI				10

void do_enable_mipi2hdmi(struct display_info_t const *dev)
{
	gpio_request(IMX_GPIO_NR(1, 8), "DSI EN");
	gpio_direction_output(IMX_GPIO_NR(1, 8), 1);

	/* ADV7353 initialization */
	adv7535_init();

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

void do_enable_mipi_led(struct display_info_t const *dev)
{
	gpio_request(IMX_GPIO_NR(1, 8), "DSI EN");
	gpio_direction_output(IMX_GPIO_NR(1, 8), 0);
	mdelay(100);
	gpio_direction_output(IMX_GPIO_NR(1, 8), 1);

	/* enable the dispmix & mipi phy power domain */
	call_imx_sip(FSL_SIP_GPC, FSL_SIP_CONFIG_GPC_PM_DOMAIN, DISPMIX, true, 0);
	call_imx_sip(FSL_SIP_GPC, FSL_SIP_CONFIG_GPC_PM_DOMAIN, MIPI, true, 0);

	/* Put lcdif out of reset */
	disp_mix_bus_rstn_reset(imx8mm_mipi_dsim_plat_data.gpr_base, false);
	disp_mix_lcdif_clks_enable(imx8mm_mipi_dsim_plat_data.gpr_base, true);

	/* Setup mipi dsim */
	sec_mipi_dsim_setup(&imx8mm_mipi_dsim_plat_data);

	rm67191_init();
	rm67191_dev.name = displays[1].mode.name;
	imx_mipi_dsi_bridge_attach(&rm67191_dev); /* attach rm67191 device */
}

void board_quiesce_devices(void)
{
	gpio_request(IMX_GPIO_NR(1, 8), "DSI EN");
	gpio_direction_output(IMX_GPIO_NR(1, 8), 0);
}

struct display_info_t const displays[] = {{
	.bus = LCDIF_BASE_ADDR,
	.addr = 0,
	.pixfmt = 24,
	.detect = NULL,
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

} }, {
	.bus = LCDIF_BASE_ADDR,
	.addr = 0,
	.pixfmt = 24,
	.detect = NULL,
	.enable	= do_enable_mipi_led,
	.mode	= {
		.name			= "RM67191_OLED",
		.refresh		= 60,
		.xres			= 1080,
		.yres			= 1920,
		.pixclock		= 7575, /* 132000000 */
		.left_margin	= 34,
		.right_margin	= 20,
		.upper_margin	= 4,
		.lower_margin	= 10,
		.hsync_len		= 2,
		.vsync_len		= 2,
		.sync			= FB_SYNC_EXT,
		.vmode			= FB_VMODE_NONINTERLACED

} } };
size_t display_count = ARRAY_SIZE(displays);
#endif


