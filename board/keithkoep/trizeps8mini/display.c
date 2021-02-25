/*
 * Copyright 2019 Keith & Koep GmbH
 */

/*
display detection routine for KuK hardware
try to detect the display/baseboard
based on the detection of the used touch or other board unique devices


*/

#include <common.h>
#include <i2c.h>
#include <dm/uclass.h>
#include <asm-generic/gpio.h>
#include <asm/mach-imx/gpio.h>
#include <asm/io.h>
#include <asm/arch/sys_proto.h>

#include <sec_mipi_dsim.h>
#include <imx_mipi_dsi_bridge.h>
#include <mipi_dsi_panel.h>
#include <asm/mach-imx/video.h>

#include "../common/kuk_boards.h"

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

	int module;
    module = kuk_GetModule();

	if(strncmp(dev->mode.name, "LVDS_SCF1001C44GGU05", sizeof(dev->mode.name)) == 0)
	{
		//DataImage
		valb[7]  = 0x2A;	
		valb[14] = 0x05;
		valb[18] = 0x00;
		valb[25] = 0x3C;	
		valb[29] = 0x03;
		valb[33] = 0x30;
    }
	else if(strncmp(dev->mode.name, "PCONXS", sizeof(dev->mode.name)) == 0)
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
	else if(strncmp(dev->mode.name, "LVDS_ATM0700D6J", sizeof(dev->mode.name)) == 0)
	{
		//printf("%s: -> IPANM7\n", __func__);
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
	else if(strncmp(dev->mode.name, "LVDS_ATM0700L61", sizeof(dev->mode.name)) == 0)
	{
		printf("%s: -> LVDS_ATM0700L61\n", __func__);
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

		gpio_request(IMX_GPIO_NR(3, 22), "BACKLIGHT EN");
		if(module == KUK_MODULE_TRIZEPS8MINI)
			gpio_direction_output(IMX_GPIO_NR(3, 22), 0);
		else
			gpio_direction_output(IMX_GPIO_NR(3, 22), 1);
	}
	else if (strncmp(dev->mode.name, "LVDS_AM19201080D1", sizeof(dev->mode.name)) == 0)
	{
		printf("%s: -> AM19201080D1\n", __func__);
		valb[2] = 0x83;
		valb[3] = 0x28;
		valb[7] = 0x48;
		valb[9] = 0x6C;	
		valb[13] = 0x80;
		valb[14] = 0x07;
		valb[17] = 0x38;
		valb[18] = 0x04;
		valb[25] = 0x23;
		valb[30] = 0x0A;
		valb[33] = 0x23;
		valb[37] = 0x46;
		valb[39] = 0x0A;
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
	.lanes   = 4,
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

	printf("%s: Enable %s \n", __func__, dev->mode.name);

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
		case KUK_MODULE_MYON2:
			gpio_request(IMX_GPIO_NR(1, 5), "DISPLAY_EN");
			gpio_request(IMX_GPIO_NR(1, 1), "BACKLIGHT_PWM");
			gpio_request(IMX_GPIO_NR(3, 22), "BACKLIGHT_EN");
			gpio_request(IMX_GPIO_NR(1, 4), "LVDS EN");

			gpio_direction_output(IMX_GPIO_NR(1, 4), 0);
			gpio_direction_output(IMX_GPIO_NR(1, 5), 0);
			gpio_direction_output(IMX_GPIO_NR(1, 1), 0);
			gpio_direction_output(IMX_GPIO_NR(3, 22), 0);
			break;
		case KUK_MODULE_TRIZEPS8MINI:
			gpio_request(IMX_GPIO_NR(1, 5), "DISPLAY_EN");
			gpio_request(IMX_GPIO_NR(1, 1), "BACKLIGHT_PWM");
			gpio_request(IMX_GPIO_NR(3, 22), "BACKLIGHT_EN");
			gpio_request(IMX_GPIO_NR(1, 4), "LVDS EN");

			gpio_direction_output(IMX_GPIO_NR(1, 4), 0);
			gpio_direction_output(IMX_GPIO_NR(1, 5), 0);
			gpio_direction_output(IMX_GPIO_NR(1, 1), 0);
			gpio_direction_output(IMX_GPIO_NR(3, 22), 0);
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


#define	FT5X06_OP_REG_CHIPID   0xA3			/* vendor’s chip id */
#define	FT5X06_OP_REG_FIRMID   0xA6			/* the firmware id of the application */
#define	FT5X06_OP_REG_FT5201ID 0xA8

static int detect_ipant7(struct display_info_t const *dev)
{
	struct udevice *bus, *main_dev;
	int i2c_bus = 1;
	int ret;
	uint8_t touch_id = 0, chip_id = 0, firm_id = 0;


	printf("%s:  %s \n", __func__, dev->mode.name);
	
	gpio_request(IMX_GPIO_NR(3, 23), "TOUCH EN");
	gpio_direction_output(IMX_GPIO_NR(3, 23), 1);
	
	mdelay(100);

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

	//Check Version
	ret = dm_i2c_read(main_dev, FT5X06_OP_REG_CHIPID, &chip_id, 1);
	ret = dm_i2c_read(main_dev, FT5X06_OP_REG_FIRMID, &firm_id, 1);
	ret = dm_i2c_read(main_dev, FT5X06_OP_REG_FT5201ID, &touch_id, 1);

	//printf("%s: ChipID 0x%x, FirmwareID 0x%x, TouchID 0x%x\n", __func__, chip_id ,firm_id, touch_id );
	if((chip_id == 0x14 && firm_id == 0x06 && touch_id == 0x51) && (!(strncmp(dev->mode.name, "RGB_ATM0700D6J", sizeof(dev->mode.name)))))
	{
		printf("%s: Version 1 AZ Coverlens Display \n",__func__  );
		kuk_panel_drv.lanes = 2;
		kuk_panel_drv.name = "RGB_ATM0700D6J";
		return 1;
	}
	else if((chip_id == 0x0A && firm_id == 0x3 && touch_id == 0x79) && (!(strncmp(dev->mode.name, "RGB_ATM0700D6J", sizeof(dev->mode.name)))))
	{
		printf("%s: Version 1 AZ Display \n",__func__  );
		kuk_panel_drv.lanes = 2;
		kuk_panel_drv.name = "RGB_ATM0700D6J";
		return 1;
	}
	else if(((chip_id == 0x0A && firm_id == 0x08 && touch_id == 0x79) && (!(strncmp(dev->mode.name, "RGB_ATM0700D6J", sizeof(dev->mode.name))))))
	{
		printf("%s: Version 1 HT Display  \n",__func__  );
		//TODO
		return 0;
	}
	else if(chip_id == 0x54 && (firm_id == 0x01 || firm_id == 0x02) && touch_id == 0x79 && (!(strncmp(dev->mode.name, "LVDS_ATM0700L61", sizeof(dev->mode.name)))))
	{
		printf("%s: Version 2 \n",__func__  );
		kuk_panel_drv.lanes = 4;
		kuk_panel_drv.name = "LVDS_ATM0700L61";
		return 1;
	}
	else 
	{
		printf("%s: unknown touch ChipID 0x%x, FirmwareID 0x%x, TouchID 0x%x \n", __func__ , chip_id ,firm_id, touch_id);
		return 0;
	}

	return 0;
}

static int detect_ipanm7(struct display_info_t const *dev)
{
	struct udevice *bus, *main_dev;
	int i2c_bus = 1;
	int ret;

	//printf("%s:  %s \n", __func__, dev->mode.name);

	//Check LVDS
	ret = uclass_get_device_by_seq(UCLASS_I2C, i2c_bus, &bus);
	if (ret) {
		//printf("%s: No bus %d\n", __func__, i2c_bus);
		return 0;
	}
	
	ret = dm_i2c_probe(bus, LVDS_ID, 0, &main_dev);
	if (ret) {
		//printf("%s: Can't find device id=0x%x, on bus %d\n",
		//	__func__, LVDS_ID, i2c_bus);
		return 0;
	}
	
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
	
	kuk_panel_drv.lanes = 4;
	kuk_panel_drv.name = "LVDS_ATM0700D6J";
	
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
	kuk_panel_drv.name = "LVDS_SCF1001C44GGU05";

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

static int detect_display(struct display_info_t const *dev)
{
	struct udevice *bus;
	int ret;
	int i2c_bus = 1;
	char *s;
	int module;

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
				kuk_panel_drv.name = "LVDS_SCF1001C44GGU05";
			}
			else if(!(strncmp(s, "IPANT7", sizeof(s))))
			{
				gpio_request(IMX_GPIO_NR(3, 23), "TOUCH EN");
				gpio_direction_output(IMX_GPIO_NR(3, 23), 1);

				kuk_panel_drv.lanes = 2;
				kuk_panel_drv.name = "RGB_ATM0700D6J";
			}
			else if(!(strncmp(s, "PCONXS", sizeof(s))))
			{
				gpio_request(IMX_GPIO_NR(3, 23), "TOUCH EN");
				gpio_direction_output(IMX_GPIO_NR(3, 23), 0);

				gpio_request(IMX_GPIO_NR(4, 14), "BL_EN");
				gpio_direction_output(IMX_GPIO_NR(4, 14), 1);

				kuk_panel_drv.lanes = 4;
				kuk_panel_drv.name = "PCONXS";
			}
			else if(!(strncmp(s, "IPANM7", sizeof(s))))
			{
				gpio_request(IMX_GPIO_NR(3, 23), "TOUCH EN");
				gpio_direction_output(IMX_GPIO_NR(3, 23), 1);

				kuk_panel_drv.lanes = 4;
				kuk_panel_drv.name = "LVDS_ATM0700D6J";
			}
			else if(!(strncmp(s, "IPANT7_V2", sizeof(s))))
			{
				gpio_request(IMX_GPIO_NR(3, 23), "TOUCH EN");
				gpio_direction_output(IMX_GPIO_NR(3, 23), 1);

				kuk_panel_drv.lanes = 4;
				kuk_panel_drv.name = "LVDS_ATM0700L61";
			}
			else if (!(strncmp(s, "LVDS_AM19201080D1", sizeof(s))))
			{
				gpio_request(IMX_GPIO_NR(4, 14), "BL_EN");
				gpio_direction_output(IMX_GPIO_NR(4, 14), 1);

				kuk_panel_drv.lanes = 4;
				kuk_panel_drv.name = "LVDS_AM19201080D1";
			}

			return 1;
		}
		if(strncmp(s, "auto", sizeof(s)))
		{
			return 0;
		}
	}
	printk("%s: Try autodetect... %s \n", __func__, dev->mode.name);
	module = kuk_GetModule();

	if((!(strncmp(dev->mode.name, "RGB_ATM0700D6J", sizeof(dev->mode.name)))) & (module == KUK_MODULE_TRIZEPS8MINI))
	{
		return detect_ipant7(dev);
	}
	else if((!(strncmp(dev->mode.name, "LVDS_ATM0700L61", sizeof(dev->mode.name)))))
	{
		return detect_ipant7(dev);
	}
	else if((!(strncmp(dev->mode.name, "LVDS_ATM0700D6J", sizeof(dev->mode.name)))) & (module == KUK_MODULE_MYON2))
	{
		return detect_ipanm7(dev);
	}
	else if(!(strncmp(dev->mode.name, "LVDS_SCF1001C44GGU05", sizeof(dev->mode.name))))
	{
		return detect_ipant10(dev);
	}
	else if(!(strncmp(dev->mode.name, "PCONXS", sizeof(dev->mode.name))))
	{
		return detect_pconxs(dev);
	}
	else if(!(strncmp(dev->mode.name, "HDMI_1920x1080", sizeof(dev->mode.name))))
	{
		return adv7535_init(dev);
	}

	return 0;
}

struct display_info_t const displays[] = {{
	.bus = LCDIF_BASE_ADDR,
	.addr = 0,
	.pixfmt = 24,
	.detect = detect_display,
	.enable	= do_enable_mipi2lvds,
	.mode	= {
		.name			= "G104XVN01",
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
	.enable	= do_enable_mipi2lvds,
	.mode	= {
		.name			= "LVDS_SCF1001C44GGU05",
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
	.enable	= do_enable_mipi2lvds,
	.mode	= {
		.name			= "LVDS_ATM0700L61",
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
		.name			= "RGB_ATM0700D6J",
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
		.name			= "LVDS_ATM0700D6J",
		.refresh		= 60,
		.xres			= 800,
		.yres			= 480,
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
	.enable	= do_enable_mipi2hdmi,
	.mode	= {
		.name			= "HDMI_1920x1080",
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
} },  {	
	.bus = LCDIF_BASE_ADDR,
	.addr = 0,
	.pixfmt = 24,
	.detect = detect_display,
	.enable	= do_enable_mipi2lvds,
	.mode	= {
		.name			= "LVDS_AM19201080D1",
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
	.detect = detect_display,
	.enable	= do_display_default,
	.mode	= {
		.name			= "none",
} }
};
size_t display_count = ARRAY_SIZE(displays);
