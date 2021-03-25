/* 
 * Kinetis MCU Firmware Revision Check
 *
 * Trizeps8Mini
 */

#include <common.h>
#include <command.h>
#include <dm.h>
#include <i2c.h>

DECLARE_GLOBAL_DATA_PTR;

#define KINETIS_MCU_ADDR 0x10
#define MCU_REVISION_REG 0xF0
#define REVISION_SIZE 16

static int do_mcu_rev(cmd_tbl_t *cmdtp, int flag, int argc, char * const argv[])
{
#ifdef CONFIG_DM_I2C

	struct udevice *bus, *dev;
	int i2c_bus = 2;
	int ret;
	uint8_t val[REVISION_SIZE];

	ret = uclass_get_device_by_seq(UCLASS_I2C, i2c_bus, &bus);
	if (ret) {
		printf("%s: No Bus %d\n", __func__, i2c_bus);
		return ret;
	}

	ret = dm_i2c_probe(bus, KINETIS_MCU_ADDR, 0, &dev);
	if (ret) {
		printf("%s: Can not fine device id = 0x%x on bus %d\n", 
		        __func__, KINETIS_MCU_ADDR, i2c_bus);
		return ret;
	}
	for(int i=0; i<REVISION_SIZE; i++)
	{
		ret = dm_i2c_read(dev, MCU_REVISION_REG+i, &val[i], 1);
		if(ret) {
			return ret;printf("%s: Error reading MCU REV: %d\n", __func__, ret);
			return ret;
		}
	}
	printf("MCU Firmware Version: ");
	for(int i=0;i<REVISION_SIZE; i++)
	{
		if(val[i] == 0)
			printf(".");
		else
			printf("%c", val[i]);
	} 
	printf("\n");
#else
	printf("error: i2c driver model not active\n");
#endif
	return 0;
}

U_BOOT_CMD(
	mcu_rev, CONFIG_SYS_MAXARGS, 1, do_mcu_rev, 
	"Check Kinetis MCU Firmware Version",
	"Checks the Kinetis MCUs Firmware Version Register via I2C and displays the response"
);
