/* 
 * Lattice FPGA Firmware Version Check
 *
 * Trizeps8Mini
 */

#include <common.h>
#include <command.h>
#include <dm.h>
#include <i2c.h>

DECLARE_GLOBAL_DATA_PTR;

#define LATTICE_FPGA_ADDR 0x41
#define FPGA_REVISION_REG 0xF0

static int do_fpga_rev(cmd_tbl_t *cmdtp, int flag, int argc, char * const argv[])
{
#ifdef CONFIG_DM_I2C

	struct udevice *bus, *dev;
	int i2c_bus = 2;
	int ret;
	uint8_t val;

	ret = uclass_get_device_by_seq(UCLASS_I2C, i2c_bus, &bus);
	if (ret) {
		printf("%s: No Bus %d\n", __func__, i2c_bus);
		return ret;
	}

	ret = dm_i2c_probe(bus, LATTICE_FPGA_ADDR, 0, &dev);
	if (ret) {
		printf("%s: Can not find device id = 0x%x on bus %d\n", 
		        __func__, LATTICE_FPGA_ADDR, i2c_bus);
	return ret;
	}

	ret = dm_i2c_read(dev, FPGA_REVISION_REG, &val, 1);
	if(ret) {
		printf("%s: Error reading FPGA REV: %d\n", __func__, ret);
		return ret;
	}
	printf("FPGA Firmware Version: 0x%x\n", val); 
#else
	printf("error: i2c driver model not active\n");
#endif
	return 0;
}

U_BOOT_CMD(
	fpga_rev, CONFIG_SYS_MAXARGS, 1, do_fpga_rev, 
	"Check Lattice FPGA Firmware Version",
	"Checks the Lattice FPGAs Version Register via I2C and displays the response"
);
