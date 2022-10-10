/* 
 * Print QuickLogic ArcticLink III Bx5 Error Registers
 *
 * Trizeps8Mini V2R1
 */

#include <common.h>
#include <command.h>
#include <dm.h>
#include <i2c.h>

DECLARE_GLOBAL_DATA_PTR;

#define BX5_ADDR 0x64

static int do_bx5_error(cmd_tbl_t *cmdtp, int flag, int argc, char * const argv[])
{
#ifdef CONFIG_DM_I2C

	struct udevice *bus, *dev;
	int i2c_bus = 2;
	int ret;
	uint8_t val[4];

	uint8_t CONTROL_BYTE_GEN = 0x09;
	uint8_t bx5_read_cmd[4] = { 0x09, 0x24, 0x05, 0x01 }; // Generic Bx5 Read Command
	uint8_t bx5_read_err1[8] = { 0x29, 0x05, 0x01, 0x41, 0x04, 0x02, 0x04, 0x00 }; // Read MIPI Client Error Register (0x204)
	uint8_t bx5_read_err2[8] = { 0x29, 0x05, 0x01, 0x41, 0x48, 0x01, 0x04, 0x00 }; // Read Device Error Register (0x148)
	struct i2c_msg i2c_msg[2];

	ret = uclass_get_device_by_seq(UCLASS_I2C, i2c_bus, &bus);
	if (ret) {
		printf("%s: No Bus %d\n", __func__, i2c_bus);
		return ret;
	}

	ret = dm_i2c_probe(bus, BX5_ADDR, 0, &dev);
	if (ret) {
		printf("%s: Can not find device id = 0x%x on bus %d\n", 
		        __func__, BX5_ADDR, i2c_bus);
		return ret;
	}


	i2c_msg[0].addr = BX5_ADDR;
	i2c_msg[0].flags = 0;
	i2c_msg[0].len = 4;
	i2c_msg[0].buf = &bx5_read_cmd;

	i2c_msg[1].addr = BX5_ADDR;
	i2c_msg[1].flags = I2C_M_RD;
	i2c_msg[1].len = 4;
	i2c_msg[1].buf = &val;
	
	ret = dm_i2c_write(dev, CONTROL_BYTE_GEN, bx5_read_err1, sizeof(bx5_read_err1));
	if(ret) {
		printf("%s: Error writing read_err1 command: %d\n", __func__, ret);
		return ret;
	}

	ret = dm_i2c_xfer(dev, &i2c_msg[0], 2);
	if(ret) {
		printf("%s: Error reading Error Registers: %d\n", __func__, ret);
		return ret;
	}
	printf("MIPI Client Error Register (0x204): 0x%02x%02x %02x%02x\n", val[3], val[2], val[1], val[0]); 


	ret = dm_i2c_write(dev, CONTROL_BYTE_GEN, bx5_read_err2, sizeof(bx5_read_err2));
	if(ret) {
		printf("%s: Error writing read_err2 command: %d\n", __func__, ret);
		return ret;
	}
	ret = dm_i2c_xfer(dev, &i2c_msg[0], 2);
	if(ret) {
		printf("%s: Error reading Error Registers: %d\n", __func__, ret);
		return ret;
	}
	printf("Device Error Register (0x148):      0x%02x%02x %02x%02x\n", val[3], val[2], val[1], val[0]);
#else
	printf("error: i2c driver model not active\n");
#endif
	return 0;
}

U_BOOT_CMD(
	bx5_error, CONFIG_SYS_MAXARGS, 1, do_bx5_error, 
	"Check QuickLogic ArcticLink III Bx5 error registers",
	"Checks the QuickLogic ArcticLink III Bx5 error registers via I2C and prints the response."
);

