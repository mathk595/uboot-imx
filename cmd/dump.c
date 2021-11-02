#include <common.h>
#include <command.h>
#include <malloc.h>
#include <part.h>
#include <fs.h>
#include <dm.h>
#include <console.h>

#define DEFAULT_BUFF_SIZE (1 << 20)

static int dump(struct blk_desc *dev, disk_partition_t *part,
				char *file_if, char *file_part, char *file_name,
				size_t buff_size, int zipped)
{
	enum {
		STATE_BLOCK_ERROR = -3,
		STATE_FILE_ERROR = -2,
		STATE_USER_ERROR = -1,
		STATE_OK = 0,
		STATE_EOF = 1
	} state = STATE_OK;
	struct blk_ops *ops;
	void *buffer;
	lbaint_t blk_start; // first block to read
	lbaint_t blk_no;    // number of blocks to read
	unsigned long blk_count; // number of blocks actualy read
	unsigned long blk_total=0;
	loff_t file_start=0;
	loff_t file_count;

	if(!(ops=blk_get_ops(dev->bdev)))
		return -1;
	if(!(buffer=malloc(buff_size)))
		return -1;

	printf("First block to dump: %"LBAFlength"d\n", part->start);
	printf("Number of blocks to dump: %"LBAFlength"d\n", part->size);
	printf("Dump to %s %s %s\n", file_if, file_part, file_name);

	blk_start=part->start;
	blk_no=buff_size/dev->blksz;
	while(state == STATE_OK)
	{
		if(blk_start+blk_no > part->start+part->size)
			blk_no=part->start+part->size-blk_start;
		blk_count=ops->read(dev->bdev, blk_start, blk_no, buffer);
		//printf("blk_no=%"LBAFlength"d, blk_count=%"LBAFlength"d\n",
		//	   blk_no, blk_count);
		if(!IS_ERR_VALUE(blk_count))
		{
			if(fs_set_blk_dev(file_if, file_part, FS_TYPE_ANY) == 0 &&
			   fs_write(file_name, (ulong)buffer,
						file_start, blk_count*dev->blksz, &file_count) == 0)
			{
				blk_start+=blk_count;
				if(blk_start < part->start+part->size)
				{
					file_start+=file_count;
					blk_total+=blk_count;
				}
				else
					state=STATE_EOF;
				
			}
			else
				state=STATE_FILE_ERROR;
		}
		else
			state=STATE_BLOCK_ERROR;

		printf("%8"LBAFlength"d / %8"LBAFlength"d\r",
			   blk_total, part->size);
		if(state == STATE_OK && ctrlc())
			state=STATE_USER_ERROR;
	}

	free(buffer);
	switch(state)
	{
	case STATE_BLOCK_ERROR:
		printf("Failed to read block device\n");
		break;
	case STATE_FILE_ERROR:
		printf("Failed to write file\n");
		break;
	case STATE_USER_ERROR:
		printf("Aborted by user\n");
		break;
	default:
		return 0;
	}
	return -1;
}

static int do_dump(cmd_tbl_t *cmdtp, int flag,
				   int argc, char * const argv[])
{
	char *p;
	struct blk_desc *dev;
	disk_partition_t part;
	size_t buff_size = DEFAULT_BUFF_SIZE;
	int zipped=0;

	if(argc < 6 || argc > 7)
		return CMD_RET_USAGE;

	if(blk_get_device_part_str(argv[4], argv[5], &dev, &part, 1) < 0)
		return CMD_RET_USAGE;

	if(argc > 6)
	{
		buff_size=simple_strtol(argv[6], &p, 16);
		if(p == argv[6] || buff_size < dev->blksz)
		{
			printf("Invalid buffer size, using default\n");
			buff_size = DEFAULT_BUFF_SIZE;
		}
	}

	p=strchr(argv[3],'.');
	if(p && !strcmp(p,".imz"))
		zipped=1;

	if(dump(dev, &part, argv[1], argv[2], argv[3],
			buff_size, zipped) < 0)
		return CMD_RET_FAILURE;

	return CMD_RET_SUCCESS;
}

U_BOOT_CMD(
	dump, 7, 0, do_dump,
	"dump a block device to a file",
	"<dst interface> <dst dev> <dst file> <src interface> <src dev> [<buffer size>]\n"
	);
