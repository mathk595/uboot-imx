#include <common.h>
#include <part.h>
#include <stdio.h>
#include <malloc.h>
#include <string.h>


#define MBR_LBA 0
#define MBR_CMD_MAGIC "MBRCMD"
#define MBR_CMD_MAGIC_OFFSET 0x10
#define MBR_CMD_MAGIC_LEN 6
#define MBR_CMD_FLAGS_OFFSET (MBR_CMD_MAGIC_OFFSET + MBR_CMD_MAGIC_LEN)

#define MBR_CMD_FLAG_RESTORE_GPT 1

int run_mbr_cmd(void)
{
	struct blk_desc *dev;
	char *buffer;

	//if(blk_first_device(IF_TYPE_MMC,&dev))
	dev=blk_get_dev("mmc",0);
	if(!dev)
	{
		printf("No device\n");
		return 0;
	}

	buffer=malloc(dev->blksz);
	if(!buffer)
	{
		printf("Failed to allocate memory\n");
		return 0;
	}
	if(blk_dread(dev, MBR_LBA, 1, (void *)buffer) > 0)
	{
		if(!strncmp(buffer + MBR_CMD_MAGIC_OFFSET,
					MBR_CMD_MAGIC, MBR_CMD_MAGIC_LEN))
		{
			uint16_t flags=*(uint16_t *)(buffer + MBR_CMD_FLAGS_OFFSET);
			if(flags & MBR_CMD_FLAG_RESTORE_GPT)
			{
				if(!restore_backup_gpt(dev))
					flags&=~MBR_CMD_FLAG_RESTORE_GPT;
				else
					printf("Failed to restore backup GPT\n");
			}
			if(!flags)
				memset(buffer + MBR_CMD_MAGIC_OFFSET, 0,
					   MBR_CMD_MAGIC_LEN + sizeof(flags));
			else
				*(uint16_t *)(buffer + MBR_CMD_FLAGS_OFFSET)=flags;
			if(blk_dwrite(dev, MBR_LBA, 1, (void *)buffer) < 1)
				printf("Failed to write MBR\n");
		}
	}
	else
		printf("Failed to read MBR\n");

	free(buffer);
	return 0;
}

