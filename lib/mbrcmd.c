#include <common.h>
#include <part.h>
#include <stdio.h>
#include <malloc.h>
#include <string.h>


#define MBR_IF "mmc"
#define MBR_DEV 0

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
	//dev=blk_get_dev("mmc",0);
	dev=blk_get_dev(MBR_IF, MBR_DEV);
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


static const char *mbr_cmds[]={
	"restore_gpt",
	NULL};

static int do_mbr_cmd(cmd_tbl_t *cmdtp, int flag,
					  int argc, char * const *argv)
{
	int err=CMD_RET_SUCCESS;
	enum {
		MBR_CMD_NONE,
		MBR_CMD_ADD,
		MBR_CMD_DEL
	} op=MBR_CMD_NONE;
	size_t cmd;
	uint16_t flags;
	struct blk_desc *dev;
	char *buffer;

	if(argc != 3)
		return CMD_RET_USAGE;
	if(!strcmp(argv[1], "add"))
		op=MBR_CMD_ADD;
	else if(!strcmp(argv[1], "del"))
		op=MBR_CMD_DEL;
	else
		return CMD_RET_USAGE;

	for(cmd=0; mbr_cmds[cmd]; cmd++)
		if(!strcmp(mbr_cmds[cmd], argv[2]))
			break;
	if(!mbr_cmds[cmd])
		return CMD_RET_USAGE;

	flags=(1 << cmd);

	dev=blk_get_dev(MBR_IF, MBR_DEV);
	if(!dev)
	{
		printf("Failed to open device\n");
		return CMD_RET_FAILURE;
	}

	buffer=malloc(dev->blksz);
	if(!buffer)
	{
		printf("Failed to allocate memory\n");
		return CMD_RET_FAILURE;
	}

	if(blk_dread(dev, MBR_LBA, 1, (void *)buffer) > 0)
	{
		switch(op)
		{
		case MBR_CMD_ADD:
			*(uint16_t *)(buffer + MBR_CMD_FLAGS_OFFSET) |= flags;
			strncpy(buffer + MBR_CMD_MAGIC_OFFSET,
					MBR_CMD_MAGIC, MBR_CMD_MAGIC_LEN);
			break;
		case MBR_CMD_DEL:
			*(uint16_t *)(buffer + MBR_CMD_FLAGS_OFFSET) &= ~flags;
			if(!*(uint16_t *)(buffer + MBR_CMD_FLAGS_OFFSET))
				memset(buffer + MBR_CMD_MAGIC_OFFSET,
					   0, MBR_CMD_MAGIC_LEN);
			break;
		default:
			// to keep compiler happy
			break;
		}

		if(blk_dwrite(dev, MBR_LBA, 1, (void *)buffer) < 1)
		{
			printf("Failed to write MBR\n");
			err=CMD_RET_FAILURE;
		}
	}
	else
	{
		printf("Failed to read MBR\n");
		err=CMD_RET_FAILURE;
	}
	
	free(buffer);
	return err;
}


U_BOOT_CMD(
	mbr_cmd, 3, 0, do_mbr_cmd,
	"add or remove a command to of from mbr",
	"add|del <command>\n"
	"known commands (so far):\n"
	"\trestore_gpt\n"
	);
