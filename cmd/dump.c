#include <common.h>
#include <command.h>
#include <malloc.h>
#include <part.h>
#include <fs.h>
#include <dm.h>
#include <console.h>
#include <u-boot/zlib.h>
#include <time.h>

#define DEFAULT_BUFF_SIZE (1 << 24) // 16MB
#define ZLIB_MEM_LEVEL 9 // maximum memory, optimal speed
#define NAME_MAX 256 // there is no limits.h in u-boot

static int dump(struct blk_desc *dev, disk_partition_t *part,
				char *file_if, char *file_part, char *file_name,
				size_t buff_size, int compress, char *orig_name)
{
	enum {
		STATE_MEM_ERROR = -5,
		STATE_ZIP_ERROR = -4,
		STATE_BLOCK_ERROR = -3,
		STATE_FILE_ERROR = -2,
		STATE_USER_ERROR = -1,
		STATE_OK = 0,
		STATE_EOF = 1
	} state = STATE_OK;
	struct blk_ops *ops;
	z_stream zstream;
	void *buff_in;
	void *buff_out=NULL;
	lbaint_t blk_start; // first block to read
	lbaint_t blk_no;    // number of blocks to read
	unsigned long blk_count; // number of blocks actualy read
	unsigned long blk_total=0;
	loff_t file_start=0;
	loff_t file_count;
	ulong t_rd=0,t_wr=0;


	if(!(ops=blk_get_ops(dev->bdev)))
		return -1;
	if(!(buff_in=malloc(buff_size)))
		state=STATE_MEM_ERROR;
	if(state==STATE_OK && compress && !(buff_out=malloc(buff_size)))
		state=STATE_MEM_ERROR;

	if(state!=STATE_OK)
	{
		printf("Error allocating memory\n");
		if(buff_in)
			free(buff_in);
		return -1;
	}

	if(compress)
	{

		zstream.zalloc=Z_NULL;
		zstream.zfree=Z_NULL;
		zstream.opaque=Z_NULL;
		if(deflateInit2_(&zstream,
						 compress,//Z_DEFAULT_COMPRESSION,
						 Z_DEFLATED,
						 2*MAX_WBITS,
						 ZLIB_MEM_LEVEL,
						 Z_DEFAULT_STRATEGY,
						 ZLIB_VERSION,
						 sizeof(z_stream)) == Z_OK)
		{
			gz_header zhead;
			memset(&zhead, 0, sizeof(zhead));
			zhead.os=3;
			zhead.name=(Bytef *)orig_name;
			if(deflateSetHeader(&zstream, &zhead) != Z_OK)
			{
				printf("Error setting file header\n");
				free(buff_in);
				return -1;
			}
		}
		else
		{
			printf("Error initializing stream\n");
			free(buff_in);
			return -1;
		}
	}

	printf("First block to dump: %"LBAFlength"d\n", part->start);
	printf("Number of blocks to dump: %"LBAFlength"d\n", part->size);
	printf("Dump to %s %s %s\n", file_if, file_part, file_name);
	if(compress)
	{
		printf("Compression level: %d\n",compress);
		printf("Original file: %s\n",orig_name);
	}
	else
		printf("No compression\n");

	blk_start=part->start;
	blk_no=buff_size/dev->blksz;
	while(state == STATE_OK)
	{
		//printf("%8"LBAFlength"d / %8"LBAFlength"d\r",
		printf("%8"LBAFlength"d / %8"LBAFlength"d",
			   blk_total, part->size);

		if(blk_start+blk_no > part->start+part->size)
			blk_no=part->start+part->size-blk_start;
		t_rd=get_timer(0);
		blk_count=ops->read(dev->bdev, blk_start, blk_no, buff_in);
		t_rd=get_timer(t_rd);
		//printf("blk_no=%"LBAFlength"d, blk_count=%"LBAFlength"d\n",
		//	   blk_no, blk_count);

		if(!IS_ERR_VALUE(blk_count))
		{
			blk_start+=blk_count;
			if(blk_start < part->start+part->size)
				blk_total+=blk_count;
			else
				state=STATE_EOF;
		}
		else
			state=STATE_BLOCK_ERROR;

		if(state==STATE_OK && fs_set_blk_dev(file_if, file_part, FS_TYPE_ANY))
			state=STATE_FILE_ERROR;
		if(state==STATE_OK || state==STATE_EOF)
		{
			t_wr=get_timer(0);
			if(compress)
			{
				zstream.next_in=buff_in;
				zstream.avail_in=blk_count*dev->blksz;
				do
				{
					zstream.next_out=buff_out;
					zstream.avail_out=buff_size;
					switch(deflate(&zstream,
								   state==STATE_EOF ? Z_FINISH : Z_NO_FLUSH))
					{
					case Z_OK:
					case Z_STREAM_END:
					case Z_BUF_ERROR:
						break;
					default:
						state=STATE_ZIP_ERROR;
					}
					if(state != STATE_ZIP_ERROR)
					{
						if(fs_set_blk_dev(file_if, file_part, FS_TYPE_ANY) == 0 &&
						   fs_write(file_name, (ulong)buff_out,
									file_start, buff_size-zstream.avail_out,
							   &file_count) ==0)
							file_start+=file_count;
						else
							state=STATE_FILE_ERROR;
					}
				} while(zstream.avail_out==0 &&
						(state==STATE_OK || state==STATE_EOF));
			}
			else if(fs_set_blk_dev(file_if, file_part, FS_TYPE_ANY) == 0 &&
					fs_write(file_name, (ulong)buff_in,
							 file_start, blk_count*dev->blksz,
							 &file_count) == 0)
					file_start+=file_count;
				else
					state=STATE_FILE_ERROR;
			t_wr=get_timer(t_wr);
		}

		printf(", rd=%dms, wr=%dms\n", t_rd, t_wr);
		if(state == STATE_OK && ctrlc())
			state=STATE_USER_ERROR;
	}

	putc('\n');
	free(buff_in);
	if(compress)
	{
		deflateEnd(&zstream);
		free(buff_out);
	}

	switch(state)
	{
	case STATE_ZIP_ERROR:
		printf("Error deflating data\n");
		return -1;
	case STATE_BLOCK_ERROR:
		printf("Error reading block device\n");
		return -1;
	case STATE_FILE_ERROR:
		printf("Error writing file\n");
		return -1;
	case STATE_USER_ERROR:
		printf("Aborted by user\n");
		return -1;
	default:
		// to keep compiler happy
		break;
	}
	return 0;
}

static int do_dump(cmd_tbl_t *cmdtp, int flag,
				   int argc, char * const argv[])
{
	char *p;
	struct blk_desc *dev;
	disk_partition_t part;
	size_t buff_size = DEFAULT_BUFF_SIZE;
	int compress=0;
	char name[NAME_MAX];

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

	if((p=strrchr(argv[3],'.')))
	{
		size_t n=p-argv[3];
		if(!strcmp(p,".gz"))
		{
			compress=Z_DEFAULT_COMPRESSION;
			strncpy(name, argv[3], n);
			*(name+n)=0;
		}
		else if(!strcmp(p,".imz"))
		{
			compress=Z_DEFAULT_COMPRESSION;
			strncpy(name, argv[3], n);
			strcpy(name+n, ".img");
		}
	}
	if(dump(dev, &part, argv[1], argv[2], argv[3],
			buff_size, compress, compress ? name : NULL) < 0)
		return CMD_RET_FAILURE;

	return CMD_RET_SUCCESS;
}

U_BOOT_CMD(
	dump, 7, 0, do_dump,
	"dump a block device to a file",
	"<dst-interface> <dst-dev> <dst-file> <src-interface> <src-dev> [<buffer-size>]\n"
	"\tdst-interface, dst-dev and dst-file specify where to dump to\n"
	"\tsrc-interface and src-device specify what to dump\n"
	"\tbuffer-size specifies the the buffer size to be used, the default is 16MB\n"
	"\twhen file-name ends with \".gz\" or \".imz\" the output will be compressed\n"
	"\tusing zlib\n\n"
	"\tsrc-dev is specified in the form <device-number>[:<partition-number>]\n"
	"\twhen the partition-number is omitted the 1st one is used\n"
	"\ta partition number of 0 means the whole device\n\n"
	"Examples:\n"
	"\tdump usb 0 part1.img mmc 0:1\n"
	"\t\twill write the 1st partition of the 1st mmc device to the file \"part1.imz\"\n"
	"\t\tin the 1st partition of the 1st usb device\n"
	"\tdump usb 0 disk.img.gz mmc 0:0\n"
	"\t\twill write the whole 1st mmc device to the compressed file \"disk.img.gz\"\n"
	"\t\tin the 1st partition of the 1st usb device\n"
	);
