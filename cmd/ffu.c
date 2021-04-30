#include <common.h>
#include <command.h>
#include <malloc.h>
#include <memalign.h>
#include "fs.h"
#include <div64.h>

__weak
void ffu2blk_progress_init(int expectedsize)
{
	putc('\n');
}

__weak
void ffu2blk_progress(int iteration,
		     int dwWriteDescriptorCount)
{
	
	if (0 == (iteration & 7))
		printf("%d%% \r", (100 * iteration / dwWriteDescriptorCount));
}

__weak
void ffu2blk_progress_finish(int returnval,
			     u64 bytes_written,
			     u64 total_bytes,
			     u32 expected_crc,
			     u32 calculated_crc)
{
	/*
	if (0 == returnval) {
		printf("\n\t%llu bytes, crc 0x%08x\n",
		       total_bytes, calculated_crc);
	} else {
		printf("\n\tuncompressed %llu of %llu\n"
		       "\tcrcs == 0x%08x/0x%08x\n",
		       bytes_written, total_bytes,
		       expected_crc, calculated_crc);
	}
	*/
}

typedef struct SECHeader_
{
	int cbSize;
	unsigned char signature[12];
	int dwChunkSizeInKb;
	int dwAlgId;
	int dwCatalogSize;
	int dwHashTableSize;
} SECHeader;

typedef struct IMGHeader_
{
	int cbSize;
	unsigned char signature[12];
	int ManifestLength;
	int dwChunkSizeInKb;
} IMGHeader;

typedef struct StoreHeader_
{
	int dwUpdateType;
	short MajorVersion;
	short MinorVersion;
	short FullFlashMajorVersion;
	short FullFlashMinorVersion;
	unsigned char szPlatformId[192];
	int dwBlockSizeInBytes;
	int dwWriteDescriptorCount;
	int dwWriteDescriptorLength;
	int dwValidateDescriptorCount;
	int dwValidateDescriptorLength;
	int dwInitialTableIndex;
	int dwInitialTableCount;
	int dwFlashOnlyTableIndex;
	int dwFlashOnlyTableCount;
	int dwFinalTableIndex;
	int dwFinalTableCount;
} StoreHeader;


typedef struct BlockDataEntry_
{
	int dwDiskAccessMethod;
	int dwBlockIndex;
	int dwBlockCount;
	int dwLocationCount;
} BlockDataEntry;

int ffu2blk(struct blk_desc *dev,
		const char *device,
		const char *part,
		const char *filename,
		unsigned long szwritebuf)
{
	int r = 0;
	lbaint_t blksperbuf, outblock;
	loff_t actread;
	loff_t fsize;
	unsigned char *writebuf = NULL;
	u64 pos = 0;
	unsigned long time;
	SECHeader FFUSECHeader;
	IMGHeader FFUIMGHeader;
	StoreHeader FFUStoreHeader;
	u64 blockdataaddress;
	BlockDataEntry *CurrentBlock = NULL;

	time = get_timer(0);

	printf ("Unpack %s %s %s to  blkdevice \n\r", device, part, filename);

	r = fs_set_blk_dev(device, part, FS_TYPE_FAT);
	if (r)
	{
		printf("Error: could not access storage.\n");
		return (-1);
	}	
	r = fs_size(filename, &fsize);
	if (r)
	{
		printf("Error: could find file\n");
		return (-1);
	}

	if (!szwritebuf ||
	    (szwritebuf % dev->blksz) ||
	    (szwritebuf < dev->blksz)) {
		printf("%s: size %lu not a multiple of %lu\n",
		       __func__, szwritebuf, dev->blksz);
		return -1;
	}

	r = fs_set_blk_dev(device, part, FS_TYPE_FAT);
	r = fs_read(filename, (long)&FFUSECHeader, pos, sizeof(FFUSECHeader) , &actread);
	
	printf("\r\nPrint FFUSECHeader (0x%llx):\r\n", pos);

	printf("cbSize %d \r\n", FFUSECHeader.cbSize);
	printf("signature %s \r\n", FFUSECHeader.signature);
	printf("dwAlgId %d \r\n", FFUSECHeader.dwAlgId);
	printf("ChunkSize 0x%x kb \r\n", FFUSECHeader.dwChunkSizeInKb);
	printf("dwCatalogSize %d \r\n", FFUSECHeader.dwCatalogSize);
	printf("dwHashTableSize %d \r\n", FFUSECHeader.dwHashTableSize);

	pos = ((FFUSECHeader.cbSize + FFUSECHeader.dwCatalogSize + FFUSECHeader.dwHashTableSize) /  (FFUSECHeader.dwChunkSizeInKb*1024) + 1)*(FFUSECHeader.dwChunkSizeInKb*1024);

	r = fs_set_blk_dev(device, part, FS_TYPE_FAT);
	r = fs_read(filename, (long)&FFUIMGHeader, pos, sizeof(FFUIMGHeader) , &actread);

	printf("\r\nPrint FFUSECHeader (0x%llx): \r\n", pos);
	printf("cbSize %d \r\n", FFUIMGHeader.cbSize);
	printf("signature %s \r\n", FFUIMGHeader.signature);
	printf("ManifestLength %d \r\n", FFUIMGHeader.ManifestLength);
	printf("ChunkSize 0x%x kb \r\n", FFUIMGHeader.dwChunkSizeInKb);

	pos = pos + (FFUIMGHeader.cbSize / (FFUIMGHeader.dwChunkSizeInKb*1024) + 1)*(FFUIMGHeader.dwChunkSizeInKb*1024);

	r = fs_set_blk_dev(device, part, FS_TYPE_FAT);
	r = fs_read(filename, (long)&FFUStoreHeader, pos, sizeof(FFUStoreHeader) , &actread);
	
	printf("\r\nPrint FFUStoreHeader (0x%llx): \r\n", pos);
	printf("dwUpdateType %d \r\n", FFUStoreHeader.dwUpdateType);
	printf("MajorVersion %d \r\n", FFUStoreHeader.MajorVersion);
	printf("MinorVersion %d \r\n", FFUStoreHeader.MinorVersion);
	printf("FullFlashMajorVersion %d \r\n", FFUStoreHeader.FullFlashMajorVersion);
	printf("FullFlashMinorVersion %d \r\n", FFUStoreHeader.FullFlashMinorVersion);
	printf("szPlatformId %s \r\n", FFUStoreHeader.szPlatformId);
	printf("int dwBlockSizeInBytes 0x%x \r\n", FFUStoreHeader.dwBlockSizeInBytes);
	printf("int dwWriteDescriptorCount %d \r\n", FFUStoreHeader.dwWriteDescriptorCount);
	printf("int dwWriteDescriptorLength %d \r\n", FFUStoreHeader.dwWriteDescriptorLength);
	printf("int dwValidateDescriptorCount %d \r\n", FFUStoreHeader.dwValidateDescriptorCount);
	printf("int dwValidateDescriptorLength %d \r\n", FFUStoreHeader.dwValidateDescriptorLength);
	printf("int dwInitialTableIndex %d \r\n", FFUStoreHeader.dwInitialTableIndex);
	printf("int dwInitialTableCount %d \r\n", FFUStoreHeader.dwInitialTableCount);
	printf("int dwFlashOnlyTableIndex %d \r\n", FFUStoreHeader.dwFlashOnlyTableIndex);
	printf("int dwFlashOnlyTableCount %d \r\n", FFUStoreHeader.dwFlashOnlyTableCount);
	printf("int dwFinalTableIndex %d \r\n", FFUStoreHeader.dwFinalTableIndex);
	printf("int dwFinalTableCount %d \r\n", FFUStoreHeader.dwFinalTableCount);

	pos += sizeof(FFUStoreHeader);
	printf("Block data entries begin: 0x%llx \r\n", pos );

	blockdataaddress = (pos - sizeof(FFUStoreHeader)) + (( FFUStoreHeader.dwWriteDescriptorLength) / (FFUIMGHeader.dwChunkSizeInKb*1024) + 1)*(FFUIMGHeader.dwChunkSizeInKb*1024);

	printf("Block data chunks begin: 0x%llx \r\n", blockdataaddress );

	CurrentBlock = (BlockDataEntry *)malloc_cache_aligned( (sizeof(BlockDataEntry) * FFUStoreHeader.dwWriteDescriptorCount) );

	r = fs_set_blk_dev(device, part, FS_TYPE_FAT);
	r = fs_read(filename, (long)CurrentBlock, pos, (sizeof(BlockDataEntry) * FFUStoreHeader.dwWriteDescriptorCount) , &actread);

	writebuf = (unsigned char *)malloc_cache_aligned(FFUStoreHeader.dwBlockSizeInBytes);

	blksperbuf = FFUStoreHeader.dwBlockSizeInBytes / dev->blksz;
	outblock = lldiv(0, dev->blksz);


	ffu2blk_progress_init(FFUStoreHeader.dwWriteDescriptorCount);

	printf("buffer 0x%lx \r\n", (unsigned long) writebuf);
	for(int i = 0; i < FFUStoreHeader.dwWriteDescriptorCount; i++)
	{
		CurrentBlock++;

		if (CurrentBlock->dwLocationCount == 0 ||  CurrentBlock->dwDiskAccessMethod == 2)
			continue;

		r = fs_set_blk_dev(device, part, FS_TYPE_FAT);
		r = fs_read(filename, (long)writebuf, (blockdataaddress + (i*FFUStoreHeader.dwBlockSizeInBytes)), FFUStoreHeader.dwBlockSizeInBytes, &actread);

		outblock = lldiv((u64)CurrentBlock->dwBlockIndex*FFUStoreHeader.dwBlockSizeInBytes, dev->blksz);
		blk_dwrite(dev, outblock, blksperbuf, writebuf);

		//printf("read 0x%llx write 0x%llx (block  0x%llx)\r\n", (blockdataaddress + (i*FFUStoreHeader.dwBlockSizeInBytes)), (u64)CurrentBlock->dwBlockIndex*FFUStoreHeader.dwBlockSizeInBytes, outblock );

		ffu2blk_progress(i, FFUStoreHeader.dwWriteDescriptorCount);

	}

	//ffu2blk_progress_finish(r, totalfilled, szexpected, expected_crc, crc);
	free(writebuf);

	time = get_timer(time);
	printf("%s restored in %lu ms \r\n", filename, time);

	return 0;
}



static int do_ffu2blk(cmd_tbl_t *cmdtp, int flag, int argc, char * const argv[])
{
	struct blk_desc *bdev;
	int ret;
	unsigned long writebuf = 1<<20;

	if (argc < 5)
		return CMD_RET_USAGE;
	ret = blk_get_device_by_str(argv[1], argv[2], &bdev);
	if (ret < 0)
		return CMD_RET_FAILURE;

	if (6 < argc) {
		writebuf = simple_strtoul(argv[6], NULL, 16);
	}

	ret = ffu2blk(bdev, argv[3], argv[4], argv[5], writebuf);

	return 0;
}

U_BOOT_CMD(
	ffu2blk, 8, 0, do_ffu2blk,
	"write ffu image file to block device",
	"<interface> <dev> <interface> <dev> <filename>\n"
);


