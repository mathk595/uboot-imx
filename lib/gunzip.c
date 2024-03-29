// SPDX-License-Identifier: GPL-2.0+
/*
 * (C) Copyright 2000-2006
 * Wolfgang Denk, DENX Software Engineering, wd@denx.de.
 */

#include <common.h>
#include <watchdog.h>
#include <command.h>
#include <console.h>
#include <image.h>
#include <malloc.h>
#include <memalign.h>
#include <u-boot/zlib.h>
#include <div64.h>
#include <linux/err.h>
#include <part_efi.h>

#include "fs.h"

#define HEADER0			'\x1f'
#define HEADER1			'\x8b'
#define	ZALLOC_ALIGNMENT	16
#define HEAD_CRC		2
#define EXTRA_FIELD		4
#define ORIG_NAME		8
#define COMMENT			0x10
#define RESERVED		0xe0
#define DEFLATED		8

void *gzalloc(void *x, unsigned items, unsigned size)
{
	void *p;

	size *= items;
	size = (size + ZALLOC_ALIGNMENT - 1) & ~(ZALLOC_ALIGNMENT - 1);

	p = malloc (size);

	return (p);
}

void gzfree(void *x, void *addr, unsigned nb)
{
	free (addr);
}

int gzip_parse_header(const unsigned char *src, unsigned long len)
{
	int i, flags;
	
	/* skip header */
	i = 10;
	flags = src[3];
	if (src[2] != DEFLATED || (flags & RESERVED) != 0) {
		puts ("Error: Bad gzipped data\n");
		return (-1);
	}
	if ((flags & EXTRA_FIELD) != 0)
		i = 12 + src[10] + (src[11] << 8);
	if ((flags & ORIG_NAME) != 0)
		while (src[i++] != 0)
			;
	if ((flags & COMMENT) != 0)
		while (src[i++] != 0)
			;
	if ((flags & HEAD_CRC) != 0)
		i += 2;
	if (i >= len) {
		puts ("Error: gunzip out of data in header\n");
		return (-1);
	}
	return i;
}

int gunzip(void *dst, int dstlen, unsigned char *src, unsigned long *lenp)
{
	int offset = gzip_parse_header(src, *lenp);

	if (offset < 0)
		return offset;

	return zunzip(dst, dstlen, src, lenp, 1, offset);
}

#ifdef CONFIG_CMD_UNZIP
__weak
void gzwrite_progress_init(u64 expectedsize)
{
	putc('\n');
}

__weak
void gzwrite_progress(int iteration,
		     u64 bytes_written,
		     u64 total_bytes)
{
	if (0 == (iteration & 7))
		printf("%3llu%% %llu/%llu\r", (100 * bytes_written / total_bytes) , bytes_written, total_bytes);
}

__weak
void gzwrite_progress_finish(int returnval,
			     u64 bytes_written,
			     u64 total_bytes,
			     u32 expected_crc,
			     u32 calculated_crc)
{
	printf("%3llu%% %llu/%llu\r", (100 * bytes_written / total_bytes) , bytes_written, total_bytes);

	if (0 == returnval) {
		printf("\n\t%llu bytes, crc 0x%08x\n",
		       total_bytes, calculated_crc);
	} else {
		printf("\n\tuncompressed %llu of %llu\n"
		       "\tcrcs == 0x%08x/0x%08x\n",
		       bytes_written, total_bytes,
		       expected_crc, calculated_crc);
	}
}

int gzwritefile(struct blk_desc *dev,
		const char *device,
		const char *part,
		const char *filename,
		unsigned long szwritebuf,
		int force,
		lbaint_t skip_blocks,
		lbaint_t count_blocks)
{
	z_stream s;
	int r = -1;
	unsigned crc = 0;
	u64 totalfilled = 0;
	lbaint_t blksperbuf; // no of blocks the buffer can hold
	lbaint_t outblock; // first block to be written
	u32 expected_crc;
	u64 payload_size;
	int iteration = 0;
	loff_t actread;
	int i, flags;
	loff_t len;
	unsigned char *writebuf = NULL;
	unsigned char *src = NULL;
	u64 szexpected = 0;
	s64 MemBiggerThanImage=0;
	unsigned long BUFFERSIZE;
	unsigned long time;
	int start=0; // writing has been started
	int stop=0;  // stop writing

	BUFFERSIZE = szwritebuf << 3;

	time = get_timer(0);

	//src = (unsigned char *)malloc_cache_aligned(BUFFERSIZE*3);

	printf ("Unpack %s %s %s to blkdevice \n\r", device, part, filename);
	printf("Size of input buffer: %ld bytes\n", BUFFERSIZE*3);
	printf("Size of output buffer: %ld bytes\n", szwritebuf);
	printf("Skip first %"LBAFlength"d blocks\n", skip_blocks);
	if(count_blocks != (lbaint_t)-1)
		printf("Write %"LBAFlength"d blocks\n", count_blocks);

	int res = fs_set_blk_dev(device, part, FS_TYPE_FAT);
	if (res)
	{
		printf("Error: could not access storage.\n");
		return (-1);
	}	
	res = fs_size(filename, &len);
	if (res)
	{
		printf("Error: could not find file\n");
		return (-1);
	}

	if (!szwritebuf ||
	    (szwritebuf % dev->blksz) ||
	    (szwritebuf < dev->blksz)) {
		printf("%s: size %lu not a multiple of %lu\n",
		       __func__, szwritebuf, dev->blksz);
		return -1;
	}

	blksperbuf = szwritebuf / dev->blksz;
	outblock = lldiv(0, dev->blksz); // this will allways be zero, or not?
	
	src = (unsigned char *)malloc_cache_aligned(BUFFERSIZE*3);
	writebuf = (unsigned char *)malloc_cache_aligned(szwritebuf);
	if(!src || !writebuf)
	{
		printf("Failed to allocate memory\n");
		if(src)
			free(src);
		if(writebuf)
			free(writebuf);
		return -1;
	}
	res = fs_set_blk_dev(device, part, FS_TYPE_FAT);
	res = fs_read(filename, (ulong)src, 0, 0xFF , &actread);

	if(src[0] == 0x50 && src[1] == 0x4B && src[2] == 0x03 && src[3] == 0x04)
	{
		//printf (".imz/.zip file \n\r") ;

		/* skip header */
		i = 30;
		
		if (src[8] != DEFLATED)
			if (src[2] != DEFLATED) {
				puts ("Error: Bad zipped data\n");
				free(src);
				free(writebuf);
				return (-1);
			}		
		if(src[26] || src[27])
		{
			i +=  (src[27] << 2) |  src[26];
		}
		if(src[28] || src[29])
		{
			memcpy(&szexpected,  src + i + 4 , sizeof(szexpected));
			szexpected = le64_to_cpu(szexpected);

			memcpy(&payload_size,  src + i + 12 , sizeof(payload_size));
			payload_size = le32_to_cpu(payload_size);

		
			i +=  (src[29] << 2) |  src[28];
		}
		else
		{
			u32 szcompressed;
			memcpy(&szcompressed,  src + 18, sizeof(szcompressed));
			payload_size = le32_to_cpu(szcompressed);

			u32 szuncompressed;
			memcpy(&szuncompressed,  src + 22 , sizeof(szuncompressed));
			szexpected = le32_to_cpu(szuncompressed);

		}
		
		
		memcpy(&expected_crc,  src + 14, sizeof(expected_crc));
		expected_crc = le32_to_cpu(expected_crc);

	}
	else
	{
		//printf (".gz file \n\r") ;

		/* skip header */
		i = 10;
		flags = src[3];
		if (src[2] != DEFLATED || (flags & RESERVED) != 0) {
			puts("Error: Bad gzipped data\n");
			free(src);
			free(writebuf);
			return -1;
		}
		if ((flags & EXTRA_FIELD) != 0)
			i = 12 + src[10] + (src[11] << 8);
		if ((flags & ORIG_NAME) != 0)
			while (src[i++] != 0)
				;
		if ((flags & COMMENT) != 0)
			while (src[i++] != 0)
				;
		if ((flags & HEAD_CRC) != 0)
			i += 2;

		if (i >= len-8) {
			puts("Error: gunzip out of data in header");
			free(src);
			free(writebuf);
			return -1;
		}

		payload_size = len - i - 8;

		res = fs_set_blk_dev(device, part, FS_TYPE_FAT);
		res = fs_read(filename, (ulong)src, len - 8, 8 , &actread);

		memcpy(&expected_crc,  src , sizeof(expected_crc));
		expected_crc = le32_to_cpu(expected_crc);
		u32 szuncompressed;
		memcpy(&szuncompressed, src + 4 , sizeof(szuncompressed));

		szexpected = le32_to_cpu(szuncompressed);
	}
	printf("\r\n");

	printf("CRC: 0x%x \r\n", expected_crc);
	printf("Uncompressed size: %llu \r\n", szexpected);
	printf("File size: %llu \r\n", payload_size);
	printf("Target media size: %llu \r\n", (u64)dev->blksz*dev->lba);

	printf("\r\n");

	MemBiggerThanImage = (dev->blksz*dev->lba) - szexpected;
	printf("Media is %lld kb %s than image \r\n",
		   MemBiggerThanImage > 0 ? MemBiggerThanImage>>10 : (-MemBiggerThanImage)>>10,
		   MemBiggerThanImage > 0 ? "bigger" : "smaller");

	if(!force)
	{
		if (lldiv(szexpected, dev->blksz) > (dev->lba - outblock)) {
			printf("%s: uncompressed size %llu exceeds device size %llu\n",
				__func__, szexpected, (u64)dev->blksz*dev->lba);
			free(src);
			free(writebuf);
			return -1;
		}
	}

	res = fs_set_blk_dev(device, part, FS_TYPE_FAT);
	res = fs_read(filename, (ulong)src, 0, BUFFERSIZE , &actread);

	gzwrite_progress_init(szexpected);

	s.zalloc = gzalloc;
	s.zfree = gzfree;

	r = inflateInit2(&s, -MAX_WBITS);
	if (r != Z_OK) {
		printf("Error: inflateInit2() returned %d\n", r);
		free(src);
		free(writebuf);
		return -1;
	}

	memcpy(src+BUFFERSIZE, src + i, BUFFERSIZE-i);

	s.next_in = src+BUFFERSIZE;
	s.avail_in = BUFFERSIZE-i;
	//writebuf = (unsigned char *)malloc_cache_aligned(szwritebuf);

	u64 readpos = 0;
	int stop_reading = 0;

	/* decompress until deflate stream ends or end of file */
	do {
		if (s.avail_in == 0) {
			printf("%s: weird termination with result %d\n",
			       __func__, r);
			break;
		}

		/* run inflate() on input until output buffer not full */
		do {
			unsigned long blocks_written; // no of blocks actually written
			int numfilled;
			lbaint_t writeblocks; // no of blocks to be written

			
			s.avail_out = szwritebuf;
			s.next_out = writebuf;
			r = inflate(&s, Z_SYNC_FLUSH);
			if ((r != Z_OK) &&
			    (r != Z_STREAM_END)) {
				printf("Error: inflate() returned %d\n", r);
				goto out;
			}

			numfilled = szwritebuf - s.avail_out;
			crc = crc32(crc, writebuf, numfilled);
			totalfilled += numfilled;
			if (numfilled < szwritebuf) {
				writeblocks = (numfilled+dev->blksz-1)
						/ dev->blksz;
				memset(writebuf+numfilled, 0,
				       dev->blksz-(numfilled%dev->blksz));
			} else {
				writeblocks = blksperbuf;
			}

			if((s.avail_in < BUFFERSIZE/2) && !stop_reading)
			{
				readpos += BUFFERSIZE;

				res = fs_set_blk_dev(device, part, FS_TYPE_FAT);
				res = fs_read(filename, (ulong)src, readpos, BUFFERSIZE , &actread);

				if(actread < BUFFERSIZE)
				{
					stop_reading = 1;
				}

				memmove(src+BUFFERSIZE, s.next_in, s.avail_in);
				memcpy(src+BUFFERSIZE+s.avail_in, src, actread);

				s.avail_in += actread;
				s.next_in = src+BUFFERSIZE;

			}

			gzwrite_progress(iteration++,
					 totalfilled,
					 szexpected);
/*
			if(count_blocks != (lbaint_t)-1 &&
			   outblock + writeblocks > skip_blocks + count_blocks)
			{
				writeblocks = skip_blocks + count_blocks - outblock;
				stop = 1;
			}
*/
			if(outblock + writeblocks > skip_blocks)
			{
				lbaint_t offset=0;

				if(skip_blocks > outblock)
					offset = skip_blocks - outblock;
				writeblocks -= offset;

				if(count_blocks != (lbaint_t)-1 &&
				   outblock + writeblocks > skip_blocks + count_blocks)
				{
					writeblocks = skip_blocks + count_blocks - outblock;
					stop = 1;
				}

				if(!start)
				{
					printf("Start writing %ld blocks at block %ld\n",
						   writeblocks, outblock + offset);
					start=1;
				}

				blocks_written = blk_dwrite(dev,
											outblock + offset,
											writeblocks,
											writebuf + offset * dev->blksz);
				if(blocks_written != writeblocks)
				{
					if(IS_ERR_VALUE(blocks_written))
					{
						printf("Write error on target device\n");
						r=-1;
					}
					else
					{
						printf("Target media full\n");
						r=0;
					}
					goto out; // ugly
				}
				outblock += blocks_written + offset;
			}
			else
				outblock += writeblocks;

			if (ctrlc()) {
				puts("abort\n");
				goto out;
			}
			WATCHDOG_RESET();
		} while (s.avail_out == 0 && !stop);
		/* done when inflate() says it's done */
	} while (r != Z_STREAM_END && !stop);

	if ((szexpected != totalfilled) ||
	    (crc != expected_crc))
		r = -1;
	else
		r = 0;

out:
	gzwrite_progress_finish(r, totalfilled, szexpected,
				expected_crc, crc);
	free(src);
	free(writebuf);
	inflateEnd(&s);

	time = get_timer(time);
	printf("%s restored in %lu ms \r\n", filename, time);

	printf("%"LBAFlength"u of %"LBAFlength"u blocks written \r\n",
		   outblock - skip_blocks, dev->lba);

	if( force && (MemBiggerThanImage != 0) )
	{
		if( MemBiggerThanImage > 0 )
		{	      
			printf("Image %lld kB Smaller than Storage device \r\n", MemBiggerThanImage>>10);
			restore_backup_gpt(dev);	    	    
		}else
		{
			printf("Storage %lld kB Smaller than Image device \r\n", (-MemBiggerThanImage)>>10);
			restore_backup_gpt(dev);
		}
	}	

	return r;
}



int gzwrite(unsigned char *src, int len,
	    struct blk_desc *dev,
	    unsigned long szwritebuf,
	    u64 startoffs,
	    u64 szexpected)
{
	int i, flags;
	z_stream s;
	int r = 0;
	unsigned char *writebuf;
	unsigned crc = 0;
	u64 totalfilled = 0;
	lbaint_t blksperbuf, outblock;
	u32 expected_crc;
	u32 payload_size;
	int iteration = 0;
	
	if (!szwritebuf ||
	    (szwritebuf % dev->blksz) ||
	    (szwritebuf < dev->blksz)) {
		printf("%s: size %lu not a multiple of %lu\n",
		       __func__, szwritebuf, dev->blksz);
		return -1;
	}

	if (startoffs & (dev->blksz-1)) {
		printf("%s: start offset %llu not a multiple of %lu\n",
		       __func__, startoffs, dev->blksz);
		return -1;
	}

	blksperbuf = szwritebuf / dev->blksz;
	outblock = lldiv(startoffs, dev->blksz);
	
	/* skip header */
	i = 10;
	flags = src[3];
	if (src[2] != DEFLATED || (flags & RESERVED) != 0) {
		puts("Error: Bad gzipped data\n");
		return -1;
	}
	if ((flags & EXTRA_FIELD) != 0)
		i = 12 + src[10] + (src[11] << 8);
	if ((flags & ORIG_NAME) != 0)
		while (src[i++] != 0)
			;
	if ((flags & COMMENT) != 0)
		while (src[i++] != 0)
			;
	if ((flags & HEAD_CRC) != 0)
		i += 2;

	if (i >= len-8) {
		puts("Error: gunzip out of data in header");
		return -1;
	}

	payload_size = len - i - 8;

	memcpy(&expected_crc, src + len - 8, sizeof(expected_crc));
	expected_crc = le32_to_cpu(expected_crc);
	u32 szuncompressed;
	memcpy(&szuncompressed, src + len - 4, sizeof(szuncompressed));
	if (szexpected == 0) {
		szexpected = le32_to_cpu(szuncompressed);
	} else if (szuncompressed != (u32)szexpected) {
		printf("size of %llx doesn't match trailer low bits %x\n",
		       szexpected, szuncompressed);
		return -1;
	}
	if (lldiv(szexpected, dev->blksz) > (dev->lba - outblock)) {
		printf("%s: uncompressed size %llu exceeds device size\n",
		       __func__, szexpected);
		return -1;
	}
	
	gzwrite_progress_init(szexpected);

	s.zalloc = gzalloc;
	s.zfree = gzfree;

	r = inflateInit2(&s, -MAX_WBITS);
	if (r != Z_OK) {
		printf("Error: inflateInit2() returned %d\n", r);
		return -1;
	}

	s.next_in = src + i;
	s.avail_in = payload_size+8;
	writebuf = (unsigned char *)malloc_cache_aligned(szwritebuf);
		
	/* decompress until deflate stream ends or end of file */
	do {
		if (s.avail_in == 0) {
			printf("%s: weird termination with result %d\n",
			       __func__, r);
			break;
		}

		/* run inflate() on input until output buffer not full */
		do {
			unsigned long blocks_written;
			int numfilled;
			lbaint_t writeblocks;
		
			s.avail_out = szwritebuf;
			s.next_out = writebuf;
			r = inflate(&s, Z_SYNC_FLUSH);
			if ((r != Z_OK) &&
			    (r != Z_STREAM_END)) {
				printf("Error: inflate() returned %d\n", r);
				goto out;
			}
			numfilled = szwritebuf - s.avail_out;
			crc = crc32(crc, writebuf, numfilled);
			totalfilled += numfilled;
			if (numfilled < szwritebuf) {
				writeblocks = (numfilled+dev->blksz-1)
						/ dev->blksz;
				memset(writebuf+numfilled, 0,
				       dev->blksz-(numfilled%dev->blksz));
			} else {
				writeblocks = blksperbuf;
			}

			gzwrite_progress(iteration++,
					 totalfilled,
					 szexpected);
			blocks_written = blk_dwrite(dev, outblock,
						    writeblocks, writebuf);
			outblock += blocks_written;
			if (ctrlc()) {
				puts("abort\n");
				goto out;
			}
			WATCHDOG_RESET();
		} while (s.avail_out == 0);
		/* done when inflate() says it's done */
	} while (r != Z_STREAM_END);

	if ((szexpected != totalfilled) ||
	    (crc != expected_crc))
		r = -1;
	else
		r = 0;

out:
	gzwrite_progress_finish(r, totalfilled, szexpected,
				expected_crc, crc);
	free(writebuf);
	inflateEnd(&s);

	return r;
}
#endif

/*
 * Uncompress blocks compressed with zlib without headers
 */
int zunzip(void *dst, int dstlen, unsigned char *src, unsigned long *lenp,
						int stoponerr, int offset)
{
	z_stream s;
	int err = 0;
	int r;

	s.zalloc = gzalloc;
	s.zfree = gzfree;

	r = inflateInit2(&s, -MAX_WBITS);
	if (r != Z_OK) {
		printf("Error: inflateInit2() returned %d\n", r);
		return -1;
	}
	s.next_in = src + offset;
	s.avail_in = *lenp - offset;
	s.next_out = dst;
	s.avail_out = dstlen;
	do {
		r = inflate(&s, Z_FINISH);
		if (stoponerr == 1 && r != Z_STREAM_END &&
		    (s.avail_in == 0 || s.avail_out == 0 || r != Z_BUF_ERROR)) {
			printf("Error: inflate() returned %d\n", r);
			err = -1;
			break;
		}
	} while (r == Z_BUF_ERROR);
	*lenp = s.next_out - (unsigned char *) dst;
	inflateEnd(&s);

	return err;
}


#define GPT_HEAD_BLOCK 1
#define BLOCK_COUNT 1
//#define PART_ENTRY_START 2
#define PART_BLOCKS_MAX 33

/**
 * restore the backup gpt after an image has been written
 * that does not exactly fit the target device
 */
int restore_backup_gpt(struct blk_desc *dev)
{
	int error=0;
	void *buffer;
	unsigned long count;
	gpt_header *header;
	uint64_t part_tab1_lba;
	uint64_t part_tab2_lba;
	uint64_t part_tab_blocks;
	lbaint_t block_no;

	printf("device number:\t\t%d\n",dev->devnum);
	printf("number of blocks:\t%"LBAFlength"d\n",dev->lba);
	printf("block size:\t\t%ld\n",dev->blksz);

	buffer=malloc(dev->blksz); // there are several mallocs
	if(!buffer)
		return -ENOMEM;

	// read primary gpt
	printf("read primary GPT header from block %d\n",GPT_HEAD_BLOCK);
	count=blk_dread(dev, GPT_HEAD_BLOCK, BLOCK_COUNT, buffer);
	if(!IS_ERR_VALUE(count))
	{
		uint32_t crc;

		header=(gpt_header *)buffer;
		if(header->signature==GPT_HEADER_SIGNATURE_UBOOT)
		{
			printf("header size: %d\n",header->header_size);
			printf("header crc: 0x%08x\n",header->header_crc32);
			printf("primary GPT header on block %lld\n",header->my_lba);
			printf("secondary GPT header on block %lld\n",header->alternate_lba);
			printf("%d partitions on media\n",header->num_partition_entries);

			// read start and size of the partition tables
			part_tab1_lba=le64_to_cpu(header->partition_entry_lba);
			part_tab2_lba=dev->lba-PART_BLOCKS_MAX;
			count=le32_to_cpu(header->num_partition_entries)*
				le32_to_cpu(header->sizeof_partition_entry);
			part_tab_blocks=count/dev->blksz;
			if(count%dev->blksz)
				part_tab_blocks++;
			// adjust the position of the secondary gpt
			header->alternate_lba=cpu_to_le64(dev->lba-GPT_HEAD_BLOCK);
			printf("secondary GPT header will be on block %lld\n",
				   header->alternate_lba);
			// ... and last usable LBA
			printf("Last usable LBA will be %lld\n",part_tab2_lba-1);
			header->last_usable_lba=cpu_to_le64(part_tab2_lba-1);
			// update checksum
			header->header_crc32=0;
			crc=crc32(0, (const unsigned char *)header,
					  le32_to_cpu(header->header_size));
			printf("new crc: 0x%08x\n",crc);
			header->header_crc32=cpu_to_le32(crc);
			// write back
			printf("write primary GPT header to block %lld\n",
				   (uint64_t)GPT_HEAD_BLOCK);
			count=blk_dwrite(dev, GPT_HEAD_BLOCK, BLOCK_COUNT, buffer);
			if(!IS_ERR_VALUE(count))
			{
				// adjust positions of gpt
				header->my_lba=cpu_to_le64(dev->lba-GPT_HEAD_BLOCK);
				header->alternate_lba=cpu_to_le64(GPT_HEAD_BLOCK);
				header->partition_entry_lba=cpu_to_le64(part_tab2_lba);
				// update checksum
				header->header_crc32=0;
				crc=crc32(0, (const unsigned char *)header,
						  le32_to_cpu(header->header_size));
				printf("new crc: 0x%08x\n",crc);
				header->header_crc32=cpu_to_le32(crc);
				// write back
				printf("write secondary GPT header to block %lld\n",
					   header->my_lba);
				count=blk_dwrite(dev, header->my_lba, BLOCK_COUNT, buffer);
				if(IS_ERR_VALUE(count))
				{
					printf("Failed to write secundary GPT header\n");
					error=-count;
				}
			}
			else
			{
				printf("Failed to write primary GPT header\n");
				error=-count;
			}
		}
		else
		{
			printf("No GUID partion table\n");
			error=-EINVAL;
		}
	}
	else
	{
		printf("Failed to read primary GPT header\n");
		error=-count;
	}

	// copy the partition table
	if(!error)
		printf("copy partition table from %lld to %lld\n"
			   ,part_tab1_lba,part_tab2_lba);
	for(block_no=0; !error && block_no < part_tab_blocks; block_no++)
	{
		count=blk_dread(dev, part_tab1_lba+block_no,
						BLOCK_COUNT, buffer);
		if(!IS_ERR_VALUE(count))
			count=blk_dwrite(dev, part_tab2_lba+block_no,
							 BLOCK_COUNT, buffer);
		if(IS_ERR_VALUE(count))
		{
			printf("Failed to copy partition table\n");
			error=-count;
		}
	}
	
	free(buffer);
	return error;
}
