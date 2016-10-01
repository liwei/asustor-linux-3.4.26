/*
 * NAND Flash Controller Device Driver
 * Copyright (c) 2008-2010, Intel Corporation and its suppliers.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin St - Fifth Floor, Boston, MA 02110-1301 USA.
 *
 */



#include "lld.h"
#include "lld_emu.h"
#ifdef ELDORA   // Code for LLD_EMU under Eldora LLD
#include "defs.h"
#else   // Code for LLD_EMU under Spectra LLD
#include "testcnf.h"
#include "flash.h"
#include "ffsdefs.h"
#endif

#if (CMD_DMA  && FLASH_EMU)
#include "lld_cdma.h"
uint32  GLOB_totalUsedBanks;
uint32  GLOB_valid_banks[LLD_NUM_FLASH_CHANNELS];
uint32  GLOB_cmd_tag_count;
BLOCKNODE discarded_blocks[LLD_NUM_FLASH_CHANNELS];
#endif

#if FLASH_EMU    // this is for entire module

//unsigned char   *GLOB_flash_memory = NULL;   //&& flag for valid flash array malloc
//extern uint16   FIQ_FLAG;               // global flag for interrupt testing; sb just a temp
extern byte     *g_pMemPoolFree;
unsigned char * GLOB_flash_memory[GLOB_LLD_BLOCKS*GLOB_LLD_PAGES];

#if CMD_DMA
extern PENDING_CMD     GLOB_PendingCMD[MAX_DESCRIPTORS + LLD_NUM_FLASH_CHANNELS];
#if TEST_FTL

extern byte forced_error;
extern byte forced_error_index;
extern uint32 forced_error_pba;
extern byte tag_for_forced_error_command ;
extern struct uncorrectable_error_tag
{
    int random_write_index;
    BLOCKNODE   block_num;
    PAGENUMTYPE page_num;
    BLOCKNODE   pba;
}uncorrectable_error[32];
#endif
#endif


#if	EMU_BAD_BLOCK
static uint32 rand_seed = 0;

BLOCKNODE  bad_blocks_array[MAX_BAD_BLOCKS];


void 	emu_init_bad_block_info(void)
{
    unsigned int i;
    for ( i = 0; i < MAX_BAD_BLOCKS; i++)
    {
        bad_blocks_array[i] = -1;
    }
}
void 	emu_append_bad_block_info(BLOCKNODE *array, unsigned int len)
{
    unsigned int i,j;
#if 0
    rand_seed = 1226360438;
    printf("random seed %lu for finding random bad blocks\n", rand_seed);


    srand(rand_seed);
#endif
    for(j=0; (j<MAX_BAD_BLOCKS) && (bad_blocks_array[j] != -1); j++);

    for ( i = j; (i < MAX_BAD_BLOCKS) && (i-j<len); i++)
    {
        bad_blocks_array[i] = array[i-j];
    }
}

uint16 	emu_bad_block_check(BLOCKNODE  block)
{
    unsigned int i;

    for ( i = 0; i < MAX_BAD_BLOCKS; i++)
    {
	if (bad_blocks_array[i] == block)
	{
	    return FAIL;
	}
    }

    return PASS;
}
#endif

#define GLOB_LLD_PAGE_SPARE_SIZE  (GLOB_LLD_PAGE_SIZE - GLOB_LLD_PAGE_DATA_SIZE)
#define MEMFILENAME   "spectra_emu_file"

#if FLASH_EMU_STATIC_MALLOC

int emu_load_file_to_mem(void)
{
    int rc = 0;

#if FLASH_EMU_STATIC_MALLOC
#ifdef LINUX_DEVICE_DRIVER
	mm_segment_t  fs;
	struct file   *nef_filp = NULL;
	struct inode  *inode = NULL;
	loff_t        nef_size = 0;
	loff_t        tmp_file_offset, file_offset;
	ssize_t	      nread;
	int           i;

    rc = -EINVAL;
	print("%s, Line %d, Function: %s\n",
	      __FILE__, __LINE__, __FUNCTION__);

	fs = get_fs();
	set_fs(get_ds());

	nef_filp = filp_open("/root/nand_emu_file",
			     O_RDWR | O_LARGEFILE, 0);
	if (IS_ERR(nef_filp)) {
		printk(KERN_ERR "filp_open error: "
		       "Unable to open nand emu file!\n");
		return PTR_ERR(nef_filp);
	}

	if (nef_filp->f_path.dentry) {
		inode = nef_filp->f_path.dentry->d_inode;
	} else {
		printk(KERN_ERR "Can not get valid inode!\n");
		goto out;
	}

	nef_size = i_size_read(inode->i_mapping->host);
	if (nef_size <= 0) {
		printk(KERN_ERR "Invalid nand emu file size: 0x%llx\n",
		       nef_size);
		goto out;
	} else {
		printk(KERN_ALERT "nand emu file size: %lld\n",
		       nef_size);
	}

	file_offset = 0;
	for (i = 0; i < GLOB_LLD_BLOCKS * GLOB_LLD_PAGES; i++) {
		tmp_file_offset = file_offset;
		nread = vfs_read(nef_filp,
				 (char __user *)GLOB_flash_memory[i],
				 GLOB_LLD_PAGE_SIZE,
				 &tmp_file_offset);
		if (nread < GLOB_LLD_PAGE_SIZE) {
			printk(KERN_ERR "%s, Line %d - nand emu file "
			       "partial read: %d bytes\n",
			       __FILE__, __LINE__, (int)nread);
			goto out;
		}
		file_offset  += GLOB_LLD_PAGE_SIZE;
	}
	rc = 0;

out:
	filp_close(nef_filp, current->files);
	set_fs(fs);
#else
    FILE *fp;
    int i;
    uint32 blocks, pages, page_data_size, page_spare_size;

    fp = fopen(MEMFILENAME, "rb");
    if (!fp) {
        print("MEMFILE could not load file %s\n", MEMFILENAME);
        rc = -1;
    }

    if (!rc) {
        fscanf(fp, "%d:%d:%d:%d:", &blocks, &pages, &page_data_size, &page_spare_size);
        if ((blocks == GLOB_LLD_BLOCKS) &&
            (pages == GLOB_LLD_PAGES) &&
            (page_data_size == GLOB_LLD_PAGE_DATA_SIZE) &&
            (page_spare_size == GLOB_LLD_PAGE_SPARE_SIZE)) {
            for (i = 0; (i < GLOB_LLD_PAGE_SIZE * GLOB_LLD_BLOCKS * GLOB_LLD_PAGES) && (!feof(fp)); i++) {
              *(GLOB_flash_memory[0] + i) = fgetc(fp);
            }
            if (i<GLOB_LLD_PAGE_SIZE * GLOB_LLD_BLOCKS * GLOB_LLD_PAGES)
              printf("MEMFILE Could not read complete memory\n");
        }
        else {
            printf("MEMFILE contains flash geometry :%d:%d:%d:%d:. Does not match current geometry of :%d:%d:%d:%d:. Not read.\n",
                blocks, pages, page_data_size, page_spare_size,
                GLOB_LLD_BLOCKS, GLOB_LLD_PAGES, GLOB_LLD_PAGE_DATA_SIZE, GLOB_LLD_PAGE_SPARE_SIZE);
            rc = -1;
        }
        fclose(fp);
    }
#endif
#endif
	return rc;
}


int emu_write_mem_to_file(void)
{
    int rc = 0;

#if FLASH_EMU_STATIC_MALLOC
#ifdef LINUX_DEVICE_DRIVER
	mm_segment_t  fs;
	struct file   *nef_filp = NULL;
	struct inode  *inode = NULL;
	loff_t        nef_size = 0;
	loff_t        tmp_file_offset, file_offset;
	ssize_t	      nwritten;
	int           i;

    rc = -EINVAL;
	print("%s, Line %d, Function: %s\n",
	      __FILE__, __LINE__, __FUNCTION__);

	fs = get_fs();
	set_fs(get_ds());

	nef_filp = filp_open("/root/nand_emu_file",
			     O_RDWR | O_LARGEFILE, 0);
	if (IS_ERR(nef_filp)) {
		printk(KERN_ERR "filp_open error: "
		       "Unable to open nand emu file!\n");
		return PTR_ERR(nef_filp);
	}

	if (nef_filp->f_path.dentry) {
		inode = nef_filp->f_path.dentry->d_inode;
	} else {
		printk(KERN_ERR "Invalid nef_filp->f_path.dentry value!\n");
		goto out;
	}

	nef_size = i_size_read(inode->i_mapping->host);
	if (nef_size <= 0) {
		printk(KERN_ERR "Invalid nand emu file size: 0x%llx\n",
		       nef_size);
		goto out;
	} else {
		printk(KERN_ALERT "nand emu file size: %lld\n",
		       nef_size);
	}

	file_offset = 0;
	for (i = 0; i < GLOB_LLD_BLOCKS * GLOB_LLD_PAGES; i++) {
		tmp_file_offset = file_offset;
		nwritten = vfs_write(nef_filp,
				     (char __user *)GLOB_flash_memory[i],
				     GLOB_LLD_PAGE_SIZE,
				     &tmp_file_offset);
		if (nwritten < GLOB_LLD_PAGE_SIZE) {
			printk(KERN_ERR "%s, Line %d - nand emu file "
			       "partial write: %d bytes\n",
			       __FILE__, __LINE__, (int)nwritten);
			goto out;
		}
		file_offset  += GLOB_LLD_PAGE_SIZE;
	}
	rc = 0;

out:
	filp_close(nef_filp, current->files);
	set_fs(fs);
#else
    FILE *fp;
    int i;

    fp = fopen(MEMFILENAME, "wb");
    if (!fp) {
        print("MEMFILE could not open file %s\n", MEMFILENAME);
        rc = -1;
    }

    if (!rc) {
        fprintf(fp, "%d:%d:%d:%d:", GLOB_LLD_BLOCKS, GLOB_LLD_PAGES, GLOB_LLD_PAGE_DATA_SIZE, GLOB_LLD_PAGE_SPARE_SIZE);
	    for (i = 0; (i < (GLOB_LLD_PAGE_SIZE * GLOB_LLD_BLOCKS * GLOB_LLD_PAGES)); i++) {
          fputc(*(GLOB_flash_memory[0] + i), fp);
	    }
        fclose(fp);
    }
#endif
#endif
	return rc;
}
#endif
/*&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
* Function:     emu_Flash_Init
* Inputs:       none
* Outputs:      PASS=0 (notice 0=ok here)
* Description:  Creates & initializes the flash RAM array.
*
*&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&*/
uint16 emu_Flash_Init()
{
    int i;
#if FLASH_EMU_STATIC_MALLOC
    GLOB_flash_memory[0] = (unsigned char
		       *)GLOB_MALLOC(GLOB_LLD_PAGE_SIZE * GLOB_LLD_BLOCKS*GLOB_LLD_PAGES * sizeof(unsigned char));
    memset((char *)(GLOB_flash_memory[0]), 0xFF,
	   GLOB_LLD_PAGE_SIZE * GLOB_LLD_BLOCKS*GLOB_LLD_PAGES * sizeof(unsigned char));
#else
    GLOB_flash_memory[0] = NULL;
#endif
    for(i=1;i<GLOB_LLD_BLOCKS*GLOB_LLD_PAGES;i++) {
#if FLASH_EMU_STATIC_MALLOC
      GLOB_flash_memory[i] = GLOB_flash_memory[i-1] + GLOB_LLD_PAGE_SIZE;
#else
    GLOB_flash_memory[i] = NULL;
#endif
    }

#if EMU_BAD_BLOCK
    emu_init_bad_block_info();
#endif

#if FLASH_EMU_STATIC_MALLOC
    emu_load_file_to_mem();
#endif
    return PASS;
}

/*&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
* Function:     emu_Flash_Release
* Inputs:       none
* Outputs:      PASS=0 (notice 0=ok here)
* Description:          Releases the flash.
*
*&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&*/
uint16 emu_Flash_Release()
{
    int i;

#if FLASH_EMU_STATIC_MALLOC
    emu_write_mem_to_file();
#endif

#if FLASH_EMU_STATIC_MALLOC
    i = 0;
    GLOB_FREE(GLOB_flash_memory[0]);
#else
    for(i=0;i<GLOB_LLD_BLOCKS*GLOB_LLD_PAGES;i++)
    {
        if (GLOB_flash_memory[i]) {
#ifdef ELDORA
            free(GLOB_flash_memory[i]);
#else
            GLOB_FREE(GLOB_flash_memory[i]);
#endif
            GLOB_flash_memory[i] = NULL;
        }
    }
#endif
    return PASS;
}

/*&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
* Function:     emu_Read_Device_ID
* Inputs:       none
* Outputs:      PASS=1 FAIL=0
* Description:  Reads the info from the controller registers.
*               Sets up GLOB_DeviceInfo structure with device parameters
*&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&*/

uint16 emu_Read_Device_ID()
{
    uint16  status = PASS;

    GLOB_DeviceInfo.wDeviceMaker    = 0xBB;
    GLOB_DeviceInfo.wDeviceType     = 4;
    GLOB_DeviceInfo.wSpectraStartBlock = 3;
    GLOB_DeviceInfo.wSpectraEndBlock  = GLOB_LLD_BLOCKS-1;
    GLOB_DeviceInfo.wTotalBlocks    = GLOB_LLD_BLOCKS;
    GLOB_DeviceInfo.wPagesPerBlock  = GLOB_LLD_PAGES;
    GLOB_DeviceInfo.wPageSize       = GLOB_LLD_PAGE_SIZE;
    GLOB_DeviceInfo.wPageDataSize   = GLOB_LLD_PAGE_DATA_SIZE;
    GLOB_DeviceInfo.wPageSpareSize  = GLOB_LLD_PAGE_SIZE - GLOB_LLD_PAGE_DATA_SIZE;
    GLOB_DeviceInfo.wBlockSize      = GLOB_DeviceInfo.wPageSize * GLOB_LLD_PAGES;
    GLOB_DeviceInfo.wBlockDataSize  = GLOB_DeviceInfo.wPageDataSize * GLOB_LLD_PAGES;
    GLOB_DeviceInfo.wDataBlockNum   = (BLOCKNODE)(GLOB_DeviceInfo.wSpectraEndBlock - GLOB_DeviceInfo.wSpectraStartBlock+1);
    GLOB_DeviceInfo.MLCDevice       = EMU_MLC_DEV;

    GLOB_DeviceInfo.nBitsInPageNumber = (byte)GLOB_Calc_Used_Bits(GLOB_DeviceInfo.wPagesPerBlock);
    GLOB_DeviceInfo.nBitsInPageDataSize = (byte)GLOB_Calc_Used_Bits(GLOB_DeviceInfo.wPageDataSize);
    GLOB_DeviceInfo.nBitsInBlockDataSize = (byte)GLOB_Calc_Used_Bits(GLOB_DeviceInfo.wBlockDataSize);
#if CMD_DMA
    GLOB_totalUsedBanks = 4;
    GLOB_valid_banks[0] = 1;
    GLOB_valid_banks[1] = 1;
    GLOB_valid_banks[2] = 1;
    GLOB_valid_banks[3] = 1;
#endif

    return status;
}


/*&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
* Function:     emu_Flash_Reset
* Inputs:       none
* Outputs:      PASS=0 (notice 0=ok here)
* Description:          Reset the flash
*
*&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&*/
uint16 emu_Flash_Reset (void)
{
    return PASS;
}


/*&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
* Function:     emu_Erase_Block
* Inputs:       Address
* Outputs:      PASS=0 (notice 0=ok here)
* Description:          Erase a block
*
*&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&*/
uint16 emu_Erase_Block(BLOCKNODE block_add)
{
    uint16 status = PASS;
    int i;


    if(block_add >= GLOB_DeviceInfo.wTotalBlocks)
    {
        status = FAIL;
    }

    if ((PASS == status) && !GLOB_flash_memory)
    {
        status = FAIL;
    }

    print("ERASING BLOCK %d\n",block_add);

#if EMU_BAD_BLOCK
    if ( FAIL == emu_bad_block_check(block_add))
    {
	status = FAIL;
    }
#endif

    if (PASS == status)
    {
        for(i=block_add*GLOB_LLD_PAGES; i<((block_add+1)*GLOB_LLD_PAGES); i++)
            if (GLOB_flash_memory[i]) {
#if FLASH_EMU_STATIC_MALLOC
		memset((unsigned char *)(GLOB_flash_memory[i]), 0xFF,
				GLOB_DeviceInfo.wPageSize*sizeof(unsigned char));
#else
#ifdef ELDORA
                free(GLOB_flash_memory[i]);
#else
                GLOB_FREE(GLOB_flash_memory[i]);
#endif
                GLOB_flash_memory[i] = NULL;
#endif
            }
    }

    return status;
}

/*&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
* Function:     emu_Write_Page_Main
* Inputs:       Write buffer address pointer
*               Block number
*               Page  number
*               Number of pages to process
* Outputs:      PASS=0 (notice 0=ok here)
* Description:  Write the data in the buffer to main area of flash
*
*&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&*/
uint16 emu_Write_Page_Main(byte * write_data, BLOCKNODE Block, PAGENUMTYPE Page, PAGENUMTYPE PageCount)
{
    int i;
    int status = PASS;

    if(Block >= GLOB_DeviceInfo.wTotalBlocks)
    {
	status = FAIL;
    }

    if(Page+PageCount > GLOB_DeviceInfo.wPagesPerBlock)
    {
	   status = FAIL;
    }

    if ((PASS == status) && !GLOB_flash_memory)
    {
	    status = FAIL;
    }
      print("emu_Write_Page_Main: lba %u Page %u PageCount\
%u\n",(unsigned int)Block,(unsigned int)Page,(unsigned int)PageCount);

#if EMU_BAD_BLOCK
    if ( FAIL == emu_bad_block_check(Block))
    {
	status = FAIL;
    }
#endif

    if (PASS == status)
    {
        for(i=0; i<PageCount; i++)
        {
            if(GLOB_flash_memory[Block*GLOB_LLD_PAGES+ Page]== NULL)
            {
		GLOB_flash_memory[Block*GLOB_LLD_PAGES+ Page] = (unsigned char
                    *)GLOB_MALLOC(GLOB_DeviceInfo.wPageSize*sizeof(unsigned char));
                if(GLOB_flash_memory[Block*GLOB_LLD_PAGES+ Page]== NULL)
                {
                  print("RAN OUT OF MEMORY\n");
                  return FAIL;
                }
		memset((char *)(GLOB_flash_memory[Block*GLOB_LLD_PAGES+ Page]) + GLOB_DeviceInfo.wPageDataSize,0xFF,
				(GLOB_DeviceInfo.wPageSize-GLOB_DeviceInfo.wPageDataSize)*sizeof(unsigned char));
            }
            memcpy((byte *)(GLOB_flash_memory[Block*GLOB_LLD_PAGES+ Page]), write_data, GLOB_DeviceInfo.wPageDataSize);
            write_data +=GLOB_DeviceInfo.wPageDataSize;
            Page++;
        }
    }

    return status;
}

/*&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
* Function:     emu_Read_Page_Main
* Inputs:       Read buffer address pointer
*               Block number
*               Page  number
*               Number of pages to process
* Outputs:      PASS=0 (notice 0=ok here)
* Description:  Read the data from the flash main area to the buffer
*
*&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&*/
uint16 emu_Read_Page_Main(byte * read_data, BLOCKNODE Block, PAGENUMTYPE Page, PAGENUMTYPE  PageCount)
{
    int i, status = PASS;

    if(Block >= GLOB_DeviceInfo.wTotalBlocks)
    {
        status = FAIL;
    }

    if(Page+PageCount > GLOB_DeviceInfo.wPagesPerBlock)
    {
        status = FAIL;
    }

    if ((PASS == status) && !GLOB_flash_memory)
    {
        status = FAIL;
    }

   print("emu_Read_Page_Main: lba %u Page %u PageCount\
%u\n",(unsigned int)Block,(unsigned int)Page,(unsigned int)PageCount);

#if EMU_BAD_BLOCK
    if ( FAIL == emu_bad_block_check(Block))
    {
	   status = FAIL;
    }
#endif

    if (PASS == status)
    {
        for(i=0; i<PageCount; i++)
        {
            if(GLOB_flash_memory[Block*GLOB_LLD_PAGES+ Page]== NULL) {
                memset(read_data,0xFF,GLOB_DeviceInfo.wPageDataSize);
            } else {
		memcpy(read_data,(byte *)(GLOB_flash_memory[Block*GLOB_LLD_PAGES+ Page]), GLOB_DeviceInfo.wPageDataSize);
            }
            read_data +=GLOB_DeviceInfo.wPageDataSize;
            Page++;
        }
    }
    return status;
}

#ifndef ELDORA
/*&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
* Function:     emu_Read_Page_Main_Spare
* Inputs:       Write Buffer
*                       Address
*                       Buffer size
* Outputs:      PASS=0 (notice 0=ok here)
* Description:          Read from flash main+spare area
*
*&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&*/
uint16 emu_Read_Page_Main_Spare(byte* read_data,BLOCKNODE Block,PAGENUMTYPE Page,PAGENUMTYPE PageCount)
{
    int i, status = PASS;

    if(Block >= GLOB_DeviceInfo.wTotalBlocks) {
    printstr("Read Page Main+Spare:Error has occured: Block Address too big\n");
    status = FAIL;
    }

    if(Page+PageCount > GLOB_DeviceInfo.wPagesPerBlock) {
    printstr("Read Page Main+Spare:Error has occured: Page Address too big\n");
    status = FAIL;
    }

    if ((PASS == status) && !GLOB_flash_memory)
    {
        print("Read Page Main+Spare: No allocated for operations\n");
        status = FAIL;
    }
#if EMU_BAD_BLOCK
    if ( FAIL == emu_bad_block_check(Block))
    {
	status = FAIL;
    }
#endif

    if (PASS == status) {
    print("\n Read Page Main+Spare - No. of pages %u block %u start page %u\n",(unsigned int)PageCount,(unsigned int)Block,(unsigned int)Page);
    for(i=0; i<PageCount; i++)
    {
        if(GLOB_flash_memory[Block*GLOB_LLD_PAGES+ Page]== NULL) {
            memset(read_data,0xFF,GLOB_DeviceInfo.wPageSize);
        } else {
            memcpy(read_data,(byte *)(GLOB_flash_memory[Block*GLOB_LLD_PAGES+ Page]), GLOB_DeviceInfo.wPageSize);
        }
        read_data +=GLOB_DeviceInfo.wPageSize;
        Page++;
    }
    }
    return status;
}


/*&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
* Function:     emu_Write_Page_Main_Spare
* Inputs:       Write buffer
*                       address
*                       buffer length
* Outputs:      PASS=0 (notice 0=ok here)
* Description:          Write the buffer to main+spare area of flash
*
*&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&*/
uint16 emu_Write_Page_Main_Spare(byte* write_data,BLOCKNODE Block,PAGENUMTYPE Page,PAGENUMTYPE page_count)
{
    uint16  i, status = PASS;

    print(" Write Page Main+Spare - block %u page %u \n",(unsigned int)Block,(unsigned int)Page);

    if(Block >= GLOB_DeviceInfo.wTotalBlocks) {
        printstr("Write Page Main+Spare:Error has occured: Block Address too big\n");
        status = FAIL;
    }

    if(Page+page_count > GLOB_DeviceInfo.wPagesPerBlock) {
        printstr("Write Page Main+Spare:Error has occured: Page Address too big\n");
        status = FAIL;
    }

#if EMU_BAD_BLOCK
    if ( FAIL == emu_bad_block_check(Block))
    {
	status = FAIL;
    }
#endif
    if ((PASS == status) && !GLOB_flash_memory)
    {
	    print("Write Page Main+Spare: No allocated for operations\n");
	    status = FAIL;
    }

    if (PASS == status) {
	print("\n Write Page Main+Spare - No. of pages %u block %u start page %u\n",(unsigned int)page_count,(unsigned int)Block,(unsigned int)Page);

	for(i=0; i<page_count; i++)
	{
        if(GLOB_flash_memory[Block*GLOB_LLD_PAGES+ Page]== NULL)
        {
            GLOB_flash_memory[Block*GLOB_LLD_PAGES+ Page] = (unsigned char
                    *)GLOB_MALLOC(GLOB_DeviceInfo.wPageSize*sizeof(unsigned char));
            if(GLOB_flash_memory[Block*GLOB_LLD_PAGES+ Page]== NULL)
            {
                print("RAN OUT OF MEMORY\n");
                return FAIL;
            }
        }

        memcpy((byte *)(GLOB_flash_memory[Block*GLOB_LLD_PAGES + Page]),write_data,GLOB_DeviceInfo.wPageSize);
	    write_data +=GLOB_DeviceInfo.wPageSize;              // inc src data pointer
	    Page++;
    }
    }

    return status;
}



/*&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
* Function:     emu_Write_Page_Spare
* Inputs:       Write buffer
*                       Address
*                       buffer size
* Outputs:      PASS=0 (notice 0=ok here)
* Description:          Write the buffer in the spare area
*
*&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&*/
uint16 emu_Write_Page_Spare(byte* write_data,BLOCKNODE Block,PAGENUMTYPE Page,PAGENUMTYPE PageCount)
{
    int status = PASS;

    if(Block >= GLOB_DeviceInfo.wTotalBlocks) {
	printstr("Read Page Spare:Error has occured: Block Address too big\n");
	status = FAIL;
    }

    if(Page+PageCount > GLOB_DeviceInfo.wPagesPerBlock) {
	printstr("Read Page Spare:Error has occured: Page Address too big\n");
	status = FAIL;
    }

#if EMU_BAD_BLOCK
    if ( FAIL == emu_bad_block_check(Block))
    {
	status = FAIL;
    }
#endif
    if ((PASS == status) && !GLOB_flash_memory)
    {
	    print("Write Page Spare: No allocated for operations\n");
	    status = FAIL;
    }

    if (PASS == status) {

	print(" Write Page Spare- block %u page %u \n",(unsigned int)Block,(unsigned
	  int)Page);

    if(GLOB_flash_memory[Block*GLOB_LLD_PAGES+ Page]== NULL)
            GLOB_flash_memory[Block*GLOB_LLD_PAGES+ Page] = (unsigned char
                    *)GLOB_MALLOC(GLOB_DeviceInfo.wPageSize*sizeof(unsigned char));

    if(GLOB_flash_memory[Block*GLOB_LLD_PAGES+ Page]== NULL)
    {
        print("RAN OUT OF MEMORY\n");
        return FAIL;
    }
    memcpy((byte *)(GLOB_flash_memory[Block*GLOB_LLD_PAGES+ Page] + GLOB_DeviceInfo.wPageDataSize), write_data, (GLOB_DeviceInfo.wPageSize -
	  GLOB_DeviceInfo.wPageDataSize));
    }
    return status;
}


/*&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
* Function:     emu_Read_Page_Spare
* Inputs:       Write Buffer
*                       Address
*                       Buffer size
* Outputs:      PASS=0 (notice 0=ok here)
* Description:          Read data from the spare area
*
*&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&*/
uint16 emu_Read_Page_Spare(byte* write_data, BLOCKNODE Block,PAGENUMTYPE Page,PAGENUMTYPE PageCount)
{
    int status = PASS;

    if(Block >= GLOB_DeviceInfo.wTotalBlocks) {
	printstr("Read Page Spare:Error has occured: Block Address too big\n");
	status = FAIL;
    }

    if(Page+PageCount > GLOB_DeviceInfo.wPagesPerBlock) {
	printstr("Read Page Spare:Error has occured: Page Address too big\n");
	status = FAIL;
    }

#if EMU_BAD_BLOCK
    if ( FAIL == emu_bad_block_check(Block))
    {
	status = FAIL;
    }
#endif
    if ((PASS == status) && !GLOB_flash_memory)
    {
	    print("Read Page Spare: No allocated for operations\n");
	    status = FAIL;
    }

    if (PASS == status) {
	print(" Read Page Spare- block %u page %u\n",(unsigned int)Block,(unsigned
	  int)Page);
    if(
       (GLOB_flash_memory[Block*GLOB_LLD_PAGES + Page] == NULL))
    {
        memset(write_data,0xFF,(GLOB_DeviceInfo.wPageSize-GLOB_DeviceInfo.wPageDataSize));
    } else
        memcpy(write_data,(byte *)(GLOB_flash_memory[Block*GLOB_LLD_PAGES+ Page] + GLOB_DeviceInfo.wPageDataSize),
                (GLOB_DeviceInfo.wPageSize-GLOB_DeviceInfo.wPageDataSize));
    }
    return status;
}

/*&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
* Function:     emu_Enable_Disable_Interrupts
* Inputs:       enable or disable
* Outputs:      none
* Description:  NOP
*&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&*/
void  emu_Enable_Disable_Interrupts(uint16 INT_ENABLE)
{
}

#if CMD_DMA
/*&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
* Support for CDMA functions
************************************
*       emu_CDMA_Flash_Init
*           CDMA_process_data command   (use LLD_CDMA)
*           CDMA_MemCopy_CMD            (use LLD_CDMA)
*       emu_CDMA_execute all commands
*       emu_CDMA_Event_Status
*&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&*/

#if DEBUG_SYNC
extern uint32  debug_sync_cnt;
#endif

/*&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
* Function:     CDMA_Flash_Init
* Inputs:       none
* Outputs:      PASS=0 (notice 0=ok here)
* Description:  This should be called at power up.
*               It disables interrupts and clears status bits
*               issues flash reset command
*               configures the controller registers
*               It sets the interrupt mask and enables interrupts
*               It pre-builds special descriptors
*&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&*/
uint16    emu_CDMA_Flash_Init (void)
{
uint16  i;






    for (i=0; i<MAX_DESCRIPTORS + LLD_NUM_FLASH_CHANNELS; i++)
    {
        GLOB_PendingCMD[i].CMD           = 0;
        GLOB_PendingCMD[i].Tag           = 0;
        GLOB_PendingCMD[i].DataAddr      = 0;
        GLOB_PendingCMD[i].Block         = 0;
        GLOB_PendingCMD[i].Page          = 0;
        GLOB_PendingCMD[i].PageCount     = 0;
        GLOB_PendingCMD[i].DataDestAddr  = 0;
        GLOB_PendingCMD[i].DataSrcAddr   = 0;
        GLOB_PendingCMD[i].MemCopyByteCnt= 0;
        GLOB_PendingCMD[i].SBDCmdIndex   = 0;
        GLOB_PendingCMD[i].ChanSync[0]   = 0;
        GLOB_PendingCMD[i].ChanSync[1]   = 0;
        GLOB_PendingCMD[i].ChanSync[2]   = 0;
        GLOB_PendingCMD[i].ChanSync[3]   = 0;
        GLOB_PendingCMD[i].ChanSync[4]   = 0;
        GLOB_PendingCMD[i].Status        = 3;
    }

    GLOB_cmd_tag_count = 0;

    return PASS;
}


/*&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
* Function:     CDMA_Data_Cmd
*               use the real function in LLD_CDMA
*&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&*/


/*&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
* Function:     CDMA_MemCopy_CMD
*               use the real function in LLD_CDMA
*&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&*/


/*&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
* Function:     CDMA_Execute_CMDs
* Inputs:       tag_count:  the number of pending cmds to do
* Outputs:      PASS/FAIL
* Description:  execute each command in the pending CMD array
*&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&*/
uint16  emu_CDMA_Execute_CMDs (uint16 tag_count)
{
uint16  i, j;
byte    CMD  ;  // cmd parameter
byte*   data ;
BLOCKNODE  block;
PAGENUMTYPE  page ;
PAGENUMTYPE  count;  // Page Count
uint16  status = PASS;
#if EMU_BAD_BLOCK
uint16  channel;
uint32  hit_bad_block = 0;
uint32  hit_sync_point = 0;
int     channels_with_bad_blocks[LLD_NUM_FLASH_CHANNELS];
#endif
#if TEST_FTL && LONG_RUN_FOR_UGLY_CASE_2
uint16  uncorrectable_error_occured =0;
int k;
#endif


    GLOB_cmd_tag_count = tag_count;

#if EMU_BAD_BLOCK
    for (i =0; i < LLD_NUM_FLASH_CHANNELS; i++)
    {
        channels_with_bad_blocks[i] = -1;
        discarded_blocks[i] = -1;
    }
#endif

    print("\n \n At start of Execute CMDs : Tag Count %u\n",GLOB_cmd_tag_count);
    for (i =0; i < GLOB_totalUsedBanks; i++)
    {
        GLOB_PendingCMD[i].CMD           = DUMMY_CMD;
        GLOB_PendingCMD[i].Tag           = 0xFF;
        GLOB_PendingCMD[i].SBDCmdIndex   = 0xFF;
        GLOB_PendingCMD[i].Block         = i * (GLOB_DeviceInfo.wTotalBlocks /
          GLOB_totalUsedBanks);
        for (j = 0; j<=LLD_NUM_FLASH_CHANNELS; j++) GLOB_PendingCMD[i].ChanSync[j] = 0;

        GLOB_PendingCMD[i].Status         = CMD_PASS;
    }

#if TEST_FTL
   GLOB_PendingCMD[LLD_NUM_FLASH_CHANNELS+GLOB_cmd_tag_count].CMD = 0; //needed to print only valid entry
#endif
   CDMA_Execute_CMDs(GLOB_cmd_tag_count);
#if DEBUG_SYNC
    if (!(debug_sync_cnt % DBG_SNC_PRINTEVERY)) {
        print("_%lu_",debug_sync_cnt);
#endif
#if VERBOSE
        PrintGLOB_PendingCMDs(GLOB_cmd_tag_count);
#endif
#if DEBUG_SYNC
#if VERBOSE
        PrintGLOB_PendingCMDsPerChannel(GLOB_cmd_tag_count);
#endif
    }
    debug_sync_cnt++;
#endif

    for (i=LLD_NUM_FLASH_CHANNELS; i<GLOB_cmd_tag_count+LLD_NUM_FLASH_CHANNELS; i++)
    {
        CMD     = GLOB_PendingCMD[i].CMD;
        data    = GLOB_PendingCMD[i].DataAddr;
        block   = GLOB_PendingCMD[i].Block;
        page    = GLOB_PendingCMD[i].Page;
        count   = GLOB_PendingCMD[i].PageCount;

#if TEST_FTL
#if LONG_RUN_FOR_UGLY_CASE_2
        for(k=0;k<32;k++)
            if(block == uncorrectable_error[k].block_num)
            {
                PAGENUMTYPE page_num;

                if(uncorrectable_error_occured)
                {
                      for(;i<GLOB_cmd_tag_count+LLD_NUM_FLASH_CHANNELS;i++)
                      GLOB_PendingCMD[i].Status = CMD_NOT_DONE;

                    return status;
                }
                else
                    uncorrectable_error_occured =1;

                for(page_num =0;page_num < GLOB_LLD_PAGES;page_num++)
                {
                    if(GLOB_flash_memory[block*GLOB_LLD_PAGES+ page_num] == NULL)
                        GLOB_flash_memory[block*GLOB_LLD_PAGES+ page_num] = (unsigned
						  char
                          *)GLOB_MALLOC(GLOB_DeviceInfo.wPageSize*sizeof(unsigned char));
                }
                GLOB_PendingCMD[i].Status = FAIL;
                i++;
                k=32;
                forced_error =1;
                CMD     = GLOB_PendingCMD[i].CMD;
                data    = GLOB_PendingCMD[i].DataAddr;
                block   = GLOB_PendingCMD[i].Block;
                page    = GLOB_PendingCMD[i].Page;
                count   = GLOB_PendingCMD[i].PageCount;
            }
#else
        if(forced_error && (i == (tag_for_forced_error_command+LLD_NUM_FLASH_CHANNELS)))
        {
          int page_num;
          print("uncorrectable error in CMD %u Block %u Page %u\n",CMD,block,page);
            for(page_num =0;page_num < GLOB_LLD_PAGES;page_num++)
            {
            if(GLOB_flash_memory[block*GLOB_LLD_PAGES+ page_num] == NULL)
            GLOB_flash_memory[block*GLOB_LLD_PAGES+ page_num] = (unsigned char
                    *)GLOB_MALLOC(GLOB_DeviceInfo.wPageSize*sizeof(unsigned char));
            }
            GLOB_PendingCMD[i].Status = CMD_FAIL;

            continue;
        }
#endif
#endif

#if EMU_BAD_BLOCK
        channel = (GLOB_PendingCMD[i].Block) / ((GLOB_DeviceInfo.wTotalBlocks) /  GLOB_totalUsedBanks);

        if (channels_with_bad_blocks[channel] == 1)
        {
            GLOB_PendingCMD[i].Status =   CMD_NOT_DONE;
            continue;
        }

        if ( ( FAIL == emu_bad_block_check(GLOB_PendingCMD[i].Block)) && (GLOB_PendingCMD[i].CMD != MEMCOPY_CMD))
        {
            channel = (GLOB_PendingCMD[i].Block) / ((GLOB_DeviceInfo.wTotalBlocks) /  GLOB_totalUsedBanks);

            channels_with_bad_blocks[channel] = 1;
#if     NO_CMDS_DONE_AFTER_ERROR
            if( hit_bad_block  == 1)
            {
                GLOB_PendingCMD[i].Status =   CMD_NOT_DONE;
            }
            else
            {
                GLOB_PendingCMD[i].Status = CMD_FAIL;
            }

#else
            GLOB_PendingCMD[i].Status = CMD_FAIL;

#endif
            hit_bad_block  = 1;

            continue;
        }

#if     NO_CMDS_DONE_AFTER_ERROR
        if(hit_sync_point)
        {
            GLOB_PendingCMD[i].Status =   CMD_NOT_DONE;
            continue;
        }

        if( (hit_bad_block == 1) )
        {
            hit_sync_point = 1;
            GLOB_PendingCMD[i].Status =   CMD_NOT_DONE;
            continue;
        }
#else
        if( (hit_bad_block == 1) && ( (GLOB_PendingCMD[i].CMD == ERASE_CMD)
                        || (GLOB_PendingCMD[i].CMD == WRITE_MAIN_SPARE_CMD)
                        || (GLOB_PendingCMD[i].CMD == WRITE_MAIN_CMD) ) )
        {
            channels_with_bad_blocks[channel] = 1;
            discarded_blocks[channel] = GLOB_PendingCMD[i].Block;
        }

#endif

#endif

        switch (CMD)
        {
        case ERASE_CMD:
            emu_Erase_Block(block);
            GLOB_PendingCMD[i].Status = CMD_PASS;
            break;
        case WRITE_MAIN_CMD:
            emu_Write_Page_Main(data, block, page, count);
            GLOB_PendingCMD[i].Status = CMD_PASS;
            break;
        case WRITE_MAIN_SPARE_CMD:
            emu_Write_Page_Main_Spare(data, block, page, count);
            GLOB_PendingCMD[i].Status = CMD_PASS;
            break;
        case READ_MAIN_SPARE_CMD:
            emu_Read_Page_Main_Spare(data, block, page, count);
            GLOB_PendingCMD[i].Status = CMD_PASS;
            break;
        case READ_MAIN_CMD:
            emu_Read_Page_Main(data, block, page, count);
            GLOB_PendingCMD[i].Status = CMD_PASS;
            break;
        case MEMCOPY_CMD:
            memcpy(GLOB_PendingCMD[i].DataDestAddr, GLOB_PendingCMD[i].DataSrcAddr, GLOB_PendingCMD[i].MemCopyByteCnt);
            GLOB_PendingCMD[i].Status = CMD_PASS;
            break;
        case DUMMY_CMD:
            GLOB_PendingCMD[i].Status = CMD_PASS;
            break;
        default:
            GLOB_PendingCMD[i].Status = CMD_FAIL;
            break;
        }
    }


    /*&&&&&&&&&&&&&&&&&&&&&&&&&&&*/
     // clear the rest of the pending CMD array
    /*&&&&&&&&&&&&&&&&&&&&&&&&&&&*/
    for (i=GLOB_cmd_tag_count+LLD_NUM_FLASH_CHANNELS; i<MAX_DESCRIPTORS; i++)
    {
        GLOB_PendingCMD[i].CMD           = 0;
        GLOB_PendingCMD[i].Tag           = 0;
        GLOB_PendingCMD[i].DataAddr      = 0;
        GLOB_PendingCMD[i].Block         = 0;
        GLOB_PendingCMD[i].Page          = 0;
        GLOB_PendingCMD[i].PageCount     = 0;
        GLOB_PendingCMD[i].DataDestAddr  = 0;
        GLOB_PendingCMD[i].DataSrcAddr   = 0;
        GLOB_PendingCMD[i].MemCopyByteCnt= 0;
        GLOB_PendingCMD[i].SBDCmdIndex   = 0;
        GLOB_PendingCMD[i].ChanSync[0]      = 0;
        GLOB_PendingCMD[i].ChanSync[1]      = 0;
        GLOB_PendingCMD[i].ChanSync[2]      = 0;
        GLOB_PendingCMD[i].ChanSync[3]      = 0;
        GLOB_PendingCMD[i].ChanSync[4]      = 0;
        GLOB_PendingCMD[i].Status        = CMD_NOT_DONE;
    }

    print(" At  end  of Execute CMDs \n\n");



    GLOB_SCHEDULE_FUNCTION(&GLOB_ISR);
    return status;
}



/*&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
* Function:     emu_Event_Status
* Inputs:       none
* Outputs:      Event_Status code
* Description:  This function can also be used to force errors
*&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&*/
uint16  emu_CDMA_Event_Status(void)
{
    uint16  event = EVENT_PASS;
    uint32 i = 0;


    for (i=LLD_NUM_FLASH_CHANNELS;  i< (GLOB_cmd_tag_count + LLD_NUM_FLASH_CHANNELS); i++)
    {
        if(GLOB_PendingCMD[i].Status == CMD_PASS)
            continue;
        else
        {
            if(GLOB_PendingCMD[i].CMD == ERASE_CMD)
                event = EVENT_ERASE_FAILURE;
            else if(GLOB_PendingCMD[i].CMD == WRITE_MAIN_CMD)
                event = EVENT_PROGRAM_FAILURE;
            else if(GLOB_PendingCMD[i].CMD == WRITE_MAIN_SPARE_CMD)
                event = EVENT_PROGRAM_FAILURE;
            else if(GLOB_PendingCMD[i].CMD == READ_MAIN_CMD)
                event = EVENT_UNCORRECTABLE_DATA_ERROR;
            else if(GLOB_PendingCMD[i].CMD == MEMCOPY_CMD)
                event = EVENT_MEMCOPY_FAILURE;
            else
                event = EVENT_FAIL;

            break;
        }
    }

    return event;
}
#endif // CMD_DMA

uint16   emu_Get_Bad_Block(BLOCKNODE block)
{
#if	EMU_BAD_BLOCK
    return (FAIL == emu_bad_block_check(block));
#else
    return 0;
#endif
}

#endif // !ELDORA
#endif // FLASH_EMU
