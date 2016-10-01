/*
 * NAND Flash Controller Device Driver
 * Copyright (c) 2010, Intel Corporation and its suppliers.
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
/***********************************************************************
*  flash_bbt.c
*  Boot partition block table management layer
*
**********************************************************************/


#include "flash_bbt.h"
#include "ffsdefs.h"
#include "lld.h"
#include <linux/crypto.h>
#include <linux/err.h>
#include <linux/scatterlist.h>
#include "lld_nand.h"
#include "NAND_Regs_4.h"

/******** Macros used in code *********/
#define BAD_BOOT_BLOCK               0x1
#define GOOD_BOOT_BLOCK               0x0
#define IS_BAD_BOOT_BLOCK(iBlockNum)       (BAD_BOOT_BLOCK == (GLOB_pBoot_BlockTable[iBlockNum] & BAD_BOOT_BLOCK))
#define MARK_BOOT_BLOCK_AS_BAD(blocknode)      ((blocknode) = BAD_BOOT_BLOCK)
#define MARK_BOOT_BLOCK_AS_GOOD(blocknode)      ((blocknode) = GOOD_BOOT_BLOCK)


#define BBT_START_BLOCK 0 //Block table locatation starts at block 0
#define BBT_COPIES 8 //Boot block table locates from block 0 to 7
#define BLOCKTABLE_INFO_TAG_OFFSET 32 //block table info tag structure start at offset 16byte
#define BLOCKTABLE_INFO_TAG_SIZE (sizeof(BOOT_BLOCKTABLE_INFO)) //16, block table info tag structure length is 16 bytes
#define BLOCK_TABLE_OFFSET_IN_PAGE0 (BLOCKTABLE_INFO_TAG_OFFSET + BLOCKTABLE_INFO_TAG_SIZE) //block table start at offset 48byte in page0
#define BBT_ENTRY_LENTH 8 // 8 bits to indicate a block status
#define BOOT_PARTITION_SIZE (GLOB_DeviceInfo.wSpectraStartBlock) //Boot partition size, this is also the length of BBT
#define MD5_HASH_LENGTH 16 //Length of MD5 hash is 16 bytes
#define ZONE1_START_BLOCK 16 //ZONE 1 start from block 16
#define DEFAULT_SPARE_SKIP_BYTE_ZONE0 0 //zone 0 spare skip byte, it's the area SEC FW touches
#define DEFAULT_SPARE_SKIP_BYTE_ZONE1 0 //Default spare skip byte in zone 1

#define MBH_SIZE 16

//#define BBT_DEBUG 1

#define ERR_BT_EXIST 2
#define MAX_BLOCK_TABLE_LENGTH (GLOB_DeviceInfo.wPageDataSize - BLOCKTABLE_INFO_TAG_OFFSET -BLOCKTABLE_INFO_TAG_SIZE - MD5_HASH_LENGTH)

#ifdef __cplusplus
extern "C"
{
#endif

/******** Global variables definition *********/
STATIC byte *pMempool = NULL;
STATIC byte *GLOB_pBoot_BlockTable = NULL;
STATIC uint16 GLOB_blocktble_existing = 0;
STATIC byte *GLOB_pPageBuffer = NULL;

STATIC byte GLOB_MBHarray[MBH_SIZE];

STATIC BOOT_BLOCKTABLE_INFO GLOB_blocktable_info_tag;

STATIC byte GLOB_spare_skip_byte_zone1 = DEFAULT_SPARE_SKIP_BYTE_ZONE1;


#define IS_IN_BOOT_PARTITION(blocknum) do{ \
                                             if (blocknum >= GLOB_DeviceInfo.wSpectraStartBlock) \
                                             { \
                                               return FAIL; \
                                             } }while(0)
/******** Functions for debug usage*********/
STATIC void hexdump(byte *pInput, int length)
{
	int size = length;
	byte *tmp =pInput;
	while(size--)
		printk("%02x",*tmp++);
	printk("\n");
}
STATIC void BBT_dump(void)
{
	int i =0;
	for (i=0;i<BOOT_PARTITION_SIZE;i++)
		printk("blk %d is %s\n",i,IS_BAD_BOOT_BLOCK(i)?"BAD":"GOOD");
}
STATIC void bt_info_tag_dump(BOOT_BLOCKTABLE_INFO *pInput)
{
	BOOT_BLOCKTABLE_INFO *tmp =pInput;
	printk("bt info tag value\n");
	printk("signatue %c %c %c %c\n",tmp->bt_sig[0],tmp->bt_sig[1],tmp->bt_sig[2],tmp->bt_sig[3]);
	printk("block table %d %d %d\n",tmp->btOffsetInBytes,tmp->btEntrySizeInBits,tmp->btEntryNum);
	printk("md5 %d %d \n",tmp->md5OffsetInBytes,tmp->md5SizeInBytes);
	printk("reserve %02x %02x %02x \n",tmp->nSpareSkipByteInZone1,tmp->nSpareSkipByteInZone0,tmp->reserve3);
	printk("\n");
}
STATIC void page0_dump(byte *pInput)
{
	byte *tmp =pInput;
	BOOT_BLOCKTABLE_INFO *pblocktable_info_tag = &GLOB_blocktable_info_tag;
	print("\n-----------------------page 0 dump----------------\n");
	hexdump(tmp+32,82);
	bt_info_tag_dump((BOOT_BLOCKTABLE_INFO *) (tmp + 32));
	BBT_dump();
	hexdump(tmp+pblocktable_info_tag->md5OffsetInBytes,16);
	print("\n");
}
/***********************************************************************
*   BBT_Generate_MD5_HASH
*   Calculate MD5 hash through random input data
**********************************************************************/
STATIC uint16 BBT_Generate_MD5_HASH(byte *pInput, uint16 SizeOfInput, byte* pOutput)
{
	uint16 SizeOfOutput = MD5_HASH_LENGTH;
	struct scatterlist sg[1];
    byte* pOut = pOutput;
	byte* pIn = pInput;
    struct crypto_hash *tfm;
    struct hash_desc desc;

	if ((pIn == NULL)|| (pOut == NULL))
		return FAIL;
	memset(pOut, 0, MD5_HASH_LENGTH);


    tfm = crypto_alloc_hash("md5", 0, CRYPTO_ALG_ASYNC);
    if (IS_ERR(tfm))
            return FAIL;

	sg_set_buf(&sg[0], pIn, SizeOfInput);

    desc.tfm = tfm;
    desc.flags = 0;

     if (crypto_hash_digest(&desc, sg, SizeOfInput, pOut))
            return FAIL;

     crypto_free_hash(tfm);
	return SizeOfOutput;
}
/***********************************************************************
*  BBT_Verify_MD5
*  Verify MD5 hash when read block table
**********************************************************************/
STATIC uint32 BBT_Verify_MD5(byte* pDataSrc, uint16 dataSize, byte *pmd5Src, uint16 md5Size)
{
	byte *pSrc = pDataSrc;
	byte *pDst = pmd5Src;
	char md5Result[MD5_HASH_LENGTH];
	uint16 md5DigestSize;

	if ((pSrc == NULL)|| (pDst == NULL))
		return FAIL;

	md5DigestSize = BBT_Generate_MD5_HASH(pSrc,dataSize,md5Result);
	if (MD5_HASH_LENGTH != md5DigestSize)
	{
		print(KERN_ERR "Fail to calculate MD5 HASH\n");
		return FAIL;
	}
	if (memcmp(md5Result,pDst,MD5_HASH_LENGTH))
	{	print("%s MD5 verify fail\n",__FUNCTION__);
		return FAIL;
	}
	print("%s MD5 verify success\n",__FUNCTION__);
	return PASS;
}
/***********************************************************************
*  BBT_Verify_MBH
*  Verify MBH value when read page 0 content
**********************************************************************/
STATIC uint16 BBT_Verify_MBH(byte* pDataSrc)
{
	byte *pbuf = pDataSrc;
	if ((pbuf[0] == 0xb)&&(pbuf[1] == 0xb0)&&(pbuf[2] == 0xad)&&(pbuf[3] == 0xde))// MBH header check
		return PASS;
	return FAIL;
}
/***********************************************************************
*  BBT_NAND_Read_Page_Main --- NAND read funtion
* spare_skip_bytes value is indicated in bt_info_tag structure
*  It sets the spare_skip_bytes first before NAND read
**********************************************************************/
STATIC uint16 BBT_NAND_Read_Page_Main(byte* read_data,BLOCKNODE block,PAGENUMTYPE page,PAGENUMTYPE page_count)
{
	uint16 status = FAIL;
	uint16 tmp = FlashReg[SPARE_AREA_SKIP_BYTES];
	FlashReg[SPARE_AREA_SKIP_BYTES] = ((block >= ZONE1_START_BLOCK)?GLOB_spare_skip_byte_zone1:GLOB_blocktable_info_tag.nSpareSkipByteInZone0);
	status= GLOB_LLD_Read_Page_Main(read_data,block,page,page_count);
	FlashReg[SPARE_AREA_SKIP_BYTES] = tmp;
	return status;
}
/***********************************************************************
*  BBT_NAND_Write_Page_Main -- NAND write function
* spare_skip_bytes value is indicated in bt_info_tag structure
*  It sets the spare_skip_bytes first before NAND write
**********************************************************************/
STATIC uint16 BBT_NAND_Write_Page_Main(byte * read_data, BLOCKNODE block, PAGENUMTYPE page, PAGENUMTYPE page_count)
{
	uint16 status = FAIL;
	uint16 tmp = FlashReg[SPARE_AREA_SKIP_BYTES];
	FlashReg[SPARE_AREA_SKIP_BYTES] = ((block >= ZONE1_START_BLOCK)?GLOB_spare_skip_byte_zone1:GLOB_blocktable_info_tag.nSpareSkipByteInZone0);
	status= GLOB_LLD_Write_Page_Main(read_data,block,page,page_count);
	FlashReg[SPARE_AREA_SKIP_BYTES] = tmp;
	return status;
}
/***********************************************************************
*  BBT_Erase_Boot_Block_withRetry
*  This function will try to erase a block for several times when erase failure occurs
**********************************************************************/
STATIC uint32 BBT_Erase_Boot_Block_withRetry(BLOCKNODE blocknum)
{
	uint32 status = FAIL;
	int i;
	for(i = 0; i < RETRY_TIMES; i++)
    {
            if( PASS == GLOB_LLD_Erase_Block(blocknum))
            {
		status = PASS;
                break;
            }
			else
				status = FAIL;
     }
	return status;
}
/***********************************************************************
*  BBT_Read_Page0_In_Copies
*  Get page 0 from several copies
*  It returns PASS once page 0 read success
**********************************************************************/
STATIC uint32 BBT_Read_Page0_In_Copies(byte* dma_buffer,BLOCKNODE blocknum, uint16 copies)
{
    uint32 status = FAIL;
	byte* pbuf = dma_buffer;
	BLOCKNODE i;
			print("%s\n",__FUNCTION__);
	if (NULL == pbuf)
		return FAIL;

	for (i = blocknum; i < (blocknum + copies);i++)
	{
		if (PASS == GLOB_BBT_NAND_Read_Page_Main(pbuf,i,0,1))
		{
			status = PASS;
			break;
		}
		else
			status = FAIL;
	}
	return status;
}
STATIC uint32 BBT_Read_MBH(byte *pagebuf)
{
	uint32 status = FAIL;
	byte *pbuf = pagebuf;

	if (PASS == BBT_Verify_MBH(GLOB_MBHarray))
	{
		memcpy(pbuf,GLOB_MBHarray,MBH_SIZE);
		return PASS;
	}
	status  = BBT_Read_Page0_In_Copies(pbuf,BBT_START_BLOCK,BBT_COPIES);
	if (FAIL == status)
	{
		print(KERN_ERR "Spectra: Faile to read page 0 in block 0 to 7\n");
		return FAIL;
	}
	status = BBT_Verify_MBH(pbuf);
	if (FAIL == status)
	{
		print(KERN_ERR "Spectra: Faile to get correct MBH in block 0 to 7\n");
		return FAIL;
	}
	memcpy(GLOB_MBHarray,pbuf,MBH_SIZE);
	return PASS;
}
/***********************************************************************
*  BBT_Write_Block_Table
* Steps:
* 	Read MBH
*	write block table info tag strucuture
*	Calculate MD5 hash value
*	Write new page 0 to block 0 - 7
* When new bad blocks found in blocks 0 - 7, it will update the block table and page 0
* first and restart from block 0
**********************************************************************/
STATIC uint32 BBT_Write_Block_Table(void)
{
	uint32 status = FAIL,i;
	uint16 md5DigestSize=0;
	uint16 bad_block_num=0;
	uint16 bt_size = 0;
	byte *pBTNode = GLOB_pBoot_BlockTable;
	BOOT_BLOCKTABLE_INFO *pblocktable_info_tag = &GLOB_blocktable_info_tag;

	uint16 md5InputSize = 0;
	byte *md5InputAddr = NULL;
	byte *md5OutputAddr = NULL;
	print("%s\n",__FUNCTION__);

	memset((void*)GLOB_pPageBuffer,0,GLOB_DeviceInfo.wPageDataSize);

	//Fill in MBH
	if (FAIL == BBT_Read_MBH(GLOB_pPageBuffer))
	{
		print(KERN_ERR "No valid MBH exist in the NAND, stop to write block table\n");
		return FAIL;
	}
	print(KERN_INFO "MBH verify success\n");

	//Fill in block table info tag
	memset(pblocktable_info_tag,0,sizeof(BOOT_BLOCKTABLE_INFO));
	pblocktable_info_tag->bt_sig[0] = 'G';
	pblocktable_info_tag->bt_sig[1] = 'O';
	pblocktable_info_tag->bt_sig[2] = 'O';
	pblocktable_info_tag->bt_sig[3] = 'D';

	pblocktable_info_tag->nSpareSkipByteInZone1 = GLOB_spare_skip_byte_zone1;
	pblocktable_info_tag->nSpareSkipByteInZone0 =	DEFAULT_SPARE_SKIP_BYTE_ZONE0;

	pblocktable_info_tag->btEntrySizeInBits = BBT_ENTRY_LENTH;
	pblocktable_info_tag->btEntryNum = (BOOT_PARTITION_SIZE < MAX_BLOCK_TABLE_LENGTH)?BOOT_PARTITION_SIZE:MAX_BLOCK_TABLE_LENGTH;
	pblocktable_info_tag->btOffsetInBytes = BLOCK_TABLE_OFFSET_IN_PAGE0;
	pblocktable_info_tag->md5SizeInBytes = MD5_HASH_LENGTH;
	bt_size = pblocktable_info_tag->btEntryNum;//*pblocktable_info_tag->btEntrySizeInBits)>>3; We fixed the block table entry size to be 8 bit
	pblocktable_info_tag->md5OffsetInBytes = pblocktable_info_tag->btOffsetInBytes + bt_size;

	//Copy tag & block table data to page 0 buffer
	memcpy((void *)(GLOB_pPageBuffer + BLOCKTABLE_INFO_TAG_OFFSET),(void *)pblocktable_info_tag,sizeof(BOOT_BLOCKTABLE_INFO));
	memcpy((void *)(GLOB_pPageBuffer + pblocktable_info_tag->btOffsetInBytes),(void *)pBTNode,pblocktable_info_tag->btEntryNum);


	//Generate MD5 hash
	md5InputSize = bt_size + sizeof(BOOT_BLOCKTABLE_INFO);
	md5InputAddr = (byte *)(GLOB_pPageBuffer + BLOCKTABLE_INFO_TAG_OFFSET);
	md5OutputAddr = (byte *)(GLOB_pPageBuffer + pblocktable_info_tag->md5OffsetInBytes);
Reburn_BBT:
	md5DigestSize = BBT_Generate_MD5_HASH((byte *)md5InputAddr,md5InputSize,(byte *)md5OutputAddr);
	if (MD5_HASH_LENGTH != md5DigestSize)
	{
		print(KERN_ERR "Spectra: Fail to calculate MD5 HASH\n");
		return FAIL;
	}

	bad_block_num = 0;
	for (i = BBT_START_BLOCK; i < (BBT_START_BLOCK + BBT_COPIES);i++)
	{
		if (IS_BAD_BOOT_BLOCK(i))
		{
			bad_block_num ++;
			continue;
		}
		 if ((FAIL == BBT_Erase_Boot_Block_withRetry(i))||(FAIL == BBT_NAND_Write_Page_Main(GLOB_pPageBuffer,i,0,1)))
			{
				MARK_BOOT_BLOCK_AS_BAD(pBTNode[i]);
				MARK_BOOT_BLOCK_AS_BAD(GLOB_pPageBuffer[pblocktable_info_tag->btOffsetInBytes + i]);
				goto Reburn_BBT;
			}
		 else
			 print("Write page 0 success in block %d\n",i);
	}
	if (BBT_COPIES <= bad_block_num)
		{
			print(KERN_ERR "This should never happen, all redundant blocks are bad\n");
			status = FAIL;
		}
	else
	{
		GLOB_blocktble_existing = 1;
		status = PASS;
	}
	return status;
}
/***********************************************************************
*  Update_BBT_with_NewBadBlock
* Update block table when new bad block discovered
**********************************************************************/
STATIC uint32 Update_BBT_with_NewBadBlock(BLOCKNODE blocknum)
{
	byte *pBTNode = GLOB_pBoot_BlockTable;
	print("%s\n",__FUNCTION__);
	IS_IN_BOOT_PARTITION(blocknum);
	MARK_BOOT_BLOCK_AS_BAD(pBTNode[blocknum]);
	return BBT_Write_Block_Table();
}
/***********************************************************************
*  BBT_Read_Block_Table_IN_Block -- Read a valid block table in a block
* Steps:
* 	Read page 0
*	Verify MBH
*	Verify signature
*	Verify MD5 hash
**********************************************************************/
STATIC uint32 BBT_Read_Block_Table_IN_Block(BLOCKNODE blocknum)
{
	uint32 status = FAIL;
	uint16 btSize = 0;
	BOOT_BLOCKTABLE_INFO *pblocktable_info_tag = &GLOB_blocktable_info_tag;

		print("%s block %d\n",__FUNCTION__,blocknum);
	memset((void *)pblocktable_info_tag,0,sizeof(BOOT_BLOCKTABLE_INFO));
	memset((void*)GLOB_MBHarray,0,MBH_SIZE);
	memset((void*)GLOB_pPageBuffer,0,GLOB_DeviceInfo.wPageDataSize);



	//Read page 0
	status  = BBT_NAND_Read_Page_Main(GLOB_pPageBuffer,blocknum,0,1);
	if (FAIL == status)
	{
		print(KERN_ERR "Spectra: Read page 0 in block %d failed\n",(uint32)blocknum);
		return FAIL;
	}

	memcpy(GLOB_MBHarray,GLOB_pPageBuffer,MBH_SIZE);
	memcpy((void*)pblocktable_info_tag,(void*)(GLOB_pPageBuffer + BLOCKTABLE_INFO_TAG_OFFSET),sizeof(BOOT_BLOCKTABLE_INFO));

	btSize = pblocktable_info_tag->btEntryNum;//* pblocktable_info_tag->btEntrySizeInBits >>3; We fixed the blocktable entry size to be 8
	if (btSize > MAX_BLOCK_TABLE_LENGTH)
	{
		print(KERN_ERR "Spectra: Boot partition is too large %d \n",btSize);
		return FAIL;
	}
	memcpy(GLOB_pBoot_BlockTable, (byte *)(GLOB_pPageBuffer + pblocktable_info_tag->btOffsetInBytes), btSize);


	//Verify MBH
	if (FAIL == BBT_Verify_MBH(GLOB_MBHarray))
		return FAIL;
	print("MBH verify success\n");

	//Verify block table signature
	if (('G' != pblocktable_info_tag->bt_sig[0])||('O' != pblocktable_info_tag->bt_sig[1])||('O' != pblocktable_info_tag->bt_sig[2])||('D' != pblocktable_info_tag->bt_sig[3]))
	{
		print(KERN_ERR "Boot block table signature verification for page 0 in block %d failed\n",blocknum);
	return FAIL;
	}
	//Verify MD5 hash
	status = BBT_Verify_MD5((byte*)(GLOB_pPageBuffer + BLOCKTABLE_INFO_TAG_OFFSET), btSize + sizeof(BOOT_BLOCKTABLE_INFO), (byte*)(GLOB_pPageBuffer + pblocktable_info_tag->md5OffsetInBytes), pblocktable_info_tag->md5SizeInBytes);
	if (FAIL == status)
	{
		print(KERN_ERR "MD5 verification for page 0 in block %d failed\n",(uint32)blocknum);
		return FAIL;
	}
	GLOB_spare_skip_byte_zone1 = pblocktable_info_tag->nSpareSkipByteInZone1;
	GLOB_blocktble_existing = 1;
	return PASS;
}
/***********************************************************************
*  Read block table from block 0 to 7
**********************************************************************/
STATIC uint32 BBT_Read_Block_Table(void)
{
    uint32 status = FAIL;
	BLOCKNODE i;
	for (i = BBT_START_BLOCK; i < (BBT_START_BLOCK + BBT_COPIES);i++)
	{
		if (PASS == BBT_Read_Block_Table_IN_Block(i))
		{
			status = PASS;
			break;
		}
		else
			status = FAIL;
	}
	return status;
}
STATIC void BBT_Mark_ALL_Blocks_Good(void)
{
	byte *pBTNode = GLOB_pBoot_BlockTable;
	int i;
	print("%s\n",__FUNCTION__);
	for(i=0; i < BOOT_PARTITION_SIZE; i++)
		MARK_BOOT_BLOCK_AS_GOOD(pBTNode[i]);
}
/***********************************************************************
*  GLOB_BBT_IO_Read_Block_Table
* This function is used to transmit block table content to user space
**********************************************************************/
uint16 GLOB_BBT_IO_Read_Block_Table(BOOT_BLOCKTABLE_IO_CMD* pbtInfo)
{
	BOOT_BLOCKTABLE_IO_CMD* ptmp = pbtInfo;
	if ((NULL == ptmp)||(0 == ptmp->AddrInRam))
		return FAIL;
	print("%s",__FUNCTION__);
	ptmp->btEntryNum = BOOT_PARTITION_SIZE;
	ptmp->btEntrySizeInBits = BBT_ENTRY_LENTH;
	if (PASS != copy_to_user((byte* )ptmp->AddrInRam,GLOB_pBoot_BlockTable,ptmp->btEntryNum))
		return FAIL;
	return PASS;
}
/***********************************************************************
*  GLOB_BBT_Erase_Block_by_Marker
*  This function will lookup initial bad block marker before erase operation proceed
**********************************************************************/
uint16 GLOB_BBT_Erase_Block_by_Marker(BLOCKNODE blocknum)//Erase the NAND block by look up intial bad block marker
{
	print("%s",__FUNCTION__);
	IS_IN_BOOT_PARTITION(blocknum);
	if (GOOD_BLOCK != GLOB_LLD_Get_Bad_Block_Raw(blocknum))
		return FAIL;
    if( PASS != GLOB_LLD_Erase_Block(blocknum))
	return FAIL;
	return PASS;
}
/***********************************************************************
*  GLOB_BBT_Erase_Block_by_BBT
*  This function will lookup block table for NAND erase
**********************************************************************/
uint16 GLOB_BBT_Erase_Block_by_BBT(BLOCKNODE blocknum)//Erase NAND block by boot block table lookup
{
	IS_IN_BOOT_PARTITION(blocknum);
	if (IS_BAD_BOOT_BLOCK(blocknum))
		return FAIL;

	if (FAIL == BBT_Erase_Boot_Block_withRetry(blocknum))
		{
			print("Erase failure on block %d, mark it bad and update the block table \n",blocknum);
			Update_BBT_with_NewBadBlock(blocknum);
			return FAIL;
	}
	else
		return PASS;
}
/***********************************************************************
*  GLOB_BBT_Create_Block_Table_Dummy
* It creates a dummy block table to show all blocks good
**********************************************************************/
uint16 GLOB_BBT_Create_Block_Table_Dummy(uint16 wforce)
{
	print("%s",__FUNCTION__);
	if ((!wforce) && GLOB_blocktble_existing)
	{
		print("No need to create block table\n");
		return ERR_BT_EXIST;//Block table exist
	}
	else
	{
		BBT_Mark_ALL_Blocks_Good();
		if (FAIL == BBT_Write_Block_Table())
		{
				print("Write block table failed\n");
				return FAIL;//Block table write failure
		}
	}
	return PASS;
}
/***********************************************************************
*  GLOB_BBT_Create_Block_Table_byErase
*  It creates block table by erasing entire NAND
*  If erase error happens, then mark it as bad and update block table
**********************************************************************/
uint16 GLOB_BBT_Create_Block_Table_byErase(void)
{
	byte *pBTNode = GLOB_pBoot_BlockTable;
	int i;
	int bad_block_num =0;
	print("%s",__FUNCTION__);
	if (FAIL == BBT_Read_MBH(GLOB_pPageBuffer))
	{
		printk(KERN_ERR "No valid MBH exist in the NAND, stop to create block table!!\n");
		return FAIL;
	}
	print(KERN_INFO "MBH verify success\n");

	for(i=0; i < BOOT_PARTITION_SIZE; i++)
    {
            if (FAIL == BBT_Erase_Boot_Block_withRetry(i))
			{
				MARK_BOOT_BLOCK_AS_BAD(pBTNode[i]);
				bad_block_num++;
			}
            else
                MARK_BOOT_BLOCK_AS_GOOD(pBTNode[i]);

    }
	if (BOOT_PARTITION_SIZE <= bad_block_num)
	{
			print("All boot partition blocks are bad, this should never happen\n");
			return FAIL;
	}
	if (FAIL == BBT_Write_Block_Table())
	{
			print("Write block table failed\n");
			return FAIL;
	}
	return PASS;
}
/***********************************************************************
*  GLOB_BBT_Create_Block_Table_byInitalMarker
* It creates block table by initial bad block marker
**********************************************************************/

uint16 GLOB_BBT_Create_Block_Table_byInitalMarker(void)
{
	byte *pBTNode = GLOB_pBoot_BlockTable;
	int i;
	int bad_block_num= 0;

	print("%s",__FUNCTION__);
	for(i=0; i < BOOT_PARTITION_SIZE; i++)
    {
            if (GOOD_BLOCK != GLOB_LLD_Get_Bad_Block_Raw(i))
			{
				MARK_BOOT_BLOCK_AS_BAD(pBTNode[i]);
				bad_block_num++;
			}
            else
                MARK_BOOT_BLOCK_AS_GOOD(pBTNode[i]);
    }
	if (BOOT_PARTITION_SIZE <= bad_block_num)
		{
			print("All boot partition blocks are bad, this should never happen\n");
			return FAIL;
	}

	#if 0
	GLOB_spare_skip_byte_zone1 = 2;
	#endif

	if (FAIL == BBT_Write_Block_Table())
	{
			print("Write block table failed\n");
			return FAIL;
	}
	return PASS;

}
/***********************************************************************
*  GLOB_BBT_NAND_Read_Page_Main
*  NAND read function
**********************************************************************/
uint16 GLOB_BBT_NAND_Read_Page_Main(byte* read_data,BLOCKNODE block,PAGENUMTYPE page,PAGENUMTYPE page_count)
{
	uint16 status = FAIL;
	print("%s blk %d, page%d",__FUNCTION__,block, page);
	IS_IN_BOOT_PARTITION(block);
	if (IS_BAD_BOOT_BLOCK(block))
		return ERR_BT_EXIST;
	status = BBT_NAND_Read_Page_Main(read_data,block,page,page_count);
	return status;
}
/***********************************************************************
*  GLOB_BBT_NAND_Write_Page_Main
*  NAND write function
*  It will update block table when write failure occurs
**********************************************************************/
uint16 GLOB_BBT_NAND_Write_Page_Main(byte* read_data,BLOCKNODE block,PAGENUMTYPE page,PAGENUMTYPE page_count)
{
	uint16 status = FAIL;
	print("%s",__FUNCTION__);

	IS_IN_BOOT_PARTITION(block);
	if (IS_BAD_BOOT_BLOCK(block))
		return FAIL;
#ifndef BBT_DEBUG
		print("NAND Write page main\n");
#endif
	status = BBT_NAND_Write_Page_Main(read_data,block,page,page_count);
	if (PASS != status)
	{
		print("Write failure on block %d, mark it bad and update the block table \n",block);
		Update_BBT_with_NewBadBlock(block);
	}
	return status;
}
uint32 GLOB_BBT_Init (void)
{
    uint32 status = FAIL;
	if(!(pMempool = (byte *)kzalloc(BOOT_PARTITION_SIZE, GFP_KERNEL)))
	{
        printk(KERN_ERR "Spectra: Unable to allocate block buffer for boot block table. Aborting\n");
            return -ENOMEM;
    }
	GLOB_pBoot_BlockTable = pMempool;
	if(!(GLOB_pPageBuffer = (byte *)kzalloc(GLOB_DeviceInfo.wPageDataSize, GFP_DMA)))
	{
        printk(KERN_ERR "Spectra: Unable to allocate block buffer for boot block table. Aborting\n");
            goto ERROR1;
    }
	status = BBT_Read_Block_Table();
	if  (FAIL == status)
	{
		print("No block table found in block 0 - 7, create a dummy one automatically\n");
		BBT_Mark_ALL_Blocks_Good();
		status = PASS;
	}
	print("Boot partition block table init success\n");
	return PASS;
ERROR1:
	kfree(pMempool);
	return -ENOMEM;

}

void GLOB_BBT_Release (void)
{
    kfree(pMempool);
	kfree(GLOB_pPageBuffer);
}

#ifdef __cplusplus
}
#endif
