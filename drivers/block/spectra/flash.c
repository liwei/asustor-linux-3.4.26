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



#include "flash.h"
#include "ffsdefs.h"
#include "lld.h"
#if CMD_DMA
#include "lld_cdma.h"
#endif



#define BLK_FROM_ADDR(addr)  ((BLOCKNODE) (addr >> GLOB_DeviceInfo.nBitsInBlockDataSize))
#define PAGE_FROM_ADDR(addr, Block)  ((PAGESIZETYPE )((addr - (ADDRESSTYPE)Block * GLOB_DeviceInfo.wBlockDataSize) >>  GLOB_DeviceInfo.nBitsInPageDataSize))

#define IS_SPARE_BLOCK(iBlockNum)     (BAD_BLOCK != (pBlockNode[iBlockNum] & BAD_BLOCK) && SPARE_BLOCK == (pBlockNode[iBlockNum] & SPARE_BLOCK))
#define IS_DATA_BLOCK(iBlockNum)      (0 == (pBlockNode[iBlockNum] & BAD_BLOCK))
#define IS_DISCARDED_BLOCK(iBlockNum) (BAD_BLOCK != (pBlockNode[iBlockNum] & BAD_BLOCK) && DISCARD_BLOCK == (pBlockNode[iBlockNum] & DISCARD_BLOCK))
#define IS_BAD_BLOCK(iBlockNum)       (BAD_BLOCK == (pBlockNode[iBlockNum] & BAD_BLOCK))

#define NUM_MEMPOOL_ALLOCS            (22+CACHE_BLOCK_NUMBER)

#ifdef __cplusplus
extern "C"
{
#endif

#if DEBUG_BNDRY
void debug_boundary_lineno_error(int chnl, int limit, int no, int lineno, char *filename) {
    if (chnl >= limit) {
#ifdef LINUX_DEVICE_DRIVER
        printk(KERN_ERR "Boundary Check Fail value %d >= limit %d, at  %s:%d. Other info:%d. Aborting...\n", chnl, limit, filename, lineno, no);
#else
        printf("Boundary Check Fail value %d >= limit %d, at  %s:%d. Other info:%d. Aborting...\n", chnl, limit, filename, lineno, no);
#endif
#if SEAMLESS
    exit_sim();
#elif !defined LINUX_DEVICE_DRIVER
    exit(1);
#endif
    }
}
#endif

STATIC byte FTL_Cache_If_Hit (ADDRESSTYPE dwPageAddr);
STATIC int  FTL_Cache_Read (ADDRESSTYPE dwPageAddr);
STATIC void FTL_Cache_Read_Page (byte * pData, ADDRESSTYPE dwPageAddr,
   byte bCacheBlockNum);
STATIC void FTL_Cache_Write_Page (byte * pData, ADDRESSTYPE dwPageAddr,
   byte bCacheBlockNum, uint16 Flags);
STATIC int  FTL_Cache_Write (void);
STATIC int  FTL_Cache_Write_Back (byte * pData, ADDRESSTYPE dwBlockAddr);
STATIC void FTL_Calculate_LFU (void);
STATIC BLOCKNODE  FTL_Get_Block_Index(BLOCKNODE wBlockNum);


STATIC int  FTL_Search_Block_Table_IN_Block(BLOCKNODE BT_Block, byte BT_Tag, PAGENUMTYPE *Page);
STATIC int  FTL_Read_Block_Table (void);
STATIC int  FTL_Write_Block_Table (int wForce);
STATIC int  FTL_Write_Block_Table_Data(void);
STATIC int  FTL_Check_Block_Table (int wOldTable);
STATIC int  FTL_Static_Wear_Leveling (void);
STATIC BLOCKNODE  FTL_Replace_Block_Table (void);
STATIC int  FTL_Write_IN_Progress_Block_Table_Page(void);


STATIC uint32 FTL_Get_Page_Num (ADDRESSTYPE length);
STATIC ADDRESSTYPE FTL_Get_Physical_Block_Addr (ADDRESSTYPE dwBlockAddr);

STATIC BLOCKNODE  FTL_Replace_OneBlock(BLOCKNODE wBlockNum,
   BLOCKNODE wReplaceNum);
STATIC BLOCKNODE  FTL_Replace_LWBlock(BLOCKNODE wBlockNum,
   int * pGarbageCollect);
STATIC BLOCKNODE  FTL_Replace_MWBlock(void);
STATIC int  FTL_Replace_Block (ADDRESSTYPE dwBlockAddr);
STATIC int  FTL_Adjust_Relative_Erase_Count(BLOCKNODE);

STATIC int  FTL_Flash_Error_Handle (byte * pData, ADDRESSTYPE dwOldPageAddr,
   ADDRESSTYPE dwBlockAddr);

DEVICE_INFO GLOB_DeviceInfo;
DEVICE_INFO RAW_DeviceInfo; //Device info for raw access


STATIC byte * g_pTempBuf;

byte *GLOB_g_pBlockTable;

byte *GLOB_g_pWearCounter;

uint16 *GLOB_g_pReadCounter;

STATIC PAGENUMTYPE g_wBlockTableOffset;

STATIC BLOCKNODE   g_wBlockTableIndex;

STATIC byte   g_cBlockTableStatus;

BLOCKNODE *GLOB_g_pBTBlocks;

FLASH_CACHE GLOB_Cache;

#if (RESTORE_CACHE_ON_CDMA_CHAIN_FAILURE && CMD_DMA)
FLASH_CACHE_DELTA_LIST GLOB_Int_Cache[LLD_NUM_FLASH_CHANNELS + MAX_DESCRIPTORS];
FLASH_CACHE GLOB_Cache_StartingCopy;
#endif


int  GLOB_g_wNumFreeBlocks;

#if CMD_DMA
   byte g_SBDCmdIndex = 0;
#endif
STATIC byte * g_pIPF;

STATIC byte bt_flag = FIRST_BT_ID;

STATIC byte bt_block_changed = 0;

#if _DEBUG_
int dbgGCCalled;
#endif

#if READBACK_VERIFY
STATIC byte * g_pCheckBuf;
#endif

STATIC byte cache_block_to_write;
STATIC byte  last_erased = FIRST_BT_ID;

STATIC byte *g_pMemPool;
STATIC byte *g_pMemPoolFree;
STATIC byte *g_temp_buf;

STATIC int globalMemSize;

STATIC byte GC_Called_Flag =0;
STATIC byte BT_GC_Called_Flag =0;

#if CMD_DMA
byte GLOB_FTLCommandCount = 0;
byte *GLOB_g_pBTDelta;
byte *GLOB_g_pBTDelta_Free;
byte *GLOB_g_pBTStartingCopy;
byte *GLOB_g_pWearCounterCopy;
uint16* GLOB_g_pReadCounterCopy;

byte *GLOB_g_pBlockTableCopies;
byte *GLOB_g_pNextBlockTable;
byte *GLOB_g_pCopyBackBufferCopies;
byte *GLOB_g_pCopyBackBufferStart;

extern PENDING_CMD     GLOB_PendingCMD[MAX_DESCRIPTORS + LLD_NUM_FLASH_CHANNELS];


#pragma pack(push, 1)
#pragma pack(1)
struct BTableChangesDelta
{
    byte         GLOB_FTLCommandCount;
    byte         ValidFields;
    PAGENUMTYPE  g_wBlockTableOffset;
    BLOCKNODE    g_wBlockTableIndex;
    BLOCKNODE    BT_Index;
    BLOCKNODE    BT_Entry_Value;
    BLOCKNODE    WC_Index;
    byte         WC_Entry_Value;
    BLOCKNODE    RC_Index;
    uint16       RC_Entry_Value;
};

#pragma pack(pop)

struct BTableChangesDelta * GLOB_p_BTableChangesDelta;
#endif



#define MARK_BLOCK_AS_BAD(blocknode)      (blocknode |= BAD_BLOCK)
#define MARK_BLOCK_AS_DISCARD(blocknode)  (blocknode = (blocknode & ~SPARE_BLOCK) | DISCARD_BLOCK)


#define FTL_Get_LBAPBA_Table_Mem_Size_Bytes() (GLOB_DeviceInfo.wDataBlockNum * sizeof(BLOCKNODE))
#define FTL_Get_WearCounter_Table_Mem_Size_Bytes() (GLOB_DeviceInfo.wDataBlockNum * sizeof(byte))
#define FTL_Get_ReadCounter_Table_Mem_Size_Bytes() (GLOB_DeviceInfo.wDataBlockNum * sizeof(uint16))
#if SUPPORT_LARGE_BLOCKNUM
#define FTL_Get_LBAPBA_Table_Flash_Size_Bytes() (GLOB_DeviceInfo.wDataBlockNum * sizeof(byte) * 3)
#else
#define FTL_Get_LBAPBA_Table_Flash_Size_Bytes() (GLOB_DeviceInfo.wDataBlockNum * sizeof(BLOCKNODE))
#endif
#define FTL_Get_WearCounter_Table_Flash_Size_Bytes  FTL_Get_WearCounter_Table_Mem_Size_Bytes
#define FTL_Get_ReadCounter_Table_Flash_Size_Bytes  FTL_Get_ReadCounter_Table_Mem_Size_Bytes

STATIC uint32 FTL_Get_Block_Table_Flash_Size_Bytes(void)
{
    uint32 bytes;
    if(GLOB_DeviceInfo.MLCDevice)
    {
        bytes = FTL_Get_LBAPBA_Table_Flash_Size_Bytes()+
            GLOB_DeviceInfo.wDataBlockNum * sizeof(byte)+
            GLOB_DeviceInfo.wDataBlockNum * sizeof(uint16);
    }
    else {
        bytes = FTL_Get_LBAPBA_Table_Flash_Size_Bytes()+
            GLOB_DeviceInfo.wDataBlockNum * sizeof(byte);
    }
    bytes += (4 * sizeof(byte));
    return bytes;
}

STATIC PAGENUMTYPE  FTL_Get_Block_Table_Flash_Size_Pages(void)
{
    return (PAGENUMTYPE)FTL_Get_Page_Num(FTL_Get_Block_Table_Flash_Size_Bytes());
}

STATIC int FTL_Copy_Block_Table_To_Flash(byte *flashBuf, uint32 sizeToTx, uint32 sizeTxed)
{
    uint32 wBytesCopied, wBlockTableSize, wBytes;
    BLOCKNODE *pBlockNode = (BLOCKNODE *)GLOB_g_pBlockTable;


    wBlockTableSize = FTL_Get_LBAPBA_Table_Flash_Size_Bytes();
    for(wBytes=0; (wBytes < sizeToTx) && ((wBytes+sizeTxed) < wBlockTableSize); wBytes++) {
#if SUPPORT_LARGE_BLOCKNUM

        flashBuf[wBytes] = (byte)(pBlockNode[(wBytes+sizeTxed)/3] >> (((wBytes+sizeTxed) % 3) ? ((((wBytes+sizeTxed) % 3)==2) ? 0 : 8): 16)) & 0xFF;
#else

        flashBuf[wBytes] = (byte)(pBlockNode[(wBytes+sizeTxed)/2] >> (((wBytes+sizeTxed) % 2) ? 0 : 8)) & 0xFF;
#endif
    }


    sizeTxed = (sizeTxed > wBlockTableSize) ? (sizeTxed - wBlockTableSize) : 0;
    wBlockTableSize = FTL_Get_WearCounter_Table_Flash_Size_Bytes();
    wBytesCopied = wBytes;
    wBytes = ((wBlockTableSize - sizeTxed) > (sizeToTx - wBytesCopied)) ?
            (sizeToTx - wBytesCopied) :
            (wBlockTableSize - sizeTxed);
    memcpy(flashBuf+wBytesCopied, GLOB_g_pWearCounter+sizeTxed, wBytes);


    sizeTxed = (sizeTxed > wBlockTableSize) ? (sizeTxed - wBlockTableSize) : 0;
    if (GLOB_DeviceInfo.MLCDevice) {
        wBlockTableSize = FTL_Get_ReadCounter_Table_Flash_Size_Bytes();
        wBytesCopied += wBytes;
        wBytes = 0;
        for(; ((wBytes+wBytesCopied) < sizeToTx) && ((wBytes+sizeTxed) < wBlockTableSize); wBytes++) {
            flashBuf[wBytes+wBytesCopied] = (GLOB_g_pReadCounter[(wBytes+sizeTxed)/2] >> (((wBytes+sizeTxed) % 2) ? 0 : 8)) & 0xFF;
        }
    }
    return wBytesCopied+wBytes;
}

STATIC int FTL_Copy_Block_Table_From_Flash(byte *flashBuf, uint32 sizeToTx, uint32 sizeTxed)
{
    uint32 wBytesCopied, wBlockTableSize, wBytes;
    BLOCKNODE *pBlockNode = (BLOCKNODE *)GLOB_g_pBlockTable;


    wBlockTableSize = FTL_Get_LBAPBA_Table_Flash_Size_Bytes();
    for(wBytes=0; (wBytes < sizeToTx) && ((wBytes+sizeTxed) < wBlockTableSize); wBytes++) {
#if SUPPORT_LARGE_BLOCKNUM
        if (!((wBytes+sizeTxed) % 3))
            pBlockNode[(wBytes+sizeTxed)/3] = 0;

        pBlockNode[(wBytes+sizeTxed)/3] |= (flashBuf[wBytes] << (((wBytes+sizeTxed) % 3) ? ((((wBytes+sizeTxed) % 3)==2) ? 0 : 8): 16));
#else
        if (!((wBytes+sizeTxed) % 2))
            pBlockNode[(wBytes+sizeTxed)/2] = 0;

        pBlockNode[(wBytes+sizeTxed)/2] |= (flashBuf[wBytes] << (((wBytes+sizeTxed) % 2) ? 0 : 8));
#endif
    }


    sizeTxed = (sizeTxed > wBlockTableSize) ? (sizeTxed - wBlockTableSize) : 0;
    wBlockTableSize = FTL_Get_WearCounter_Table_Flash_Size_Bytes();
    wBytesCopied = wBytes;
    wBytes = ((wBlockTableSize - sizeTxed) > (sizeToTx - wBytesCopied)) ?
            (sizeToTx - wBytesCopied) :
            (wBlockTableSize - sizeTxed);
    memcpy(GLOB_g_pWearCounter+sizeTxed, flashBuf+wBytesCopied, wBytes);


    sizeTxed = (sizeTxed > wBlockTableSize) ? (sizeTxed - wBlockTableSize) : 0;
    if (GLOB_DeviceInfo.MLCDevice) {
        wBytesCopied += wBytes;
        wBytes = 0;
        wBlockTableSize = FTL_Get_ReadCounter_Table_Flash_Size_Bytes();
        for(; ((wBytes+wBytesCopied) < sizeToTx) && ((wBytes+sizeTxed) < wBlockTableSize); wBytes++) {
            if (((wBytes+sizeTxed) % 2))
                GLOB_g_pReadCounter[(wBytes+sizeTxed)/2] = 0;
            GLOB_g_pReadCounter[(wBytes+sizeTxed)/2] |= (flashBuf[wBytes] << (((wBytes+sizeTxed) % 2) ? 0 : 8));
        }
    }
    return wBytesCopied+wBytes;
}

STATIC int FTL_Insert_Block_Table_Signature(byte *buf, byte tag)
{
    int i;
    for (i=0; i<BTSIG_BYTES; i++) {
        buf[BTSIG_OFFSET + i] = ((tag + (i*BTSIG_DELTA) - FIRST_BT_ID) % (1+LAST_BT_ID-FIRST_BT_ID)) + FIRST_BT_ID;
    }
    return PASS;
}


STATIC int FTL_Extract_Block_Table_Tag(byte *buf, byte **tagarray)
{
    static byte tag[BTSIG_BYTES >> 1];
    int i, j, k, tagi, tagtemp, status;

    *tagarray = (byte *)tag;
    tagi = 0;
    for (i=0; i<(BTSIG_BYTES - 1); i++) {
        for (j=i+1; (j<BTSIG_BYTES) && (tagi < (BTSIG_BYTES >> 1)); j++) {
            tagtemp = buf[BTSIG_OFFSET + j] - buf[BTSIG_OFFSET + i];
            if (tagtemp && !(tagtemp % BTSIG_DELTA)) {
		tagtemp = (buf[BTSIG_OFFSET + i] + (1+LAST_BT_ID-FIRST_BT_ID) - (i*BTSIG_DELTA) - FIRST_BT_ID)% (1+LAST_BT_ID-FIRST_BT_ID) + FIRST_BT_ID;
                status = FAIL;
                for (k=0; k<tagi; k++) {
                    if (tagtemp == tag[k])
                        status = PASS;
                }
                if (status == FAIL) {
                    tag[tagi++] = tagtemp;

                    i = (j == (i+1)) ? i+1 : i;
                    j = (j == (i+1)) ? i+1 : i;
                }
            }
        }
    }
    return tagi;
}




STATIC int FTL_Execute_SPL_Recovery(void)
{
    BLOCKNODE j, block;
    BLOCKNODE * pBlockNode = (BLOCKNODE *)GLOB_g_pBlockTable;
    for(j = 0; j <= (GLOB_DeviceInfo.wSpectraEndBlock - GLOB_DeviceInfo.wSpectraStartBlock); j++) {
        block = (pBlockNode[j]);
        if (((block & BAD_BLOCK) != BAD_BLOCK) && ((block & SPARE_BLOCK) == SPARE_BLOCK)) {
            if (FAIL == GLOB_LLD_Erase_Block(block & ~BAD_BLOCK
#if CMD_DMA
            ,GLOB_FTLCommandCount,LLD_CMD_FLAG_MODE_POLL
#endif
            )) {
                MARK_BLOCK_AS_BAD(pBlockNode[j]);
            }
        }
    }
    return PASS;
}


/*&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
* Function:     GLOB_FTL_IdentifyDevice
* Inputs:       pointer to identify data structure
* Outputs:      PASS / FAIL
* Description:  the identify data structure is filled in with
*                   information for the block driver.
*&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&*/
int GLOB_FTL_IdentifyDevice(IDENTFY_DEVICE_DATA *IdentfyDeviceData)
{
    int status = PASS;
    int bufMem;


    bufMem = (GLOB_DeviceInfo.wPageDataSize -
        ((GLOB_DeviceInfo.wDataBlockNum * (sizeof(BLOCKNODE) + sizeof(byte)
        + (GLOB_DeviceInfo.MLCDevice ? sizeof(uint16) : 0)
        )) % GLOB_DeviceInfo.wPageDataSize)) % GLOB_DeviceInfo.wPageDataSize;

    print("  In new GLOB_FTL_IdentifyDevice function \n");
    IdentfyDeviceData->NumBlocks          = GLOB_DeviceInfo.wTotalBlocks;
    IdentfyDeviceData->PagesPerBlock      = GLOB_DeviceInfo.wPagesPerBlock;
    IdentfyDeviceData->PageDataSize       = GLOB_DeviceInfo.wPageDataSize;
    IdentfyDeviceData->wECCBytesPerSector = GLOB_DeviceInfo.wECCBytesPerSector;
    IdentfyDeviceData->wDataBlockNum      = GLOB_DeviceInfo.wDataBlockNum;

    IdentfyDeviceData->SizeOfGlobalMem    =
      (GLOB_DeviceInfo.wDataBlockNum  * sizeof(BLOCKNODE) * 2) +
      (GLOB_DeviceInfo.wDataBlockNum * sizeof(byte) + 2) +
      (GLOB_DeviceInfo.MLCDevice ? (GLOB_DeviceInfo.wDataBlockNum * sizeof(uint16)
#if CMD_DMA
      * (1+1+1)
#endif
      ) : 0) +
      bufMem +
#if (PAGES_PER_CACHE_BLOCK > 0)
      ((CACHE_BLOCK_NUMBER+1) * PAGES_PER_CACHE_BLOCK * GLOB_DeviceInfo.wPageDataSize * sizeof(byte)) +
#else
      ((CACHE_BLOCK_NUMBER+1) * GLOB_DeviceInfo.wPagesPerBlock * GLOB_DeviceInfo.wPageDataSize * sizeof(byte)) +
#endif
      (GLOB_DeviceInfo.wPageSize*sizeof(byte)) +
      (GLOB_DeviceInfo.wPagesPerBlock *GLOB_DeviceInfo.wPageDataSize * sizeof(byte)) +
#if CMD_DMA
      (GLOB_DeviceInfo.wDataBlockNum  * sizeof(BLOCKNODE)) +
      (GLOB_DeviceInfo.wDataBlockNum * sizeof(byte)) +
      (5 * ((GLOB_DeviceInfo.wDataBlockNum * sizeof(BLOCKNODE))
            + (GLOB_DeviceInfo.wDataBlockNum * sizeof(byte))
            + (GLOB_DeviceInfo.wDataBlockNum * sizeof(uint16))
      )    ) +
      (MAX_DESCRIPTORS * sizeof(struct BTableChangesDelta)) +
      (10 * GLOB_DeviceInfo.wPagesPerBlock * GLOB_DeviceInfo.wPageDataSize) +
#endif
      ((1+LAST_BT_ID - FIRST_BT_ID) * sizeof(BLOCKNODE)) +


      (GLOB_DeviceInfo.wDataBlockNum) +
      (GLOB_DeviceInfo.wPageDataSize*sizeof(byte)*2) +
      (((GLOB_DeviceInfo.wPageSize-GLOB_DeviceInfo.wPageDataSize) * sizeof(byte)) * 2) +
      (GLOB_DeviceInfo.wDataBlockNum) +
#if !CMD_DMA
      (GLOB_DeviceInfo.wPageDataSize *GLOB_DeviceInfo.wPagesPerBlock *sizeof(byte) * 2) +
#endif
      GLOB_DeviceInfo.wBlockSize +
      GLOB_LLD_Memory_Pool_Size() +
      (NUM_MEMPOOL_ALLOCS * sizeof(byte) * 4);
    globalMemSize = IdentfyDeviceData->SizeOfGlobalMem;

    return status;
}

/*&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
* Function:        GLOB_FTL_Mem_Config
* Inputs:          pointer to the memory that is allocated
* Outputs:         PASS / FAIL
* Description:     This allows the Block Driver to do the memory allocation
*                  and is used in place of the FTL doing malloc's.  The
*                  Block Driver assigns the length based on data passed
*                  to it in the GLOB_FTL_IdentifyDevice function.
*                  There is sanity checking that the pointers are not NULL
*                  There is no sanity checking for the length. If this
*                  becomes neccessary, an additioanl parameter will
*                  be needed.
*&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&*/
int GLOB_FTL_Mem_Config (byte * pMem)
{
    int status = FAIL;

    if(pMem != NULL)
    {
        g_pMemPool = pMem;
        status = GLOB_LLD_Mem_Config(pMem + globalMemSize - GLOB_LLD_Memory_Pool_Size());
    }

    return status;
}

/*&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
* Function:     GLOB_FTL_Init
* Inputs:       none
* Outputs:      PASS=0 / FAIL=1
* Description:  allocates the memory for cache array,
*               important data structures
*               clears the cache array
*               reads the block table from flash into array
*&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&*/
int GLOB_FTL_Init(void)
{
    int i;
    int status =PASS;

#if (PAGES_PER_CACHE_BLOCK > 0)
    GLOB_Cache.wCachePageNum = PAGES_PER_CACHE_BLOCK;
#else
    GLOB_Cache.wCachePageNum = GLOB_DeviceInfo.wPagesPerBlock;
#endif

    GLOB_Cache.dwCacheDataSize = GLOB_Cache.wCachePageNum*GLOB_DeviceInfo.wPageDataSize;

    g_pMemPoolFree = (byte *)g_pMemPool;

    GLOB_g_pBlockTable = (byte *) g_pMemPoolFree;
    memset(GLOB_g_pBlockTable, 0, GLOB_DeviceInfo.wDataBlockNum * sizeof(BLOCKNODE));
    g_pMemPoolFree += (GLOB_DeviceInfo.wDataBlockNum  * sizeof(BLOCKNODE));
    ALIGN_DWORD_FWD(g_pMemPoolFree);

    GLOB_g_pWearCounter = (byte *)g_pMemPoolFree;
    memset(GLOB_g_pWearCounter, 0,GLOB_DeviceInfo.wDataBlockNum * sizeof(byte));
    g_pMemPoolFree += (GLOB_DeviceInfo.wDataBlockNum * sizeof(byte));
    ALIGN_DWORD_FWD(g_pMemPoolFree);

    if(GLOB_DeviceInfo.MLCDevice)
    {
        GLOB_g_pReadCounter = (uint16 *)g_pMemPoolFree;
        g_pMemPoolFree += (GLOB_DeviceInfo.wDataBlockNum * sizeof(uint16));
        memset(GLOB_g_pReadCounter, 0,GLOB_DeviceInfo.wDataBlockNum * sizeof(uint16));
        ALIGN_DWORD_FWD(g_pMemPoolFree);
    }

    for(i = 0; i < CACHE_BLOCK_NUMBER; i++)
    {
        GLOB_Cache.ItemArray[i].dwAddress = INVALID_CACHE_BLK_ADD;
        GLOB_Cache.ItemArray[i].bUCount = 0;
        GLOB_Cache.ItemArray[i].bChanged  = CLEAR;
        GLOB_Cache.ItemArray[i].pContent = (byte *)g_pMemPoolFree;
        g_pMemPoolFree += (GLOB_Cache.dwCacheDataSize * sizeof(byte));
        ALIGN_DWORD_FWD(g_pMemPoolFree);
    }

    g_pIPF = (byte *)g_pMemPoolFree;
    g_pMemPoolFree += (GLOB_DeviceInfo.wPageSize*sizeof(byte));
    memset(g_pIPF,0,GLOB_DeviceInfo.wPageSize);
    ALIGN_DWORD_FWD(g_pMemPoolFree);

    g_pTempBuf = (byte *)g_pMemPoolFree;
    g_pMemPoolFree += (GLOB_Cache.dwCacheDataSize*sizeof(byte));
    ALIGN_DWORD_FWD(g_pMemPoolFree);

    g_temp_buf = (byte *)g_pMemPoolFree;
    g_pMemPoolFree += (GLOB_DeviceInfo.wPagesPerBlock *GLOB_DeviceInfo.wPageDataSize *
    sizeof(byte));
    memset(g_temp_buf,0xFF,GLOB_DeviceInfo.wPagesPerBlock * GLOB_DeviceInfo.wPageDataSize);
    ALIGN_DWORD_FWD(g_pMemPoolFree);


#if CMD_DMA
    GLOB_g_pBTStartingCopy = (byte *)g_pMemPoolFree;
    memset(GLOB_g_pBTStartingCopy, 0, GLOB_DeviceInfo.wDataBlockNum * sizeof(BLOCKNODE));
    g_pMemPoolFree += (GLOB_DeviceInfo.wDataBlockNum  * sizeof(BLOCKNODE));
    ALIGN_DWORD_FWD(g_pMemPoolFree);

    GLOB_g_pWearCounterCopy = (byte *)g_pMemPoolFree;
    memset(GLOB_g_pWearCounterCopy, 0,GLOB_DeviceInfo.wDataBlockNum * sizeof(byte));
    g_pMemPoolFree += (GLOB_DeviceInfo.wDataBlockNum * sizeof(byte));
    ALIGN_DWORD_FWD(g_pMemPoolFree);

    if(GLOB_DeviceInfo.MLCDevice)
    {
        GLOB_g_pReadCounterCopy = (uint16 *)g_pMemPoolFree;
        memset(GLOB_g_pReadCounterCopy, 0,GLOB_DeviceInfo.wDataBlockNum * sizeof(uint16));
        g_pMemPoolFree += (GLOB_DeviceInfo.wDataBlockNum * sizeof(uint16));
        ALIGN_DWORD_FWD(g_pMemPoolFree);
    }
    GLOB_g_pBlockTableCopies = (byte *)g_pMemPoolFree;
    GLOB_g_pNextBlockTable = GLOB_g_pBlockTableCopies;

    if(GLOB_DeviceInfo.MLCDevice)
    {
        g_pMemPoolFree += (5*(GLOB_DeviceInfo.wDataBlockNum
        * sizeof(BLOCKNODE)+GLOB_DeviceInfo.wDataBlockNum * sizeof(byte)+
        GLOB_DeviceInfo.wDataBlockNum * sizeof(uint16)));
    }
    else
    {
        g_pMemPoolFree += (5*(GLOB_DeviceInfo.wDataBlockNum
        * sizeof(BLOCKNODE)+GLOB_DeviceInfo.wDataBlockNum * sizeof(byte)));
    }
    ALIGN_DWORD_FWD(g_pMemPoolFree);

    GLOB_g_pBTDelta = (byte *)g_pMemPoolFree;
    g_pMemPoolFree += (MAX_DESCRIPTORS  * sizeof(struct BTableChangesDelta));
    ALIGN_DWORD_FWD(g_pMemPoolFree);
    GLOB_FTLCommandCount = 0;
    GLOB_g_pBTDelta_Free = (byte *)GLOB_g_pBTDelta;
    GLOB_g_pCopyBackBufferCopies = (byte *) g_pMemPoolFree;
    g_pMemPoolFree += (10* GLOB_DeviceInfo.wPagesPerBlock *GLOB_DeviceInfo.wPageDataSize);
    ALIGN_DWORD_FWD(g_pMemPoolFree);
    GLOB_g_pCopyBackBufferStart = GLOB_g_pCopyBackBufferCopies;

#if RESTORE_CACHE_ON_CDMA_CHAIN_FAILURE
    memcpy((void *) &GLOB_Cache_StartingCopy,(void *) &GLOB_Cache, sizeof(FLASH_CACHE));

    memset((void *) &GLOB_Int_Cache, -1, sizeof(FLASH_CACHE_DELTA_LIST) * (MAX_DESCRIPTORS + LLD_NUM_FLASH_CHANNELS) );
#endif
#endif
    GLOB_g_pBTBlocks = (BLOCKNODE *)g_pMemPoolFree;
    g_pMemPoolFree += ((1+LAST_BT_ID - FIRST_BT_ID) * sizeof(BLOCKNODE));
    ALIGN_DWORD_FWD(g_pMemPoolFree);
    memset(GLOB_g_pBTBlocks,0xFF,(1+LAST_BT_ID - FIRST_BT_ID) * sizeof(BLOCKNODE));
    debug_boundary_error(((int)g_pMemPoolFree-(int)g_pMemPool)-1, globalMemSize, 0);

    status =FTL_Read_Block_Table();

#if CMD_DMA
    GLOB_FTLCommandCount = 0;
#endif
    return status;
}

#if CMD_DMA
/*&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
* Function:    	GLOB_FTL_Event_Status
* Inputs:       Pointer to location to return the first failed command
*               This is undefined in case of EVENT_PASS or
* Outputs:      Event Code
* Description:	It is called by SBD after hardware interrupt signalling
*               completion of commands chain
*               It does following things
*               get event status from LLD
*               analyze command chain status
*               determine last command executed
*               analyze results
*               rebuild the block table in case of uncorrectable error
*               return event code
*&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&*/
int GLOB_FTL_Event_Status (int* first_failed_cmd)
{
    int event_code = PASS;
    byte *GLOB_g_pBTDelta_current = GLOB_g_pBTDelta;
    uint16  i_P;
    BLOCKNODE * pLocalBlockNode = (BLOCKNODE *)GLOB_g_pBTStartingCopy;
    byte Current_FTLCommand;
    BLOCKNODE s_lba;
    byte k,bCacheBlockNum;

#if RESTORE_CACHE_ON_CDMA_CHAIN_FAILURE
    uint16  i;
    uint16  i_P_C;
#endif

    bCacheBlockNum = UNHIT_BLOCK;

    *first_failed_cmd =0;

    event_code = GLOB_LLD_Event_Status();
    print("GLOB_FTL_Event_Status function is called \n");
    print("Event Code got from lld %d\n",event_code);
    switch (event_code)
    {
        case  EVENT_PASS:
            print(" Handling EVENT_PASS\n");
            break;
        case  EVENT_CORRECTABLE_DATA_ERROR_FIXED:
            print("Handling EVENT_CORRECTABLE_DATA_ERROR_FIXED");
            return event_code;
        case  EVENT_UNCORRECTABLE_DATA_ERROR:
        case  EVENT_PROGRAM_FAILURE:
        case  EVENT_ERASE_FAILURE:
        case  EVENT_TIME_OUT:

            print("Handling Ugly case\n");



            print("UNCORRECTABLE DATA ERROR HAS HAPPENED\n");


            GLOB_p_BTableChangesDelta = (struct BTableChangesDelta *)
                    GLOB_g_pBTDelta_current;
            for (i_P = LLD_NUM_FLASH_CHANNELS;i_P < (GLOB_FTLCommandCount+LLD_NUM_FLASH_CHANNELS);i_P++)
            {
                if(GLOB_PendingCMD[i_P].Status == CMD_PASS)
                {
                    if (0 == *first_failed_cmd)
                    {
#if RESTORE_CACHE_ON_CDMA_CHAIN_FAILURE
                        i_P_C = i_P - LLD_NUM_FLASH_CHANNELS;
                        if(GLOB_Int_Cache[i_P_C].nUsedCacheItem != -1)
                        {
                            bCacheBlockNum = GLOB_Int_Cache[i_P_C].nUsedCacheItem;

                            GLOB_Cache_StartingCopy.ItemArray[bCacheBlockNum].dwAddress=
                                GLOB_Int_Cache[i_P_C].IntCache.dwAddress;
                            GLOB_Cache_StartingCopy.ItemArray[bCacheBlockNum].bChanged  =
                                GLOB_Int_Cache[i_P_C].IntCache.bChanged;
                        }
#endif

                        Current_FTLCommand =
                            GLOB_p_BTableChangesDelta->GLOB_FTLCommandCount;

                        while (Current_FTLCommand <= GLOB_PendingCMD[i_P].Tag)
                        {
                            if(GLOB_p_BTableChangesDelta->ValidFields == 0x01)
                            {
                                g_wBlockTableOffset =
                                    GLOB_p_BTableChangesDelta->g_wBlockTableOffset;
                            }
                            else if(GLOB_p_BTableChangesDelta->ValidFields == 0x0C)
                            {
                                pLocalBlockNode[GLOB_p_BTableChangesDelta->BT_Index] =
                                    GLOB_p_BTableChangesDelta->BT_Entry_Value;
                                debug_boundary_error(((GLOB_p_BTableChangesDelta->BT_Index)), GLOB_DeviceInfo.wDataBlockNum, 0);
                            }
                            else if(GLOB_p_BTableChangesDelta->ValidFields == 0x03)
                            {
                                g_wBlockTableOffset =
                                    GLOB_p_BTableChangesDelta->g_wBlockTableOffset;
                                g_wBlockTableIndex =
                                    GLOB_p_BTableChangesDelta->g_wBlockTableIndex;
                            }
                            else if(GLOB_p_BTableChangesDelta->ValidFields == 0x30)
                            {
                                GLOB_g_pWearCounterCopy[GLOB_p_BTableChangesDelta->WC_Index]
                                    = GLOB_p_BTableChangesDelta->WC_Entry_Value;
                            }
                            else if((GLOB_DeviceInfo.MLCDevice) && (GLOB_p_BTableChangesDelta->ValidFields == 0xC0))
                            {
                                GLOB_g_pReadCounterCopy[GLOB_p_BTableChangesDelta->RC_Index]
                                    = GLOB_p_BTableChangesDelta->RC_Entry_Value;
                                print("In event status setting read counter GLOB_FTLCommandCount %u Count %u Index %u\n",Current_FTLCommand,GLOB_p_BTableChangesDelta->RC_Entry_Value,GLOB_p_BTableChangesDelta->RC_Index);
                            }
                            else
                            {
                                print("GLOB_FTL_Event_Status - This should never occur\n");
                            }
                            GLOB_p_BTableChangesDelta +=1;
                            Current_FTLCommand =
                                GLOB_p_BTableChangesDelta->GLOB_FTLCommandCount;
                        }
                    }
                    else
                    {
                        print("CMD PASSED after a cmd failure on another channel \n");

                        if ((GLOB_PendingCMD[i_P].CMD == WRITE_MAIN_CMD) ||
                                (GLOB_PendingCMD[i_P].CMD == WRITE_MAIN_SPARE_CMD))
                        {
                            for(s_lba =0; s_lba <GLOB_DeviceInfo.wDataBlockNum;s_lba++)
                            {
                                if(GLOB_PendingCMD[i_P].Block ==
                                        (pLocalBlockNode[s_lba]&(~BAD_BLOCK)))
                                {
                                    MARK_BLOCK_AS_DISCARD(pLocalBlockNode[s_lba]);
                                }
                            }
                        }

                        Current_FTLCommand =
                            GLOB_p_BTableChangesDelta->GLOB_FTLCommandCount;

                        while (Current_FTLCommand <= GLOB_PendingCMD[i_P].Tag)
                        {
                            GLOB_p_BTableChangesDelta +=1;
                            Current_FTLCommand =
                                GLOB_p_BTableChangesDelta->GLOB_FTLCommandCount;
                        }

#if RESTORE_CACHE_ON_CDMA_CHAIN_FAILURE
                        i_P_C = i_P - LLD_NUM_FLASH_CHANNELS;

                        if(GLOB_Int_Cache[i_P_C].nUsedCacheItem != -1)
                        {
                            bCacheBlockNum = GLOB_Int_Cache[i_P_C].nUsedCacheItem;
                            if(GLOB_PendingCMD[i_P].CMD == MEMCOPY_CMD)
                            {

                                if( (GLOB_Cache_StartingCopy.ItemArray[bCacheBlockNum].pContent <= GLOB_PendingCMD[i_P].DataDestAddr) &&
                                        ( (GLOB_Cache_StartingCopy.ItemArray[bCacheBlockNum].pContent + GLOB_Cache.dwCacheDataSize) >
                                                GLOB_PendingCMD[i_P].DataDestAddr) )
                                {
                                    GLOB_Cache_StartingCopy.ItemArray[bCacheBlockNum].dwAddress = INVALID_CACHE_BLK_ADD;
                                    GLOB_Cache_StartingCopy.ItemArray[bCacheBlockNum].bUCount = 0;
                                    GLOB_Cache_StartingCopy.ItemArray[bCacheBlockNum].bChanged  = CLEAR;
                                }
                            }
                            else
                            {
                                GLOB_Cache_StartingCopy.ItemArray[bCacheBlockNum].dwAddress=
                                    GLOB_Int_Cache[i_P_C].IntCache.dwAddress;

                                GLOB_Cache_StartingCopy.ItemArray[bCacheBlockNum].bChanged  =
                                    GLOB_Int_Cache[i_P_C].IntCache.bChanged;
                            }
                        }
#endif
                    }
                }
                else if((GLOB_PendingCMD[i_P].Status ==	  CMD_FAIL)||
                                (GLOB_PendingCMD[i_P].Status == CMD_ABORT))
                {

                    if (0 == *first_failed_cmd)
                        *first_failed_cmd = GLOB_PendingCMD[i_P].SBDCmdIndex;




                    print("Uncorrectable error has occured while executing %u\
                            Command %u accesing Block %u\n",
                            (unsigned int)GLOB_p_BTableChangesDelta->GLOB_FTLCommandCount,
                            GLOB_PendingCMD[i_P].CMD,GLOB_PendingCMD[i_P].Block);

                    Current_FTLCommand =
                        GLOB_p_BTableChangesDelta->GLOB_FTLCommandCount;

                    while (Current_FTLCommand <= GLOB_PendingCMD[i_P].Tag)
                    {
                        GLOB_p_BTableChangesDelta +=1;
                        Current_FTLCommand =
                            GLOB_p_BTableChangesDelta->GLOB_FTLCommandCount;
                    }

#if RESTORE_CACHE_ON_CDMA_CHAIN_FAILURE
                        i_P_C = i_P - LLD_NUM_FLASH_CHANNELS;

                        if(GLOB_Int_Cache[i_P_C].nUsedCacheItem != -1)
                        {
                            bCacheBlockNum = GLOB_Int_Cache[i_P_C].nUsedCacheItem;
                            if ((GLOB_PendingCMD[i_P].CMD == WRITE_MAIN_CMD))
                            {
                                GLOB_Cache_StartingCopy.ItemArray[bCacheBlockNum].dwAddress=
                                    GLOB_Int_Cache[i_P_C].IntCache.dwAddress;
                                GLOB_Cache_StartingCopy.ItemArray[bCacheBlockNum].bChanged  = SET;
                            }
                            else if((GLOB_PendingCMD[i_P].CMD == READ_MAIN_CMD))
                            {

                                GLOB_Cache_StartingCopy.ItemArray[bCacheBlockNum].dwAddress = INVALID_CACHE_BLK_ADD;
                                GLOB_Cache_StartingCopy.ItemArray[bCacheBlockNum].bUCount = 0;
                                GLOB_Cache_StartingCopy.ItemArray[bCacheBlockNum].bChanged  = CLEAR;
                            }
                            else if(GLOB_PendingCMD[i_P].CMD == ERASE_CMD)
                            {
                            }
                            else if(GLOB_PendingCMD[i_P].CMD == MEMCOPY_CMD)
                            {

                            }
                        }
#endif



                    if(((event_code == EVENT_ERASE_FAILURE) && (GLOB_PendingCMD[i_P].CMD ==
                            ERASE_CMD)) || ((event_code == EVENT_PROGRAM_FAILURE) &&
                                    ((GLOB_PendingCMD[i_P].CMD == WRITE_MAIN_CMD) ||
                                            (GLOB_PendingCMD[i_P].CMD == WRITE_MAIN_SPARE_CMD))))
                    {

                        for(s_lba =0; s_lba <GLOB_DeviceInfo.wDataBlockNum;s_lba++)
                        {
                            if(GLOB_PendingCMD[i_P].Block ==
                                    (pLocalBlockNode[s_lba]&(~BAD_BLOCK)))
                            {
                                MARK_BLOCK_AS_BAD(pLocalBlockNode[s_lba]);
                            }
                        }
                    }
                }
                else if((GLOB_PendingCMD[i_P].Tag) && (GLOB_PendingCMD[i_P].Status ==
                                CMD_NOT_DONE))
                {

                    print(" Command no. %hu is not\
                        executed\n",(unsigned int)GLOB_PendingCMD[i_P].Tag);

                    Current_FTLCommand =
                        GLOB_p_BTableChangesDelta->GLOB_FTLCommandCount;

                    while (Current_FTLCommand <= GLOB_PendingCMD[i_P].Tag)
                    {
                        GLOB_p_BTableChangesDelta +=1;
                        Current_FTLCommand =
                            GLOB_p_BTableChangesDelta->GLOB_FTLCommandCount;
                    }
                }

            }

            memcpy(GLOB_g_pBlockTable,GLOB_g_pBTStartingCopy,(GLOB_DeviceInfo.wDataBlockNum*sizeof(BLOCKNODE)));
            memcpy(GLOB_g_pWearCounter,GLOB_g_pWearCounterCopy,GLOB_DeviceInfo.wDataBlockNum*sizeof(byte));
            if(GLOB_DeviceInfo.MLCDevice)
            {
                memcpy(GLOB_g_pReadCounter,GLOB_g_pReadCounterCopy,GLOB_DeviceInfo.wDataBlockNum*sizeof(uint16));
            }

#if RESTORE_CACHE_ON_CDMA_CHAIN_FAILURE
            memcpy((void *) &GLOB_Cache,(void *) &GLOB_Cache_StartingCopy, sizeof(FLASH_CACHE));

            memset((void *) &GLOB_Int_Cache, -1, sizeof(FLASH_CACHE_DELTA_LIST) * (MAX_DESCRIPTORS + LLD_NUM_FLASH_CHANNELS) );
#endif
            break;

        default:
		print("Handling default case\n");
            event_code = FAIL;
            break;
    }

    memcpy(GLOB_g_pBTStartingCopy,GLOB_g_pBlockTable,(GLOB_DeviceInfo.wDataBlockNum*sizeof(BLOCKNODE)));
    memcpy(GLOB_g_pWearCounterCopy,GLOB_g_pWearCounter,GLOB_DeviceInfo.wDataBlockNum*sizeof(byte));


    if(GLOB_DeviceInfo.MLCDevice)
    {
        memcpy(GLOB_g_pReadCounterCopy,GLOB_g_pReadCounter,GLOB_DeviceInfo.wDataBlockNum*sizeof(uint16));
    }

    GLOB_g_pBTDelta_Free = GLOB_g_pBTDelta;
    GLOB_FTLCommandCount = 0;

    GLOB_g_pNextBlockTable = GLOB_g_pBlockTableCopies;

    GLOB_g_pCopyBackBufferStart = GLOB_g_pCopyBackBufferCopies;

#if RESTORE_CACHE_ON_CDMA_CHAIN_FAILURE
    memcpy((void *) &GLOB_Cache_StartingCopy, (void *) &GLOB_Cache, sizeof(FLASH_CACHE));

    memset((void *) &GLOB_Int_Cache, -1, sizeof(FLASH_CACHE_DELTA_LIST) * (MAX_DESCRIPTORS + LLD_NUM_FLASH_CHANNELS) );
#endif

    return event_code;
}


/*&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
* Function:    	GLOB_FTL_Enable_Disable_Interrupts
* Inputs:       enable or disable
* Outputs:      none
* Description:	pass thru to LLD
***********************************************************************/
void GLOB_FTL_Enable_Disable_Interrupts(uint16 INT_ENABLE)
{
    GLOB_LLD_Enable_Disable_Interrupts( INT_ENABLE);
}


/*&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
* Function:     GLOB_FTL_Execute_CMDS
* Inputs:       none
* Outputs:      none
* Description:  pass thru to LLD
***********************************************************************/
void GLOB_FTL_Execute_CMDS(void)
{
    print("GLOB_FTL_Execute_CMDS: GLOB_FTLCommandCount %u\n",(unsigned int)GLOB_FTLCommandCount);
    g_SBDCmdIndex = 0;
    GLOB_LLD_Execute_CMDs(GLOB_FTLCommandCount);
}

#endif

/*&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
* Function:     GLOB_FTL_Read Immediate
* Inputs:         pointer to data
*                     address of data
* Outputs:      PASS / FAIL
* Description:  Reads one page of data into RAM directly from flash without
*       using or disturbing GLOB_Cache.It is assumed this function is called
*       with CMD-DMA disabled.
***********************************************************************/
#if !CMD_DMA
int  GLOB_FTL_Read_Immediate (byte * read_data, ADDRESSTYPE addr)
{
    int wResult = FAIL;
    BLOCKNODE   Block;
    PAGENUMTYPE Page;
    BLOCKNODE physical_block;

    BLOCKNODE * pBlockNode = (BLOCKNODE *)GLOB_g_pBlockTable;

    Block     = BLK_FROM_ADDR(addr);
    Page      = PAGE_FROM_ADDR(addr, Block);

    if(!IS_SPARE_BLOCK(Block))
        return FAIL;

    physical_block = pBlockNode[Block];
    wResult =
        GLOB_LLD_Read_Page_Main(read_data,physical_block,Page,1);

    if(GLOB_DeviceInfo.MLCDevice)
    {
        GLOB_g_pReadCounter[physical_block - GLOB_DeviceInfo.wSpectraStartBlock]++;

        if(GLOB_g_pReadCounter[physical_block - GLOB_DeviceInfo.wSpectraStartBlock] >= MAX_READ_COUNTER)
            FTL_Read_Disturbance(physical_block);

        if(IN_PROGRESS_BLOCK_TABLE != g_cBlockTableStatus)
        {
            g_cBlockTableStatus = IN_PROGRESS_BLOCK_TABLE;
            FTL_Write_IN_Progress_Block_Table_Page();
        }
    }


    return wResult;
}
#endif


#ifdef SUPPORT_BIG_ENDIAN
/*&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
* Function:     FTL_Invert_Block_Table
* Inputs:       none
* Outputs:      none
* Description:  Re-format the block table in ram based on BIG_ENDIAN and
                LARGE_BLOCKNUM if necessary
*&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&*/
STATIC void FTL_Invert_Block_Table(void)
{
    BLOCKNODE i;
    BLOCKNODE * pBlockNode = (BLOCKNODE *)GLOB_g_pBlockTable;

#ifdef SUPPORT_LARGE_BLOCKNUM
    for(i = 0; i < GLOB_DeviceInfo.wDataBlockNum; i++)
    {
        pBlockNode[i] = INVERTUINT32(pBlockNode[i]);
        GLOB_g_pWearCounter[i] = INVERTUINT32(GLOB_g_pWearCounter[i]);
    }
#else
    for(i = 0; i < GLOB_DeviceInfo.wDataBlockNum; i++)
    {
        pBlockNode[i] = INVERTUINT16(pBlockNode[i]);
        GLOB_g_pWearCounter[i] = INVERTUINT16(GLOB_g_pWearCounter[i]);
    }
#endif

}
#else
#define FTL_Invert_Block_Table()
#endif



/*&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
* Function:     GLOB_FTL_Flash_Init
* Inputs:       none
* Outputs:      PASS=0 / FAIL=0x01 (based on read ID)
* Description:  The flash controller is initialized
*               The flash device is reset
*               Perform a flash READ ID command to confirm that a
*                   valid device is attached and active.
*                   The GLOB_DeviceInfo structure gets filled in
*&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&*/
int GLOB_FTL_Flash_Init (void)
{
    int status = FAIL;

#if CMD_DMA
    GLOB_LLD_Flash_Init(LLD_CMD_FLAG_MODE_POLL);
#else
    if ((status = GLOB_LLD_Flash_Init())!= PASS)
		return status;
#endif
    status = GLOB_LLD_Read_Device_ID();

    return status;
}

/*&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
* Function:     GLOB_FTL_Flash_Release
* Inputs:       none
* Outputs:      PASS=0 / FAIL=0x01 (based on read ID)
* Description:  The flash controller is released
*&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&*/
int GLOB_FTL_Flash_Release (void)
{
    return GLOB_LLD_Flash_Release();
}

/*&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
* Function:     GLOB_FTL_Cache_Release
* Inputs:       none
* Outputs:      none
* Description:  release all allocated memory in GLOB_FTL_Init
*               (allocated in GLOB_FTL_Init)
*&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&*/
void GLOB_FTL_Cache_Release(void)
{
    return;
}


/*&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
* Function:     FTL_Cache_If_Hit
* Inputs:       Page Address
* Outputs:      Block number/UNHIT BLOCK
* Description:  Determines if the addressed page is in cache
*&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&*/
STATIC byte FTL_Cache_If_Hit (ADDRESSTYPE dwPageAddr)
{
    byte i,bCacheBlockNum;
    ADDRESSTYPE dwAddress;

    bCacheBlockNum = UNHIT_BLOCK;
    for(i = 0; i < CACHE_BLOCK_NUMBER; i++)
    {
        dwAddress = GLOB_Cache.ItemArray[i].dwAddress;
        if((dwAddress <= dwPageAddr) && (dwAddress + GLOB_Cache.dwCacheDataSize >
        dwPageAddr))
        {
            bCacheBlockNum = i;
            break;
        }
    }

    return bCacheBlockNum;
}


/*&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
* Function:     FTL_Calculate_LFU
* Inputs:       None
* Outputs:      None
* Description:  Calculate the least frequently used block in a cache and record
*               its index in bLFU field.
*&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&*/
STATIC void FTL_Calculate_LFU(void)
{
    byte i, bCurrentLFU, bTempCount;

    bCurrentLFU = 0;
    bTempCount  = MAX_BYTE_VALUE;
    for(i = 0; i < CACHE_BLOCK_NUMBER; i++)
    {
        if(GLOB_Cache.ItemArray[i].bUCount < bTempCount)
        {
            bCurrentLFU = i;
            bTempCount = GLOB_Cache.ItemArray[i].bUCount;
        }
    }
    GLOB_Cache.bLFU = bCurrentLFU;
}

/*&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
* Function:     FTL_Cache_Read_Page
* Inputs:       pointer to read buffer,page address and block number in a cache
* Outputs:      None
* Description:  Read the page from the cached block addressed by blocknumber
*&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&*/
STATIC void FTL_Cache_Read_Page(byte * pData, ADDRESSTYPE dwPageAddr, byte bCacheBlockNum)
{
    byte *pSrc;
    ADDRESSTYPE dwAddress;

    dwAddress = GLOB_Cache.ItemArray[bCacheBlockNum].dwAddress;
    pSrc = GLOB_Cache.ItemArray[bCacheBlockNum].pContent;

    pSrc += (uint32)( ((dwPageAddr - dwAddress) >> GLOB_DeviceInfo.nBitsInPageDataSize) *
    GLOB_DeviceInfo.wPageDataSize);
#if CMD_DMA
    GLOB_LLD_MemCopy_CMD(GLOB_FTLCommandCount,pData, pSrc,
        GLOB_DeviceInfo.wPageDataSize,0);
    GLOB_FTLCommandCount++;
#else
    memcpy(pData, pSrc, GLOB_DeviceInfo.wPageDataSize);
#endif

    if(GLOB_Cache.ItemArray[bCacheBlockNum].bUCount < MAX_BYTE_VALUE)
        GLOB_Cache.ItemArray[bCacheBlockNum].bUCount++;

    return;
}

/*&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
* Function:     FTL_Cache_Read_All
* Inputs:       pointer to read buffer,block address
* Outputs:      PASS=0 / FAIL =1
* Description:  It reads pages in cache
*&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&*/
STATIC int FTL_Cache_Read_All(byte *pData, ADDRESSTYPE dwBlockAddr)
{
    int wResult;
    BLOCKNODE Block;
    BLOCKNODE lba = BAD_BLOCK;
    PAGENUMTYPE Page;
    PAGENUMTYPE PageCount;
    BLOCKNODE * pBlockNode = (BLOCKNODE *)GLOB_g_pBlockTable;
    uint32 i;
    byte bCacheBlockNum;

    wResult = PASS;

    Block     = BLK_FROM_ADDR(dwBlockAddr);
    Page      = PAGE_FROM_ADDR(dwBlockAddr, Block);
    PageCount = GLOB_Cache.wCachePageNum;

    print("FTL_Cache_Read_All: Reading Block %u\n",Block);



    for(i=0;i<GLOB_DeviceInfo.wDataBlockNum;i++)
    {
        if(Block == ( pBlockNode[i] & (~BAD_BLOCK)))
        {
            lba = i;
            if(IS_SPARE_BLOCK(i))
            {
#if CMD_DMA
                GLOB_LLD_MemCopy_CMD(GLOB_FTLCommandCount,pData,g_temp_buf,PageCount*GLOB_DeviceInfo.wPageDataSize,0);

                GLOB_FTLCommandCount++;
#else
                memset(pData,0xFF,PageCount*GLOB_DeviceInfo.wPageDataSize);
#endif
                return wResult;
            }
            else
                continue;
        }
    }

    if (lba == BAD_BLOCK)
    {
        print(" FTL_Cache_Read_All: Block is not found in BT \n");
    }

#if CMD_DMA
    wResult =
        GLOB_LLD_Read_Page_Main(pData,Block,Page,PageCount,GLOB_FTLCommandCount,LLD_CMD_FLAG_MODE_CDMA);

    if(GLOB_DeviceInfo.MLCDevice)
    {
        GLOB_g_pReadCounter[Block - GLOB_DeviceInfo.wSpectraStartBlock]++;
        print("Read Counter modified in GLOB_FTLCommandCount %u Block %u Counter%u\n",GLOB_FTLCommandCount,Block,GLOB_g_pReadCounter[Block - GLOB_DeviceInfo.wSpectraStartBlock]);

        GLOB_p_BTableChangesDelta = (struct BTableChangesDelta *)GLOB_g_pBTDelta_Free;
        GLOB_g_pBTDelta_Free += sizeof(struct BTableChangesDelta);
        GLOB_p_BTableChangesDelta->GLOB_FTLCommandCount = GLOB_FTLCommandCount;
        GLOB_p_BTableChangesDelta->RC_Index = Block - GLOB_DeviceInfo.wSpectraStartBlock;
        GLOB_p_BTableChangesDelta->RC_Entry_Value = GLOB_g_pReadCounter[Block -
            GLOB_DeviceInfo.wSpectraStartBlock];
        GLOB_p_BTableChangesDelta->ValidFields = 0xC0;

        GLOB_FTLCommandCount++;
        if(GLOB_g_pReadCounter[Block - GLOB_DeviceInfo.wSpectraStartBlock] >= MAX_READ_COUNTER)
            FTL_Read_Disturbance(Block);
        if(IN_PROGRESS_BLOCK_TABLE != g_cBlockTableStatus)
        {
            g_cBlockTableStatus = IN_PROGRESS_BLOCK_TABLE;
            FTL_Write_IN_Progress_Block_Table_Page();
        }
    }
    else
        GLOB_FTLCommandCount++;
#else
    wResult = GLOB_LLD_Read_Page_Main(pData,Block,Page,PageCount);

    if(wResult == FAIL)
    {
        return wResult;
    }

    if(GLOB_DeviceInfo.MLCDevice)
    {
        GLOB_g_pReadCounter[Block - GLOB_DeviceInfo.wSpectraStartBlock]++;

        if(GLOB_g_pReadCounter[Block - GLOB_DeviceInfo.wSpectraStartBlock] >= MAX_READ_COUNTER)
            FTL_Read_Disturbance(Block);

        if(IN_PROGRESS_BLOCK_TABLE != g_cBlockTableStatus)
        {
            g_cBlockTableStatus = IN_PROGRESS_BLOCK_TABLE;
            FTL_Write_IN_Progress_Block_Table_Page();
        }
    }
#endif

    return wResult;
}

/*&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
* Function:     FTL_Cache_Write_All
* Inputs:       pointer to cache in sys memory
*               address of free block in flash
* Outputs:      PASS=0 / FAIL=1
* Description:  writes all the pages of the block in cache to flash
*
*               NOTE:need to make sure this works ok when cache is limited
*               to a partial block. This is where copy-back would be
*               activated.  This would require knowing which pages in the
*               cached block are clean/dirty.Right now we only know if
*               the whole block is clean/dirty.
*&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&*/
STATIC int FTL_Cache_Write_All(byte *pData, ADDRESSTYPE dwBlockAddr)
{
    uint16 wResult = PASS;
    BLOCKNODE Block;
    PAGENUMTYPE Page;
    PAGENUMTYPE PageCount;
    //BLOCKNODE * pBlockNode = (BLOCKNODE *)GLOB_g_pBlockTable;
    //byte bCacheBlockNum = UNHIT_BLOCK;
   // ADDRESSTYPE dwAddress;
   //int j;
    //BLOCKNODE   lba = BAD_BLOCK;

    print(" This block %hu going to be written on %lu\n",(unsigned short)cache_block_to_write,
        (dwBlockAddr >> GLOB_DeviceInfo.nBitsInBlockDataSize));

    Block     = BLK_FROM_ADDR(dwBlockAddr);
    Page      = PAGE_FROM_ADDR(dwBlockAddr, Block);
    PageCount = GLOB_Cache.wCachePageNum;

#if CMD_DMA
    if(FAIL == GLOB_LLD_Write_Page_Main(pData,
        Block,Page,PageCount,GLOB_FTLCommandCount))
    {
        wResult = FAIL;
    }

    GLOB_FTLCommandCount++;
#else
    if(FAIL == GLOB_LLD_Write_Page_Main(pData, Block,Page,PageCount))
    {
        wResult = FAIL;
    }
#endif
    return wResult;
}

/*&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
* Function:     FTL_Cache_Update_Block
* Inputs:       pointer to buffer,page address,block address
* Outputs:      PASS=0 / FAIL=1
* Description:  It updates the cache
*&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&*/
STATIC int FTL_Cache_Update_Block(byte * pData, ADDRESSTYPE dwOldPageAddr, ADDRESSTYPE dwBlockAddr)
{
    int i;
    int j;
    ADDRESSTYPE dwPageAddrOffset;
    ADDRESSTYPE dwTempCacheAddr;
    byte * pDataBuf = pData;
    int wResult = PASS;
    int wFoundInCache;

    ADDRESSTYPE dwOldBlockAddr = (ADDRESSTYPE)(dwOldPageAddr >> GLOB_DeviceInfo.nBitsInBlockDataSize)
        * GLOB_DeviceInfo.wBlockDataSize;

    PAGENUMTYPE wPageOffset = (PAGENUMTYPE)(GLOB_u64_Remainder(dwOldPageAddr,2) >> GLOB_DeviceInfo.nBitsInPageDataSize);

    for(i = 0; i < GLOB_DeviceInfo.wPagesPerBlock; i += GLOB_Cache.wCachePageNum)
    {
        dwPageAddrOffset = dwOldBlockAddr + i * GLOB_DeviceInfo.wPageDataSize;
        if(i != wPageOffset)
        {
            wFoundInCache = FAIL;
            for(j = 0; j < CACHE_BLOCK_NUMBER; j++)
            {
                dwTempCacheAddr = GLOB_Cache.ItemArray[j].dwAddress;

                dwTempCacheAddr = FTL_Get_Physical_Block_Addr(dwTempCacheAddr)
                    + GLOB_u64_Remainder(dwTempCacheAddr, 2);

                if(dwTempCacheAddr >= dwPageAddrOffset
                        && dwTempCacheAddr < (dwPageAddrOffset +
                        GLOB_Cache.dwCacheDataSize))
                {
                    wFoundInCache = PASS;
                    pDataBuf = GLOB_Cache.ItemArray[j].pContent;
                    GLOB_Cache.ItemArray[j].bChanged = SET;
#if CMD_DMA
#if RESTORE_CACHE_ON_CDMA_CHAIN_FAILURE
                    GLOB_Int_Cache[GLOB_FTLCommandCount].nUsedCacheItem = j;
                    GLOB_Int_Cache[GLOB_FTLCommandCount].IntCache.dwAddress =
                    GLOB_Cache.ItemArray[j].dwAddress;

                    GLOB_Int_Cache[GLOB_FTLCommandCount].IntCache.bChanged =
                        GLOB_Cache.ItemArray[j].bChanged;
#endif
#endif

                    break;
                }
            }
            if(FAIL == wFoundInCache)
            {
                if(ERR == FTL_Cache_Read_All(g_pTempBuf, dwPageAddrOffset))
                {
                    wResult = FAIL;
                    break;
                }
                pDataBuf = g_pTempBuf;
            }
        }
        else
        {
            pDataBuf = pData;
        }
        if(FAIL == FTL_Cache_Write_All(pDataBuf,
                dwBlockAddr + (dwPageAddrOffset - dwOldBlockAddr)))
        {
            wResult = FAIL;
            break;
        }
    }
    return wResult;
}

/*&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
* Function:     FTL_Copy_Block
* Inputs:       source block address
*               Destination block address
* Outputs:      PASS=0 / FAIL=1
* Description:  used only for static wear leveling to move the block
*               containing static data to new blocks(more worn)
*&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&*/
int FTL_Copy_Block(ADDRESSTYPE dwOldBlockAddr, ADDRESSTYPE dwBlockAddr)
{
    int i;
    int wResult = PASS;

    for(i = 0; i < GLOB_DeviceInfo.wPagesPerBlock; i += GLOB_Cache.wCachePageNum)
    {
        if(ERR == FTL_Cache_Read_All(g_pTempBuf, dwOldBlockAddr +
                i *  GLOB_DeviceInfo.wPageDataSize) ||
                FAIL == FTL_Cache_Write_All(g_pTempBuf, dwBlockAddr +
                i *  GLOB_DeviceInfo.wPageDataSize))
        {
            wResult = FAIL;
            break;
        }
    }
    return wResult;
}

/*&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
* Function:     FTL_Cache_Write_Back
* Inputs:       pointer to data cached in sys memory
*               address of free block in flash
* Outputs:      PASS=0 / FAIL=1
* Description:  writes all the pages of Cache Block to flash
*
*&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&*/
STATIC int FTL_Cache_Write_Back(byte * pData, ADDRESSTYPE dwBlockAddr)
{
    int i,j;
    ADDRESSTYPE dwOldPageAddr,dwAddress;
    int iErase;
    BLOCKNODE * pBlockNode = (BLOCKNODE *)GLOB_g_pBlockTable;

    BLOCKNODE lba;

    if(GLOB_SYSTEM_FATAL_ERROR())
        return ERR;

    dwOldPageAddr = FTL_Get_Physical_Block_Addr(dwBlockAddr) +
        GLOB_u64_Remainder(dwBlockAddr,2);

    iErase = (FAIL == FTL_Replace_Block(dwBlockAddr)) ? PASS : FAIL;

    pBlockNode[BLK_FROM_ADDR(dwBlockAddr)] &=
      (~SPARE_BLOCK);
#if CMD_DMA
    GLOB_p_BTableChangesDelta = (struct BTableChangesDelta *)GLOB_g_pBTDelta_Free;
    GLOB_g_pBTDelta_Free += sizeof(struct BTableChangesDelta);

    GLOB_p_BTableChangesDelta->GLOB_FTLCommandCount = GLOB_FTLCommandCount;
    GLOB_p_BTableChangesDelta->BT_Index = (BLOCKNODE)(dwBlockAddr >>
            GLOB_DeviceInfo.nBitsInBlockDataSize);
    GLOB_p_BTableChangesDelta->BT_Entry_Value = pBlockNode[(BLOCKNODE)(dwBlockAddr >>
            GLOB_DeviceInfo.nBitsInBlockDataSize)];
    GLOB_p_BTableChangesDelta->ValidFields = 0x0C ;
#endif

    if(IN_PROGRESS_BLOCK_TABLE != g_cBlockTableStatus)
    {
        g_cBlockTableStatus = IN_PROGRESS_BLOCK_TABLE;
        FTL_Write_IN_Progress_Block_Table_Page();
    }

    for(i = 0; i < RETRY_TIMES; i++)
    {
        if(PASS == iErase)
        {
            if( FAIL == GLOB_FTL_Block_Erase(FTL_Get_Physical_Block_Addr(dwBlockAddr)))
            {
                lba = BLK_FROM_ADDR(dwBlockAddr);
                MARK_BLOCK_AS_BAD(pBlockNode[lba]);
                i = RETRY_TIMES;
                break;
            }
        }
        for(j = 0; j < CACHE_BLOCK_NUMBER; j++)
        {
            dwAddress = GLOB_Cache.ItemArray[j].dwAddress;
            if((dwAddress <= dwBlockAddr) && (dwAddress + GLOB_Cache.dwCacheDataSize >
                    dwBlockAddr))
                cache_block_to_write = j;
        }
        if(PASS == FTL_Cache_Update_Block(pData,
                dwOldPageAddr,
                FTL_Get_Physical_Block_Addr(dwBlockAddr)))
        {
            cache_block_to_write = UNHIT_BLOCK;
            break;
        }
        else
            iErase = PASS;
    }
    if( i >= RETRY_TIMES)
    {
        if(ERR == FTL_Flash_Error_Handle(pData, dwOldPageAddr, dwBlockAddr))
            return ERR;
        else
            return FAIL;
    }
    return PASS;
}


/*&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
* Function:     FTL_Cache_Write_Page
* Inputs:       Pointer to buffer, page address, cache block number, CDMA Flags
* Outputs:      PASS=0 / FAIL=1
* Description:  It writes the data in Cache Block
*&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&*/
STATIC void FTL_Cache_Write_Page(byte * pData, ADDRESSTYPE dwPageAddr, byte bCacheBlockNum,uint16
        Flags)
{
    byte * pDest;
    ADDRESSTYPE dwAddress;

    dwAddress = GLOB_Cache.ItemArray[bCacheBlockNum].dwAddress;
    pDest = GLOB_Cache.ItemArray[bCacheBlockNum].pContent;

    pDest += (uint32)(dwPageAddr - dwAddress);

    GLOB_Cache.ItemArray[bCacheBlockNum].bChanged = SET;
#if CMD_DMA
#if RESTORE_CACHE_ON_CDMA_CHAIN_FAILURE
    GLOB_Int_Cache[GLOB_FTLCommandCount].nUsedCacheItem = bCacheBlockNum;
    GLOB_Int_Cache[GLOB_FTLCommandCount].IntCache.dwAddress =
        GLOB_Cache.ItemArray[bCacheBlockNum].dwAddress;

    GLOB_Int_Cache[GLOB_FTLCommandCount].IntCache.bChanged =
        GLOB_Cache.ItemArray[bCacheBlockNum].bChanged;
#endif
    GLOB_LLD_MemCopy_CMD(GLOB_FTLCommandCount,pDest, pData,
	  GLOB_DeviceInfo.wPageDataSize,Flags);

    GLOB_FTLCommandCount++;
#else
    memcpy(pDest, pData, GLOB_DeviceInfo.wPageDataSize);
#endif

    if(GLOB_Cache.ItemArray[bCacheBlockNum].bUCount < MAX_BYTE_VALUE)
        GLOB_Cache.ItemArray[bCacheBlockNum].bUCount++;

    return;
}

/*&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
* Function:     FTL_Cache_Write
* Inputs:       none
* Outputs:      PASS=0 / FAIL=1
* Description:  It writes least frequently used Cache block to flash if it
*               has been changed
*&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&*/
STATIC int FTL_Cache_Write(void)
{
    byte bNO;
    int bResult = PASS;
    int i;
    byte least_count = 0xFF;


    FTL_Calculate_LFU ();
    bNO = GLOB_Cache.bLFU;
    print(" FTL_Cache_Write: Least used cache block is %hu\n",(unsigned short)bNO);
    if(SET == GLOB_Cache.ItemArray[bNO].bChanged)
    {
        print("FTL_Cache_Write: Cache Block %hu containing logical block %lu is\
            dirty\n",(unsigned short)bNO,(GLOB_Cache.ItemArray[bNO].dwAddress >>
            GLOB_DeviceInfo.nBitsInBlockDataSize));

#if CMD_DMA
#if RESTORE_CACHE_ON_CDMA_CHAIN_FAILURE
        GLOB_Int_Cache[GLOB_FTLCommandCount].nUsedCacheItem = bNO;
        GLOB_Int_Cache[GLOB_FTLCommandCount].IntCache.dwAddress =
                GLOB_Cache.ItemArray[bNO].dwAddress;

        GLOB_Int_Cache[GLOB_FTLCommandCount].IntCache.bChanged = CLEAR;
#endif
#endif
        bResult = FTL_Cache_Write_Back(GLOB_Cache.ItemArray[bNO].pContent,
            GLOB_Cache.ItemArray[bNO].dwAddress);

        if(bResult != ERR)
        {
            GLOB_Cache.ItemArray[bNO].bChanged = CLEAR;
        }


        least_count = GLOB_Cache.ItemArray[bNO].bUCount;

        for(i = 0; i < CACHE_BLOCK_NUMBER; i++)
        {
            if(i==bNO)
                continue;

            if(GLOB_Cache.ItemArray[i].bUCount >0)
            GLOB_Cache.ItemArray[i].bUCount -= least_count;
        }
    }
    return bResult;
}

/*&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
* Function:     FTL_Cache_Read
* Inputs:       Page address
* Outputs:      PASS=0 / FAIL=1
* Description:  It reads the block from device in Cache Block
*               Set the Use count to 1
*               Mark the Cache Block as clean
*&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&*/
STATIC int FTL_Cache_Read(ADDRESSTYPE dwPageAddr)
{
    ADDRESSTYPE dwCacheAddr;
    byte  bNO = GLOB_Cache.bLFU;
    dwCacheAddr = (ADDRESSTYPE)GLOB_u64_Div(dwPageAddr, GLOB_Cache.dwCacheDataSize) * GLOB_Cache.dwCacheDataSize;
    GLOB_Cache.ItemArray[bNO].bUCount = 1;
    GLOB_Cache.ItemArray[bNO].dwAddress = dwCacheAddr;
    GLOB_Cache.ItemArray[bNO].bChanged  = CLEAR ;
#if CMD_DMA
#if RESTORE_CACHE_ON_CDMA_CHAIN_FAILURE
    GLOB_Int_Cache[GLOB_FTLCommandCount].nUsedCacheItem = bNO;
    GLOB_Int_Cache[GLOB_FTLCommandCount].IntCache.dwAddress =
        GLOB_Cache.ItemArray[bNO].dwAddress;

    GLOB_Int_Cache[GLOB_FTLCommandCount].IntCache.bChanged =
        GLOB_Cache.ItemArray[bNO].bChanged;
#endif
#endif

    print("FTL_Cache_Read: Logical Block %lu is read into cache block no.\
        %hu\n",GLOB_u64_Div(GLOB_Cache.ItemArray[bNO].dwAddress, GLOB_Cache.dwCacheDataSize),(unsigned
        short)bNO);

    return FTL_Cache_Read_All(GLOB_Cache.ItemArray[bNO].pContent,
            FTL_Get_Physical_Block_Addr(dwCacheAddr) + GLOB_u64_Remainder(dwCacheAddr, 2));
}

/*&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
* Function:     FTL_Check_Block_Table
* Inputs:       ?
* Outputs:      PASS=0 / FAIL=1
* Description:  It checks the correctness of each block table entry
*&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&*/
STATIC int FTL_Check_Block_Table (int wOldTable)
{
    BLOCKNODE i;
    int wResult = PASS;
    BLOCKNODE wBlockIndex;
    BLOCKNODE * pBlockNode = (BLOCKNODE *)GLOB_g_pBlockTable;
    byte * pFlag = g_pMemPoolFree;
    g_pMemPoolFree += (GLOB_DeviceInfo.wDataBlockNum);
    ALIGN_DWORD_FWD(g_pMemPoolFree);
    debug_boundary_error(((int)g_pMemPoolFree-(int)g_pMemPool)-1, globalMemSize, 0);

    if(NULL != pFlag)
    {
        memset(pFlag, FAIL, GLOB_DeviceInfo.wDataBlockNum);
        for(i =0; i < GLOB_DeviceInfo.wDataBlockNum ; i++)
        {
            wBlockIndex = (BLOCKNODE)(pBlockNode[i] & (~BAD_BLOCK));

            if(wBlockIndex > GLOB_DeviceInfo.wSpectraEndBlock || PASS ==
                pFlag[i])
            {
                wResult = FAIL;
                break;
            }
            else
                pFlag[i] = PASS;
        }
        g_pMemPoolFree -= (GLOB_DeviceInfo.wDataBlockNum);
        ALIGN_DWORD_BWD(g_pMemPoolFree);
    }
    return wResult;
}

/*&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
* Function:     FTL_Write_Block_Table
* Inputs:       flasg
* Outputs:      0=Block Table was updated. No write done. 1=Block write needs to happen. -1 Error
* Description:  It writes the block table
*               Block table always mapped to LBA 0 which inturn mapped
*               to any physical block
*&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&*/
STATIC int FTL_Write_Block_Table(int wForce)
{
    BLOCKNODE * pBlockNode = (BLOCKNODE *)GLOB_g_pBlockTable;
    int         wSuccess = PASS;
    BLOCKNODE   wTempBlockTableIndex;
    PAGENUMTYPE wBlockTablePageNum;
    byte        blockchangeoccured = 0;

    wBlockTablePageNum = FTL_Get_Block_Table_Flash_Size_Pages();
    if(IN_PROGRESS_BLOCK_TABLE != g_cBlockTableStatus)
        return 0;

    if(PASS == wForce)
    {
        g_wBlockTableOffset = (PAGENUMTYPE)(GLOB_DeviceInfo.wPagesPerBlock -
            wBlockTablePageNum);
#if CMD_DMA
        GLOB_p_BTableChangesDelta = (struct BTableChangesDelta *)GLOB_g_pBTDelta_Free;
        GLOB_g_pBTDelta_Free += sizeof(struct BTableChangesDelta);

        GLOB_p_BTableChangesDelta->GLOB_FTLCommandCount = GLOB_FTLCommandCount;
        GLOB_p_BTableChangesDelta->g_wBlockTableOffset = g_wBlockTableOffset;
        GLOB_p_BTableChangesDelta->ValidFields =0x01;
#endif
    }

    print(" Inside FTL_Write_Block_Table: block %u\
        Page:%u\n",(unsigned int)g_wBlockTableIndex,(unsigned int)g_wBlockTableOffset);

    do
    {
        if(0 == ((g_wBlockTableOffset + wBlockTablePageNum + 1) %
            GLOB_DeviceInfo.wPagesPerBlock) || ((g_wBlockTableOffset + wBlockTablePageNum +
                1)>GLOB_DeviceInfo.wPagesPerBlock) || (FAIL == wSuccess))
        {
            wTempBlockTableIndex = FTL_Replace_Block_Table();
            if(BAD_BLOCK == wTempBlockTableIndex)
                return ERR;
            if (!blockchangeoccured) {
                bt_block_changed = 1;
                blockchangeoccured = 1;
            }

            g_wBlockTableIndex = wTempBlockTableIndex;
            g_wBlockTableOffset = 0;
            pBlockNode[BLOCK_TABLE_INDEX] = g_wBlockTableIndex;
#if CMD_DMA
            GLOB_p_BTableChangesDelta = (struct BTableChangesDelta
			  *)GLOB_g_pBTDelta_Free;
            GLOB_g_pBTDelta_Free += sizeof(struct BTableChangesDelta);

            GLOB_p_BTableChangesDelta->GLOB_FTLCommandCount = GLOB_FTLCommandCount;
            GLOB_p_BTableChangesDelta->g_wBlockTableOffset = g_wBlockTableOffset;
            GLOB_p_BTableChangesDelta->g_wBlockTableIndex= g_wBlockTableIndex;
            GLOB_p_BTableChangesDelta->ValidFields = 0x03;

            GLOB_p_BTableChangesDelta = (struct BTableChangesDelta
                  *)GLOB_g_pBTDelta_Free;
            GLOB_g_pBTDelta_Free += sizeof(struct
                  BTableChangesDelta);

            GLOB_p_BTableChangesDelta->GLOB_FTLCommandCount = GLOB_FTLCommandCount;
            GLOB_p_BTableChangesDelta->BT_Index = BLOCK_TABLE_INDEX;
            GLOB_p_BTableChangesDelta->BT_Entry_Value =  pBlockNode[BLOCK_TABLE_INDEX];
            GLOB_p_BTableChangesDelta->ValidFields = 0x0C;
#endif
        }
        wSuccess = FTL_Write_Block_Table_Data();
        if(FAIL == wSuccess)
        {
            MARK_BLOCK_AS_BAD(pBlockNode[BLOCK_TABLE_INDEX]);
        }

    }while(FAIL == wSuccess);

    g_cBlockTableStatus = CURRENT_BLOCK_TABLE;

    return 1;
}


/*&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
* Function:     GLOB_FTL_Flash_Format
* Inputs:       none
* Outputs:      PASS
* Description:  The block table stores bad block info, including MDF+
*               blocks gone bad over the ages. Therefore, if we have a
*               block table in place, then use it to scan for bad blocks
*               If not, then scan for MDF.
*               Now, a block table will only be found if spectra was already
*               being used. For a fresh flash, we'll go thru scanning for
*               MDF. If spectra was being used, then there is a chance that
*               the MDF has been corrupted. Spectra avoids writing to the
*               first 2 bytes of the spare area to all pages in a block. This
*               covers all known flash devices. However, since flash
*               manufacturers have no standard of where the MDF is stored,
*               this cannot guarantee that the MDF is protected for future
*               devices too. The initial scanning for the block table assures
*               this. It is ok even if the block table is outdated, as all
*               we're looking for are bad block markers.
*               Use this when mounting a file system or starting a
*               new flash  .
*&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&*/
static int  FTL_Format_Flash (byte valid_block_table)
{
    BLOCKNODE i,j;
    BLOCKNODE * pBlockNode = (BLOCKNODE *)GLOB_g_pBlockTable;
    BLOCKNODE tempNode;
#if CMD_DMA
    BLOCKNODE * pBlockNodeStartingCopy = (BLOCKNODE *)GLOB_g_pBTStartingCopy;

    if (GLOB_FTLCommandCount)
        return FAIL;
#endif
    if (FAIL == FTL_Check_Block_Table(FAIL))
        valid_block_table = 0;
    if (valid_block_table) {
        byte nodes_switched = 1;
        BLOCKNODE block,k;
        k = GLOB_DeviceInfo.wSpectraStartBlock;
        while((nodes_switched) && (k<GLOB_DeviceInfo.wSpectraEndBlock)) {
            nodes_switched = 0; k++;
            for(j = GLOB_DeviceInfo.wSpectraStartBlock,i=0; j <= GLOB_DeviceInfo.wSpectraEndBlock; j++,i++)
            {
                block = (pBlockNode[i] & ~BAD_BLOCK) - GLOB_DeviceInfo.wSpectraStartBlock;
                if (block != i) {
                    nodes_switched = 1;
                    tempNode = pBlockNode[i];
                    pBlockNode[i] = pBlockNode[block];
                    pBlockNode[block] = tempNode;
                }
            }
        }
        if ((k == GLOB_DeviceInfo.wSpectraEndBlock) && nodes_switched) {
            valid_block_table = 0;
        }
    }
    if (!valid_block_table) {
        memset(GLOB_g_pBlockTable, 0, GLOB_DeviceInfo.wDataBlockNum * sizeof(BLOCKNODE));
        memset(GLOB_g_pWearCounter, 0,GLOB_DeviceInfo.wDataBlockNum * sizeof(byte));
        if(GLOB_DeviceInfo.MLCDevice)
            memset(GLOB_g_pReadCounter, 0,GLOB_DeviceInfo.wDataBlockNum * sizeof(uint16));
#if CMD_DMA
        memset(GLOB_g_pBTStartingCopy, 0, GLOB_DeviceInfo.wDataBlockNum * sizeof(BLOCKNODE));
        memset(GLOB_g_pWearCounterCopy, 0,GLOB_DeviceInfo.wDataBlockNum * sizeof(byte));
        if(GLOB_DeviceInfo.MLCDevice)
            memset(GLOB_g_pReadCounterCopy, 0,GLOB_DeviceInfo.wDataBlockNum * sizeof(uint16));
#endif
#if READ_BADBLOCK_INFO
        for(j = GLOB_DeviceInfo.wSpectraStartBlock,i=0; j <= GLOB_DeviceInfo.wSpectraEndBlock; j++,i++)
        {
            if(GLOB_LLD_Get_Bad_Block((BLOCKNODE)j))
                pBlockNode[i] = (BLOCKNODE)(BAD_BLOCK | j);
        }
#endif
    }

    print(" erasing all the blocks in the flash\n");
    for(j = GLOB_DeviceInfo.wSpectraStartBlock,i=0; j <= GLOB_DeviceInfo.wSpectraEndBlock; j++,i++)
    {
        if ((pBlockNode[i] & BAD_BLOCK) != BAD_BLOCK) {
            if (FAIL == GLOB_LLD_Erase_Block(j
#if CMD_DMA
            ,GLOB_FTLCommandCount,LLD_CMD_FLAG_MODE_POLL
#endif
            )) {
                pBlockNode[i] = (BLOCKNODE)(j);
                MARK_BLOCK_AS_BAD(pBlockNode[i]);
            }
            else
                pBlockNode[i] = (BLOCKNODE)(SPARE_BLOCK | j);
        }
#if CMD_DMA
        pBlockNodeStartingCopy[i] = pBlockNode[i];
#endif
    }

    g_wBlockTableOffset = 0;
    for(i=0; (i <= (GLOB_DeviceInfo.wSpectraEndBlock-GLOB_DeviceInfo.wSpectraStartBlock))
        && ((pBlockNode[i] & BAD_BLOCK) == BAD_BLOCK); i++);
    if (i > (GLOB_DeviceInfo.wSpectraEndBlock-GLOB_DeviceInfo.wSpectraStartBlock)) {
        print("All blocks bad!\n");
        return FAIL;
    }
    else {
        g_wBlockTableIndex = pBlockNode[i] & ~BAD_BLOCK;
        if (i != BLOCK_TABLE_INDEX) {
            tempNode = pBlockNode[i];
            pBlockNode[i] = pBlockNode[BLOCK_TABLE_INDEX];
            pBlockNode[BLOCK_TABLE_INDEX] = tempNode;
        }
    }
    pBlockNode[BLOCK_TABLE_INDEX] &= (~SPARE_BLOCK);
#if CMD_DMA
    pBlockNodeStartingCopy[BLOCK_TABLE_INDEX] &= (~SPARE_BLOCK);
#endif

    g_cBlockTableStatus = IN_PROGRESS_BLOCK_TABLE;
    memset(GLOB_g_pBTBlocks,0xFF,(1+LAST_BT_ID - FIRST_BT_ID) * sizeof(BLOCKNODE));
    GLOB_g_pBTBlocks[FIRST_BT_ID-FIRST_BT_ID]=g_wBlockTableIndex;
    FTL_Write_Block_Table(FAIL);

    for(i = 0; i < CACHE_BLOCK_NUMBER; i++)
    {
        GLOB_Cache.ItemArray[i].dwAddress = INVALID_CACHE_BLK_ADD;
        GLOB_Cache.ItemArray[i].bUCount = 0;
        GLOB_Cache.ItemArray[i].bChanged  = CLEAR;
    }
#if (RESTORE_CACHE_ON_CDMA_CHAIN_FAILURE && CMD_DMA)
    memcpy((void *) &GLOB_Cache_StartingCopy,(void *) &GLOB_Cache, sizeof(FLASH_CACHE));
#endif
    return PASS;
}

int  GLOB_FTL_Flash_Format (void)
{
    return FTL_Format_Flash(1);
}

/*&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
* Function:     FTL_Search_Block_Table_IN_Block
* Inputs:       Block Number
*               Pointer to page
* Outputs:      PASS / FAIL
*               Page contatining the block table
* Description:  It searches the block table in the block
*               passed as an argument.
*
*&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&*/
STATIC int FTL_Search_Block_Table_IN_Block(BLOCKNODE BT_Block, byte BT_Tag, PAGENUMTYPE *Page)
{
    uint16 i, j, k;
    uint16 Result = PASS;
    uint16 Last_IPF =0;
    byte   BT_Found =0;
    byte *tempbuf, *tagarray;
    byte *pSpareBuf;
    byte *pSpareBufBTLastPage;
    byte bt_flag_last_page=0xFF;
    byte search_in_previous_pages =0;
    PAGENUMTYPE wBlockTablePageNum;

    wBlockTablePageNum = FTL_Get_Block_Table_Flash_Size_Pages();

    tempbuf = g_pMemPoolFree;
    g_pMemPoolFree += (GLOB_DeviceInfo.wPageDataSize*sizeof(byte));
    ALIGN_DWORD_FWD(g_pMemPoolFree);
    pSpareBuf = g_pMemPoolFree;
    g_pMemPoolFree +=
        (((GLOB_DeviceInfo.wPageSize-GLOB_DeviceInfo.wPageDataSize)*sizeof(byte)));
    ALIGN_DWORD_FWD(g_pMemPoolFree);
    pSpareBufBTLastPage = g_pMemPoolFree;
    g_pMemPoolFree +=
        (((GLOB_DeviceInfo.wPageSize-GLOB_DeviceInfo.wPageDataSize)*sizeof(byte)));
    ALIGN_DWORD_FWD(g_pMemPoolFree);
    debug_boundary_error(((int)g_pMemPoolFree-(int)g_pMemPool)-1, globalMemSize, 0);

    print("FTL_Search_Block_Table_IN_Block: Searching block table in %u block\n",BT_Block);
    for(i = wBlockTablePageNum ; i < GLOB_DeviceInfo.wPagesPerBlock;
        i +=(wBlockTablePageNum+1))
    {
#if CMD_DMA
        print(" Searching last IPF: %d\n", i);
        Result = GLOB_LLD_Read_Page_Main(tempbuf,BT_Block,i,1,GLOB_FTLCommandCount,LLD_CMD_FLAG_MODE_POLL);
#else
        print(" Searching last IPF: %d\n", i);
        Result = GLOB_LLD_Read_Page_Main(tempbuf,BT_Block,i,1);
#endif
        if(0 == memcmp(tempbuf,g_pIPF,GLOB_DeviceInfo.wPageDataSize)) {
            if((i+wBlockTablePageNum+1)<GLOB_DeviceInfo.wPagesPerBlock)
                continue;
            else
            {
                search_in_previous_pages =1;
                Last_IPF = i;
            }
        }

        if(!search_in_previous_pages)
        {
            if(i != wBlockTablePageNum)
            {
                i -=(wBlockTablePageNum+1);
                Last_IPF = i;
            }
        }

        if(Last_IPF == 0)
            break;

        if(!search_in_previous_pages )
        {
            i=i+1;
            print("Reading the spare area of Block %u Page %u",BT_Block,i);
            Result = GLOB_LLD_Read_Page_Spare(pSpareBuf,BT_Block,i,1);
            print("Reading the spare area of Block %u Page %u",BT_Block,i+wBlockTablePageNum-1);
            Result = GLOB_LLD_Read_Page_Spare(pSpareBufBTLastPage,BT_Block,i+wBlockTablePageNum-1,1);

            k = 0;
            if ((j = FTL_Extract_Block_Table_Tag(pSpareBuf, &tagarray))) {
                for (; k<j; k++) {
                    if (tagarray[k] == BT_Tag)
                        break;
                }
            }
            if (k<j)
                bt_flag = tagarray[k];
            else
                Result = FAIL;

            if(Result == PASS)
            {
                k = 0;
                if ((j = FTL_Extract_Block_Table_Tag(pSpareBufBTLastPage, &tagarray))) {
                    for (; k<j; k++) {
                        if (tagarray[k] == BT_Tag)
                            break;
                    }
                }
                if (k<j)
                    bt_flag_last_page = tagarray[k];
                else
                    Result = FAIL;

                if(Result == PASS)
                {
                    if(bt_flag == bt_flag_last_page)
                    {
                        print(" Block table is found in page after IPF at block %u page\
                        %u\n",(unsigned int)BT_Block,(unsigned int)i);
                        BT_Found =1;
                        *Page  = i;
                        g_cBlockTableStatus = CURRENT_BLOCK_TABLE;
                        break;
                    }
                    else
                        Result = FAIL;
                }
            }
        }


        if(search_in_previous_pages)
            i -=wBlockTablePageNum;
        else
            i -=(wBlockTablePageNum+1);

        Result = PASS;
        print("Reading the spare area of Block %u Page %u",BT_Block,i);
        Result = GLOB_LLD_Read_Page_Spare(pSpareBuf,BT_Block,i,1);
        print("Reading the spare area of Block %u Page %u",BT_Block,i+wBlockTablePageNum-1);
        Result = GLOB_LLD_Read_Page_Spare(pSpareBufBTLastPage,BT_Block,i+wBlockTablePageNum-1,1);

        k = 0;
        if ((j = FTL_Extract_Block_Table_Tag(pSpareBuf, &tagarray))) {
            for (; k<j; k++) {
                if (tagarray[k] == BT_Tag)
                    break;
            }
        }
        if (k<j)
            bt_flag = tagarray[k];
        else
            Result = FAIL;

        if(Result == PASS)
        {
            k = 0;
            if ((j = FTL_Extract_Block_Table_Tag(pSpareBufBTLastPage, &tagarray))) {
                for (; k<j; k++) {
                    if (tagarray[k] == BT_Tag)
                        break;
                }
            }
            if (k<j)
                bt_flag_last_page = tagarray[k];
            else {
                Result = FAIL;
                break;
            }

            if(Result == PASS)
            {
                if(bt_flag == bt_flag_last_page)
                {
                    print(" Block table is found in page prior to IPF at block\
                        %u page %u\n",(unsigned int)BT_Block,(unsigned int)i);
                    BT_Found =1;
                    *Page  = i;
                    g_cBlockTableStatus = IN_PROGRESS_BLOCK_TABLE;
                    break;
                }
                else {
                    Result = FAIL;
                    break;
                }
            }
        }
    }

    if(Result == FAIL)
    {
        if((Last_IPF > wBlockTablePageNum) && (i<Last_IPF) && (BT_Found == 0))
        {
            BT_Found =1;
            *Page = i-(wBlockTablePageNum+1);
        }
        if(Last_IPF == wBlockTablePageNum && (i<Last_IPF) && (BT_Found == 0))
            goto func_return;
    }

    if(Last_IPF == 0)
    {
        i=0;
        Result = PASS;

        print("Reading the spare area of Block %u Page %u",BT_Block,i);
        Result = GLOB_LLD_Read_Page_Spare(pSpareBuf,BT_Block,i,1);
        print("Reading the spare area of Block %u Page %u",BT_Block,i+wBlockTablePageNum-1);
        Result = GLOB_LLD_Read_Page_Spare(pSpareBufBTLastPage,BT_Block,i+wBlockTablePageNum-1,1);

        k = 0;
        if ((j = FTL_Extract_Block_Table_Tag(pSpareBuf, &tagarray))) {
            for (; k<j; k++) {
                if (tagarray[k] == BT_Tag)
                    break;
            }
        }
        if (k<j)
            bt_flag = tagarray[k];
        else
            Result = FAIL;

    if(Result == PASS)
    {
        k = 0;
        if ((j = FTL_Extract_Block_Table_Tag(pSpareBufBTLastPage, &tagarray))) {
            for (; k<j; k++) {
                if (tagarray[k] == BT_Tag)
                    break;
            }
        }
        if (k<j)
            bt_flag_last_page = tagarray[k];
        else {
            Result = FAIL;
        }

        if(Result == PASS)
        {
            if(bt_flag == bt_flag_last_page)
            {
                print(" Block table is found in page after IPF at block %u page\
                    %u\n",(unsigned int)BT_Block,(unsigned int)i);
                BT_Found =1;
                *Page  = i;
                g_cBlockTableStatus = CURRENT_BLOCK_TABLE;
                goto func_return;
            }
            else
                Result = FAIL;
        }
    }

    if(Result == FAIL)
        goto func_return;

    }

func_return:
    g_pMemPoolFree -=
      ((GLOB_DeviceInfo.wPageSize-GLOB_DeviceInfo.wPageDataSize)*sizeof(byte));
    ALIGN_DWORD_BWD(g_pMemPoolFree);
    g_pMemPoolFree -=
      ((GLOB_DeviceInfo.wPageSize-GLOB_DeviceInfo.wPageDataSize)*sizeof(byte));
    ALIGN_DWORD_BWD(g_pMemPoolFree);
    g_pMemPoolFree -= ((GLOB_DeviceInfo.wPageDataSize*sizeof(byte)));
    ALIGN_DWORD_BWD(g_pMemPoolFree);
    return Result;
}


/*&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
* Function:     FTL_Read_Block_Table
* Inputs:       none
* Outputs:      PASS / FAIL
* Description:  read the flash spare area and find a block containing the
*               most recent block table(having largest block_table_counter).
*               Find the last written Block table in this block.
*               Check the correctness of Block Table
*               If CDMA is enabled, this function is called in
*               polling mode.
*               We don't need to store changes in Block table in this
*               function as it is called only at initialization
*
*               Note: Currently this function is called at initialization
*               before any read/erase/write command issued to flash so,
*               there is no need to wait for CDMA list to complete as of now
*&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&*/
STATIC int FTL_Read_Block_Table(void)
{
    int k;
    uint16 i;
    int j;
    byte *tempBuf, *tagarray;
    int wResult = FAIL;
    int status = FAIL;
    byte block_table_found =0;
    int search_result;
    BLOCKNODE Block;
    PAGENUMTYPE Page;
    PAGENUMTYPE PageCount;
    PAGENUMTYPE wBlockTablePageNum;
    int wBytesCopied = 0, tempvar;

    wBlockTablePageNum = FTL_Get_Block_Table_Flash_Size_Pages();

    tempBuf = g_pMemPoolFree;
    g_pMemPoolFree +=
        (GLOB_DeviceInfo.wPageDataSize*sizeof(byte));
    ALIGN_DWORD_FWD(g_pMemPoolFree);
    debug_boundary_error(((int)g_pMemPoolFree-(int)g_pMemPool)-1, globalMemSize, 0);

    print(" FTL_READ_BLOCK_TABLE function called \n");
    for (j = GLOB_DeviceInfo.wSpectraStartBlock; j <= (int)GLOB_DeviceInfo.wSpectraEndBlock; j++)
    {
        status = GLOB_LLD_Read_Page_Spare(tempBuf,j,0,1);
        k = 0;
        if ((i = FTL_Extract_Block_Table_Tag(tempBuf, &tagarray)))
        {
#if CMD_DMA
            status = GLOB_LLD_Read_Page_Main(tempBuf,
                    j,0,1,GLOB_FTLCommandCount,LLD_CMD_FLAG_MODE_POLL);
#else
            status  = GLOB_LLD_Read_Page_Main(tempBuf,
                    j,0,1);
#endif
            if(status == FAIL)
            {
                print("FTL_Read_Block_Table: Error in Reading Block Table!!!\n");
            }

            for (; k<i; k++) {
                if (tagarray[k] == tempBuf[3])
                    break;
            }
        }
        if (k<i)
            k = tagarray[k];
        else
            continue;

        print("Block table is contained in Block %d %d\n",(unsigned int)j,(unsigned int)k);

        if(GLOB_g_pBTBlocks[k-FIRST_BT_ID] == BTBLOCK_INVAL)
        {
            GLOB_g_pBTBlocks[k-FIRST_BT_ID]=j;
            block_table_found =1;
        }
        else
        {
            print("FTL_Read_Block_Table: This should never happens. Two block table having same counter %u!!!\n",k);
        }
    }
    g_pMemPoolFree -=
        (GLOB_DeviceInfo.wPageDataSize*sizeof(byte));
    ALIGN_DWORD_BWD(g_pMemPoolFree);

    if(block_table_found)
    {

        if(GLOB_g_pBTBlocks[FIRST_BT_ID-FIRST_BT_ID] != BTBLOCK_INVAL && GLOB_g_pBTBlocks[LAST_BT_ID-FIRST_BT_ID] != BTBLOCK_INVAL)
        {
            j = LAST_BT_ID;
            while((GLOB_g_pBTBlocks[j-FIRST_BT_ID]!=BTBLOCK_INVAL) && (j>FIRST_BT_ID))
                j--;
            if(j==FIRST_BT_ID) {
                j = LAST_BT_ID;
                last_erased = LAST_BT_ID;
            }
            else {
                last_erased = (byte)j+1;
                while((GLOB_g_pBTBlocks[j-FIRST_BT_ID]==BTBLOCK_INVAL) && (j>FIRST_BT_ID))
                    j--;
            }
        }
        else {
            j=FIRST_BT_ID;
            while(GLOB_g_pBTBlocks[j-FIRST_BT_ID] == BTBLOCK_INVAL)
                j++;
            last_erased = (byte)j;
            while((GLOB_g_pBTBlocks[j-FIRST_BT_ID] != BTBLOCK_INVAL) && (j<LAST_BT_ID))
                j++;
            if (GLOB_g_pBTBlocks[j-FIRST_BT_ID] == BTBLOCK_INVAL) j--;
        }

        if (last_erased > j)
            j += (1+LAST_BT_ID-FIRST_BT_ID);
        for (; (j>=last_erased) && (FAIL==wResult); j--) {
            i = ((j-FIRST_BT_ID) % (1+LAST_BT_ID-FIRST_BT_ID));
            search_result = FTL_Search_Block_Table_IN_Block(GLOB_g_pBTBlocks[i], i+FIRST_BT_ID, &Page);
            if (g_cBlockTableStatus == IN_PROGRESS_BLOCK_TABLE)
                block_table_found = 0;
            while ((search_result == PASS) && (FAIL == wResult)) {
                print("\n\n FTL_Read_Block_Table: Block: %u Page: %u contains block\
                    table\n",(unsigned int)GLOB_g_pBTBlocks[i],(unsigned int)Page);
                tempBuf = g_pMemPoolFree;
                g_pMemPoolFree +=
                    (GLOB_DeviceInfo.wPageDataSize * sizeof(byte));
                ALIGN_DWORD_FWD(g_pMemPoolFree);
                debug_boundary_error(((int)g_pMemPoolFree-(int)g_pMemPool)-1, globalMemSize, 0);

                for(k = 0; k < wBlockTablePageNum ; k++)
                {
                    Block     = GLOB_g_pBTBlocks[i];
                    PageCount = 1;
    #if CMD_DMA
                    status  = GLOB_LLD_Read_Page_Main(tempBuf,
                           Block,Page,PageCount,GLOB_FTLCommandCount,LLD_CMD_FLAG_MODE_POLL);
    #else
                    status  = GLOB_LLD_Read_Page_Main(tempBuf,
                                    Block,Page,PageCount);
    #endif
                    tempvar = k ? 0 : 4;
                    wBytesCopied += FTL_Copy_Block_Table_From_Flash(tempBuf + tempvar, GLOB_DeviceInfo.wPageDataSize - tempvar, wBytesCopied);
                    Page++;
                }
                g_pMemPoolFree -=
                  (GLOB_DeviceInfo.wPageDataSize * sizeof(byte));
                ALIGN_DWORD_BWD(g_pMemPoolFree);

                wResult = FTL_Check_Block_Table(FAIL);
                if (FAIL == wResult) {
                    block_table_found = 0;
                    if (Page > wBlockTablePageNum) {
                        Page -=((wBlockTablePageNum<<1)+1);
                    }
                    else
                        search_result = FAIL;
                }
            }
        }
    }

    if(PASS == wResult) {
        if (!block_table_found)
            FTL_Execute_SPL_Recovery();
        if (g_cBlockTableStatus == IN_PROGRESS_BLOCK_TABLE)
            g_wBlockTableOffset = (PAGENUMTYPE)Page+1;
        else
            g_wBlockTableOffset = (PAGENUMTYPE)Page-wBlockTablePageNum;
        g_wBlockTableIndex = (BLOCKNODE)GLOB_g_pBTBlocks[i];

#if CMD_DMA
        if(GLOB_DeviceInfo.MLCDevice)
        {
            memcpy(GLOB_g_pBTStartingCopy,GLOB_g_pBlockTable,(GLOB_DeviceInfo.wDataBlockNum  * sizeof(BLOCKNODE)+
                GLOB_DeviceInfo.wDataBlockNum * sizeof(byte)+
                GLOB_DeviceInfo.wDataBlockNum * sizeof(uint16)));
        }
        else
        {
            memcpy(GLOB_g_pBTStartingCopy,GLOB_g_pBlockTable,(GLOB_DeviceInfo.wDataBlockNum  * sizeof(BLOCKNODE)+
                GLOB_DeviceInfo.wDataBlockNum * sizeof(byte)));
        }
#endif
    }
#if CMD_DMA
    GLOB_LLD_Flash_Init(LLD_CMD_FLAG_MODE_CDMA);
#endif
#if AUTO_FORMAT_FLASH
    if(FAIL == wResult) {
        print("doing auto-format\n");
        wResult = FTL_Format_Flash(0);
    }
#endif

    return wResult;
}

/*&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
* Function:     FTL_Flash_Error_Handle
* Inputs:       Pointer to data
*               Page address
*               Block address
* Outputs:      PASS=0 / FAIL=1
* Description:  It handles any error occured during Spectra operation
*&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&*/
STATIC int FTL_Flash_Error_Handle(byte * pData, ADDRESSTYPE dwOldPageAddr, ADDRESSTYPE dwBlockAddr)
{
    BLOCKNODE  i;
    int j;
    BLOCKNODE wTempNode, wBlockNode=BLK_FROM_ADDR(dwBlockAddr);
    ADDRESSTYPE physical_addr;
    int wErase  = FAIL;
    int wResult = FAIL;
    BLOCKNODE * pBlockNode = (BLOCKNODE *)GLOB_g_pBlockTable;

    if(GLOB_SYSTEM_FATAL_ERROR())
        return ERR;
    if(ERR == GLOB_FTL_Garbage_Collection())
        return ERR;
    do
    {
        for(i = (GLOB_DeviceInfo.wSpectraEndBlock - GLOB_DeviceInfo.wSpectraStartBlock) ; i > 0; i--)
        {
            if(IS_SPARE_BLOCK(i))
            {
                wTempNode = (BLOCKNODE)(BAD_BLOCK | pBlockNode[wBlockNode]);

                pBlockNode[wBlockNode] = (BLOCKNODE)(pBlockNode[i]&(~SPARE_BLOCK));
                pBlockNode[i] = wTempNode;
#if CMD_DMA
                GLOB_p_BTableChangesDelta = (struct BTableChangesDelta
                    *)GLOB_g_pBTDelta_Free;
                GLOB_g_pBTDelta_Free += sizeof(struct
                    BTableChangesDelta);

                GLOB_p_BTableChangesDelta->GLOB_FTLCommandCount = GLOB_FTLCommandCount;
                GLOB_p_BTableChangesDelta->BT_Index = wBlockNode;
                GLOB_p_BTableChangesDelta->BT_Entry_Value =  pBlockNode[wBlockNode];
                GLOB_p_BTableChangesDelta->ValidFields = 0x0C;

                GLOB_p_BTableChangesDelta = (struct
                    BTableChangesDelta*)GLOB_g_pBTDelta_Free;
                GLOB_g_pBTDelta_Free += sizeof(struct
                    BTableChangesDelta);

                GLOB_p_BTableChangesDelta->GLOB_FTLCommandCount = GLOB_FTLCommandCount;
                GLOB_p_BTableChangesDelta->BT_Index = i;
                GLOB_p_BTableChangesDelta->BT_Entry_Value =  pBlockNode[i];
                GLOB_p_BTableChangesDelta->ValidFields = 0x0C;
#endif
                wResult = PASS;
                break;
            }
        }
        if(FAIL == wResult)
        {
            if(FAIL == GLOB_FTL_Garbage_Collection())
                break;
            else
                continue;
        }
        if(IN_PROGRESS_BLOCK_TABLE != g_cBlockTableStatus)
        {
            g_cBlockTableStatus = IN_PROGRESS_BLOCK_TABLE;
            FTL_Write_IN_Progress_Block_Table_Page();

        }
        physical_addr = FTL_Get_Physical_Block_Addr(dwBlockAddr);
        for(j = 0; j < RETRY_TIMES; j++)
        {
            if(PASS == wErase)
            {
                if( FAIL == GLOB_FTL_Block_Erase(physical_addr))
                {
                    MARK_BLOCK_AS_BAD(pBlockNode[wBlockNode]);
                    break;

                }
            }
            if(PASS == FTL_Cache_Update_Block(pData, dwOldPageAddr, physical_addr))
            {
                wResult = PASS;
                break;
            }
            else
            {
                wResult = FAIL;
                wErase = PASS;
            }
        }
    }while(FAIL == wResult);
    FTL_Write_Block_Table(FAIL);
    return wResult;
}

/*&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
* Function:     FTL_Get_Page_Num
* Inputs:       Size in bytes
* Outputs:      Size in pages
* Description:  It calculates the pages required for the length passed
*&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&*/
STATIC uint32 FTL_Get_Page_Num(ADDRESSTYPE length)
{
    return (uint32)((length >> GLOB_DeviceInfo.nBitsInPageDataSize) +
        (GLOB_u64_Remainder(length , 1)  > 0 ? 1 : 0));
}


/*&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&& ff
* Function:     FTL_Get_Physical_Block_Addr
* Inputs:       Block Address (byte format)
* Outputs:      Physical address of the block. f
* Description:  It translates LBA to PBA by returning address stored
*               at the LBA location in the block table
*&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&*/
STATIC ADDRESSTYPE FTL_Get_Physical_Block_Addr(ADDRESSTYPE dwBlockAddr)
{
    BLOCKNODE * pBlockNode = (BLOCKNODE *)GLOB_g_pBlockTable;
    ADDRESSTYPE physical_addr = (ADDRESSTYPE)GLOB_DeviceInfo.wBlockDataSize
        * (pBlockNode[BLK_FROM_ADDR(dwBlockAddr)]&(~BAD_BLOCK));
    return physical_addr;
}

/*&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
* Function:     FTL_Get_Block_Index
* Inputs:       Physical Block no.
* Outputs:      Logical block no. /BAD_BLOCK
* Description:  It returns the logical block no. for the PBA passed
*&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&*/
STATIC BLOCKNODE FTL_Get_Block_Index(BLOCKNODE wBlockNum)
{
    BLOCKNODE * pBlockNode = (BLOCKNODE *)GLOB_g_pBlockTable;
    BLOCKNODE i;
    for(i = 0; i < GLOB_DeviceInfo.wDataBlockNum; i++)
        if(wBlockNum == (pBlockNode[i]&(~BAD_BLOCK)))
            return i;
    return BAD_BLOCK;

}

/*&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
* Function:     GLOB_FTL_Wear_Leveling
* Inputs:       none
* Outputs:      PASS=0
* Description:  This is static wear leveling (done by explicit call)
*               do complete static wear leveling
*               do complete garbage collection
*&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&*/
int GLOB_FTL_Wear_Leveling (void)
{
    FTL_Static_Wear_Leveling ();

    GLOB_FTL_Garbage_Collection ();

    return PASS;
}


/*&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
* Function:     FTL_Static_Wear_Leveling
* Inputs:       none
* Outputs:      PASS=0 / FAIL=1
* Description:  This is static wear leveling (done by explicit call)
*               search for most&least used
*               if difference < GATE:
*                   update the block table with exhange
*                   mark block table in flash as IN_PROGRESS
*                   copy flash block
*               the caller should handle GC clean up after calling this function
*&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&*/
int FTL_Static_Wear_Leveling (void)
{
    byte wMostWornCounter;
    byte wLeastWornCounter;
    byte wWearCounter;
    BLOCKNODE wLeastWornIdx;
    BLOCKNODE wMostWornIdx;
    BLOCKNODE i, j, wWearIndex;
    BLOCKNODE wReplacedBlock;
    int wResult = PASS;
    int wContinue = PASS;
    BLOCKNODE wReplacedBlockNum = 0;
    BLOCKNODE * pBlockNode = (BLOCKNODE *)GLOB_g_pBlockTable;
    byte * pszChangedFlag;

    print("     starting static wear leveling \n");

    pszChangedFlag = g_pMemPoolFree;
    g_pMemPoolFree += (GLOB_DeviceInfo.wDataBlockNum);
    ALIGN_DWORD_FWD(g_pMemPoolFree);
    debug_boundary_error(((int)g_pMemPoolFree-(int)g_pMemPool)-1, globalMemSize, 0);

    if(pszChangedFlag)
    {
        memset(pszChangedFlag, FAIL,GLOB_DeviceInfo.wDataBlockNum);
        while(PASS == wContinue)
        {
            print("     starting static wear levleing pass  \n");
            wMostWornCounter= 0;
            wLeastWornCounter = 0xFF;
            wLeastWornIdx = BLOCK_TABLE_INDEX;
            wMostWornIdx = BLOCK_TABLE_INDEX;

            for(i = BLOCK_TABLE_INDEX+1; i < GLOB_DeviceInfo.wDataBlockNum; i++)
            {
                if(IS_BAD_BLOCK(i) || PASS == pszChangedFlag[i])
                    continue;

                wWearIndex = (BLOCKNODE)((~BAD_BLOCK)&pBlockNode[i]);
                wWearCounter = GLOB_g_pWearCounter[wWearIndex -
                    GLOB_DeviceInfo.wSpectraStartBlock];

                if(IS_SPARE_BLOCK(i))
                {
                    if(wWearCounter> wMostWornCounter)
                    {
                        wMostWornCounter = wWearCounter;
                        wMostWornIdx = wWearIndex;
                    }
                }
                if(IS_DATA_BLOCK(i))
                {
                    if(wWearCounter < wLeastWornCounter)
                    {
                        wLeastWornCounter = wWearCounter;
                        wLeastWornIdx = wWearIndex;
                    }
                }
                if(PASS == pszChangedFlag[wMostWornIdx] || PASS ==
                    pszChangedFlag[wLeastWornIdx]) {
                  debug_boundary_error(wMostWornIdx, GLOB_DeviceInfo.wDataBlockNum, 0);
                  debug_boundary_error(wLeastWornIdx, GLOB_DeviceInfo.wDataBlockNum, 0);
                  continue;
                }
            }


            print ("     Used and least worn is block %u whos count is %u\n",
                    (unsigned int)wLeastWornIdx,(unsigned int)wLeastWornCounter);
            print ("     Free and  most worn is block %u whos count is %u\n",
            (unsigned int)wMostWornIdx,(unsigned int)wMostWornCounter);

            wContinue = (((wMostWornCounter > wLeastWornCounter)
                    && (wMostWornCounter - wLeastWornCounter > WEAR_LEVELING_GATE)) ? PASS :
                    FAIL);
            if(PASS == wContinue)
            {
                print("     count difference is GT GATE so do it \n");
                pszChangedFlag[wLeastWornIdx] = PASS;
                debug_boundary_error(wLeastWornIdx, GLOB_DeviceInfo.wDataBlockNum, 0);
                wReplacedBlock = FTL_Replace_MWBlock();
                if(BAD_BLOCK != wReplacedBlock)
                {
                    print("     more than two spare blocks exist so do it \n");
                    print(" Block Replaced is %u \n",(unsigned int)wReplacedBlock );
                    pszChangedFlag[wReplacedBlock] = PASS;
                    debug_boundary_error(wReplacedBlock, GLOB_DeviceInfo.wDataBlockNum, 0);

                    if(IN_PROGRESS_BLOCK_TABLE != g_cBlockTableStatus)
                    {
                        g_cBlockTableStatus = IN_PROGRESS_BLOCK_TABLE;
                        FTL_Write_IN_Progress_Block_Table_Page();

                    }

                    for(j = 0; j < RETRY_TIMES; j++)
                    {
                        if(FAIL == FTL_Copy_Block((ADDRESSTYPE)wLeastWornIdx*GLOB_DeviceInfo.wBlockDataSize,
                                (ADDRESSTYPE)wReplacedBlock*GLOB_DeviceInfo.wBlockDataSize))
                        {
                            if( FAIL == GLOB_FTL_Block_Erase((ADDRESSTYPE)wReplacedBlock*GLOB_DeviceInfo.wBlockDataSize))
                            {
                                MARK_BLOCK_AS_BAD(pBlockNode[wReplacedBlock]);
                            }
                        }
                        else
                        {
                            print("     FTL_Copy_Block == OK \n");
                            break;
                        }
                    }
                    if( j < RETRY_TIMES)
                    {
                        BLOCKNODE wTempNode;
                        BLOCKNODE wOldIndex = FTL_Get_Block_Index(wLeastWornIdx);
                        BLOCKNODE wReplacedIndex = FTL_Get_Block_Index(wReplacedBlock);
                        wTempNode = (BLOCKNODE)(DISCARD_BLOCK |
                            pBlockNode[wOldIndex]);

                        pBlockNode[wOldIndex] = (BLOCKNODE)((~SPARE_BLOCK) & pBlockNode[wReplacedIndex]);
                        pBlockNode[wReplacedIndex] = wTempNode;
#if CMD_DMA
                        GLOB_p_BTableChangesDelta = (struct BTableChangesDelta
                            *)GLOB_g_pBTDelta_Free;
                        GLOB_g_pBTDelta_Free += sizeof(struct
                            BTableChangesDelta);
                        GLOB_p_BTableChangesDelta->GLOB_FTLCommandCount = GLOB_FTLCommandCount;
                        GLOB_p_BTableChangesDelta->BT_Index = wOldIndex;
                        GLOB_p_BTableChangesDelta->BT_Entry_Value =
                            pBlockNode[wOldIndex];
                        GLOB_p_BTableChangesDelta->ValidFields = 0x0C;


                        GLOB_p_BTableChangesDelta = (struct BTableChangesDelta
                            *)GLOB_g_pBTDelta_Free;
                        GLOB_g_pBTDelta_Free += sizeof(struct
                            BTableChangesDelta);

                        GLOB_p_BTableChangesDelta->GLOB_FTLCommandCount = GLOB_FTLCommandCount;
                        GLOB_p_BTableChangesDelta->BT_Index = wReplacedIndex;
                        GLOB_p_BTableChangesDelta->BT_Entry_Value =
                            pBlockNode[wReplacedIndex];
                        GLOB_p_BTableChangesDelta->ValidFields = 0x0C;
#endif
                    }
                    else
                    {
                        pBlockNode[FTL_Get_Block_Index(wReplacedBlock)] |=
                          BAD_BLOCK;
#if CMD_DMA
                        GLOB_p_BTableChangesDelta = (struct BTableChangesDelta
                            *)GLOB_g_pBTDelta_Free;
                        GLOB_g_pBTDelta_Free += sizeof(struct
                            BTableChangesDelta);

                        GLOB_p_BTableChangesDelta->GLOB_FTLCommandCount = GLOB_FTLCommandCount;
                        GLOB_p_BTableChangesDelta->BT_Index = FTL_Get_Block_Index(wReplacedBlock);
                        GLOB_p_BTableChangesDelta->BT_Entry_Value =
                            pBlockNode[FTL_Get_Block_Index(wReplacedBlock)];
                        GLOB_p_BTableChangesDelta->ValidFields = 0x0C;
#endif
                        wResult = FAIL;
                        wContinue = FAIL;
                    }
                    if((wReplacedBlockNum++) > WEAR_LEVELING_BLOCK_NUM)
                        wContinue = FAIL;
                }
                else
                {
                    print("     less than 3 spare blocks exist so quit \n");
                    wContinue = FAIL;
                }

            }

        }

        g_pMemPoolFree -= (GLOB_DeviceInfo.wDataBlockNum);
        ALIGN_DWORD_BWD(g_pMemPoolFree);
    }
    return wResult;

}



/*&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
* Function:     GLOB_FTL_Garbage_Collection
* Inputs:       none
* Outputs:      PASS / FAIL (If Erase fails on any discarded Block)
* Description:  search the block table for all discarded blocks to erase
*               for each discarded block:
*                   set the flash block to IN_PROGRESS
*                   erase the block
*                   update the block table
*                   write the block table to flash
*&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&*/
int GLOB_FTL_Garbage_Collection ()
{
    BLOCKNODE i;
    BLOCKNODE wDiscard = 0;
    int wResult  = PASS;
    BLOCKNODE pba;
    uint32 btarray_counter;

    BLOCKNODE * pBlockNode = (BLOCKNODE *)GLOB_g_pBlockTable;
    byte  bt_block_erased =0;

#if _DEBUG_
    dbgGCCalled = 1;
#endif

    if(GC_Called_Flag  == 1)
        return PASS;

    GC_Called_Flag  = 1;

    GLOB_FTL_BT_Garbage_Collection();

    for(i = 0;i < GLOB_DeviceInfo.wDataBlockNum; i++)
    {
        if(IS_DISCARDED_BLOCK(i))
            wDiscard++;
    }

    print(" GARBAGE COLLECTION IS CALLED \n");

    if(wDiscard > 0)
    {
        print(" DISCARDED BLOCKS FOUND %u\n",(unsigned int)wDiscard);

        FTL_Write_Block_Table(FAIL);

        i = 0;
#if CMD_DMA
        while(i < GLOB_DeviceInfo.wDataBlockNum && wDiscard > 0 && (GLOB_FTLCommandCount+28) <
		  256 /*40*/)
#else
        while(i < GLOB_DeviceInfo.wDataBlockNum && wDiscard > 0)
#endif
        {
            if(BAD_BLOCK != (pBlockNode[i] & BAD_BLOCK) && 0 != (pBlockNode[i] &
                DISCARD_BLOCK))
            {
                if(IN_PROGRESS_BLOCK_TABLE != g_cBlockTableStatus)
                {
                    g_cBlockTableStatus = IN_PROGRESS_BLOCK_TABLE;
                    FTL_Write_IN_Progress_Block_Table_Page();
                }

                pba = BLK_FROM_ADDR(FTL_Get_Physical_Block_Addr((ADDRESSTYPE)i*GLOB_DeviceInfo.wBlockDataSize));
                for(btarray_counter=FIRST_BT_ID;btarray_counter<=LAST_BT_ID;btarray_counter++)
                {

                    if(pba == GLOB_g_pBTBlocks[btarray_counter-FIRST_BT_ID])
                    {
                        print(" GC tries to erase BT Block %u\n",pba);
                        wDiscard--;
                        i++;
                        bt_block_erased =1;
                        break;
                    }

                }

                if(bt_block_erased)
                {
                    bt_block_erased =0;
                    continue;
                }
                if(PASS == GLOB_FTL_Block_Erase(FTL_Get_Physical_Block_Addr((ADDRESSTYPE)i*GLOB_DeviceInfo.wBlockDataSize)))
                {
                    pBlockNode[i] &= (BLOCKNODE)~DISCARD_BLOCK;
                    pBlockNode[i] |= (BLOCKNODE)SPARE_BLOCK;
#if CMD_DMA
                    GLOB_p_BTableChangesDelta = (struct BTableChangesDelta
					  *)GLOB_g_pBTDelta_Free;
                    GLOB_g_pBTDelta_Free += sizeof(struct
					  BTableChangesDelta);


                    GLOB_p_BTableChangesDelta->GLOB_FTLCommandCount = GLOB_FTLCommandCount - 1;
                    GLOB_p_BTableChangesDelta->BT_Index = i;
                    GLOB_p_BTableChangesDelta->BT_Entry_Value = pBlockNode[i];

                    GLOB_p_BTableChangesDelta->ValidFields = 0x0C;
#endif
                    wDiscard--;
                }
                else
                {
                    MARK_BLOCK_AS_BAD(pBlockNode[i]);
                    wResult = FAIL;
                }
            }
            i++;
        }
        FTL_Write_Block_Table(FAIL);
    }
    GC_Called_Flag  = 0;

    return wResult;
}

/*&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
* Function:     GLOB_FTL_BT_Garbage_Collection
* Inputs:       none
* Outputs:      PASS / FAIL (returns the number of un-erased blocks
* Description:  Erases discarded blocks containing Block table.
*               There is no need to call Write_Block_Table here because it only gets called from
*               - GC, where the block table gets updated,
*               - Write block table!
*
*&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&*/
int GLOB_FTL_BT_Garbage_Collection(void)
{
    BLOCKNODE i;
    BLOCKNODE pba,lba;
    int wResult  = FAIL;

    BLOCKNODE * pBlockNode = (BLOCKNODE *)GLOB_g_pBlockTable;
    BLOCKNODE * pBTBlocksNode = (BLOCKNODE *)GLOB_g_pBTBlocks;

    if(BT_GC_Called_Flag  == 1)
        return PASS;

    BT_GC_Called_Flag  = 1;


    print(" BT GARBAGE COLLECTION IS CALLED \n");
#if CMD_DMA
    for(i=last_erased; (i <=LAST_BT_ID) && (GLOB_g_pBTBlocks[((i+2) % (1+LAST_BT_ID-FIRST_BT_ID)) + FIRST_BT_ID-FIRST_BT_ID]!= BTBLOCK_INVAL) &&
   (( GLOB_FTLCommandCount+28)) < 256;i++)
#else
    for(i=last_erased; i<=LAST_BT_ID && GLOB_g_pBTBlocks[((i+2) % (1+LAST_BT_ID-FIRST_BT_ID)) + FIRST_BT_ID-FIRST_BT_ID]!= BTBLOCK_INVAL ;i++)
#endif
    {
        pba = pBTBlocksNode[i-FIRST_BT_ID];
        lba = FTL_Get_Block_Index(pba);

        if(BAD_BLOCK != (pBlockNode[lba] & BAD_BLOCK) && 0 != (pBlockNode[lba] &
            DISCARD_BLOCK))
        {
            print("GLOB_FTL_BT_Garbage_Collection: Erasing Block tables present in %u Block\n",pba);
            if(PASS == GLOB_FTL_Block_Erase(FTL_Get_Physical_Block_Addr((ADDRESSTYPE)lba*GLOB_DeviceInfo.wBlockDataSize)))
            {
                pBlockNode[lba] &= (BLOCKNODE)~DISCARD_BLOCK;
                pBlockNode[lba] |= (BLOCKNODE)SPARE_BLOCK;
#if CMD_DMA
                GLOB_p_BTableChangesDelta = (struct BTableChangesDelta
                    *)GLOB_g_pBTDelta_Free;
                GLOB_g_pBTDelta_Free += sizeof(struct
                    BTableChangesDelta);


                GLOB_p_BTableChangesDelta->GLOB_FTLCommandCount = GLOB_FTLCommandCount - 1;
                GLOB_p_BTableChangesDelta->BT_Index = lba;
                GLOB_p_BTableChangesDelta->BT_Entry_Value = pBlockNode[lba];
                GLOB_p_BTableChangesDelta->ValidFields = 0x0C;
#endif
                wResult = PASS;
                pBTBlocksNode[last_erased-FIRST_BT_ID] = BTBLOCK_INVAL;
                print(" resetting bt entry at index %u value %u\n",i,pBTBlocksNode[i-FIRST_BT_ID]);
                if(last_erased == LAST_BT_ID)
                    last_erased = FIRST_BT_ID;
                else
                    last_erased++;
            }
            else
            {
                MARK_BLOCK_AS_BAD(pBlockNode[lba]);
            }
        }
    }
    BT_GC_Called_Flag  = 0;

    return wResult;
}

/*&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
* Function:     FTL_Replace_OneBlock
* Inputs:       Block number 1
*               Block number 2
* Outputs:      Replaced Block Number
* Description:  Interchange block table entries at wBlockNum and wReplaceNum
*
*&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&*/
STATIC BLOCKNODE FTL_Replace_OneBlock(BLOCKNODE wBlockNum, BLOCKNODE wReplaceNum)
{
    BLOCKNODE wTempNode;
    BLOCKNODE wReplacedNode = BAD_BLOCK;
    BLOCKNODE * pBlockNode = (BLOCKNODE *)GLOB_g_pBlockTable;

    if(wReplaceNum != BAD_BLOCK)
    {
        if(IS_BAD_BLOCK(wBlockNum))
            wTempNode = (BLOCKNODE)(pBlockNode[wBlockNum]);
        else
            wTempNode = (BLOCKNODE)(DISCARD_BLOCK | (~SPARE_BLOCK & pBlockNode[wBlockNum]));

        wReplacedNode = (BLOCKNODE)((~SPARE_BLOCK) & pBlockNode[wReplaceNum]);

        pBlockNode[wBlockNum] = wReplacedNode;
        pBlockNode[wReplaceNum] = wTempNode;
        if((pBlockNode[wBlockNum] & (~BAD_BLOCK))> GLOB_DeviceInfo.wDataBlockNum)
        print("Error Occured in FTL_Replace_OneBlock: (pBlockNode[wBlockNum] %d\n",(pBlockNode[wBlockNum] & (~BAD_BLOCK)));

        if((pBlockNode[wReplaceNum] & (~BAD_BLOCK))> GLOB_DeviceInfo.wDataBlockNum)
        print("Error Occured in FTL_Replace_OneBlock: (pBlockNode[wReplaceNum] %d\n",(pBlockNode[wReplaceNum] & (~BAD_BLOCK)));

#if CMD_DMA
        GLOB_p_BTableChangesDelta = (struct BTableChangesDelta *)GLOB_g_pBTDelta_Free;
        GLOB_g_pBTDelta_Free += sizeof(struct BTableChangesDelta);

        GLOB_p_BTableChangesDelta->GLOB_FTLCommandCount = GLOB_FTLCommandCount;
        GLOB_p_BTableChangesDelta->BT_Index = wBlockNum;
        GLOB_p_BTableChangesDelta->BT_Entry_Value = pBlockNode[wBlockNum];

        GLOB_p_BTableChangesDelta->ValidFields = 0x0C;

        GLOB_p_BTableChangesDelta = (struct BTableChangesDelta *)GLOB_g_pBTDelta_Free;
        GLOB_g_pBTDelta_Free += sizeof(struct BTableChangesDelta);

        GLOB_p_BTableChangesDelta->GLOB_FTLCommandCount = GLOB_FTLCommandCount;
        GLOB_p_BTableChangesDelta->BT_Index = wReplaceNum;
        GLOB_p_BTableChangesDelta->BT_Entry_Value = pBlockNode[wReplaceNum];
        GLOB_p_BTableChangesDelta->ValidFields = 0x0C;
#endif
    }
    return wReplacedNode;
}

/*&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
* Function:     FTL_Write_Block_Table_Data
* Inputs:       Block table size in pages
* Outputs:      PASS=0 / FAIL=1
* Description:  Write block table data in flash
*               If first page and last page
*                  Write data+BT flag
*               else
*                  Write data
*               BT flag is a counter. Its value is incremented for block table
*               write in a new Block
*&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&*/
STATIC int FTL_Write_Block_Table_Data(void)
{
    ADDRESSTYPE dwBlockTableAddr = (ADDRESSTYPE)((ADDRESSTYPE)g_wBlockTableIndex*GLOB_DeviceInfo.wBlockDataSize +
        (ADDRESSTYPE)g_wBlockTableOffset * GLOB_DeviceInfo.wPageDataSize);
    ADDRESSTYPE pTempAddr = dwBlockTableAddr;
    int iSuccess = PASS;
    BLOCKNODE Block;
    PAGENUMTYPE Page;
    PAGENUMTYPE PageCount;
    byte * tempBuf;
    int wBytesCopied;
    PAGENUMTYPE wBlockTablePageNum;
    byte bCacheBlockNum;

    wBlockTablePageNum = FTL_Get_Block_Table_Flash_Size_Pages();

    print("FTL_Write_Block_Table_Data: page= %u BlockTableIndex= %d BlockTableOffset=%d\n",(unsigned int)wBlockTablePageNum,g_wBlockTableIndex,g_wBlockTableOffset);

    Block     = BLK_FROM_ADDR(pTempAddr);
    Page      = PAGE_FROM_ADDR(pTempAddr, Block);
    PageCount = 1;

    if(bt_block_changed)
    {
        if(bt_flag == LAST_BT_ID)
        {
            bt_flag = FIRST_BT_ID;
            GLOB_g_pBTBlocks[bt_flag-FIRST_BT_ID]=Block;
        }
        else if(bt_flag < LAST_BT_ID)
        {
            bt_flag++;
            GLOB_g_pBTBlocks[bt_flag-FIRST_BT_ID]=Block;
        }
        if((bt_flag > (LAST_BT_ID-4)) && GLOB_g_pBTBlocks[FIRST_BT_ID-FIRST_BT_ID]!= BTBLOCK_INVAL)
        {
            bt_block_changed = 0;
            GLOB_FTL_BT_Garbage_Collection();
        }

        bt_block_changed = 0;
        print("Block Table Counter is %u Block %u\n",bt_flag,Block);
    }


    tempBuf = g_pMemPoolFree;
    g_pMemPoolFree += (wBlockTablePageNum > 3)
        ? (FTL_Get_Block_Table_Flash_Size_Bytes() - (GLOB_DeviceInfo.wPageSize<<1)) : GLOB_DeviceInfo.wPageSize;
    ALIGN_DWORD_FWD(g_pMemPoolFree);
    debug_boundary_error(((int)g_pMemPoolFree-(int)g_pMemPool)-1, globalMemSize, 0);


    memset(tempBuf, 0, 3);
    tempBuf[3] = bt_flag;
    wBytesCopied = FTL_Copy_Block_Table_To_Flash(tempBuf+4, GLOB_DeviceInfo.wPageDataSize-4, 0);
    memset(&tempBuf[wBytesCopied+4], 0xff, GLOB_DeviceInfo.wPageSize-(wBytesCopied+4));
    FTL_Insert_Block_Table_Signature(&tempBuf[GLOB_DeviceInfo.wPageDataSize], bt_flag);
#if CMD_DMA
    memcpy(GLOB_g_pNextBlockTable,tempBuf,GLOB_DeviceInfo.wPageSize *
                    sizeof(byte));
    print(" Writing First Page of Block Table Block %u Page %u\n",Block,Page);
    if(FAIL ==
        GLOB_LLD_Write_Page_Main_Spare(GLOB_g_pNextBlockTable,Block,Page,1,GLOB_FTLCommandCount,LLD_CMD_FLAG_MODE_CDMA|LLD_CMD_FLAG_ORDER_BEFORE_REST))
    {
        iSuccess = FAIL;
        goto func_return;
    }
    GLOB_FTLCommandCount++;
    GLOB_g_pNextBlockTable += ((GLOB_DeviceInfo.wPageSize * sizeof(byte)));
#else
    if(FAIL == GLOB_LLD_Write_Page_Main_Spare(tempBuf,Block,Page,1))
    {
        iSuccess = FAIL;
        goto func_return;
    }
#endif


    if(wBlockTablePageNum >1)
    {
        PageCount = wBlockTablePageNum -1;
        if(PageCount >1)
        {
            wBytesCopied += FTL_Copy_Block_Table_To_Flash(tempBuf, GLOB_DeviceInfo.wPageDataSize * (PageCount-1), wBytesCopied);

#if CMD_DMA
            memcpy(GLOB_g_pNextBlockTable,tempBuf,(PageCount-1)*GLOB_DeviceInfo.wPageDataSize);

            if(FAIL ==
                    GLOB_LLD_Write_Page_Main(GLOB_g_pNextBlockTable,Block,Page+1,PageCount-1,GLOB_FTLCommandCount))
            {
                iSuccess = FAIL;
                goto func_return;
            }
            GLOB_FTLCommandCount++;
            GLOB_g_pNextBlockTable += (((PageCount-1)*GLOB_DeviceInfo.wPageDataSize *
                sizeof(byte)));
#else
            if(FAIL == GLOB_LLD_Write_Page_Main(tempBuf,Block,Page+1,PageCount-1))
            {
                iSuccess = FAIL;
                goto func_return;
            }
#endif
        }

        wBytesCopied = FTL_Copy_Block_Table_To_Flash(tempBuf, GLOB_DeviceInfo.wPageDataSize, wBytesCopied);
        memset(&tempBuf[wBytesCopied], 0xff, GLOB_DeviceInfo.wPageSize-wBytesCopied);
        FTL_Insert_Block_Table_Signature(&tempBuf[GLOB_DeviceInfo.wPageDataSize], bt_flag);

#if CMD_DMA
        memcpy(GLOB_g_pNextBlockTable,tempBuf,GLOB_DeviceInfo.wPageSize *
                           sizeof(byte));
        print(" Writing the last Page of Block Table Block %u Page %u \n",Block,Page+wBlockTablePageNum-1);
        if(FAIL ==
            GLOB_LLD_Write_Page_Main_Spare(GLOB_g_pNextBlockTable,Block,Page+wBlockTablePageNum-1,1,GLOB_FTLCommandCount,LLD_CMD_FLAG_MODE_CDMA|LLD_CMD_FLAG_ORDER_BEFORE_REST))
        {
            iSuccess = FAIL;
            goto func_return;
        }
        GLOB_FTLCommandCount++;
#else
        if(FAIL == GLOB_LLD_Write_Page_Main_Spare(tempBuf,Block,Page+wBlockTablePageNum-1,1))
        {
            iSuccess = FAIL;
            goto func_return;
        }
#endif
    }

    print("FTL_Write_Block_Table_Data: done\n");

func_return:
    g_pMemPoolFree -= (wBlockTablePageNum > 3)
        ? (FTL_Get_Block_Table_Flash_Size_Bytes() - (GLOB_DeviceInfo.wPageSize<<1)) : GLOB_DeviceInfo.wPageSize;
    ALIGN_DWORD_BWD(g_pMemPoolFree);

    return iSuccess;
}

/*&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
* Function:     FTL_Replace_Block_Table
* Inputs:       None
* Outputs:      PASS=0 / FAIL=1
* Description:  Get a new block to write block table
*&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&*/
STATIC BLOCKNODE FTL_Replace_Block_Table(void)
{
    BLOCKNODE wReplacedBlock;
    int GarbageCollect;

    print("FTL_Replace_Block_Table\n");
    wReplacedBlock = FTL_Replace_LWBlock(BLOCK_TABLE_INDEX, &GarbageCollect);

    if((BAD_BLOCK == wReplacedBlock) && (PASS == GarbageCollect))
    {
	GLOB_FTL_Garbage_Collection();
	wReplacedBlock = FTL_Replace_LWBlock(BLOCK_TABLE_INDEX, &GarbageCollect);
    }
    if(BAD_BLOCK == wReplacedBlock)
    {
        print("FTL_Replace_Block_Table: There is no spare block. It should never happen\n");
    }

    print(" New Block table Block is %u\n",(unsigned int)wReplacedBlock );

    return wReplacedBlock;
}



/*&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
* Function:     FTL_Replace_LWBlock
* Inputs:       Block number
*               Pointer to Garbage Collect flag
* Outputs:
* Description:  Determine the least weared block by traversing
*               block table
*               Set Garbage collection to be called if number of spare
*               block is less than Free Block Gate count
*               Change Block table entry to map least worn block for current
*               operation
*&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&*/
STATIC BLOCKNODE FTL_Replace_LWBlock(BLOCKNODE wBlockNum, int * pGarbageCollect)
{
    BLOCKNODE i;
    BLOCKNODE * pBlockNode = (BLOCKNODE *)GLOB_g_pBlockTable;
    byte wLeastWornCounter = 0xFF;
    BLOCKNODE wLeastWornIndex = BAD_BLOCK;
    BLOCKNODE wSpareBlockNum = 0;

    BLOCKNODE wDiscardBlockNum = 0;

    if(IS_SPARE_BLOCK(wBlockNum))
    {
        *pGarbageCollect = FAIL;

        pBlockNode[wBlockNum] = (BLOCKNODE)((~SPARE_BLOCK & pBlockNode[wBlockNum]));

#if CMD_DMA
    GLOB_p_BTableChangesDelta = (struct BTableChangesDelta *)GLOB_g_pBTDelta_Free;
    GLOB_g_pBTDelta_Free += sizeof(struct BTableChangesDelta);

    GLOB_p_BTableChangesDelta->GLOB_FTLCommandCount = GLOB_FTLCommandCount;
    GLOB_p_BTableChangesDelta->BT_Index = (BLOCKNODE)(wBlockNum);
    GLOB_p_BTableChangesDelta->BT_Entry_Value = pBlockNode[wBlockNum];
    GLOB_p_BTableChangesDelta->ValidFields = 0x0C ;
#endif
        return pBlockNode[wBlockNum];
     }

    for(i = 0; i < GLOB_DeviceInfo.wDataBlockNum; i++)
    {
        if(IS_DISCARDED_BLOCK(i))
            wDiscardBlockNum++;

        if(IS_SPARE_BLOCK(i))
        {
            BLOCKNODE wPhysicalIndex = (BLOCKNODE)((~BAD_BLOCK) & pBlockNode[i]);
            if(wPhysicalIndex > GLOB_DeviceInfo.wSpectraEndBlock)
                print("FTL_Replace_LWBlock:This should never occur\n");

            if(GLOB_g_pWearCounter[wPhysicalIndex - GLOB_DeviceInfo.wSpectraStartBlock] <
                wLeastWornCounter)
            {
                wLeastWornCounter = GLOB_g_pWearCounter[wPhysicalIndex -
                    GLOB_DeviceInfo.wSpectraStartBlock];
                wLeastWornIndex = i;
            }
            wSpareBlockNum++;
        }
    }

    if((wDiscardBlockNum >= NUM_FREE_BLOCKS_GATE) || (wSpareBlockNum <=
        NUM_FREE_BLOCKS_GATE))
    {
        *pGarbageCollect = PASS;
    }
    else
        *pGarbageCollect = FAIL;

    print(" FTL_Replace_LWBlock: Discarded Blocks %u Spare Blocks %u\n",wDiscardBlockNum,wSpareBlockNum);
    return FTL_Replace_OneBlock(wBlockNum, wLeastWornIndex);
}

/*&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
* Function:     FTL_Replace_MWBlock
* Inputs:       None
* Outputs:      most worn spare block no./BAD_BLOCK
* Description:  It finds most worn spare block.
*&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&*/
STATIC BLOCKNODE FTL_Replace_MWBlock(void)
{
    BLOCKNODE i;
    BLOCKNODE * pBlockNode = (BLOCKNODE *)GLOB_g_pBlockTable;
    byte wMostWornCounter = 0;
    BLOCKNODE wMostWornIndex = BAD_BLOCK;
    BLOCKNODE wSpareBlockNum = 0;

    for(i = 0; i < GLOB_DeviceInfo.wDataBlockNum; i++)
    {
        if(IS_SPARE_BLOCK(i))
        {
            BLOCKNODE wPhysicalIndex = (BLOCKNODE)((~SPARE_BLOCK)& pBlockNode[i]);
            if(GLOB_g_pWearCounter[wPhysicalIndex - GLOB_DeviceInfo.wSpectraStartBlock] > wMostWornCounter)
            {
                wMostWornCounter = GLOB_g_pWearCounter[wPhysicalIndex - GLOB_DeviceInfo.wSpectraStartBlock];
                wMostWornIndex = wPhysicalIndex;
            }
            wSpareBlockNum++;
        }
    }
    if(wSpareBlockNum <= 2)
        return BAD_BLOCK;

    return wMostWornIndex;

}


/*&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
* Function:     FTL_Replace_Block
* Inputs:       Block Address
* Outputs:      PASS=0 / FAIL=1
* Description:  If block specified by dwBlockAddr parameter is not free,
*               replace it with the least worn block.
*&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&*/
STATIC int FTL_Replace_Block(ADDRESSTYPE dwBlockAddr)
{
    BLOCKNODE wCurrentBlockNum = BLK_FROM_ADDR(dwBlockAddr);
    BLOCKNODE * pBlockNode = (BLOCKNODE *)GLOB_g_pBlockTable;
    int wResult = PASS;
    int GarbageCollect = FAIL;

    if(IS_SPARE_BLOCK(wCurrentBlockNum))
    {
        pBlockNode[wCurrentBlockNum] = (BLOCKNODE)((~SPARE_BLOCK & pBlockNode[wCurrentBlockNum]));

#if CMD_DMA
        GLOB_p_BTableChangesDelta = (struct BTableChangesDelta *)GLOB_g_pBTDelta_Free;
        GLOB_g_pBTDelta_Free += sizeof(struct BTableChangesDelta);

        GLOB_p_BTableChangesDelta->GLOB_FTLCommandCount = GLOB_FTLCommandCount;
        GLOB_p_BTableChangesDelta->BT_Index = (BLOCKNODE)(wCurrentBlockNum);
        GLOB_p_BTableChangesDelta->BT_Entry_Value = pBlockNode[wCurrentBlockNum];
        GLOB_p_BTableChangesDelta->ValidFields = 0x0C ;
#endif

        return wResult;
    }

    FTL_Replace_LWBlock(wCurrentBlockNum, &GarbageCollect);

    if(PASS == GarbageCollect)
    {
        wResult = GLOB_FTL_Garbage_Collection();
    }

    return wResult;
}


/*&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
* Function:     GLOB_FTL_Is_BadBlock
* Inputs:       block number to test
* Outputs:      PASS (block is BAD) / FAIL (block is not bad)
* Description:  test if this block number is flagged as bad
*&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&*/
int GLOB_FTL_Is_BadBlock (BLOCKNODE wBlockNum)
{
    BLOCKNODE * pBlockNode = (BLOCKNODE *)GLOB_g_pBlockTable;

    if(wBlockNum >= GLOB_DeviceInfo.wSpectraStartBlock && BAD_BLOCK == (pBlockNode[wBlockNum] & BAD_BLOCK))
        return PASS;
    else
        return FAIL;

}

/*&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
* Function:     GLOB_FTL_Flush_Cache
* Inputs:       none
* Outputs:      0=Nothing to be done. Cache was clean.
*               1=Data flushed. Requires an execute CMD to complete execution.
*               -1 (ERR)= Error
* Description:  flush all the cache blocks to flash
*               if a cache block is not dirty, don't do anything with it
*               else, write the block and update the block table
* Note:         This function should be called at shutdown/power down.
*               to write important data into device
*&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&*/
int GLOB_FTL_Flush_Cache(void)
{
    int i, retval = 0;
    if(GLOB_SYSTEM_FATAL_ERROR())
        return ERR;

    for(i = 0; i < CACHE_BLOCK_NUMBER; i++)
    {
        if(SET == GLOB_Cache.ItemArray[i].bChanged)
        {
#if CMD_DMA
#if RESTORE_CACHE_ON_CDMA_CHAIN_FAILURE
            GLOB_Int_Cache[GLOB_FTLCommandCount].nUsedCacheItem = i;
            GLOB_Int_Cache[GLOB_FTLCommandCount].IntCache.dwAddress =
                GLOB_Cache.ItemArray[i].dwAddress;

            GLOB_Int_Cache[GLOB_FTLCommandCount].IntCache.bChanged = CLEAR;
#endif
#endif

            if(ERR != FTL_Cache_Write_Back(GLOB_Cache.ItemArray[i].pContent,
                GLOB_Cache.ItemArray[i].dwAddress))
            {
                GLOB_Cache.ItemArray[i].bChanged = CLEAR;
            }
            else
                return ERR;

            retval = 1;
        }
    }

    if (FTL_Write_Block_Table(FAIL) == 1)
        retval = 1;

    return retval;
}

/*&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
* Function:     GLOB_FTL_Page_Read
* Inputs:       pointer to data
*               address of data (ADDRESSTYPE is LBA * Bytes/Page)
* Outputs:      PASS=0 / FAIL=1
* Description:  reads a page of data into RAM from the cache
*               if the data is not already in cache, read from flash to cache
*&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&*/
int GLOB_FTL_Page_Read(byte *pData, ADDRESSTYPE dwPageAddr)
{
    byte bCacheBlockNum;
    int wResult = PASS;

    print(" GLOB_FTL_Page_Read  %lX = %lX\n",dwPageAddr,(dwPageAddr >> 9));

#if CMD_DMA
    g_SBDCmdIndex++;
#endif


    if(UNHIT_BLOCK == (bCacheBlockNum = FTL_Cache_If_Hit(dwPageAddr)))
    {
        print(" GLOB_FTL_Page_Read: Cache not hit\n");
        wResult = FTL_Cache_Write();
        if(ERR == FTL_Cache_Read(dwPageAddr))
            wResult = ERR;
        bCacheBlockNum = GLOB_Cache.bLFU;
    }
    FTL_Cache_Read_Page(pData, dwPageAddr, bCacheBlockNum);
    return wResult;
}

/*&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
* Function:     GLOB_FTL_Page_Write
* Inputs:       pointer to data
*               address of data (ADDRESSTYPE is LBA * Bytes/Page)
* Outputs:      PASS=0 / FAIL=1
* Description:  writes a page of data from RAM to the cache
*               if the data is not already in cache, write back the
*               least frequently used block and read the addressed block
*               from flash to cache
*&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&*/
int GLOB_FTL_Page_Write(byte *pData, ADDRESSTYPE dwPageAddr)
{
    byte bCacheBlockNum;
    int wResult = PASS;

    BLOCKNODE * pBlockNode = (BLOCKNODE *)GLOB_g_pBlockTable;

    print("  GLOB_FTL_Page_Write %lX = %lX\n",dwPageAddr,(dwPageAddr>> 9));

#if CMD_DMA
    g_SBDCmdIndex++;
#endif

    if(UNHIT_BLOCK == (bCacheBlockNum = FTL_Cache_If_Hit(dwPageAddr)))
    {
        wResult = FTL_Cache_Write();

        if (IS_BAD_BLOCK(BLK_FROM_ADDR(dwPageAddr)))
        {
            wResult = FTL_Replace_Block(dwPageAddr);

            pBlockNode[BLK_FROM_ADDR(dwPageAddr)] |= SPARE_BLOCK;
            if (wResult == FAIL)
               return FAIL;
        }

        if(ERR == FTL_Cache_Read(dwPageAddr))
            wResult = ERR;
        bCacheBlockNum = GLOB_Cache.bLFU;


        FTL_Cache_Write_Page(pData, dwPageAddr, bCacheBlockNum,0);

    }
    else
    {
#if CMD_DMA
        FTL_Cache_Write_Page(pData, dwPageAddr, bCacheBlockNum,LLD_CMD_FLAG_ORDER_BEFORE_REST);
#else
        FTL_Cache_Write_Page(pData, dwPageAddr, bCacheBlockNum,0);
#endif
    }
    return wResult;
}

/*&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
* Function:     GLOB_FTL_Block_Erase
* Inputs:       address of block to erase (now in byte format, should change to block format)
* Outputs:      PASS=0 / FAIL=1
* Description:  erases the specified block
*               increments the erase count
*               If erase count reaches its upper limit,call function to
*               do the ajustment as per the relative erase count values
*&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&*/
int GLOB_FTL_Block_Erase(ADDRESSTYPE dwBlockAddr)
{
    int     status;
    BLOCKNODE  BlkIdx;
    //byte bCacheBlockNum = UNHIT_BLOCK;

    BlkIdx = (BLOCKNODE)((dwBlockAddr >> GLOB_DeviceInfo.nBitsInBlockDataSize));

    if(BlkIdx < GLOB_DeviceInfo.wSpectraStartBlock)
        print("GLOB_FTL_Block_Erase:This should never occur \n");
#if CMD_DMA
    status = GLOB_LLD_Erase_Block(BlkIdx,GLOB_FTLCommandCount,LLD_CMD_FLAG_MODE_CDMA);
#else
    status = GLOB_LLD_Erase_Block(BlkIdx);

    if (status == FAIL)
        return status;

#endif

    if(GLOB_DeviceInfo.MLCDevice)
    {
        GLOB_g_pReadCounter[BlkIdx - GLOB_DeviceInfo.wSpectraStartBlock] = 0;
        if(IN_PROGRESS_BLOCK_TABLE != g_cBlockTableStatus)
        {
            g_cBlockTableStatus = IN_PROGRESS_BLOCK_TABLE;
            FTL_Write_IN_Progress_Block_Table_Page();
        }
    }

    GLOB_g_pWearCounter[BlkIdx - GLOB_DeviceInfo.wSpectraStartBlock]++;

#if CMD_DMA
    GLOB_p_BTableChangesDelta = (struct BTableChangesDelta *)GLOB_g_pBTDelta_Free;
    GLOB_g_pBTDelta_Free += sizeof(struct BTableChangesDelta);
    GLOB_p_BTableChangesDelta->GLOB_FTLCommandCount = GLOB_FTLCommandCount;
    GLOB_p_BTableChangesDelta->WC_Index =BlkIdx - GLOB_DeviceInfo.wSpectraStartBlock ;
    GLOB_p_BTableChangesDelta->WC_Entry_Value = GLOB_g_pWearCounter[BlkIdx -
            GLOB_DeviceInfo.wSpectraStartBlock];
    GLOB_p_BTableChangesDelta->ValidFields = 0x30;
    if(GLOB_DeviceInfo.MLCDevice)
    {
        GLOB_p_BTableChangesDelta = (struct BTableChangesDelta *)GLOB_g_pBTDelta_Free;
        GLOB_g_pBTDelta_Free += sizeof(struct BTableChangesDelta);
        GLOB_p_BTableChangesDelta->GLOB_FTLCommandCount = GLOB_FTLCommandCount;
        GLOB_p_BTableChangesDelta->RC_Index = BlkIdx - GLOB_DeviceInfo.wSpectraStartBlock;
        GLOB_p_BTableChangesDelta->RC_Entry_Value = GLOB_g_pReadCounter[BlkIdx - GLOB_DeviceInfo.wSpectraStartBlock];
        GLOB_p_BTableChangesDelta->ValidFields = 0xC0;

    }
    GLOB_FTLCommandCount++;
#endif

    if(GLOB_g_pWearCounter[BlkIdx - GLOB_DeviceInfo.wSpectraStartBlock] == 0xFE)
        FTL_Adjust_Relative_Erase_Count(BlkIdx);
    return status;
}



/*&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
* Function:     FTL_Adjust_Relative_Erase_Count
* Inputs:       index to block that was just incremented and is at the max
* Outputs:      PASS=0 / FAIL=1
* Description:  If any erase counts at MAX, adjusts erase count of every
*               block by substracting least worn
*               counter from counter value of every entry in wear table
*&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&*/
STATIC int FTL_Adjust_Relative_Erase_Count(BLOCKNODE Index_of_MAX)
{
    byte wLeastWornCounter= MAX_BYTE_VALUE;
    byte wWearCounter;
    BLOCKNODE i,wWearIndex;

    BLOCKNODE * pBlockNode = (BLOCKNODE *)GLOB_g_pBlockTable;
    int wResult = PASS;

    print(" Adjusting Wear Levelling Counters \n");
    for(i = 0; i < GLOB_DeviceInfo.wDataBlockNum; i++)
    {
        if(IS_BAD_BLOCK(i))
            continue;
        wWearIndex = (BLOCKNODE)(pBlockNode[i]&(~BAD_BLOCK));

        if((wWearIndex - GLOB_DeviceInfo.wSpectraStartBlock) < 0)
            print("FTL_Adjust_Relative_Erase_Count:This should never occur\n");

        wWearCounter = GLOB_g_pWearCounter[wWearIndex - GLOB_DeviceInfo.wSpectraStartBlock];
        if(wWearCounter < wLeastWornCounter)
            wLeastWornCounter = wWearCounter;
    }

    if (wLeastWornCounter == 0)
    {
        print(" Adjusting Wear Levelling Counters: Special Case \n");

        GLOB_g_pWearCounter[Index_of_MAX - GLOB_DeviceInfo.wSpectraStartBlock]--;

#if CMD_DMA
        GLOB_p_BTableChangesDelta = (struct BTableChangesDelta *)GLOB_g_pBTDelta_Free;
        GLOB_g_pBTDelta_Free += sizeof(struct BTableChangesDelta);
        GLOB_p_BTableChangesDelta->GLOB_FTLCommandCount = GLOB_FTLCommandCount;
        GLOB_p_BTableChangesDelta->WC_Index = Index_of_MAX - GLOB_DeviceInfo.wSpectraStartBlock;
        GLOB_p_BTableChangesDelta->WC_Entry_Value = GLOB_g_pWearCounter[Index_of_MAX -
            GLOB_DeviceInfo.wSpectraStartBlock];
        GLOB_p_BTableChangesDelta->ValidFields = 0x30;
#endif
        FTL_Static_Wear_Leveling();
    }
    else
    {

        for(i = 0; i < GLOB_DeviceInfo.wDataBlockNum; i++)
            if(!IS_BAD_BLOCK(i))
            {
                wWearIndex = (BLOCKNODE)(pBlockNode[i]&(~BAD_BLOCK));

                GLOB_g_pWearCounter[wWearIndex - GLOB_DeviceInfo.wSpectraStartBlock] = (byte)(GLOB_g_pWearCounter[wWearIndex - GLOB_DeviceInfo.wSpectraStartBlock] -
                  wLeastWornCounter);

#if CMD_DMA
                GLOB_p_BTableChangesDelta = (struct BTableChangesDelta
                    *)GLOB_g_pBTDelta_Free;
                GLOB_g_pBTDelta_Free += sizeof(struct
                    BTableChangesDelta);

                GLOB_p_BTableChangesDelta->GLOB_FTLCommandCount = GLOB_FTLCommandCount;
                GLOB_p_BTableChangesDelta->WC_Index = wWearIndex - GLOB_DeviceInfo.wSpectraStartBlock;
                GLOB_p_BTableChangesDelta->WC_Entry_Value = GLOB_g_pWearCounter[wWearIndex -
                    GLOB_DeviceInfo.wSpectraStartBlock];
                GLOB_p_BTableChangesDelta->ValidFields = 0x30;
#endif
            }
    }
    return wResult;
}


/*&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
* Function:     FTL_Write_IN_Progress_Block_Table_Page
* Inputs:       None
* Outputs:      None
* Description:  It writes in-progress flag page to the page next to
*               block table
***********************************************************************/
STATIC int FTL_Write_IN_Progress_Block_Table_Page(void)
{
    int  wResult=PASS;
    PAGENUMTYPE wBlockTablePageNum;
    PAGENUMTYPE dwIPFPageAddr;

    wBlockTablePageNum = FTL_Get_Block_Table_Flash_Size_Pages();

    dwIPFPageAddr = g_wBlockTableOffset+wBlockTablePageNum;

    print(" Writing IPF at Block %u Page %u\n",(unsigned
        int)g_wBlockTableIndex,(unsigned int)dwIPFPageAddr);
#if CMD_DMA
    wResult = GLOB_LLD_Write_Page_Main_Spare(g_pIPF,g_wBlockTableIndex,dwIPFPageAddr,1,GLOB_FTLCommandCount,LLD_CMD_FLAG_MODE_CDMA|LLD_CMD_FLAG_ORDER_BEFORE_REST);

    g_wBlockTableOffset = dwIPFPageAddr+1;

    GLOB_p_BTableChangesDelta = (struct BTableChangesDelta *)GLOB_g_pBTDelta_Free;
    GLOB_g_pBTDelta_Free += sizeof(struct BTableChangesDelta);
    GLOB_p_BTableChangesDelta->GLOB_FTLCommandCount = GLOB_FTLCommandCount;
    GLOB_p_BTableChangesDelta->g_wBlockTableOffset = g_wBlockTableOffset;
    GLOB_p_BTableChangesDelta->ValidFields =0x01;

    GLOB_FTLCommandCount++;
#else
        wResult = GLOB_LLD_Write_Page_Main_Spare(g_pIPF,g_wBlockTableIndex,dwIPFPageAddr,1);
        if (wResult == FAIL)
        {
            BLOCKNODE * pBlockNode = (BLOCKNODE *)GLOB_g_pBlockTable;
            BLOCKNODE   wTempBlockTableIndex;
            MARK_BLOCK_AS_BAD(pBlockNode[BLOCK_TABLE_INDEX]);

            wTempBlockTableIndex = FTL_Replace_Block_Table();
            bt_block_changed = 1;
            if(BAD_BLOCK == wTempBlockTableIndex)
                return ERR;
            g_wBlockTableIndex = wTempBlockTableIndex;
            g_wBlockTableOffset = 0;
            pBlockNode[BLOCK_TABLE_INDEX] = g_wBlockTableIndex;

            return FAIL;
        }

    g_wBlockTableOffset = dwIPFPageAddr+1;
#endif
    return wResult;
}

/*&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
* Function:     FTL_Read_Disturbance
* Inputs:       block address
* Outputs:      PASS=0 / FAIL=1
* Description:  used to handle read disturbance. Data in block that
*               reaches its read limit is moved to new block
*&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&*/
int FTL_Read_Disturbance(BLOCKNODE dwBlockAddr)
{
    int wResult = FAIL;
    BLOCKNODE * pBlockNode = (BLOCKNODE *)GLOB_g_pBlockTable;
    BLOCKNODE dwOldBlockAddr = dwBlockAddr;
    BLOCKNODE wBlockNum;
    BLOCKNODE i;
    BLOCKNODE wLeastReadCounter = 0xFFFF;
    BLOCKNODE wLeastReadIndex = BAD_BLOCK;
    BLOCKNODE wSpareBlockNum = 0;
    BLOCKNODE wTempNode;
    BLOCKNODE wReplacedNode;
    byte *g_pTempBuf;
#if CMD_DMA
    byte bCacheBlockNum;
    g_pTempBuf = (byte *)GLOB_g_pCopyBackBufferStart;
    GLOB_g_pCopyBackBufferStart += (GLOB_DeviceInfo.wPageDataSize *GLOB_DeviceInfo.wPagesPerBlock *sizeof(byte));
#else
    g_pTempBuf = (byte *)g_pMemPoolFree;
    g_pMemPoolFree += (GLOB_DeviceInfo.wPageDataSize *GLOB_DeviceInfo.wPagesPerBlock *sizeof(byte));
    ALIGN_DWORD_FWD(g_pMemPoolFree);
    debug_boundary_error(((int)g_pMemPoolFree-(int)g_pMemPool)-1, globalMemSize, 0);
#endif

    wBlockNum = FTL_Get_Block_Index(dwBlockAddr);

    print("FTL_READ_DISTURBANCE Called\n");
    do
    {
        for(i = 1; i < GLOB_DeviceInfo.wDataBlockNum; i++)
        {
            if(IS_SPARE_BLOCK(i))
            {
                BLOCKNODE wPhysicalIndex = (BLOCKNODE)((~SPARE_BLOCK) &
                    pBlockNode[i]);
                if(GLOB_g_pReadCounter[wPhysicalIndex - GLOB_DeviceInfo.wSpectraStartBlock] <
                        wLeastReadCounter)
                {
                    wLeastReadCounter = GLOB_g_pReadCounter[wPhysicalIndex -
                            GLOB_DeviceInfo.wSpectraStartBlock];
                    wLeastReadIndex = i;
                }
                wSpareBlockNum++;
            }
        }

        if(wSpareBlockNum <= NUM_FREE_BLOCKS_GATE)
        {
            wResult = GLOB_FTL_Garbage_Collection();
            if(PASS == wResult)
                continue;
            else
                break;
        }
        else
        {

            wTempNode = (BLOCKNODE)(DISCARD_BLOCK | pBlockNode[wBlockNum]);
            wReplacedNode = (BLOCKNODE)((~SPARE_BLOCK) &
                pBlockNode[wLeastReadIndex]);

#if CMD_DMA
            pBlockNode[wBlockNum] = wReplacedNode;
            pBlockNode[wLeastReadIndex] = wTempNode;

            GLOB_p_BTableChangesDelta = (struct BTableChangesDelta *)GLOB_g_pBTDelta_Free;
            GLOB_g_pBTDelta_Free += sizeof(struct BTableChangesDelta);

            GLOB_p_BTableChangesDelta->GLOB_FTLCommandCount = GLOB_FTLCommandCount;
            GLOB_p_BTableChangesDelta->BT_Index = wBlockNum;
            GLOB_p_BTableChangesDelta->BT_Entry_Value = pBlockNode[wBlockNum];
            GLOB_p_BTableChangesDelta->ValidFields = 0x0C;

            GLOB_p_BTableChangesDelta = (struct BTableChangesDelta *)GLOB_g_pBTDelta_Free;
            GLOB_g_pBTDelta_Free += sizeof(struct BTableChangesDelta);

            GLOB_p_BTableChangesDelta->GLOB_FTLCommandCount = GLOB_FTLCommandCount;
            GLOB_p_BTableChangesDelta->BT_Index = wLeastReadIndex;
            GLOB_p_BTableChangesDelta->BT_Entry_Value = pBlockNode[wLeastReadIndex];
            GLOB_p_BTableChangesDelta->ValidFields = 0x0C;

            wResult = GLOB_LLD_Read_Page_Main(g_pTempBuf,dwOldBlockAddr,0,GLOB_DeviceInfo.wPagesPerBlock,GLOB_FTLCommandCount,LLD_CMD_FLAG_MODE_CDMA);
            if (wResult == FAIL)
            {
                return wResult;
            }

            GLOB_FTLCommandCount++;
            if(wResult !=FAIL)
            {
                if(FAIL == GLOB_LLD_Write_Page_Main(g_pTempBuf,
                        pBlockNode[wBlockNum],0,GLOB_DeviceInfo.wPagesPerBlock,GLOB_FTLCommandCount))
                {
                    wResult = FAIL;
                    MARK_BLOCK_AS_BAD(pBlockNode[wBlockNum]);
                }
                GLOB_FTLCommandCount++;
            }
#else
            wResult = GLOB_LLD_Read_Page_Main(g_pTempBuf,dwOldBlockAddr,0,GLOB_DeviceInfo.wPagesPerBlock);
            if (wResult == FAIL)
            {
                g_pMemPoolFree -= (GLOB_DeviceInfo.wPageDataSize *GLOB_DeviceInfo.wPagesPerBlock *sizeof(byte));
                ALIGN_DWORD_BWD(g_pMemPoolFree);

                return wResult;
            }
            if(wResult !=FAIL)
            {

                wResult = GLOB_LLD_Write_Page_Main(g_pTempBuf,
                        wReplacedNode,0,GLOB_DeviceInfo.wPagesPerBlock);

                if( wResult == FAIL)
                {
                    MARK_BLOCK_AS_BAD(wReplacedNode);
                }
                else
                {
                    pBlockNode[wBlockNum] = wReplacedNode;
                    pBlockNode[wLeastReadIndex] = wTempNode;
                }
            }
            if((wResult == PASS) && (IN_PROGRESS_BLOCK_TABLE != g_cBlockTableStatus))
            {
                g_cBlockTableStatus = IN_PROGRESS_BLOCK_TABLE;
                FTL_Write_IN_Progress_Block_Table_Page();
            }
#endif
        }
    }while(PASS != wResult);


#if CMD_DMA
#else
    g_pMemPoolFree -= (GLOB_DeviceInfo.wPageDataSize *GLOB_DeviceInfo.wPagesPerBlock *sizeof(byte));
    ALIGN_DWORD_BWD(g_pMemPoolFree);
#endif
    return wResult;
}

#ifdef __cplusplus
}
#endif
