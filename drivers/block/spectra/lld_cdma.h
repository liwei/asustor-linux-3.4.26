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


// header for LLD_CDMA.c module

#ifndef _LLD_CDMA_
#define _LLD_CDMA_

#include "flash.h"

/////////////   CDMA specific MACRO definition
#define MAX_DESCRIPTORS         (255)
#define LLD_NUM_FLASH_CHANNELS  (4)
#define MAX_SYNC_POINTS         (16)

#define CHANNEL_SYNC_MASK       (0x000F)
#define CHANNEL_DMA_MASK        (0x00F0)
#define CHANNEL_ID_MASK         (0x0300)
#define CHANNEL_CONT_MASK       (0x4000)
#define CHANNEL_INTR_MASK       (0x8000)

#define CHANNEL_SYNC_OFFSET     (0)
#define CHANNEL_DMA_OFFSET      (4)
#define CHANNEL_ID_OFFSET       (8)
#define CHANNEL_CONT_OFFSET     (14)
#define CHANNEL_INTR_OFFSET     (15)

#if CMD_DMA
uint16  CDMA_Data_CMD (byte tag, byte CMD, byte* data, BLOCKNODE block, uint16 page, uint16 count, uint16 flags);
uint16  CDMA_MemCopy_CMD (byte tag, byte* dest, byte* src, uint16 ByteCount, uint16 flags);
uint16  CDMA_Execute_CMDs (uint16 tag_count);
void    CDMA_AddSyncPoints (uint16 tag_count);
void    CDMA_CheckSyncPoints (uint16 tag_count);
void    PrintGLOB_PendingCMDs(uint16 tag_count);
void    PrintGLOB_PendingCMDsPerChannel(uint16 tag_count);
void    PrintCDMA_Descriptors(void);
uint32 CDMA_Memory_Pool_Size(void);
int CDMA_Mem_Config(byte * pMem);
extern byte g_SBDCmdIndex;
#endif

#if FLASH_CDMA
/////////////   prototypes: APIs for LLD_CDMA
uint16  CDMA_Flash_Init (void);
uint16  CDMA_Event_Status(void);
#endif

// CMD-DMA Descriptor Struct.  These are defined by the CMD_DMA HW
typedef struct CMD_Packet_tag
{
    uint32  NxtPointerHi;
    uint32  NxtPointerLo;
    uint32  FlashPointerHi;
    uint32  FlashPointerLo;
    uint32  CommandType;
    uint32  MemAddrHi;
    uint32  MemAddrLo;
    uint32  CommandFlags;
    uint32  Channel;
    uint32  Status;
    uint32  MemCopyPointerHi;
    uint32  MemCopyPointerLo;
    uint32  Reserved12;
    uint32  Reserved13;
    uint32  Reserved14;
    uint32  Tag;
}CDMA_DESCRIPTOR;

// This struct holds one MemCopy descriptor as defined by the HW
typedef struct MemCopy_tag
{
    uint32  NxtPointerHi;
    uint32  NxtPointerLo;
    uint32  SrcAddrHi;
    uint32  SrcAddrLo;
    uint32  DestAddrHi;
    uint32  DestAddrLo;
    uint32  XferSize;
    uint32  MemCopyFlags;
    uint32  MemCopyStatus;
    uint32  reserved9;
    uint32  reserved10;
    uint32  reserved11;
    uint32  reserved12;
    uint32  reserved13;
    uint32  reserved14;
    uint32  reserved15;
}MEM_COPY_DESCRIPTOR;

// Pending CMD table entries (includes MemCopy parameters
typedef struct GLOB_PendingCMD_tag
{
    byte    Tag;
    byte    CMD;
    byte*   DataAddr;
    BLOCKNODE  Block;
    uint16  Page;
    uint16  PageCount;
    byte*   DataDestAddr;
    byte*   DataSrcAddr;
    uint16  MemCopyByteCnt;
    uint16  Flags;
    uint16  ChanSync[LLD_NUM_FLASH_CHANNELS+1];
    uint16  Status;
    byte    SBDCmdIndex;
}PENDING_CMD;

// Definitions for CMD DMA descriptor chain fields
#define     CMD_DMA_DESC_COMP   0x8000
#define     CMD_DMA_DESC_FAIL   0x4000

#endif //_LLD_CDMA_
