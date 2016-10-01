/*
 * NAND Flash Controller Device Driver
 * Copyright (c) 2008-2012, Intel Corporation and its suppliers.
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


#ifndef _FLASH_INTERFACE_
#define _FLASH_INTERFACE_

#include "ffsport.h"
#include "spectraswconfig.h"


#if SUPPORT_LARGE_BLOCKNUM
#define MAX_BLOCKNODE_VALUE     0xFFFFFF
#define DISCARD_BLOCK           0x800000
#define SPARE_BLOCK             0x400000
#define BAD_BLOCK               0xC00000
typedef uint32 BLOCKNODE;
#else
#define MAX_BLOCKNODE_VALUE     0xFFFF
#define DISCARD_BLOCK           0x8000
#define SPARE_BLOCK             0x4000
#define BAD_BLOCK               0xC000
typedef uint16 BLOCKNODE;
#endif

#define MAX_BYTE_VALUE      0xFF
#define UNHIT_BLOCK         0xFF

#define IN_PROGRESS_BLOCK_TABLE   0x00
#define CURRENT_BLOCK_TABLE       0x01


#define BTSIG_OFFSET   (0)
#define BTSIG_BYTES    (5)
#define BTSIG_DELTA    (3)

#define MAX_TWO_BYTE_VALUE 0xFFFF
#define MAX_READ_COUNTER  0x2710

#define FIRST_BT_ID		(1)
#define LAST_BT_ID    (254)
#define BTBLOCK_INVAL  (BLOCKNODE)(0xFFFFFFFF)

#define RESTORE_CACHE_ON_CDMA_CHAIN_FAILURE    1


#define ALIGN_DWORD_FWD(ptr)          (ptr = (byte *)((unsigned long)(ptr+3) & ~0x3))
#define ALIGN_DWORD_BWD(ptr)          (ptr = (byte *)((unsigned long)ptr & ~0x3))


typedef uint16 PAGENUMTYPE;
typedef uint16 PAGESIZETYPE;
typedef uint32 BLOCKSIZETYPE;


//! \file
//IOCTL command set
/** @defgroup raw_data_access APIs provided for raw data access
The feature of NAND raw data access provides the user with the capability of operating NAND flash physical blocks, while a normal file system operations described in previous sections can only access logical blocks, which being dynamically mapped to physical blocks in the driver internal. The following sections will focus on the raw data access functions.

\anchor raw_data_access_area_range_index
@section raw_data_access_section Raw data access area range configuration
The area can be raw accessed ranges from block 0 to SPECTRA_RAW_END_BLOCK, which is a configurable macro defined in "spectraswconfig.h". If not set, the whole NAND device could be accessed.

\anchor data_structures_communicated_index
@section data_structures_communicated Data structures communicated between driver and application layer

<b>DEVICE_INFO</b> - This structure is used to describe NAND device identification information
\code
typedef struct device_info_tag DEVICE_INFO;
\endcode


<b>SPEC_IO_CMD</b> - This structure defines the parameter of IOCTL operations.
\code
typedef struct _spectra_io_cmd_tag SPEC_IO_CMD;
\endcode

*/
/** @addtogroup raw_data_access */
/*@{*/
/** IOCTL raw access command type */
typedef struct _spectra_io_cmd_tag
{
    uint32 	NumPagesToTransfer; /**< Number of Pages to transfer */
    ADDRESSTYPE 	AddrInRam;    /**< Source Address for Image in SRAM */
    BLOCKNODE 		StartBlockNum; /**< Image Destination Block number */
    PAGENUMTYPE 	StartPageNum;  /**< Image Destination start Page */
}SPEC_IO_CMD;

/**
Command to read data from NAND flash page main area to user space; the largest data length should be no more than one block.
<BR/>
Usage:
\code
        result = ioctl(int fd, GLOB_SBD_IOCTL_RD_MAIN, (SPEC_IO_CMD *) arg);
\endcode

<dl><dt><b>Parameters:</b></dt><dd>
  <table class="params">
    <tr><td class="paramdir">[in]</td><td class="paramname">fd</td><td> - file descriptor </td></tr>
    <tr><td class="paramdir">[in]</td><td class="paramname">arg</td><td> - A pointer points to a data structure with the type of SPEC_IO_CMD, which describes the page numbers to be transferred, the external buffer start address and the target page/block number in NAND device.  </td></tr>
  </table>
  </dd>
</dl>
<dl><dt><b>Return values:</b></dt><dd>
  <table class="retval">
    <tr><td class="paramname">0</td><td>success</td></tr>
    <tr><td class="paramname">-1</td><td>failure</td></tr>
  </table>
  </dd>
</dl>
*/
#define GLOB_SBD_IOCTL_RD_MAIN      (0x8810)
/**
Command to write data from user space to NAND flash page main area; the largest data length should be no more than one block.
<BR/>
Usage:
\code
        result = ioctl (int fd, GLOB_SBD_IOCTL_WRITE_MAIN, (SPEC_IO_CMD *) arg);
\endcode
<dl><dt><b>Parameters:</b></dt><dd>
  <table class="params">
    <tr><td class="paramdir">[in]</td><td class="paramname">fd</td><td> - file descriptor </td></tr>
    <tr><td class="paramdir">[in]</td><td class="paramname">arg</td><td> - A pointer points to a data structure with the type of SPEC_IO_CMD, which describes the page numbers to be transferred, the external buffer start address and the target page/block number in NAND device. </td></tr>
  </table>
  </dd>
</dl>
<dl><dt><b>Return values:</b></dt><dd>
  <table class="retval">
    <tr><td class="paramname">0</td><td>success</td></tr>
    <tr><td class="paramname">-1</td><td>failure</td></tr>
  </table>
  </dd>
</dl>
*/
#define GLOB_SBD_IOCTL_WR_MAIN      (0x8811)
/**
This command lookup BBT first before erasing a block, and will return failure quickly once it's recorded as a bad block.
<BR/>
Usage:
\code
        result = ioctl (int fd, GLOB_SBD_IOCTL_ERASE, (BLOCKNODE) arg);
\endcode
<dl><dt><b>Parameters:</b></dt><dd>
  <table class="params">
    <tr><td class="paramdir">[in]</td><td class="paramname">fd</td><td> - file descriptor </td></tr>
    <tr><td class="paramdir">[in]</td><td class="paramname">arg</td><td> - The block number to be erased. </td></tr>
  </table>
  </dd>
</dl>
<dl><dt><b>Return values:</b></dt><dd>
  <table class="retval">
    <tr><td class="paramname">0</td><td>success</td></tr>
    <tr><td class="paramname">-1</td><td>failure</td></tr>
  </table>
  </dd>
</dl>
*/
#define GLOB_SBD_IOCTL_ERASE      (0x8812)
/**
Command to read NAND device identification information, such as manufacture ID, block number, page size, etc.
<BR/>
Usage:
\code
        result = ioctl (int fd, GLOB_SBD_IOCTL_RD_ID, (DEVICE_INFO *) arg);
\endcode
<dl><dt><b>Parameters:</b></dt><dd>
  <table class="params">
    <tr><td class="paramdir">[in]</td><td class="paramname">fd</td><td> - file descriptor </td></tr>
    <tr><td class="paramdir">[in]</td><td class="paramname">arg</td><td> - A pointer points to an empty data structure with the type of DEVICE_INFO.  </td></tr>
    <tr><td class="paramdir">[out]</td><td class="paramname">arg</td><td> - The pointed structure will be filled in NAND device identification information. </td></tr>
  </table>
  </dd>
</dl>
<dl><dt><b>Return values:</b></dt><dd>
  <table class="retval">
    <tr><td class="paramname">0</td><td>success</td></tr>
    <tr><td class="paramname">-1</td><td>failure</td></tr>
  </table>
  </dd>
</dl>
*/
#define GLOB_SBD_IOCTL_RD_ID      (0x8813)
/**
Command to detect a block is good or bad.
<BR/>
Usage:
\code
        result = ioctl (int fd, GLOB_SBD_IOCTL_CHK_BADBLK, (BLOCKNODE) arg);
\endcode
<dl><dt><b>Parameters:</b></dt><dd>
  <table class="params">
    <tr><td class="paramdir">[in]</td><td class="paramname">fd</td><td> - file descriptor </td></tr>
    <tr><td class="paramdir">[in]</td><td class="paramname">arg</td><td> - The block number to be detected.  </td></tr>
  </table>
  </dd>
</dl>
<dl><dt><b>Return values:</b></dt><dd>
  <table class="retval">
    <tr><td class="paramname">0</td><td>good block</td></tr>
    <tr><td class="paramname">1</td><td>bad block</td></tr>
    <tr><td class="paramname">2</td><td>read error</td></tr>
  </table>
  </dd>
</dl>
*/
#define GLOB_SBD_IOCTL_CHK_BADBLK      (0x8814)
/**
This command is used to read physical page data (page main and spare areas); the largest data length should be no more than one block.
<BR/>
Usage:
\code
        result = ioctl (int fd, GLOB_SBD_IOCTL_RD_MAIN_SPARE_RAW, (SPEC_IO_CMD *) arg);
\endcode
<dl><dt><b>Parameters:</b></dt><dd>
  <table class="params">
    <tr><td class="paramdir">[in]</td><td class="paramname">fd</td><td> - file descriptor </td></tr>
    <tr><td class="paramdir">[in]</td><td class="paramname">arg</td><td> - A pointer points to a data structure with the type of SPEC_IO_CMD, which describes the page numbers to be transferred, the external buffer start address and the target page/block number in NAND device. </td></tr>
  </table>
  </dd>
</dl>
<dl><dt><b>Return values:</b></dt><dd>
  <table class="retval">
    <tr><td class="paramname">0</td><td>success</td></tr>
    <tr><td class="paramname">-1</td><td>failure</td></tr>
  </table>
  </dd>
</dl>
*/
#define GLOB_SBD_IOCTL_RD_MAIN_SPARE_RAW      (0x8815)
#define GLOB_SBD_IOCTL_WR_MAIN_SPARE_RAW      (0x8816)
/*@}*/

/** @defgroup special_api Special APIs

The ioctl commands above are provided to perform the special operations on Spectra partition. These commands are described in the following sections.

In order to ensure data safety, Spectra driver automatically handles garbage collection, dynamic wear levering when necessary and periodically flushes the internal cache. The cache flush period is tunable and default is 18 seconds.

*/
/** @addtogroup special_api */
/*@{*/
/**
This ioctl command is used by the application to let the Linux NAND flash driver perform garbage collection for NAND flash. <BR/>
Here is an example:
\code
int fd;
fd = open("/dev/Glob_Spectraa", O_RDWR);
ioctl(fd, GLOB_SBD_IOCTL_GC, NULL);
close(fd);
\endcode
*/
#define GLOB_SBD_IOCTL_GC          (0x7701)
/**
This ioctl command is used by the application to let the Linux NAND flash driver perform static wear-leveling for NAND flash. Here is an example:
\code
int fd;
fd = open("/dev/Glob_Spectraa", O_RDWR);
ioctl(fd, GLOB_SBD_IOCTL_WL, NULL);
close(fd);
\endcode
*/
#define GLOB_SBD_IOCTL_WL          (0x7702)
/**
This ioctl command is used by the application to let the Linux NAND flash driver perform internal formatting (erase flash and establish block table) for NAND flash.
<BR/>Here is an example:
\code
int fd;
fd = open("/dev/Glob_Spectraa", O_RDWR);
ioctl(fd, GLOB_SBD_IOCTL_FORMAT, NULL);
close(fd);
\endcode
*/
#define GLOB_SBD_IOCTL_FORMAT      (0x7703)
/**
This ioctl command is used by the application to let the Linux NAND flash driver flush the internal cache (used to speed up accessing NAND flash) for NAND flash.
<BR/>Here is an example:
\code
int fd;
fd = open("/dev/Glob_Spectraa", O_RDWR);
ioctl(fd, GLOB_SBD_IOCTL_FLUSH_CACHE, NULL);
close(fd);
\endcode
*/
#define GLOB_SBD_IOCTL_FLUSH_CACHE (0x7704)

/** This structure is used to describe NAND device identification information. It is defined as follows: */
typedef struct device_info_tag
{
    uint16        wDeviceMaker;              /**< mfg code from ReadID */
    uint32        wDeviceType;
    BLOCKNODE     wSpectraStartBlock;
    BLOCKNODE     wSpectraEndBlock;
    BLOCKNODE     wTotalBlocks;              /**< blocks in a disk (All flash devices connected to controller) */
    PAGENUMTYPE   wPagesPerBlock;            /**< pages in a block */
    PAGESIZETYPE  wPageSize;                 /**< Total page_size in bytes */
    PAGESIZETYPE  wPageDataSize;             /**< Total page data_size in bytes */
    PAGESIZETYPE  wPageSpareSize;            /**< Total spare_size in bytes */
    uint16        wNumPageSpareFlag;         /**< number of Spare bytes available for software flags */
    uint16        wECCBytesPerSector;        /**< number of ECC bytes used by a Page */
    BLOCKSIZETYPE wBlockSize;                /**< block size in bytes */
    BLOCKSIZETYPE wBlockDataSize;            /**< block data size in bytes */
    BLOCKNODE     wDataBlockNum;             /**< data block number which can be used */
    byte          bPlaneNum;                 /**< number of planes */
    PAGESIZETYPE  wDeviceMainAreaSize;       /**< main area of an individual device */
    PAGESIZETYPE  wDeviceSpareAreaSize;      /**< spare area of an individual device */
    uint16        wDevicesConnected;         /**< Num of devices connected in parallel for width exp */
    uint16        wDeviceWidth;              /**< Width of the device connected 0->8 bit, 1->16it */
    uint16        wHWRevision;
    uint16        wHWFeatures;               /**< Ctrl feature register */

    uint16        wONFIDevFeatures;
    uint16        wONFIOptCommands;
    uint16        wONFITimingMode;
    uint16        wONFIPgmCacheTimingMode;

    uint16        MLCDevice;
    uint16        wSpareSkipBytes;
    byte          nBitsInPageNumber;
    byte          nBitsInPageDataSize;
    byte          nBitsInBlockDataSize;
}DEVICE_INFO;
/*@}*/

typedef enum
{
	FS_ACCESS,
	READ_PAGE_MAIN_SPARE_RAW,
	WRITE_PAGE_MAIN_SPARE_RAW,
	GET_BAD_BLOCK_RAW
}ACCESS_TYPE;

extern DEVICE_INFO GLOB_DeviceInfo;
extern DEVICE_INFO RAW_DeviceInfo;

//Cache item format
typedef struct flash_cache_item_tag
{
    ADDRESSTYPE  dwAddress;
    byte   bUCount;
    byte   bChanged;
    byte * pContent;
}FLASH_CACHE_ITEM;

typedef struct flash_cache_tag
{
    byte bLFU;
    BLOCKSIZETYPE dwCacheDataSize;
    PAGENUMTYPE wCachePageNum;
    FLASH_CACHE_ITEM ItemArray[CACHE_BLOCK_NUMBER];
}FLASH_CACHE;
/*@}*/
#if RESTORE_CACHE_ON_CDMA_CHAIN_FAILURE
#if SUPPORT_LARGE_FILESYS
#define INVALID_CACHE_BLK_ADD   (0xFFFFFFFFFFFFFFFFULL)
#else
#define INVALID_CACHE_BLK_ADD   (0xF0000000)
#endif

typedef struct flash_cache_mod_item_tag
{
    ADDRESSTYPE  dwAddress;
    byte   bChanged;
}FLASH_CACHE_MOD_ITEM;

typedef struct flash_cache_delta_list_tag
{
    char nUsedCacheItem;
    FLASH_CACHE_MOD_ITEM IntCache;
} FLASH_CACHE_DELTA_LIST;
#endif

// structure used for IndentfyDevice function
typedef struct IndentfyDevice_tag
{
    BLOCKNODE  NumBlocks;
    PAGENUMTYPE    PagesPerBlock;
    PAGESIZETYPE   PageDataSize;
    uint16     wECCBytesPerSector;
    BLOCKNODE  wDataBlockNum;
    uint32     SizeOfGlobalMem;

}IDENTFY_DEVICE_DATA;


//extern IDENTFY_DEVICE_DATA IdentfyDeviceData;


int  GLOB_FTL_Flash_Init (void);
int  GLOB_FTL_Flash_Release (void);
int  GLOB_FTL_Block_Erase (ADDRESSTYPE block_addr);
int  GLOB_FTL_Is_BadBlock (BLOCKNODE block_num);
int  GLOB_FTL_IdentifyDevice(IDENTFY_DEVICE_DATA *IdentfyDeviceData);
int  GLOB_FTL_Mem_Config (byte * pMem);
int  GLOB_FTL_Event_Status (int*);
void GLOB_FTL_Enable_Disable_Interrupts(uint16 INT_ENABLE);
#if CMD_DMA
void GLOB_FTL_Execute_CMDS(void);
#endif
int FTL_Read_Disturbance(BLOCKNODE dwBlockAddr);

//Flash r/w based on cache
int  GLOB_FTL_Page_Read (byte * read_data, ADDRESSTYPE page_addr);
int  GLOB_FTL_Page_Write (byte * write_data, ADDRESSTYPE page_addr);
int  GLOB_FTL_Wear_Leveling (void);
int  GLOB_FTL_Flash_Format (void);
int  GLOB_FTL_Init (void);
int  GLOB_FTL_Flush_Cache(void);
int  GLOB_FTL_Garbage_Collection (void);
int GLOB_FTL_BT_Garbage_Collection(void);
void GLOB_FTL_Cache_Release (void);

#if DEBUG_BNDRY
void debug_boundary_lineno_error(int chnl, int limit, int no, int lineno, char *filename);
#define debug_boundary_error(chnl, limit, no) debug_boundary_lineno_error(chnl, limit, no, __LINE__, __FILE__)
#else
#define debug_boundary_error(chnl, limit, no) ;
#endif

#if _DEBUG_
extern int dbgGCCalled;
#endif

#endif //_FLASH_INTERFACE_
