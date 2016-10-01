/*
 * NAND Flash Controller Device Driver
 * Copyright (c) 2011-2012, Intel Corporation and its suppliers.
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

#include "ffsport.h"
//! \file
/** @addtogroup raw_data_access */
/*@{*/
/**
This command forcibly erases a NAND block, no matter it’s a good block or not.
<BR/>
Usage:
\code
ioctl(int fd, GLOB_SBD_IOCTL_ERASE_RAW, ADDRESSTYPE arg)
\endcode
<dl><dt><b>Parameters:</b></dt><dd>
  <table class="params">
    <tr><td class="paramdir">[in]</td><td class="paramname">fd</td><td> - file descriptor </td></tr>
    <tr><td class="paramdir">[in]</td><td class="paramname">arg</td><td> - The block number to be erased.  </td></tr>
  </table>
  </dd>
</dl>
<dl><dt><b>Return values:</b></dt><dd>
  <table class="retval">
    <tr><td class="paramname">0</td><td>success</td></tr>
    <tr><td class="paramname">1</td><td>failure</td></tr>
  </table>
  </dd>
</dl>
*/
/** \anchor GLOB_SBD_IOCTL_ERASE_RAW */
#define GLOB_SBD_IOCTL_ERASE_RAW (0x9910)

/**
This command checks manufacture bad block marker first before erasing a block, and will return failure quickly once it is a manufacture bad block.
<BR/>
Usage:
\code
ioctl(int fd, GLOB_SBD_IOCTL_ERASE_BYMARKER, ADDRESSTYPE arg)
\endcode
<dl><dt><b>Parameters:</b></dt><dd>
  <table class="params">
    <tr><td class="paramdir">[in]</td><td class="paramname">fd</td><td> - file descriptor </td></tr>
    <tr><td class="paramdir">[in]</td><td class="paramname">arg</td><td> - The block number to be erased.  </td></tr>
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
#define GLOB_SBD_IOCTL_ERASE_BYMARKER (0x9915)

/*@}*/

/** @defgroup	boot_paraition_bad Boot Partition Bad Block Table management */
/** @addtogroup boot_paraition_bad */
/*@{*/
/**
Command to read BBT content.
<BR/>
Usage:
\code
ioctl(int fd, GLOB_SBD_IOCTL_RD_BT, (BOOT_BLOCKTABLE_IO_CMD)* arg)
\endcode
<dl><dt><b>Parameters:</b></dt><dd>
  <table class="params">
    <tr><td class="paramdir">[in]</td><td class="paramname">fd</td><td> - file descriptor </td></tr>
    <tr><td class="paramdir">[in]</td><td class="paramname">arg</td><td> - structure to receive BBT content from driver. The parameter should follow below prototype.  </td></tr>
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
#define GLOB_SBD_IOCTL_RD_BT (0x9911)
/**
Command to create a dummy BBT in NAND that shows all blocks are good.
<BR/>
Usage:
\code
        result = ioctl(int fd, GLOB_SBD_IOCTL_CREATE_BT_DUMMY, ADDRESSTYPE arg);
\endcode
<dl><dt><b>Parameters:</b></dt><dd>
  <table class="params">
    <tr><td class="paramdir">[in]</td><td class="paramname">arg = 0</td><td> create a dummy BBT when there's no BBT exist in NAND. </td></tr>
    <tr><td class="paramdir">[in]</td><td class="paramname">arg = 1</td><td> forcibly create a dummy BBT, no matter BBT exist or not </td></tr>
  </table>
  </dd>
</dl>
<dl><dt><b>Return values:</b></dt><dd>
  <table class="retval">
    <tr><td class="paramname">0</td><td>if success</td></tr>
    <tr><td class="paramname">1</td><td>if failed to create</td></tr>
    <tr><td class="paramname">2</td><td>if there's block table exist</td></tr>
  </table>
  </dd>
</dl>
*/
#define GLOB_SBD_IOCTL_CREATE_BT_DUMMY (0x9912)
/**
Create a BBT by erasing the entire Boot Partition, and marking any blocks as bad which return failure status during the erase operation. The driver will read out MBH first before block erase and rewrite it to NAND with the new BBT. Thus there's no need to rewrite new MBH manually.
<BR/>
Usage:
\code
        result = ioctl (int fd, GLOB_SBD_IOCTL_CREATE_BT_BYERASE);
\endcode
<dl><dt><b>Return values:</b></dt><dd>
  <table class="retval">
    <tr><td class="paramname">0</td><td>success</td></tr>
    <tr><td class="paramname">-1</td><td>failure</td></tr>
  </table>
  </dd>
</dl>
*/
#define GLOB_SBD_IOCTL_CREATE_BT_BYERASE (0x9913)
/**
Create a BBT by scanning the initial manufactured bad block markers in the entire boot partition.
<BR/>
Usage:
\code
        result = ioctl (int fd, GLOB_SBD_IOCTL_CREATE_BT_BYMARKER)
\endcode
<dl><dt><b>Return values:</b></dt><dd>
  <table class="retval">
    <tr><td class="paramname">0</td><td>success</td></tr>
    <tr><td class="paramname">-1</td><td>failure</td></tr>
  </table>
  </dd>
</dl>
*/
#define GLOB_SBD_IOCTL_CREATE_BT_BYMARKER (0x9914)





typedef struct _boot_blocktable_info_tag
{
	byte bt_sig[4];/**< "GOOD" means a block table exist */
	byte nSpareSkipByteInZone1;/**<Spare area skip byte value in zone 1, default 0 */
	byte nSpareSkipByteInZone0; /**< Spare area skip byte value in zone 0, default 0 */
	byte reserve3;
	byte btEntrySizeInBits;/**<default to be 0x8, means each block status is represented by 8 bits */
	uint16 btEntryNum;/**< Block table entry number, equal to block number in boot partition */
	uint16 btOffsetInBytes;/**< Block table page offset in byte */
	uint16 md5OffsetInBytes;/**< Md5 hash page offset in byte */
	uint16 md5SizeInBytes;/**< Size of md5 hash */
} __attribute__ ((aligned(1), packed)) BOOT_BLOCKTABLE_INFO;

typedef struct _boot_block_table_io_cmd_tag
{
	byte btEntrySizeInBits;/**< default to be 0x8, means each block status is represented by 8 bits */
	uint16 btEntryNum;/**< Block table entry number, equal to block number in boot partition */
    ADDRESSTYPE 	AddrInRam;    /**< Source Address for Image in SRAM */
}BOOT_BLOCKTABLE_IO_CMD;

/*@}*/

uint32 GLOB_BBT_Init (void);
void GLOB_BBT_Release (void);
uint16 GLOB_BBT_NAND_Write_Page_Main(byte* read_data,BLOCKNODE block,PAGENUMTYPE page,PAGENUMTYPE page_count);
uint16 GLOB_BBT_NAND_Read_Page_Main(byte* read_data,BLOCKNODE block,PAGENUMTYPE page,PAGENUMTYPE page_count);
uint16 GLOB_BBT_Create_Block_Table_byInitalMarker(void);
uint16 GLOB_BBT_Create_Block_Table_byErase(void);
uint16 GLOB_BBT_Create_Block_Table_Dummy(uint16 wforce);
uint16 GLOB_BBT_Erase_Block_by_BBT(BLOCKNODE blocknum);
uint16 GLOB_BBT_Erase_Block_by_Marker(BLOCKNODE blocknum);//Erase the NAND block by look up initial bad block marker
uint16 GLOB_BBT_IO_Read_Block_Table(BOOT_BLOCKTABLE_IO_CMD* pbtInfo);
