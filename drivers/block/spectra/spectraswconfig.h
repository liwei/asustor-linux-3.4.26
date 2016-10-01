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



#ifndef _SPECTRASWCONFIG_
#define _SPECTRASWCONFIG_

/***** Common Parameters *****/
#define RETRY_TIMES                   3

#define READ_BADBLOCK_INFO            1
#define READBACK_VERIFY               0
#define AUTO_FORMAT_FLASH             1

/***** Cache Parameters *****/
#define PAGES_PER_CACHE_BLOCK         0
#define CACHE_BLOCK_NUMBER            2

/***** Block Table Parameters *****/
#define BLOCK_TABLE_INDEX             0

/***** Wear Leveling Parameters *****/
#define WEAR_LEVELING_GATE         0x10
#define WEAR_LEVELING_BLOCK_NUM      10

/***** Product Feature Support *****/
#define CMD_DMA     0
#define DDMA        1

/**** Partition Logic Related Information ****/
#define SPECTRA_PARTITION_ID    0
#define SPECTRA_MIN_BANK        0
#define SPECTRA_MAX_BANK        3

/* Enable this macro if the number of flash blocks is larger than 16K. */
#define SUPPORT_LARGE_BLOCKNUM  1

/* Enable this macro to support file-system larger than 4G. */
#define SUPPORT_LARGE_FILESYS   1

/**** Block Table and Reserved Block Parameters *****/
#define SPECTRA_START_BLOCK     66
#define NUM_FREE_BLOCKS_GATE    100

/**** Linux Block Driver Parameters ****/
#define SBD_MEMPOOL_PTR      0x0c800000

/**** Hardware Parameters ****/
//#define GLOB_HWCTL_REG_BASE     0x60000000
#define GLOB_HWCTL_REG_BASE     0xdff10000

#define GLOB_HWCTL_REG_SIZE     0x1000

//#define GLOB_HWCTL_MEM_BASE     0x50000000
#define GLOB_HWCTL_MEM_BASE     0xdff00000
#define GLOB_HWCTL_MEM_SIZE     0x40

#define GLOB_HWCTL_DEFAULT_BLKS 4096

/****Raw access partition range****/
/****Raw access starts from block 0****/
//#define SPECTRA_RAW_END_BLOCK	64 // The end block of raw access partition;if not set, it will be the end of NAND device

/**** Toshiba Device Parameters ****/

#define GLOB_DEVTSBA_ALT_BLK_NFO 0
#define GLOB_DEVTSBA_ALT_BLK_ADD \
    (GLOB_HWCTL_REG_BASE + (DEVICE_PARAM_2 << 2))


#define SUPPORT_15BITECC        1
#define SUPPORT_8BITECC         1

#define CUSTOM_CONF_PARAMS      0

#define ONFI_BLOOM_TIME         0
#define MODE5_WORKAROUND        1

#endif //_SPECTRASWCONFIG_
