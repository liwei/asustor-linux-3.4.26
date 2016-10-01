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



#ifndef _testcnf_h
#define _testcnf_h
// flags used to select which test to run

#define TEST_LLD                0
#define TEST_FTL                0
#define TEST_LXDRV              0
#define TEST_LARGE_FILE_SYS     0

#define DEBUG_DEMO              0

#define SEAMLESS                0

#define FLASH_EMU               0
#define FLASH_ESL               0
#define FLASH_NAND              1
#define FLASH_CDMA              0
#define FLASH_EMU_STATIC_MALLOC 1
#define EMU_MLC_DEV     1

#if TEST_LARGE_FILE_SYS
#define GLOB_LLD_PAGES           128
#define GLOB_LLD_PAGE_SIZE       (4096+218)
#define GLOB_LLD_PAGE_DATA_SIZE  4096
#define GLOB_LLD_BLOCKS          16384
#define TOTAL_TEST_BLOCKS          2500
#else
#define GLOB_LLD_PAGES           64
#define GLOB_LLD_PAGE_SIZE       (512+16)
#define GLOB_LLD_PAGE_DATA_SIZE  512
#define GLOB_LLD_BLOCKS          320
#endif

#define  _DEBUG_  (0)

#if _DEBUG_
#define  VERBOSE  0
#define DEBUG_BNDRY             (0)

#define DEBUG_SYNC             0
#define DBG_SNC_PRINTEVERY    (1000000)
#define DBG_SNC_PRINTFROM     (0)
#define DBG_SNC_PRINTTILL     (9999)
#else
#define  VERBOSE    0
#define DEBUG_BNDRY 0
#define DEBUG_SYNC  0
#endif

#if 	FLASH_EMU
#define	EMU_BAD_BLOCK	0

#if	EMU_BAD_BLOCK
#define	NO_CMDS_DONE_AFTER_ERROR	0
#define MAX_BAD_BLOCKS 	(unsigned int)(GLOB_LLD_BLOCKS / 5)
#endif
#endif


// prototypes: public
void PrintBlockTable(void);
void TestFTL (int argc, char *argv[]);
void TestLLD(void);
#endif
