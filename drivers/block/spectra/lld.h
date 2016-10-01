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



#ifndef _LLD_
#define _LLD_

#ifdef ELDORA
#include "defs.h"
#else
#include "ffsport.h"
#include "testcnf.h"
#include "spectraswconfig.h"
#include "flash.h"
#endif

#define GOOD_BLOCK 0
#define DEFECTIVE_BLOCK 1
#define READ_ERROR 2

#define CLK_X  5
#define CLK_MULTI 4

// Max main & spare sizes supported in LLD
#define MAX_PAGE_MAIN_AREA          8192
#define MAX_PAGE_SPARE_AREA          512
#define MAX_PAGE_MAINSPARE_AREA     8704
#define MAX_LLD_BLOCKS              128

// for GLOB_LLD_Enable_Disable_Interrupts
#define ENABLE_INTERRUPTS           0x0001
#define DISABLE_INTERRUPTS          0x0000



// Typedefs

//  prototypes: API for LLD
/* Currently, Write_Page_Main
 * 			  MemCopy
 * 			  Read_Page_Main_Spare
 * do not have flag because they were not implemented prior to this
 * They are not being added to keep changes to a minimum for now.
 * Currently, they are not required (only reqd for Wr_P_M_S.)
 * Later on, these NEED to be changed.
 */
extern uint16   GLOB_LLD_Flash_Release (void);
extern uint16   GLOB_LLD_Flash_Reset(void);
extern uint16   GLOB_LLD_Read_Device_ID (void);
#if CMD_DMA
extern uint16   GLOB_LLD_Flash_Init ( uint16 Flags);
extern uint16   GLOB_LLD_Execute_CMDs(uint16 count);
extern uint16   GLOB_LLD_Erase_Block(BLOCKNODE block, byte TagCount, uint16 Flags);
extern uint16   GLOB_LLD_Write_Page_Main(byte* write_data,BLOCKNODE block,PAGENUMTYPE Page,PAGENUMTYPE
                    PageCount,byte CommandCount);
extern uint16   GLOB_LLD_Read_Page_Main(byte* read_data, BLOCKNODE block,PAGENUMTYPE Page,PAGENUMTYPE
                    PageCount,byte CommandCount,uint16 Flags);
extern uint16  GLOB_LLD_MemCopy_CMD(byte tag, byte* dest, byte* src, uint16
                    ByteCount, uint16 Flags);
#else
extern uint16   GLOB_LLD_Flash_Init (void);
extern uint16   GLOB_LLD_Erase_Block(BLOCKNODE block_add);
extern uint16   GLOB_LLD_Write_Page_Main(byte* write_data,BLOCKNODE block,PAGENUMTYPE Page,PAGENUMTYPE PageCount);
extern uint16   GLOB_LLD_Read_Page_Main(byte* read_data, BLOCKNODE block,PAGENUMTYPE Page,PAGENUMTYPE
                    PageCount);
#endif

extern uint16   GLOB_LLD_Event_Status(void);
extern void     GLOB_LLD_Enable_Disable_Interrupts(uint16 INT_ENABLE);

#ifndef ELDORA
extern uint16   GLOB_LLD_UnlockArrayAll (void);
extern uint16   GLOB_LLD_Read_Page_Spare(byte* read_data,BLOCKNODE block,PAGENUMTYPE Page,PAGENUMTYPE PageCount);
extern uint16   GLOB_LLD_Write_Page_Spare(byte* write_data,BLOCKNODE block,PAGENUMTYPE Page,PAGENUMTYPE PageCount);
extern uint16   GLOB_LLD_Get_Bad_Block(BLOCKNODE block);
extern uint16	GLOB_LLD_Get_Bad_Block_Raw(BLOCKNODE block);
#if CMD_DMA
extern uint16   GLOB_LLD_Write_Page_Main_Spare(byte* write_data,BLOCKNODE block,PAGENUMTYPE Page,PAGENUMTYPE
                    PageCount,byte CommandCount,uint16 Flags);
extern uint16   GLOB_LLD_Read_Page_Main_Spare(byte* read_data, BLOCKNODE block,PAGENUMTYPE Page,PAGENUMTYPE
                    PageCount,byte CommandCount);
#else
extern uint16   GLOB_LLD_Write_Page_Main_Spare(byte* write_data,BLOCKNODE block,PAGENUMTYPE Page,PAGENUMTYPE PageCount);
extern uint16   GLOB_LLD_Read_Page_Main_Spare(byte* read_data, BLOCKNODE block,PAGENUMTYPE Page,PAGENUMTYPE
                    PageCount);
extern uint16  GLOB_LLD_Read_Page_Main_Spare_Raw(byte* read_data,BLOCKNODE block,PAGENUMTYPE page,PAGENUMTYPE page_count);


#endif // CMD_DMA

extern uint32  GLOB_LLD_Memory_Pool_Size(void);
extern int GLOB_LLD_Mem_Config(byte * pMem);

#if CMD_DMA
#define LLD_CMD_FLAG_ORDER_BEFORE_REST		(0x1)
#define LLD_CMD_FLAG_MODE_POLL				(0x4)
#define LLD_CMD_FLAG_MODE_CDMA				(0x8)
#endif // CMD_DMA

#endif // !ELDORA


#endif //_LLD_
