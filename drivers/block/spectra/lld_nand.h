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

/*&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
* copyright (C) 2006 Spectra software, Inc.
* All rights reserved.
***********************************************************************
*  lld_nand.h
*&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&*/

#ifndef _LLD_NAND_
#define _LLD_NAND_

#ifdef ELDORA
#include "defs.h"
#else
#include "flash.h"
#include "ffsport.h"
#endif


extern volatile uint32 *FlashReg;
extern volatile uint32 *FlashMem;

struct nand_irq_info {
	uint32 state;
	uint32 flash_bank;
	struct completion complete;
};

uint16  NAND_Flash_Init (void);
uint16  NAND_Flash_Reset (void);
uint16  NAND_Read_Device_ID (void);
uint16  NAND_Erase_Block(BLOCKNODE flash_add);
uint16  NAND_Write_Page_Main(byte* write_data,BLOCKNODE block,uint16 page,uint16 page_count);
uint16  NAND_Read_Page_Main(byte* read_data,BLOCKNODE block,uint16 page,uint16 page_count);
uint16  NAND_UnlockArrayAll (void);
uint16  NAND_Write_Page_Main_Spare(byte* write_data,BLOCKNODE block,uint16 page,uint16 page_count);
uint16  NAND_Write_Page_Spare(byte* read_data,BLOCKNODE block,uint16 page,uint16 page_count);
uint16  NAND_Read_Page_Main_Spare (byte* read_data,BLOCKNODE block,uint16 page,uint16 page_count);
uint16  NAND_Read_Page_Main_Spare_Raw (byte* read_data,BLOCKNODE block,uint16 page,uint16 page_count);
uint16  NAND_Read_Page_Spare(byte* read_data,BLOCKNODE block,uint16 page,uint16 page_count);
uint16  NAND_LLD_Event_Status(void);
void    NAND_LLD_Enable_Disable_Interrupts(uint16 INT_ENABLE);
uint16  NAND_Get_Bad_Block(BLOCKNODE block);
uint16  NAND_Get_Bad_Block_Raw(BLOCKNODE block);


uint32  NAND_Memory_Pool_Size(void);
int NAND_Mem_Config(byte * pMem);

uint16  NAND_Pipeline_Read_Ahead(byte* read_data,BLOCKNODE block,uint16 page,uint16 page_count);
uint16  NAND_Pipeline_Write_Ahead(byte* write_data,BLOCKNODE block,uint16 page,uint16 page_count);
uint16  NAND_Multiplane_Read(byte* read_data,BLOCKNODE block,uint16 page,uint16 page_count);
uint16  NAND_Multiplane_Write(byte* write_data,BLOCKNODE block,uint16 page,uint16 page_count);

void NAND_Conv_Spare_Data_Log2Phy_Format(byte* data);
void NAND_Conv_Spare_Data_Phy2Log_Format(byte* data);
void NAND_Conv_Main_Spare_Data_Log2Phy_Format(byte* data, uint16 page_count);
void NAND_Conv_Main_Spare_Data_Phy2Log_Format(byte* data, uint16 page_count);

uint16  NAND_DDMA_IRQ_Init (void);
uint16  NAND_DDMA_IRQ_Release (void);
uint16 NAND_Flash_Release(void);



#define MODE_00    0x00000000
#define MODE_01    0x04000000
#define MODE_10    0x08000000
#define MODE_11    0x0C000000


#define DATA_TRANSFER_MODE              0
#define PROTECTION_PER_BLOCK            1
#define LOAD_WAIT_COUNT                 2
#define PROGRAM_WAIT_COUNT              3
#define ERASE_WAIT_COUNT                4
#define INT_MONITOR_CYCLE_COUNT         5
#define READ_BUSY_PIN_ENABLED           6
#define MULTIPLANE_OPERATION_SUPPORT    7
#define PRE_FETCH_MODE                  8
#define CE_DONT_CARE_SUPPORT            9
#define COPYBACK_SUPPORT                10
#define CACHE_WRITE_SUPPORT             11
#define CACHE_READ_SUPPORT              12
#define NUM_PAGES_IN_BLOCK              13
#define ECC_ENABLE_SELECT               14
#define WRITE_ENABLE_2_READ_ENABLE      15
#define ADDRESS_2_DATA                  16
#define READ_ENABLE_2_WRITE_ENABLE      17
#define TWO_ROW_ADDRESS_CYCLES          18
#define MULTIPLANE_ADDRESS_RESTRICT     19
#define ACC_CLOCKS                      20
#define READ_WRITE_ENABLE_LOW_COUNT     21
#define READ_WRITE_ENABLE_HIGH_COUNT    22

#define ECC_SECTOR_SIZE     512
#define LLD_MAX_FLASH_BANKS     4

#endif //_LLD_NAND_
