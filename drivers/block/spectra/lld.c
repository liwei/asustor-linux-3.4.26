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



#ifdef ELDORA
#include "defs.h"
#include "lld.h"
#else
#include "spectraswconfig.h"
#include "ffsport.h"
#include "ffsdefs.h"
#include "lld.h"

#ifdef NEW_LLD_API
#include "flash.h"
#endif

#endif


/*&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&*/
#if FLASH_EMU     // vector all the LLD calls to the LLD_EMU code
#include "lld_emu.h"
#include "lld_cdma.h"

// common functions:
uint16  GLOB_LLD_Flash_Reset(void)
{
    return emu_Flash_Reset();
}

uint16  GLOB_LLD_Read_Device_ID(void)
{
    return emu_Read_Device_ID();
}

uint16  GLOB_LLD_Flash_Release(void)
{
    return emu_Flash_Release();
}


#if CMD_DMA // new APIs with tags
uint16 GLOB_LLD_Flash_Init (uint16 Flags)
{
  if (Flags & LLD_CMD_FLAG_MODE_POLL)
  {
    return emu_Flash_Init();
  }
  else
  {
    return emu_CDMA_Flash_Init();
  }
}


uint16  GLOB_LLD_Erase_Block          (BLOCKNODE block, byte TagCount, uint16 Flags)
{
    if(Flags & LLD_CMD_FLAG_MODE_POLL)
        return  emu_Erase_Block(block);
    else
        return      CDMA_Data_CMD   (TagCount,           ERASE_CMD,   0,block,   0,    0, Flags);
}

uint16  GLOB_LLD_Write_Page_Main      (byte* data,BLOCKNODE block,PAGENUMTYPE page,PAGENUMTYPE  count,byte TagCount)
{
    return      CDMA_Data_CMD   (TagCount,      WRITE_MAIN_CMD,data,block,page,count, 0);
}

uint16  GLOB_LLD_Read_Page_Main       (byte* data,BLOCKNODE block,PAGENUMTYPE page,PAGENUMTYPE count,byte
  TagCount,uint16 Flags)
{
    if(Flags & LLD_CMD_FLAG_MODE_POLL)
        return emu_Read_Page_Main(data,block, page,count);
    else
    return      CDMA_Data_CMD   (TagCount,       READ_MAIN_CMD,data,block,page,count, Flags);
}

uint16  GLOB_LLD_MemCopy_CMD          (byte TagCount, byte* dest, byte* src, uint16 ByteCount,
        uint16 Flags)
{
    return     CDMA_MemCopy_CMD (TagCount, dest, src, ByteCount, Flags);
}

uint16  GLOB_LLD_Execute_CMDs         (uint16 count)
{
    return     emu_CDMA_Execute_CMDs(count);
}

uint16  GLOB_LLD_Event_Status(void)
{
    return     emu_CDMA_Event_Status();
}


#ifndef ELDORA
void GLOB_LLD_Enable_Disable_Interrupts(uint16 INT_ENABLE)
{
    emu_Enable_Disable_Interrupts(INT_ENABLE);
}

uint16 GLOB_LLD_Write_Page_Main_Spare(byte* write_data, BLOCKNODE block,PAGENUMTYPE Page,PAGENUMTYPE
  PageCount,byte   TagCount,uint16 Flags)
{
    if(Flags & LLD_CMD_FLAG_MODE_POLL)
    return emu_Write_Page_Main_Spare(write_data, block, Page, PageCount);
    else
    return      CDMA_Data_CMD(TagCount,WRITE_MAIN_SPARE_CMD,write_data,block,Page,PageCount, Flags);
}

uint16  GLOB_LLD_Read_Page_Main_Spare(byte* read_data,BLOCKNODE block,PAGENUMTYPE Page,PAGENUMTYPE
  PageCount,byte TagCount)
{
    return  CDMA_Data_CMD       (TagCount, READ_MAIN_SPARE_CMD,read_data,block,Page,PageCount,LLD_CMD_FLAG_MODE_CDMA);

}

uint16  GLOB_LLD_Write_Page_Spare(byte* write_data,BLOCKNODE block,PAGENUMTYPE Page,PAGENUMTYPE
  PageCount)
{
    return emu_Write_Page_Spare(write_data, block, Page, PageCount);
}

uint16  GLOB_LLD_Read_Page_Spare(byte* read_data, BLOCKNODE block,PAGENUMTYPE Page,PAGENUMTYPE
  PageCount)
{
    return  emu_Read_Page_Spare(read_data,block, Page, PageCount);
}

uint32  GLOB_LLD_Memory_Pool_Size(void)
{
    return CDMA_Memory_Pool_Size();
}

int GLOB_LLD_Mem_Config(byte * pMem)
{
    return CDMA_Mem_Config(pMem);
}
#endif // !ELDORA


#else // if not CMD_DMA, use old style parameters without tags
uint16  GLOB_LLD_Flash_Init(void)
{
    return emu_Flash_Init();
}

uint16  GLOB_LLD_Erase_Block(BLOCKNODE block_add)
{
    return  emu_Erase_Block(block_add);
}

uint16 GLOB_LLD_Write_Page_Main(byte* write_data,BLOCKNODE block,PAGENUMTYPE Page,PAGENUMTYPE PageCount)
{
    return emu_Write_Page_Main(write_data, block, Page, PageCount);
}

uint16  GLOB_LLD_Read_Page_Main(byte* read_data,BLOCKNODE block,PAGENUMTYPE Page,PAGENUMTYPE PageCount)
{
    return emu_Read_Page_Main(read_data, block, Page, PageCount);
}

#ifndef ELDORA
void GLOB_LLD_Enable_Disable_Interrupts(uint16 INT_ENABLE)
{
    emu_Enable_Disable_Interrupts(INT_ENABLE);
}

uint16 GLOB_LLD_Write_Page_Main_Spare(byte* write_data, BLOCKNODE block,PAGENUMTYPE Page,PAGENUMTYPE PageCount)
{
    return emu_Write_Page_Main_Spare(write_data, block, Page, PageCount);
}

uint16  GLOB_LLD_Read_Page_Main_Spare(byte* read_data,BLOCKNODE block,PAGENUMTYPE Page,PAGENUMTYPE PageCount)
{
    return  emu_Read_Page_Main_Spare(read_data,block, Page, PageCount);
}

uint16  GLOB_LLD_Write_Page_Spare(byte* write_data,BLOCKNODE block,PAGENUMTYPE Page,PAGENUMTYPE PageCount)
{
    return emu_Write_Page_Spare(write_data, block, Page, PageCount);
}

uint16  GLOB_LLD_Read_Page_Spare(byte* read_data, BLOCKNODE block,PAGENUMTYPE Page,PAGENUMTYPE PageCount)
{
    return  emu_Read_Page_Spare(read_data,block, Page, PageCount);
}

uint32  GLOB_LLD_Memory_Pool_Size(void)
{
    return 0;
}

int GLOB_LLD_Mem_Config(byte * pMem)
{
    return 0;
}
#endif // !ELDORA
#endif // CMD_DMA or not
#ifndef ELDORA
uint16  GLOB_LLD_Get_Bad_Block(BLOCKNODE block)
{
    return  emu_Get_Bad_Block(block);
}
#endif // !ELDORA
#endif // FLASH_EMU
/*&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&*/

/*&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&*/
#if FLASH_ESL     // vector all the LLD calls to the LLD_ESL code
#include "lld_esl.h"
#include "lld_cdma.h"

// common functions
uint16 GLOB_LLD_Flash_Reset (void)
{
  return esl_Flash_Reset ();
}

uint16 GLOB_LLD_Read_Device_ID (void)
{
  return esl_Read_Device_ID ();
}

/*&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
 * new APIs with tags to support CDMA
 *&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&*/
#if CMD_DMA
uint16 GLOB_LLD_Flash_Init ( uint16 Flags)
{
  if (Flags & LLD_CMD_FLAG_MODE_POLL)
  {
    return esl_Flash_Init();
  }
  else
  {
    return esl_CDMA_Flash_Init();
  }
}

uint16 GLOB_LLD_Erase_Block (BLOCKNODE block, byte TagCount, uint16 Flags)
{
  if (Flags & LLD_CMD_FLAG_MODE_POLL) {
    return esl_Erase_Block (block);
  }
  else {
    return CDMA_Data_CMD (TagCount, ERASE_CMD, 0, block, 0, 0, Flags);
  }
}

uint16 GLOB_LLD_Write_Page_Main (byte* data, BLOCKNODE block, PAGENUMTYPE page, PAGENUMTYPE count, byte TagCount)
{
  return CDMA_Data_CMD (TagCount, WRITE_MAIN_CMD, data, block, page, count, 0);
}

uint16 GLOB_LLD_Read_Page_Main (byte* data, BLOCKNODE block, PAGENUMTYPE page, PAGENUMTYPE  count, byte TagCount, uint16 Flags)
{
  if (Flags & LLD_CMD_FLAG_MODE_POLL) {
    return esl_Read_Page_Main (data, block, page, count);
  }
  else {
    return CDMA_Data_CMD (TagCount, READ_MAIN_CMD, data, block, page, count, Flags);
  }
}

uint16 GLOB_LLD_MemCopy_CMD (byte TagCount, byte* dest, byte* src, uint16 ByteCount,
        uint16 Flags)
{
  return CDMA_MemCopy_CMD (TagCount, dest, src, ByteCount, Flags);
}

uint16 GLOB_LLD_Execute_CMDs (uint16 count)
{
  return esl_CDMA_Execute_CMDs (count);
}

uint16 GLOB_LLD_Event_Status(void)
{
  return esl_CDMA_Event_Status();
}

#ifndef ELDORA
void GLOB_LLD_Enable_Disable_Interrupts (uint16 INT_ENABLE)
{
  esl_Enable_Disable_Interrupts (INT_ENABLE);
}

uint16 GLOB_LLD_Write_Page_Main_Spare (byte* write_data,
                                         BLOCKNODE block,
                                         PAGENUMTYPE Page,
                                         PAGENUMTYPE PageCount,
                                         byte TagCount,
                                         uint16 Flags)
{
  if (Flags & LLD_CMD_FLAG_MODE_POLL) {
    return esl_Write_Page_Main_Spare (write_data, block, Page, PageCount);
  }
  else {
    return CDMA_Data_CMD (TagCount, WRITE_MAIN_SPARE_CMD, write_data, block, Page, PageCount, Flags);
  }
}

uint16  GLOB_LLD_Read_Page_Main_Spare(byte* read_data,
                                        BLOCKNODE block,
                                        PAGENUMTYPE Page,
                                        PAGENUMTYPE PageCount,
                                        byte TagCount)
{
  return  esl_Read_Page_Main_Spare (read_data,block, Page, PageCount);
}

uint16  GLOB_LLD_Write_Page_Spare (byte* write_data, BLOCKNODE block, PAGENUMTYPE Page, PAGENUMTYPE PageCount)
{
    return esl_Write_Page_Spare (write_data, block, Page, PageCount);
}

uint16  GLOB_LLD_Read_Page_Spare (byte* read_data,
                                    BLOCKNODE block,
                                    PAGENUMTYPE Page,
                                    PAGENUMTYPE PageCount)
{
  return esl_Read_Page_Spare (read_data, block, Page, PageCount);
}

uint16  GLOB_LLD_Flash_Release(void)
{
    return 0;
}

uint32  GLOB_LLD_Memory_Pool_Size(void)
{
    return CDMA_Memory_Pool_Size();
}

int GLOB_LLD_Mem_Config(byte * pMem)
{
    return CDMA_Mem_Config(pMem);
}
#endif // !ELDORA

#else   // CMD_DMA
uint16  GLOB_LLD_Flash_Init(void)
{
  return esl_Flash_Init();
}

uint16  GLOB_LLD_Erase_Block(BLOCKNODE block_add)
{
  return  esl_Erase_Block(block_add);
}

uint16 GLOB_LLD_Write_Page_Main(byte* write_data,BLOCKNODE block,PAGENUMTYPE Page,PAGENUMTYPE PageCount)
{
    return esl_Write_Page_Main(write_data, block, Page, PageCount);
}

uint16  GLOB_LLD_Read_Page_Main(byte* read_data,BLOCKNODE block,PAGENUMTYPE Page,PAGENUMTYPE PageCount)
{
    return esl_Read_Page_Main(read_data, block, Page, PageCount);
}

#ifndef ELDORA
uint16  GLOB_LLD_Flash_Release(void)
{
    return 0;
}

uint16 GLOB_LLD_Write_Page_Main_Spare(byte* write_data, BLOCKNODE block,PAGENUMTYPE Page,PAGENUMTYPE PageCount)
{
    return esl_Write_Page_Main_Spare(write_data, block, Page, PageCount);
}

uint16  GLOB_LLD_Read_Page_Main_Spare(byte* read_data,BLOCKNODE block,PAGENUMTYPE Page,PAGENUMTYPE PageCount)
{
    return  esl_Read_Page_Main_Spare(read_data,block, Page, PageCount);
}

uint16  GLOB_LLD_Write_Page_Spare(byte* write_data,BLOCKNODE block,PAGENUMTYPE Page,PAGENUMTYPE PageCount)
{
    return esl_Write_Page_Spare(write_data, block, Page, PageCount);
}
uint16  GLOB_LLD_Read_Page_Spare(byte* read_data, BLOCKNODE block,PAGENUMTYPE Page,PAGENUMTYPE PageCount)
{
    return  esl_Read_Page_Spare(read_data,block, Page, PageCount);
}

uint16  GLOB_LLD_Event_Status(void)  // this is a NOP for now
{
    return PASS;
}

void GLOB_LLD_Enable_Disable_Interrupts(uint16 INT_ENABLE)  // this is a NOP
{
}

uint32  GLOB_LLD_Memory_Pool_Size(void)
{
    return 0;
}

int GLOB_LLD_Mem_Config(byte * pMem)
{
     return 0;
}
#endif // !ELDORA
#endif // !CMD_DMA
#endif // FLASH_ESL

/*&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&*/


/*&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&*/
#if FLASH_NAND     // vector all the LLD calls to the NAND controller code
#include "lld_nand.h"
#ifndef ELDORA
#include "flash.h"
#endif

// common functions for LLD_NAND
uint16  GLOB_LLD_Flash_Reset(void)
{
    return NAND_Flash_Reset();
}

uint16  GLOB_LLD_Read_Device_ID(void)
{
    return NAND_Read_Device_ID();
}

uint16    GLOB_LLD_UnlockArrayAll(void)
{
    return  NAND_UnlockArrayAll();
}

void GLOB_LLD_Enable_Disable_Interrupts(uint16 INT_ENABLE)
{
    NAND_LLD_Enable_Disable_Interrupts(INT_ENABLE);
}

uint16  GLOB_LLD_Flash_Init(void)
{
    return NAND_Flash_Init();
}

uint16  GLOB_LLD_Flash_Release(void)
{
	NAND_Flash_Release();
    return 0;
}

uint16  GLOB_LLD_Event_Status(void)
{
    return NAND_LLD_Event_Status();
}


uint16  GLOB_LLD_Erase_Block(BLOCKNODE block_add)
{
    return  NAND_Erase_Block(block_add);
}

uint16 GLOB_LLD_Write_Page_Main(byte* write_data,BLOCKNODE block,PAGENUMTYPE Page,PAGENUMTYPE PageCount)
{
    return NAND_Write_Page_Main(write_data,block,Page,PageCount);
}

uint16  GLOB_LLD_Read_Page_Main(byte* read_data,BLOCKNODE block,PAGENUMTYPE page,PAGENUMTYPE page_count)
{
    return NAND_Read_Page_Main(read_data,block,page,page_count);
}

#ifndef ELDORA
uint16 GLOB_LLD_Write_Page_Main_Spare(byte* write_data,BLOCKNODE block,PAGENUMTYPE Page,PAGENUMTYPE PageCount)
{
    return NAND_Write_Page_Main_Spare(write_data,block,Page,PageCount);
}

uint16 GLOB_LLD_Write_Page_Spare(byte* write_data,BLOCKNODE block,PAGENUMTYPE Page,PAGENUMTYPE PageCount)
{
    return NAND_Write_Page_Spare(write_data, block, Page, PageCount);
}
uint16  GLOB_LLD_Read_Page_Main_Spare_Raw(byte* read_data,BLOCKNODE block,PAGENUMTYPE page,PAGENUMTYPE page_count)
{
	return NAND_Read_Page_Main_Spare_Raw(read_data,block,page,page_count);
}

uint16  GLOB_LLD_Read_Page_Main_Spare(byte* read_data,BLOCKNODE block,PAGENUMTYPE page,PAGENUMTYPE page_count)
{
    return NAND_Read_Page_Main_Spare(read_data,block,page,page_count);
}

uint16  GLOB_LLD_Read_Page_Spare(byte* read_data, BLOCKNODE block,PAGENUMTYPE Page,PAGENUMTYPE PageCount)
{
    return  NAND_Read_Page_Spare(read_data,block, Page, PageCount);
}
uint16 GLOB_LLD_Get_Bad_Block_Raw(BLOCKNODE block)
{
    return  NAND_Get_Bad_Block_Raw(block);
}
uint16  GLOB_LLD_Get_Bad_Block(BLOCKNODE block)
{
    return  NAND_Get_Bad_Block(block);
}

uint32  GLOB_LLD_Memory_Pool_Size(void)
{
    return NAND_Memory_Pool_Size();
}

int GLOB_LLD_Mem_Config(byte * pMem)
{
    return NAND_Mem_Config(pMem);
}
#endif // !ELDORA


#endif // FLASH_NAND
/*&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&*/

// CMD DMA is not applicable for Eldora
#ifndef ELDORA

#if FLASH_CDMA     // vector all the LLD data calls to the LLD_CDMA module
                   // vector some other  LLD  calls to the LLD_CDMA module
                   // vector the common  LLD  calls to the LLD_NAND module
#include "lld_cdma.h"
#include "lld_nand.h"

uint16  GLOB_LLD_Flash_Reset(void)
{
    return NAND_Flash_Reset();
}

uint16  GLOB_LLD_Read_Device_ID(void)
{
    return NAND_Read_Device_ID();
}

uint16    GLOB_LLD_UnlockArrayAll(void)
{
    return  NAND_UnlockArrayAll();
}

void GLOB_LLD_Enable_Disable_Interrupts(uint16 INT_ENABLE)
{
    NAND_LLD_Enable_Disable_Interrupts(INT_ENABLE);
}

uint16  GLOB_LLD_Flash_Release(void)  // not used; NOP
{
    return 0;
}

uint16 GLOB_LLD_Flash_Init ( uint16 Flags)
{
  if (Flags & LLD_CMD_FLAG_MODE_POLL)
  {
    return NAND_Flash_Init();
  }
  else
    return CDMA_Flash_Init();
}


uint16  GLOB_LLD_Event_Status(void)
{
    return CDMA_Event_Status();
}

uint16  GLOB_LLD_MemCopy_CMD(byte TagCount, byte* dest, byte* src, uint16 ByteCount,
        uint16 Flags)
{
    return CDMA_MemCopy_CMD (TagCount, dest, src, ByteCount, Flags);
}

uint16  GLOB_LLD_Execute_CMDs(uint16 count)
{
    return CDMA_Execute_CMDs (count);
}


uint16  GLOB_LLD_Erase_Block          (BLOCKNODE block, byte TagCount, uint16 Flags)
{
    if(Flags & LLD_CMD_FLAG_MODE_POLL)
        return  NAND_Erase_Block(block);
    else
        return      CDMA_Data_CMD       (TagCount,      ERASE_CMD,   0,block,   0,    0, Flags);
}


uint16  GLOB_LLD_Write_Page_Main      (byte* data,BLOCKNODE block,PAGENUMTYPE page,PAGENUMTYPE count,byte TagCount)
{
    return  CDMA_Data_CMD       (TagCount,      WRITE_MAIN_CMD,data,block,page,count, 0);
}

uint16  GLOB_LLD_Read_Page_Main       (byte* data,BLOCKNODE block,PAGENUMTYPE page,PAGENUMTYPE count,byte
  TagCount,uint16 Flags)
{
    if(Flags & LLD_CMD_FLAG_MODE_POLL)
    {
       return NAND_Read_Page_Main(data,block, page,count);
    }
    else
        return  CDMA_Data_CMD       (TagCount,       READ_MAIN_CMD,data,block,page,count, Flags);}

    uint16 GLOB_LLD_Write_Page_Spare      (byte* data,BLOCKNODE block,PAGENUMTYPE page,PAGENUMTYPE count)
{
    return  NAND_Write_Page_Spare(data,block, page, count);
}

uint16  GLOB_LLD_Read_Page_Spare      (byte* data, BLOCKNODE block,PAGENUMTYPE page,PAGENUMTYPE count)
{
    return  NAND_Read_Page_Spare(data,block, page, count);
}

uint16 GLOB_LLD_Write_Page_Main_Spare (byte* data,BLOCKNODE block,PAGENUMTYPE page,PAGENUMTYPE count,byte
  TagCount,uint16 Flags)
{
    return  CDMA_Data_CMD       (TagCount,WRITE_MAIN_SPARE_CMD,data,block,page,count, Flags);
}

uint16  GLOB_LLD_Read_Page_Main_Spare (byte* data,BLOCKNODE block,PAGENUMTYPE page,PAGENUMTYPE count,byte TagCount)
{
    return  CDMA_Data_CMD       (TagCount, READ_MAIN_SPARE_CMD,data,block,page,count,LLD_CMD_FLAG_MODE_CDMA);
}

uint16  GLOB_LLD_Get_Bad_Block(BLOCKNODE block)
{
    return  NAND_Get_Bad_Block(block);
}

uint32  GLOB_LLD_Memory_Pool_Size(void)
{
    return CDMA_Memory_Pool_Size();
}

int GLOB_LLD_Mem_Config(byte * pMem)
{
    return CDMA_Mem_Config(pMem);
}
#endif // FLASH_CDMA

#endif // !ELDORA
/*&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&*/


// end of LLD.c
