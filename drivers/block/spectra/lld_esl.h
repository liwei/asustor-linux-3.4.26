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



#ifndef _LLD_ESL_
#define _LLD_ESL_

#ifdef ELDORA
#include "defs.h"
#else
#include "ffsport.h"
#include "ffsdefs.h"
#endif

#ifdef __cplusplus
extern "C"
{
#endif

#if CMD_DMA
extern uint16 esl_CDMA_Flash_Init (void);
extern uint16 esl_CDMA_Execute_CMDs (uint16 tag_count);
extern uint16 esl_CDMA_Event_Status (void);
extern void esl_Enable_Disable_Interrupts (uint16 INT_ENABLE);
#endif

// prototypes: ESL API functions
extern uint16   esl_Flash_Reset (void);
extern uint16   esl_Flash_Init (void);
extern uint16   esl_Read_Device_ID (void);
extern uint16   esl_Erase_Block (BLOCKNODE block_addr);
extern uint16   esl_Write_Page_Main(byte * write_data, BLOCKNODE block, uint16 Page, uint16 PageCount);
extern uint16   esl_Read_Page_Main(byte * read_data, BLOCKNODE block, uint16 Page, uint16 PageCount);

#ifndef ELDORA
extern uint16   esl_Write_Page_Main_Spare(byte * write_data,  BLOCKNODE block, uint16 Page, uint16 PageCount);
extern uint16   esl_Write_Page_Spare (byte * write_data,BLOCKNODE block, uint16 Page, uint16 PageCount);
extern uint16   esl_Read_Page_Main_Spare(byte * read_data,BLOCKNODE block, uint16 Page, uint16 PageCount);
extern uint16   esl_Read_Page_Spare (byte * read_data, BLOCKNODE block, uint16 Page, uint16 PageCount);
#endif // !ELDORA


#ifdef __cplusplus
}
#endif

#endif //_LLD_ESL_
