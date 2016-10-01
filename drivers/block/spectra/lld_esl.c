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
#else
#include "flash.h"
#include "ffsdefs.h"
#endif

#include "lld.h"
#include "lld_esl.h"
#if CMD_DMA
#include "lld_cdma.h"
#endif


#if FLASH_ESL
uint32 GLOB_totalUsedBanks;

#if CMD_DMA
extern PENDING_CMD GLOB_PendingCMD[MAX_DESCRIPTORS + LLD_NUM_FLASH_CHANNELS];
#endif

#include "glob_fc_esl_reg.h"
#ifdef GLOB_ESL_TEST
uint32 *Reg_Data_Port;
uint32 *Reg_Block_Number;
uint32 *Reg_Page_Number;
uint32 *Reg_Interrupt_Status;
uint32 *Reg_Device_Param_0;

uint32 *Reg_Active_Src_ID;
uint32 *Reg_Perm_Src_ID_0;
uint32 *Reg_Max_Blk_Addr_0;
uint32 *Reg_Min_Blk_Addr_0;
uint32 *Reg_Min_Max_Bank_0;
uint32 *Reg_Perm_Src_ID_1;
uint32 *Reg_Max_Blk_Addr_1;
uint32 *Reg_Min_Blk_Addr_1;
uint32 *Reg_Min_Max_Bank_1;

extern void issueCpuWrite (unsigned int, unsigned int);
extern void issueCpuRead (unsigned int, unsigned int);
extern void issueCpuInterruptWrite ();
extern void issueCpuInterruptRead ();
#else
#define Reg_Data_Port_ADD        REG_DATA_PORT_BEGIN
#define Reg_Block_Number_ADD     REG_BLOCK_NUMBER
#define Reg_Page_Number_ADD      REG_PAGE_NUMBER
#define Reg_Interrupt_Status_ADD REG_INTERRUPT_STATUS
#define Reg_Device_Param_0_ADD   REG_DEVICE_PARAM_0

#define Reg_Active_Src_ID_ADD    REG_ACTIVE_SRC_ID;
#define Reg_Perm_Src_ID_0_ADD	 REG_PERM_SRC_ID_0;
#define Reg_Max_Blk_Addr_0_ADD   REG_MAX_BLK_ADDR_0;
#define Reg_Min_Blk_Addr_0_ADD	 REG_MIN_BLK_ADDR_0;
#define Reg_Min_Max_Bank_0_ADD	 REG_MIN_MAX_BANK_0;
#define Reg_Perm_Src_ID_1_ADD 	 REG_PERM_SRC_ID_1;
#define Reg_Max_Blk_Addr_1_ADD   REG_MIN_BLK_ADDR_1;
#define Reg_Min_Blk_Addr_1_ADD   REG_MAX_BLK_ADDR_1;
#define Reg_Min_Max_Bank_1_ADD   REG_MIN_MAX_BANK_1;
byte   *Reg_Data_Port	     = (byte*)Reg_Data_Port_ADD;
uint32 *Reg_Block_Number     = (uint32*)Reg_Block_Number_ADD;
uint32 *Reg_Page_Number	     = (uint32*)Reg_Page_Number_ADD;
uint32 *Reg_Interrupt_Status = (uint32*)Reg_Interrupt_Status_ADD;
uint32 *Reg_Device_Param_0   =  (uint32*)Reg_Device_Param_0_ADD;

uint32 *Reg_Active_Src_ID    = (uint32*)Reg_Active_Src_ID_ADD;
uint32 *Reg_Perm_Src_ID_0    = (uint32*)Reg_Perm_Src_ID_0_ADD;
uint32 *Reg_Max_Blk_Addr_0   = (uint32*)Reg_Max_Blk_Addr_0_ADD;
uint32 *Reg_Min_Blk_Addr_0   = (uint32*)Reg_Min_Blk_Addr_0_ADD;
uint32 *Reg_Min_Max_Bank_0   = (uint32*)Reg_Min_Max_Bank_0_ADD;
uint32 *Reg_Perm_Src_ID_1    = (uint32*)Reg_Perm_Src_ID_1_ADD;
uint32 *Reg_Max_Blk_Addr_1   = (uint32*)Reg_Max_Blk_Addr_1_ADD;
uint32 *Reg_Min_Blk_Addr_1   = (uint32*)Reg_Min_Blk_Addr_1_ADD;
uint32 *Reg_Min_Max_Bank_1   = (uint32*)Reg_Min_Max_Bank_1_ADD;

#endif

#ifdef ELDORA
extern DEVICE_INFO GLOB_DeviceInfo;
#endif

/*&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
* Function:     esl_Flash_Init
* Inputs:       none
* Outputs:      PASS=0 (notice 0=ok here)
* Description:          Initializes the flash.
*
***********************************************************************/
uint16 esl_Flash_Init()
{
    return PASS;
}



/*&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
* Function:     esl_Read_Device_ID
* Inputs:       none
* Outputs:      PASS=1 FAIL=0
* Description:  Reads the info from the controller registers.
*               Sets up GLOB_DeviceInfo structure with device parameters
***********************************************************************/

uint16 esl_Read_Device_ID()
{
    uint16  status = PASS;

    GLOB_DeviceInfo.wDeviceMaker    = 0xBB;
    GLOB_DeviceInfo.wDeviceType     = 4;
    GLOB_DeviceInfo.wTotalBlocks    = ESL_BLOCK_COUNT;
    GLOB_DeviceInfo.wPagesPerBlock  = ESL_BLOCK_SIZE;
    GLOB_DeviceInfo.wPageSize       = ESL_PAGE_SIZE;
    GLOB_DeviceInfo.wPageDataSize   = ESL_PAGE_SIZE - ESL_SPARE_SIZE;
    GLOB_DeviceInfo.wPageSpareSize  = ESL_SPARE_SIZE;
    GLOB_DeviceInfo.wBlockSize      = GLOB_DeviceInfo.wPageSize * ESL_BLOCK_SIZE;
    GLOB_DeviceInfo.wBlockDataSize  = GLOB_DeviceInfo.wPageDataSize * ESL_BLOCK_SIZE;
    GLOB_DeviceInfo.wDataBlockNum   = (BLOCKNODE)(GLOB_DeviceInfo.wTotalBlocks   - SPECTRA_START_BLOCK);

#ifdef GLOB_ESL_TEST
    issueRegRead(REG_DEVICE_PARAM_0, Reg_Device_Param_0);
#endif

    GLOB_DeviceInfo.MLCDevice       = *Reg_Device_Param_0 &  0x000c;
    GLOB_totalUsedBanks = 1;
#ifdef GLOB_ESL_TEST

    issueRegRead(REG_PERM_SRC_ID_0, Reg_Perm_Src_ID_0);
    issueRegRead(REG_MAX_BLK_ADDR_0, Reg_Max_Blk_Addr_0);
    issueRegRead(REG_MIN_BLK_ADDR_0, Reg_Min_Blk_Addr_0);
    issueRegRead(REG_MIN_MAX_BANK_0, Reg_Min_Max_Bank_0);

    issueRegRead(REG_PERM_SRC_ID_1, Reg_Perm_Src_ID_1);
    issueRegRead(REG_MIN_BLK_ADDR_1, Reg_Max_Blk_Addr_1);
    issueRegRead(REG_MAX_BLK_ADDR_1, Reg_Min_Blk_Addr_1);
    issueRegRead(REG_MIN_MAX_BANK_1, Reg_Min_Max_Bank_1);
#endif


    if ((*Reg_Perm_Src_ID_0 & PERM_SRC_ID_0__SRCID) == SPECTRA_PARTITION_ID)
    {

	GLOB_DeviceInfo.wSpectraStartBlock    = ( (*Reg_Min_Max_Bank_0 & MIN_MAX_BANK_0__MIN_VALUE) * GLOB_DeviceInfo.wTotalBlocks )
	    +  (*Reg_Min_Blk_Addr_0 & MIN_BLK_ADDR_0__VALUE);

	GLOB_DeviceInfo.wSpectraEndBlock     = ( ( (*Reg_Min_Max_Bank_0& MIN_MAX_BANK_0__MAX_VALUE) >>  2) * GLOB_DeviceInfo.wTotalBlocks )
	    +  (*Reg_Max_Blk_Addr_0 & MAX_BLK_ADDR_0__VALUE);

	GLOB_DeviceInfo.wTotalBlocks =  GLOB_DeviceInfo.wTotalBlocks * GLOB_totalUsedBanks;

	if(GLOB_DeviceInfo.wSpectraEndBlock >= GLOB_DeviceInfo.wTotalBlocks)
	{

	    GLOB_DeviceInfo.wSpectraEndBlock  = GLOB_DeviceInfo.wTotalBlocks - 1;
	}

	GLOB_DeviceInfo.wDataBlockNum =(BLOCKNODE)(GLOB_DeviceInfo.wSpectraEndBlock - GLOB_DeviceInfo.wSpectraStartBlock + 1);
    }

    if ((*Reg_Perm_Src_ID_1 & PERM_SRC_ID_1__SRCID) == SPECTRA_PARTITION_ID)
    {
	GLOB_DeviceInfo.wSpectraStartBlock    = ( (*Reg_Min_Max_Bank_1& MIN_MAX_BANK_1__MIN_VALUE) * GLOB_DeviceInfo.wTotalBlocks )
	    +  (*Reg_Min_Blk_Addr_1 & MIN_BLK_ADDR_1__VALUE);

	GLOB_DeviceInfo.wSpectraEndBlock     = ( ( (*Reg_Min_Max_Bank_1& MIN_MAX_BANK_1__MAX_VALUE) >>  2) * GLOB_DeviceInfo.wTotalBlocks )
	    +  (*Reg_Max_Blk_Addr_1 & MAX_BLK_ADDR_1__VALUE);

	GLOB_DeviceInfo.wTotalBlocks =  GLOB_DeviceInfo.wTotalBlocks * GLOB_totalUsedBanks;


	if(GLOB_DeviceInfo.wSpectraEndBlock >= GLOB_DeviceInfo.wTotalBlocks)
	{

	    GLOB_DeviceInfo.wSpectraEndBlock  = GLOB_DeviceInfo.wTotalBlocks - 1;
	}

	GLOB_DeviceInfo.wDataBlockNum =(BLOCKNODE)(GLOB_DeviceInfo.wSpectraEndBlock - GLOB_DeviceInfo.wSpectraStartBlock + 1);
    }

    return status;
}


/*&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
* Function:     esl_Write_Page_Main
* Inputs:       Write buffer address pointer
*               Block number
*               Page  number
*               Number of pages to process
* Outputs:      PASS=0 (notice 0=ok here)
* Description:  Write the data in the buffer to main area of flash
*
***********************************************************************/
uint16 esl_Write_Page_Main(byte * write_data, BLOCKNODE Block, uint16 Page, uint16 PageCount)
{
    int i,j;
    int status = PASS;
    *Reg_Block_Number = Block;
    *Reg_Page_Number  = Page;

    if(Block >= GLOB_DeviceInfo.wTotalBlocks)
    {
	   status = FAIL;
    }

    if(Page+PageCount > GLOB_DeviceInfo.wPagesPerBlock)
    {
	   status = FAIL;
    }

    if (PASS == status)
    {
        for(i = 0; i < PageCount; i++)
        {
            *Reg_Page_Number = Page;

            for(j = 0; j < GLOB_DeviceInfo.wPageDataSize; j++)
            {
                Reg_Data_Port[j] = write_data[j];
            }
            write_data +=GLOB_DeviceInfo.wPageDataSize;
            Page++;
#ifdef GLOB_ESL_TEST
        /* indicate the greeensocs master to issue write to _one_ flash page
         * and wait fot the page write to complete
         */
            issueCpuWrite (0, GLOB_DeviceInfo.wPageDataSize);
#endif
        }
    }

    return status;
}

/*&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
* Function:     esl_Read_Page_Main
* Inputs:       Read buffer address pointer
*               Block number
*               Page  number
*               Number of pages to process
* Outputs:      PASS=0 (notice 0=ok here)
* Description:  Read the data from the flash main area to the buffer
*
***********************************************************************/
uint16 esl_Read_Page_Main(byte * read_data, BLOCKNODE Block, uint16 Page, uint16  PageCount)
{
    int i,j, status = PASS;

    *Reg_Block_Number = Block;
    *Reg_Page_Number  = Page;

    if(Block >= GLOB_DeviceInfo.wTotalBlocks)
    {
	   status = FAIL;
    }

    if(Page+PageCount > GLOB_DeviceInfo.wPagesPerBlock)
    {
	   status = FAIL;
    }

    if (PASS == status)
    {
        for (i = 0; i < PageCount; i++)
        {
            *Reg_Page_Number  = Page;

#ifdef GLOB_ESL_TEST
        /* indicate the master to issue a read command for _one_ page and wait
         * untill the page is read
         */
            issueCpuRead (0, GLOB_DeviceInfo.wPageDataSize);
#endif

            for(j = 0; j < GLOB_DeviceInfo.wPageDataSize; j++)
            {
                read_data[j] = Reg_Data_Port[j];
            }

            read_data +=GLOB_DeviceInfo.wPageDataSize;
            Page++;
        }
    }
    return status;
}
/*&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
* Function:     esl_Erase_Block
* Inputs:       Address
* Outputs:      PASS=0 (notice 0=ok here)
* Description:          Erase a block
*
***********************************************************************/
uint16 esl_Erase_Block(BLOCKNODE block_add)
{
    return PASS;
}

/*&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
* Function:     esl_Flash_Reset
* Inputs:       none
* Outputs:      PASS=0 (notice 0=ok here)
* Description:          Reset the flash
*
***********************************************************************/
uint16 esl_Flash_Reset (void)
{
    return PASS;
}


#ifndef ELDORA

/*&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
* Function:     esl_Write_Page_Main_Spare
* Inputs:       Write buffer
*                       address
*                       buffer length
* Outputs:      PASS=0 (notice 0=ok here)
* Description:          Write the buffer to main+spare area of flash
*
***********************************************************************/
uint16 esl_Write_Page_Main_Spare(byte* write_data,BLOCKNODE Block,uint16 Page,uint16 page_count)
{
    uint16  i,j, status = PASS;

    *Reg_Block_Number = Block;
    *Reg_Page_Number  = Page;

    print(" Write Page - block %d page %d \n",Block,Page);

    if(Block >= GLOB_DeviceInfo.wTotalBlocks) {
	printstr("Write Page:Error has occured: Block Address too big\n");
	status = FAIL;
    }

    if(Page+page_count > GLOB_DeviceInfo.wPagesPerBlock) {
	printstr("Write Page:Error has occured: Page Address too big\n");
	status = FAIL;
    }

    if (PASS == status)
    {
	for(i=0; i<page_count; i++)
	{

	    *Reg_Page_Number  = Page;
	    print("\n Write Page Main+Spare - No. of pages %d block %d start page %d\n",page_count,Block,Page);

	    for(j=0; j<GLOB_DeviceInfo.wPageSize; j++)
	    {
		Reg_Data_Port[j] = write_data[j];
	    }

	    write_data +=GLOB_DeviceInfo.wPageSize;
	    Page++;
#ifdef GLOB_ESL_TEST
            /* indicate the greeensocs master to issue write to _one_ flash page
             * and wait fot the page write to complete
             */
            issueCpuWrite (0, GLOB_DeviceInfo.wPageSize);
#endif
	}
    }

    return status;
}



/*&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
* Function:     esl_Write_Page_Spare
* Inputs:       Write buffer
*                       Address
*                       buffer size
* Outputs:      PASS=0 (notice 0=ok here)
* Description:          Write the buffer in the spare area
*
***********************************************************************/
uint16 esl_Write_Page_Spare(byte* write_data,BLOCKNODE Block,uint16 Page,uint16 PageCount)
{
    uint16  i, status = PASS;

    *Reg_Block_Number = Block;
    *Reg_Page_Number  = Page;


    if(Block >= GLOB_DeviceInfo.wTotalBlocks) {
	printstr("Read Page:Error has occured: Block Address too big\n");
	status = FAIL;
    }

    if(Page+PageCount > GLOB_DeviceInfo.wPagesPerBlock) {
	printstr("Read Page:Error has occured: Page Address too big\n");
	status = FAIL;
    }


    if (PASS == status)
    {
	print(" Write Page Spare- block %d page %d \n",Block,Page);

	for(i=GLOB_DeviceInfo.wPageDataSize; i<GLOB_DeviceInfo.wPageSize; i++)
	{
	    Reg_Data_Port[i] = write_data[i];
	}
#ifdef GLOB_ESL_TEST
        /* indicate the greeensocs master to issue write to _one_ flash page
         * and wait fot the page write to complete
         */
        issueCpuWrite (GLOB_DeviceInfo.wPageDataSize, (GLOB_DeviceInfo.wPageSize - GLOB_DeviceInfo.wPageDataSize));
#endif
    }

    return status;
}


/*&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
* Function:     esl_Read_Page_Main_Spare
* Inputs:       Write Buffer
*                       Address
*                       Buffer size
* Outputs:      PASS=0 (notice 0=ok here)
* Description:          Read from flash main+spare area
*
***********************************************************************/
uint16 esl_Read_Page_Main_Spare(byte* read_data,BLOCKNODE Block,uint16 Page,uint16 PageCount)
{
    int i,j, status = PASS;

    *Reg_Block_Number = Block;
    *Reg_Page_Number  = Page;

    if(Block+1 > GLOB_DeviceInfo.wTotalBlocks) {
	printstr("Read Page:Error has occured: Block Address too big\n");
	status = FAIL;
    }

    if(Page+PageCount > GLOB_DeviceInfo.wPagesPerBlock) {
	printstr("Read Page:Error has occured: Page Address too big\n");
	status = FAIL;
    }

    if (PASS == status)
    {
	for(i=0; i<PageCount; i++) {

	    *Reg_Page_Number  = Page;
	    print("\n Read Page Main+Spare - No. of pages %d block %d start page %d\n",PageCount,Block,Page);

#ifdef GLOB_ESL_TEST
            /* indicate the master to issue a read command for _one_ page and wait
             * untill the page is read
             */
            issueCpuRead (0, GLOB_DeviceInfo.wPageSize);
#endif

	    for(j = 0; j < GLOB_DeviceInfo.wPageSize; j++)
	    {
		read_data[j] = Reg_Data_Port[j];
	    }

	    read_data +=GLOB_DeviceInfo.wPageSize;
	    Page++;
	}
    }
    return status;
}


/*&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
* Function:     esl_Read_Page_Spare
* Inputs:       Write Buffer
*                       Address
*                       Buffer size
* Outputs:      PASS=0 (notice 0=ok here)
* Description:          Read data from the spare area
*
***********************************************************************/
uint16 esl_Read_Page_Spare(byte* read_data, BLOCKNODE Block,uint16 Page,uint16 PageCount)
{
    int i, status = PASS;

    *Reg_Block_Number = Block;
    *Reg_Page_Number  = Page;

    if(Block >= GLOB_DeviceInfo.wTotalBlocks) {
	printstr("Read Page:Error has occured: Block Address too big\n");
	status = FAIL;
    }

    if(Page+PageCount > GLOB_DeviceInfo.wPagesPerBlock) {
	printstr("Read Page:Error has occured: Page Address too big\n");
	status = FAIL;
    }

    if (PASS == status) {
	print(" Read Page Spare- block %d page %d\n",Block,Page);

#ifdef GLOB_ESL_TEST
        /* indicate the master to issue a read command for _one_ page and wait
         * untill the page is read
         */
        issueCpuRead (GLOB_DeviceInfo.wPageDataSize, (GLOB_DeviceInfo.wPageSize - GLOB_DeviceInfo.wPageDataSize));
#endif

	for(i=GLOB_DeviceInfo.wPageDataSize; i<GLOB_DeviceInfo.wPageSize; i++)
	{
	    read_data[i] = Reg_Data_Port[i];
	}
    }

    return PASS;
}


/*&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
* Function:     esl_Enable_Disable_Interrupts
* Inputs:       enable or disable
* Outputs:      none
* Description:  NOP
***********************************************************************/
void  esl_Enable_Disable_Interrupts(uint16 INT_ENABLE)
{
}


#if CMD_DMA
/***********************************
 * Support for CDMA functions
 ************************************
 *       esl_CDMA_Flash_Init
 *           CDMA_process_data command   (use LLD_CDMA)
 *           CDMA_MemCopy_CMD            (use LLD_CDMA)
 *       esl_CDMA_execute all commands
 *       esl_CDMA_Event_Status
 *************************************/

/*&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
 * Function:     CDMA_Flash_Init
 * Inputs:       none
 * Outputs:      PASS=0 (notice 0=ok here)
 * Description:  This should be called at power up.
 *               It disables interrupts and clears status bits
 *               issues flash reset command
 *               configures the controller registers
 *               It sets the interrupt mask and enables interrupts
 *               It pre-builds special descriptors
 ***********************************************************************/
uint16 esl_CDMA_Flash_Init (void)
{
  uint16 i, j = MAX_DESCRIPTORS + LLD_NUM_FLASH_CHANNELS;
  esl_Flash_Init ();
  for (i = 0; i < j; i++) {
    GLOB_PendingCMD[i].CMD           = 0;
    GLOB_PendingCMD[i].Tag           = 0;
    GLOB_PendingCMD[i].DataAddr      = 0;
    GLOB_PendingCMD[i].Block         = 0;
    GLOB_PendingCMD[i].Page          = 0;
    GLOB_PendingCMD[i].PageCount     = 0;
    GLOB_PendingCMD[i].DataDestAddr  = 0;
    GLOB_PendingCMD[i].DataSrcAddr   = 0;
    GLOB_PendingCMD[i].MemCopyByteCnt= 0;
    GLOB_PendingCMD[i].SBDCmdIndex   = 0;
    GLOB_PendingCMD[i].ChanSync[0]   = 0;
    GLOB_PendingCMD[i].ChanSync[1]   = 0;
    GLOB_PendingCMD[i].ChanSync[2]   = 0;
    GLOB_PendingCMD[i].ChanSync[3]   = 0;
    GLOB_PendingCMD[i].ChanSync[4]   = 0;
    GLOB_PendingCMD[i].Status        = 0;

  }

  return PASS;
}

/*&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
 * Function:     CDMA_Data_Cmd
 *               use the real function in LLD_CDMA
 ***********************************************************************/


/*&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
 * Function:     CDMA_MemCopy_CMD
 *               use the real function in LLD_CDMA
 ***********************************************************************/


/*&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
* Function:     CDMA_Execute_CMDs
* Inputs:       tag_count:  the number of pending cmds to do
* Outputs:      PASS/FAIL
* Description:  execute each command in the pending CMD array
*
* Note: 1. does not support "special" command sequences
*       2. always posts good status for every command
*       3. emulates the real CDMA operation
***********************************************************************/
uint16  esl_CDMA_Execute_CMDs (uint16 tag_count)
{
  uint16  i, j;
  byte    CMD  ;
  byte*   data ;
  uint16  block;
  uint16  page ;
  uint16  count;
  uint16  status = PASS;

  for (i = 0; i < GLOB_totalUsedBanks; i++) {
    GLOB_PendingCMD[i].CMD   = DUMMY_CMD;
    GLOB_PendingCMD[i].Tag   = 0xFF;
    GLOB_PendingCMD[i].SBDCmdIndex   = 0xFF;
    GLOB_PendingCMD[i].Block = i * (GLOB_DeviceInfo.wTotalBlocks / GLOB_totalUsedBanks);
    for (j = 0; j <= LLD_NUM_FLASH_CHANNELS; j++) {
      GLOB_PendingCMD[i].ChanSync[j] = 0;
    }
  }
  CDMA_AddSyncPoints (tag_count);

#ifdef VERBOSE
  PrintGLOB_PendingCMDs (tag_count);
  CDMA_CheckSyncPoints (tag_count);
#endif
  j = tag_count + LLD_NUM_FLASH_CHANNELS;
  for (i = LLD_NUM_FLASH_CHANNELS; i < j; i++) {
    CMD   = GLOB_PendingCMD[i].CMD;
    data  = GLOB_PendingCMD[i].DataAddr;
    block = GLOB_PendingCMD[i].Block;
    page  = GLOB_PendingCMD[i].Page;
    count = GLOB_PendingCMD[i].PageCount;

    switch (CMD) {
    case ERASE_CMD:
      esl_Erase_Block (block);
      GLOB_PendingCMD[i].Status = PASS;
      break;
    case WRITE_MAIN_CMD:
      esl_Write_Page_Main (data, block, page, count);
      GLOB_PendingCMD[i].Status = PASS;
      break;
    case WRITE_MAIN_SPARE_CMD:
      esl_Write_Page_Main_Spare (data, block, page, count);
      GLOB_PendingCMD[i].Status = PASS;
      break;
    case READ_MAIN_CMD:
      esl_Read_Page_Main (data, block, page, count);
      GLOB_PendingCMD[i].Status = PASS;
      break;
    case MEMCOPY_CMD:
      memcpy (GLOB_PendingCMD[i].DataDestAddr, GLOB_PendingCMD[i].DataSrcAddr, GLOB_PendingCMD[i].MemCopyByteCnt);
    case DUMMY_CMD:
      GLOB_PendingCMD[i].Status = PASS;
      break;
    default:
      GLOB_PendingCMD[i].Status = FAIL;
      break;
    }
  }
  /* Write to the interrupt status register to generate an interrupt
   */
  *Reg_Interrupt_Status = 1;

#ifdef GLOB_ESL_TEST
  issueCpuInterruptWrite ();
#endif

  return status;
}


/*&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
 * Function:     esl_Event_Status
 * Inputs:       none
 * Outputs:      Event_Status code
 * Description:
 ***********************************************************************/
uint16  esl_CDMA_Event_Status (void)
{
  unsigned int i, j, interruptStatus;

#ifdef GLOB_ESL_TEST
  issueCpuInterruptRead ();
#endif
  interruptStatus = *Reg_Interrupt_Status;
  if (interruptStatus) {
    *Reg_Interrupt_Status = 0;

#ifdef GLOB_ESL_TEST
    issueCpuInterruptWrite ();
#endif
  }

  /* clear the pending CMD array
   */
  j = MAX_DESCRIPTORS + LLD_NUM_FLASH_CHANNELS;
  for (i = 0; i < j; i++) {
    GLOB_PendingCMD[i].CMD              = 0;
    GLOB_PendingCMD[i].Tag              = 0;
    GLOB_PendingCMD[i].DataAddr         = 0;
    GLOB_PendingCMD[i].Block            = 0;
    GLOB_PendingCMD[i].Page             = 0;
    GLOB_PendingCMD[i].PageCount        = 0;
    GLOB_PendingCMD[i].DataDestAddr     = 0;
    GLOB_PendingCMD[i].DataSrcAddr      = 0;
    GLOB_PendingCMD[i].SBDCmdIndex     =  0;
    GLOB_PendingCMD[i].MemCopyByteCnt   = 0;
    GLOB_PendingCMD[i].ChanSync[0]      = 0;
    GLOB_PendingCMD[i].ChanSync[1]      = 0;
    GLOB_PendingCMD[i].ChanSync[2]      = 0;
    GLOB_PendingCMD[i].ChanSync[3]      = 0;
    GLOB_PendingCMD[i].ChanSync[4]      = 0;
    GLOB_PendingCMD[i].Status           = 0;
  }
  /* always return PASS
   */
  return (EVENT_PASS);
}


#endif // CMD_DMA
#endif // !ELDORA
#endif
