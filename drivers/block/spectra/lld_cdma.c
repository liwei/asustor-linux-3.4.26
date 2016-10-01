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


// note: compile with LLD_NAND.C as it contains some common functions
#include "spectraswconfig.h"
#include "lld.h"
#include "lld_nand.h"
#include "lld_cdma.h"
#include "lld_emu.h"
#include "flash.h"
#include "NAND_Regs_4.h"


#if CMD_DMA

#define         MODE_02             (0x2 << 26)
#define         MAX_DESC_PER_CHANNEL        (MAX_DESCRIPTORS*3 + MAX_SYNC_POINTS  +2)

#define CALC_CDMA_NEXT_PTR(CDMA_Descriptor, c, d) \
        (unsigned int)( (uint32)GLOB_MEMMAP_TOBUS((uint32 *)CDMA_Descriptor) \
                      + ( sizeof(CDMA_DESCRIPTOR)                              \
                        * ( (c * MAX_DESC_PER_CHANNEL)                         \
                          + d + 1                                              \
                          )                                                    \
                        )                                                      \
                      )

extern uint16 GLOB_init_firsttime_done;
extern uint32 GLOB_totalUsedBanks;

// Local function definition
#if FLASH_CDMA
static void ResetSyncModule(void);
#endif

// This section is shared between lld_emu.c and lld_cdma.c
PENDING_CMD     GLOB_PendingCMD[MAX_DESCRIPTORS + LLD_NUM_FLASH_CHANNELS];
//  command is sent.  This is global so FTL can check
//  final cmd results

/////// Extern variables
extern uint16   FIQ_FLAG;
extern byte     *g_pMemPoolFree;
extern uint16   conf_parameters[];
extern uint32   GLOB_valid_banks[LLD_MAX_FLASH_BANKS];

//////// Global variables ////////
#pragma alignvar (4)
CDMA_DESCRIPTOR      (*CDMA_Descriptor)[MAX_DESC_PER_CHANNEL];
#pragma alignvar (4)
MEM_COPY_DESCRIPTOR  (*MemCopyDescriptor)[MAX_DESC_PER_CHANNEL];

uint16  dcount[LLD_NUM_FLASH_CHANNELS];


extern uint16      InterruptEnableMask;
extern uint16      CMD_DMA_InterruptEnableMask;

/*&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
* Function:     CDMA_Data_Cmd
* Inputs:       tag (0-255)
*               cmd code (aligned for hw)
*               data: pointer to source or destination
*               block: block address
*               page: page address
*               count: num pages to transfer
* Outputs:      PASS
* Description:  This function takes the parameters and puts them
*                   into the "pending commands" array.
*               It does not parse or validate the parameters.
*               The array index is same as the tag.
*&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&*/
uint16  CDMA_Data_CMD (byte tag, byte CMD, byte* data, BLOCKNODE block, uint16 page, uint16 count, uint16 flags)
{
    int i;

    debug_boundary_error(block, GLOB_DeviceInfo.wTotalBlocks, tag);
    debug_boundary_error(count, GLOB_DeviceInfo.wPagesPerBlock+1, tag);
    debug_boundary_error(tag, 252, 0);
    tag += LLD_NUM_FLASH_CHANNELS;
    GLOB_PendingCMD[tag].Tag             = tag - LLD_NUM_FLASH_CHANNELS;
    GLOB_PendingCMD[tag].CMD             = CMD;
    GLOB_PendingCMD[tag].DataAddr        = data;
    GLOB_PendingCMD[tag].Block           = block;
    GLOB_PendingCMD[tag].Page            = page;
    GLOB_PendingCMD[tag].PageCount       = count;
    GLOB_PendingCMD[tag].DataDestAddr    = 0x00;   // not used for data commands
    GLOB_PendingCMD[tag].DataSrcAddr     = 0x00;   // not used for data commands
    GLOB_PendingCMD[tag].MemCopyByteCnt  = 0x00;   // not used for data commands
    GLOB_PendingCMD[tag].Flags            = flags;
    GLOB_PendingCMD[tag].SBDCmdIndex      = g_SBDCmdIndex;

    for (i=0; i<=LLD_NUM_FLASH_CHANNELS; i++)
        GLOB_PendingCMD[tag].ChanSync[i]   = 0;

    GLOB_PendingCMD[tag].Status          = 0xB0B;

#if FLASH_CDMA
    switch (CMD) {
    case WRITE_MAIN_SPARE_CMD:
        NAND_Conv_Main_Spare_Data_Log2Phy_Format(data, count);
        break;
    case WRITE_SPARE_CMD:
        NAND_Conv_Spare_Data_Log2Phy_Format(data);
        break;
    default:
        break;
    }
#endif
    return PASS;
}

/*&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
* Function:     CDMA_MemCopy_CMD
* Inputs:       tag (0-255)
*               dest: pointer to destination
*               src:  pointer to source
*               count: num bytes to transfer
* Outputs:      PASS
* Description:  This function takes the parameters and puts them
*                   into the "pending commands" array.
*               It does not parse or validate the parameters.
*               The array index is same as the tag.
*&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&*/
uint16  CDMA_MemCopy_CMD (byte tag, byte* dest, byte* src, uint16 ByteCount, uint16 flags)
{
    int i;
    print("CDMA MemC Command called tag=%u\n", tag);

    debug_boundary_error(tag, 252, 0);
    tag += LLD_NUM_FLASH_CHANNELS;
    GLOB_PendingCMD[tag].Tag             = tag - LLD_NUM_FLASH_CHANNELS;
    GLOB_PendingCMD[tag].CMD             = MEMCOPY_CMD;
    GLOB_PendingCMD[tag].DataAddr        = 0x00;   // not used for MemCopy commands
    GLOB_PendingCMD[tag].Block           = 0x00;   // not used for MemCopy commands
    GLOB_PendingCMD[tag].Page            = 0x00;   // not used for MemCopy commands
    GLOB_PendingCMD[tag].PageCount       = 0x00;   // not used for MemCopy commands
    GLOB_PendingCMD[tag].DataDestAddr    = dest;
    GLOB_PendingCMD[tag].DataSrcAddr     = src;
    GLOB_PendingCMD[tag].MemCopyByteCnt  = ByteCount;
    GLOB_PendingCMD[tag].Flags            = flags;
    GLOB_PendingCMD[tag].SBDCmdIndex      = g_SBDCmdIndex;

    for (i=0; i<=LLD_NUM_FLASH_CHANNELS; i++)
        GLOB_PendingCMD[tag].ChanSync[i]        = 0;

    GLOB_PendingCMD[tag].Status          = 0xB0B;

    return PASS;
}


#if DEBUG_SYNC || VERBOSE
/* Double check here because CheckSyncPoints also uses it */
static void GLOB_PendingCMDToPerChannelArray(PENDING_CMD (*GLOB_PendingCMDChannel)[LLD_NUM_FLASH_CHANNELS+MAX_DESCRIPTORS], uint16 tag_count, uint32 *chIndexes)
{
    uint32 i, j, chnl;

    for(i = 0; i < LLD_NUM_FLASH_CHANNELS; i++)
    {
        chIndexes[i] = 0;
    }
    for(i = 0; i < tag_count + LLD_NUM_FLASH_CHANNELS; i++)
    {
        chnl = GLOB_PendingCMD[i].Block / (GLOB_DeviceInfo.wTotalBlocks /
                GLOB_totalUsedBanks);
        debug_boundary_error(chnl, GLOB_totalUsedBanks, i);

        GLOB_PendingCMDChannel[chnl][chIndexes[chnl]].Tag = GLOB_PendingCMD[i].Tag;
        GLOB_PendingCMDChannel[chnl][chIndexes[chnl]].CMD = GLOB_PendingCMD[i].CMD;
        GLOB_PendingCMDChannel[chnl][chIndexes[chnl]].DataAddr = GLOB_PendingCMD[i].DataAddr;
        GLOB_PendingCMDChannel[chnl][chIndexes[chnl]].Block = GLOB_PendingCMD[i].Block;
        GLOB_PendingCMDChannel[chnl][chIndexes[chnl]].Page = GLOB_PendingCMD[i].Page;
        GLOB_PendingCMDChannel[chnl][chIndexes[chnl]].DataDestAddr = GLOB_PendingCMD[i].DataDestAddr;
        GLOB_PendingCMDChannel[chnl][chIndexes[chnl]].PageCount = GLOB_PendingCMD[i].PageCount;
        GLOB_PendingCMDChannel[chnl][chIndexes[chnl]].DataSrcAddr = GLOB_PendingCMD[i].DataSrcAddr;
        GLOB_PendingCMDChannel[chnl][chIndexes[chnl]].MemCopyByteCnt = GLOB_PendingCMD[i].MemCopyByteCnt;
        GLOB_PendingCMDChannel[chnl][chIndexes[chnl]].ChanSync[0] = GLOB_PendingCMD[i].ChanSync[0];
        GLOB_PendingCMDChannel[chnl][chIndexes[chnl]].Status = GLOB_PendingCMD[i].Status;
        chIndexes[chnl]++;
        for (j=1; (j<=LLD_NUM_FLASH_CHANNELS) && (GLOB_PendingCMD[i].ChanSync[j]); j++) {
            GLOB_PendingCMDChannel[chnl][chIndexes[chnl]].Tag = 0xFF;
            GLOB_PendingCMDChannel[chnl][chIndexes[chnl]].CMD = DUMMY_CMD;
            GLOB_PendingCMDChannel[chnl][chIndexes[chnl]].Block = GLOB_PendingCMD[i].Block;
            GLOB_PendingCMDChannel[chnl][chIndexes[chnl]].ChanSync[0] = GLOB_PendingCMD[i].ChanSync[j];
            chIndexes[chnl]++;
        }
    }
}
#endif

#if VERBOSE
/*&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
* Function:     PrintGLOB_PendingCMDs
* Inputs:       none
* Outputs:      none
* Description:  prints the GLOB_PendingCMDs array
*               number of elements to print needs manual control to keep it small
*&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&*/
void PrintGLOB_PendingCMDs(uint16 tag_count)
{
    uint16  i;
    uint16  not_print=0;

    print(" Printing GLOB_PendingCMDs Table \n");
    print(" -------------------------------------------------------------------------|\n");
    print("           | Cache  |     Flash      |        MemCopy       |        |    |\n");
    print("Tag Command DataAddr Block Page PgCnt DestAddr SrcAddr  BCnt ChanSync Stat|\n");

    for(i = 0; i < tag_count + LLD_NUM_FLASH_CHANNELS; i++) // use this for real testing
    {
        not_print = 0;
        switch (GLOB_PendingCMD[i].CMD)
        {
        case ERASE_CMD:
            print("%03d", GLOB_PendingCMD[i].Tag);
            print(" ERASE  ");
            break;
        case WRITE_MAIN_CMD:
            print("%03d", GLOB_PendingCMD[i].Tag);
            print(" WRITE  ");
            break;
        case WRITE_MAIN_SPARE_CMD:
            print("%03d", GLOB_PendingCMD[i].Tag);
            print(" WRITE MAIN+SPARE  ");
            break;
        case READ_MAIN_SPARE_CMD:
            print("%03d", GLOB_PendingCMD[i].Tag);
            print(" WRITE MAIN+SPARE  ");
            break;
        case READ_MAIN_CMD:
            print("%03d", GLOB_PendingCMD[i].Tag);
            print(" READ   ");
            break;
        case MEMCOPY_CMD:
            print("%03d", GLOB_PendingCMD[i].Tag);
            print(" MemCpy ");
            break;
        case DUMMY_CMD:
            print("%03d", GLOB_PendingCMD[i].Tag);
            print("  DUMMY ");
            break;
        default:
            if(i!=0)
            {
                //i=MAX_DESCRIPTORS + LLD_NUM_FLASH_CHANNELS;
                not_print =1;
            }
        }
        if(!not_print)
        {
            print(" %p", GLOB_PendingCMD[i].DataAddr);
            print("  %04X",  GLOB_PendingCMD[i].Block);
            print(" %04X",  GLOB_PendingCMD[i].Page);
            print(" %04X",  GLOB_PendingCMD[i].PageCount);
            print("  %p", GLOB_PendingCMD[i].DataDestAddr);
            print(" %p", GLOB_PendingCMD[i].DataSrcAddr);
            print(" %04X",  GLOB_PendingCMD[i].MemCopyByteCnt);
            print(" %04X",  GLOB_PendingCMD[i].ChanSync[0]);
            print(" %04X",  GLOB_PendingCMD[i].Status);
            print("|\n");
        }
    }
    print(" -------------------------------------------------------------------------|\n");
}

/*&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
* Function:     PrintGLOB_PendingCMDsPerChannel
* Inputs:       none
* Outputs:      none
* Description:  prints the GLOB_PendingCMDs array on a per channel basis
*               number of elements to print needs manual control to keep it small
*&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&*/
void PrintGLOB_PendingCMDsPerChannel(uint16 tag_count)
{
    uint16  i, chnl;
    uint16  not_print=0;
    PENDING_CMD GLOB_PendingCMDChannel[LLD_NUM_FLASH_CHANNELS][LLD_NUM_FLASH_CHANNELS+MAX_DESCRIPTORS];
    uint32 chIndexes[LLD_NUM_FLASH_CHANNELS], maxChIndexes;

    GLOB_PendingCMDToPerChannelArray(GLOB_PendingCMDChannel, tag_count, chIndexes);
    print(" Printing GLOB_PendingCMDsPerChannel Table \n");
    for(i = 0; i < LLD_NUM_FLASH_CHANNELS; i++)
    {
        print(" -------------------------------------|");
    }
    print("\n");
    for(i = 0; i < LLD_NUM_FLASH_CHANNELS; i++)
    {
        print(" Ch%1d                                  |", i);
    }
    print("\n");
    maxChIndexes = 0;
    for(i = 0; i < LLD_NUM_FLASH_CHANNELS; i++)
    {
        print("Tag Command  FromAddr   DestAddr  Sync|");
        if (maxChIndexes < chIndexes[i])
            maxChIndexes = chIndexes[i];
    }
    print("\n");

    for(i = 0; i <= maxChIndexes; i++) // use this for real testing
    {
        for (chnl = 0; chnl < LLD_NUM_FLASH_CHANNELS; chnl++)
        {
            not_print = 0;
            if (chIndexes[chnl] > i)
            {
                switch (GLOB_PendingCMDChannel[chnl][i].CMD)
                {
                case ERASE_CMD:
                    print("%03d", GLOB_PendingCMDChannel[chnl][i].Tag);
                    print("  ERASE ");
                    print("         ");
                    print("   %04X:0000",  GLOB_PendingCMDChannel[chnl][i].Block);
                    break;
                case WRITE_MAIN_CMD:
                    print("%03d", GLOB_PendingCMDChannel[chnl][i].Tag);
                    print("  WR_MN ");
                    print("  %p", GLOB_PendingCMDChannel[chnl][i].DataAddr);
                    print("  %04X",  GLOB_PendingCMDChannel[chnl][i].Block);
                    print(":%04X",  GLOB_PendingCMDChannel[chnl][i].Page);
                    break;
                case WRITE_MAIN_SPARE_CMD:
                    print("%03d", GLOB_PendingCMDChannel[chnl][i].Tag);
                    print(" WR_M+S ");
                    print("  %p", GLOB_PendingCMDChannel[chnl][i].DataAddr);
                    print("  %04X",  GLOB_PendingCMDChannel[chnl][i].Block);
                    print(":%04X",  GLOB_PendingCMDChannel[chnl][i].Page);
                    break;
                case READ_MAIN_SPARE_CMD:
                    print("%03d", GLOB_PendingCMDChannel[chnl][i].Tag);
                    print(" RD_M+S ");
                    print("  %04X",  GLOB_PendingCMDChannel[chnl][i].Block);
                    print(":%04X",  GLOB_PendingCMDChannel[chnl][i].Page);
                    print("  %p", GLOB_PendingCMDChannel[chnl][i].DataAddr);
                    break;
                case READ_MAIN_CMD:
                    print("%03d", GLOB_PendingCMDChannel[chnl][i].Tag);
                    print("   READ ");
                    print(" %04X",  GLOB_PendingCMDChannel[chnl][i].Block);
                    print(":%04X",  GLOB_PendingCMDChannel[chnl][i].Page);
                    print("   %p", GLOB_PendingCMDChannel[chnl][i].DataAddr);
                    break;
                case MEMCOPY_CMD:
                    print("%03d", GLOB_PendingCMDChannel[chnl][i].Tag);
                    print(" MemCpy ");
                    print("  %p", GLOB_PendingCMDChannel[chnl][i].DataSrcAddr);
                    print("  %p", GLOB_PendingCMDChannel[chnl][i].DataDestAddr);
                    break;
                case DUMMY_CMD:
                    print("%03d", GLOB_PendingCMDChannel[chnl][i].Tag);
                    print("  DUMMY ");
                    print("            %04X:0000",  GLOB_PendingCMDChannel[chnl][i].Block);
                    break;
                default:
                    //i=MAX_DESCRIPTORS + LLD_NUM_FLASH_CHANNELS;
                    not_print =1;
                }
            }
            else
                not_print = 1;

            if(!not_print)
            {
                //print(" %04X",  GLOB_PendingCMDChannel[chnl][i].PageCount);
                //print(" %04X",  GLOB_PendingCMDChannel[chnl][i].MemCopyByteCnt);
                print("  %04X|",  GLOB_PendingCMDChannel[chnl][i].ChanSync[0]);
            }
            else
                print("                                      |");
            if (chnl == LLD_NUM_FLASH_CHANNELS-1) print("\n");
        }
    }
    print(" -------------------------------------------------------------------------|\n");
}
#endif

// Local Functions
#if VERBOSE
/*&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
* Function:     PrintCDMA_Descriptors
* Inputs:       none
* Outputs:      none
* Description:  prints the CDMA_Descriptors array
*               number of elements to print needs manual control to keep it small
*&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&*/
void PrintCDMA_Descriptors()
{
    uint16  i;
    CDMA_DESCRIPTOR  *cdmaChanPtr[LLD_NUM_FLASH_CHANNELS];
    CDMA_DESCRIPTOR *cdmaChanPtrTotal = NULL;
    MEM_COPY_DESCRIPTOR *mcpyPtr;

    char str[LLD_NUM_FLASH_CHANNELS * 50 + 2];
    char *strp;

    for(i = 0; i < LLD_NUM_FLASH_CHANNELS; i++)
    {
        cdmaChanPtr[i] = &(CDMA_Descriptor[i][0]);
        cdmaChanPtrTotal += (uint32)cdmaChanPtr[i];
    }

    print(" Printing CDMA_Descriptors Table \n");
    print("------------------------------------------------------------------------------------------------------------------------------------------------------------------------------\n");
    print(" CMD | FromAddr |   ToAddr | Siz | Channel | CMD | FromAddr |   ToAddr | Siz | Channel | CMD | FromAddr |   ToAddr | Siz | Channel | CMD | FromAddr |   ToAddr | Siz | Channel\n");

    while (cdmaChanPtrTotal)
    {
        cdmaChanPtrTotal = NULL;
        for(i = 0; i < LLD_NUM_FLASH_CHANNELS; i++)
        {
            strp = &str[i * (5+22+6+11)];
            if (cdmaChanPtr[i]) {

                switch((cdmaChanPtr[i]->CommandType) >> 8) {
                    case 0x21:
                        sprintf(strp, " FWr "); strp += 5;
                        sprintf(strp, " 0x%04x%04x", (unsigned)cdmaChanPtr[i]->MemAddrHi, (uint16)cdmaChanPtr[i]->MemAddrLo); strp += 11;
                        sprintf(strp, " 0x%04x%04x", (unsigned)cdmaChanPtr[i]->FlashPointerHi, (uint16)cdmaChanPtr[i]->FlashPointerLo); strp += 11;
                        break;
                    case 0x20:
                        if ((cdmaChanPtr[i]->CommandFlags >> 10)) {
                            sprintf(strp, " Mcp "); strp += 5;
                            mcpyPtr = (MEM_COPY_DESCRIPTOR *)((cdmaChanPtr[i]->MemCopyPointerHi << 16) | cdmaChanPtr[i]->MemCopyPointerLo);
                            sprintf(strp, " 0x%04x%04x",  (unsigned)mcpyPtr->SrcAddrHi, (uint16)mcpyPtr->SrcAddrLo); strp += 11;
                            sprintf(strp, " 0x%04x%04x", (unsigned)mcpyPtr->DestAddrHi, (uint16)mcpyPtr->DestAddrLo); strp += 11;
                        }
                        else {
                            sprintf(strp, " FRd "); strp += 5;
                            sprintf(strp, " 0x%04x%04x", (unsigned)cdmaChanPtr[i]->FlashPointerHi, (uint16)cdmaChanPtr[i]->FlashPointerLo); strp += 11;
                            sprintf(strp, " 0x%04x%04x", (unsigned)cdmaChanPtr[i]->MemAddrHi, (uint16)cdmaChanPtr[i]->MemAddrLo); strp += 11;
                        }
                        break;
                    default:
                        if (cdmaChanPtr[i]->CommandType == 0x1) {
                            sprintf(strp, " Ers "); strp += 5;
                        }
                        else {
                            sprintf(strp, " INV "); strp += 5;
                        }
                            sprintf(strp, "                          "); strp += 22;
                        break;
                }
                sprintf(strp, "  %3d ",  (int)(cdmaChanPtr[i]->CommandType & 0xFFF)); strp += 6;
                sprintf(strp, "  0x%04x ||", (unsigned)cdmaChanPtr[i]->Channel); strp += 11;

                cdmaChanPtr[i] = (CDMA_DESCRIPTOR *)((cdmaChanPtr[i]->NxtPointerHi << 16) | cdmaChanPtr[i]->NxtPointerLo);
                cdmaChanPtrTotal += (uint32)cdmaChanPtr[i];
            }
            else {
                sprintf(strp, "                                                  |"); strp += 44;
            }
        }
        sprintf(strp, "\n");
        print("%s",str);
    }
    print(" -------------------------------------------------------------------------|\n");
}
#endif
/*&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
* Function:     CDMA_AddDummyDesc
* Inputs:       Channel number
* Outputs:      None
* Description:  This function adds a dummy descriptor at the descriptor
*               location (from dcount structure) in the given channel.
*&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&*/
static void     CDMA_AddDummyDesc(uint16 channel)
{
    uint16 c,d;
    uint32 *FBA_FPA_FSA_P;
    uint32 cont, tempcalcvar;

    c = channel;

    d = dcount[c];
    debug_boundary_error(d, MAX_DESC_PER_CHANNEL, 0);

    CDMA_Descriptor[c][d].NxtPointerHi     =  0;
    CDMA_Descriptor[c][d].NxtPointerLo     =  0;
    CDMA_Descriptor[c][d].FlashPointerHi   =  0;
    CDMA_Descriptor[c][d].FlashPointerLo   =  0;
    CDMA_Descriptor[c][d].CommandType      =  0;
    CDMA_Descriptor[c][d].MemAddrHi        =  0;
    CDMA_Descriptor[c][d].MemAddrLo        =  0;
    CDMA_Descriptor[c][d].CommandFlags     =  0;
    CDMA_Descriptor[c][d].Channel          =  0;
    CDMA_Descriptor[c][d].Status           =  0;
    CDMA_Descriptor[c][d].MemCopyPointerHi =  0;
    CDMA_Descriptor[c][d].MemCopyPointerLo =  0;
    //CDMA_Descriptor[c][d].Reserved12       =  0;
    //CDMA_Descriptor[c][d].Reserved13       =  0;
    //CDMA_Descriptor[c][d].Reserved14       =  0;
    CDMA_Descriptor[c][d].Tag              =  0;

    tempcalcvar = CALC_CDMA_NEXT_PTR(CDMA_Descriptor, c, d);
    CDMA_Descriptor[c][d].NxtPointerHi = tempcalcvar >> 16;
    CDMA_Descriptor[c][d].NxtPointerLo = tempcalcvar & 0xFFFF;


    FBA_FPA_FSA_P   = (uint32 *) (uint32 )( MODE_10 | (c << 24));


    CDMA_Descriptor[c][d].FlashPointerHi = (uint32)((uint32)FBA_FPA_FSA_P >> 16);
    CDMA_Descriptor[c][d].FlashPointerLo = (uint32)FBA_FPA_FSA_P;


    CDMA_Descriptor[c][d].CommandType =  0x42;
    cont =1;
    CDMA_Descriptor[c][d].CommandFlags = (0 << 10) | (cont << 9) | (0 << 8)| 0x40;

    CDMA_Descriptor[c][d].Status = 0;
    CDMA_Descriptor[c][d].Tag  = 0xFF;

    return;
}

/*&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
* Function:     CDMA_AddDummyDescAtEnd
* Inputs:       Channel number
* Outputs:      None
* Description:  This function adds a dummy descriptor at the end of the
*               descriptor chain for the given channel.
*               The purpose of these descriptors is to get a single
*               interrupt on cmd dma chain completion using sync.
*&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&*/

#if FLASH_ESL
#else

static void     CDMA_AddDummyDescAtEnd(uint16 channel)
{
    uint16 c,d;
    uint32 *FBA_FPA_FSA_P;
    uint32 cont;

    c = channel;

    d = dcount[c];
    debug_boundary_error(d, MAX_DESC_PER_CHANNEL, 0);

    CDMA_Descriptor[c][d].NxtPointerHi     =  0;
    CDMA_Descriptor[c][d].NxtPointerLo     =  0;
    CDMA_Descriptor[c][d].FlashPointerHi   =  0;
    CDMA_Descriptor[c][d].FlashPointerLo   =  0;
    CDMA_Descriptor[c][d].CommandType      =  0;
    CDMA_Descriptor[c][d].MemAddrHi        =  0;
    CDMA_Descriptor[c][d].MemAddrLo        =  0;
    CDMA_Descriptor[c][d].CommandFlags     =  0;
    CDMA_Descriptor[c][d].Channel          =  0;
    CDMA_Descriptor[c][d].Status           =  0;
    CDMA_Descriptor[c][d].MemCopyPointerHi =  0;
    CDMA_Descriptor[c][d].MemCopyPointerLo =  0;
    CDMA_Descriptor[c][d].Tag              =  0;

    FBA_FPA_FSA_P   = (uint32 *) (uint32)( MODE_10 | (c << 24));


    CDMA_Descriptor[c][d].FlashPointerHi = (uint32)((uint32)FBA_FPA_FSA_P >> 16);
    CDMA_Descriptor[c][d].FlashPointerLo = (uint32)FBA_FPA_FSA_P;


    CDMA_Descriptor[c][d].CommandType =  0xFFFF;
    cont =0;
    CDMA_Descriptor[c][d].CommandFlags = (0 << 10) | (cont << 9) | (1 << 8)| 0x40;


    CDMA_Descriptor[c][d].Channel = ( (1 << 15) | (1 << 14) | (c << CHANNEL_ID_OFFSET) |
                                        ( (GLOB_valid_banks[3] <<7) | (GLOB_valid_banks[2] <<6)| (GLOB_valid_banks[1] <<5) | (GLOB_valid_banks[0] <<4)) |
                                    0x0 );

    CDMA_Descriptor[c][d].Status = 0;
    CDMA_Descriptor[c][d].Tag  = 0xFF;

    return;
}

uint32 CDMA_Memory_Pool_Size(void)
{
    return (sizeof(CDMA_DESCRIPTOR) * LLD_NUM_FLASH_CHANNELS * MAX_DESC_PER_CHANNEL) +
           (sizeof(MEM_COPY_DESCRIPTOR) * LLD_NUM_FLASH_CHANNELS * MAX_DESC_PER_CHANNEL) +
           6;
}

int CDMA_Mem_Config(byte * pMem)
{
    ALIGN_DWORD_FWD(pMem);
    CDMA_Descriptor = (CDMA_DESCRIPTOR (*)[MAX_DESC_PER_CHANNEL])pMem;
    pMem += (sizeof(CDMA_DESCRIPTOR) * LLD_NUM_FLASH_CHANNELS * MAX_DESC_PER_CHANNEL);
    ALIGN_DWORD_FWD(pMem);
    MemCopyDescriptor = (MEM_COPY_DESCRIPTOR (*)[MAX_DESC_PER_CHANNEL])pMem;
    return PASS;
}

#if FLASH_CDMA
/*&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
* Function:     CDMA_Flash_Init
* Inputs:       none
* Outputs:      PASS=0 (notice 0=ok here)
* Description:  This should be called at power up.
*               It disables interrupts and clears status bits
*               issues flash reset command
*               configures the controller registers
*               It sets the interrupt mask and enables interrupts
*               It pre-builds special descriptors
*&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&*/
uint16    CDMA_Flash_Init (void)
{
uint16  i, j;  // loop counter


   if(GLOB_init_firsttime_done == 0)
    {
        while((FlashReg[INTR_STATUS0]& INTR_EN0__RST_COMP ) == 0);
        GLOB_init_firsttime_done = 1;
    }
    else
    {
        //  may need to issue reset and wait for rst_comp
    }

    // Disable all interrupts
    FlashReg[GLOBAL_INT_ENABLE] = 0;
    FlashReg[INTR_EN0]          = 0;
    FlashReg[INTR_EN1]          = 0;
    FlashReg[INTR_EN2]          = 0;
    FlashReg[INTR_EN3]          = 0;

    // Clear all status bits
    FlashReg[INTR_STATUS0]      = 0xFFFF;
    FlashReg[INTR_STATUS1]      = 0xFFFF;
    FlashReg[INTR_STATUS2]      = 0xFFFF;
    FlashReg[INTR_STATUS3]      = 0xFFFF;

    FlashReg[DMA_INTR_EN]       = 0;
    FlashReg[DMA_INTR]          = 0xFFFF;

    // Set the global Enable masks for only those interrupts that are supported
    CMD_DMA_InterruptEnableMask = (DMA_INTR__DESC_COMP_CHANNEL0 |
                                   DMA_INTR__DESC_COMP_CHANNEL1 |
                                   DMA_INTR__DESC_COMP_CHANNEL2 |
                                   DMA_INTR__DESC_COMP_CHANNEL3 |
                                   DMA_INTR__MEMCOPY_DESC_COMP
                                   );
    FlashReg[DMA_INTR_EN] = CMD_DMA_InterruptEnableMask;

    InterruptEnableMask =  (INTR_STATUS0__ECC_ERR               |
                            INTR_STATUS0__TIME_OUT              |
                            INTR_STATUS0__PROGRAM_FAIL          |
                            INTR_STATUS0__ERASE_FAIL
                           );

    FlashReg[INTR_EN0] = InterruptEnableMask;
    FlashReg[INTR_EN1] = InterruptEnableMask;
    FlashReg[INTR_EN2] = InterruptEnableMask;
    FlashReg[INTR_EN3] = InterruptEnableMask;

    // Enable global interrupt to host
    FlashReg[GLOBAL_INT_ENABLE]     =   GLOBAL_INT_ENABLE__FLAG;

    // clear the pending CMD array
    for (i=0; i<MAX_DESCRIPTORS+LLD_NUM_FLASH_CHANNELS; i++)
    {
        GLOB_PendingCMD[i].CMD           = 0;
        GLOB_PendingCMD[i].Tag           = 0;
        GLOB_PendingCMD[i].DataAddr      = 0;
        GLOB_PendingCMD[i].Block         = 0;
        GLOB_PendingCMD[i].Page          = 0;
        GLOB_PendingCMD[i].PageCount     = 0;
        GLOB_PendingCMD[i].DataDestAddr  = 0;
        GLOB_PendingCMD[i].DataSrcAddr   = 0;
        GLOB_PendingCMD[i].MemCopyByteCnt= 0;


        for (j=0; j<=LLD_NUM_FLASH_CHANNELS; j++)
            GLOB_PendingCMD[i].ChanSync[j]        = 0;

        GLOB_PendingCMD[i].Status        = 0;
        GLOB_PendingCMD[i].SBDCmdIndex   = 0;
    }

    return PASS;
}

/*&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
* Function:     CDMA_AbortChannels
* Inputs:       channel with failed descriptor
* Outputs:      PASS/ FAIL status
* Description:  This function is called to Abort all the other active channels
*               when a channel gets an error.
*
*&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&*/
uint16  CDMA_AbortChannels(uint16    chan)
{
    uint16  c,d;
    uint32 *FBA_FPA_FSA_P;
    uint16  aborts_comp;
    uint16  DescB4Abort[LLD_NUM_FLASH_CHANNELS];
    uint16  status = PASS;
    uint32  chnl_active_channel=0;

    debug_boundary_error(chan, GLOB_totalUsedBanks, 0);
    // if status not complete, Abort the channel
    for (c =0; c < LLD_NUM_FLASH_CHANNELS; c++)
    {
        // initialize the descriptor to be aborted
        DescB4Abort[c] = 0xFF;

        if (( c != chan) && (GLOB_valid_banks[c] == 1))
        {
            for(d =0; d < dcount[c]; d++)
            {
                if( (CDMA_Descriptor[c][d].Status & CMD_DMA_DESC_COMP) != CMD_DMA_DESC_COMP )
                {

                    if(CDMA_Descriptor[c][d].Tag != 0xFF)
                        GLOB_PendingCMD[CDMA_Descriptor[c][d].Tag].Status = CMD_PASS;

                    break;
                }
                else
                {
                    if(CDMA_Descriptor[c][d].Tag != 0xFF)
                        GLOB_PendingCMD[CDMA_Descriptor[c][d].Tag].Status = CMD_PASS;
                }
            }

            if ( (FlashReg[CHNL_ACTIVE] & (1 << c)) == (1 << c) )
            {
                DescB4Abort[c] = d;

                aborts_comp = 0;

                if(GLOB_DeviceInfo.wHWFeatures & FEATURES__INDEX_ADDR)
                {
                    FBA_FPA_FSA_P = (uint32 *)(FlashMem);
                    *FBA_FPA_FSA_P = (uint32 )( MODE_02 | (0x0 << 4) );


                    FBA_FPA_FSA_P = (uint32 *)((uint32)FlashMem | 0x10);
                    *FBA_FPA_FSA_P = (0xF << 4) | c;
                }
                else
                {
                    FBA_FPA_FSA_P   = (uint32 *) (uint32)((uint32)FlashMem | MODE_10 | (c << 24) | MODE_02 | (0x0 << 4 ) );
                    *FBA_FPA_FSA_P = (0xF << 4) | c;

                }
            }
        }
    }


    // Check if aborts (of all active channels) are done
    while(1)
    {
        aborts_comp = 1;
        for (c =0; c < LLD_NUM_FLASH_CHANNELS; c++)
        {
            if ( (DescB4Abort[c] != 0xFF) &&  (c != chan) )
            {
                if (c == 0)
                    chnl_active_channel = CHNL_ACTIVE__CHANNEL0;
                else if (c == 1)
                    chnl_active_channel = CHNL_ACTIVE__CHANNEL1;
                else if (c == 2)
                    chnl_active_channel = CHNL_ACTIVE__CHANNEL2;
                else if (c == 3)
                    chnl_active_channel = CHNL_ACTIVE__CHANNEL3;

                if( (FlashReg[CHNL_ACTIVE] & chnl_active_channel) == 0 )
                {

                    DescB4Abort[c] = 0xFF;

                }
                else
                {
                    aborts_comp = 0;
                }
            }
        }

        if(aborts_comp == 1)
            break;
    }


    // At this stage all the channels are Aborted, All CMD_DMA related logic is reset

    // Reset the sync module
    ResetSyncModule();

    return status;

}
/*&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
* Function:     CDMA_UpdateEventStatus
* Inputs:       none
* Outputs:      none
* Description:  This function update the event status of all the channels
*               when an error condition is reported.
*
*&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&*/
uint16 CDMA_UpdateEventStatus(void)
{
uint16 i,j,c;
uint16 d;
uint16  status = PASS;


    for(c=0; c < LLD_NUM_FLASH_CHANNELS; c++)
    {
        if(GLOB_valid_banks[c] == 1)
        {
            d = dcount[c];
            debug_boundary_error(d, MAX_DESC_PER_CHANNEL, 0);

            for(j = 0; j < d; j++)
            {
                // Check for the descriptor with failure (not just desc_complete)
                if ((CDMA_Descriptor[c][j].Status) & CMD_DMA_DESC_FAIL)
                {
                    // All the previous command's status for this channel must
                    // be good (no errors reported)
                    for(i = 0; i < j; i++)
                    {
                        if (CDMA_Descriptor[c][i].Tag != 0xFF)
                            GLOB_PendingCMD[CDMA_Descriptor[c][i].Tag].Status = CMD_PASS;
                    }

                    status = CDMA_AbortChannels(c);
                    return status;
                }
            }
        }
     }
    return status;
}
#endif

#define FCMD_INDX_TX2_ADD(c, CDMA_Descriptor) \
    (uint32) \
    ( MODE_10 | (c << 24) | \
      ( ( 0x0000FFFF \
        & ( (uint32)( (uint32)GLOB_MEMMAP_TOBUS((uint32 *)CDMA_Descriptor) \
                    + ( sizeof (CDMA_DESCRIPTOR) \
                      * (c * MAX_DESC_PER_CHANNEL ) \
                      ) \
                    ) >> 16 \
          ) \
        ) << 8 \
      ) \
    )

#define FCMD_INDX_TX3_ADD(c, CDMA_Descriptor) \
    (uint32) \
    ( MODE_10 | (c << 24) | \
      ( ( 0x0000FFFF \
        & ( (uint32)( (uint32)GLOB_MEMMAP_TOBUS((uint32 *)CDMA_Descriptor) \
                    + ( sizeof (CDMA_DESCRIPTOR) \
                      * (c * MAX_DESC_PER_CHANNEL ) \
                      ) \
                    ) \
          ) \
        ) << 8 \
      ) \
    )

/*&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
* Function:     CDMA_Execute_CMDs (for use with CMD_DMA)
* Inputs:       tag_count:  the number of pending cmds to do
* Outputs:      PASS/FAIL
* Description:  Build the SDMA chain(s) by making one CMD-DMA descriptor
*               for each pending command, start the CDMA engine, and return.
*&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&*/
uint16  CDMA_Execute_CMDs (uint16 tag_count)
{
    uint16  i,j;
    byte    cont;               // continue flag
    ADDRESSTYPE flash_add;
    uint32 *FBA_FPA_FSA_P, tempcalcvar;
    uint16  status = PASS;
    uint16  c,d;
    uint16  tmp_c;

    // tag_count needs to be within MAX_DESCRIPTOR RANGE
    if (tag_count >= MAX_DESCRIPTORS)
    {
        status = FAIL;
    }
    else
    {
        c=0;
        d=0;

        // Reset the whole CDMA_Descriptor structure to 0
        for (c =0; c < LLD_NUM_FLASH_CHANNELS; c++)
        {
            for (d = 0; d < MAX_DESC_PER_CHANNEL; d++)
            {
                CDMA_Descriptor[c][d].NxtPointerHi     =  0;
                CDMA_Descriptor[c][d].NxtPointerLo     =  0;
                CDMA_Descriptor[c][d].FlashPointerHi   =  0;
                CDMA_Descriptor[c][d].FlashPointerLo   =  0;
                CDMA_Descriptor[c][d].CommandType      =  0;
                CDMA_Descriptor[c][d].MemAddrHi        =  0;
                CDMA_Descriptor[c][d].MemAddrLo        =  0;
                CDMA_Descriptor[c][d].CommandFlags     =  0;
                CDMA_Descriptor[c][d].Channel          =  0;
                CDMA_Descriptor[c][d].Status           =  0;
                CDMA_Descriptor[c][d].MemCopyPointerHi =  0;
                CDMA_Descriptor[c][d].MemCopyPointerLo =  0;
                CDMA_Descriptor[c][d].Tag              =  0;


                CDMA_Descriptor[c][d].NxtPointerHi  =   0;
                CDMA_Descriptor[c][d].NxtPointerLo  =   0;

                CDMA_Descriptor[c][d].FlashPointerHi =  0;
                CDMA_Descriptor[c][d].FlashPointerLo =  0;
            }
        }

        debug_boundary_error(GLOB_totalUsedBanks-1, LLD_NUM_FLASH_CHANNELS, 0);
        for (c =0; c < GLOB_totalUsedBanks; c++)
        {
            dcount[c] = 0;

            GLOB_PendingCMD[c].CMD           = DUMMY_CMD;
            GLOB_PendingCMD[c].SBDCmdIndex   = 0xFF;
            GLOB_PendingCMD[c].Tag           = 0xFF;
            GLOB_PendingCMD[c].Block         = c * (GLOB_DeviceInfo.wTotalBlocks / GLOB_totalUsedBanks);

            for (i = 0; i<=LLD_NUM_FLASH_CHANNELS; i++)
                GLOB_PendingCMD[c].ChanSync[i] = 0;
        }

        c= 0;

        CDMA_AddSyncPoints(tag_count);
#if DEBUG_SYNC
        CDMA_CheckSyncPoints(tag_count);
#endif

        for (i = 0; i<tag_count + LLD_NUM_FLASH_CHANNELS; i++)
        {

            if ( (i >= GLOB_totalUsedBanks) && (i < LLD_NUM_FLASH_CHANNELS))
                continue;


            if (GLOB_PendingCMD[i].Block >= GLOB_DeviceInfo.wTotalBlocks)
            {
                GLOB_PendingCMD[i].Status = CMD_NOT_DONE;
                continue;
            }

            c = 0;
            tmp_c = GLOB_PendingCMD[i].Block / (GLOB_DeviceInfo.wTotalBlocks / GLOB_totalUsedBanks);
            debug_boundary_error(tmp_c, GLOB_totalUsedBanks, 0);
            if (tmp_c == 0)
                c = tmp_c;
            else
            {
                for (j =1; j < LLD_NUM_FLASH_CHANNELS; j++)
                {
                    if(GLOB_valid_banks[j])
                    {
                        tmp_c--;
                        if(tmp_c == 0)
                        {
                            c = j;
                            break;
                        }
                    }
                }
            }

            if (GLOB_valid_banks[c] == 1)
            {
                d = dcount[c];
                dcount[c]++;
            }
            else
            {

                continue;
            }

            tempcalcvar = CALC_CDMA_NEXT_PTR(CDMA_Descriptor, c, d);
            CDMA_Descriptor[c][d].NxtPointerHi = tempcalcvar >> 16;
            CDMA_Descriptor[c][d].NxtPointerLo = tempcalcvar & 0xFFFF;


            // Use the Block offset within a bank
            tmp_c = GLOB_PendingCMD[i].Block / (GLOB_DeviceInfo.wTotalBlocks / GLOB_totalUsedBanks);
            debug_boundary_error(tmp_c, GLOB_totalUsedBanks, i);
            flash_add = (GLOB_PendingCMD[i].Block - (tmp_c * (GLOB_DeviceInfo.wTotalBlocks / GLOB_totalUsedBanks)) ) *GLOB_DeviceInfo.wBlockDataSize
                            + (GLOB_PendingCMD[i].Page)*GLOB_DeviceInfo.wPageDataSize;


#if FLASH_CDMA
            if(GLOB_DeviceInfo.wHWFeatures & FEATURES__INDEX_ADDR)
            {
                FBA_FPA_FSA_P   = (uint32 *)( MODE_10 | (c << 24) |
                        ( GLOB_u64_Div((flash_add), GLOB_DeviceInfo.wPageDataSize )));
            }
            else
            {
                FBA_FPA_FSA_P   = (uint32 *)((uint32)(uint32)FlashMem |MODE_10 | (c << 24) |
                        ( ( GLOB_u64_Div((flash_add), GLOB_DeviceInfo.wPageDataSize ))   <<2 ) );
            }

            CDMA_Descriptor[c][d].FlashPointerHi = (uint32)((uint32)FBA_FPA_FSA_P >> 16);
            CDMA_Descriptor[c][d].FlashPointerLo = (uint32)FBA_FPA_FSA_P;
#endif
            // set continue flag except if last cmd-descriptor
            cont = 1;

            if( (GLOB_PendingCMD[i].CMD == WRITE_MAIN_SPARE_CMD) || (GLOB_PendingCMD[i].CMD == READ_MAIN_SPARE_CMD) )

            {
                // Descriptor to set Main+Spare Access Mode-- Begin
                CDMA_Descriptor[c][d].CommandType   = 0x43;
                CDMA_Descriptor[c][d].CommandFlags  = (0 << 10) | (cont << 9) | (0 << 8)| 0x40;
                CDMA_Descriptor[c][d].MemAddrHi = 0;
                CDMA_Descriptor[c][d].MemAddrLo = 0;

                CDMA_Descriptor[c][d].Channel = 0;
                CDMA_Descriptor[c][d].Status = 0;
                CDMA_Descriptor[c][d].Tag  = i;


                dcount[c]++;
                d++;

                CDMA_Descriptor[c][d].NxtPointerHi     =  0;
                CDMA_Descriptor[c][d].NxtPointerLo     =  0;
                CDMA_Descriptor[c][d].FlashPointerHi   =  0;
                CDMA_Descriptor[c][d].FlashPointerLo   =  0;
                CDMA_Descriptor[c][d].CommandType      =  0;
                CDMA_Descriptor[c][d].MemAddrHi        =  0;
                CDMA_Descriptor[c][d].MemAddrLo        =  0;
                CDMA_Descriptor[c][d].CommandFlags     =  0;
                CDMA_Descriptor[c][d].Channel          =  0;
                CDMA_Descriptor[c][d].Status           =  0;
                CDMA_Descriptor[c][d].MemCopyPointerHi =  0;
                CDMA_Descriptor[c][d].MemCopyPointerLo =  0;
                CDMA_Descriptor[c][d].Tag              =  0;

                tempcalcvar = CALC_CDMA_NEXT_PTR(CDMA_Descriptor, c, d);
                CDMA_Descriptor[c][d].NxtPointerHi = tempcalcvar >> 16;
                CDMA_Descriptor[c][d].NxtPointerLo = tempcalcvar & 0xFFFF;

#if FLASH_CDMA
                CDMA_Descriptor[c][d].FlashPointerHi = (uint32)((uint32)FBA_FPA_FSA_P >> 16);
                CDMA_Descriptor[c][d].FlashPointerLo = (uint32)FBA_FPA_FSA_P;
#endif
            }

            switch (GLOB_PendingCMD[i].CMD)
            {
            case ERASE_CMD:
                CDMA_Descriptor[c][d].CommandType     = 0x1;
                CDMA_Descriptor[c][d].CommandFlags = (0 << 10) | (cont << 9) | (0 << 8)| 0x40;
                CDMA_Descriptor[c][d].MemAddrHi = 0;
                CDMA_Descriptor[c][d].MemAddrLo = 0;
                break;
            case WRITE_MAIN_CMD:
                CDMA_Descriptor[c][d].CommandType =  0x2100 |(GLOB_PendingCMD[i].PageCount);
                CDMA_Descriptor[c][d].CommandFlags = (0 << 10) | (cont << 9) | (0 << 8)| 0x40;
                CDMA_Descriptor[c][d].MemAddrHi = ((uint32)GLOB_MEMMAP_TOBUS((uint32 *)GLOB_PendingCMD[i].DataAddr)) >> 16;
                CDMA_Descriptor[c][d].MemAddrLo = (uint32)GLOB_MEMMAP_TOBUS((uint32 *)GLOB_PendingCMD[i].DataAddr);
                break;

            case READ_MAIN_CMD:
                CDMA_Descriptor[c][d].CommandType =  0x2000 |(GLOB_PendingCMD[i].PageCount);
                CDMA_Descriptor[c][d].CommandFlags = (0 << 10) | (cont << 9) | (0 << 8)| 0x40;
                CDMA_Descriptor[c][d].MemAddrHi = ((uint32)GLOB_MEMMAP_TOBUS((uint32 *)GLOB_PendingCMD[i].DataAddr)) >> 16;
                CDMA_Descriptor[c][d].MemAddrLo = (uint32)GLOB_MEMMAP_TOBUS((uint32 *)GLOB_PendingCMD[i].DataAddr);
                break;

            case WRITE_MAIN_SPARE_CMD:
                CDMA_Descriptor[c][d].CommandType =  0x2100 |(GLOB_PendingCMD[i].PageCount);
                CDMA_Descriptor[c][d].CommandFlags = (0 << 10) | (cont << 9) | (0 << 8)| 0x40;
                CDMA_Descriptor[c][d].MemAddrHi = ((uint32)GLOB_MEMMAP_TOBUS((uint32 *)GLOB_PendingCMD[i].DataAddr)) >> 16;
                CDMA_Descriptor[c][d].MemAddrLo = (uint32)GLOB_MEMMAP_TOBUS((uint32 *)GLOB_PendingCMD[i].DataAddr);
                break;

            case READ_MAIN_SPARE_CMD:
                CDMA_Descriptor[c][d].CommandType =  0x2000 |(GLOB_PendingCMD[i].PageCount);
                CDMA_Descriptor[c][d].CommandFlags = (0 << 10) | (cont << 9) | (0 << 8)| 0x40;
                CDMA_Descriptor[c][d].MemAddrHi = ((uint32)GLOB_MEMMAP_TOBUS((uint32 *)GLOB_PendingCMD[i].DataAddr)) >> 16;
                CDMA_Descriptor[c][d].MemAddrLo = (uint32)GLOB_MEMMAP_TOBUS((uint32 *)GLOB_PendingCMD[i].DataAddr);
                break;

            case MEMCOPY_CMD:
                CDMA_Descriptor[c][d].CommandType      =  0XFFFF;
                CDMA_Descriptor[c][d].CommandFlags     = (1 << 11)| (1 << 10) | (cont << 9) | (0 << 8)| 0x40;
                CDMA_Descriptor[c][d].MemCopyPointerHi = ((unsigned int)GLOB_MEMMAP_TOBUS((uint32 *)&MemCopyDescriptor[c][d])) >> 16;
                CDMA_Descriptor[c][d].MemCopyPointerLo = (unsigned int)GLOB_MEMMAP_TOBUS((uint32 *)&MemCopyDescriptor[c][d]);

                MemCopyDescriptor[c][d].NxtPointerHi   = 0;
                MemCopyDescriptor[c][d].NxtPointerLo   = 0;
                MemCopyDescriptor[c][d].SrcAddrHi      = ((uint32)GLOB_MEMMAP_TOBUS((uint32 *)GLOB_PendingCMD[i].DataSrcAddr)) >> 16;
                MemCopyDescriptor[c][d].SrcAddrLo      = (uint32)GLOB_MEMMAP_TOBUS((uint32 *)GLOB_PendingCMD[i].DataSrcAddr);
                MemCopyDescriptor[c][d].DestAddrHi     = ((uint32)GLOB_MEMMAP_TOBUS((uint32 *)GLOB_PendingCMD[i].DataDestAddr)) >> 16;
                MemCopyDescriptor[c][d].DestAddrLo     = (uint32)GLOB_MEMMAP_TOBUS((uint32 *)GLOB_PendingCMD[i].DataDestAddr);
                MemCopyDescriptor[c][d].XferSize       = GLOB_PendingCMD[i].MemCopyByteCnt;
                MemCopyDescriptor[c][d].MemCopyFlags   = (0<<15|0<<14|27<<8|0x40);
                MemCopyDescriptor[c][d].MemCopyStatus  = 0;
                break;

            case DUMMY_CMD:
            default:
                CDMA_Descriptor[c][d].CommandType     = 0XFFFF;
                CDMA_Descriptor[c][d].CommandFlags = (0 << 10) | (cont << 9) | (0 << 8)| 0x40;
                CDMA_Descriptor[c][d].MemAddrHi = 0;
                CDMA_Descriptor[c][d].MemAddrLo = 0;
                break;
            }

            CDMA_Descriptor[c][d].Channel = GLOB_PendingCMD[i].ChanSync[0];
            CDMA_Descriptor[c][d].Status = 0;
            CDMA_Descriptor[c][d].Tag  = i;

            for (j=1;j<=LLD_NUM_FLASH_CHANNELS;j++)
            {
                if (GLOB_PendingCMD[i].ChanSync[j])
                {
                    if (GLOB_valid_banks[c] == 1)
                    {
                        CDMA_AddDummyDesc(c);
                        d = dcount[c]++;
                        CDMA_Descriptor[c][d].Channel = GLOB_PendingCMD[i].ChanSync[j];
                    }
                }
            }

            if( (GLOB_PendingCMD[i].CMD == WRITE_MAIN_SPARE_CMD) || (GLOB_PendingCMD[i].CMD == READ_MAIN_SPARE_CMD) )
            {
                // Descriptor to set back Main Area Access Mode-- Begin

                dcount[c]++;
                d++;
                debug_boundary_error(d, MAX_DESC_PER_CHANNEL, 0);

                tempcalcvar = CALC_CDMA_NEXT_PTR(CDMA_Descriptor, c, d);
                CDMA_Descriptor[c][d].NxtPointerHi = tempcalcvar >> 16;
                CDMA_Descriptor[c][d].NxtPointerLo = tempcalcvar & 0xFFFF;
#if FLASH_CDMA
                CDMA_Descriptor[c][d].FlashPointerHi = (uint32)((uint32)FBA_FPA_FSA_P >> 16);
                CDMA_Descriptor[c][d].FlashPointerLo = (uint32)FBA_FPA_FSA_P;
#endif
                CDMA_Descriptor[c][d].CommandType   = 0x42;
                CDMA_Descriptor[c][d].CommandFlags  = (0 << 10) | (cont << 9) | (0 << 8)| 0x40;
                CDMA_Descriptor[c][d].MemAddrHi = 0;
                CDMA_Descriptor[c][d].MemAddrLo = 0;

                CDMA_Descriptor[c][d].Channel = GLOB_PendingCMD[i].ChanSync[0];
                CDMA_Descriptor[c][d].Status = 0;
                CDMA_Descriptor[c][d].Tag  = i;

                // Descriptor to set back Main Area Access Mode-- Begin
            }
        }


        for (c =0; c < LLD_NUM_FLASH_CHANNELS; c++)
        {

            if (GLOB_valid_banks[c] == 1)
            {
                CDMA_AddDummyDescAtEnd(c);
            }
        }

#if FLASH_CDMA
        // Enable DMA
        FlashReg[DMA_ENABLE] = 1;

        // Wait for DMA to be enabled before issuing the next command.
        while(!(FlashReg[DMA_ENABLE] & DMA_ENABLE__FLAG));


        for (c =0; c < LLD_NUM_FLASH_CHANNELS; c++)
        {
            if(GLOB_valid_banks[c] == 0)
                continue;

            if(GLOB_DeviceInfo.wHWFeatures & FEATURES__INDEX_ADDR)
            {

                FBA_FPA_FSA_P = (uint32 *)(FlashMem);
                *FBA_FPA_FSA_P = (uint32 )(MODE_10 | (c << 24));


                FBA_FPA_FSA_P = (uint32 *)((uint32)FlashMem | 0x10);
                *FBA_FPA_FSA_P = (1 << 7) | c;


                FBA_FPA_FSA_P = (uint32 *)(FlashMem);
                *FBA_FPA_FSA_P = FCMD_INDX_TX2_ADD(c, CDMA_Descriptor);

                FBA_FPA_FSA_P = (uint32 *)((uint32)FlashMem | 0x10);
                *FBA_FPA_FSA_P = (1 << 7) | ( 1 << 4) | 0;


                FBA_FPA_FSA_P = (uint32 *)(FlashMem);
                *FBA_FPA_FSA_P = FCMD_INDX_TX3_ADD(c, CDMA_Descriptor);


                FBA_FPA_FSA_P = (uint32 *)((uint32)FlashMem | 0x10);
                *FBA_FPA_FSA_P = (1 << 7) | ( 1 << 5) | 0;


                FBA_FPA_FSA_P = (uint32 *)(FlashMem);
                *FBA_FPA_FSA_P = (uint32 )(MODE_10 | (c << 24) );


                FBA_FPA_FSA_P = (uint32 *)((uint32)FlashMem | 0x10);
                *FBA_FPA_FSA_P = (1 << 7) | ( 1 << 5) | (1 << 4) | 0;

            }
            else
            {
                FBA_FPA_FSA_P   = (uint32 *) (uint32)((uint32)FlashMem | MODE_10 | (c << 24) );
                *FBA_FPA_FSA_P = (1 << 7) | c;


                FBA_FPA_FSA_P   = (uint32 *)( (uint32)FlashMem | FCMD_INDX_TX2_ADD(c, CDMA_Descriptor));
                *FBA_FPA_FSA_P = (1 << 7) | ( 1 << 4) | 0;


                FBA_FPA_FSA_P   = (uint32 *)( (uint32)FlashMem | FCMD_INDX_TX3_ADD(c, CDMA_Descriptor));
                *FBA_FPA_FSA_P = (1 << 7) | ( 1 << 5) | 0;


                FBA_FPA_FSA_P   = (uint32 *)((uint32)FlashMem | MODE_10 |
                        (0 << 16)    | (0x40 << 8) );
                *FBA_FPA_FSA_P = (1 << 7) | ( 1 << 5) | (1 << 4) | 0;

            }
        }
#endif
    }
    return status;
}// end CDMA_Execute_CMDs



#if FLASH_CDMA
static void ResetSyncModule(void)
{
    uint16 c,d;
    uint32 *FBA_FPA_FSA_P;
    uint32 cont, tempcalcvar;

    // Disable all interrupts
    FlashReg[GLOBAL_INT_ENABLE] = 0;

    // Clear all DMA interrupt bits before starting the chains
    FlashReg[DMA_INTR] =  FlashReg[DMA_INTR];

    for (c =0; c < LLD_NUM_FLASH_CHANNELS; c++)
    {
        for (d =0; d < MAX_SYNC_POINTS; d++)
        {
            CDMA_Descriptor[c][d].NxtPointerHi     =  0;
            CDMA_Descriptor[c][d].NxtPointerLo     =  0;
            CDMA_Descriptor[c][d].FlashPointerHi   =  0;
            CDMA_Descriptor[c][d].FlashPointerLo   =  0;
            CDMA_Descriptor[c][d].CommandType      =  0;
            CDMA_Descriptor[c][d].MemAddrHi        =  0;
            CDMA_Descriptor[c][d].MemAddrLo        =  0;
            CDMA_Descriptor[c][d].CommandFlags     =  0;
            CDMA_Descriptor[c][d].Channel          =  0;
            CDMA_Descriptor[c][d].Status           =  0;
            CDMA_Descriptor[c][d].MemCopyPointerHi =  0;
            CDMA_Descriptor[c][d].MemCopyPointerLo =  0;
            CDMA_Descriptor[c][d].Tag              =  0;

            tempcalcvar = CALC_CDMA_NEXT_PTR(CDMA_Descriptor, c, d);
            CDMA_Descriptor[c][d].NxtPointerHi = tempcalcvar >> 16;
            CDMA_Descriptor[c][d].NxtPointerLo = tempcalcvar & 0xFFFF;


            FBA_FPA_FSA_P   = (uint32 *) (uint32)( MODE_10 | (c << 24));


            CDMA_Descriptor[c][d].FlashPointerHi = (uint32)((uint32)FBA_FPA_FSA_P >> 16);
            CDMA_Descriptor[c][d].FlashPointerLo = (uint32)FBA_FPA_FSA_P;


            CDMA_Descriptor[c][d].CommandType =  0xFFFF;

            if (d == (MAX_SYNC_POINTS -1) )
            {
                cont =0;
                CDMA_Descriptor[c][d].CommandFlags = (0 << 10) | (cont << 9) | (1 << 8)| 0x40;
            }
            else
            {
                cont =1;
                CDMA_Descriptor[c][d].CommandFlags = (0 << 10) | (cont << 9) | (0 << 8)| 0x40;
            }



            CDMA_Descriptor[c][d].Channel = ( (0 << 15) | (1 << 14) | (c << CHANNEL_ID_OFFSET) |
                    (1 << (4+c)) |
                    d );


            CDMA_Descriptor[c][d].Status = 0;
            CDMA_Descriptor[c][d].Tag  = c*MAX_SYNC_POINTS + d;

        }
    }

    for (c =0; c < LLD_NUM_FLASH_CHANNELS; c++)
    {
        if(GLOB_DeviceInfo.wHWFeatures & FEATURES__INDEX_ADDR)
        {

            FBA_FPA_FSA_P = (uint32 *)(FlashMem);
            *FBA_FPA_FSA_P = (uint32 )(MODE_10 | (c << 24));


            FBA_FPA_FSA_P = (uint32 *)((uint32)FlashMem | 0x10);
            *FBA_FPA_FSA_P = (1 << 7) | c;


            FBA_FPA_FSA_P = (uint32 *)(FlashMem);
            *FBA_FPA_FSA_P = FCMD_INDX_TX2_ADD(c, CDMA_Descriptor);

            FBA_FPA_FSA_P = (uint32 *)((uint32)FlashMem | 0x10);
            *FBA_FPA_FSA_P = (1 << 7) | ( 1 << 4) | 0;


            FBA_FPA_FSA_P = (uint32 *)(FlashMem);
            *FBA_FPA_FSA_P = FCMD_INDX_TX3_ADD(c, CDMA_Descriptor);


            FBA_FPA_FSA_P = (uint32 *)((uint32)FlashMem | 0x10);
            *FBA_FPA_FSA_P = (1 << 7) | ( 1 << 5) | 0;


            FBA_FPA_FSA_P = (uint32 *)(FlashMem);
            *FBA_FPA_FSA_P = (uint32 )(MODE_10 | (c << 24) );


            FBA_FPA_FSA_P = (uint32 *)((uint32)FlashMem | 0x10);
            *FBA_FPA_FSA_P = (1 << 7) | ( 1 << 5) | (1 << 4) | 0;

        }
        else
        {
            FBA_FPA_FSA_P   = (uint32 *) (uint32)((uint32)FlashMem | MODE_10 | (c << 24) );
            *FBA_FPA_FSA_P = (1 << 7) | c;


            FBA_FPA_FSA_P   = (uint32 *)( (uint32)FlashMem | FCMD_INDX_TX2_ADD(c, CDMA_Descriptor));
            *FBA_FPA_FSA_P = (1 << 7) | ( 1 << 4) | 0;


            FBA_FPA_FSA_P   = (uint32 *)( (uint32)FlashMem | FCMD_INDX_TX3_ADD(c, CDMA_Descriptor));
            *FBA_FPA_FSA_P = (1 << 7) | ( 1 << 5) | 0;


            FBA_FPA_FSA_P   = (uint32 *)((uint32)FlashMem | MODE_10 |
                    (0 << 16)    | (0x40 << 8) );
            *FBA_FPA_FSA_P = (1 << 7) | ( 1 << 5) | (1 << 4) | 0;

        }
    }

    while( (FlashReg[DMA_INTR] &  (DMA_INTR__DESC_COMP_CHANNEL0|
                                        DMA_INTR__DESC_COMP_CHANNEL1|
                                        DMA_INTR__DESC_COMP_CHANNEL2|
                                        DMA_INTR__DESC_COMP_CHANNEL3))
                                !=     (DMA_INTR__DESC_COMP_CHANNEL0|
                                        DMA_INTR__DESC_COMP_CHANNEL1|
                                        DMA_INTR__DESC_COMP_CHANNEL2|
                                        DMA_INTR__DESC_COMP_CHANNEL3) );


    FlashReg[DMA_INTR] =  FlashReg[DMA_INTR];

    FlashReg[GLOBAL_INT_ENABLE]     =   GLOBAL_INT_ENABLE__FLAG;

}
/*&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
* Function:     CDMA_Event_Status (for use with CMD_DMA)
* Inputs:       none
* Outputs:      Event_Status code
* Description:  This function is called after an interrupt has happened
*               It reads the HW status register and ...tbd
*               It returns the appropriate event status
*&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&*/

uint16  CDMA_Event_Status(void) // for CMD-DMA
{
uint16  FIQstatus[LLD_NUM_FLASH_CHANNELS];   // local copy of controller HW reg

uint16  CMD_DMA_InterruptStatus;// local copy of controller HW reg
uint16  event = EVENT_NONE;     // var for return value

uint16  err_byte;
byte    err_sector;
byte    err_page=0;
byte    err_device;
uint16  ecc_correction_info;
uint16  err_address;
uint32  eccSectorSize;
uint16  i=0;
uint16  c,d;

    // Initialize event status to EVENT_PASS
    event = EVENT_PASS;

    // Take Width  Expansion (if any) into account
    eccSectorSize   = ECC_SECTOR_SIZE * (GLOB_DeviceInfo.wDevicesConnected);


    FIQstatus[0] = (FlashReg[INTR_STATUS0] & InterruptEnableMask);
    FIQstatus[1] = (FlashReg[INTR_STATUS1] & InterruptEnableMask);
    FIQstatus[2] = (FlashReg[INTR_STATUS2] & InterruptEnableMask);
    FIQstatus[3] = (FlashReg[INTR_STATUS3] & InterruptEnableMask);
    CMD_DMA_InterruptStatus =
                 (FlashReg[DMA_INTR] & CMD_DMA_InterruptEnableMask);

    if (CMD_DMA_InterruptStatus)
    {
        if ( (CMD_DMA_InterruptStatus & DMA_INTR__DESC_COMP_CHANNEL0)
            || (CMD_DMA_InterruptStatus & DMA_INTR__DESC_COMP_CHANNEL1)
            || (CMD_DMA_InterruptStatus & DMA_INTR__DESC_COMP_CHANNEL2)
            || (CMD_DMA_InterruptStatus & DMA_INTR__DESC_COMP_CHANNEL3))
        {
            event = EVENT_PASS;

            for(c=0; c < LLD_NUM_FLASH_CHANNELS; c++)
            {
                if(GLOB_valid_banks[c] == 1)
                {
                    d = dcount[c];
                    debug_boundary_error(d, MAX_DESC_PER_CHANNEL, 0);

                    for(i = 0; i < d; i++)
                    {
                        if (CDMA_Descriptor[c][i].Tag != 0xFF)
                            GLOB_PendingCMD[CDMA_Descriptor[c][i].Tag].Status = CMD_PASS;

#if FLASH_CDMA
                        if( (CDMA_Descriptor[c][i].CommandType   == 0x41) ||
                             (CDMA_Descriptor[c][i].CommandType   == 0x42)||
                             (CDMA_Descriptor[c][i].CommandType   == 0x43) )
                             continue;

                        switch (GLOB_PendingCMD[CDMA_Descriptor[c][i].Tag].CMD)
                        {
                        case READ_MAIN_SPARE_CMD:
                            NAND_Conv_Main_Spare_Data_Phy2Log_Format(GLOB_PendingCMD[CDMA_Descriptor[c][i].Tag].DataAddr, GLOB_PendingCMD[CDMA_Descriptor[c][i].Tag].PageCount);
                            break;
                        case READ_SPARE_CMD:
                            NAND_Conv_Spare_Data_Phy2Log_Format(GLOB_PendingCMD[CDMA_Descriptor[c][i].Tag].DataAddr);
                            break;
                        default:
                            break;
                        }
#endif
                    }
                }
            }
        }
        else
        {
            // TODO -- What kind of status can be reported back to FTL in PendindCMD
            event = EVENT_DMA_CMD_FAIL;
        }
        FlashReg[DMA_ENABLE] = 0;

        while((FlashReg[DMA_ENABLE] & DMA_ENABLE__FLAG));

        FlashReg[DMA_INTR] =  CMD_DMA_InterruptStatus;
    }



    for ( c=0; c <LLD_NUM_FLASH_CHANNELS; c++)
    {
        if (FIQstatus[c])
        {
            if ((FIQstatus[c] & INTR_STATUS0__ECC_ERR) && (FlashReg[ECC_ENABLE]))
            {


                d = dcount[c];

                for(i=0; i < d; i++)
                {
                    if ((CDMA_Descriptor[c][i].Status & CMD_DMA_DESC_COMP) != CMD_DMA_DESC_COMP)
                        break;

                }

                if (i == d)
                {
                    event = EVENT_UNCORRECTABLE_DATA_ERROR;
                    return event;
                }

                do
                {
                    if (c==0)
                        err_page = FlashReg[ERR_PAGE_ADDR0];
                    else if(c==1)
                        err_page = FlashReg[ERR_PAGE_ADDR1];
                    else if(c==2)
                        err_page = FlashReg[ERR_PAGE_ADDR2];
                    else if(c==3)
                        err_page = FlashReg[ERR_PAGE_ADDR3];

                    err_address = FlashReg[ECC_ERROR_ADDRESS];
                    err_byte = err_address & ECC_ERROR_ADDRESS__OFFSET ;
                    err_sector = ((err_address & ECC_ERROR_ADDRESS__SECTOR_NR)>>12);
                    /* correction_bytemask = FlashReg[ERR_CORRECTION_INFO]>>8; */

                    ecc_correction_info = FlashReg[ERR_CORRECTION_INFO];
                    err_device = ( ( ecc_correction_info & ERR_CORRECTION_INFO__DEVICE_NR) >> 8);

                    if(ecc_correction_info & ERR_CORRECTION_INFO__ERROR_TYPE)
                    {

                        if(CDMA_Descriptor[c][i].Tag != 0xFF)
                            GLOB_PendingCMD[CDMA_Descriptor[c][i].Tag].Status = CMD_FAIL;

                        CDMA_UpdateEventStatus();

                        FlashReg[DMA_ENABLE] = 0;

                        while((FlashReg[DMA_ENABLE] & DMA_ENABLE__FLAG));

                        FlashReg[INTR_STATUS0] =  FIQstatus[0];
                        FlashReg[INTR_STATUS1] =  FIQstatus[1];
                        FlashReg[INTR_STATUS2] =  FIQstatus[2];
                        FlashReg[INTR_STATUS3] =  FIQstatus[3];

                        CMD_DMA_InterruptStatus = (FlashReg[DMA_INTR] & CMD_DMA_InterruptEnableMask);
                        FlashReg[DMA_INTR] =    CMD_DMA_InterruptStatus;

                        event = EVENT_UNCORRECTABLE_DATA_ERROR;
                        return event;
                    }
                    else
                    {
                        if (err_byte < ECC_SECTOR_SIZE)
                        {
                            event = EVENT_CORRECTABLE_DATA_ERROR_FIXED;

                            *((byte *)((GLOB_PendingCMD[CDMA_Descriptor[c][i].Tag].DataAddr) + ((err_page -
                                    GLOB_PendingCMD[CDMA_Descriptor[c][i].Tag].Page) *  GLOB_DeviceInfo.wPageDataSize)
                                    + err_sector*eccSectorSize + (err_byte * GLOB_DeviceInfo.wDevicesConnected) +
                                    err_device) )^= (ecc_correction_info & ERR_CORRECTION_INFO__BYTEMASK);
                        }
                        else
                        {
                            event = EVENT_CORRECTABLE_DATA_ERROR_FIXED;
                        }
                    }
                }while(0 == (ecc_correction_info & ERR_CORRECTION_INFO__LAST_ERR_INFO));

                if (c==0)
                    FlashReg[INTR_STATUS0] = INTR_STATUS0__ECC_ERR;
                else if(c==1)
                    FlashReg[INTR_STATUS1] = INTR_STATUS1__ECC_ERR;
                else if(c==2)
                    FlashReg[INTR_STATUS2] = INTR_STATUS2__ECC_ERR;
                else if(c==3)
                    FlashReg[INTR_STATUS3] = INTR_STATUS3__ECC_ERR;

            }
            if (FIQstatus[c] & INTR_STATUS0__PROGRAM_FAIL)
            {
                if(CDMA_Descriptor[c][i].Tag != 0xFF)
                    GLOB_PendingCMD[CDMA_Descriptor[c][i].Tag].Status = CMD_FAIL;

                CDMA_UpdateEventStatus();

                // Disable DMA
                FlashReg[DMA_ENABLE] = 0;

                // Wait for DMA to be disabled before issuing the next command.
                while((FlashReg[DMA_ENABLE] & DMA_ENABLE__FLAG));

                FlashReg[INTR_STATUS0] =  FIQstatus[0];
                FlashReg[INTR_STATUS1] =  FIQstatus[1];
                FlashReg[INTR_STATUS2] =  FIQstatus[2];
                FlashReg[INTR_STATUS3] =  FIQstatus[3];

                CMD_DMA_InterruptStatus = (FlashReg[DMA_INTR] & CMD_DMA_InterruptEnableMask);
                FlashReg[DMA_INTR] =    CMD_DMA_InterruptStatus;

                event = EVENT_PROGRAM_FAILURE;
                return event;
            }
            if (FIQstatus[c] & INTR_STATUS0__ERASE_FAIL)
            {
                if(CDMA_Descriptor[c][i].Tag != 0xFF)
                    GLOB_PendingCMD[CDMA_Descriptor[c][i].Tag].Status = CMD_FAIL;

                CDMA_UpdateEventStatus();

                FlashReg[DMA_ENABLE] = 0;

                while((FlashReg[DMA_ENABLE] & DMA_ENABLE__FLAG));


                FlashReg[INTR_STATUS0] =  FIQstatus[0];
                FlashReg[INTR_STATUS1] =  FIQstatus[1];
                FlashReg[INTR_STATUS2] =  FIQstatus[2];
                FlashReg[INTR_STATUS3] =  FIQstatus[3];

                CMD_DMA_InterruptStatus = (FlashReg[DMA_INTR] & CMD_DMA_InterruptEnableMask);
                FlashReg[DMA_INTR] =    CMD_DMA_InterruptStatus;

                event = EVENT_ERASE_FAILURE;
                return event;
            }
            if (FIQstatus[c] & INTR_STATUS0__TIME_OUT)
            {
                d = dcount[c];
                for(i=0; i < d; i++)
                {
                    if(CDMA_Descriptor[c][i].Tag != 0xFF)
                        GLOB_PendingCMD[CDMA_Descriptor[c][i].Tag].Status = CMD_FAIL;
                }
                CDMA_AbortChannels(c);

                FlashReg[DMA_ENABLE] = 0;

                while((FlashReg[DMA_ENABLE] & DMA_ENABLE__FLAG));


                FlashReg[INTR_STATUS0] =  FIQstatus[0];
                FlashReg[INTR_STATUS1] =  FIQstatus[1];
                FlashReg[INTR_STATUS2] =  FIQstatus[2];
                FlashReg[INTR_STATUS3] =  FIQstatus[3];

                CMD_DMA_InterruptStatus = (FlashReg[DMA_INTR] & CMD_DMA_InterruptEnableMask);
                FlashReg[DMA_INTR] =    CMD_DMA_InterruptStatus;

                event = EVENT_TIME_OUT;
                return event;
            }
            else
            {
                if (c==0)
                    FlashReg[INTR_STATUS0] = FIQstatus[0];
                else if(c==1)
                    FlashReg[INTR_STATUS1] = FIQstatus[1];
                else if(c==2)
                    FlashReg[INTR_STATUS2] = FIQstatus[2];
                else if(c==3)
                    FlashReg[INTR_STATUS3] = FIQstatus[3];
            }
        }
    }


    return event;
}
#endif

#endif

/****** Sync related functions ********/
#define MAX_SYNC            (14)
#define FORCED_ORDERED_SYNC (15)
#define SNUS_CHAN_OFFSET    (24)
#define SNUS_LASTID_MASK    (0xFFFFFF)

#if DEBUG_SYNC
uint32  debug_sync_cnt = 1;
#endif

static uint32 isFlashReadCMD(byte CMD)
{
    switch(CMD) {
        case READ_MAIN_CMD :
        case READ_SPARE_CMD :
        case READ_MAIN_SPARE_CMD :
            return 1;
        default:
            break;
    }
    return 0;
}

static uint32 isFlashWriteCMD(byte CMD)
{
    switch(CMD) {
        case WRITE_MAIN_CMD :
        case WRITE_SPARE_CMD :
        case WRITE_MAIN_SPARE_CMD :
            return 1;
        default:
            break;
    }
    return 0;
}

/*&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
* Function:     generateSyncNum
* Inputs:       sync_usage array, a new sync number in case no reusable one
*               was found. The bit vector of channels taking place in current
*               sync operation, and the earliest cmd id for the new sync op.
* Outputs:      The sync number to be used for the current syncing
* Description:
* Assumption :  A sync point is always used between 2 and only 2 channels.
*&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&*/
static uint32 generateSyncNum(uint32 *sync_usage, uint32 *newSyncNum, uint32 syncedChans, uint32 lastid)
{
    uint32 synci, toUseSyncNum = 0;

#if 1
    // We try to reuse syncs as much as possible with this algorithm
    for (synci=1; synci<*newSyncNum; synci++) {
        if (((sync_usage[synci] >> SNUS_CHAN_OFFSET) == syncedChans) &&
            ((sync_usage[synci] & SNUS_LASTID_MASK) < lastid)) {
            toUseSyncNum = synci;
            break;
        }
    }
#endif
    if (!toUseSyncNum && (*newSyncNum <= MAX_SYNC))
        toUseSyncNum = (*newSyncNum)++;
#if 0
    The rest is to find another sync point which has at least one channel in common, and then add a sync point to the
    extra channel, and then use it.
    -- This will not result in the sync number being used by 3 channels, since the new use will have just the
    syncedChans values. So our assumption still holds valid.
    -- However, adding the new channel is not easy. We need to find the id, which is after the last sync number
    that existed between the common channel, and our new channel, and before the next sync that will exist
    between the new and common channel!
    else {
        for (synci=1; synci<=MAX_SYNC; synci++) {
            if (((sync_usage[synci] >> SNUS_CHAN_OFFSET) & syncedChans) &&
                ((sync_usage[synci] & SNUS_LASTID_MASK) < lastid)) {
            }
        }
    }
#endif

    return toUseSyncNum;
}


#define getChannelGLOB_PendingCMD(pend_cmd_index)    (GLOB_PendingCMD[pend_cmd_index].Block / (GLOB_DeviceInfo.wTotalBlocks / GLOB_totalUsedBanks))

#define isOrderedGLOB_PendingCMD(pend_cmd_index)     ((GLOB_PendingCMD[pend_cmd_index].Flags & LLD_CMD_FLAG_ORDER_BEFORE_REST) != 0)

#define getSyncFromChannel(c)                   ((c & CHANNEL_SYNC_MASK ) >> CHANNEL_SYNC_OFFSET )
#define getIdFromChannel(c)                     ((c & CHANNEL_ID_MASK   ) >> CHANNEL_ID_OFFSET )
#define getContFromChannel(c)                   ((c & CHANNEL_CONT_MASK ) >> CHANNEL_CONT_OFFSET )
#define getIntrFromChannel(c)                   ((c & CHANNEL_INTR_MASK ) >> CHANNEL_INTR_OFFSET )
#define getChanFromChannel(c)                   ((c & CHANNEL_DMA_MASK  ) >> CHANNEL_DMA_OFFSET )

#define putSyncInChannel(c, v)                  (c |= ((v << CHANNEL_SYNC_OFFSET ) & CHANNEL_SYNC_MASK ))
#define putIdInChannel(c, v)                    (c |= ((v << CHANNEL_ID_OFFSET   ) & CHANNEL_ID_MASK ))
#define putContInChannel(c, v)                  (c |= ((v << CHANNEL_CONT_OFFSET ) & CHANNEL_CONT_MASK ))
#define putIntrInChannel(c, v)                  (c |= ((v << CHANNEL_INTR_OFFSET ) & CHANNEL_INTR_MASK ))
#define putChanInChannel(c, v)                  (c |= ((v << CHANNEL_DMA_OFFSET  ) & CHANNEL_DMA_MASK ))

#define addChanToChannel(c, v)                  (c |= ((1 << CHANNEL_DMA_OFFSET ) << v ))

#define isWithinRange(toChk, Addr, Bytes)       ((toChk >= Addr) && (toChk < (Addr + Bytes)))
//#define isWithinRange(toChk, Addr, Bytes)       (toChk == Addr)

/*&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
* Function:     CDMA_AddSyncPoints
* Inputs:       tag_count:- Number of commands in GLOB_PendingCMD list
* Outputs:      NONE
* Description:  This function takes the GLOB_PendingCMD list, and adds sync
*               points between each entry on it, and any preceding entry
*               in other channels that have conflicts with the Cache Block
*               pointer.
*               The design also takes care of syncing between memcopy
*               and flash read/write operations. However, this function
*               does not sync between 2 memcopy operations that have a conflict
*               in a RAM pointer other than the cache block one. It is the
*               responsibility of the calling function, probablt the
*               application calling spectra, to take care of that.
* Assumptions:  + This function is before the CDMA_Descriptor list is created.
*               + This function takes care of the fact that memcopy accesses
*                 might be just a few bytes within a cache block, and uses a
*                 knowledge of the cache block to check for accesses anywhere
*                 within it. However, it is assumed that we dont have ranges
*                 that overlap one another. Either ranges overlap perfectly, or
*                 the memcopy range is a subset of the flash address range.
*&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&*/















void CDMA_AddSyncPoints(uint16 tag_count)
// Global Variables used here. GLOB_PendingCMD
{
    uint32 i, j, k, l, numSync, numSyncOther, chnl, chnlOther, stopLoop, newSyncNum, toUseSyncNum,
      syncedChans, addrWithinRange, synci;
    uint32 indx_last_cmd[LLD_NUM_FLASH_CHANNELS];
    uint32 ifnathenmb[LLD_NUM_FLASH_CHANNELS][LLD_NUM_FLASH_CHANNELS];
    uint32 sync_usage[MAX_SYNC+1];
    byte  *fromAddr, *toAddr, CMD;
    uint32 writeOpSyncPlaced;

    /* Initializations */
    newSyncNum = 1;
    debug_boundary_error(GLOB_totalUsedBanks-1, LLD_NUM_FLASH_CHANNELS, 0);
    for (i=0; i<GLOB_totalUsedBanks; i++) {
        chnl = getChannelGLOB_PendingCMD(i);
        debug_boundary_error(chnl, GLOB_totalUsedBanks, i);
        indx_last_cmd[chnl] = i;
        for (j=0; j<GLOB_totalUsedBanks; j++)
            ifnathenmb[i][j] = 0;
    }
    for (synci=0; synci<=MAX_SYNC; synci++) {
        sync_usage[synci] = 0;
    }

    /* Begin Main Loop */
    for (i=LLD_NUM_FLASH_CHANNELS; i<(tag_count + LLD_NUM_FLASH_CHANNELS); i++) {
        writeOpSyncPlaced = 0;
        CMD = GLOB_PendingCMD[i].CMD;
        chnl = getChannelGLOB_PendingCMD(i);
        debug_boundary_error(chnl, GLOB_totalUsedBanks, i);

        if (CMD == MEMCOPY_CMD) {
            fromAddr = GLOB_PendingCMD[i].DataSrcAddr;
            toAddr = GLOB_PendingCMD[i].DataDestAddr;
            stopLoop = 0;

            for (j=i-1; (j>=LLD_NUM_FLASH_CHANNELS) && !stopLoop; j--) {

                addrWithinRange = isWithinRange(toAddr, GLOB_PendingCMD[j].DataAddr, GLOB_PendingCMD[j].PageCount * GLOB_DeviceInfo.wPageDataSize);
                if (((GLOB_PendingCMD[j].CMD != MEMCOPY_CMD) &&
                     (GLOB_PendingCMD[j].CMD != ERASE_CMD)
                    ) &&
                    (addrWithinRange ||
                     isWithinRange(fromAddr, GLOB_PendingCMD[j].DataAddr, GLOB_PendingCMD[j].PageCount * GLOB_DeviceInfo.wPageDataSize)
                    )
                   ) {
                    stopLoop = 1;
                    GLOB_PendingCMD[i].Block = GLOB_PendingCMD[j].Block;
                    chnl = getChannelGLOB_PendingCMD(i);
                    debug_boundary_error(chnl, GLOB_totalUsedBanks, i);
                    if (isFlashWriteCMD(GLOB_PendingCMD[j].CMD) &&
                        addrWithinRange) {
                        CMD = READ_MAIN_CMD;
                        GLOB_PendingCMD[i].DataAddr = toAddr;
                    }
                }
            }
        }
        if (isFlashReadCMD(CMD) || isFlashWriteCMD(CMD)) {
            fromAddr = GLOB_PendingCMD[i].DataAddr;
            k = indx_last_cmd[chnl];
            stopLoop = 0;

            for (j=i-1; (j>=LLD_NUM_FLASH_CHANNELS) && !stopLoop; j--) {

                chnlOther = getChannelGLOB_PendingCMD(j);
                debug_boundary_error(chnlOther, GLOB_totalUsedBanks, j);
                addrWithinRange = isWithinRange(GLOB_PendingCMD[j].DataDestAddr, fromAddr, GLOB_PendingCMD[i].PageCount * GLOB_DeviceInfo.wPageDataSize);
                if (((fromAddr == GLOB_PendingCMD[j].DataAddr) ||
                     ((GLOB_PendingCMD[j].CMD == MEMCOPY_CMD) &&
                      (addrWithinRange ||
                       isWithinRange(GLOB_PendingCMD[j].DataSrcAddr, fromAddr, GLOB_PendingCMD[i].PageCount * GLOB_DeviceInfo.wPageDataSize)
                      )
                     )
                    )
                   ) {

                    if (ifnathenmb[chnl][chnlOther] >= j) {
                        stopLoop = 1;
                    }
                    else if (chnlOther == chnl) {
                        if ((isFlashWriteCMD(CMD)  ||
                             isFlashReadCMD(GLOB_PendingCMD[j].CMD) ||
                             ((GLOB_PendingCMD[j].CMD == MEMCOPY_CMD) &&
                              addrWithinRange
                             )
                            )
                           ) {
                            stopLoop = 1;
                        }
                    }
                    else {
                        if (isFlashReadCMD(CMD) ||
                            isFlashReadCMD(GLOB_PendingCMD[j].CMD) ||
                            ((GLOB_PendingCMD[j].CMD == MEMCOPY_CMD) &&
                             addrWithinRange
                            )) {
                            if (isFlashReadCMD(GLOB_PendingCMD[j].CMD) ||
                                ((GLOB_PendingCMD[j].CMD == MEMCOPY_CMD) &&
                                 addrWithinRange
                                )) {
                                stopLoop = 1;
                                if (writeOpSyncPlaced)
                                    break;
                            }
                            if (writeOpSyncPlaced & (1 << chnlOther))
                                break;

                            for (numSync=0; (numSync<=LLD_NUM_FLASH_CHANNELS) && (GLOB_PendingCMD[k].ChanSync[numSync] & CHANNEL_DMA_MASK); numSync++);
                            for (numSyncOther=0; (numSyncOther<=LLD_NUM_FLASH_CHANNELS) && (GLOB_PendingCMD[j].ChanSync[numSyncOther] & CHANNEL_DMA_MASK); numSyncOther++);

                            if ((numSync>LLD_NUM_FLASH_CHANNELS) || (numSyncOther>LLD_NUM_FLASH_CHANNELS))
                                print("LLD_CDMA: Sync Algorithm failed to place a Sync between command tags %ld and %ld\n", i-LLD_NUM_FLASH_CHANNELS, j-LLD_NUM_FLASH_CHANNELS);
                            else {
                                writeOpSyncPlaced |= (1 << chnlOther);
                                syncedChans = ((1 << chnl) | (1 << chnlOther));

                                if (!(toUseSyncNum = generateSyncNum(&sync_usage[0], &newSyncNum, syncedChans, (j<k?j:k)))) {
                                    print("LLD_CDMA: Sync Algorithm ran out of Syncs during syncing command tags %ld and %ld\n", i-LLD_NUM_FLASH_CHANNELS, j-LLD_NUM_FLASH_CHANNELS);
                                } else {
                                    putSyncInChannel(GLOB_PendingCMD[k].ChanSync[numSync], toUseSyncNum);
                                    putContInChannel(GLOB_PendingCMD[k].ChanSync[numSync], 1);
                                    putIdInChannel(GLOB_PendingCMD[k].ChanSync[numSync], chnl);

                                    putSyncInChannel(GLOB_PendingCMD[j].ChanSync[numSyncOther], toUseSyncNum);
                                    putContInChannel(GLOB_PendingCMD[j].ChanSync[numSyncOther], 1);
                                    putIdInChannel(GLOB_PendingCMD[j].ChanSync[numSyncOther], chnlOther);

                                    putChanInChannel(GLOB_PendingCMD[j].ChanSync[numSyncOther], syncedChans);
                                    putChanInChannel(GLOB_PendingCMD[k].ChanSync[numSync], syncedChans);

                                    sync_usage[toUseSyncNum] = (syncedChans << SNUS_CHAN_OFFSET) | ((j>k?j:k) & SNUS_LASTID_MASK);

                                    ifnathenmb[chnl][chnlOther] = j;
                                    if (ifnathenmb[chnlOther][chnl] > k)
                                        print("LLD_CDMA: Sync Algorithm detected a possible deadlock in its assignments.\n");
                                    else
                                        ifnathenmb[chnlOther][chnl] = k;
                                    for (l=0; l<GLOB_totalUsedBanks; l++) {
                                        if ((l!=chnl) && (l!=chnlOther)) {
                                            if (ifnathenmb[l][chnlOther] <= j) {
                                                if (ifnathenmb[chnl][l] < ifnathenmb[chnlOther][l])
                                                    ifnathenmb[chnl][l] = ifnathenmb[chnlOther][l];
                                            } else {
                                                if (ifnathenmb[l][chnl] < ifnathenmb[chnlOther][chnl])
                                                    ifnathenmb[l][chnl] = ifnathenmb[chnlOther][chnl];
                                            }
                                            if (ifnathenmb[l][chnl] <= k) {
                                                if (ifnathenmb[chnlOther][l] < ifnathenmb[chnl][l])
                                                    ifnathenmb[chnlOther][l] = ifnathenmb[chnl][l];
                                            } else {
                                                if (ifnathenmb[l][chnlOther] < ifnathenmb[chnl][chnlOther])
                                                    ifnathenmb[l][chnlOther] = ifnathenmb[chnl][chnlOther];
                                            }
                                        }
                                    }
#if DEBUG_SYNC
                                    {
                                        int m;
                                        if (!(debug_sync_cnt % DBG_SNC_PRINTEVERY)) {
                                            print("ADDSYNC: Placed Sync point 0x%x with chanvectors 0x%x betn tags %ld & prev(%ld)=%ld\n", (unsigned)toUseSyncNum, (unsigned)syncedChans, j-LLD_NUM_FLASH_CHANNELS, i-LLD_NUM_FLASH_CHANNELS, k-LLD_NUM_FLASH_CHANNELS);
                                            for (m=0; m<GLOB_totalUsedBanks; m++) {
                                                print("ADDSYNC: ch:%d ->", m);
                                                for (l=0; l<GLOB_totalUsedBanks; l++)
                                                    print(" (ch:%lu tag:%3d)", l, GLOB_PendingCMD[ifnathenmb[m][l]].Tag==255?
                                                            -1:GLOB_PendingCMD[ifnathenmb[m][l]].Tag);
                                                print("\n");
                                            }
                                        }
                                    }
#endif
                                }
                            }
                        }
                    }
                }
            }
        }
        indx_last_cmd[chnl] = i;

        // Simple one sync to rule them all approach
        if (isOrderedGLOB_PendingCMD(i)) {
		uint32 syncNums[LLD_NUM_FLASH_CHANNELS];

		stopLoop = 0;
            for (k=i-1; (k>=LLD_NUM_FLASH_CHANNELS); k--) {
		if (chnl != getChannelGLOB_PendingCMD(k))
			k = LLD_NUM_FLASH_CHANNELS-1;
		else if (isOrderedGLOB_PendingCMD(k))
			break;
            }

            if (k>=LLD_NUM_FLASH_CHANNELS) {
                for (syncNums[chnl]=0; (syncNums[chnl]<=LLD_NUM_FLASH_CHANNELS) && (GLOB_PendingCMD[i].ChanSync[syncNums[chnl]] & CHANNEL_DMA_MASK); syncNums[chnl]++);

                if ((syncNums[chnl]>LLD_NUM_FLASH_CHANNELS)) {
                    print("LLD_CDMA: Sync Algorithm failed to place a Forced Sync at command tag %ld\n", i-LLD_NUM_FLASH_CHANNELS);
                }
                else {
                    chnlOther = (chnl+1) % GLOB_totalUsedBanks;
                    for (syncNums[chnlOther]=0; (syncNums[chnlOther]<=LLD_NUM_FLASH_CHANNELS) &&
                    (getSyncFromChannel(GLOB_PendingCMD[k].ChanSync[syncNums[chnlOther]])!=FORCED_ORDERED_SYNC); syncNums[chnlOther]++);

                    if ((syncNums[chnlOther]>LLD_NUM_FLASH_CHANNELS)) {
                        print("LLD_CDMA: Sync Algorithm failed find previously placed Forced Sync at command tag %d, chnl %lu\n", (int)k-LLD_NUM_FLASH_CHANNELS, chnl);
                    }
                    else {
                        syncedChans = getChanFromChannel(GLOB_PendingCMD[k].ChanSync[syncNums[chnlOther]]);
                        l = getIntrFromChannel(GLOB_PendingCMD[k].ChanSync[syncNums[chnlOther]]);
                        GLOB_PendingCMD[k].ChanSync[syncNums[chnlOther]] = 0;
                        putIntrInChannel(GLOB_PendingCMD[k].ChanSync[syncNums[chnlOther]], l);
                        putSyncInChannel(GLOB_PendingCMD[i].ChanSync[syncNums[chnl]], FORCED_ORDERED_SYNC);
                        putContInChannel(GLOB_PendingCMD[i].ChanSync[syncNums[chnl]], 1);
                        putIdInChannel(GLOB_PendingCMD[i].ChanSync[syncNums[chnl]], chnl);
                        putChanInChannel(GLOB_PendingCMD[i].ChanSync[syncNums[chnl]], syncedChans);

                        for (l=0; l<GLOB_totalUsedBanks; l++) {
                            if (l!=chnl) {
                                ifnathenmb[l][chnl] = i;
                            }
                        }
    #if DEBUG_SYNC
                        {
                            if (!(debug_sync_cnt % DBG_SNC_PRINTEVERY)) {
                                print("ADDSYNC: Moved Forced Sync point in chnl %lu from tag %ld to %ld\n", chnl, k-LLD_NUM_FLASH_CHANNELS, i-LLD_NUM_FLASH_CHANNELS);
                            }
                        }
    #endif
                    }
                }
            }
            else {
                syncedChans = 0;
			for (j=0; j<GLOB_totalUsedBanks; j++) {
			k = indx_last_cmd[j];
	                for (syncNums[j]=0; (syncNums[j]<=LLD_NUM_FLASH_CHANNELS) && (GLOB_PendingCMD[k].ChanSync[syncNums[j]] & CHANNEL_DMA_MASK); syncNums[j]++);

	                if ((syncNums[j]>LLD_NUM_FLASH_CHANNELS)) {// This should not happen!!!
	                    print("LLD_CDMA: Sync Algorithm failed to place a Forced Sync at command tag %ld\n", k-LLD_NUM_FLASH_CHANNELS);
	                    syncNums[0] = LLD_NUM_FLASH_CHANNELS+1;
	                }
	                syncedChans |= (1<<j);
	            }
			if (syncNums[0] <= LLD_NUM_FLASH_CHANNELS) {
		            for (j=0; j<GLOB_totalUsedBanks; j++) {
				k = indx_last_cmd[j];

	                    putSyncInChannel(GLOB_PendingCMD[k].ChanSync[syncNums[j]], FORCED_ORDERED_SYNC);
	                    putContInChannel(GLOB_PendingCMD[k].ChanSync[syncNums[j]], 1);
	                    putIdInChannel(GLOB_PendingCMD[k].ChanSync[syncNums[j]], j);
	                    putChanInChannel(GLOB_PendingCMD[k].ChanSync[syncNums[j]], syncedChans);

	                    for (l=0; l<GLOB_totalUsedBanks; l++) {
	                        if (l!=j) {
					ifnathenmb[l][j] = k;
	                        }
	                    }
	                }
#if DEBUG_SYNC
                    {
                        int m;
                        if (!(debug_sync_cnt % DBG_SNC_PRINTEVERY)) {
                            print("ADDSYNC: Placed Forced Sync point for tag %ld in tags", i-LLD_NUM_FLASH_CHANNELS);
                            for (m=0; m<GLOB_totalUsedBanks; m++) {
                                if (m!=chnl) print(" %d", (int)indx_last_cmd[m]-LLD_NUM_FLASH_CHANNELS);
                            }
                            print("\n");
                        }
                    }
#endif
			}
            }
        }/* if (isOrderedGLOB_PendingCMD(i)) */
    }/* End Main Loop */

    return;
}

#if DEBUG_SYNC
/*&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
* Function:     CDMA_SyncCheck
* Inputs:       tag_count:- Number of commands in GLOB_PendingCMD list
* Outputs:      NONE
* Description:  This function takes a long time to run!
*               So use only during testing with lld_emu. The job of this fn
*               is to go through the post-synced GLOB_PendingCMD array, and check
*               for a) buffers getting accessed out of order (which should
*               not happen), and b) deadlocks. i.e. 2 channels waiting on 2
*               different sync points both of which occur on the other channel
*&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&*/

#include "flash.h"
extern FLASH_CACHE GLOB_Cache;

typedef struct pencmdindex {
  uint32 cmdi; // command index
  uint32 chans; // chanSync field index
} pencmdindex_t;

#define EOLIST(i)   (chis[i] >= chMaxIndexes[i])

static void CSP_printOps(PENDING_CMD (*GLOB_PendingCMDChannel)[LLD_NUM_FLASH_CHANNELS+MAX_DESCRIPTORS], uint32 rwop, uint32 i, uint32 chisi)
{
    if (rwop & 2)
        print("one or more read operations(indx:%lu, tag:%d)", chisi>>16, GLOB_PendingCMDChannel[i][chisi>>16].Tag);
    if (rwop & 1)
        print(" one or more write operations(indx:%lu, tag:%d)", chisi & 0xFFFF, GLOB_PendingCMDChannel[i][chisi & 0xFFFF].Tag);
}

static byte CSP_getSyncChanFromPCMD(PENDING_CMD (*GLOB_PendingCMDChannel)[LLD_NUM_FLASH_CHANNELS+MAX_DESCRIPTORS], uint32 i, uint32 chisi, uint32 *syncNum, uint32 *i2)
{
    uint32 syncVal;
    syncVal = GLOB_PendingCMDChannel[i][chisi].ChanSync[0];
    if (syncVal) {
        *syncNum = getSyncFromChannel(syncVal);
        *i2 = getChanFromChannel(syncVal) & ~(1<<i);
        if ((*i2 != 1) && (*i2 != 2) && (*i2 != 4) && (*i2 != 8) && (*syncNum != FORCED_ORDERED_SYNC))
            print("SYNCCHECK: ASSERT FAIL: second channel of sync(%lu) got from sync val of (ch:%lu, indx:%lu, tag:%d) is not a valid one!\n", *i2, i, chisi, GLOB_PendingCMDChannel[i][chisi].Tag);
        *i2 = (*i2==1)?0:(*i2==2?1:(*i2==4?2:(i!=3?3:2)));
    }
    return (syncVal!=0);
}

static uint32 CSP_checkOrdering(PENDING_CMD (*GLOB_PendingCMDChannel)[LLD_NUM_FLASH_CHANNELS+MAX_DESCRIPTORS],
        uint32 ch1, uint32 ch1_fromi, uint32 ch1_toi, uint32 ch2, uint32 ch2_fromi, uint32 ch2_toi)
{
    uint32 sync2syncops[2], i, j, rwop1, rwop2, lastcmd[2][CACHE_BLOCK_NUMBER];
    uint32 chi, ch, chfromi, chtoi;
    uint32 allok = 1;

    for (chi=0; chi<2; chi++) {
        if (chi) {
            ch = ch2; chfromi = ch2_fromi; chtoi = ch2_toi;
        } else {
            ch = ch1; chfromi = ch1_fromi; chtoi = ch1_toi;
        }
        sync2syncops[chi] = 0;
        for (j=0; j<CACHE_BLOCK_NUMBER; j++)
            lastcmd[chi][j] = 0;
        for (i=chfromi; i<=chtoi; i++) {
            for (j=0; j<CACHE_BLOCK_NUMBER; j++) {
                if ((isFlashReadCMD(GLOB_PendingCMDChannel[ch][i].CMD) &&
                     (GLOB_PendingCMDChannel[ch][i].DataAddr == GLOB_Cache.ItemArray[j].pContent)
                    ) ||
                    ((GLOB_PendingCMDChannel[ch][i].CMD == MEMCOPY_CMD) &&
                     isWithinRange(GLOB_PendingCMDChannel[ch][i].DataDestAddr, GLOB_Cache.ItemArray[j].pContent, GLOB_DeviceInfo.wBlockDataSize)
                    )) {
                    sync2syncops[chi] |= (1<<(j<<1));
                    lastcmd[chi][j] &= 0xFFFF0000;
                    lastcmd[chi][j] |= (i & 0xFFFF);
                }
                if ((isFlashWriteCMD(GLOB_PendingCMDChannel[ch][i].CMD) &&
                     (GLOB_PendingCMDChannel[ch][i].DataAddr == GLOB_Cache.ItemArray[j].pContent)
                    ) ||
                    ((GLOB_PendingCMDChannel[ch][i].CMD == MEMCOPY_CMD) &&
                     isWithinRange(GLOB_PendingCMDChannel[ch][i].DataSrcAddr, GLOB_Cache.ItemArray[j].pContent, GLOB_DeviceInfo.wBlockDataSize)
                    )) {
                    sync2syncops[chi] |= (1<<((j<<1)+1));
                    lastcmd[chi][j] &= 0xFFFF;
                    lastcmd[chi][j] |= ((i & 0xFFFF) << 16);
                }
            }
        }
    }
    for (j=0; j<CACHE_BLOCK_NUMBER; j++) {
        rwop1 = (sync2syncops[0] >> (j<<1)) & 3;
        rwop2 = (sync2syncops[1] >> (j<<1)) & 3;
        if (((rwop1 & 1) && rwop2) || ((rwop2 & 1) && rwop1)) {
            print("SYNCCHECK: ORDERING PROBLEM in cache buffer %lu: Between (ch:%lu, indx:%lu, tag:%d) & \
(ch:%lu, indx:%lu, tag:%d), there has been \n", j, ch1, ch1_fromi, GLOB_PendingCMDChannel[ch1][ch1_fromi].Tag, ch1, ch1_toi, GLOB_PendingCMDChannel[ch1][ch1_toi].Tag);
            CSP_printOps(GLOB_PendingCMDChannel, rwop1, ch1, lastcmd[0][j]);
            print(".\nWhich are not ordered w.r.t to ");
            CSP_printOps(GLOB_PendingCMDChannel, rwop2, ch2, lastcmd[1][j]);
            print("\nbetween (ch:%lu, indx:%lu, tag:%d) & (ch:%lu, indx:%lu, tag:%d).\n",
                    ch2, ch2_fromi, GLOB_PendingCMDChannel[ch2][ch2_fromi].Tag, ch2, ch2_toi, GLOB_PendingCMDChannel[ch2][ch2_toi].Tag);
            allok = 0;
        }
    }
    return allok;
}

void CDMA_CheckSyncPoints(uint16 tag_count)
{
    PENDING_CMD GLOB_PendingCMDChannel[LLD_NUM_FLASH_CHANNELS][LLD_NUM_FLASH_CHANNELS+MAX_DESCRIPTORS];
    uint32 chMaxIndexes[LLD_NUM_FLASH_CHANNELS];
    int ifnathenmb[LLD_NUM_FLASH_CHANNELS][LLD_NUM_FLASH_CHANNELS];
    int chis[LLD_NUM_FLASH_CHANNELS];
    uint32 i, j, k, alldone, ch1, ch2, syncNum, syncNum2;
    byte indexchgd;

    /* Initial Checks */
    if (CACHE_BLOCK_NUMBER > 16) {
        print("SYNCCHECK: INIT FAILED: SyncCheck can only work with upto 16 cache blocks \n");
        return;
    }
    /* Initializations */
    for (i=0; i<GLOB_totalUsedBanks; i++) {
        chis[i] = 0;
        for (j=0; j<GLOB_totalUsedBanks; j++) {
            ifnathenmb[i][j] = -1;
        }
    }
    GLOB_PendingCMDToPerChannelArray(GLOB_PendingCMDChannel, tag_count, chMaxIndexes);
        if (!(debug_sync_cnt % DBG_SNC_PRINTEVERY)) {
    print("SYNCCHECK: Cache Ptrs:");
    for (j=0; j<CACHE_BLOCK_NUMBER; j++)
        print(" %p", GLOB_Cache.ItemArray[j].pContent);
    print("\n");
        }
    /* Begin main loop */
    alldone = 0;
    while (!alldone) {

        for (i=0; i<GLOB_totalUsedBanks; i++) {
            while (!EOLIST(i)) {
                if (!GLOB_PendingCMDChannel[i][chis[i]].ChanSync[0])
                    chis[i]++;
                else
                    break;
            }
        }

        for (i=0; !alldone && (i<GLOB_totalUsedBanks); i++) {
            if (!EOLIST(i) && CSP_getSyncChanFromPCMD(GLOB_PendingCMDChannel, i, chis[i], &syncNum, &ch1)) {
                j = 0;
                ch2 = ch1;
                ch1 = i;
                syncNum2 = syncNum;
                syncNum = 0xFF;
                while ((syncNum!=syncNum2) && (j<=GLOB_totalUsedBanks) && !EOLIST(ch2) && (ch2!=i)
				&& ((syncNum == 0xFF) || (syncNum2!=FORCED_ORDERED_SYNC))) {
                    ch1 = ch2; syncNum = syncNum2;
                    CSP_getSyncChanFromPCMD(GLOB_PendingCMDChannel, ch1, chis[ch1], &syncNum2, &ch2);
                    j++;
                }
                if ((j<=GLOB_totalUsedBanks) && (syncNum!=syncNum2)) {
                    print("SYNCCHECK: DEADLOCK:\n");
                    ch1 = i;
                    syncNum = 0xFF;
                    CSP_getSyncChanFromPCMD(GLOB_PendingCMDChannel, ch1, chis[ch1], &syncNum2, &ch2);
                    debug_boundary_error(ch2, GLOB_totalUsedBanks, 0);
                    while (!EOLIST(ch2) && (ch2!=i)
				&& ((syncNum == 0xFF) || (syncNum2!=FORCED_ORDERED_SYNC))) {
                        print("Channel %lu, cmdindx %d, tag %d is waiting for sync number %lu from channel %lu\n",
                                ch1, chis[ch1], GLOB_PendingCMDChannel[ch1][chis[ch1]].Tag, syncNum2, ch2);
                        ch1 = ch2; syncNum = syncNum2;
                        CSP_getSyncChanFromPCMD(GLOB_PendingCMDChannel, ch1, chis[ch1], &syncNum2, &ch2);
                        debug_boundary_error(ch2, GLOB_totalUsedBanks, 0);
                    }
                    print("Channel %lu, cmdindx %d, tag %d is waiting for sync number %lu from channel %lu",
                            ch1, chis[ch1], GLOB_PendingCMDChannel[ch1][chis[ch1]].Tag, syncNum2, ch2);
                    if (!EOLIST(ch2))
                        print(", which is the initial channel!\n");
                    else if (syncNum2!=FORCED_ORDERED_SYNC)
                        print(" which does not have that sync number!\n");
                    else
                        print(" which is th forced ordered sync number that cannot proceed until all channels reach it!\n");
                    print("Sync checking is aborting.\n");
                    alldone = 1;
                }
                if (j>GLOB_totalUsedBanks) {
                    print("SYNCCHECK: DEADLOCK: Unknown case. Infinite loop in deadlock check. Aborting.\n");
                    alldone = 1;
                }
            }
        }


        indexchgd = 0;
        for (i=0; (i<GLOB_totalUsedBanks) && !alldone && !indexchgd; i++) {
            if (!EOLIST(i) && CSP_getSyncChanFromPCMD(GLOB_PendingCMDChannel, i, chis[i], &syncNum, &ch1)) {
                debug_boundary_error(ch1, GLOB_totalUsedBanks, 0);
                if (!EOLIST(ch1) && CSP_getSyncChanFromPCMD(GLOB_PendingCMDChannel, ch1, chis[ch1], &syncNum2, &ch2)) {
                    debug_boundary_error(ch2, GLOB_totalUsedBanks, 0);

                    if ((syncNum == syncNum2) && (syncNum != FORCED_ORDERED_SYNC)) {
                        if (ch2 != i) {
                            print("SYNCCHECK: ILLEGAL CASE: Channel %lu, cmdindx %d, tag %d is waiting for sync number %lu \
from channel %lu, which is waiting for the same sync number from channel %lu. \
Sync checking is aborting.\n", i, chis[i], GLOB_PendingCMDChannel[i][chis[i]].Tag, syncNum, ch1, ch2);
                            alldone = 1;
                        } else {
                                if (!(debug_sync_cnt % DBG_SNC_PRINTEVERY)) {
                            print("SYNCCHECK: syncnum %lu betn Ch %lu, cmdindx %d, tag %d &\
 Ch %lu, cmdindx %d, tag %d. chis={%d, %d, %d, %d}\n", syncNum, i, chis[i],\
 GLOB_PendingCMDChannel[i][chis[i]].Tag, ch1, chis[ch1], GLOB_PendingCMDChannel[ch1][chis[ch1]].Tag,\
 chis[0], chis[1], chis[2], chis[3]);
                                }
                            if (!CSP_checkOrdering(GLOB_PendingCMDChannel, i, ifnathenmb[ch1][i]+1, chis[i], ch1, ifnathenmb[i][ch1]+1, chis[ch1]))
                                print("Above problem occured when analyzing sync %lu between (ch:%lu, indx:%d, tag:%d) & (ch:%lu, indx:%d, tag:%d)\n",
                                        syncNum, i, chis[i], GLOB_PendingCMDChannel[i][chis[i]].Tag, ch1, chis[ch1], GLOB_PendingCMDChannel[ch1][chis[ch1]].Tag);
                            ifnathenmb[ch1][i] = chis[i];
                            ifnathenmb[i][ch1] = chis[ch1];

                            for (k=0; k<GLOB_totalUsedBanks; k++) {
                                if ((k!=i) && (k!=ch1)) {
                                    if (ifnathenmb[ch1][k] > ifnathenmb[i][k]) {
                                        if (!CSP_checkOrdering(GLOB_PendingCMDChannel, i, ifnathenmb[k][i]+1, chis[i], k, ifnathenmb[i][k]+1, ifnathenmb[ch1][k]))
                                            print("Above problem occured when analyzing sync %lu between (ch:%lu, indx:%d, tag:%d) & (ch:%lu, indx:%d, tag:%d)\n",
                                                    syncNum, i, chis[i], GLOB_PendingCMDChannel[i][chis[i]].Tag, ch1, chis[ch1], GLOB_PendingCMDChannel[ch1][chis[ch1]].Tag);
                                        ifnathenmb[i][k] = ifnathenmb[ch1][k];
                                    } else if (ifnathenmb[ch1][k] < ifnathenmb[i][k]) {
                                        if (!CSP_checkOrdering(GLOB_PendingCMDChannel, ch1, ifnathenmb[k][ch1]+1, chis[ch1], k, ifnathenmb[ch1][k]+1, ifnathenmb[i][k]))
                                            print("Above problem occured when analyzing sync %lu between (ch:%lu, indx:%d, tag:%d) & (ch:%lu, indx:%d, tag:%d)\n",
                                                    syncNum, i, chis[i], GLOB_PendingCMDChannel[i][chis[i]].Tag, ch1, chis[ch1], GLOB_PendingCMDChannel[ch1][chis[ch1]].Tag);
                                        ifnathenmb[ch1][k] = ifnathenmb[i][k];
                                    }
                                }
                            }
                            chis[i]++;
                            chis[ch1]++;
                            indexchgd = 1;
                        }
                    }
                    else if ((syncNum == FORCED_ORDERED_SYNC) && (syncNum2 == FORCED_ORDERED_SYNC)) {
                        for (k=0; k<GLOB_totalUsedBanks; k++) {
				if ((k!=i) && (k!=ch1)) {
                                if (!EOLIST(k) && CSP_getSyncChanFromPCMD(GLOB_PendingCMDChannel, k, chis[k], &syncNum, &ch2)) {
					if (syncNum != FORCED_ORDERED_SYNC)
						k=GLOB_totalUsedBanks+2;
                                }
				}
                        }
                        if (k==GLOB_totalUsedBanks) {
                            for (k=0; k<(GLOB_totalUsedBanks-1); k++) {
                                for (ch1=k+1; ch1<GLOB_totalUsedBanks; ch1++) {
                                    if (!CSP_checkOrdering(GLOB_PendingCMDChannel, k, ifnathenmb[ch1][k]+1, chis[k], ch1, ifnathenmb[k][ch1]+1, chis[ch1]))
                                        print("Above problem occured when analyzing sync %lu between (ch:%lu, indx:%d, tag:%d) & (ch:%lu, indx:%d, tag:%d)\n",
                                                syncNum, k, chis[k], GLOB_PendingCMDChannel[k][chis[k]].Tag, ch1, chis[ch1], GLOB_PendingCMDChannel[ch1][chis[ch1]].Tag);
                                    ifnathenmb[ch1][k] = chis[k];
                                    ifnathenmb[k][ch1] = chis[ch1];
                                }
                                chis[k]++;
                            }
                            chis[k]++;
                            indexchgd = 1;
                        }
                    }
                }
            }
        }

        if (!alldone) {
            alldone = 1;
            for (i=0; alldone && (i<GLOB_totalUsedBanks); i++) {
                if (!EOLIST(i)) {
                    alldone = 0;
                }
            }
        }
    }

    for (i=0; i<GLOB_totalUsedBanks; i++) {
        for (k=i+1; k<GLOB_totalUsedBanks; k++) {
            if (!CSP_checkOrdering(GLOB_PendingCMDChannel, i, ifnathenmb[k][i]+1, chMaxIndexes[i]-1, k, ifnathenmb[i][k]+1, chMaxIndexes[k]-1))
                print("Above problem occured when doing end of list checks on channels %lu & %lu\n", i, k);
        }
    }
}
#endif  // DEBUG_SYNC
#endif  // CMD_DMA

// end of LLD_CDMA.c
