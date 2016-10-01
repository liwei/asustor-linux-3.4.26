/*
 * NAND Flash Controller Device Driver
 * Copyright (c) 2008-2011, Intel Corporation and its suppliers.
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


#include <linux/interrupt.h>
#include <linux/wait.h>
#include <linux/pci.h>
#include "lld.h"
#include "lld_nand.h"

#ifdef ELDORA
#include "defs.h"
#else
#include "spectraswconfig.h"
#include "flash.h"
#include "ffsdefs.h"
#endif

#if (FLASH_NAND  || FLASH_CDMA)
#include "NAND_Regs_4.h"

#define MAX_PAGES_PER_RW    128

#define SKIP_IN_SIMULATION     1

#define CEIL_DIV(X,Y) (((X)%(Y))?((X)/(Y)+1):((X)/(Y)))

uint32 GLOB_totalUsedBanks;
uint16 GLOB_init_firsttime_done = 1;
uint16 n_of_luns, blockperlun_l, blockperlun_h;
uint32 blockperlun;

uint16      InterruptEnableMask;
uint16      CMD_DMA_InterruptEnableMask;

uint32      GLOB_valid_banks[LLD_MAX_FLASH_BANKS];

#ifndef ELDORA
extern uint16  FIQ_FLAG;
extern byte     *g_pMemPoolFree;
#endif

volatile uint32* FlashReg;
volatile uint32* FlashMem;

byte* page_main_spare;

ACCESS_TYPE access_type = FS_ACCESS;//It defines the driver access type, default to be file system access
#define COMMAND_REGISTER 0
#define DATA_REGISTER 0x10
#define MAP11_COMMAND 0x3 << 26
#define COMMAND_CYCLE 0
#define ADDRESS_CYCLE 1
#define STATUS_CYCLE 2


uint16 conf_parameters[] = {
                               0x0000,
                               0x0000,
                               0x01F4,
                               0x01F4,
                               0x01F4,
                               0x01F4,
                               0x0000,
                               0x0000,
                               0x0001,
                               0x0000,
                               0x0000,
                               0x0000,
                               0x0000,
                               0x0040,
                               0x0001,
                               0x000A,
                               0x000A,
                               0x000A,
                               0x0000,
                               0x0000,
                               0x0005,
                               0x0012,
                               0x000C
};



#if DDMA
/*DDMA interrupt*/
int ddma_irq_number;
#define DDMA_IRQ_NUM (4)
#define IRQ_WAIT_TIME_OUT (10*HZ)
#define DDMA_IRQ_NAME "GLOB_Spectra_IRQ"
#define DDMA_INT_MASK (INTR_STATUS0__DMA_CMD_COMP | \
		INTR_STATUS0__ECC_TRANSACTION_DONE |\
		INTR_STATUS0__ECC_ERR | \
		INTR_STATUS0__PROGRAM_FAIL | \
		INTR_STATUS0__TIME_OUT)

//Device state for interrupt check
#define INT_IDLE_STATE                 0
#define INT_READ_PAGE_MAIN    0x01
#define INT_WRITE_PAGE_MAIN    0x02
#define INT_PIPELINE_READ_AHEAD    0x04
#define INT_PIPELINE_WRITE_AHEAD    0x08
#define INT_MULTI_PLANE_READ    0x10
#define INT_MULTI_PLANE_WRITE    0x11

static struct nand_irq_info info;
#define NAND_INTERRUPT_IS_ENABLED (FlashReg[GLOBAL_INT_ENABLE] & GLOBAL_INT_ENABLE__FLAG)
#define DISABLE_NAND_INTERRUPT_WHEN_ENABLED do{ \
										if(NAND_INTERRUPT_IS_ENABLED) \
										{ \
											GLOB_LLD_Enable_Disable_Interrupts(0);\
										} }while(0)
#endif


uint16   NAND_Get_Bad_Block(BLOCKNODE block)
{

    uint32 status = PASS;
    byte pReadSpareBuf[MAX_PAGE_SPARE_AREA];
    uint32 spareSkipBytes  = GLOB_DeviceInfo.wSpareSkipBytes;
    uint32 spareFlagBytes  = 0;
    uint32 page, i;

    if(FlashReg[ECC_ENABLE])
    {
        spareFlagBytes = GLOB_DeviceInfo.wNumPageSpareFlag;
    }
    if (access_type == GET_BAD_BLOCK_RAW)
		spareFlagBytes = 0;
    for (page = 0; page < 2; page++)
    {
        status = NAND_Read_Page_Spare(pReadSpareBuf, block, page, 1);
        if(PASS != status)
            return READ_ERROR;
        for (i=spareFlagBytes; i<(spareSkipBytes+spareFlagBytes); i++)
            if (pReadSpareBuf[i] != 0xff)
                return DEFECTIVE_BLOCK;
    }

    for (page = 1; page < 3; page++)
    {
        status = NAND_Read_Page_Spare(pReadSpareBuf, block, GLOB_DeviceInfo.wPagesPerBlock - page , 1);
        if(PASS != status)
            return READ_ERROR;
        for (i=spareFlagBytes; i<(spareSkipBytes+spareFlagBytes); i++)
            if (pReadSpareBuf[i] != 0xff)
                return DEFECTIVE_BLOCK;
    }

    return GOOD_BLOCK;

}

uint16   NAND_Get_Bad_Block_Raw(BLOCKNODE block)
{
	uint32 status = PASS;
	access_type = GET_BAD_BLOCK_RAW;
	status = NAND_Get_Bad_Block(block);
	access_type = FS_ACCESS;
	return status;
}

uint16    NAND_Flash_Reset (void)
{
    uint32  i;
    uint32 intr_status_rst_comp[4] = {INTR_STATUS0__RST_COMP, INTR_STATUS1__RST_COMP, INTR_STATUS2__RST_COMP, INTR_STATUS3__RST_COMP};
    uint32 intr_status_time_out[4] = {INTR_STATUS0__TIME_OUT, INTR_STATUS1__TIME_OUT, INTR_STATUS2__TIME_OUT, INTR_STATUS3__TIME_OUT};
    uint32 intr_status[4] = { INTR_STATUS0, INTR_STATUS1, INTR_STATUS2, INTR_STATUS3};
    uint32 device_reset_banks[4] = {DEVICE_RESET__BANK0, DEVICE_RESET__BANK1, DEVICE_RESET__BANK2, DEVICE_RESET__BANK3};

    for ( i =0 ; i < LLD_MAX_FLASH_BANKS; i++)
    {
        FlashReg[intr_status[i]] = intr_status_rst_comp[i] | intr_status_time_out[i] ;

        if(GLOB_valid_banks[i])
        {
            FlashReg[DEVICE_RESET] =    device_reset_banks[i];
            while (!(FlashReg[intr_status[i]] &
                (intr_status_rst_comp[i] |
               intr_status_time_out[i])))
                ;
            if(FlashReg[intr_status[i]] &
                intr_status_time_out[i])
            {
            }
        }
        FlashReg[intr_status[i]] = intr_status_rst_comp[i] | intr_status_time_out[i] ;
    }

    return PASS;
}


static void  NAND_ONFi_Timing_Mode (uint16 mode)
{
    uint16 Trea[6]  = {40,30,25,20,20,16};
    uint16 Trp[6]   = {50,25,17,15,12,10};
    uint16 Treh[6]  = {30,15,15,10,10,7};
    uint16 Trc[6]   = {100,50,35,30,25,20};
    uint16 Trhoh[6] = {0,15,15,15,15,15};
    uint16 Trloh[6] = {0,0,0,0,5,5};
    uint16 Tcea[6]  = {100,45,30,25,25,25};
    uint16 Tadl[6]  = {200,100,100,100,70,70};
    uint16 Trhw[6]  = {200,100,100,100,100,100};
    uint16 Trhz[6]  = {200,100,100,100,100,100};
    uint16 Twhr[6]  = {120,80,80,60,60,60};
    uint16 Tcs[6]   = {70,35,25,25,20,15};

    uint16 TclsRising = 1;
    uint16 data_invalid_rhoh, data_invalid_rloh, data_invalid;
    uint16 dv_window = 0;
    uint16 en_lo, en_hi;
    uint16 acc_clks;
    uint16 addr_2_data, re_2_we, re_2_re, we_2_re, cs_cnt;

    en_lo = CEIL_DIV(Trp[mode], CLK_X);
    en_hi = CEIL_DIV(Treh[mode], CLK_X);

#if ONFI_BLOOM_TIME
    if ((en_hi * CLK_X) < (Treh[mode] + 2))
    {
        en_hi++;
    }
#endif

    if ((en_lo + en_hi) * CLK_X < Trc[mode])
    {
        en_lo +=  CEIL_DIV( (Trc[mode] - (en_lo + en_hi)*  CLK_X), CLK_X );
    }

    if ((en_lo + en_hi) < CLK_MULTI)
    {
        en_lo += CLK_MULTI - en_lo - en_hi;
    }

    while (dv_window < 8)
    {
        data_invalid_rhoh = en_lo * CLK_X + Trhoh[mode];

        data_invalid_rloh = (en_lo + en_hi) * CLK_X + Trloh[mode];

        data_invalid = data_invalid_rhoh < data_invalid_rloh ?  data_invalid_rhoh : data_invalid_rloh;

        dv_window = data_invalid - Trea[mode];

        if (dv_window < 8)
        {
            en_lo++;
        }
    }

    acc_clks = CEIL_DIV(Trea[mode], CLK_X);

    while (((acc_clks * CLK_X)- Trea[mode]) < 3)
    {
        acc_clks++;
    };

    if ((data_invalid - acc_clks * CLK_X) < 2)
    {
    }

    addr_2_data = CEIL_DIV(Tadl[mode], CLK_X);
    re_2_we = CEIL_DIV(Trhw[mode], CLK_X);
    re_2_re = CEIL_DIV(Trhz[mode], CLK_X);
    we_2_re = CEIL_DIV(Twhr[mode], CLK_X);
    cs_cnt = CEIL_DIV((Tcs[mode] - Trp[mode]), CLK_X);
    if(!TclsRising)  { cs_cnt = CEIL_DIV(Tcs[mode], CLK_X);  } ;
    if(cs_cnt == 0)  {cs_cnt = 1 ;}

    if(Tcea[mode])
    {
        while ((( cs_cnt* CLK_X)+ Trea[mode]) < Tcea[mode]) { cs_cnt++;  }
    }

#if MODE5_WORKAROUND
    if (mode == 5)
        acc_clks = 5;
#endif


    if (  (  (FlashReg[MANUFACTURER_ID] == 0x2C)
          || (FlashReg[MANUFACTURER_ID] == 0)
          )
       && (FlashReg[DEVICE_ID] == 0x88)
       && (mode < 5)
       )
        acc_clks = 6;

    FlashReg[ACC_CLKS ] = acc_clks ;
    FlashReg[RE_2_WE ]  = re_2_we;
    FlashReg[RE_2_RE ]  = re_2_re;
    FlashReg[WE_2_RE ]  = we_2_re;
    FlashReg[ADDR_2_DATA ]  = addr_2_data;
    FlashReg[RDWR_EN_LO_CNT  ]  = en_lo;
    FlashReg[RDWR_EN_HI_CNT ]   = en_hi;
    FlashReg[CS_SETUP_CNT ]     = cs_cnt;
}


static void Index_Addr(volatile uint32* FBA_FPA_FSA_P, uint32 address, uint32 data)
{
    FBA_FPA_FSA_P = (volatile uint32*)(FlashMem);
    *FBA_FPA_FSA_P = address;

    FBA_FPA_FSA_P = (volatile uint32*)((uint32)FlashMem | 0x10);
    *FBA_FPA_FSA_P = data;
}

static void Index_Addr_Read_Data(volatile uint32* FBA_FPA_FSA_P, uint32 address, uint32* data)
{
    FBA_FPA_FSA_P = (volatile uint32*)(FlashMem);
    *FBA_FPA_FSA_P = address;

    FBA_FPA_FSA_P = (volatile uint32*)((uint32)FlashMem | 0x10);
    *data = *FBA_FPA_FSA_P;
}


uint16  NAND_Read_Device_ID (void)
{
    uint16  status = PASS;
    byte    mfg_code,dev_code;
    byte    no_of_planes;
    uint32   plane_size, block_size;

    volatile uint32 *FBA_FPA_FSA_P;

    uint32 i;
    uint32 map11_data[LLD_MAX_FLASH_BANKS];
    //Workaround for Hynix H27U1G8F2BTR
    volatile uint32 manu_id, device_id, dev_param0, dev_param1, dev_param2 = 0;
    uint32  known_non_compliant_onfi_part = 0;

    FlashReg[SPARE_AREA_SKIP_BYTES]        = 0x0002;
    FlashReg[SPARE_AREA_MARKER]            = 0xffff;
	// Determin whether there's NAND flash chip on board
    if ((FlashReg[MANUFACTURER_ID] == 0) && (FlashReg[DEVICE_ID] == 0))
    {
		print("No NAND flash on board!!\n");
		status = FAIL;
		return status;
    }

    if (((FlashReg[MANUFACTURER_ID] == 0)&&(FlashReg[DEVICE_ID] == 0xF1))||((FlashReg[MANUFACTURER_ID] == 0xAD)&&(FlashReg[DEVICE_ID] == 0xF1)))
    {
        printk("detected no manufacture id, read from NAND part\n");

        //Need to detect the part and read in the Electronic Signature
        FlashMem[COMMAND_REGISTER] = MAP11_COMMAND | COMMAND_CYCLE;
        FlashMem[DATA_REGISTER] = 0x90;

        FlashMem[COMMAND_REGISTER] = MAP11_COMMAND | ADDRESS_CYCLE;
        FlashMem[DATA_REGISTER] = 0x0;

        FlashMem[COMMAND_REGISTER] = MAP11_COMMAND | STATUS_CYCLE;
        manu_id = FlashMem[DATA_REGISTER] & 0xff;

        FlashMem[COMMAND_REGISTER] = MAP11_COMMAND | STATUS_CYCLE;
        device_id = FlashMem[DATA_REGISTER] & 0xff;

        FlashMem[COMMAND_REGISTER] = MAP11_COMMAND | STATUS_CYCLE;
        dev_param0 = FlashMem[DATA_REGISTER];

        FlashMem[COMMAND_REGISTER] = MAP11_COMMAND | STATUS_CYCLE;
        dev_param1 = FlashMem[DATA_REGISTER];

        FlashMem[COMMAND_REGISTER] = MAP11_COMMAND | STATUS_CYCLE;
        dev_param2 = FlashMem[DATA_REGISTER];

        //check if part is the Hynix H27U1G8F2BTR
        if ((manu_id == 0xAD) && (device_id == 0xF1))
        {
            //Values Hard Coded to those in the Hynix data sheet
            FlashReg[MANUFACTURER_ID] = 0xAD;
            FlashReg[DEVICE_ID] = 0xF1;
            FlashReg[DEVICE_PARAM_0] = 0x00;
            FlashReg[DEVICE_PARAM_1] = 0x1D;
            FlashReg[DEVICE_PARAM_2] = 0xAD;
            FlashReg[LOGICAL_PAGE_DATA_SIZE] = 2048;
            FlashReg[LOGICAL_PAGE_SPARE_SIZE] = 64;
            FlashReg[NUMBER_OF_PLANES] = 0x00; //0 is value for monoplane device
            FlashReg[PAGES_PER_BLOCK] = 64; //128kB block with 2 kB pages
            FlashReg[DEVICE_WIDTH] = 0x00; //0 is for 8-bit
            FlashReg[DEVICE_MAIN_AREA_SIZE] = 2048;
            FlashReg[DEVICE_SPARE_AREA_SIZE] = 64;
            FlashReg[DEVICES_CONNECTED] = 1;
            FlashReg[ONFI_DEVICE_FEATURES] = 0x00;
            FlashReg[ONFI_OPTIONAL_COMMANDS] = 0x00;
            FlashReg[ONFI_TIMING_MODE] = 0x00;
            FlashReg[ONFI_PGM_CACHE_TIMING_MODE] = 0x00;

            printk("detected Hynix H27U1G8F2BTR\n");
            known_non_compliant_onfi_part = 1;
        }
    }

    GLOB_DeviceInfo.wDeviceMaker         = FlashReg[MANUFACTURER_ID];
    GLOB_DeviceInfo.wDeviceType          = (((FlashReg[DEVICE_WIDTH]>>2)>0)?16:8);
    GLOB_DeviceInfo.wPagesPerBlock       = FlashReg[PAGES_PER_BLOCK];
    GLOB_DeviceInfo.wPageDataSize        = FlashReg[LOGICAL_PAGE_DATA_SIZE];
    GLOB_DeviceInfo.wPageSpareSize       = FlashReg[LOGICAL_PAGE_SPARE_SIZE];
    GLOB_DeviceInfo.wPageSize            = GLOB_DeviceInfo.wPageDataSize+GLOB_DeviceInfo.wPageSpareSize;
    GLOB_DeviceInfo.wBlockSize           = GLOB_DeviceInfo.wPageSize*GLOB_DeviceInfo.wPagesPerBlock;
    GLOB_DeviceInfo.wBlockDataSize       = GLOB_DeviceInfo.wPagesPerBlock*GLOB_DeviceInfo.wPageDataSize;
    GLOB_DeviceInfo.wHWRevision          = FlashReg[REVISION];

    GLOB_DeviceInfo.wDeviceMainAreaSize  = FlashReg[DEVICE_MAIN_AREA_SIZE];
    GLOB_DeviceInfo.wDeviceSpareAreaSize = FlashReg[DEVICE_SPARE_AREA_SIZE];
    GLOB_DeviceInfo.wDeviceWidth         = FlashReg[DEVICE_WIDTH];

    GLOB_DeviceInfo.wDevicesConnected    = FlashReg[DEVICES_CONNECTED];

    GLOB_DeviceInfo.wHWFeatures          = FlashReg[FEATURES];

    GLOB_DeviceInfo.MLCDevice           = FlashReg[DEVICE_PARAM_0] &  0x000c;
    GLOB_DeviceInfo.wSpareSkipBytes     = FlashReg[SPARE_AREA_SKIP_BYTES] * GLOB_DeviceInfo.wDevicesConnected;

    GLOB_DeviceInfo.nBitsInPageNumber = (byte)GLOB_Calc_Used_Bits(GLOB_DeviceInfo.wPagesPerBlock);
    GLOB_DeviceInfo.nBitsInPageDataSize = (byte)GLOB_Calc_Used_Bits(GLOB_DeviceInfo.wPageDataSize);
    GLOB_DeviceInfo.nBitsInBlockDataSize = (byte)GLOB_Calc_Used_Bits(GLOB_DeviceInfo.wBlockDataSize);

    if((FlashReg[DEVICE_MAIN_AREA_SIZE] < 4096) || (FlashReg[DEVICE_SPARE_AREA_SIZE] <= 128))
    {
#if SUPPORT_8BITECC
        FlashReg[ECC_CORRECTION]  = 8;
#endif
    }

    if((FlashReg[MANUFACTURER_ID] == 0x98) && (FlashReg[DEVICE_MAIN_AREA_SIZE]
            == 4096) &&  (FlashReg[DEVICE_SPARE_AREA_SIZE] == 64))
    {
        FlashReg[DEVICE_SPARE_AREA_SIZE] = 216;

        GLOB_DeviceInfo.wDeviceSpareAreaSize = FlashReg[DEVICE_SPARE_AREA_SIZE];

        GLOB_DeviceInfo.wPageSpareSize = FlashReg[LOGICAL_PAGE_SPARE_SIZE];

#if SUPPORT_15BITECC
        FlashReg[ECC_CORRECTION]  = 15;
#elif SUPPORT_8BITECC
        FlashReg[ECC_CORRECTION]  = 8;
#endif
    }



    mfg_code = GLOB_DeviceInfo.wDeviceMaker;
    dev_code = FlashReg[DEVICE_ID] & DEVICE_ID__VALUE;
    // We promote the non-ONFI device with ONFI timing 1
    NAND_ONFi_Timing_Mode (1);

    if ( (mfg_code == 0xAD )  && ((dev_code == 0xD7) || (dev_code == 0xD5)) )
    {
        FlashReg[PAGES_PER_BLOCK] = 128;
        FlashReg[DEVICE_MAIN_AREA_SIZE] = 4096;
        FlashReg[DEVICE_SPARE_AREA_SIZE] = 224;
        FlashReg[DEVICE_WIDTH] = 0;
#if SUPPORT_15BITECC
        FlashReg[ECC_CORRECTION]  = 15;
#elif SUPPORT_8BITECC
        FlashReg[ECC_CORRECTION]  = 8;
#endif

        GLOB_DeviceInfo.MLCDevice  = 1;

        GLOB_DeviceInfo.wDeviceMainAreaSize   = FlashReg[DEVICE_MAIN_AREA_SIZE];
        GLOB_DeviceInfo.wDeviceSpareAreaSize  = FlashReg[DEVICE_SPARE_AREA_SIZE];
        GLOB_DeviceInfo.wDeviceWidth         = FlashReg[DEVICE_WIDTH];
        GLOB_DeviceInfo.wPagesPerBlock        = FlashReg[PAGES_PER_BLOCK];

        GLOB_DeviceInfo.wPageDataSize         = FlashReg[LOGICAL_PAGE_DATA_SIZE];
        GLOB_DeviceInfo.wPageSpareSize        = FlashReg[LOGICAL_PAGE_SPARE_SIZE];
        GLOB_DeviceInfo.wPageSize             = GLOB_DeviceInfo.wPageDataSize +
                                                    GLOB_DeviceInfo.wPageSpareSize;

        GLOB_DeviceInfo.wBlockSize            = GLOB_DeviceInfo.wPageSize *
                                                    GLOB_DeviceInfo.wPagesPerBlock;
        GLOB_DeviceInfo.wBlockDataSize        = GLOB_DeviceInfo.wPagesPerBlock*
                                                    GLOB_DeviceInfo.wPageDataSize;

        GLOB_DeviceInfo.wSpareSkipBytes       = FlashReg[SPARE_AREA_SKIP_BYTES]*
                                                GLOB_DeviceInfo.wDevicesConnected;


        GLOB_DeviceInfo.nBitsInPageNumber = (byte)GLOB_Calc_Used_Bits(GLOB_DeviceInfo.wPagesPerBlock);
        GLOB_DeviceInfo.nBitsInPageDataSize = (byte)GLOB_Calc_Used_Bits(GLOB_DeviceInfo.wPageDataSize);
        GLOB_DeviceInfo.nBitsInBlockDataSize = (byte)GLOB_Calc_Used_Bits(GLOB_DeviceInfo.wBlockDataSize);

    }


    if ( (FlashReg[ECC_CORRECTION] & ECC_CORRECTION__VALUE  ) == 1)
    {
        GLOB_DeviceInfo.wECCBytesPerSector = 4;

        GLOB_DeviceInfo.wECCBytesPerSector   *=  GLOB_DeviceInfo.wDevicesConnected;

        GLOB_DeviceInfo.wNumPageSpareFlag    =   GLOB_DeviceInfo.wPageSpareSize - GLOB_DeviceInfo.wPageDataSize / (ECC_SECTOR_SIZE *
            (GLOB_DeviceInfo.wDevicesConnected)) *(GLOB_DeviceInfo.wECCBytesPerSector)  -  GLOB_DeviceInfo.wSpareSkipBytes;
    }
    else
    {
        GLOB_DeviceInfo.wECCBytesPerSector =   (uint32) ((FlashReg[ECC_CORRECTION] & ECC_CORRECTION__VALUE  ) * 13) / 8;

        if ( ( GLOB_DeviceInfo.wECCBytesPerSector) % 2 == 0)
            GLOB_DeviceInfo.wECCBytesPerSector += 2;
        else
            GLOB_DeviceInfo.wECCBytesPerSector += 1;

        GLOB_DeviceInfo.wECCBytesPerSector   *=  GLOB_DeviceInfo.wDevicesConnected;

        GLOB_DeviceInfo.wNumPageSpareFlag    =   GLOB_DeviceInfo.wPageSpareSize - GLOB_DeviceInfo.wPageDataSize / (ECC_SECTOR_SIZE *
            (GLOB_DeviceInfo.wDevicesConnected)) *(GLOB_DeviceInfo.wECCBytesPerSector) - GLOB_DeviceInfo.wSpareSkipBytes;
    }

	if ((!known_non_compliant_onfi_part) && (FlashReg[ONFI_DEVICE_NO_OF_LUNS] & ONFI_DEVICE_NO_OF_LUNS__ONFI_DEVICE))
    {
        FlashReg[DEVICE_RESET] =    DEVICE_RESET__BANK0;
        while (((FlashReg[INTR_STATUS0] & INTR_STATUS0__RST_COMP )|
                (FlashReg[INTR_STATUS0] &  INTR_STATUS0__TIME_OUT)) == 0  );
        if (FlashReg[INTR_STATUS0] & INTR_STATUS0__RST_COMP )
        {
	        FlashReg[DEVICE_RESET] =    DEVICE_RESET__BANK1;
            while (((FlashReg[INTR_STATUS1] & INTR_STATUS1__RST_COMP )|
                    (FlashReg[INTR_STATUS1] &  INTR_STATUS1__TIME_OUT)) == 0);
            if (FlashReg[INTR_STATUS1] & INTR_STATUS0__RST_COMP )
            {
                FlashReg[DEVICE_RESET] =    DEVICE_RESET__BANK2;
                while (((FlashReg[INTR_STATUS2] & INTR_STATUS2__RST_COMP )|
                        (FlashReg[INTR_STATUS2] &  INTR_STATUS2__TIME_OUT)) == 0);
                if (FlashReg[INTR_STATUS2] & INTR_STATUS0__RST_COMP )
                {
                    FlashReg[DEVICE_RESET] =    DEVICE_RESET__BANK3;
                    while (((FlashReg[INTR_STATUS3] & INTR_STATUS3__RST_COMP )|
                            (FlashReg[INTR_STATUS3] &  INTR_STATUS3__TIME_OUT)) == 0);
                }
                else
                {
                    print("getting a time out for bank 2!!");

                }
            }
            else
            {
                print("getting a time out for bank 2!!");
            }
        }

        FlashReg[INTR_STATUS0] = INTR_STATUS0__TIME_OUT ;
        FlashReg[INTR_STATUS1] = INTR_STATUS1__TIME_OUT ;
        FlashReg[INTR_STATUS2] = INTR_STATUS2__TIME_OUT ;
        FlashReg[INTR_STATUS3] = INTR_STATUS3__TIME_OUT ;


        GLOB_DeviceInfo.wONFIDevFeatures         = FlashReg[ONFI_DEVICE_FEATURES];
        GLOB_DeviceInfo.wONFIOptCommands         = FlashReg[ONFI_OPTIONAL_COMMANDS];
        GLOB_DeviceInfo.wONFITimingMode          = FlashReg[ONFI_TIMING_MODE];
        GLOB_DeviceInfo.wONFIPgmCacheTimingMode  = FlashReg[ONFI_PGM_CACHE_TIMING_MODE];

        n_of_luns      =  FlashReg[ONFI_DEVICE_NO_OF_LUNS]& ONFI_DEVICE_NO_OF_LUNS__NO_OF_LUNS;
        blockperlun_l  =  FlashReg[ONFI_DEVICE_NO_OF_BLOCKS_PER_LUN_L];
        blockperlun_h  =  FlashReg[ONFI_DEVICE_NO_OF_BLOCKS_PER_LUN_U];

        blockperlun = (blockperlun_h << 16) | blockperlun_l;

        GLOB_DeviceInfo.wTotalBlocks = n_of_luns *  blockperlun;

        if ((FlashReg[ONFI_TIMING_MODE] & ONFI_TIMING_MODE__VALUE) == 0)
        {
            status = FAIL;
            return status;
        }


	    for(i = 1; i > 0; i--)
        {
            if (FlashReg[ONFI_TIMING_MODE]& (0x01 << i)  )
                break;
        }

        NAND_ONFi_Timing_Mode (i);

	    if(GLOB_DeviceInfo.wHWFeatures & FEATURES__INDEX_ADDR)
        {
            Index_Addr(FBA_FPA_FSA_P, (uint32)(MODE_11 | 0), 0x90);
            Index_Addr(FBA_FPA_FSA_P, (uint32)(MODE_11 | 1), 0);

            for (i = 0; i < 3; i++)
                Index_Addr_Read_Data(FBA_FPA_FSA_P, (uint32)(MODE_11 | 2), map11_data);
        }
        else
        {
            FBA_FPA_FSA_P = (volatile uint32 *)(MODE_11 | 0);
            *FBA_FPA_FSA_P = 0x90;

            FBA_FPA_FSA_P = (volatile uint32 *)(MODE_11 | 1);
            *FBA_FPA_FSA_P = 0x00;

            for (i = 0; i < 3; i++)
            {
                FBA_FPA_FSA_P = (volatile uint32 *)(MODE_11 | 2);
                map11_data[0] = *FBA_FPA_FSA_P;
            }
        }

        print("3rd ID: %x \n" , map11_data[0]);
        GLOB_DeviceInfo.MLCDevice           = map11_data[0] &  0x000c;
    }
    else if (mfg_code == 0xEC )
    {

	dev_code = FlashReg[DEVICE_ID] & DEVICE_ID__VALUE;
        if ((0xD3 == dev_code) && (0x40 == FlashReg[DEVICE_SPARE_AREA_SIZE]) && (0x51 == FlashReg[DEVICE_PARAM_0]))
        {
                        // Hardcode block number for part K9K8G08U0B and K9WAG08U1B(2 banks)
                        GLOB_DeviceInfo.wTotalBlocks = 8192;
        }
        else if ((0xD3 == dev_code) && (0x80 == FlashReg[DEVICE_SPARE_AREA_SIZE]))
        {
                        // Hardcode block number for part k918g08u0m
                        GLOB_DeviceInfo.wTotalBlocks = 4096;
        }
	else if (0xD5 == dev_code)
	{
		// Hardcode block number for part K9GAG08U0D
		GLOB_DeviceInfo.wTotalBlocks = 4096;
	}
	else if (0xD7 == dev_code)
	{
		// Hardcode block number for part K9LBG08U0D
		GLOB_DeviceInfo.wTotalBlocks = 8192;
	}
	else
	{

        if(GLOB_DeviceInfo.wHWFeatures & FEATURES__INDEX_ADDR)
        {
            Index_Addr(FBA_FPA_FSA_P, (uint32)(MODE_11 | 0), 0x90);
            Index_Addr(FBA_FPA_FSA_P, (uint32)(MODE_11 | 1), 0);

            for (i = 0; i < 5; i++)
                Index_Addr_Read_Data(FBA_FPA_FSA_P, (uint32)(MODE_11 | 2), map11_data);
        }
        else
        {
            FBA_FPA_FSA_P = (volatile uint32 *)(MODE_11 | 0);
            *FBA_FPA_FSA_P = 0x90;

            FBA_FPA_FSA_P = (volatile uint32 *)(MODE_11 | 1);
            *FBA_FPA_FSA_P = 0x00;

            for (i = 0; i < 5; i++)
            {
                FBA_FPA_FSA_P = (volatile uint32 *)(MODE_11 | 2);
                map11_data[0] = *FBA_FPA_FSA_P;
            }
        }

        print("5th ID: %x \n" , map11_data[0]);

        no_of_planes = 1 << ((map11_data[0] & 0x0c) >> 2) ;
        print("  no_of_planes_per_ce: %d    \n", no_of_planes);

	plane_size   = 64 << (((map11_data[0] & 0x70 ) >> 4)) ;
        print("  plane_size in Mb : %d    \n", plane_size);



        block_size   = 64 << ((FlashReg[DEVICE_PARAM_1] &  0x30) >> 4);


        GLOB_DeviceInfo.wTotalBlocks = 128 * (plane_size * no_of_planes )/block_size;
        print("  Total Nubmer of Blocks: %d    \n", GLOB_DeviceInfo.wTotalBlocks);
		}
    }
	else if (mfg_code == 0x20 )
    {//For ST Micro, get total blocks by device ID
        dev_code = FlashReg[DEVICE_ID]& DEVICE_ID__VALUE;
		switch(dev_code)
		{
			case 0xA1://NAND01GR3B2B
			case 0xF1://NAND01GW3B2B
			case 0xB1://NAND01GR4B2B
			case 0xC1://NAND01GW4B2B
				 GLOB_DeviceInfo.wTotalBlocks = 1024;
				 break;
			case 0xAA://NAND02GR3B2C
			case 0xDA://NAND02GW3B2C
			case 0xBA://NAND02GR4B2C
			case 0xCA://NAND02GW4B2C
				 GLOB_DeviceInfo.wTotalBlocks = 2048;
				 break;
			default:
				GLOB_DeviceInfo.wTotalBlocks = GLOB_HWCTL_DEFAULT_BLKS;
		}
	}
	else if (mfg_code == 0x98 )
	    {//For Toshiba, get total blocks by device ID
	        dev_code = FlashReg[DEVICE_ID]& DEVICE_ID__VALUE;
               switch(dev_code)
               {
                       case 0xD1://TC58NVG0S3ETA00
                                GLOB_DeviceInfo.wTotalBlocks = 1024;
                                break;
                       case 0xF0://TC58NVM9S3ETA00
                                GLOB_DeviceInfo.wTotalBlocks = 512;
                                break;
                       case 0xDA://TC58NVG1S3ETA00
                                GLOB_DeviceInfo.wTotalBlocks = 2048;
                                break;
                       default:
                                GLOB_DeviceInfo.wTotalBlocks = GLOB_HWCTL_DEFAULT_BLKS;
               }
	}

    else if (mfg_code == 0xAD )
    {//For Hynix workround, get total blocks by device ID
	dev_code = FlashReg[DEVICE_ID]& DEVICE_ID__VALUE;
	switch(dev_code)
        {
           case 0xF1://H27U1G8F2BTR
			GLOB_DeviceInfo.wTotalBlocks = 1024;
			print("detected Hynix H27U1G8F2BTR block number is %d\n",GLOB_DeviceInfo.wTotalBlocks);
                break;
           case 0xDC://HY27UT084G2A
                        GLOB_DeviceInfo.wTotalBlocks = 2048;
                break;
		default:
		GLOB_DeviceInfo.wTotalBlocks = GLOB_HWCTL_DEFAULT_BLKS;
                break;
	}
    }
#if GLOB_DEVTSBA_ALT_BLK_NFO
    else
    {
        byte *tsba_ptr = (byte *)GLOB_DEVTSBA_ALT_BLK_ADD;
        GLOB_DeviceInfo.wTotalBlocks = (1 << *tsba_ptr);
        if (GLOB_DeviceInfo.wTotalBlocks < 512)
            GLOB_DeviceInfo.wTotalBlocks = GLOB_HWCTL_DEFAULT_BLKS;
    }
#else
    else
    {
        GLOB_DeviceInfo.wTotalBlocks = GLOB_HWCTL_DEFAULT_BLKS;
    }
#endif
    GLOB_DeviceInfo.nBitsInPageNumber = (byte)GLOB_Calc_Used_Bits(GLOB_DeviceInfo.wPagesPerBlock);
    GLOB_DeviceInfo.nBitsInPageDataSize = (byte)GLOB_Calc_Used_Bits(GLOB_DeviceInfo.wPageDataSize);
    GLOB_DeviceInfo.nBitsInBlockDataSize = (byte)GLOB_Calc_Used_Bits(GLOB_DeviceInfo.wBlockDataSize);

	no_of_planes = FlashReg[NUMBER_OF_PLANES]& NUMBER_OF_PLANES__VALUE;
    switch(no_of_planes)
    {
    case 0:
    case 1:
    case 3:
    case 7:
        GLOB_DeviceInfo.bPlaneNum = no_of_planes + 1;
        break;
    default:
        status = FAIL;
        break;
    }

    GLOB_totalUsedBanks = 0;
    for ( i =0 ; i < LLD_MAX_FLASH_BANKS; i++)
    {
        if(GLOB_DeviceInfo.wHWFeatures & FEATURES__INDEX_ADDR)
        {
            Index_Addr(FBA_FPA_FSA_P, (uint32)(MODE_11 |(i << 24)| 0), 0x90);

            Index_Addr(FBA_FPA_FSA_P, (uint32)(MODE_11 |(i << 24)| 1), 0);

            Index_Addr_Read_Data(FBA_FPA_FSA_P, (uint32)(MODE_11 |(i << 24)| 2), &map11_data[i]);
        }
        else
        {
            FBA_FPA_FSA_P = (volatile uint32 *)(MODE_11 |(i << 24)| 0);
            *FBA_FPA_FSA_P = 0x90;

            FBA_FPA_FSA_P = (volatile uint32 *)(MODE_11 |(i << 24)| 1);
            *FBA_FPA_FSA_P = 0x00;

            FBA_FPA_FSA_P = (volatile uint32 *)(MODE_11 |(i << 24)| 2);
            map11_data[i] = *FBA_FPA_FSA_P;
        }
        print("1st ID: %x for bank %d \n" , map11_data[i], i);

        if (i != 0)
        {
            if (  (map11_data[0] & 0x000000FF)  == (map11_data[i] & 0x000000FF) )
                GLOB_valid_banks[i] = 1;

        }
        else
        {
            if (  (map11_data[0] & 0x000000FF)  != 0x0 )
                GLOB_valid_banks[i] = 1;

        }
        GLOB_totalUsedBanks += GLOB_valid_banks[i];
    }

    if(FlashReg[FEATURES] & FEATURES__PARTITION)
    {
#if (SEAMLESS & TEST_LLD) || (SEAMLESS &  ELDORA) || (SEAMLESS & TEST_FTL)
        if ((FlashReg[PERM_SRC_ID_0] & PERM_SRC_ID_0__SRCID) == SPECTRA_PARTITION_ID)
        {
            GLOB_DeviceInfo.wSpectraStartBlock    = ( (FlashReg[MIN_MAX_BANK_0]& MIN_MAX_BANK_0__MIN_VALUE) * GLOB_DeviceInfo.wTotalBlocks )
                +  (FlashReg[MIN_BLK_ADDR_0] & MIN_BLK_ADDR_0__VALUE);

            GLOB_DeviceInfo.wSpectraEndBlock     = ( ( (FlashReg[MIN_MAX_BANK_0]& MIN_MAX_BANK_0__MAX_VALUE) >>  2) * GLOB_DeviceInfo.wTotalBlocks )
                +  (FlashReg[MAX_BLK_ADDR_0] & MAX_BLK_ADDR_0__VALUE);

            GLOB_DeviceInfo.wTotalBlocks =  GLOB_DeviceInfo.wTotalBlocks * GLOB_totalUsedBanks;

            if(GLOB_DeviceInfo.wSpectraEndBlock >= GLOB_DeviceInfo.wTotalBlocks)
            {

                GLOB_DeviceInfo.wSpectraEndBlock  = GLOB_DeviceInfo.wTotalBlocks - 1;
            }

            GLOB_DeviceInfo.wDataBlockNum =(BLOCKNODE)(GLOB_DeviceInfo.wSpectraEndBlock - GLOB_DeviceInfo.wSpectraStartBlock + 1);
        }
#else
        if ((FlashReg[PERM_SRC_ID_1] & PERM_SRC_ID_1__SRCID) == SPECTRA_PARTITION_ID)
        {
            GLOB_DeviceInfo.wSpectraStartBlock    = ( (FlashReg[MIN_MAX_BANK_1]& MIN_MAX_BANK_1__MIN_VALUE) * GLOB_DeviceInfo.wTotalBlocks )
                +  (FlashReg[MIN_BLK_ADDR_1] & MIN_BLK_ADDR_1__VALUE);

            GLOB_DeviceInfo.wSpectraEndBlock     = ( ( (FlashReg[MIN_MAX_BANK_1]& MIN_MAX_BANK_1__MAX_VALUE) >>  2) * GLOB_DeviceInfo.wTotalBlocks )
                +  (FlashReg[MAX_BLK_ADDR_1] & MAX_BLK_ADDR_1__VALUE);

            GLOB_DeviceInfo.wTotalBlocks =  GLOB_DeviceInfo.wTotalBlocks * GLOB_totalUsedBanks;


            if(GLOB_DeviceInfo.wSpectraEndBlock >= GLOB_DeviceInfo.wTotalBlocks)
            {

                GLOB_DeviceInfo.wSpectraEndBlock  = GLOB_DeviceInfo.wTotalBlocks - 1;
            }

            GLOB_DeviceInfo.wDataBlockNum =(BLOCKNODE)(GLOB_DeviceInfo.wSpectraEndBlock - GLOB_DeviceInfo.wSpectraStartBlock + 1);
        }
#endif
        else
        {
            GLOB_DeviceInfo.wTotalBlocks =  GLOB_DeviceInfo.wTotalBlocks * GLOB_totalUsedBanks;

            GLOB_DeviceInfo.wSpectraStartBlock   =   SPECTRA_START_BLOCK;
            GLOB_DeviceInfo.wSpectraEndBlock     =   GLOB_DeviceInfo.wTotalBlocks - 1;

            GLOB_DeviceInfo.wDataBlockNum =(BLOCKNODE)(GLOB_DeviceInfo.wSpectraEndBlock - GLOB_DeviceInfo.wSpectraStartBlock + 1);
        }
    }
    else
    {
        GLOB_DeviceInfo.wTotalBlocks =  GLOB_DeviceInfo.wTotalBlocks * GLOB_totalUsedBanks;

        GLOB_DeviceInfo.wSpectraStartBlock   =   SPECTRA_START_BLOCK;
        GLOB_DeviceInfo.wSpectraEndBlock     =   GLOB_DeviceInfo.wTotalBlocks - 1;

        GLOB_DeviceInfo.wDataBlockNum =(BLOCKNODE)(GLOB_DeviceInfo.wSpectraEndBlock - GLOB_DeviceInfo.wSpectraStartBlock  + 1);
    }
	//Initialize raw access data structures
    memset(&RAW_DeviceInfo,0,sizeof(RAW_DeviceInfo));
	GLOB_DeviceInfo.wDeviceType = dev_code;
	RAW_DeviceInfo = GLOB_DeviceInfo;
	RAW_DeviceInfo.wSpectraStartBlock = 0;
	#ifdef SPECTRA_RAW_END_BLOCK
	RAW_DeviceInfo.wSpectraEndBlock = SPECTRA_RAW_END_BLOCK;
	#else
	RAW_DeviceInfo.wSpectraEndBlock = GLOB_DeviceInfo.wTotalBlocks - 1;
	#endif
	RAW_DeviceInfo.wDataBlockNum = (BLOCKNODE)(RAW_DeviceInfo.wSpectraEndBlock - RAW_DeviceInfo.wSpectraStartBlock + 1);
	print("RAW_DeviceInfo.wSpectraStartBlock %d,RAW_DeviceInfo.wSpectraEndBlock %d, Page size %d KB, Block size %dKB\n",RAW_DeviceInfo.wSpectraStartBlock,RAW_DeviceInfo.wSpectraEndBlock,RAW_DeviceInfo.wPageDataSize>>10,RAW_DeviceInfo.wBlockDataSize>>10);
    return status;
}




uint16  NAND_UnlockArrayAll (void)
{
   uint32 status = PASS;
   volatile uint32 *FBA_FPA_FSA_P;
   uint32 unlock_start_address = 0;
   uint32 unlock_end_address = GLOB_DeviceInfo.wBlockSize*(GLOB_DeviceInfo.wTotalBlocks-1);

   if(GLOB_DeviceInfo.wHWFeatures & FEATURES__INDEX_ADDR)
   {
       Index_Addr(FBA_FPA_FSA_P, (uint32)(MODE_10 | unlock_start_address), 0x10);

       Index_Addr(FBA_FPA_FSA_P, (uint32)(MODE_10 | unlock_end_address), 0x11);
   }
   else
   {
       FBA_FPA_FSA_P = (volatile uint32*)((uint32)FlashMem | MODE_10 |unlock_start_address);
       *FBA_FPA_FSA_P = 0x10;

       FBA_FPA_FSA_P = (volatile uint32*)((uint32)FlashMem | MODE_10 |unlock_end_address);
       *FBA_FPA_FSA_P = 0x11;
   }

   return status;
}





void  NAND_LLD_Enable_Disable_Interrupts(uint16 INT_ENABLE)
{
    if (INT_ENABLE)
        FlashReg[GLOBAL_INT_ENABLE]     =   GLOBAL_INT_ENABLE__FLAG;
    else
        FlashReg[GLOBAL_INT_ENABLE]     =   0x0000;
}



uint16  NAND_Erase_Block(BLOCKNODE block)
{
    uint16  status = PASS;
    volatile uint32 *FBA_FPA_FSA_P;
    ADDRESSTYPE flash_add;
    uint16 flash_bank;
    uint32 intr_status=0;
    uint32 intr_status_addresses[4] = {INTR_STATUS0, INTR_STATUS1, INTR_STATUS2, INTR_STATUS3};

    flash_add = ( (ADDRESSTYPE)(block % ( (GLOB_DeviceInfo.wTotalBlocks) / GLOB_totalUsedBanks) ))*GLOB_DeviceInfo.wBlockDataSize;

    flash_bank = (block) / (GLOB_DeviceInfo.wTotalBlocks /
                GLOB_totalUsedBanks);

    if(block >= GLOB_DeviceInfo.wTotalBlocks)
    {
        status = FAIL;
    }
    if (status==PASS)
    {
        intr_status = intr_status_addresses[flash_bank];


        FlashReg[intr_status] = (INTR_STATUS0__ERASE_COMP |INTR_STATUS0__ERASE_FAIL |INTR_STATUS0__TIME_OUT );

        if(GLOB_DeviceInfo.wHWFeatures & FEATURES__INDEX_ADDR)
        {
            Index_Addr(FBA_FPA_FSA_P, (uint32 )(MODE_10 | (flash_bank << 24) |
                    ( ((flash_add) >> GLOB_DeviceInfo.nBitsInPageDataSize ) ) ), 0x01);
        }
        else
        {
            FBA_FPA_FSA_P = (volatile uint32 *)(uint32)((uint32)FlashMem | MODE_10 | (flash_bank << 24) |
                    ( ( (flash_add) >> GLOB_DeviceInfo.nBitsInPageDataSize ) << 2 ) );
            *FBA_FPA_FSA_P = 0x01;
        }

        while ((FlashReg[intr_status] & (INTR_STATUS0__ERASE_COMP |INTR_STATUS0__ERASE_FAIL|INTR_STATUS0__TIME_OUT)) == 0);

        if (FlashReg[intr_status] & (INTR_STATUS0__ERASE_FAIL|INTR_STATUS0__TIME_OUT))
            status = FAIL;

        FlashReg[intr_status] = (INTR_STATUS0__ERASE_COMP |INTR_STATUS0__ERASE_FAIL|INTR_STATUS0__TIME_OUT);
    }

    return status;
}

static uint32 Boundary_Check_Block_Page(BLOCKNODE block,uint16 page,uint16 page_count)
{
    uint32 status = PASS;
    if(block >= GLOB_DeviceInfo.wTotalBlocks)
    {
        status = FAIL;
    }

    if(page+page_count > GLOB_DeviceInfo.wPagesPerBlock)
    {
        status = FAIL;
    }
    return status;
}



uint16  NAND_Read_Page_Spare(byte* read_data,BLOCKNODE block,uint16 page,uint16 page_count)
{
    uint32 status = PASS;
    uint32 i;
    volatile uint32 *FBA_FPA_FSA_P;
    ADDRESSTYPE flash_add;
    byte page_spare[MAX_PAGE_SPARE_AREA];
    uint32 PageSpareSize     =     GLOB_DeviceInfo.wPageSpareSize;
    uint32 spareFlagBytes  = GLOB_DeviceInfo.wNumPageSpareFlag;


    uint32  flash_bank;
    uint32  intr_status=0;
    uint32 intr_status_addresses[4] = {INTR_STATUS0, INTR_STATUS1, INTR_STATUS2, INTR_STATUS3};

    status = Boundary_Check_Block_Page(block, page, page_count);

    if(page_count > 1)
    {
        status = FAIL;
    }

    flash_add = ( (ADDRESSTYPE)(block % ( (GLOB_DeviceInfo.wTotalBlocks) / GLOB_totalUsedBanks) ))*
                GLOB_DeviceInfo.wBlockDataSize+page*GLOB_DeviceInfo.wPageDataSize;

    flash_bank = (block) / (GLOB_DeviceInfo.wTotalBlocks /
                GLOB_totalUsedBanks);

    if(status == PASS)
    {
        intr_status = intr_status_addresses[flash_bank];

        FlashReg[intr_status] =  FlashReg[intr_status];

        if(GLOB_DeviceInfo.wHWFeatures & FEATURES__INDEX_ADDR)
        {

            Index_Addr(FBA_FPA_FSA_P, (uint32 )(MODE_10 | (flash_bank << 24) |
                    ( (flash_add) >> GLOB_DeviceInfo.nBitsInPageDataSize ) ), 0x41);

            Index_Addr(FBA_FPA_FSA_P, (uint32 )(MODE_10 | (flash_bank << 24) |
                    ( (flash_add) >> GLOB_DeviceInfo.nBitsInPageDataSize ) ), 0x2000|page_count);

            while ((FlashReg[intr_status] & (INTR_STATUS0__LOAD_COMP) ) == 0);


            FBA_FPA_FSA_P   = (volatile uint32*)(FlashMem);
            *FBA_FPA_FSA_P =  (MODE_01 | (flash_bank << 24) |
                    ( (flash_add) >> GLOB_DeviceInfo.nBitsInPageDataSize ) );

            FBA_FPA_FSA_P = (volatile uint32*)((uint32)FlashMem | 0x10);
        }
        else
        {
            FBA_FPA_FSA_P   = (volatile uint32*)((uint32)FlashMem | MODE_10 | (flash_bank << 24) |
                    ( ( (flash_add) >> GLOB_DeviceInfo.nBitsInPageDataSize )   <<2 ) );
            *FBA_FPA_FSA_P = 0x41;

            FBA_FPA_FSA_P   = (volatile uint32*)((uint32)FlashMem | MODE_10 | (flash_bank << 24) |
                    ( ( (flash_add) >> GLOB_DeviceInfo.nBitsInPageDataSize )   <<2 ) );
            *FBA_FPA_FSA_P = 0x2000|page_count;

            while ((FlashReg[intr_status] & (INTR_STATUS0__LOAD_COMP) ) == 0);

            FBA_FPA_FSA_P   = (volatile uint32*)((uint32)FlashMem | MODE_01 | (flash_bank << 24) |
                    ( ( (flash_add) >> GLOB_DeviceInfo.nBitsInPageDataSize )   <<2 ) );
        }


        for (i=0; i<(PageSpareSize + 3)/4; i++)
        {
            *(((uint32 *)page_spare )+ i) = *FBA_FPA_FSA_P;
        }


        if(FlashReg[ECC_ENABLE])
        {






            for (i=0; i<spareFlagBytes ; i++)
                read_data[i] =  page_spare[PageSpareSize-spareFlagBytes +i];


            for (i=0; i< (PageSpareSize - spareFlagBytes   ); i++)
                read_data[spareFlagBytes + i] =  page_spare[i];
        }
        else
        {
            for (i=0; i<PageSpareSize; i++)
                read_data[i] =  page_spare[i];
        }

        if(GLOB_DeviceInfo.wHWFeatures & FEATURES__INDEX_ADDR)
        {
            Index_Addr(FBA_FPA_FSA_P, (uint32 )(MODE_10 | (flash_bank << 24) |
                    ( (flash_add) >> GLOB_DeviceInfo.nBitsInPageDataSize ) ), 0x42);
        }
        else
        {
            FBA_FPA_FSA_P   = (volatile uint32*)((uint32)FlashMem | MODE_10 | (flash_bank << 24) |
                    ( ( (flash_add) >> GLOB_DeviceInfo.nBitsInPageDataSize )   <<2 ) );
            *FBA_FPA_FSA_P = 0x42;
        }
    }

    return status;
}




uint16  NAND_Write_Page_Spare(byte* write_data,BLOCKNODE block,uint16 page,uint16 page_count)
{
    uint32 status = PASS;
    uint32 i;
    volatile uint32 *FBA_FPA_FSA_P;
    ADDRESSTYPE flash_add;
    uint32 PageSpareSize     =     GLOB_DeviceInfo.wPageSpareSize;
    byte page_spare[MAX_PAGE_SPARE_AREA];
    byte read_page_spare[MAX_PAGE_SPARE_AREA];
    uint32 spareFlagBytes  = GLOB_DeviceInfo.wNumPageSpareFlag;
    uint32  flash_bank;
    uint32  intr_status=0;
    uint32 intr_status_addresses[4] = {INTR_STATUS0, INTR_STATUS1, INTR_STATUS2, INTR_STATUS3};

    status = Boundary_Check_Block_Page(block, page, page_count);

    flash_add = ( (ADDRESSTYPE)(block % ( (GLOB_DeviceInfo.wTotalBlocks) / GLOB_totalUsedBanks) ))*
        GLOB_DeviceInfo.wBlockDataSize+page*GLOB_DeviceInfo.wPageDataSize;

    flash_bank = (block) / (GLOB_DeviceInfo.wTotalBlocks /
                GLOB_totalUsedBanks);

    if(status == PASS)
    {
        intr_status = intr_status_addresses[flash_bank];

        NAND_Read_Page_Spare(read_page_spare, block,page,page_count);

        FlashReg[intr_status] =  FlashReg[intr_status];

        if(GLOB_DeviceInfo.wHWFeatures & FEATURES__INDEX_ADDR)
        {
            Index_Addr(FBA_FPA_FSA_P, (uint32 )(MODE_10 | (flash_bank << 24) |
                    ( (flash_add) >> GLOB_DeviceInfo.nBitsInPageDataSize ) ), 0x41);


            FBA_FPA_FSA_P = (volatile uint32*)(FlashMem);
            *FBA_FPA_FSA_P = (uint32 )(MODE_01 | (flash_bank << 24) |
                    ( (flash_add) >> GLOB_DeviceInfo.nBitsInPageDataSize ) );

            FBA_FPA_FSA_P = (volatile uint32*)((uint32)FlashMem | 0x10);
        }
        else
        {
            FBA_FPA_FSA_P   = (volatile uint32*)((uint32)FlashMem | MODE_10 | (flash_bank << 24) |
                    ( ( (flash_add) >> GLOB_DeviceInfo.nBitsInPageDataSize )   <<2 ) );

            *FBA_FPA_FSA_P = 0x41;

            FBA_FPA_FSA_P   = (volatile uint32*)((uint32)FlashMem | MODE_01 | (flash_bank << 24) |
                    ( ( (flash_add) >> GLOB_DeviceInfo.nBitsInPageDataSize )   <<2 ) );
        }

        if(FlashReg[ECC_ENABLE])
        {

            for (i=0; i<spareFlagBytes ; i++)
                page_spare[PageSpareSize-spareFlagBytes   +i] = read_page_spare[i];

            for (i=0; i<(PageSpareSize - spareFlagBytes); i++)
                page_spare[i] = read_page_spare[i];

            for (i=0; i<spareFlagBytes ; i++)
                page_spare[PageSpareSize-spareFlagBytes  +i] = write_data[i];
        }
        else
        {
            for (i=0; i<PageSpareSize; i++)
                page_spare[i] = write_data[i];

        }

        for (i=0; i<(PageSpareSize + 3)/4; i++)
        {
            *FBA_FPA_FSA_P = *(((uint32 *)page_spare)+ i);
        }

        while ((FlashReg[intr_status] & (INTR_STATUS0__PROGRAM_COMP | INTR_STATUS0__PROGRAM_FAIL|INTR_STATUS0__TIME_OUT )) == 0);

        if (FlashReg[intr_status] & (INTR_STATUS0__PROGRAM_FAIL|INTR_STATUS0__TIME_OUT) )
            status = FAIL;

        FlashReg[intr_status] = FlashReg[intr_status];

        if(GLOB_DeviceInfo.wHWFeatures & FEATURES__INDEX_ADDR)
        {

            Index_Addr(FBA_FPA_FSA_P, (uint32 )(MODE_10 | (flash_bank << 24) |
                    ( (flash_add) >> GLOB_DeviceInfo.nBitsInPageDataSize ) ), 0x42);
        }
        else
        {
            FBA_FPA_FSA_P   = (volatile uint32*)((uint32)FlashMem | MODE_10 | (flash_bank << 24) |
                    ( ( (flash_add) >> GLOB_DeviceInfo.nBitsInPageDataSize )   <<2 ) );

            *FBA_FPA_FSA_P = 0x42;
        }
    }

    return status;
}

#if DDMA
static irqreturn_t NAND_DDMA_Isr(int irq, void *dev_id)
{
		struct nand_irq_info *dev = dev_id;
		u32 ints0, ints1, ints2, ints3,intr_status;
		u32 intr[4] = {INTR_STATUS0, INTR_STATUS1,
		INTR_STATUS2, INTR_STATUS3};

	if (!NAND_INTERRUPT_IS_ENABLED)//Return if no Denalli interrupt
		return IRQ_NONE;

	intr_status = intr[dev->flash_bank];
	if (!(FlashReg[intr_status] & DDMA_INT_MASK)) {
		print("ddma_isr: Invalid interrupt for NAND controller. "
			"Ignore it %x\n",FlashReg[intr_status]);
		return IRQ_NONE;
	}
	switch (dev->state) {
	case INT_READ_PAGE_MAIN:
	case INT_PIPELINE_READ_AHEAD:
	case INT_WRITE_PAGE_MAIN:
	case INT_PIPELINE_WRITE_AHEAD:
		NAND_LLD_Enable_Disable_Interrupts(0);//Disable interrupts
		break;
	default:
		printk(KERN_ERR "ddma_isr - Illegal state: 0x%x\n",
		FlashReg[intr_status]);
		return IRQ_NONE;
	}
	dev->state = INT_IDLE_STATE;
	complete(&dev->complete);
	return IRQ_HANDLED;
}
uint16    NAND_DDMA_IRQ_Init (void)
{
	int ret = PASS;

	struct pci_dev *host;
	host = pci_get_device(0x8086, 0x0701, NULL);
	if(host)
	{
		pci_enable_device(host);
		pci_intx(host, 1);
		ddma_irq_number = host->irq;
		pci_dev_put(host);
	}

	//Enable DDMA interrupts
	FlashReg[INTR_EN0]          = DDMA_INT_MASK;
    FlashReg[INTR_EN1]          = DDMA_INT_MASK;
    FlashReg[INTR_EN2]          = DDMA_INT_MASK;
    FlashReg[INTR_EN3]          = DDMA_INT_MASK;


	info.state = INT_IDLE_STATE;
	init_completion(&info.complete);
	if (request_irq(ddma_irq_number, NAND_DDMA_Isr, IRQF_SHARED,
			DDMA_IRQ_NAME, &info))
	{
		printk(KERN_ERR "Spectra: Unable to allocate IRQ\n");
		ret = -ENODEV;
	}

	return ret;
}
uint16    NAND_DDMA_IRQ_Release (void)
{
	free_irq(ddma_irq_number, &info);
	return PASS;
}
#endif



uint16    NAND_Flash_Init (void)
{
#if !SKIP_IN_SIMULATION
    uint32 i;
#endif

#ifdef ELDORA
    FlashReg = (uint32 *)GLOB_HWCTL_REG_BASE;
    FlashMem = (uint32 *)GLOB_HWCTL_MEM_BASE;
#else
    FlashReg = GLOB_MEMMAP_NOCACHE(GLOB_HWCTL_REG_BASE, GLOB_HWCTL_REG_SIZE);
    FlashMem = GLOB_MEMMAP_NOCACHE(GLOB_HWCTL_MEM_BASE, GLOB_HWCTL_MEM_SIZE);
	printk("flashreg is %x\n", (unsigned long)FlashReg);
    printk("flashmem is %x\n", (unsigned long)FlashMem);

#endif

    if(GLOB_init_firsttime_done == 0)
    {
        while((FlashReg[INTR_STATUS0]& INTR_EN0__RST_COMP ) == 0);

        GLOB_init_firsttime_done = 1;
    }
    else
    {
    }

#if !SKIP_IN_SIMULATION

    for ( i =0 ; i < LLD_MAX_FLASH_BANKS; i++)
        GLOB_valid_banks[i] = 1;

    NAND_Flash_Reset();

    for ( i =0 ; i < LLD_MAX_FLASH_BANKS; i++)
        GLOB_valid_banks[i] = 0;
#endif

    FlashReg[GLOBAL_INT_ENABLE] = 0;
    FlashReg[INTR_EN0]          = 0;
    FlashReg[INTR_EN1]          = 0;
    FlashReg[INTR_EN2]          = 0;
    FlashReg[INTR_EN3]          = 0;

    FlashReg[INTR_STATUS0]      = 0xFFFF;
    FlashReg[INTR_STATUS1]      = 0xFFFF;
    FlashReg[INTR_STATUS2]      = 0xFFFF;
    FlashReg[INTR_STATUS3]      = 0xFFFF;

    FlashReg[RB_PIN_ENABLED]    = 0xF;
    FlashReg[CHIP_ENABLE_DONT_CARE] = CHIP_ENABLE_DONT_CARE__FLAG;



#if CUSTOM_CONF_PARAMS
    FlashReg[TRANSFER_SPARE_REG ]     = conf_parameters[DATA_TRANSFER_MODE ];
    FlashReg[LOAD_WAIT_CNT ]          = conf_parameters[LOAD_WAIT_COUNT ];
    FlashReg[PROGRAM_WAIT_CNT ]       = conf_parameters[PROGRAM_WAIT_COUNT ];
    FlashReg[ERASE_WAIT_CNT ]         = conf_parameters[ERASE_WAIT_COUNT ];
    FlashReg[INT_MON_CYCCNT ]         = conf_parameters[INT_MONITOR_CYCLE_COUNT ];
    FlashReg[RB_PIN_ENABLED ]         = conf_parameters[READ_BUSY_PIN_ENABLED ];
    FlashReg[MULTIPLANE_OPERATION ]   = conf_parameters[MULTIPLANE_OPERATION_SUPPORT ];
    FlashReg[PREFETCH_MODE ]          = conf_parameters[PRE_FETCH_MODE ];
    FlashReg[CHIP_ENABLE_DONT_CARE ]  = conf_parameters[CE_DONT_CARE_SUPPORT ];
    FlashReg[COPYBACK_DISABLE ]       = conf_parameters[COPYBACK_SUPPORT ];
    FlashReg[CACHE_WRITE_ENABLE ]     = conf_parameters[CACHE_WRITE_SUPPORT ];
    FlashReg[CACHE_READ_ENABLE ]      = conf_parameters[CACHE_READ_SUPPORT ];
    FlashReg[ECC_ENABLE ]             = conf_parameters[ECC_ENABLE_SELECT ];
    FlashReg[WE_2_RE ]                = conf_parameters[WRITE_ENABLE_2_READ_ENABLE ];
    FlashReg[ADDR_2_DATA ]            = conf_parameters[ADDRESS_2_DATA ];
    FlashReg[RE_2_WE ]                = conf_parameters[READ_ENABLE_2_WRITE_ENABLE ];
    FlashReg[TWO_ROW_ADDR_CYCLES]     = conf_parameters[TWO_ROW_ADDRESS_CYCLES];
    FlashReg[MULTIPLANE_ADDR_RESTRICT]= conf_parameters[MULTIPLANE_ADDRESS_RESTRICT];
    FlashReg[ACC_CLKS]                = conf_parameters[ACC_CLOCKS];
    FlashReg[RDWR_EN_LO_CNT]          = conf_parameters[READ_WRITE_ENABLE_LOW_COUNT];
    FlashReg[RDWR_EN_HI_CNT]          = conf_parameters[READ_WRITE_ENABLE_HIGH_COUNT];
#else
#endif
    // Workaround: This is for Samsung K9GAG08U0D and K9LBG08U0D NAND parts.
    if (FlashReg[MANUFACTURER_ID] == 0xEC && (FlashReg[DEVICE_ID] == 0xD5||FlashReg[DEVICE_ID] == 0xD7))
    {
	//Workaround: This is for Samsung K9HCG08U1M. All parameters of this NAND chip are correct except total blocks
       if(FlashReg[DEVICE_SPARE_AREA_SIZE] == 0x80)
		goto DDMA_NEXT;
	else
	{
	FlashReg[DEVICE_MAIN_AREA_SIZE] = 2048 << (FlashReg[DEVICE_PARAM_1] & 0x3);
	if ((FlashReg[DEVICE_PARAM_1] & 0x4C) == 0x4)
	{
	    FlashReg[DEVICE_SPARE_AREA_SIZE] = 0x80;  // 128B spare area
	} else if ((FlashReg[DEVICE_PARAM_1] & 0x4C) == 0x8)
	 {
	    FlashReg[DEVICE_SPARE_AREA_SIZE] = 0xDA;  // 218B spare area
	}
    }
	}
DDMA_NEXT:
	#if DDMA
	//DDMA interrupt initialize
	return NAND_DDMA_IRQ_Init();
	#endif


    return PASS;
}

#if DDMA
typedef enum OPTYPE
{
    READ_OP,
    WRITE_OP
}OPTYPE;

static void DDMA_Index_Addr_Transactions(volatile uint32 *FBA_FPA_FSA_P, byte* data, ADDRESSTYPE flash_add, uint32 flash_bank, OPTYPE op, uint32 numPages)
{
#ifndef ELDORA
    data = (byte *)GLOB_MEMMAP_TOBUS((uint32 *)data);
#endif
    Index_Addr(FBA_FPA_FSA_P, (uint32 )(MODE_10 | (flash_bank << 24) |
	    ( (flash_add) >> GLOB_DeviceInfo.nBitsInPageDataSize ) ), (uint16)(2 << 12) | ( op << 8) | numPages);

    Index_Addr(FBA_FPA_FSA_P, (uint32 )(MODE_10 | (flash_bank << 24) |
	    ( (uint16)( 0x0000FFFF & ( (uint32)data >> 16) ) << 8) ), (uint16)(2 << 12) | ( 2 << 8) | 0);

    Index_Addr(FBA_FPA_FSA_P, (uint32 )(MODE_10 | (flash_bank << 24) |
	    ( (uint16)(0x0000FFFF & (uint32)data) << 8) ), (uint16)(2 << 12) | ( 3 << 8) | 0);

    Index_Addr(FBA_FPA_FSA_P, (uint32 )(MODE_10 | (flash_bank << 24) |
	    (1 << 16)    | (0x40 << 8) ), (uint16)(2 << 12) | ( 4 << 8) | 0);
}


static void DDMA_Direct_Addr_Transactions(volatile uint32 *FBA_FPA_FSA_P, byte* data, ADDRESSTYPE flash_add, uint32 flash_bank, OPTYPE op, uint32 numPages)
{
#ifndef ELDORA
    data = (byte *)GLOB_MEMMAP_TOBUS((uint32 *)data);
#endif
    FBA_FPA_FSA_P   = (volatile uint32*)((uint32)FlashMem | MODE_10 | (flash_bank << 24) |
	    ( ( (flash_add) >> GLOB_DeviceInfo.nBitsInPageDataSize )   <<2 ) );

    *FBA_FPA_FSA_P = (uint16)(2 << 12) | ( 0 << 8) | numPages;



    FBA_FPA_FSA_P   = (volatile uint32*)((uint32)FlashMem | MODE_10 | (flash_bank << 24) |
	    ( ( 0x0000FFFF & ( (uint32)data >> 16) ) << 8) );
    *FBA_FPA_FSA_P = (uint16)(2 << 12) | ( 2 << 8) | 0;


    FBA_FPA_FSA_P   = (volatile uint32*)((uint32)FlashMem | MODE_10 | (flash_bank << 24) |
	    ( (0x0000FFFF & (uint32)data) << 8) );
    *FBA_FPA_FSA_P = (uint16)(2 << 12) | ( 3 << 8) | 0;


    FBA_FPA_FSA_P   = (volatile uint32*)((uint32)FlashMem | MODE_10 | (flash_bank << 24) |
	    (1 << 16)    | (0x40 << 8) );
    *FBA_FPA_FSA_P = (uint16)(2 << 12) | ( 4 << 8) | 0;
}

#endif





uint16 NAND_Read_Page_Main(byte* read_data, BLOCKNODE block,uint16 page,uint16 page_count)
{
    uint32 status = PASS;
    ADDRESSTYPE flash_add;
    uint16 err_byte;
    byte  err_sector;
    byte    err_device;
    uint16 ecc_correction_info;
    uint16 err_address;
    volatile uint32 *FBA_FPA_FSA_P;
    uint32  eccSectorSize;
    uint32 intr_status=0;
    uint32 flash_bank;

    uint32 intr_status_addresses[4] = {INTR_STATUS0, INTR_STATUS1, INTR_STATUS2, INTR_STATUS3};
#if DDMA

#else
    uint32  i;
#endif

    byte* read_data_l;

    eccSectorSize   = ECC_SECTOR_SIZE * (GLOB_DeviceInfo.wDevicesConnected);


    status = Boundary_Check_Block_Page(block, page, page_count);


    flash_add = ( (ADDRESSTYPE)(block % ( (GLOB_DeviceInfo.wTotalBlocks) / GLOB_totalUsedBanks) ))*GLOB_DeviceInfo.wBlockDataSize+page*GLOB_DeviceInfo.wPageDataSize;

    flash_bank = (block) / (GLOB_DeviceInfo.wTotalBlocks /
                GLOB_totalUsedBanks);

    if(status == PASS)
    {
        intr_status = intr_status_addresses[flash_bank];

        FlashReg[TRANSFER_SPARE_REG] = 0;

        FlashReg[intr_status] =  FlashReg[intr_status];

        if(page_count >1)
        {
#ifdef ELDORA
            status = FAIL;
#else
            read_data_l = read_data;

            while (page_count > MAX_PAGES_PER_RW)
            {

                if(FlashReg[MULTIPLANE_OPERATION])
                    status = NAND_Multiplane_Read(read_data_l,block,page,MAX_PAGES_PER_RW);
                else
                    status = NAND_Pipeline_Read_Ahead(read_data_l,block,page,MAX_PAGES_PER_RW);

                if (status == FAIL)
                    return status;

                read_data_l  +=  MAX_PAGES_PER_RW  * GLOB_DeviceInfo.wPageDataSize;
                page_count -= MAX_PAGES_PER_RW;
                page += MAX_PAGES_PER_RW;
            }

            if(FlashReg[MULTIPLANE_OPERATION])
                NAND_Multiplane_Read(read_data_l,block,page,page_count);
            else
                NAND_Pipeline_Read_Ahead(read_data_l,block,page,page_count);
#endif
        }
        else
        {
#if DDMA
            FlashReg[DMA_ENABLE] = 1;

            while(!(FlashReg[DMA_ENABLE] & DMA_ENABLE__FLAG));


            FlashReg[TRANSFER_SPARE_REG] = 0;

            FlashReg[intr_status] =  FlashReg[intr_status];

			/* Fill the int_nand_info structure */
			info.state = INT_READ_PAGE_MAIN;
			info.flash_bank = flash_bank;

            if(GLOB_DeviceInfo.wHWFeatures & FEATURES__INDEX_ADDR)
            {
                DDMA_Index_Addr_Transactions(FBA_FPA_FSA_P, read_data, flash_add, flash_bank, READ_OP, 1);
            }
            else
            {
                DDMA_Direct_Addr_Transactions(FBA_FPA_FSA_P, read_data, flash_add, flash_bank, READ_OP, 1);
            }
			NAND_LLD_Enable_Disable_Interrupts(1);/* Enable interrupt */


		if (!wait_for_completion_timeout(&info.complete, IRQ_WAIT_TIME_OUT))
		{
			print( "Wait for completion timeout blk %d page %d \n status %x intr %x", block,page,(FlashReg[intr_status] & DDMA_INT_MASK),(FlashReg[GLOBAL_INT_ENABLE] & GLOBAL_INT_ENABLE__FLAG));
			DISABLE_NAND_INTERRUPT_WHEN_ENABLED;
			status = FAIL;
		}
		else
		{
			DISABLE_NAND_INTERRUPT_WHEN_ENABLED;

            if(FlashReg[ECC_ENABLE])
            {
                while ((FlashReg[intr_status] & (INTR_STATUS0__ECC_TRANSACTION_DONE
                        |INTR_STATUS0__ECC_ERR|INTR_STATUS0__TIME_OUT )) == 0);

                if (FlashReg[intr_status] & INTR_STATUS0__TIME_OUT)
                {
                    status = FAIL;
                    FlashReg[intr_status] = INTR_STATUS0__TIME_OUT;
                }

                if (FlashReg[intr_status] & INTR_STATUS0__ECC_ERR)
                {
                    FlashReg[intr_status] = INTR_STATUS0__ECC_ERR;

                    do
                    {

                        err_address = FlashReg[ECC_ERROR_ADDRESS];
                        err_byte = err_address & ECC_ERROR_ADDRESS__OFFSET ;
                        err_sector = ((err_address & ECC_ERROR_ADDRESS__SECTOR_NR)>>12);

                        ecc_correction_info = FlashReg[ERR_CORRECTION_INFO];
                        err_device = ( ( ecc_correction_info & ERR_CORRECTION_INFO__DEVICE_NR) >> 8);
                        if(ecc_correction_info & ERR_CORRECTION_INFO__ERROR_TYPE)
                        {
                            status = FAIL;
                        }
                        else
                        {
                            if (err_byte < ECC_SECTOR_SIZE)
                            {
                                *((byte *)read_data+err_sector*eccSectorSize+(err_byte * GLOB_DeviceInfo.wDevicesConnected) + err_device )^=
                                    (ecc_correction_info & ERR_CORRECTION_INFO__BYTEMASK);
                            }
                        }
                    }while(0 == (ecc_correction_info & ERR_CORRECTION_INFO__LAST_ERR_INFO));
                }

                if( FlashReg[intr_status]  & INTR_STATUS0__ECC_TRANSACTION_DONE & INTR_STATUS0__ECC_ERR )
                {
                    FlashReg[intr_status] = (INTR_STATUS0__ECC_TRANSACTION_DONE |INTR_STATUS0__ECC_ERR );
                }
                else if( FlashReg[intr_status]  & INTR_STATUS0__ECC_TRANSACTION_DONE )
                {
                    FlashReg[intr_status] = INTR_STATUS0__ECC_TRANSACTION_DONE;
                }
                else if( FlashReg[intr_status]  & INTR_STATUS0__ECC_ERR )
                {
                    FlashReg[intr_status] = INTR_STATUS0__ECC_ERR;
                }
            }
            else
            {
                while ((FlashReg[intr_status] & (INTR_STATUS0__DMA_CMD_COMP|INTR_STATUS0__TIME_OUT))  == 0);
                if (FlashReg[intr_status] & INTR_STATUS0__TIME_OUT)
                 {
                    status = FAIL;
                    FlashReg[intr_status] = INTR_STATUS0__TIME_OUT;
                 }


                FlashReg[intr_status] = INTR_STATUS0__DMA_CMD_COMP;

            }
			}
            FlashReg[intr_status] = FlashReg[intr_status];

            FlashReg[DMA_ENABLE] = 0;

            while((FlashReg[DMA_ENABLE] & DMA_ENABLE__FLAG));


            return status;

#else
            if(GLOB_DeviceInfo.wHWFeatures & FEATURES__INDEX_ADDR)
            {

                Index_Addr(FBA_FPA_FSA_P, (uint32 )(MODE_10 | (flash_bank << 24) |
                        ( (flash_add) >> GLOB_DeviceInfo.nBitsInPageDataSize ) ),  0x42);

                Index_Addr(FBA_FPA_FSA_P, (uint32 )(MODE_10 | (flash_bank << 24) |
                        ( (flash_add) >> GLOB_DeviceInfo.nBitsInPageDataSize ) ),  0x2000|page_count);

                while ((FlashReg[intr_status] & (INTR_STATUS0__LOAD_COMP|INTR_STATUS0__TIME_OUT) ) == 0);

                FBA_FPA_FSA_P   = (volatile uint32*)(FlashMem);
                *FBA_FPA_FSA_P =  (MODE_01 | (flash_bank << 24) |
                        ( (flash_add) >> GLOB_DeviceInfo.nBitsInPageDataSize ) );

                FBA_FPA_FSA_P = (volatile uint32*)((uint32)FlashMem | 0x10);
            }
            else
            {
                FBA_FPA_FSA_P   = (volatile uint32*)((uint32)FlashMem | MODE_10 | (flash_bank << 24) |
                        ( ( (flash_add) >> GLOB_DeviceInfo.nBitsInPageDataSize )   <<2 ) );
                *FBA_FPA_FSA_P = 0x42;


                FBA_FPA_FSA_P   = (volatile uint32*)((uint32)FlashMem | MODE_10 | (flash_bank << 24) |
                        ( ( (flash_add) >> GLOB_DeviceInfo.nBitsInPageDataSize )   <<2 ) );
                *FBA_FPA_FSA_P = 0x2000|page_count;

                while ((FlashReg[intr_status] & (INTR_STATUS0__LOAD_COMP|INTR_STATUS0__TIME_OUT) ) == 0);


                FBA_FPA_FSA_P   = (volatile uint32*)((uint32)FlashMem | MODE_01 | (flash_bank << 24) |
                        ( ( (flash_add) >> GLOB_DeviceInfo.nBitsInPageDataSize )   <<2 ) );

            }

            for (i=0; i<GLOB_DeviceInfo.wPageDataSize/4; i++)
            {
                *(((uint32 *)read_data )+ i) = *FBA_FPA_FSA_P;
            }


            if(FlashReg[ECC_ENABLE])
            {
                while ((FlashReg[intr_status] & (INTR_STATUS0__ECC_TRANSACTION_DONE
                        |INTR_STATUS0__ECC_ERR |INTR_STATUS0__TIME_OUT)) == 0);
                if (FlashReg[intr_status] & INTR_STATUS0__TIME_OUT)
                {
                    status = FAIL;
                    FlashReg[intr_status] = INTR_STATUS0__TIME_OUT;
                }

                if (FlashReg[intr_status] & INTR_STATUS0__ECC_ERR)
                {
                    FlashReg[intr_status] = INTR_STATUS0__ECC_ERR;

                    do
                    {

                        err_address = FlashReg[ECC_ERROR_ADDRESS];
                        err_byte = err_address & ECC_ERROR_ADDRESS__OFFSET ;
                        err_sector = ((err_address & ECC_ERROR_ADDRESS__SECTOR_NR)>>12);

                        ecc_correction_info = FlashReg[ERR_CORRECTION_INFO];
                        err_device = ( ( ecc_correction_info & ERR_CORRECTION_INFO__DEVICE_NR) >> 8);
                        if(ecc_correction_info & ERR_CORRECTION_INFO__ERROR_TYPE)
                        {
                            status = FAIL;
                        }
                        else
                        {
                            if (err_byte < ECC_SECTOR_SIZE)
                            {
                                *((byte *)read_data+err_sector*eccSectorSize+(err_byte * GLOB_DeviceInfo.wDevicesConnected) + err_device )^=
                                    (ecc_correction_info & ERR_CORRECTION_INFO__BYTEMASK);
                            }
                        }
                    }while(0 == (ecc_correction_info & ERR_CORRECTION_INFO__LAST_ERR_INFO));
                }

                if( FlashReg[intr_status]  & INTR_STATUS0__ECC_TRANSACTION_DONE & INTR_STATUS0__ECC_ERR )
                {
                    FlashReg[intr_status] = (INTR_STATUS0__ECC_TRANSACTION_DONE |INTR_STATUS0__ECC_ERR );

                }
                else if( FlashReg[intr_status]  & INTR_STATUS0__ECC_TRANSACTION_DONE )
                {
                    FlashReg[intr_status] = INTR_STATUS0__ECC_TRANSACTION_DONE;
                }
                else if( FlashReg[intr_status]  & INTR_STATUS0__ECC_ERR )
                {
                    FlashReg[intr_status] = INTR_STATUS0__ECC_ERR;
                }
            }
            else
            {
                if (FlashReg[intr_status] & INTR_STATUS0__TIME_OUT)
                    status = FAIL;
                FlashReg[intr_status] = FlashReg[intr_status];
            }
#endif
        }
    }

    return status;
}

#ifndef ELDORA



void NAND_Conv_Spare_Data_Log2Phy_Format(byte* data)
{
    int i;
    const uint32 spareFlagBytes = GLOB_DeviceInfo.wNumPageSpareFlag;
    const uint32 PageSpareSize  = GLOB_DeviceInfo.wPageSpareSize;

    if(FlashReg[ECC_ENABLE])
    {
        for (i=spareFlagBytes-1; i>=0; i++)
            data[PageSpareSize-spareFlagBytes +i] = data[i];
    }
}

void NAND_Conv_Spare_Data_Phy2Log_Format(byte* data)
{
    int i;
    const uint32 spareFlagBytes = GLOB_DeviceInfo.wNumPageSpareFlag;
    const uint32 PageSpareSize  = GLOB_DeviceInfo.wPageSpareSize;

    if(FlashReg[ECC_ENABLE])
    {
        for (i=0; i<spareFlagBytes ; i++)
            data[i] =  data[PageSpareSize-spareFlagBytes +i];
    }
}


void NAND_Conv_Main_Spare_Data_Log2Phy_Format(byte* data, uint16 page_count)
{
    const uint32 PageSize  = GLOB_DeviceInfo.wPageSize;
    const uint32 PageDataSize  = GLOB_DeviceInfo.wPageDataSize;
    const uint32 eccBytes = GLOB_DeviceInfo.wECCBytesPerSector;
    const uint32 spareSkipBytes  = GLOB_DeviceInfo.wSpareSkipBytes;
    const uint32 spareFlagBytes = GLOB_DeviceInfo.wNumPageSpareFlag;
    const uint32 eccSectorSize = ECC_SECTOR_SIZE * (GLOB_DeviceInfo.wDevicesConnected);
    uint32 page_offset;
    int i,j;

    if(FlashReg[ECC_ENABLE])
    {
        while(page_count > 0)
        {
            page_offset = (page_count-1)*PageSize;

            j = (GLOB_DeviceInfo.wPageDataSize / eccSectorSize);
            for (i=spareFlagBytes-1; i>=0; i--)
                data[page_offset + (eccSectorSize+eccBytes)*j + i] = data[page_offset + PageDataSize + i];
            for (j--; j>=1; j--)
            {
                for (i=eccSectorSize-1; i>=0; i--)
                    data[page_offset + (eccSectorSize+eccBytes)*j + i] = data[page_offset + eccSectorSize*j + i];
            }

            for (i=(PageSize - spareSkipBytes)-1 ;  i >= PageDataSize; i--)
                data[page_offset + i + spareSkipBytes] = data[page_offset + i];
            page_count--;
        }
    }
}

void NAND_Conv_Main_Spare_Data_Phy2Log_Format(byte* data, uint16 page_count)
{
    const uint32 PageSize  = GLOB_DeviceInfo.wPageSize;
    const uint32 PageDataSize  = GLOB_DeviceInfo.wPageDataSize;
    const uint32 eccBytes = GLOB_DeviceInfo.wECCBytesPerSector;
    const uint32 spareSkipBytes  = GLOB_DeviceInfo.wSpareSkipBytes;
    const uint32 spareFlagBytes = GLOB_DeviceInfo.wNumPageSpareFlag;
    const uint32 eccSectorSize = ECC_SECTOR_SIZE * (GLOB_DeviceInfo.wDevicesConnected);
    uint32 page_offset;
    int i,j;

    if(FlashReg[ECC_ENABLE])
    {
        while(page_count > 0)
        {
            page_offset = (page_count-1)*PageSize;
            for (i= PageDataSize;  i < PageSize - spareSkipBytes; i++)
                data[page_offset + i] = data[page_offset + i + spareSkipBytes];
            for (j=1; j<GLOB_DeviceInfo.wPageDataSize / eccSectorSize; j++)
            {
                for (i=0; i<eccSectorSize; i++)
                    data[page_offset + eccSectorSize*j + i] = data[page_offset + (eccSectorSize+eccBytes)*j + i];
            }
            for (i=0; i<spareFlagBytes; i++)
                data[page_offset + PageDataSize + i] =  data[page_offset + (eccSectorSize+eccBytes)*j + i];

            page_count--;
        }
    }
}




uint16  NAND_Multiplane_Read(byte* read_data,BLOCKNODE block,uint16 page,uint16 page_count)
{
    uint32 status = PASS;
    volatile uint32 *FBA_FPA_FSA_P;
    uint32 PageSize     = GLOB_DeviceInfo.wPageDataSize;
    uint32 NumPages=page_count;
    ADDRESSTYPE flash_add;
    uint16  err_byte;
    byte    err_sector;
    byte    err_device;
    uint16  err_page=0;
    uint16  ecc_correction_info;
    uint16  err_address;

    uint32  eccSectorSize;
    uint32  flash_bank;
    uint32  intr_status=0;
    uint32 intr_status_addresses[4] = {INTR_STATUS0, INTR_STATUS1, INTR_STATUS2, INTR_STATUS3};
    uint32 err_page_addresses[4] = {ERR_PAGE_ADDR0, ERR_PAGE_ADDR1, ERR_PAGE_ADDR2, ERR_PAGE_ADDR3};




#if DDMA
    uint32      ecc_done_OR_dma_comp;
#else
    uint32 sector_count = 0;
    uint32 SectorStart, SectorEnd;
    uint32      bSectorsPerPage =4;
    uint32 i,page_num=0;
    uint32 plane =0;
    byte *read_data_l   = read_data;
#endif

    eccSectorSize   = ECC_SECTOR_SIZE * (GLOB_DeviceInfo.wDevicesConnected);



    status = Boundary_Check_Block_Page(block, page, page_count);

    flash_add = ( (ADDRESSTYPE)(block % ( (GLOB_DeviceInfo.wTotalBlocks) / GLOB_totalUsedBanks) ))*GLOB_DeviceInfo.wBlockDataSize+page*GLOB_DeviceInfo.wPageDataSize;

    flash_bank = (block) / (GLOB_DeviceInfo.wTotalBlocks /
                GLOB_totalUsedBanks);

    if(status == PASS)
    {
        intr_status = intr_status_addresses[flash_bank];

        FlashReg[intr_status] =  FlashReg[intr_status];

        FlashReg[TRANSFER_SPARE_REG] = 0;

        FlashReg[MULTIPLANE_OPERATION ]     = 0x0001;

#if DDMA

        FlashReg[DMA_ENABLE] = 1;

        while(!(FlashReg[DMA_ENABLE] & DMA_ENABLE__FLAG));


        if(GLOB_DeviceInfo.wHWFeatures & FEATURES__INDEX_ADDR)
        {

            Index_Addr(FBA_FPA_FSA_P, (uint32 )(MODE_10 | (flash_bank << 24) |
                    ( (flash_add) >> GLOB_DeviceInfo.nBitsInPageDataSize ) ), 0x42);

            DDMA_Index_Addr_Transactions(FBA_FPA_FSA_P, read_data, flash_add, flash_bank, READ_OP, NumPages);
        }
        else
        {
            FBA_FPA_FSA_P   = (volatile uint32*)((uint32)FlashMem | MODE_10 | (flash_bank << 24) |
                    ( ( (flash_add) >> GLOB_DeviceInfo.nBitsInPageDataSize )   <<2 ) );

            *FBA_FPA_FSA_P = 0x42;

            DDMA_Direct_Addr_Transactions(FBA_FPA_FSA_P, read_data, flash_add, flash_bank, READ_OP, NumPages);
        }

        ecc_done_OR_dma_comp = 0;
        while(1)
        {
            if(FlashReg[ECC_ENABLE])
            {
                while (FlashReg[intr_status]  == 0);


                if (FlashReg[intr_status] & INTR_STATUS0__TIME_OUT)
                {
                    status = FAIL;
                    FlashReg[intr_status] = INTR_STATUS0__TIME_OUT;
                    break;
                }

                else if( FlashReg[intr_status]  & INTR_STATUS0__ECC_ERR )
                {
                    FlashReg[intr_status] = INTR_STATUS0__ECC_ERR;


                    do
                    {

                        err_page = FlashReg[err_page_addresses[flash_bank]];

                        err_address = FlashReg[ECC_ERROR_ADDRESS];
                        err_byte = err_address & ECC_ERROR_ADDRESS__OFFSET ;
                        err_sector = ((err_address & ECC_ERROR_ADDRESS__SECTOR_NR)>>12);

                        ecc_correction_info = FlashReg[ERR_CORRECTION_INFO];
                        err_device = ( ( ecc_correction_info & ERR_CORRECTION_INFO__DEVICE_NR) >> 8);
                        if(ecc_correction_info & ERR_CORRECTION_INFO__ERROR_TYPE)
                        {
                            status = FAIL;
                        }
                        else
                        {
                            if (err_byte < ECC_SECTOR_SIZE)
                            {
                                 *((byte *)(read_data + ((err_page -page) *  GLOB_DeviceInfo.wPageDataSize) + err_sector*eccSectorSize + (err_byte *
                                 GLOB_DeviceInfo.wDevicesConnected) + err_device) )^= (ecc_correction_info & ERR_CORRECTION_INFO__BYTEMASK);
                            }
                        }
                    }while(0 == (ecc_correction_info & ERR_CORRECTION_INFO__LAST_ERR_INFO));
                }
                else if( FlashReg[intr_status]  & INTR_STATUS0__DMA_CMD_COMP)
                {
                    FlashReg[intr_status] = (INTR_STATUS0__DMA_CMD_COMP);

                    if(ecc_done_OR_dma_comp == 1)
                        break;

                    ecc_done_OR_dma_comp = 1;
                }
                else if( FlashReg[intr_status]  & INTR_STATUS0__ECC_TRANSACTION_DONE )
                {
                    FlashReg[intr_status] = (INTR_STATUS0__ECC_TRANSACTION_DONE);

                    if(ecc_done_OR_dma_comp == 1)
                        break;

                    ecc_done_OR_dma_comp = 1;
                }
            }
            else
            {
                while( (FlashReg[intr_status] & (INTR_STATUS0__DMA_CMD_COMP|INTR_STATUS0__TIME_OUT))  == 0);
                if (FlashReg[intr_status] & INTR_STATUS0__TIME_OUT)
                {
                    status = FAIL;
                    FlashReg[intr_status] = INTR_STATUS0__TIME_OUT;
                }

                FlashReg[intr_status] = INTR_STATUS0__DMA_CMD_COMP;
                break;
            }

            FlashReg[intr_status] = ( (~INTR_STATUS0__ECC_ERR) & (~INTR_STATUS0__ECC_TRANSACTION_DONE)
                    & (~INTR_STATUS0__DMA_CMD_COMP) ) ;

        }

        FlashReg[intr_status] = FlashReg[intr_status];

        FlashReg[DMA_ENABLE] = 0;

        while((FlashReg[DMA_ENABLE] & DMA_ENABLE__FLAG));

        FlashReg[MULTIPLANE_OPERATION ]     = 0x0000;

#else


        if(FlashReg[ECC_ENABLE])
            FlashReg[intr_status] = (INTR_STATUS0__ECC_ERR );

        if(GLOB_DeviceInfo.wHWFeatures & FEATURES__INDEX_ADDR)
        {
            Index_Addr(FBA_FPA_FSA_P, (uint32 )(MODE_10 | (flash_bank << 24) |
                    ( (flash_add) >> GLOB_DeviceInfo.nBitsInPageDataSize ) ), 0x42);

            Index_Addr(FBA_FPA_FSA_P, (uint32 )(MODE_10 | (flash_bank << 24) |
                    ( (flash_add) >> GLOB_DeviceInfo.nBitsInPageDataSize ) ), 0x2000|page_count);
        }
        else
        {

            FBA_FPA_FSA_P   = (volatile uint32*)((uint32)FlashMem | MODE_10 | (flash_bank << 24) |
                    ( ( (flash_add) >> GLOB_DeviceInfo.nBitsInPageDataSize )   <<2 ) );

            *FBA_FPA_FSA_P = 0x42;

            FBA_FPA_FSA_P   = (volatile uint32*)((uint32)FlashMem | MODE_10 | (flash_bank << 24) |
                    ( ( (flash_add) >> GLOB_DeviceInfo.nBitsInPageDataSize )   <<2 ) );

            *FBA_FPA_FSA_P = 0x2000|NumPages;
        }

        while(NumPages >0)
        {

            if(plane==0)
            {

                if(GLOB_DeviceInfo.wHWFeatures & FEATURES__INDEX_ADDR)
                {
                    FBA_FPA_FSA_P = (volatile uint32*)(FlashMem);
                    *FBA_FPA_FSA_P = (uint32 )(MODE_01 | (flash_bank << 24) |
                            ( (flash_add + page_num * (GLOB_DeviceInfo.wPageDataSize) ) >> GLOB_DeviceInfo.nBitsInPageDataSize ) );

                    FBA_FPA_FSA_P = (volatile uint32*)((uint32)FlashMem | 0x10);
                }
                else
                {
                    FBA_FPA_FSA_P   = (volatile uint32*)((uint32)FlashMem | MODE_01 | (flash_bank << 24) |
                            ( ( (flash_add + page_num * (GLOB_DeviceInfo.wPageDataSize) ) >> GLOB_DeviceInfo.nBitsInPageDataSize ) << 2 )  );
                }

                plane=1;
            }
            else
            {
                if(GLOB_DeviceInfo.wHWFeatures & FEATURES__INDEX_ADDR)
                {
                    FBA_FPA_FSA_P = (volatile uint32*)(FlashMem);
                    *FBA_FPA_FSA_P = (uint32 )(MODE_01 | (flash_bank << 24) |
                            ( ( (flash_add+GLOB_DeviceInfo.wBlockDataSize) + page_num * (GLOB_DeviceInfo.wPageDataSize) ) >> GLOB_DeviceInfo.nBitsInPageDataSize )  );

                    FBA_FPA_FSA_P = (volatile uint32*)((uint32)FlashMem | 0x10);
                }
                else
                {
                    FBA_FPA_FSA_P   = (volatile uint32*)((uint32)FlashMem | MODE_01 | (flash_bank << 24) |
                            ( ( ( (flash_add+GLOB_DeviceInfo.wBlockDataSize) + page_num * (GLOB_DeviceInfo.wPageDataSize) ) >> GLOB_DeviceInfo.nBitsInPageDataSize ) << 2 ) );
                }

                plane=0;
            }

            for(sector_count = 0; sector_count < bSectorsPerPage ; sector_count++)
            {

                SectorStart = sector_count * (GLOB_DeviceInfo.wPageDataSize/( 4 * bSectorsPerPage ) );
                SectorEnd = (sector_count + 1) * (GLOB_DeviceInfo.wPageDataSize/( 4 * bSectorsPerPage ) );


                for (i=SectorStart; i<SectorEnd; i++)
                {
                    *(((uint32 *)read_data_l )+ i) = *FBA_FPA_FSA_P;
                }



                if(FlashReg[ECC_ENABLE])
                {
                    if (FlashReg[intr_status] & INTR_STATUS0__ECC_ERR)
                    {
                        FlashReg[intr_status] = INTR_STATUS0__ECC_ERR;

                        do
                        {
                            err_page = FlashReg[err_page_addresses[flash_bank]];

                            err_address = FlashReg[ECC_ERROR_ADDRESS];
                            err_byte = err_address & ECC_ERROR_ADDRESS__OFFSET ;
                            err_sector = ((err_address & ECC_ERROR_ADDRESS__SECTOR_NR)>>12);

                            ecc_correction_info = FlashReg[ERR_CORRECTION_INFO];
                            err_device = ( ( ecc_correction_info & ERR_CORRECTION_INFO__DEVICE_NR) >> 8);
                            if(ecc_correction_info & ERR_CORRECTION_INFO__ERROR_TYPE)
                            {
                                status = FAIL;
                            }
                            else
                            {
                                if (err_byte < ECC_SECTOR_SIZE)
                                {
                                    *((byte *)(read_data + ((err_page-page) *  GLOB_DeviceInfo.wPageDataSize) + err_sector*eccSectorSize + (err_byte *
                                            GLOB_DeviceInfo.wDevicesConnected) + err_device) )^= (ecc_correction_info & ERR_CORRECTION_INFO__BYTEMASK);
                                }
                            }
                        }while(0 == (ecc_correction_info & ERR_CORRECTION_INFO__LAST_ERR_INFO));
                    }
                }
                else
                {
                }
            }

            if(plane==0)
                page_num++;

            read_data_l += PageSize;
            --NumPages;
        }



        if(FlashReg[ECC_ENABLE])
        {
            while ((FlashReg[intr_status] & (INTR_STATUS0__ECC_TRANSACTION_DONE
                    |INTR_STATUS0__ECC_ERR |INTR_STATUS0__TIME_OUT)) == 0);

            if (FlashReg[intr_status] & INTR_STATUS0__TIME_OUT)
            {
                status = FAIL;
                FlashReg[intr_status] = INTR_STATUS0__TIME_OUT;
            }

            if( FlashReg[intr_status]  & INTR_STATUS0__ECC_ERR )
            {
                FlashReg[intr_status] = INTR_STATUS0__ECC_ERR;


                do
                {
                    err_page = FlashReg[err_page_addresses[flash_bank]];

                    err_address = FlashReg[ECC_ERROR_ADDRESS];
                    err_byte = err_address & ECC_ERROR_ADDRESS__OFFSET ;
                    err_sector = ((err_address & ECC_ERROR_ADDRESS__SECTOR_NR)>>12);

                    ecc_correction_info = FlashReg[ERR_CORRECTION_INFO];
                    err_device = ( ( ecc_correction_info & ERR_CORRECTION_INFO__DEVICE_NR) >> 8);
                    if(ecc_correction_info & ERR_CORRECTION_INFO__ERROR_TYPE)
                    {
                        status = FAIL;
                    }
                    else
                    {
                        if (err_byte < ECC_SECTOR_SIZE)
                        {
                            *((byte *)(read_data + ((err_page -page) *  GLOB_DeviceInfo.wPageDataSize) + err_sector*eccSectorSize + (err_byte *
                                    GLOB_DeviceInfo.wDevicesConnected) + err_device) )^= (ecc_correction_info & ERR_CORRECTION_INFO__BYTEMASK);
                        }
                    }
                }while(0 == (ecc_correction_info & ERR_CORRECTION_INFO__LAST_ERR_INFO));

                while ((FlashReg[intr_status] & (INTR_STATUS0__ECC_TRANSACTION_DONE)) == 0);


                FlashReg[intr_status] = (INTR_STATUS0__ECC_TRANSACTION_DONE);
            }
            else if( FlashReg[intr_status]  & INTR_STATUS0__ECC_TRANSACTION_DONE )
            {
                FlashReg[intr_status] = (INTR_STATUS0__ECC_TRANSACTION_DONE);
            }
        }
        else
        {
        }

        FlashReg[MULTIPLANE_OPERATION ]     = 0x0000;

#endif
    }
    return status;
}


uint16  NAND_Pipeline_Read_Ahead(byte* read_data,BLOCKNODE block,uint16 page,uint16 page_count)
{
    uint32 status = PASS;
    volatile uint32 *FBA_FPA_FSA_P;
    uint32 NumPages=page_count;
    ADDRESSTYPE flash_add;
    uint16 err_byte;
    byte   err_sector;
    byte   err_device;
    uint16 err_page=0;
    uint16 ecc_correction_info;
    uint16 err_address;

    uint32  eccSectorSize;
    uint32  flash_bank;
    uint32  intr_status=0;
    uint32 intr_status_addresses[4] = {INTR_STATUS0, INTR_STATUS1, INTR_STATUS2, INTR_STATUS3};
    uint32 err_page_addresses[4] = {ERR_PAGE_ADDR0, ERR_PAGE_ADDR1, ERR_PAGE_ADDR2, ERR_PAGE_ADDR3};
#if DDMA
    uint32  ecc_done_OR_dma_comp;
#else
    uint32 PageSize     = GLOB_DeviceInfo.wPageDataSize;
    uint32  sector_count = 0;
    uint32  SectorStart, SectorEnd;
    uint32  bSectorsPerPage =4;
    uint32  i,page_num=0;
    byte *read_data_l   = read_data;
#endif

    eccSectorSize   = ECC_SECTOR_SIZE * (GLOB_DeviceInfo.wDevicesConnected);

    status = Boundary_Check_Block_Page(block, page, page_count);

    if(page_count < 2)
    {
        status = FAIL;
    }

    flash_add = ( (ADDRESSTYPE)(block % ( (GLOB_DeviceInfo.wTotalBlocks) / GLOB_totalUsedBanks) ))*GLOB_DeviceInfo.wBlockDataSize+page*GLOB_DeviceInfo.wPageDataSize;

    flash_bank = (block) / (GLOB_DeviceInfo.wTotalBlocks /
                GLOB_totalUsedBanks);

    if(status == PASS)
    {
        intr_status = intr_status_addresses[flash_bank];

        FlashReg[intr_status] =  FlashReg[intr_status];

#if DDMA
        FlashReg[DMA_ENABLE] = 1;

        while(!(FlashReg[DMA_ENABLE] & DMA_ENABLE__FLAG));


        FlashReg[TRANSFER_SPARE_REG] = 0;
		/* Fill the int_nand_info structure */
		info.state = INT_PIPELINE_READ_AHEAD;
		info.flash_bank = flash_bank;

        if(GLOB_DeviceInfo.wHWFeatures & FEATURES__INDEX_ADDR)
        {

            Index_Addr(FBA_FPA_FSA_P, (uint32 )(MODE_10 | (flash_bank << 24) |
                    ( (flash_add) >> GLOB_DeviceInfo.nBitsInPageDataSize ) ), 0x42);

            DDMA_Index_Addr_Transactions(FBA_FPA_FSA_P, read_data, flash_add, flash_bank, READ_OP, NumPages);
        }
        else
        {
            FBA_FPA_FSA_P   = (volatile uint32*)((uint32)FlashMem | MODE_10 | (flash_bank << 24) |
                    ( ( (flash_add) >> GLOB_DeviceInfo.nBitsInPageDataSize )   <<2 ) );

            *FBA_FPA_FSA_P = 0x42;

            DDMA_Direct_Addr_Transactions(FBA_FPA_FSA_P, read_data, flash_add, flash_bank, READ_OP, NumPages);
        }
		NAND_LLD_Enable_Disable_Interrupts(1);/* Enable interrupt */

        ecc_done_OR_dma_comp = 0;

		if (!wait_for_completion_timeout(&info.complete, IRQ_WAIT_TIME_OUT))
		{
			print(KERN_DEBUG "Wait for completion timeout blk %d page %d \n", block,page);
			DISABLE_NAND_INTERRUPT_WHEN_ENABLED;
			status = FAIL;
		}
		else
		{
			DISABLE_NAND_INTERRUPT_WHEN_ENABLED;

        while(1)
        {
            if(FlashReg[ECC_ENABLE])
            {
            //    while (FlashReg[intr_status]  == 0);


                if (FlashReg[intr_status] & INTR_STATUS0__TIME_OUT)
                {
                    status = FAIL;
                    break;
                }

                else if( FlashReg[intr_status]  & INTR_STATUS0__ECC_ERR )
                {
                    FlashReg[intr_status] = INTR_STATUS0__ECC_ERR;


                    do
                    {

                        if(flash_bank == 0)
                            err_page = FlashReg[ERR_PAGE_ADDR0];
                        else if(flash_bank == 1)
                            err_page = FlashReg[ERR_PAGE_ADDR1];
                        else if(flash_bank == 2)
                            err_page = FlashReg[ERR_PAGE_ADDR2];
                        else if(flash_bank == 3)
                            err_page = FlashReg[ERR_PAGE_ADDR3];

                        err_address = FlashReg[ECC_ERROR_ADDRESS];
                        err_byte = err_address & ECC_ERROR_ADDRESS__OFFSET ;
                        err_sector = ((err_address & ECC_ERROR_ADDRESS__SECTOR_NR)>>12);

                        ecc_correction_info = FlashReg[ERR_CORRECTION_INFO];
                        err_device = ( ( ecc_correction_info & ERR_CORRECTION_INFO__DEVICE_NR) >> 8);
                        if(ecc_correction_info & ERR_CORRECTION_INFO__ERROR_TYPE)
                        {
                            status = FAIL;
                        }
                        else
                        {
                            if (err_byte < ECC_SECTOR_SIZE)
                            {
                                *((byte *)(read_data + ((err_page - page) *  GLOB_DeviceInfo.wPageDataSize) + err_sector*eccSectorSize + (err_byte *
                                        GLOB_DeviceInfo.wDevicesConnected) + err_device) )^= (ecc_correction_info & ERR_CORRECTION_INFO__BYTEMASK);
                            }
                        }
                    }while(0 == (ecc_correction_info & ERR_CORRECTION_INFO__LAST_ERR_INFO));
                }
                else if( FlashReg[intr_status]  & INTR_STATUS0__DMA_CMD_COMP)
                {
                    FlashReg[intr_status] = (INTR_STATUS0__DMA_CMD_COMP);

                    if(ecc_done_OR_dma_comp == 1)
                        break;

                    ecc_done_OR_dma_comp = 1;
                }
                else if( FlashReg[intr_status]  & INTR_STATUS0__ECC_TRANSACTION_DONE )
                {
                    FlashReg[intr_status] = (INTR_STATUS0__ECC_TRANSACTION_DONE);

                    if(ecc_done_OR_dma_comp == 1)
                        break;

                    ecc_done_OR_dma_comp = 1;
                }
            }
            else
            {
                //while( (FlashReg[intr_status] & (INTR_STATUS0__DMA_CMD_COMP|INTR_STATUS0__TIME_OUT))  == 0);
                if (FlashReg[intr_status] & INTR_STATUS0__TIME_OUT)
                {
                    status = FAIL;
                    FlashReg[intr_status] = INTR_STATUS0__TIME_OUT;
                }

                if ( (FlashReg[intr_status] & INTR_STATUS0__DMA_CMD_COMP)  == 0)
				{	status = FAIL;
					break;
                }
            }
            FlashReg[intr_status] = ( (~INTR_STATUS0__ECC_ERR) & (~INTR_STATUS0__ECC_TRANSACTION_DONE)
                    & (~INTR_STATUS0__DMA_CMD_COMP) ) ;

        }
			}
        FlashReg[intr_status] = FlashReg[intr_status];

        FlashReg[DMA_ENABLE] = 0;

        while((FlashReg[DMA_ENABLE] & DMA_ENABLE__FLAG));
#else

        FlashReg[TRANSFER_SPARE_REG] = 0;

        if(GLOB_DeviceInfo.wHWFeatures & FEATURES__INDEX_ADDR)
        {
            FBA_FPA_FSA_P = (volatile uint32*)(FlashMem);
            *FBA_FPA_FSA_P = (uint32 )(MODE_10 | (flash_bank << 24) |
                    ( (flash_add) >> GLOB_DeviceInfo.nBitsInPageDataSize ) );

            FBA_FPA_FSA_P = (volatile uint32*)((uint32)FlashMem | 0x10);
            *FBA_FPA_FSA_P =  0x42;

            FBA_FPA_FSA_P = (volatile uint32*)(FlashMem);
            *FBA_FPA_FSA_P = (uint32 )(MODE_10 | (flash_bank << 24) |
                    ( (flash_add) >> GLOB_DeviceInfo.nBitsInPageDataSize ) );

            FBA_FPA_FSA_P = (volatile uint32*)((uint32)FlashMem | 0x10);

            *FBA_FPA_FSA_P =  0x2000|NumPages;
        }
        else
        {
            FBA_FPA_FSA_P   = (volatile uint32*)((uint32)FlashMem | MODE_10 | (flash_bank << 24) |
                    ( ( (flash_add) >> GLOB_DeviceInfo.nBitsInPageDataSize )   <<2 ) );

            *FBA_FPA_FSA_P = 0x42;

            FBA_FPA_FSA_P   = (volatile uint32*)((uint32)FlashMem | MODE_10 | (flash_bank << 24) |
                    ( ( (flash_add) >> GLOB_DeviceInfo.nBitsInPageDataSize )   <<2 ) );

            *FBA_FPA_FSA_P = 0x2000|NumPages;
        }

        while(NumPages >0)
        {
            if(GLOB_DeviceInfo.wHWFeatures & FEATURES__INDEX_ADDR)
            {
                FBA_FPA_FSA_P = (volatile uint32*)(FlashMem);
                *FBA_FPA_FSA_P = (uint32 )(MODE_01 | (flash_bank << 24) |
                        ( (flash_add + page_num * (GLOB_DeviceInfo.wPageDataSize) ) >> GLOB_DeviceInfo.nBitsInPageDataSize ) );

                FBA_FPA_FSA_P = (volatile uint32*)((uint32)FlashMem | 0x10);
            }
            else
            {
                FBA_FPA_FSA_P   = (volatile uint32*)((uint32)FlashMem | MODE_01 | (flash_bank << 24) |
                       ( ( (flash_add + page_num * (GLOB_DeviceInfo.wPageDataSize) ) >> GLOB_DeviceInfo.nBitsInPageDataSize ) << 2 ) );
            }


            for(sector_count = 0; sector_count < bSectorsPerPage ; sector_count++)
            {
                SectorStart = sector_count * (GLOB_DeviceInfo.wPageDataSize/( 4 * bSectorsPerPage ) );
                SectorEnd = (sector_count + 1) * (GLOB_DeviceInfo.wPageDataSize/( 4 * bSectorsPerPage ) );


                for (i=SectorStart; i<SectorEnd; i++)
                {
                    *(((uint32 *)read_data_l )+ i) = *FBA_FPA_FSA_P;
                }

                if(FlashReg[ECC_ENABLE])
                {
                    if (FlashReg[intr_status] & INTR_STATUS0__ECC_ERR)
                    {
                        FlashReg[intr_status] = INTR_STATUS0__ECC_ERR;

                        do
                        {

                            err_page = FlashReg[err_page_addresses[flash_bank]];

                            err_address = FlashReg[ECC_ERROR_ADDRESS];
                            err_byte = err_address & ECC_ERROR_ADDRESS__OFFSET ;
                            err_sector = ((err_address & ECC_ERROR_ADDRESS__SECTOR_NR)>>12);

                            ecc_correction_info = FlashReg[ERR_CORRECTION_INFO];
                            err_device = ( ( ecc_correction_info & ERR_CORRECTION_INFO__DEVICE_NR) >> 8);
                            if(ecc_correction_info & ERR_CORRECTION_INFO__ERROR_TYPE)
                            {
                                status = FAIL;
                            }
                            else
                            {
                                if (err_byte < ECC_SECTOR_SIZE)
                                {
                                    *((byte *)(read_data + ((err_page - page) *  GLOB_DeviceInfo.wPageDataSize) + err_sector*eccSectorSize + (err_byte *
                                            GLOB_DeviceInfo.wDevicesConnected) + err_device) )^= (ecc_correction_info & ERR_CORRECTION_INFO__BYTEMASK);
                                }
                            }
                        }while(0 == (ecc_correction_info & ERR_CORRECTION_INFO__LAST_ERR_INFO));
                    }
                }
                else
                {
                }
            }

            read_data_l   += PageSize;
            --NumPages;
            page_num++;
        }

        if(FlashReg[ECC_ENABLE])
        {
            while ((FlashReg[intr_status] & (INTR_STATUS0__ECC_TRANSACTION_DONE
                    |INTR_STATUS0__ECC_ERR|INTR_STATUS0__TIME_OUT )) == 0);

            if (FlashReg[intr_status] & INTR_STATUS0__TIME_OUT)
            {
                    status = FAIL;
                    FlashReg[intr_status] = INTR_STATUS0__TIME_OUT;
            }

            else if( FlashReg[intr_status]  & INTR_STATUS0__ECC_ERR )
            {
                FlashReg[intr_status] = INTR_STATUS0__ECC_ERR;


                do
                {
                    err_page = FlashReg[err_page_addresses[flash_bank]];

                    err_address = FlashReg[ECC_ERROR_ADDRESS];
                    err_byte = err_address & ECC_ERROR_ADDRESS__OFFSET ;
                    err_sector = ((err_address & ECC_ERROR_ADDRESS__SECTOR_NR)>>12);

                    ecc_correction_info = FlashReg[ERR_CORRECTION_INFO];
                    err_device = ( ( ecc_correction_info & ERR_CORRECTION_INFO__DEVICE_NR) >> 8);
                    if(ecc_correction_info & ERR_CORRECTION_INFO__ERROR_TYPE)
                    {
                        status = FAIL;
                    }
                    else
                    {
                        if (err_byte < ECC_SECTOR_SIZE)
                        {
                            *((byte *)(read_data + ((err_page - page) *  GLOB_DeviceInfo.wPageDataSize) + err_sector*eccSectorSize + (err_byte *
                                    GLOB_DeviceInfo.wDevicesConnected) + err_device) )^= (ecc_correction_info & ERR_CORRECTION_INFO__BYTEMASK);
                        }
                    }
                }while(0 == (ecc_correction_info & ERR_CORRECTION_INFO__LAST_ERR_INFO));

                while ((FlashReg[intr_status] & (INTR_STATUS0__ECC_TRANSACTION_DONE)) == 0);


                FlashReg[intr_status] = (INTR_STATUS0__ECC_TRANSACTION_DONE);
            }
            else if( FlashReg[intr_status]  & INTR_STATUS0__ECC_TRANSACTION_DONE )
            {
                FlashReg[intr_status] = (INTR_STATUS0__ECC_TRANSACTION_DONE);
            }
        }
        else
        {
        }
#endif
    }
    return status;
}

#endif

#endif
#if FLASH_NAND


uint16  NAND_Write_Page_Main(byte* write_data, BLOCKNODE block,uint16 page,uint16 page_count)
{
    uint32 status = PASS;
    ADDRESSTYPE flash_add;
    volatile uint32 *FBA_FPA_FSA_P;
    uint32 intr_status=0;
    uint32 flash_bank;
    uint32 intr_status_addresses[4] = {INTR_STATUS0, INTR_STATUS1, INTR_STATUS2, INTR_STATUS3};


#if  DDMA
    uint32 intr_status_val;
#else
    uint32  i;
#endif

    byte* write_data_l;

    status = Boundary_Check_Block_Page(block, page, page_count);


    flash_add = ( (ADDRESSTYPE)(block % ( (GLOB_DeviceInfo.wTotalBlocks) / GLOB_totalUsedBanks) ))*GLOB_DeviceInfo.wBlockDataSize+page*GLOB_DeviceInfo.wPageDataSize;

    flash_bank = (block) / (GLOB_DeviceInfo.wTotalBlocks /
                GLOB_totalUsedBanks);

    if(status == PASS)
    {
        intr_status = intr_status_addresses[flash_bank];

        FlashReg[TRANSFER_SPARE_REG] = 0;

        FlashReg[intr_status] = (INTR_STATUS0__PROGRAM_COMP | INTR_STATUS0__PROGRAM_FAIL |INTR_STATUS0__TIME_OUT);

        if(page_count >1)
        {
#ifdef ELDORA
            status = FAIL;
#else
            write_data_l = write_data;

            while (page_count > MAX_PAGES_PER_RW)
            {

                if(FlashReg[MULTIPLANE_OPERATION])
                    status = NAND_Multiplane_Write(write_data_l,block,page,MAX_PAGES_PER_RW);
                else
                    status = NAND_Pipeline_Write_Ahead(write_data_l,block,page,MAX_PAGES_PER_RW);

                if (status == FAIL)
                    return status;

                write_data_l  +=  MAX_PAGES_PER_RW  * GLOB_DeviceInfo.wPageDataSize;
                page_count -= MAX_PAGES_PER_RW;
                page += MAX_PAGES_PER_RW;
            }

            if(FlashReg[MULTIPLANE_OPERATION])
                NAND_Multiplane_Write(write_data_l,block,page,page_count);
            else
                NAND_Pipeline_Write_Ahead(write_data_l,block,page,page_count);
#endif
        }
        else
        {
#if DDMA
        FlashReg[DMA_ENABLE] = 1;

        while(!(FlashReg[DMA_ENABLE] & DMA_ENABLE__FLAG));

        FlashReg[TRANSFER_SPARE_REG] = 0;

        FlashReg[intr_status] =  FlashReg[intr_status];

		/* Fill the int_nand_info structure */
		info.state = INT_WRITE_PAGE_MAIN;
		info.flash_bank = flash_bank;



        if(GLOB_DeviceInfo.wHWFeatures & FEATURES__INDEX_ADDR)
        {

            DDMA_Index_Addr_Transactions(FBA_FPA_FSA_P, write_data, flash_add, flash_bank, WRITE_OP, 1);


        }
        else
        {

            DDMA_Direct_Addr_Transactions(FBA_FPA_FSA_P, write_data, flash_add, flash_bank, WRITE_OP, 1);


        }
		NAND_LLD_Enable_Disable_Interrupts(1);/* Enable interrupt */

		if (!wait_for_completion_timeout(&info.complete, IRQ_WAIT_TIME_OUT))
		{
			print(KERN_DEBUG "Wait for completion timeout blk %d page %d \n", block,page);
			DISABLE_NAND_INTERRUPT_WHEN_ENABLED;
			status = FAIL;
		}
		else
		{
			DISABLE_NAND_INTERRUPT_WHEN_ENABLED;

        while(1)
        {
            while(FlashReg[intr_status] == 0);

            intr_status_val =    FlashReg[intr_status];

            if(FlashReg[intr_status]  & INTR_STATUS0__DMA_CMD_COMP)
            {
                FlashReg[intr_status] = INTR_STATUS0__DMA_CMD_COMP;
                status = PASS;
                break;
            }
            else if (FlashReg[intr_status] & INTR_STATUS0__TIME_OUT)
            {
                status = FAIL;
                FlashReg[intr_status] = INTR_STATUS0__TIME_OUT;
                break;
            }
            else if(FlashReg[intr_status]  & INTR_STATUS0__PROGRAM_FAIL)
            {
                status = FAIL;
                FlashReg[intr_status] &= INTR_STATUS0__PROGRAM_FAIL;
                break;
            }
            else
            {
                FlashReg[intr_status] = intr_status_val;
            }
        }
			}
        FlashReg[intr_status] = FlashReg[intr_status];

        FlashReg[DMA_ENABLE] = 0;

        while((FlashReg[DMA_ENABLE] & DMA_ENABLE__FLAG));

        return status;
#else

        if(GLOB_DeviceInfo.wHWFeatures & FEATURES__INDEX_ADDR)
        {
            FBA_FPA_FSA_P = (volatile uint32*)(FlashMem);
            *FBA_FPA_FSA_P = (uint32 )(MODE_01 | (flash_bank << 24) |
                    ( (flash_add) >> GLOB_DeviceInfo.nBitsInPageDataSize ) );


            FBA_FPA_FSA_P = (volatile uint32*)((uint32)FlashMem | 0x10);

            for (i=0; i<GLOB_DeviceInfo.wPageDataSize/4; i++)
            {
                *FBA_FPA_FSA_P = *(((uint32 *)write_data )+ i);
            }
        }
        else
        {
            FBA_FPA_FSA_P   = (volatile uint32*)((uint32)FlashMem | MODE_01 | (flash_bank << 24) |
                    ( ( (flash_add) >> GLOB_DeviceInfo.nBitsInPageDataSize )   <<2 ) );

            for (i=0; i<GLOB_DeviceInfo.wPageDataSize/4; i++)
            {
                *FBA_FPA_FSA_P = *(((uint32 *)write_data )+ i);
            }
        }

        while ((FlashReg[intr_status] & (INTR_STATUS0__PROGRAM_COMP | INTR_STATUS0__PROGRAM_FAIL|INTR_STATUS0__TIME_OUT )) == 0);

        if (FlashReg[intr_status] & INTR_STATUS0__TIME_OUT)
        {
            status = FAIL;
            FlashReg[intr_status] = INTR_STATUS0__TIME_OUT;
        }
        if (FlashReg[intr_status] & INTR_STATUS0__PROGRAM_FAIL )
            status = FAIL;
        FlashReg[intr_status] = (INTR_STATUS0__PROGRAM_COMP | INTR_STATUS0__PROGRAM_FAIL );

#endif
        }
    }
    return status;
}





#ifndef ELDORA

uint16  NAND_ECC_Ctrl(byte ecc_status,byte correction_span)
{
    if(1 == ecc_status)
        FlashReg[ECC_ENABLE]= 1;



    return PASS;
}

uint32 NAND_Memory_Pool_Size(void)
{
    return (MAX_PAGE_MAINSPARE_AREA * sizeof(byte));
}

int NAND_Mem_Config(byte * pMem)
{
    ALIGN_DWORD_FWD(pMem);
    page_main_spare = pMem;
    return PASS;
}




uint16 NAND_Write_Page_Main_Spare(byte* write_data,BLOCKNODE block,uint16 page,uint16 page_count)
{
    uint32 status = PASS;
    uint32 i,j,page_num=0;
    volatile uint32 *FBA_FPA_FSA_P;
    uint32 PageSize  = GLOB_DeviceInfo.wPageSize;
    uint32 PageDataSize  = GLOB_DeviceInfo.wPageDataSize;
    uint32 eccBytes = GLOB_DeviceInfo.wECCBytesPerSector;
    uint32 spareFlagBytes  = GLOB_DeviceInfo.wNumPageSpareFlag;
    uint32 spareSkipBytes  = GLOB_DeviceInfo.wSpareSkipBytes;

    ADDRESSTYPE flash_add;
    uint32  eccSectorSize;
    uint32  flash_bank;
    uint32  intr_status=0;
    uint32 intr_status_addresses[4] = {INTR_STATUS0, INTR_STATUS1, INTR_STATUS2, INTR_STATUS3};


    eccSectorSize   = ECC_SECTOR_SIZE * (GLOB_DeviceInfo.wDevicesConnected);


    status = Boundary_Check_Block_Page(block, page, page_count);

    flash_bank = (block) / (GLOB_DeviceInfo.wTotalBlocks /
                GLOB_totalUsedBanks);

    if(status == PASS)
    {
        intr_status = intr_status_addresses[flash_bank];

        FlashReg[TRANSFER_SPARE_REG] = 1;

        while((status != FAIL) && (page_count > 0))
        {
            flash_add = ( (ADDRESSTYPE)(block % ( (GLOB_DeviceInfo.wTotalBlocks) / GLOB_totalUsedBanks) ))*GLOB_DeviceInfo.wBlockDataSize+page*GLOB_DeviceInfo.wPageDataSize;

            FlashReg[intr_status] =  FlashReg[intr_status];


            if(GLOB_DeviceInfo.wHWFeatures & FEATURES__INDEX_ADDR)
            {
                FBA_FPA_FSA_P = (volatile uint32*)(FlashMem);
                *FBA_FPA_FSA_P = (uint32 )(MODE_01 | (flash_bank << 24) |
                        ( (flash_add) >> GLOB_DeviceInfo.nBitsInPageDataSize ) );

                FBA_FPA_FSA_P = (volatile uint32*)((uint32)FlashMem | 0x10);
            }
            else
            {
                FBA_FPA_FSA_P   = (volatile uint32*)((uint32)FlashMem | MODE_01 | (flash_bank << 24) |
                        ( ( (flash_add) >> GLOB_DeviceInfo.nBitsInPageDataSize )   <<2 ) );
            }

            if(FlashReg[ECC_ENABLE])
            {


                for (j=0; j<GLOB_DeviceInfo.wPageDataSize / eccSectorSize; j++)
                {
                    for (i=0; i<eccSectorSize; i++)
                        page_main_spare[(eccSectorSize+eccBytes)*j + i] =  write_data[eccSectorSize *j + i];

                    for (i=0; i<eccBytes; i++)
                        page_main_spare[(eccSectorSize+eccBytes)*j + eccSectorSize+i] =  write_data[PageDataSize+spareFlagBytes + eccBytes*j + i];
                }

                for (i=0; i<spareFlagBytes; i++)
                    page_main_spare[(eccSectorSize+eccBytes)*j +i] =  write_data[PageDataSize+i];

                for (i=PageSize - 1; i >= PageDataSize + spareSkipBytes; i--)
                    page_main_spare[i] = page_main_spare[i - spareSkipBytes];

                for (i=PageDataSize; i < PageDataSize + spareSkipBytes; i++)
                    page_main_spare[i] = 0xff;

		//Fix up, since spare area size of some chips is not a multiple of 4, the controller will hang here in that case
                for (i=0; i<(PageSize + 3)/4; i++)
                    *FBA_FPA_FSA_P = *(((uint32 *)page_main_spare )+ i);
            }
            else
            {

                for (i=0; i<(PageSize + 3)/4; i++)
                    *FBA_FPA_FSA_P = *(((uint32 *)write_data )+ i);
            }



            while ((FlashReg[intr_status] & (INTR_STATUS0__PROGRAM_COMP | INTR_STATUS0__PROGRAM_FAIL|INTR_STATUS0__TIME_OUT )) == 0);

            if (FlashReg[intr_status] & (INTR_STATUS0__PROGRAM_FAIL|INTR_STATUS0__TIME_OUT) )
                status = FAIL;


            FlashReg[intr_status] =  FlashReg[intr_status];

            page_num++;
            page_count--;
            write_data +=PageSize;
        }

        FlashReg[TRANSFER_SPARE_REG] = 0;
    }

    return status;
}







uint16 NAND_Read_Page_Main_Spare(byte* read_data,BLOCKNODE block,uint16 page,uint16 page_count)
{
    uint32 status = PASS;
    uint32 i, j;
    volatile uint32 *FBA_FPA_FSA_P;
    ADDRESSTYPE flash_add=0;
    uint32 PageSize         = GLOB_DeviceInfo.wPageSize;
    uint32 PageDataSize     = GLOB_DeviceInfo.wPageDataSize;
    uint32 PageSpareSize    = GLOB_DeviceInfo.wPageSpareSize;
    uint16 err_byte;
    byte    err_device;
    byte  err_sector;
    uint16 err_page=0;
    uint16 ecc_correction_info;
    uint16 err_address;
    uint32 eccBytes = GLOB_DeviceInfo.wECCBytesPerSector;
    uint32 spareFlagBytes  = GLOB_DeviceInfo.wNumPageSpareFlag;
    uint32 spareSkipBytes  = GLOB_DeviceInfo.wSpareSkipBytes;
    uint32  eccSectorSize;
    uint32  flash_bank;
    uint32  intr_status=0;
    uint32 intr_status_addresses[4] = {INTR_STATUS0, INTR_STATUS1, INTR_STATUS2, INTR_STATUS3};
    uint32 err_page_addresses[4] = {ERR_PAGE_ADDR0, ERR_PAGE_ADDR1, ERR_PAGE_ADDR2, ERR_PAGE_ADDR3};

    byte *read_data_l = read_data;

    eccSectorSize   = ECC_SECTOR_SIZE * (GLOB_DeviceInfo.wDevicesConnected);


    status = Boundary_Check_Block_Page(block, page, page_count);


    flash_bank = (block) / (GLOB_DeviceInfo.wTotalBlocks /
                GLOB_totalUsedBanks);

    if(status == PASS)
    {
        intr_status = intr_status_addresses[flash_bank];

        FlashReg[TRANSFER_SPARE_REG] = 1;

        FlashReg[intr_status] =  FlashReg[intr_status];

        while((status != FAIL) && (page_count >0))
        {
            flash_add = ( (ADDRESSTYPE)(block % ( (GLOB_DeviceInfo.wTotalBlocks) / GLOB_totalUsedBanks) ))*GLOB_DeviceInfo.wBlockDataSize+page*GLOB_DeviceInfo.wPageDataSize;

            if(GLOB_DeviceInfo.wHWFeatures & FEATURES__INDEX_ADDR)
            {
                Index_Addr(FBA_FPA_FSA_P, (uint32 )(MODE_10 | (flash_bank << 24) |
                        ( (flash_add) >> GLOB_DeviceInfo.nBitsInPageDataSize ) ), 0x43);

                Index_Addr(FBA_FPA_FSA_P, (uint32 )(MODE_10 | (flash_bank << 24) |
                        ( (flash_add) >> GLOB_DeviceInfo.nBitsInPageDataSize ) ), 0x2000|page_count);

                while ((FlashReg[intr_status] & (INTR_STATUS0__LOAD_COMP) ) == 0);


                FBA_FPA_FSA_P   = (volatile uint32*)(FlashMem);
                *FBA_FPA_FSA_P =  (MODE_01 | (flash_bank << 24) |
                        ( (flash_add) >> GLOB_DeviceInfo.nBitsInPageDataSize ) );

                FBA_FPA_FSA_P = (volatile uint32*)((uint32)FlashMem | 0x10);
            }
            else
            {
                FBA_FPA_FSA_P   = (volatile uint32*)((uint32)FlashMem | MODE_10 | (flash_bank << 24) |
                        ( ( (flash_add) >> GLOB_DeviceInfo.nBitsInPageDataSize )   <<2 ) );
                *FBA_FPA_FSA_P = 0x43;


                FBA_FPA_FSA_P   = (volatile uint32*)((uint32)FlashMem | MODE_10 | (flash_bank << 24) |
                        ( ( (flash_add) >> GLOB_DeviceInfo.nBitsInPageDataSize )   <<2 ) );
                *FBA_FPA_FSA_P = 0x2000|page_count;

                while ((FlashReg[intr_status] & (INTR_STATUS0__LOAD_COMP) ) == 0);

                FBA_FPA_FSA_P   = (volatile uint32*)((uint32)FlashMem | MODE_01 | (flash_bank << 24) |
                        ( ( (flash_add) >> GLOB_DeviceInfo.nBitsInPageDataSize )   <<2 ) );

            }
		//Fix up, since spare area size of some chips is not a multiple of 4, the controller will hang here in that case

            for (i=0; i<(PageSize + 3)/4; i++)
            {
                *(((uint32 *)page_main_spare )+ i) = *FBA_FPA_FSA_P;
            }


            if(FlashReg[ECC_ENABLE] && (access_type != READ_PAGE_MAIN_SPARE_RAW))
            {



                for (i=PageDataSize;  i < PageSize - spareSkipBytes; i++)
                    page_main_spare[i] = page_main_spare[i + spareSkipBytes];


                for (j=0; j<GLOB_DeviceInfo.wPageDataSize / eccSectorSize; j++)
                {

                    for (i=0; i<eccSectorSize; i++)
                        read_data_l[eccSectorSize *j + i] = page_main_spare[(eccSectorSize+eccBytes)*j + i];

                    for (i=0; i<eccBytes; i++)
                        read_data_l[PageDataSize+spareFlagBytes + eccBytes*j + i] = page_main_spare[(eccSectorSize+eccBytes)*j + eccSectorSize+i];
                }

                for (i=0; i<spareFlagBytes; i++)
                    read_data_l[PageDataSize+i] =  page_main_spare[(eccSectorSize+eccBytes)*j +i];
            }
            else
            {
                for (i=0; i<(PageDataSize+PageSpareSize); i++)
                    read_data_l[i] =  page_main_spare[i];
            }

            if(FlashReg[ECC_ENABLE])
            {
                while ((FlashReg[intr_status] & (INTR_STATUS0__ECC_TRANSACTION_DONE
                        |INTR_STATUS0__ECC_ERR|INTR_STATUS0__TIME_OUT )) == 0);

                if (FlashReg[intr_status] & INTR_STATUS0__TIME_OUT)
                {
                    status = FAIL;
                    FlashReg[intr_status] = INTR_STATUS0__TIME_OUT;
                }

                else if (FlashReg[intr_status] & INTR_STATUS0__ECC_ERR)
                {

                    FlashReg[intr_status] = INTR_STATUS0__ECC_ERR;
                    do
                    {
                        err_page = FlashReg[err_page_addresses[flash_bank]];

                        err_address = FlashReg[ECC_ERROR_ADDRESS];
                        err_byte = err_address & ECC_ERROR_ADDRESS__OFFSET ;
                        err_sector = ((err_address & ECC_ERROR_ADDRESS__SECTOR_NR)>>12);

                        ecc_correction_info = FlashReg[ERR_CORRECTION_INFO];
                        err_device = ( ( ecc_correction_info & ERR_CORRECTION_INFO__DEVICE_NR) >> 8);
                        if(ecc_correction_info & ERR_CORRECTION_INFO__ERROR_TYPE)
                        {
                            status = FAIL;
                        }
                        else
                        {
                            if (err_byte < ECC_SECTOR_SIZE)
                            {
                                *((byte *)(read_data + ((err_page -page) *  GLOB_DeviceInfo.wPageDataSize) + err_sector*eccSectorSize + (err_byte *
                                        GLOB_DeviceInfo.wDevicesConnected) + err_device) )^= (ecc_correction_info & ERR_CORRECTION_INFO__BYTEMASK);
                            }
                        }
                    }while(0 == (ecc_correction_info & ERR_CORRECTION_INFO__LAST_ERR_INFO));
                }

                if( FlashReg[intr_status]  & INTR_STATUS0__ECC_TRANSACTION_DONE & INTR_STATUS0__ECC_ERR )
                {
                    FlashReg[intr_status] = (INTR_STATUS0__ECC_TRANSACTION_DONE |INTR_STATUS0__ECC_ERR );
                }
                else if( FlashReg[intr_status]  & INTR_STATUS0__ECC_TRANSACTION_DONE )
                {
                    FlashReg[intr_status] = INTR_STATUS0__ECC_TRANSACTION_DONE;
                }
                else if( FlashReg[intr_status]  & INTR_STATUS0__ECC_ERR )
                {
                    FlashReg[intr_status] = INTR_STATUS0__ECC_ERR;
                }
            }
            else
            {
            }

            page++;
            page_count--;
            read_data_l +=PageSize;
        }
    }

    FlashReg[TRANSFER_SPARE_REG] = 0;

    if(GLOB_DeviceInfo.wHWFeatures & FEATURES__INDEX_ADDR)
    {
        Index_Addr(FBA_FPA_FSA_P, (uint32 )(MODE_10 | (flash_bank << 24) |
                     ( (flash_add) >> GLOB_DeviceInfo.nBitsInPageDataSize ) ), 0x42);
    }
    else
    {
        FBA_FPA_FSA_P   = (volatile uint32*)((uint32)FlashMem | MODE_10 | (flash_bank << 24) |
                ( ( (flash_add) >> GLOB_DeviceInfo.nBitsInPageDataSize )   <<2 ) );
        *FBA_FPA_FSA_P = 0x42;
    }

    return status;
}


uint16 NAND_Read_Page_Main_Spare_Raw(byte* read_data,BLOCKNODE block,uint16 page,uint16 page_count)
{
    uint32 status = PASS;
	access_type = READ_PAGE_MAIN_SPARE_RAW;
	status = NAND_Read_Page_Main_Spare(read_data,block,page,page_count);
	access_type = FS_ACCESS;
	return status;
}




uint16  NAND_Pipeline_Write_Ahead(byte* write_data,BLOCKNODE block,uint16 page,uint16 page_count)
{
    uint16 status = PASS;
    uint16 status2 = PASS;
    uint32 NumPages=page_count;
    volatile uint32 *FBA_FPA_FSA_P;
    uint32 PageSize  = GLOB_DeviceInfo.wPageDataSize;
    ADDRESSTYPE flash_add;

    uint32  flash_bank;
    uint32  intr_status=0;
    uint32 intr_status_addresses[4] = {INTR_STATUS0, INTR_STATUS1, INTR_STATUS2, INTR_STATUS3};
    uint32 err_page_addresses[4] = {ERR_PAGE_ADDR0, ERR_PAGE_ADDR1, ERR_PAGE_ADDR2, ERR_PAGE_ADDR3};



#if DDMA
#else
    uint32 i,page_num=0;
#endif

    status = Boundary_Check_Block_Page(block, page, page_count);


    if(page_count < 2)
    {
        status = FAIL;
    }

    flash_add = ( (ADDRESSTYPE)(block % ( (GLOB_DeviceInfo.wTotalBlocks) / GLOB_totalUsedBanks) ))*GLOB_DeviceInfo.wBlockDataSize+page*GLOB_DeviceInfo.wPageDataSize;

    flash_bank = (block) / (GLOB_DeviceInfo.wTotalBlocks /
                GLOB_totalUsedBanks);

    if(status == PASS)
    {
        intr_status = intr_status_addresses[flash_bank];

        FlashReg[intr_status] =  FlashReg[intr_status];

#if DDMA
        FlashReg[DMA_ENABLE] = 1;

        while(!(FlashReg[DMA_ENABLE] & DMA_ENABLE__FLAG));

        FlashReg[TRANSFER_SPARE_REG] = 0;

		/* Fill the int_nand_info structure */
		info.state = INT_PIPELINE_WRITE_AHEAD;
		info.flash_bank = flash_bank;

        if(GLOB_DeviceInfo.wHWFeatures & FEATURES__INDEX_ADDR)
        {

            FBA_FPA_FSA_P = (volatile uint32*)(FlashMem);
            *FBA_FPA_FSA_P = (uint32 )(MODE_10 | (flash_bank << 24) |
                    ( (flash_add) >> GLOB_DeviceInfo.nBitsInPageDataSize ) );

            FBA_FPA_FSA_P = (volatile uint32*)((uint32)FlashMem | 0x10);
            *FBA_FPA_FSA_P =  0x42;

            DDMA_Index_Addr_Transactions(FBA_FPA_FSA_P, write_data, flash_add, flash_bank, WRITE_OP, NumPages);



        }
        else
        {
            FBA_FPA_FSA_P   = (volatile uint32*)((uint32)FlashMem | MODE_10 | (flash_bank << 24) |
                    ( ( (flash_add) >> GLOB_DeviceInfo.nBitsInPageDataSize )   <<2 ) );

            *FBA_FPA_FSA_P = 0x42;

            DDMA_Direct_Addr_Transactions(FBA_FPA_FSA_P, write_data, flash_add, flash_bank, WRITE_OP, NumPages);
        }

		NAND_LLD_Enable_Disable_Interrupts(1);/* Enable interrupt */

		if (!wait_for_completion_timeout(&info.complete, IRQ_WAIT_TIME_OUT))
		{
			print(KERN_DEBUG "Wait for completion timeout blk %d page %d \n", block,page);
			DISABLE_NAND_INTERRUPT_WHEN_ENABLED;
			status = FAIL;
		}
		else
		{
		DISABLE_NAND_INTERRUPT_WHEN_ENABLED;
        while(1)
        {
            while(FlashReg[intr_status] == 0);

            if(FlashReg[intr_status]  & INTR_STATUS0__DMA_CMD_COMP)
            {
                FlashReg[intr_status] = INTR_STATUS0__DMA_CMD_COMP;
                status = PASS;

                if(status2 == FAIL)
                    status = FAIL;

                break;
            }
            else if(FlashReg[intr_status]  & (INTR_STATUS0__PROGRAM_FAIL|INTR_STATUS0__TIME_OUT))
            {
                status2 = FAIL;
                status = FAIL;
                FlashReg[intr_status] &= (INTR_STATUS0__PROGRAM_FAIL|INTR_STATUS0__TIME_OUT);
            }
            else
            {
                FlashReg[intr_status] = (~INTR_STATUS0__PROGRAM_FAIL) & (~INTR_STATUS0__DMA_CMD_COMP);
            }
        }
			}
        FlashReg[intr_status] = FlashReg[intr_status];

        FlashReg[DMA_ENABLE] = 0;

        while((FlashReg[DMA_ENABLE] & DMA_ENABLE__FLAG));

        return status;

#else
        FlashReg[TRANSFER_SPARE_REG] = 0;

        if(GLOB_DeviceInfo.wHWFeatures & FEATURES__INDEX_ADDR)
        {
            FBA_FPA_FSA_P = (volatile uint32*)(FlashMem);
            *FBA_FPA_FSA_P = (uint32 )(MODE_10 | (flash_bank << 24) |
                    ( (flash_add) >> GLOB_DeviceInfo.nBitsInPageDataSize ) );

            FBA_FPA_FSA_P = (volatile uint32*)((uint32)FlashMem | 0x10);
            *FBA_FPA_FSA_P =  0x42;

            FBA_FPA_FSA_P = (volatile uint32*)(FlashMem);
            *FBA_FPA_FSA_P = (uint32 )(MODE_10 | (flash_bank << 24) |
                    ( (flash_add) >> GLOB_DeviceInfo.nBitsInPageDataSize ) );

            FBA_FPA_FSA_P = (volatile uint32*)((uint32)FlashMem | 0x10);
            *FBA_FPA_FSA_P = 0x2100|NumPages;

        }
        else
        {
            FBA_FPA_FSA_P   = (volatile uint32*)((uint32)FlashMem | MODE_10 | (flash_bank << 24) |
                    ( ( (flash_add) >> GLOB_DeviceInfo.nBitsInPageDataSize )   <<2 ) );

            *FBA_FPA_FSA_P =0x42;

            FBA_FPA_FSA_P   = (volatile uint32*)((uint32)FlashMem | MODE_10 | (flash_bank << 24) |
                    ( ( (flash_add) >> GLOB_DeviceInfo.nBitsInPageDataSize )   <<2 ) );

            *FBA_FPA_FSA_P = 0x2100|NumPages;
        }

        while(NumPages >0)
        {
            if(GLOB_DeviceInfo.wHWFeatures & FEATURES__INDEX_ADDR)
            {
                FBA_FPA_FSA_P = (volatile uint32*)(FlashMem);
                *FBA_FPA_FSA_P = (uint32 )(MODE_01 | (flash_bank << 24) |
                        ( (flash_add + page_num * (GLOB_DeviceInfo.wPageDataSize) ) >> GLOB_DeviceInfo.nBitsInPageDataSize ) );

                FBA_FPA_FSA_P = (volatile uint32*)((uint32)FlashMem | 0x10);
            }
            else
            {
                FBA_FPA_FSA_P   = (volatile uint32*)((uint32)FlashMem | MODE_01 | (flash_bank << 24) |
                        ( ( (flash_add + page_num * (GLOB_DeviceInfo.wPageDataSize) ) >> GLOB_DeviceInfo.nBitsInPageDataSize ) << 2 ) );
            }



            for (i=0; i<GLOB_DeviceInfo.wPageDataSize/4; i++)
            {
                *FBA_FPA_FSA_P = *(((uint32 *)write_data )+ i);
            }

            while ((FlashReg[intr_status] & INTR_STATUS0__INT_ACT) == 0);

            FlashReg[intr_status] = INTR_STATUS0__INT_ACT;


            write_data += PageSize;
            --NumPages;
            page_num++;
        }

        while ((FlashReg[intr_status] & (INTR_STATUS0__PROGRAM_COMP | INTR_STATUS0__PROGRAM_FAIL|INTR_STATUS0__TIME_OUT )) == 0);


        if (FlashReg[intr_status] & (INTR_STATUS0__PROGRAM_FAIL|INTR_STATUS0__TIME_OUT) )
            status = FAIL;

        FlashReg[intr_status] = (INTR_STATUS0__PROGRAM_COMP | INTR_STATUS0__PROGRAM_FAIL|INTR_STATUS0__TIME_OUT );

#endif
    }
    return status;
}





uint16  NAND_Multiplane_Write(byte* write_data,BLOCKNODE block,uint16 page,uint16 page_count)
{
    uint16 status = PASS;
    uint16 status2 = PASS;
    uint32 NumPages=page_count;
    volatile uint32 *FBA_FPA_FSA_P;
    uint32 PageSize  = GLOB_DeviceInfo.wPageDataSize;
    ADDRESSTYPE flash_add;

    uint32  flash_bank;
    uint32  intr_status=0;
    uint32 intr_status_addresses[4] = {INTR_STATUS0, INTR_STATUS1, INTR_STATUS2, INTR_STATUS3};


#if DDMA
#else
    uint32 i,page_num=0;
    uint32 plane =0;
#endif

    status = Boundary_Check_Block_Page(block, page, page_count);

    flash_add = ( (ADDRESSTYPE)(block % ( (GLOB_DeviceInfo.wTotalBlocks) / GLOB_totalUsedBanks) ))*GLOB_DeviceInfo.wBlockDataSize+page*GLOB_DeviceInfo.wPageDataSize;

    flash_bank = (block) / (GLOB_DeviceInfo.wTotalBlocks /
                GLOB_totalUsedBanks);

    if(status == PASS)
    {
        intr_status = intr_status_addresses[flash_bank];

        FlashReg[intr_status] =  FlashReg[intr_status];

        FlashReg[TRANSFER_SPARE_REG] = 0;

        FlashReg[MULTIPLANE_OPERATION ]     = 0x0001;

#if DDMA
        FlashReg[DMA_ENABLE] = 1;

        while(!(FlashReg[DMA_ENABLE] & DMA_ENABLE__FLAG));

        FlashReg[TRANSFER_SPARE_REG] = 0;

        if(GLOB_DeviceInfo.wHWFeatures & FEATURES__INDEX_ADDR)
        {

            FBA_FPA_FSA_P = (volatile uint32*)(FlashMem);
            *FBA_FPA_FSA_P = (uint32 )(MODE_10 | (flash_bank << 24) |
                    ( (flash_add) >> GLOB_DeviceInfo.nBitsInPageDataSize ) );

            FBA_FPA_FSA_P = (volatile uint32*)((uint32)FlashMem | 0x10);
            *FBA_FPA_FSA_P =  0x42;

            FBA_FPA_FSA_P = (volatile uint32*)(FlashMem);
            *FBA_FPA_FSA_P = (uint32 )(MODE_10 | (flash_bank << 24) |
                    ( (flash_add) >> GLOB_DeviceInfo.nBitsInPageDataSize ) );

            FBA_FPA_FSA_P = (volatile uint32*)((uint32)FlashMem | 0x10);
            *FBA_FPA_FSA_P =  0x42;

           DDMA_Index_Addr_Transactions(FBA_FPA_FSA_P, write_data, flash_add, flash_bank, WRITE_OP, NumPages);
        }
        else
        {
            FBA_FPA_FSA_P   = (volatile uint32*)((uint32)FlashMem | MODE_10 | (flash_bank << 24) |
                    ( ( (flash_add) >> GLOB_DeviceInfo.nBitsInPageDataSize )   <<2 ) );

            *FBA_FPA_FSA_P = 0x42;

            DDMA_Index_Addr_Transactions(FBA_FPA_FSA_P, write_data, flash_add, flash_bank, READ_OP, NumPages);
        }

        while(1)
        {
            while(FlashReg[intr_status] == 0);

            if(FlashReg[intr_status]  & INTR_STATUS0__DMA_CMD_COMP)
            {
                FlashReg[intr_status] = INTR_STATUS0__DMA_CMD_COMP;
                status = PASS;
                if (status2 == FAIL)
                    status = FAIL;
                break;
            }
            else if(FlashReg[intr_status]  & (INTR_STATUS0__PROGRAM_FAIL|INTR_STATUS0__TIME_OUT))
            {
                status2 = FAIL;
                status = FAIL;
                FlashReg[intr_status] &= (INTR_STATUS0__PROGRAM_FAIL|INTR_STATUS0__TIME_OUT);
            }
            else
            {
                FlashReg[intr_status] = (~INTR_STATUS0__PROGRAM_FAIL) & (~INTR_STATUS0__DMA_CMD_COMP);
            }
        }

        FlashReg[intr_status] = FlashReg[intr_status];

        FlashReg[DMA_ENABLE] = 0;

        while((FlashReg[DMA_ENABLE] & DMA_ENABLE__FLAG));

        FlashReg[MULTIPLANE_OPERATION ]     = 0x0000;
#else

        FlashReg[TRANSFER_SPARE_REG] = 0;

        if(GLOB_DeviceInfo.wHWFeatures & FEATURES__INDEX_ADDR)
        {
            FBA_FPA_FSA_P = (volatile uint32*)(FlashMem);
            *FBA_FPA_FSA_P = (uint32 )(MODE_10 | (flash_bank << 24) |
                    ( (flash_add) >> GLOB_DeviceInfo.nBitsInPageDataSize ) );

            FBA_FPA_FSA_P = (volatile uint32*)((uint32)FlashMem | 0x10);
            *FBA_FPA_FSA_P =  0x42;

            FBA_FPA_FSA_P = (volatile uint32*)(FlashMem);
            *FBA_FPA_FSA_P = (uint32 )(MODE_10 | (flash_bank << 24) |
                    ( (flash_add) >> GLOB_DeviceInfo.nBitsInPageDataSize ) );

            FBA_FPA_FSA_P = (volatile uint32*)((uint32)FlashMem | 0x10);
            *FBA_FPA_FSA_P = 0x2100|NumPages;

        }
        else
        {
            FBA_FPA_FSA_P   = (volatile uint32*)((uint32)FlashMem | MODE_10 | (flash_bank << 24) |
                    ( ( (flash_add) >> GLOB_DeviceInfo.nBitsInPageDataSize )   <<2 ) );

            *FBA_FPA_FSA_P =0x42;

            FBA_FPA_FSA_P   = (volatile uint32*)((uint32)FlashMem | MODE_10 | (flash_bank << 24) |
                    ( ( (flash_add) >> GLOB_DeviceInfo.nBitsInPageDataSize )   <<2 ) );

            *FBA_FPA_FSA_P = 0x2100|NumPages;
        }

        while(NumPages >0)
        {
            if(plane==0)
            {
                if(GLOB_DeviceInfo.wHWFeatures & FEATURES__INDEX_ADDR)
                {
                    FBA_FPA_FSA_P = (volatile uint32*)(FlashMem);
                    *FBA_FPA_FSA_P = (uint32 )(MODE_01 | (flash_bank << 24) |
                            ( (flash_add + page_num * (GLOB_DeviceInfo.wPageDataSize) ) >> GLOB_DeviceInfo.nBitsInPageDataSize ) );

                    FBA_FPA_FSA_P = (volatile uint32*)((uint32)FlashMem | 0x10);

                }
                else
                {
                    FBA_FPA_FSA_P   = (volatile uint32*)((uint32)FlashMem | MODE_01 | (flash_bank << 24) |
                            ( ( (flash_add + page_num * (GLOB_DeviceInfo.wPageDataSize) ) >> GLOB_DeviceInfo.nBitsInPageDataSize ) << 2 ) );
                }

                plane=1;
            }
            else
            {
                if(GLOB_DeviceInfo.wHWFeatures & FEATURES__INDEX_ADDR)

                {
                    FBA_FPA_FSA_P = (volatile uint32*)(FlashMem);
                    *FBA_FPA_FSA_P = (uint32 )(MODE_01 | (flash_bank << 24) |
                            ( ( (flash_add+GLOB_DeviceInfo.wBlockDataSize) + page_num * (GLOB_DeviceInfo.wPageDataSize) ) >> GLOB_DeviceInfo.nBitsInPageDataSize )  );

                    FBA_FPA_FSA_P = (volatile uint32*)((uint32)FlashMem | 0x10);

                }
                else
                {
                    FBA_FPA_FSA_P   = (volatile uint32*)((uint32)FlashMem | MODE_01 | (flash_bank << 24) |
                            ( ( ( (flash_add+GLOB_DeviceInfo.wBlockDataSize) + page_num * (GLOB_DeviceInfo.wPageDataSize) ) >> GLOB_DeviceInfo.nBitsInPageDataSize ) << 2 )  );
                }

                plane=0;
            }


            for (i=0; i<GLOB_DeviceInfo.wPageDataSize/4; i++)
                *FBA_FPA_FSA_P = *(((uint32 *)write_data )+ i);

            write_data += PageSize;

            if(plane ==0)
                page_num++;

            --NumPages;
        }

        while ((FlashReg[intr_status] & (INTR_STATUS0__PROGRAM_COMP | INTR_STATUS0__PROGRAM_FAIL|INTR_STATUS0__TIME_OUT )) == 0);

        if (FlashReg[intr_status] & (INTR_STATUS0__PROGRAM_FAIL|INTR_STATUS0__TIME_OUT) )
            status = FAIL;
        FlashReg[intr_status] = (INTR_STATUS0__PROGRAM_COMP | INTR_STATUS0__PROGRAM_FAIL|INTR_STATUS0__TIME_OUT );

        FlashReg[MULTIPLANE_OPERATION ]     = 0x0000;

#endif

    }

    return status;
}
#if DDMA
uint16 NAND_Flash_Release(void)
{
	NAND_DDMA_IRQ_Release();//Free IRQ
    GLOB_MEMUNMAP_NOCACHE(FlashReg);
    GLOB_MEMUNMAP_NOCACHE(FlashMem);
	return PASS;
}
#endif
#endif


uint16 NAND_LLD_Event_Status(void)
{
    return PASS;
}

#endif
