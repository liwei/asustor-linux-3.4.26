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


#include <linux/pci.h>
#include "ffsport.h"
#include "flash.h"
#include "lld_nand.h"
#include "NAND_Regs_4.h"
#include "lld.h"
#include "flash_bbt.h"



#define FAKE_INTERRUPTS  1 /* i.e. test the interrupt support of sbd without
                             actually using interrupts of lld_nand (use lld_emu)
							 */
#define USEMEMCPY    0
#define SBDBG        0

#ifndef LINUX_DEVICE_DRIVER
#include <stdio.h>
#include <stdlib.h>
#else
#include <linux/interrupt.h>
#include <linux/wait.h>
#endif

#ifdef __cplusplus
extern "C" {
#endif

#if defined(OSIA_CFG) && defined(XCODE)
#define    NULL    0
#endif




/*&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
* Function:     GLOB_Calc_Used_Bits
* Inputs:       Power of 2 number
* Outputs:      Number of Used Bits
*               0, if the argument is 0
* Description:  Calculate the number of bits used by a given power of 2 number
*               Number can be upto 32 bit
*&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&*/
int GLOB_Calc_Used_Bits(unsigned int n)
{
    int tot_bits = 0;

    if (n >= 1<<16)
    {
        n >>= 16;
        tot_bits += 16;
    }

    if (n >= 1<< 8)
    {
        n >>=  8;
        tot_bits +=  8;
    }

    if (n >= 1<< 4)
    {
        n >>=  4;
        tot_bits +=  4;
    }

    if (n >= 1<< 2)
    {
        n >>=  2;
        tot_bits +=  2;
    }

    if (n >= 1<< 1)
        tot_bits +=  1;

    return ((n == 0) ? (0) : tot_bits);
}

/*&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
* Function:     GLOB_u64_Div
* Inputs:       Number of ADDRESSTYPE
*               A power of 2 number as Division
* Outputs:      Quotient of the Divisor operation
* Description:  It divides the address by divisor by using bit shift operation
*               (essentially without explicitely using "/").
*               Divisor is a power of 2 number and Divided is of ADDRESSTYPE
*&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&*/
ADDRESSTYPE GLOB_u64_Div(ADDRESSTYPE addr, uint32 divisor)
{
    return  (ADDRESSTYPE)(addr >> GLOB_Calc_Used_Bits(divisor));
}

/*&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
* Function:     GLOB_u64_Remainder
* Inputs:       Number of ADDRESSTYPE
*               Divisor Type (1 -PageAddress, 2- BlockAddress)
* Outputs:      Remainder of the Division operation
* Description:  It calculates the remainder of a number (of ADDRESSTYPE) by
*               divisor(power of 2 number ) by using bit shifting and multiply
*               operation(essentially without explicitely using "/").
*&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&*/
ADDRESSTYPE GLOB_u64_Remainder(ADDRESSTYPE addr, uint32 divisor_type)
{
    ADDRESSTYPE result =0;
    if (divisor_type == 1)
    {
        result = (addr >> GLOB_DeviceInfo.nBitsInPageDataSize);
        result = result * GLOB_DeviceInfo.wPageDataSize;
    }
    else if (divisor_type == 2)
    {
        result = (addr >> GLOB_DeviceInfo.nBitsInBlockDataSize);
        result = result * GLOB_DeviceInfo.wBlockDataSize;
    }
    result = addr - result;
    return result;
}


#ifdef _SMX

FFS_LOCK_HANDLE GLOB_CREATE_FFS_API_LOCK()
{
    return create_sema(1);
}

void GLOB_RELEASE_FFS_API_LOCK(FFS_LOCK_HANDLE handle)
{
    delete_sem(handle);
}

int GLOB_FFS_API_ENTER(FFS_LOCK_HANDLE handle)
{
    return test(handle, INF) ? PASS : FAIL;
}

int GLOB_FFS_API_EXIT(FFS_LOCK_HANDLE handle)
{
    return signalx(handle) ? PASS : FAIL;
}

int GLOB_SYSTEM_FATAL_ERROR(void)
{
    return 0;
}

void *GLOB_MALLOC(ADDRESSTYPE size)
{
    return malloc(size);
}

void GLOB_FREE(void *ptr)
{
    free(ptr);
}

void GLOB_SCHEDULE_FUNCTION(ISR_FN_T fn)
{
    fn(0, 0);
}

#if CMD_DMA
int GLOB_ISR(int dummy1, void *dummy2)
{
  return 0;
}
#endif

uint32 *GLOB_MEMMAP_NOCACHE(unsigned long addr, unsigned long size)
{
    return (uint32 *)addr;
}

uint32 *GLOB_MEMMAP_TOBUS(uint32 *ptr)
{
    return ptr;
}
#elif defined WIN32
FFS_LOCK_HANDLE GLOB_CREATE_FFS_API_LOCK()
{
    return CreateSemaphore(NULL, 0, 1, "FFS_Sema");
}

void GLOB_RELEASE_FFS_API_LOCK(FFS_LOCK_HANDLE handle)
{
    CloseHandle(handle);
    return;
}

int GLOB_FFS_API_ENTER(FFS_LOCK_HANDLE handle)
{
    WaitForSingleObject(handle, INFINITE);
    return PASS;
}

int GLOB_FFS_API_EXIT(FFS_LOCK_HANDLE handle)
{
    ReleaseSemaphore(handle, 1, NULL);
    return PASS;
}

int GLOB_SYSTEM_FATAL_ERROR(void)
{
    return 0;
}

void *GLOB_MALLOC(ADDRESSTYPE size)
{
    return malloc(size);
}

void GLOB_FREE(void *ptr)
{
    free(ptr);
}

void GLOB_SCHEDULE_FUNCTION(ISR_FN_T fn)
{
    fn(0, 0);
}

#if CMD_DMA
int GLOB_ISR(int dummy1, void *dummy2)
{
  return 0;
}
#endif

uint32 *GLOB_MEMMAP_NOCACHE(unsigned long addr, unsigned long size)
{
    return (uint32 *)addr;
}

uint32 *GLOB_MEMMAP_TOBUS(uint32 *ptr)
{
    return ptr;
}
#elif defined LINUX_DEVICE_DRIVER

FFS_LOCK_HANDLE GLOB_CREATE_FFS_API_LOCK()
{
    FFS_LOCK_HANDLE sem;
    sem = kmalloc(sizeof(struct semaphore), GFP_KERNEL);
    if (!sem)
        printk(KERN_ALERT "Spectra: Unable to allocate memory for semaphore. Semaphore not created.\n");
    else
	sema_init(sem,1);
    return sem;
}

void GLOB_RELEASE_FFS_API_LOCK(FFS_LOCK_HANDLE handle)
{
    if (handle) kfree (handle);
    return;
}

int GLOB_FFS_API_ENTER(FFS_LOCK_HANDLE handle)
{
    if (handle) down_interruptible(handle);
    return PASS;
}

int GLOB_FFS_API_EXIT(FFS_LOCK_HANDLE handle)
{
    if (handle) up(handle);
    return PASS;
}

int GLOB_SYSTEM_FATAL_ERROR(void)
{
    return 0;
}

void *GLOB_MALLOC(ADDRESSTYPE size)
{
    void *ptr;
    if (!(ptr = vmalloc(size)))
        printk(KERN_ERR "Spectra: MALLOC for size %lu FAILED!\n", size);
    return ptr;
}

void GLOB_FREE(void *ptr)
{
    vfree(ptr);
}

void GLOB_SCHEDULE_FUNCTION(ISR_FN_T fn)
{
    fn(0, 0);
}

#if CMD_DMA
static void GLOB_SBD_ISR(void);
int GLOB_ISR(int dummy1, void *dummy2)
{
    GLOB_SBD_ISR();
    return 0;
}
#endif

uint32 *GLOB_MEMMAP_NOCACHE(unsigned long addr, unsigned long size)
{
#if (FLASH_NAND || FLASH_CDMA)
    return (uint32 *)ioremap_nocache(addr, (size+0xfff)&(~0xfff));
#else
    return (uint32 *)addr;
#endif
}

void GLOB_MEMUNMAP_NOCACHE(volatile uint32 *addr)
{
#if (FLASH_NAND || FLASH_CDMA)
    iounmap(addr);
#endif
}

uint32 *GLOB_MEMMAP_TOBUS(uint32 *ptr)
{
#if (FLASH_NAND || FLASH_CDMA)
    return virt_to_bus(ptr);
#else
    return ptr;
#endif
}

#else
FFS_LOCK_HANDLE GLOB_CREATE_FFS_API_LOCK()
{
    return NULL;
}

void GLOB_RELEASE_FFS_API_LOCK(FFS_LOCK_HANDLE handle)
{
    return;
}

int GLOB_FFS_API_ENTER(FFS_LOCK_HANDLE handle)
{
    return PASS;
}

int GLOB_FFS_API_EXIT(FFS_LOCK_HANDLE handle)
{
    return FAIL;
}

int GLOB_SYSTEM_FATAL_ERROR(void)
{
    return 0;
}

void *GLOB_MALLOC(ADDRESSTYPE size)
{
    return malloc(size);
}

void GLOB_FREE(void *ptr)
{
    free(ptr);
}

void GLOB_SCHEDULE_FUNCTION(ISR_FN_T fn)
{
    fn(0, 0);
}

#if CMD_DMA
int GLOB_ISR(int dummy1, void *dummy2)
{
  return 0;
}
#endif

uint32 *GLOB_MEMMAP_NOCACHE(unsigned long addr, unsigned long size)
{
    return (uint32 *)addr;
}

uint32 *GLOB_MEMMAP_TOBUS(uint32 *ptr)
{
    return ptr;
}
#endif
#ifdef __cplusplus
}
#endif

#if defined __GNUC__
#else
int _stricmp(const char *__s1, const char *__s2)
{
    int i;
    int result = 0;
    unsigned char c1, c2;
    i = 0;
    while(__s1[i] != 0 && __s2[i] != 0)
    {
        c1 = __s1[i];
        c2 = __s2[i];
        if(c1 >= 'A' && c1 <= 'Z')
            c1 += ('a' -'A');
        if(c2 >= 'A' && c2 <= 'Z')
            c2 += ('a' -'A');
        result = (int)(c1 - c2);
        if(result != 0)
            break;
        i++;
    }
    return result;
}

int _strnicmp(const char *__s1, const char *__s2, int __n)
{
    int i;
    int result = 0;
    unsigned char c1, c2;
    for(i = 0; i < __n; i++)
    {
        c1 = __s1[i];
        c2 = __s2[i];
        if(c1 >= 'A' && c1 <= 'Z')
            c1 += ('a' -'A');
        if(c2 >= 'A' && c2 <= 'Z')
            c2 += ('a' -'A');
        result = (int)(c1 - c2);
        if(result != 0)
            break;
    }
    return result;
}
#endif

#ifdef LINUX_DEVICE_DRIVER
/*&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
  Linux device driver module. This prepares spectra to behave as a linux block
  driver. This code was tested with linux kernels 2.6, 2.6.17, 2.6.22. For
  the earlier kernel versions, the end_that_request_last function requires a
  second argument 1. The later versions (post 22) do not require it. The
  behavior has been tested to be the same in both cases.
 &&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&*/

#define NUM_DEVICES             1
#define PARTITIONS              4
#define KERNEL_SECTOR_SIZE	 512
#define NUM_OUTSTANDING_FTL_REQUESTS    8


#define CE4100_DEVID 0x0708

#define GLOB_SBD_NAME          "Glob_Spectra"
#define GLOB_SBD_IRQ_NUM       (4)


/*************************************************
* Spectra raw data access
**************************************************/
#if CMD_DMA

#else
#define DOWN_HW_SEM DOWN_SEM(spectra_hw_sem)
#define UP_HW_SEM UP_SEM(spectra_hw_sem)

#define DOWN_RW_SEM DOWN_SEM(spectra_rw_sem)
#define UP_RW_SEM UP_SEM(spectra_rw_sem)

#define DOWN_SEM(sem)  do{\
		if(down_interruptible(&sem))\
		{\
			printk(KERN_ERR "error in getting semaphore\n");\
			return -ERESTARTSYS;\
		}\
	}while(0)

#define UP_SEM(sem) do{\
		up(&sem);\
	}while(0)

#define CHCK(func, pass_val, action_on_fail) do{ \
										int ret_val; \
										ret_val = func; \
										if(ret_val != pass_val) \
										{ \
											printk(KERN_DEBUG " failed. return code = %d\n", ret_val); \
											action_on_fail; \
										} }while(0)

#define BOOT_PARTITION_ACCESS(blocknum) (blocknum < GLOB_DeviceInfo.wSpectraStartBlock)

/*Mutex semaphore to control hardware access*/
static struct semaphore spectra_hw_sem;

/*Read/Write mutex semaphore*/
static struct semaphore spectra_rw_sem;

/*Read/Write buffer pointer for raw access*/
byte* raw_blk_buf = NULL;

#endif

MODULE_LICENSE("GPL");

struct GLOB_SBD_dev {
    ADDRESSTYPE size;
    short users;
    spinlock_t qlock;
    struct request_queue *queue;
    struct gendisk *gd;
#if CMD_DMA
    wait_queue_head_t irq_wait_queue;
    int irq_count;
#endif
}ffs_dev;

static int GLOB_SBD_majornum;

struct GLOB_SBD_dev Device[NUM_DEVICES];

IDENTFY_DEVICE_DATA IdentifyDeviceData;
#if USEMEMCPY
#define MEM_ARRAY_SIZE     (GLOB_LLD_BLOCKS * GLOB_LLD_PAGES * GLOB_LLD_PAGE_DATA_SIZE)
static char *mem_array = NULL;
#define SBD_SECTOR_SIZE         (GLOB_LLD_PAGE_DATA_SIZE)
#define SBD_BLOCK_SIZE          (GLOB_LLD_PAGE_DATA_SIZE * GLOB_LLD_PAGES)
#else
#define SBD_SECTOR_SIZE         (IdentifyDeviceData.PageDataSize)
#define SBD_BLOCK_SIZE          (IdentifyDeviceData.PageDataSize * IdentifyDeviceData.PagesPerBlock)
#endif
byte *mem_pool_ptr = NULL;

#if CMD_DMA
struct GLOB_SBD_CMDDMA_Q {
    struct request *req;
    int num_sbd_sects;
}glob_sbd_cmddma_q;

struct GLOB_SBD_CMDDMA_Q cmddma_request_queue[NUM_OUTSTANDING_FTL_REQUESTS];
unsigned int cmddma_num_requests;
unsigned int cmddma_num_ftl_requests;

#if SBDBG
ADDRESSTYPE SBDBG_cdma_address[NUM_OUTSTANDING_FTL_REQUESTS];
#endif
#else
#include<linux/workqueue.h>
struct sbd_work_block {
	struct request *sbd_req;
	struct work_struct sbd_work;
};
struct workqueue_struct *sbd_wq;
//#include <linux/completion.h>
struct sbd_nandflush_info{
	struct task_struct *nandflush_task;
	struct completion nandflush_thread_start;
	struct completion nandflush_thread_exit;
	spinlock_t flush_thread_lock;

};
struct sbd_nandflush_info nand_flush_inf;
#define NAND_FLUSH_PERIOD (18 * HZ) //Spectra will run flush FTL cache each 18 seconds. it could be adapted.
extern int register_reboot_notifier(struct notifier_block *);
extern int unregister_reboot_notifier(struct notifier_block *);

#endif

#define SECTOR_SIZE_RATIO	(SBD_SECTOR_SIZE/KERNEL_SECTOR_SIZE)

// Static Function Declarations
static void GLOB_SBD_request(struct request_queue *q);

#if CMD_DMA
#include <linux/irq.h>

void GLOB_SBD_PostProcess(unsigned long int dummy)
{
    int i, status, first_failed_cmd;
    struct request *req;
    unsigned long irqflags;
#if (LINUX_KERNEL_VER == LINUX_KERNEL_2_6_22)
    struct bio *bio;
#endif
    struct GLOB_SBD_dev *dev = &Device[0];
    spin_lock_irqsave(&dev->qlock, irqflags);
#if USEMEMCPY
    status = EVENT_PASS;
#else
    status = GLOB_FTL_Event_Status(&first_failed_cmd);
#endif
    if (status == EVENT_CORRECTABLE_DATA_ERROR_FIXED) {

        goto postprocess_exit;
    }
    first_failed_cmd--;
    print(" Post Process called\n");
    switch(status)
    {
        case EVENT_PASS:
          first_failed_cmd = cmddma_num_requests;
        default:
            for (i=0; i<cmddma_num_requests; i++)
            {
                status = (i < first_failed_cmd) ? 1 : 0;
                req = cmddma_request_queue[i].req;
#if (LINUX_KERNEL_VER == LINUX_KERNEL_2_6_24)
                if (!status)
                {

                    printk(KERN_ERR "Spectra: FTL layer failed to transfer \
                        %lu bytes\n", req->hard_nr_sectors * KERNEL_SECTOR_SIZE);
                }
#else
#if (LINUX_KERNEL_VER == LINUX_KERNEL_2_6_22)
                rq_for_each_bio(bio, req) {
                    if (!status)
                    {

                        printk(KERN_ERR "Spectra: FTL layer failed a %s operation \
                                of %d bytes to flash address[0x%x]\n",
                                ((bio_data_dir(bio) == WRITE) ? "write" : "read"),
                                bio->bi_size, (unsigned)(bio->bi_sector *
                                        KERNEL_SECTOR_SIZE));
                    }
                }
#endif
#endif
                if (cmddma_request_queue[i].num_sbd_sects == -1) {
                    if (!end_that_request_first(req, status, req->hard_nr_sectors)) {
                        add_disk_randomness(req->rq_disk);
                        end_that_request_last(req, status);
                    }
                    else
                        printk(KERN_ALERT "Spectra: Block driver inconsistency. \
                          Summation of bytes in all the bios do not add up to the \
                          total number kernel expects.\n");
                    cmddma_request_queue[i].req = NULL;
                } else {
                    cmddma_request_queue[0].num_sbd_sects = cmddma_request_queue[i].num_sbd_sects;
                    cmddma_request_queue[0].req = cmddma_request_queue[i].req;
                    if (i) cmddma_request_queue[i].req = NULL;
                }
            }
        break;
    }
    cmddma_num_ftl_requests = 0;
    cmddma_num_requests = 0;
    if ( cmddma_request_queue[0].req) {
      GLOB_SBD_request(NULL);
    }
    else {
      blk_start_queue(dev->queue);
    }
postprocess_exit:
    spin_unlock_irqrestore(&dev->qlock, irqflags);
    GLOB_FTL_Enable_Disable_Interrupts(1);
    if (!dev->irq_count) {
      dev->irq_count++;
      wake_up_interruptible(&dev->irq_wait_queue);
    }
}

static DECLARE_TASKLET(GLOB_SBD_PostProcessStruct, GLOB_SBD_PostProcess, 0);

static void GLOB_SBD_ISR(void)
{
    GLOB_FTL_Enable_Disable_Interrupts(0);
    tasklet_schedule(&GLOB_SBD_PostProcessStruct);
    return;
}
#else
static int SBD_xfer_request(struct request *req);

static void sbd_work_func(struct work_struct *work)
{
	int sectors_xferred, success = 1;
	struct request *req;
	struct sbd_work_block *work_block = container_of(work, struct sbd_work_block, sbd_work);
	struct GLOB_SBD_dev *dev = &Device[0];
	req = work_block->sbd_req;
	DOWN_HW_SEM;
	sectors_xferred = SBD_xfer_request(req);
	UP_HW_SEM;

	success = (sectors_xferred < 0) ? -1 : 0;

	if (blk_end_request(req, success, sectors_xferred*KERNEL_SECTOR_SIZE)) {
            printk(KERN_ERR "request transfer error\n");
	}

	kfree(work_block);
}

static int sbd_submit_work(void (*func)(struct work_struct *work), struct request *req)
{
	struct sbd_work_block *new_work_block;
	new_work_block = kmalloc(sizeof(struct sbd_work_block), GFP_ATOMIC);
	new_work_block->sbd_req = req;
	INIT_WORK(&(new_work_block->sbd_work), func);
	queue_work(sbd_wq, &(new_work_block->sbd_work));
	return 0;
}

#endif


static int GLOB_SBD_transfer(unsigned long sector,
			       unsigned long nsect, int prevnsect, char *buffer, int write)
{
    ADDRESSTYPE dwAddress;
    u16 i;
    int result = PASS;

    if (sector % SECTOR_SIZE_RATIO) printk(KERN_ERR "Spectra: %s requested with sector %lu not a multiple of Spectra sector size %d\n",(write?"Write":"Read"), sector, SBD_SECTOR_SIZE);
    if (nsect % SECTOR_SIZE_RATIO) printk(KERN_ERR "Spectra: %s requested at sector %lu with num bytes %lu not a multiple of Spectra sector size %d\n",(write?"Write":"Read"), sector, nsect, SBD_SECTOR_SIZE);

    dwAddress = (ADDRESSTYPE)sector*KERNEL_SECTOR_SIZE;
    dwAddress += SBD_BLOCK_SIZE;
    nsect /= SECTOR_SIZE_RATIO;
    if (!nsect) printk(KERN_ERR "Spectra: %s of less than 1 sector requested\n",(write?"Write":"Read"));

    nsect += prevnsect;
#if CMD_DMA
    if (nsect < cmddma_request_queue[cmddma_num_requests].num_sbd_sects)
        return (nsect - prevnsect);
    if (prevnsect >  cmddma_request_queue[cmddma_num_requests].num_sbd_sects) {
        cmddma_request_queue[cmddma_num_requests].num_sbd_sects = prevnsect;
        printk(KERN_ERR "Spectra: CDMA_Request Queue Error. num_sbd_sects=%d, less than prevnsects=%d. Please contact Spectra for support.\n",
	     cmddma_request_queue[cmddma_num_requests].num_sbd_sects, prevnsect);
    }
#endif
    for(i = prevnsect; i < nsect; i++)
    {
#if CMD_DMA
        if (cmddma_num_ftl_requests >= NUM_OUTSTANDING_FTL_REQUESTS)
            break;
        if (i >=  cmddma_request_queue[cmddma_num_requests].num_sbd_sects) {
#if SBDBG
	        if (i==prevnsect)
                if ((NUM_OUTSTANDING_FTL_REQUESTS - cmddma_num_ftl_requests) >= (nsect - prevnsect))
                    printk(KERN_ERR "--- SBDBG_LOG: %d %lu %lu\n", write, dwAddress, nsect - prevnsect);
                else
                    printk(KERN_ERR "--- SBDBG_LOG: %d %lu %lu\n", write, dwAddress, NUM_OUTSTANDING_FTL_REQUESTS - cmddma_num_ftl_requests);
            {
                int sbdbg_i;
                for (sbdbg_i = 0; sbdbg_i < cmddma_num_ftl_requests; sbdbg_i++) {
                    if ((SBDBG_cdma_address[sbdbg_i] < (dwAddress + SBD_SECTOR_SIZE)) &&
                        (SBDBG_cdma_address[sbdbg_i] > (dwAddress - SBD_SECTOR_SIZE)))
                        printk(KERN_ERR "--- SBDBG_ERR: FTL req %d at addr %lu is within %d (1 sector) bytes of req %d at addr %lu\n",
                               cmddma_num_ftl_requests, dwAddress, SBD_SECTOR_SIZE, sbdbg_i, SBDBG_cdma_address[sbdbg_i]);
                }
                SBDBG_cdma_address[cmddma_num_ftl_requests] = dwAddress;
            }
#endif
#endif
            if (write)
#if USEMEMCPY
                memcpy(mem_array + dwAddress, buffer, SBD_SECTOR_SIZE);
#else
                result = GLOB_FTL_Page_Write(buffer, dwAddress);
#endif
            else
#if USEMEMCPY
                memcpy(buffer, mem_array + dwAddress, SBD_SECTOR_SIZE);
#else
                result = GLOB_FTL_Page_Read(buffer, dwAddress);
#endif
            if(PASS != result)
                break;
#if CMD_DMA
            cmddma_num_ftl_requests++;
            cmddma_request_queue[cmddma_num_requests].num_sbd_sects++;
        }
#endif
        dwAddress += SBD_SECTOR_SIZE;
        buffer += SBD_SECTOR_SIZE;
    }
    return ((result == PASS) ? (i-prevnsect) : -1);
}


/*&&&&&&&&&&&&&&&&&&&&&&
 * Transfer a full request.
 &&&&&&&&&&&&&&&&&&&&&&&*/
static int SBD_xfer_request(struct request *req)
{
    int nsect = 0, sbd_nsect = 0, ret_val = 0;
    struct bio_vec *bvec;
    struct req_iterator iter;
    unsigned long flags, prevsects = 0, currsects;
    sector_t sector;

    rq_for_each_segment(bvec, req, iter) {
        if (ret_val >= 0) {
	    char *buffer = (char *)kmap(bvec->bv_page) + bvec->bv_offset;
	    currsects = bvec->bv_len/KERNEL_SECTOR_SIZE;
	    if (!(iter.i)) {
	        sector = iter.bio->bi_sector;
                prevsects = currsects;
            }
            else {
	        sector = iter.bio->bi_sector + prevsects;
                prevsects += currsects;
	    }
	    ret_val = GLOB_SBD_transfer(sector, currsects, sbd_nsect,
		      buffer, bio_data_dir(iter.bio) == WRITE);
	    kunmap(bvec->bv_page);
	    nsect += currsects;
	    sbd_nsect += ret_val;
        }
    }
#if CMD_DMA

    if ((nsect/SECTOR_SIZE_RATIO) == sbd_nsect)
        cmddma_request_queue[cmddma_num_requests].num_sbd_sects = -1;
#endif
    return (ret_val>=0) ? (sbd_nsect * SECTOR_SIZE_RATIO) : -1;
}

/*&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
 * Request function that "handles clustering".
 &&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&*/
static void GLOB_SBD_request(struct request_queue *q)
{
    struct request *req;
    int sectors_xferred, success = 1;

#if CMD_DMA
    int  act_on_old_req = 0;

    /*&&&&&&&&&&&&&&&&&&&&&&&&&&&&
      1 line hack to make kernel work.
      This will only happen if-
      a. req function gets called even after blk_stop_queue is called
      b. the req function is reentrant.
      (a) has been seen happening. So this hack.
     &&&&&&&&&&&&&&&&&&&&&&&&&&&&&&*/
    if (cmddma_num_requests) return;

    if ((cmddma_num_ftl_requests < NUM_OUTSTANDING_FTL_REQUESTS) &&
	(cmddma_request_queue[0].req)) {
        sectors_xferred = SBD_xfer_request(cmddma_request_queue[0].req);
	cmddma_num_requests++;
	act_on_old_req = 1;
    }
    else {
#endif
    while (
#if CMD_DMA
	   (cmddma_num_ftl_requests < NUM_OUTSTANDING_FTL_REQUESTS) &&
#endif
       ((req = blk_peek_request(q)) != NULL)) {
	if (req->cmd_type != REQ_TYPE_FS) {
	    printk (KERN_NOTICE "Spectra: Skip non-fs request\n");
	    __blk_end_request_all(req, -EIO);
	    continue;
	}
#if CMD_DMA
	cmddma_request_queue[cmddma_num_requests].req = req;
	cmddma_request_queue[cmddma_num_requests].num_sbd_sects = 0;
	sectors_xferred = SBD_xfer_request(req);
	success = (sectors_xferred < 0) ? -1 : 0;


	blkdev_dequeue_request(req);
	cmddma_num_requests++;
#else
	blk_start_request(req);

	sbd_submit_work(sbd_work_func, req);
#endif
    }
#if CMD_DMA
    }
    if (cmddma_num_ftl_requests) {
	if (!act_on_old_req)
	    blk_stop_queue(q);
#if SBDBG
	printk(KERN_ERR "--- SBDBG_LOG: 2 0 0\n");
#endif
#if USEMEMCPY
	GLOB_ISR(0,0);
#else
	GLOB_FTL_Execute_CMDS ();
#endif
    }
#endif
}

static int  GLOB_SBD_open(struct block_device *bdev,fmode_t mode)
{
    int status = 0;
    struct GLOB_SBD_dev *dev = bdev->bd_disk->private_data;
#if CMD_DMA

#if !FAKE_INTERRUPTS
    if (!dev->users) {
        status = request_irq(GLOB_SBD_IRQ_NUM, GLOB_ISR, 0, "Glob_Spectra_Irq",
		 (void *)dev);
         if (status) {
             printk(KERN_ERR "Spectra: Cannot get assigned IRQ %d\n", GLOB_SBD_IRQ_NUM);
             status = -EACCES;
        }
       }
#endif
#endif
//    dev->users++;
    return status;
}
static int  GLOB_SBD_release(struct gendisk *gd,fmode_t mode)
{
    int status = 0;
    struct GLOB_SBD_dev *dev = gd->private_data;

#if CMD_DMA
    if (cmddma_num_requests) {
      dev->irq_count = 0;
      if (cmddma_num_requests)
        wait_event_interruptible(dev->irq_wait_queue, dev->irq_count);
      else
        dev->irq_count = 1;
    }
    cmddma_num_requests = 0;
    cmddma_num_ftl_requests = 0;
#else
	DOWN_RW_SEM;
    flush_workqueue(sbd_wq);
	UP_RW_SEM;

#endif

    DOWN_HW_SEM;
    if (GLOB_FTL_Flush_Cache() == 1) {
#if CMD_DMA
        dev->irq_count = 0;
        GLOB_FTL_Execute_CMDS();
        if (!dev->irq_count) wait_event_interruptible(dev->irq_wait_queue, dev->irq_count);
#endif
    }
	UP_HW_SEM;

#if CMD_DMA

#if !FAKE_INTERRUPTS
    if (dev->users) {
        free_irq(GLOB_SBD_IRQ_NUM, (void *)dev);
    }
#endif
#endif
    //dev->users--;
    return status;
}

int GLOB_SBD_getgeo(struct block_device *bdev, struct hd_geometry *geo)
{
    ADDRESSTYPE size;
	struct GLOB_SBD_dev *dev = bdev->bd_disk->private_data;


    size = dev->size;
    geo->cylinders = (size & ~0x3f) >> 6;
    geo->heads = 4;
    geo->sectors = 16;
	return 0;
}
static int GLOB_SBD_ioctl_rw( SPEC_IO_CMD *pcmd, unsigned int cmd, unsigned long arg)
{
	uint32  status = PASS;
	if (PASS != copy_from_user(pcmd, (void *)arg, sizeof(SPEC_IO_CMD)))
		return FAIL;

	if((pcmd->StartBlockNum < 0)||(pcmd->StartPageNum < 0)||(pcmd->NumPagesToTransfer < 0))
        return FAIL;

	if (pcmd->StartBlockNum > RAW_DeviceInfo.wSpectraEndBlock)
        return FAIL;

	memset(raw_blk_buf,0,RAW_DeviceInfo.wBlockSize);

    switch(cmd) {
		case GLOB_SBD_IOCTL_RD_MAIN:
			if (BOOT_PARTITION_ACCESS(pcmd->StartBlockNum))
				CHCK((status = GLOB_BBT_NAND_Read_Page_Main(raw_blk_buf, pcmd->StartBlockNum, pcmd->StartPageNum, pcmd->NumPagesToTransfer)),PASS,return status);
			else
				CHCK((status = GLOB_LLD_Read_Page_Main(raw_blk_buf, pcmd->StartBlockNum, pcmd->StartPageNum, pcmd->NumPagesToTransfer)),PASS,return status);
			if (PASS != copy_to_user((byte*)pcmd->AddrInRam,raw_blk_buf,pcmd->NumPagesToTransfer * RAW_DeviceInfo.wPageDataSize))
					return FAIL;
			break;
		case GLOB_SBD_IOCTL_WR_MAIN:
			if (PASS != copy_from_user(raw_blk_buf,(byte*)pcmd->AddrInRam,pcmd->NumPagesToTransfer * RAW_DeviceInfo.wPageDataSize))
					return FAIL;
			if (BOOT_PARTITION_ACCESS(pcmd->StartBlockNum))
				CHCK(GLOB_BBT_NAND_Write_Page_Main(raw_blk_buf, pcmd->StartBlockNum, pcmd->StartPageNum, pcmd->NumPagesToTransfer),PASS,return FAIL);
			else
				CHCK(GLOB_LLD_Write_Page_Main(raw_blk_buf, pcmd->StartBlockNum, pcmd->StartPageNum, pcmd->NumPagesToTransfer),PASS,return FAIL);
			break;
		case GLOB_SBD_IOCTL_RD_MAIN_SPARE_RAW:
			CHCK(GLOB_LLD_Read_Page_Main_Spare_Raw(raw_blk_buf, pcmd->StartBlockNum, pcmd->StartPageNum, pcmd->NumPagesToTransfer),PASS,return FAIL);
			if (PASS != copy_to_user((byte*)pcmd->AddrInRam,raw_blk_buf,pcmd->NumPagesToTransfer * RAW_DeviceInfo.wPageSize))
					return FAIL;
			break;
		case GLOB_SBD_IOCTL_WR_MAIN_SPARE_RAW:
				return FAIL;
		default:
			return FAIL;
    }
	return PASS;
}



int GLOB_SBD_ioctl(struct block_device *bdev,fmode_t mode,unsigned int cmd,unsigned long arg)
{
    struct GLOB_SBD_dev *dev = bdev->bd_disk->private_data;
    struct hd_geometry geo;
    ADDRESSTYPE size;
    SPEC_IO_CMD rwcmd;
	uint32  status = PASS;

//    if (!filp || !(dev = filp->private_data)) {
//        printk(KERN_ERR "Spectra: IOCTL called with NULL file pointer\n");
//        return -ENOTTY;
//    }
    if(!dev)
    {
	printk(KERN_ERR "Spectra: Private data is NULL\n");
	return -ENOTTY;
    }

    if (dev != Device) {
        printk(KERN_ERR "Spectra: IOCTL called with no dev\n");
        return -ENOTTY;
    }

    DOWN_HW_SEM;
    switch(cmd) {
        case HDIO_GETGEO:
            size = dev->size;
            geo.cylinders = (size & ~0x3f)>>6;
            geo.heads = 4;
            geo.sectors = 16;
            if (copy_to_user((void *) arg, &geo, sizeof(geo)))
                goto out_error1;
            break;
        case GLOB_SBD_IOCTL_GC:
            printk(KERN_DEBUG "Spectra: IOCTL: Garbage Collection being performed\n");
            if (PASS != GLOB_FTL_Garbage_Collection ())
                goto out_error1;
            break;
        case GLOB_SBD_IOCTL_WL:
            printk(KERN_DEBUG "Spectra: IOCTL: Static Wear Leveling being performed\n");
            if (PASS != GLOB_FTL_Wear_Leveling ())
                goto out_error1;
            break;
        case GLOB_SBD_IOCTL_FORMAT:
            printk(KERN_DEBUG "Spectra: IOCTL: Flash format being performed\n");
            if (PASS != GLOB_FTL_Flash_Format ())
                goto out_error1;
            break;
        case GLOB_SBD_IOCTL_FLUSH_CACHE:
            printk(KERN_DEBUG "Spectra: IOCTL: Cache flush being performed\n");
            if (GLOB_FTL_Flush_Cache() == 1) {
#if CMD_DMA
                dev->irq_count = 0;
                GLOB_FTL_Execute_CMDS();
                if (!dev->irq_count) wait_event_interruptible(dev->irq_wait_queue, dev->irq_count);
#endif
            }
            break;
		case GLOB_SBD_IOCTL_RD_MAIN:
			status = GLOB_SBD_ioctl_rw(&rwcmd, GLOB_SBD_IOCTL_RD_MAIN,arg);
			break;
		case GLOB_SBD_IOCTL_WR_MAIN:
			status = GLOB_SBD_ioctl_rw(&rwcmd, GLOB_SBD_IOCTL_WR_MAIN,arg);
			break;
		case GLOB_SBD_IOCTL_ERASE:
			if (BOOT_PARTITION_ACCESS((BLOCKNODE)arg))
				status = GLOB_BBT_Erase_Block_by_BBT((BLOCKNODE)arg);
			else
				status = GLOB_LLD_Erase_Block((BLOCKNODE)arg);
			break;
		case GLOB_SBD_IOCTL_RD_ID:
			CHCK(copy_to_user((void*)arg, &RAW_DeviceInfo, sizeof(RAW_DeviceInfo)), PASS, goto out_error1);
            break;
		case GLOB_SBD_IOCTL_CHK_BADBLK:
			rwcmd.StartBlockNum = (BLOCKNODE)arg;
			CHCK(((rwcmd.StartBlockNum  < 0)||(rwcmd.StartBlockNum >RAW_DeviceInfo.wSpectraEndBlock)), PASS, goto out_error1);
	        status = GLOB_LLD_Get_Bad_Block_Raw(rwcmd.StartBlockNum);
			break;
		case GLOB_SBD_IOCTL_RD_MAIN_SPARE_RAW:
			status = GLOB_SBD_ioctl_rw(&rwcmd, GLOB_SBD_IOCTL_RD_MAIN_SPARE_RAW,arg);
			break;
		case GLOB_SBD_IOCTL_WR_MAIN_SPARE_RAW:
			status = GLOB_SBD_ioctl_rw(&rwcmd, GLOB_SBD_IOCTL_WR_MAIN_SPARE_RAW,arg);
			break;
		case GLOB_SBD_IOCTL_ERASE_RAW:
			rwcmd.StartBlockNum = (BLOCKNODE)arg;
			CHCK((rwcmd.StartBlockNum >RAW_DeviceInfo.wSpectraEndBlock), PASS, goto out_error1);
			status = GLOB_LLD_Erase_Block(rwcmd.StartBlockNum);
			break;
		case GLOB_SBD_IOCTL_ERASE_BYMARKER:
			status = GLOB_BBT_Erase_Block_by_Marker((BLOCKNODE)arg);
			break;
		case GLOB_SBD_IOCTL_RD_BT:
			status = GLOB_BBT_IO_Read_Block_Table((BOOT_BLOCKTABLE_IO_CMD*)arg);
			break;
		case GLOB_SBD_IOCTL_CREATE_BT_DUMMY:
			status = GLOB_BBT_Create_Block_Table_Dummy(arg);
			break;
		case GLOB_SBD_IOCTL_CREATE_BT_BYERASE:
			status = GLOB_BBT_Create_Block_Table_byErase();
			break;
		case GLOB_SBD_IOCTL_CREATE_BT_BYMARKER:
			status = GLOB_BBT_Create_Block_Table_byInitalMarker();
			break;
		default:
			goto out_error2;
    }
	UP_HW_SEM;
	return status;
out_error1:
	UP_HW_SEM;
	return -EFAULT;
out_error2:
	UP_HW_SEM;
    return -ENOTTY;
}
static int GLOB_SBD_reboot(struct notifier_block *self, unsigned long event, void *data)
{

               DOWN_HW_SEM;
               GLOB_FTL_Flush_Cache();
               UP_HW_SEM;
               return NOTIFY_OK;
}
static struct notifier_block GLOB_reboot_notifier={
           .notifier_call   = GLOB_SBD_reboot
};

static int GLOB_nandflush_thread(struct sbd_nandflush_info *pflush)
{
	siginfo_t info;
	unsigned long signr;
	daemonize("nandflush");
	allow_signal(SIGKILL);
	allow_signal(SIGTERM);

	pflush->nandflush_task = current;
	complete(&pflush->nandflush_thread_start);

	set_user_nice(current, 10);
	for ( ; ; ) {
		while (!signal_pending(current)) {
		schedule_timeout_interruptible(NAND_FLUSH_PERIOD);
		DOWN_HW_SEM;
		GLOB_FTL_Flush_Cache();
		UP_HW_SEM;
		}
		signr = dequeue_signal_lock(current, &current->blocked, &info);
		switch(signr)
		{
		case SIGTERM:
		case SIGKILL:
			printk(KERN_DEBUG "GLOB_nandflush_thread(): SIGKILL received.\n");
			goto die;
		default:
			printk(KERN_DEBUG "GLOB_nandflush_thread(): signal %ld received\n", signr);
		}
		}
die:
	spin_lock(&pflush->flush_thread_lock);
	pflush->nandflush_task = NULL;
	spin_unlock(&pflush->flush_thread_lock);
	complete_and_exit(&pflush->nandflush_thread_exit,0);
}
static int start_nandflush_thread(struct sbd_nandflush_info *pflush)
{
	pid_t pid;
	int ret = 0;

	pflush->nandflush_task = NULL;
	init_completion(&pflush->nandflush_thread_start);
	init_completion(&pflush->nandflush_thread_exit);
	spin_lock_init(&pflush->flush_thread_lock);

	pid = kernel_thread(GLOB_nandflush_thread,pflush,CLONE_FS|CLONE_FILES);
	if (pid < 0) {
		printk(KERN_WARNING "fork failed for NAND flush thread thread: %d\n", -pid);
		complete(&pflush->nandflush_thread_exit);
		ret = pid;
	} else {
		/* Wait for it... */
		printk(KERN_DEBUG "NAND flush thread is pid %d\n", pid);
		wait_for_completion(&pflush->nandflush_thread_start);
	}

	return ret;
}
static void stop_nandflush_thread(struct sbd_nandflush_info *pflush)
{
	int wait = 0;
	spin_lock(&pflush->flush_thread_lock);
	if (pflush->nandflush_task)
	{
	send_sig(SIGKILL, pflush->nandflush_task, 1);
	wait = 1;
	}
	spin_unlock(&pflush->flush_thread_lock);
	if (wait)
		wait_for_completion(&pflush->nandflush_thread_exit);

}

static struct block_device_operations GLOB_SBD_ops ={
    .owner   = THIS_MODULE,
    .open    = GLOB_SBD_open,
    .release = GLOB_SBD_release,
    .getgeo  = GLOB_SBD_getgeo,
    .ioctl   = GLOB_SBD_ioctl
};


static int SBD_setup_device(struct GLOB_SBD_dev* dev, int which)
{
    memset(dev,0,sizeof(struct GLOB_SBD_dev));
#if USEMEMCPY
    dev->size = (ADDRESSTYPE)MEM_ARRAY_SIZE - (ADDRESSTYPE)(33 * GLOB_LLD_PAGES * GLOB_LLD_PAGE_DATA_SIZE);
#else
    dev->size = (ADDRESSTYPE)IdentifyDeviceData.PageDataSize * IdentifyDeviceData.PagesPerBlock * (IdentifyDeviceData.wDataBlockNum-200);
#if SBDBG
    printk(KERN_ERR "----LOG: Num blocks=%d, pagesperblock=%d, pagedatasize=%d, RATIO=%d\n",
	   IdentifyDeviceData.wDataBlockNum, IdentifyDeviceData.PagesPerBlock,
	   IdentifyDeviceData.PageDataSize, SECTOR_SIZE_RATIO);
#endif
#endif
    spin_lock_init(&dev->qlock);

    dev->queue = blk_init_queue(GLOB_SBD_request, &dev->qlock);
    if(dev->queue == NULL) {
        printk(KERN_ERR "Spectra: Request queue could not be initialized. \n ");
        goto out_vfree;
    }
	blk_queue_logical_block_size(dev->queue, SBD_SECTOR_SIZE);
    dev->queue->queuedata = dev;


    dev->gd = alloc_disk(PARTITIONS);
    if(! dev->gd)
    {
        printk(KERN_ERR "Spectra: Could not allocate disk.  \n ");
        goto out_vfree;
    }
    dev->gd->major = GLOB_SBD_majornum;
    dev->gd->first_minor = which*PARTITIONS;
    dev->gd->fops = &GLOB_SBD_ops;
    dev->gd->queue = dev->queue;
    dev->gd->private_data = dev;
    snprintf(dev->gd->disk_name,32,"%s%c", GLOB_SBD_NAME, which+'a');
    set_capacity(dev->gd,(dev->size >> GLOB_Calc_Used_Bits(KERNEL_SECTOR_SIZE)));
#if CMD_DMA
    cmddma_request_queue[0].req = NULL;
    cmddma_num_requests = 0;
    cmddma_num_ftl_requests = 0;
    init_waitqueue_head(&dev->irq_wait_queue);
    dev->irq_count = 1;
#endif
    add_disk(dev->gd);
    return 0;
out_vfree:
    return -ENOMEM;
}


static int GLOB_SBD_init(void)
{
    int i;
    unsigned int root_devid = 0;
/*
 *  Detemine SOC type, install the driver only in Intel Media CE4100 platform
 */
    raw_pci_read(0, 0, PCI_DEVFN(0, 0), PCI_DEVICE_ID, 2, &root_devid);
    if (CE4100_DEVID != root_devid)
	    return 0;

    GLOB_SBD_majornum = register_blkdev(0, GLOB_SBD_NAME);
    if(GLOB_SBD_majornum <= 0)
    {
        printk(KERN_ALERT "Unable to get the major %d for Spectra",GLOB_SBD_majornum);
        return -EBUSY;
    }

#if CMD_DMA
    for (i=0; i<NUM_OUTSTANDING_FTL_REQUESTS; i++)
      cmddma_request_queue[i].req = NULL;
#else
    sbd_wq = create_singlethread_workqueue(GLOB_SBD_NAME);
	sema_init(&spectra_hw_sem,1);
	sema_init(&spectra_rw_sem,1);

#endif

#if USEMEMCPY
    mem_array = (char *)vmalloc(sizeof(char) * MEM_ARRAY_SIZE);
#endif
    if (PASS != GLOB_FTL_Flash_Init()) {
        printk(KERN_ERR "Spectra: unable to initialize nand flash device. \n");
        goto out_flash_register;
    }


    if (PASS != GLOB_FTL_IdentifyDevice(&IdentifyDeviceData)) {
        printk(KERN_ERR "Spectra: Unable to Read Flash Device. \n");
        goto out_flash_register;
    }
    if (SBD_SECTOR_SIZE % KERNEL_SECTOR_SIZE) {
        printk(KERN_ERR "Spectra: Flash page data size is not an integral multiple of kernel sector size %d. \n", KERNEL_SECTOR_SIZE);
        goto out_flash_register;
    }
#if !(FLASH_NAND || FLASH_CDMA)

    if (!(mem_pool_ptr = (byte *)vmalloc(IdentifyDeviceData.SizeOfGlobalMem))) {
        printk(KERN_ERR "Spectra: Unable to Initialize Memory Pool. \n");
        goto out_mempool_flash_register;
    }
#else
//     mem_pool_ptr = GLOB_MEMMAP_NOCACHE(SBD_MEMPOOL_PTR, IdentifyDeviceData.SizeOfGlobalMem);
     printk("Global Memory is %d\n",IdentifyDeviceData.SizeOfGlobalMem);

     if(!(mem_pool_ptr = (byte *)kmalloc(IdentifyDeviceData.SizeOfGlobalMem, GFP_KERNEL))) {
        printk(KERN_ERR "Spectra: Unable to Initialize Memory Pool. \n");
        goto out_mempool_flash_register;
    }
#endif
    if (PASS != GLOB_FTL_Mem_Config(mem_pool_ptr)) {
        printk(KERN_ERR "Spectra: Unable to Read Flash Device. \n");
        goto out_mempool_flash_register;
    }
    if (PASS != GLOB_FTL_Init()) {
        printk(KERN_ERR "Spectra: Unable to Initialize FTL Layer. \n");
        goto out_ftl_flash_register;
    }
	if (PASS != GLOB_BBT_Init()) {
        printk(KERN_ERR "Spectra: Unable to Initialize BBT Layer. \n");
        goto out_ftl_flash_register;
    }

    for(i=0;i<NUM_DEVICES; i++)
        if(SBD_setup_device(&Device[i], i) == -ENOMEM)
            goto out_bbt_register;
    printk(KERN_INFO "Spectra: Spectra module loaded with major number %d\n", GLOB_SBD_majornum);
	if(!(raw_blk_buf = (byte *)kmalloc(GLOB_DeviceInfo.wBlockSize, GFP_DMA))) {
        printk(KERN_ERR "Spectra: Unable to allocate block buffer for raw access. \n");
            goto out_bbt_register;
    }
	if (start_nandflush_thread(&nand_flush_inf)){
		   printk(KERN_ERR "Spectra: Unable to start nand flush thread. \n");
            goto out_bbt_register;
    }
	register_reboot_notifier(&GLOB_reboot_notifier);

#if CMD_DMA
#if !FAKE_INTERRUPTS

#endif
#endif
    return 0;
out_bbt_register:
	GLOB_BBT_Release();
out_ftl_flash_register:
	GLOB_FTL_Cache_Release();

out_mempool_flash_register:
#if !(FLASH_NAND || FLASH_CDMA)
	vfree(mem_pool_ptr);
#else
	kfree(mem_pool_ptr);
#endif

out_flash_register:
#if USEMEMCPY
	vfree(mem_array);
#endif
	GLOB_FTL_Flash_Release();
	unregister_blkdev(GLOB_SBD_majornum, GLOB_SBD_NAME);
	return -ENOMEM;
}

static void __exit GLOB_SBD_exit(void)
{
    int i;
	unregister_reboot_notifier(&GLOB_reboot_notifier);
	stop_nandflush_thread(&nand_flush_inf);
    for(i=0; i<NUM_DEVICES; i++) {
        struct GLOB_SBD_dev *dev = &Device[i];
        if (dev->gd) {
            del_gendisk(dev->gd);
            put_disk(dev->gd);
        }
        if (dev->queue) blk_cleanup_queue(dev->queue);
    }

    unregister_blkdev(GLOB_SBD_majornum, GLOB_SBD_NAME);
#if CMD_DMA
#else
    destroy_workqueue(sbd_wq);
#endif
#if USEMEMCPY
    vfree(mem_array);
#endif
    GLOB_BBT_Release();
    GLOB_FTL_Flush_Cache();
    GLOB_FTL_Cache_Release();

#if !(FLASH_NAND || FLASH_CDMA)
    vfree(mem_pool_ptr);
#else
    kfree(mem_pool_ptr);

#endif
    kfree(raw_blk_buf);
    GLOB_FTL_Flash_Release();
    printk(KERN_INFO "Spectra FTL module (major number %d) unloaded.\n", GLOB_SBD_majornum);
}

module_init(GLOB_SBD_init);
module_exit(GLOB_SBD_exit);
#endif

#if SEAMLESS
#include <stdarg.h>

int exit_sim(){
    return 0;
}

int embedded_strprintf(char *m){
    return 0;
}
int printf(const char *format, ...)
{
    va_list ap;
    static char str[300];
    va_start(ap, format);
    vsnprintf(str, 300, format, ap);
    va_end(ap);
    return embedded_strprintf(str);
}

int reset_flash_memory_device() {
  return 0;
}
#endif
