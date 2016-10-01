
/*
 *  linux/drivers/char/mem.c
 *
 *  Copyright (C) 1991, 1992  Linus Torvalds
 *
 *  Added devfs support.
 *    Jan-11-1998, C. Scott Ananian <cananian@alumni.princeton.edu>
 *  Shared /dev/zero mmaping support, Feb 2000, Kanoj Sarcar <kanoj@sgi.com>
 */
/*
 *  GPL LICENSE SUMMARY
 *
 *  Copyright(c) 2011-2012 Intel Corporation. All rights reserved.
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of version 2 of the GNU General Public License as
 *  published by the Free Software Foundation.
 *
 *  This program is distributed in the hope that it will be useful, but
 *  WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *  General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 51 Franklin St - Fifth Floor, Boston, MA 02110-1301 USA.
 *  The full GNU General Public License is included in this distribution
 *  in the file called LICENSE.GPL.
 *
 *  Contact Information:
 *    Intel Corporation
 *    2200 Mission College Blvd.
 *    Santa Clara, CA  97052
 *
 */

/*
 * The following code is to create /dev/devmem which simulates the behavior of /dev/mem
 * to solve common user without root privilege can not open /dev/mem issue.
 *
*/
#include <linux/mm.h>
#include <linux/miscdevice.h>
#include <linux/slab.h>
#include <linux/vmalloc.h>
#include <linux/mman.h>
#include <linux/random.h>
#include <linux/init.h>
#include <linux/raw.h>
#include <linux/tty.h>
#include <linux/capability.h>
#include <linux/ptrace.h>
#include <linux/device.h>
#include <linux/highmem.h>
#include <linux/crash_dump.h>
#include <linux/backing-dev.h>
#include <linux/bootmem.h>
#include <linux/splice.h>
#include <linux/pfn.h>
#include <linux/pci.h>

#include <asm/uaccess.h>
#include <asm/io.h>

#ifdef CONFIG_IA64
# include <linux/efi.h>
#endif

/* I/O range starts from 2GB on CE4100/CE4200/CE5300 platforms. */
#define CE_MEDIA_IO_BASE 0x80000000UL
/* I/O range starts from 3GB on CE2600 platform. */
#define CE2600_IO_BASE 0xC0000000UL

extern unsigned long max_pfn;
static unsigned long io_base, io_base_pfn;


int __attribute__((weak)) phys_mem_access_prot_allowed(struct file *file,
	unsigned long pfn, unsigned long size, pgprot_t *vma_prot)
{
	return 1;
}

#ifdef CONFIG_STRICT_DEVMEM
static inline int range_is_allowed(unsigned long pfn, unsigned long size)
{
	u64 from = ((u64)pfn) << PAGE_SHIFT;
	u64 to = from + size;
	u64 cursor = from;

	while (cursor < to) {
		if (!devmem_is_allowed(pfn)) {
			printk(KERN_INFO
		"Program %s tried to access /dev/devmem between %Lx->%Lx.\n",
				current->comm, from, to);
			return 0;
		}
		cursor += PAGE_SIZE;
		pfn++;
	}
	return 1;
}
#else
static inline int range_is_allowed(unsigned long pfn, unsigned long size)
{
	return 1;
}
#endif

void __attribute__((weak))
map_devmem(unsigned long pfn, unsigned long len, pgprot_t prot)
{
	/* nothing. architectures can override. */
}

void __attribute__((weak))
unmap_devmem(unsigned long pfn, unsigned long len, pgprot_t prot)
{
	/* nothing. architectures can override. */
}

static void mmap_mem_open(struct vm_area_struct *vma)
{
	map_devmem(vma->vm_pgoff,  vma->vm_end - vma->vm_start,
			vma->vm_page_prot);
}

static void mmap_mem_close(struct vm_area_struct *vma)
{
	unmap_devmem(vma->vm_pgoff,  vma->vm_end - vma->vm_start,
			vma->vm_page_prot);
}


static struct vm_operations_struct mmap_mem_ops = {
	.open  = mmap_mem_open,
	.close = mmap_mem_close,
#ifdef CONFIG_HAVE_IOREMAP_PROT
	.access = generic_access_phys
#endif
};

static inline int range_is_sys_ram(unsigned long pfn, unsigned long size)
{
        u64 from = ((u64)pfn) << PAGE_SHIFT;
        u64 to = from + size;
        u64 cursor = from;

        while (cursor < to) {
                if (page_is_ram(pfn)) {
                        printk(KERN_INFO "system ram! pfn 0x%x.\n", (unsigned int)pfn);
                        return 1;
                }
                cursor += PAGE_SIZE;
                pfn++;
        }
        return 0;
}

static int mmap_mem(struct file * file, struct vm_area_struct * vma)
{
	size_t size = vma->vm_end - vma->vm_start;

	unsigned long size_of_pfn = (size % PAGE_SIZE) ? ((size >> PAGE_SHIFT) + 1) : (size >> PAGE_SHIFT);
	if ((vma->vm_pgoff < 0) || (size < 0))
		return -EINVAL;


	/* I/O registers space access without CAP_SYS_RAWIO capability is not allowed. */
	if ((vma->vm_pgoff + size_of_pfn > io_base_pfn) && !capable(CAP_SYS_RAWIO))
		return -EPERM;

	if (range_is_sys_ram(vma->vm_pgoff, size))
		return -EPERM;

	if (!range_is_allowed(vma->vm_pgoff, size))
		return -EPERM;

	if (!phys_mem_access_prot_allowed(file, vma->vm_pgoff, size,
						&vma->vm_page_prot))
		return -EINVAL;

	vma->vm_page_prot = phys_mem_access_prot(file, vma->vm_pgoff,
							size,
							vma->vm_page_prot);

	vma->vm_ops = &mmap_mem_ops;

	/* Remap-pfn-range will mark the range VM_IO and VM_RESERVED */
	if (remap_pfn_range(vma,
			    vma->vm_start,
			    vma->vm_pgoff,
			    size,
			    vma->vm_page_prot)) {
		return -EAGAIN;
	}
	return 0;
}

static int open_mem(struct inode * inode, struct file * filp)
{
	// Not need superuser privilege
	/* return capable(CAP_SYS_RAWIO) ? 0 : -EPERM;*/
	return 0;
}

static const struct file_operations mem_fops = {
	.mmap		= mmap_mem,
	.open		= open_mem,
};

static int __init devmem_init(void)
{
	unsigned int soc_id = 0;
	if (0 > register_chrdev(0,"devmem",&mem_fops))
		printk("unable to get major for memory devs\n");

	intelce_get_soc_info(&soc_id, NULL);

	switch(soc_id) {
		case CE3100_SOC_DEVICE_ID:
		case CE4100_SOC_DEVICE_ID:
		case CE4200_SOC_DEVICE_ID:
		case CE5300_SOC_DEVICE_ID:
			io_base = CE_MEDIA_IO_BASE;
			break;
		case CE2600_SOC_DEVICE_ID:
			io_base = CE2600_IO_BASE;
			break;
		default:
			return -ENODEV;
	}
	io_base_pfn = (io_base >> PAGE_SHIFT ) - 1;
	return 0;
}

fs_initcall(devmem_init);
