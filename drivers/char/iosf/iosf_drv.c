/*
 *  GPL LICENSE SUMMARY
 *
 *  Copyright(c) 2010-2013 Intel Corporation. All rights reserved.
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

/*------------------------------------------------------------------------------
 * File Name: iosf_drv.c
 *------------------------------------------------------------------------------
 */


#include <linux/kernel.h>
#include <linux/version.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/sched.h>
#include <linux/interrupt.h>
#include <linux/proc_fs.h>
#include <linux/wait.h>

#include <linux/list.h>
#include <linux/interrupt.h>
#include <linux/device.h>
#include <linux/pci.h>
#include <linux/cdev.h>
#include <asm/uaccess.h>
#include <asm/irq.h>
#include <asm/io.h>


#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,28)
    #include <asm/semaphore.h>
#else
    #include <linux/semaphore.h>
#endif

#include "iosf_common.h"
#include <linux/iosf_core.h>

/* Driver identification */
MODULE_AUTHOR("Intel Corporation, (C) 2006 - 2010 - All Rights Reserved");
MODULE_DESCRIPTION("IOSF Device Driver for Linux 2.6");
MODULE_SUPPORTED_DEVICE("Intel Media Processors");
MODULE_LICENSE("Dual BSD/GPL"); /* Inform kernel that driver is not GPL. */

/* Unique name for driver */
#ifndef DEV_NAME
#define DEV_NAME  "iosf"
#endif
const static char *devname = DEV_NAME;

#ifndef MOD_NAME
#define MOD_NAME "iosf.ko"
#endif


/* This function is the first function called to load the driver after an insmod */
/* command */


#define NR_IOSF_CHRDEVS  2

static int iosf_major;

extern  struct class *iosf_class;



/*------------------------------------------------------------------------------
 * iosf_open
 *------------------------------------------------------------------------------
 */

static int iosf_drv_open(struct inode *inode, struct file *filp)
{
		struct iosf_host *host;
		uint32_t bus_id;
		bus_id = iminor(inode);
		host = iosf_request(bus_id);

		if (!host) return -ENODEV;

		filp->private_data = host;
        return 0;
}

/*------------------------------------------------------------------------------
 * scard_close
 *------------------------------------------------------------------------------
 */

static int iosf_drv_close(struct inode *inode, struct file *filp)
{
		struct iosf_host *host;

		host = (struct iosf_host *)filp->private_data;
		iosf_release(host);

		filp->private_data = NULL;
        return 0;
}


/*------------------------------------------------------------------------------
 * iosf_ioctl
 *------------------------------------------------------------------------------
 */
static long iosf_unlocked_ioctl(struct file *filp, u_int cmd, u_long arg)
{
	struct iosf_info_user  iosf_info;
	struct iosf_host *host;
	void __user *argp = (void __user *)arg;
	int ret = 0;

    /* Check for valid pointer to the parameter list */
     if (0 == arg)
	 {
	printk(KERN_ERR "%s:%4i:  iosf chrdev ioctl failed\n", __FILE__, __LINE__);
        return -EINVAL;
     }

	if (copy_from_user(&iosf_info, argp, sizeof(iosf_info)))
		return -EFAULT;

	host = (struct iosf_host *)filp->private_data;



        /* Execute ioctl request */
	switch (cmd)
	{
	case IOSF_IOC_RD:
			if (iosf_info.flag & IOSF_32BITS_FLAG)
			{
				ret = kiosf_reg_read32(host, iosf_info.dest_port, iosf_info.offset,&iosf_info.value);
				if (copy_to_user(argp, &iosf_info, sizeof(iosf_info)))
					return -EFAULT;
			}
			else if(iosf_info.flag & (IOSF_16BITS_FLAG | IOSF_8BITS_FLAG))
			{
				ret = -ENOSYS;
			}
			break;

		case IOSF_IOC_WR:
			if (iosf_info.flag & IOSF_32BITS_FLAG)
			{
				ret = kiosf_reg_write32(host, iosf_info.dest_port, iosf_info.offset,iosf_info.value);
			}
			else if(iosf_info.flag & (IOSF_16BITS_FLAG | IOSF_8BITS_FLAG))
			{
				ret = -ENOSYS;
			}
			break;
		case IOSF_IOC_MODIFY:
			if (iosf_info.flag & IOSF_32BITS_FLAG)
			{
				ret = kiosf_reg_modify(host, iosf_info.dest_port, iosf_info.offset, iosf_info.mask, iosf_info.value);
			}
			else if(iosf_info.flag & (IOSF_16BITS_FLAG | IOSF_8BITS_FLAG))
			{
				ret = -ENOSYS;
			}
			break;
		case IOSF_IOC_MSG:
			ret = kiosf_msg(host, iosf_info.dest_port, iosf_info.opcode);
			break;
		case IOSF_IOC_MSG_DATA:
			ret = kiosf_msg_data(host, iosf_info.dest_port, iosf_info.opcode, iosf_info.value);
			break;
		default:
			break;

        }

        return ret;
}
static unsigned int iosf_poll(struct file *filp, struct poll_table_struct *table)
{
	return -ENOSYS;
}


/* Structure to map driver functions to kernel */
struct file_operations iosf_drv_fops = {
        .owner   = THIS_MODULE,
        .unlocked_ioctl   = iosf_unlocked_ioctl,
        .poll	 = iosf_poll,
        .open    = iosf_drv_open,
        .release = iosf_drv_close,
};

/*------------------------------------------------------------------------------
 * mdoule init
 *------------------------------------------------------------------------------
 */


int
iosf_drv_init(void)
{
	int ret = 0;
	if ((iosf_major = register_chrdev(0, devname, &iosf_drv_fops)) < 0)
	{
		ret = iosf_major;
		printk("iosf alloc character dev no failed!\n");
		return ret;
	 }

    return ret;

}

/*------------------------------------------------------------------------------
 * module exit
 *------------------------------------------------------------------------------
 */

void
iosf_drv_exit(void)
{

	unregister_chrdev(iosf_major, devname);
	iosf_major = 0;

}
