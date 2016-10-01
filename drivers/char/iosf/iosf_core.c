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
 * File Name: iosf_core.c
 *------------------------------------------------------------------------------
 */


#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/device.h>
#include <linux/spinlock.h>
#include <linux/fs.h>
#include <linux/completion.h>
#include <linux/workqueue.h>
#include <linux/list.h>
#include <linux/sched.h>
#include <asm/atomic.h>


#include <linux/iosf_core.h>
#include "iosf_common.h"

static gid_t gid = 510;
module_param(gid, uint, S_IRUGO | S_IWUSR);

struct class *iosf_class;

static LIST_HEAD(iosf_host_list);
static DEFINE_SPINLOCK(host_list_lock);

int iosf_register(struct iosf_host *iosf)
{
	atomic_set(&iosf->users, 0);
	spin_lock(&host_list_lock);
	list_add_tail(&iosf->list, &iosf_host_list);
	spin_unlock(&host_list_lock);

	return 0;

}
EXPORT_SYMBOL(iosf_register);

int iosf_unregister(struct iosf_host *iosf)
{
	if (atomic_read (&iosf->users) > 0)
		return -EBUSY;

	spin_lock(&host_list_lock);
	list_del(&iosf->list);
	spin_unlock(&host_list_lock);
	return 0;
}
EXPORT_SYMBOL(iosf_unregister);

struct iosf_host *iosf_request(uint32_t bus_id)
{
	struct iosf_host *p;

	list_for_each_entry(p, &iosf_host_list, list)
	{
		if (bus_id == p->bus_id)
		{
			atomic_inc(&p->users);
			return p;
		}
	}
	return NULL;

}
EXPORT_SYMBOL(iosf_request);

void iosf_release(struct iosf_host *host)
{
	atomic_dec(&host->users);
}

EXPORT_SYMBOL(iosf_release);


int  kiosf_reg_read32(struct iosf_host *iosf, uint8_t dest_port, uint32_t offset, uint32_t *value)
{
	if ((0 != current_euid()) && (gid != current_egid())) return -EPERM;

	if (!iosf->port_is_valid(iosf, dest_port))
		return -EINVAL;

	return iosf->reg_read32(iosf, dest_port, offset, value);

}
EXPORT_SYMBOL(kiosf_reg_read32);

int  kiosf_reg_write32(struct iosf_host *iosf, uint8_t dest_port, uint32_t offset, uint32_t value)
{
	if ((0 != current_euid()) && (gid != current_egid())) return -EPERM;

	if (!iosf->port_is_valid(iosf, dest_port))
		return -EINVAL;

	return iosf->reg_write32(iosf, dest_port, offset, value);

}
EXPORT_SYMBOL(kiosf_reg_write32);

int  kiosf_reg_modify(struct iosf_host *iosf, uint8_t dest_port, uint32_t offset, uint32_t mask,  uint32_t value)
{
	if ((0 != current_euid()) && (gid != current_egid())) return -EPERM;

	if (!iosf->port_is_valid(iosf, dest_port))
		return -EINVAL;

	return iosf->reg_modify(iosf, dest_port, offset, mask, value);

}
EXPORT_SYMBOL(kiosf_reg_modify);

int kiosf_msg(struct iosf_host *iosf, uint8_t dest_port, uint8_t opcode)
{
	if ((0 != current_euid()) && (gid != current_egid())) return -EPERM;

	if (!iosf->port_is_valid(iosf, dest_port) || !iosf->msg_opcode_is_valid(iosf, opcode))
		return -EINVAL;

	return iosf->msg(iosf, dest_port, opcode);

}
EXPORT_SYMBOL(kiosf_msg);

int kiosf_msg_data(struct iosf_host *iosf, uint8_t dest_port, uint8_t opcode, uint32_t data)
{
	if ((0 != current_euid()) && (gid != current_egid())) return -EPERM;

	if (!iosf->port_is_valid(iosf, dest_port) || !iosf->msg_data_opcode_is_valid(iosf, opcode))
		return -EINVAL;

	return iosf->msg_data(iosf, dest_port, opcode, data);

}
EXPORT_SYMBOL(kiosf_msg_data);

int __init iosf_core_init(void)
{

	iosf_class = class_create(THIS_MODULE, "iosf");
	if (IS_ERR(iosf_class))
	{
		printk(KERN_ERR "iosf class create failed!\n");
		return -1;
	}
	return 0;
}

void __exit iosf_core_exit(void)
{
	class_destroy(iosf_class);

}
