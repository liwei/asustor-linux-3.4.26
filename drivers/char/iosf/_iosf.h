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
 * File Name:_iosf.h
 * Driver for  IOSF(Intel On chip System Fabric)
 *------------------------------------------------------------------------------
 */

#ifndef _LINUX_DRV_IOSF_H
#define _LINUX_DRV_IOSF_H

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/pci.h>
#include <linux/slab.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/iosf_core.h>

#define CUNIT_SPR             0xD0
#define CUNIT_SDR             0xD4
#define CUNIT_SPRX            0xD8
#define CUNIT_SPRX2           0xDC

int  _common_iosf_reg_read32(struct iosf_host * host, uint8_t dest_port, uint8_t opcode, uint32_t offset, uint32_t *value);
int  _common_iosf_reg_write32(struct iosf_host * host, uint8_t dest_port, uint8_t opcode, uint32_t offset, uint32_t value);
int  _common_iosf_reg_modify(struct iosf_host * host, uint8_t dest_port, uint8_t rd_opcode, uint8_t wr_opcode,  uint32_t offset, uint32_t mask, uint32_t     value);

#endif //__LINUX_DRV_IOSF_H
