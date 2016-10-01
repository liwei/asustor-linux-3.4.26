/*
#
#  This file is provided under a dual BSD/GPLv2 license.  When using or
#  redistributing this file, you may do so under either license.
#
#  GPL LICENSE SUMMARY
#
#  Copyright(c) 2010-2012 Intel Corporation. All rights reserved.
#
#  This program is free software; you can redistribute it and/or modify
#  it under the terms of version 2 of the GNU General Public License as
#  published by the Free Software Foundation.
#
#  This program is distributed in the hope that it will be useful, but
#  WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
#  General Public License for more details.
#
#  You should have received a copy of the GNU General Public License
#  along with this program; if not, write to the Free Software
#  Foundation, Inc., 51 Franklin St - Fifth Floor, Boston, MA 02110-1301 USA.
#  The full GNU General Public License is included in this distribution
#  in the file called LICENSE.GPL.
#
#  Contact Information:
#  intel.com
#  Intel Corporation
#  2200 Mission College Blvd.
#  Santa Clara, CA  95052
#  USA
#  (408) 765-8080
#
#
#  BSD LICENSE
#
#  Copyright(c) 2010-2012 Intel Corporation. All rights reserved.
#  All rights reserved.
#
#  Redistribution and use in source and binary forms, with or without
#  modification, are permitted provided that the following conditions
#  are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in
#      the documentation and/or other materials provided with the
#      distribution.
#    * Neither the name of Intel Corporation nor the names of its
#      contributors may be used to endorse or promote products derived
#      from this software without specific prior written permission.
#
#  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
#  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
#  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
#  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
#  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
#  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
#  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
#  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
#  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
#  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
#  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#
#*/
/*------------------------------------------------------------------------------
 * File Name:iosf_core.h
 * Driver for  IOSF(Intel On chip System Fabric)
 *------------------------------------------------------------------------------
 */
#ifndef _LINUX_DRIVER_IOSF_CORE_H
#define _LINUX_DRIVER_IOSF_CORE_H

#define DEBUG
#ifdef DEBUG
#define iosf_dbg(fmt, args...) do \
    { \
	            printk(KERN_INFO fmt, ##args); \
	    } while(0)
#else
#define iosf_dbg(fmt, arg...) do { } while (0)
#endif


struct iosf_host {
	struct list_head list;
	uint32_t bus_id;
	struct module *owner;
	atomic_t users;
	int (*port_is_valid)(struct iosf_host *host, uint8_t dest_port);
	int (*msg_opcode_is_valid)(struct iosf_host *host, uint8_t opcode);
	int (*msg_data_opcode_is_valid)(struct iosf_host *host, uint8_t opcode);
	int (*reg_read32)(struct iosf_host *host, uint8_t dest_port, uint32_t offset, uint32_t *value);
	int (*reg_write32)(struct iosf_host *host, uint8_t dest_port,uint32_t offset, uint32_t value);
	int (*reg_modify)(struct iosf_host *host, uint8_t dest_port, uint32_t offset, uint32_t mask, uint32_t value);
	int (*msg)(struct iosf_host *host, uint8_t dest_port, uint8_t opcode);
	int (*msg_data)(struct iosf_host *host, uint8_t dest_port, uint8_t opcode, uint32_t data);

};

// OPCODE Definitions:
#define IOSF_OPCODE_CRRd            0x06
#define IOSF_OPCODE_CRWr            0x07
#define IOSF_OPCODE_RegRd 	 		0x10
#define IOSF_OPCODE_RegWr			0x11

#define IOSF_OPCODE_0_RegRd 	    0x0
#define IOSF_OPCODE_1_RegWr			0x1

#define IOSF_OPCODE_4_RegRd 	    0x4
#define IOSF_OPCODE_5_RegWr			0x5

#define IOSF_OPCODE_2_RegRd 	    0x2
#define IOSF_OPCODE_3_RegWr			0x3


int iosf_register(struct iosf_host *host);
int iosf_unregister(struct iosf_host *host);

struct iosf_host *iosf_request(uint32_t bus_id);
void iosf_release(struct iosf_host *host);


int kiosf_reg_read32(struct iosf_host *host, uint8_t dest_port, uint32_t offset, uint32_t *value);
int kiosf_reg_write32(struct iosf_host *host, uint8_t dest_port, uint32_t offset, uint32_t value);
int kiosf_reg_modify(struct iosf_host *host, uint8_t dest_port, uint32_t offset, uint32_t mask, uint32_t value);
int kiosf_msg(struct iosf_host *host, uint8_t dest_port, uint8_t opcode);
int kiosf_msg_data(struct iosf_host *host, uint8_t dest_port, uint8_t opcode, uint32_t data);


#endif /* __LINUX_DRIVER_IOSF_CORE_H */
