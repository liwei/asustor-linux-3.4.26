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
* File Name: iosf_common.h
*------------------------------------------------------------------------------
*/
//! \file
#ifndef  __LINUX_IOSF_COMMON_H
#define  __LINUX_IOSF_COMMON_H


#include <linux/types.h>

#define IOSF_MAGIC (('I' << 24) || ('O' << 16) | ('S' << 8) | ('F'))

struct iosf_info_user
{
	uint32_t  dest_port;
	uint32_t  opcode;
	uint32_t  offset;
	uint32_t  mask;
	uint32_t  flag;
	uint32_t  value;

};


// IOSF PORT Definitions:
#define IOSF_PORT_VT              0x00
#define IOSF_PORT_MCU             0x01

#define IOSF_PORT_HUNIT           0x02
#define IOSF_PORT_BUNIT           0x03

#define IOSF_PORT_PUNIT           0x04
#define IOSF_PORT_CPUNIT          0x0A

#define IOSF_PORT_PCIE_AFE        0x11

#define IOSF_PORT_DFX_LAKEMORE    0x38
#define IOSF_PORT_DFX_OMAR        0x39
#define IOSF_PORT_DFX_TG          0x3A
#define IOSF_PORT_ITUNIT1         0x40
#define IOSF_PORT_SAPms_BRIDGE    0x41
#define IOSF_PORT_ITUNIT2         0x42

#define IOSF_PORT_DDR_IO          0x50
#define IOSF_PORT_REUT0           0x54
#define IOSF_PORT_REUT1           0x55
#define IOSF_PORT_SATA_AFE        0x59
#define IOSF_PORT_USB_AFE         0x60


#define IOSF_PORT_ADAC            0x81
#define IOSF_PORT_HDMI_TX_AFE     0x82
#define IOSF_PORT_HDMI_RX_AFE     0x83

#define IOSF_PORT_FPLL            0x88
#define IOSF_PORT_HPLL            0x89
#define IOSF_PORT_DPLL            0x8A
#define IOSF_PORT_APLL            0x8B

#define IOSF_PORT_MOCA_AFE        0x8E
#define IOSF_PORT_MOCA_MAC        0x8F


#define IOSF_PORT_PSF_0S          0x90  /* south */
#define IOSF_PORT_PSF_0N		  0x32  /*north */


#define IOSF_PORT_PSF1            0x91

#define IOSF_PORT_PSF3            0x93



#define  IOSF_8BITS_FLAG		1
#define  IOSF_16BITS_FLAG		(1 << 1)
#define  IOSF_32BITS_FLAG		(1 << 2)

#define IOSF_IOC_MAGIC  'I'

#define IOSF_IOC_RD		_IOW(IOSF_IOC_MAGIC, 1, struct iosf_info_user)
#define IOSF_IOC_WR		_IOW(IOSF_IOC_MAGIC, 2, struct iosf_info_user)
#define IOSF_IOC_MODIFY		_IOW(IOSF_IOC_MAGIC, 3, struct iosf_info_user)
#define IOSF_IOC_MSG		_IOW(IOSF_IOC_MAGIC, 4, struct iosf_info_user)
#define IOSF_IOC_MSG_DATA	_IOW(IOSF_IOC_MAGIC, 5, struct iosf_info_user)






#endif // __LINUX_IOSF_COMMON_H
