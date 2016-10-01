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
 * File Name:_ce5300_iosf.c
 * Driver for  IOSF(Intel On chip System Fabric)
 *------------------------------------------------------------------------------
 */


#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/pci.h>
#include <linux/slab.h>
#include <linux/err.h>
#include <linux/io.h>

#include <linux/iosf_core.h>
#include "iosf_common.h"

#include "_ce5300_iosf.h"
#include "_iosf.h"

int  _ce5300_iosf_port_is_valid(struct iosf_host * host, uint8_t dest_port)
{
	int ret = 0;

	switch (dest_port)
	{
		case IOSF_PORT_VT:
		case IOSF_PORT_MCU:
		case IOSF_PORT_PUNIT:
		case IOSF_PORT_CPUNIT:
		case IOSF_PORT_ITUNIT1:
		case IOSF_PORT_SAPms_BRIDGE:
		case IOSF_PORT_ITUNIT2:
		case IOSF_PORT_USB_AFE:
		case IOSF_PORT_ADAC:
		case IOSF_PORT_HDMI_TX_AFE:
		case IOSF_PORT_HDMI_RX_AFE:
		case IOSF_PORT_FPLL:
		case IOSF_PORT_HPLL:
		case IOSF_PORT_DPLL:
		case IOSF_PORT_APLL:
		case IOSF_PORT_PSF_0S:
		case IOSF_PORT_PSF_0N:
		case IOSF_PORT_PSF1:
		case IOSF_PORT_PSF3:

		case IOSF_PORT_HUNIT:
		case IOSF_PORT_BUNIT:
		case IOSF_PORT_DDR_IO:
		case IOSF_PORT_REUT0:
		case IOSF_PORT_REUT1:

		case IOSF_PORT_DFX_LAKEMORE:
		case IOSF_PORT_DFX_OMAR:
		case IOSF_PORT_SATA_AFE:
		case IOSF_PORT_PCIE_AFE:
			break;

		default:
				ret = -EINVAL;
	}

	return !ret;
}

int _ce5300_iosf_msg_opcode_is_valid(struct iosf_host * host, uint8_t opcode)
{
	int ret = false;

	if ( ((opcode >= 0x80) && (opcode <= 0x9F)) ||
		 ((opcode >= 0xA0) && (opcode <= 0xFF) ) )
		ret = true;
	return ret;
}

int _ce5300_iosf_msg_data_opcode_is_valid(struct iosf_host * host, uint8_t opcode)
{
	int ret = false;

	if ( ((opcode >= 0x40) && (opcode <= 0x5F)) ||
		 ((opcode >= 0x60) && (opcode <= 0x7F) ) )
		ret = true;
	return ret;
}

int _ce5300_iosf_reg_read32(struct iosf_host * host, uint8_t dest_port, uint32_t offset, uint32_t *value)
{
	uint8_t rd_opcode = 0;
	int ret = 0;


	switch (dest_port)
	{
		case IOSF_PORT_VT:
		case IOSF_PORT_MCU:
		case IOSF_PORT_PUNIT:
		case IOSF_PORT_CPUNIT:
		case IOSF_PORT_ITUNIT1:
		case IOSF_PORT_SAPms_BRIDGE:
		case IOSF_PORT_ITUNIT2:
		case IOSF_PORT_USB_AFE:
		case IOSF_PORT_ADAC:
		case IOSF_PORT_HDMI_TX_AFE:
		case IOSF_PORT_HDMI_RX_AFE:
		case IOSF_PORT_FPLL:
		case IOSF_PORT_HPLL:
		case IOSF_PORT_DPLL:
		case IOSF_PORT_APLL:
		case IOSF_PORT_PSF_0S:
		case IOSF_PORT_PSF_0N:
		case IOSF_PORT_PSF1:
		case IOSF_PORT_PSF3:
			rd_opcode = IOSF_OPCODE_CRRd;
			break;

		case IOSF_PORT_HUNIT:
		case IOSF_PORT_BUNIT:
		case IOSF_PORT_DDR_IO:
		case IOSF_PORT_REUT0:
		case IOSF_PORT_REUT1:
			rd_opcode = IOSF_OPCODE_RegRd;
			break;

		case IOSF_PORT_DFX_LAKEMORE:
		case IOSF_PORT_DFX_OMAR:
		case IOSF_PORT_SATA_AFE:
		case IOSF_PORT_PCIE_AFE:
			rd_opcode = IOSF_OPCODE_0_RegRd;
			break;

		default:
				ret = -EINVAL;


	}
	if (ret)
		return ret;

	return _common_iosf_reg_read32(host, dest_port, rd_opcode, offset, value);

}

int _ce5300_iosf_reg_write32(struct iosf_host * host, uint8_t dest_port, uint32_t offset, uint32_t value)
{
	uint8_t wr_opcode = 0;
	int ret = 0;

	switch (dest_port)
	{
		case IOSF_PORT_VT:
		case IOSF_PORT_MCU:
		case IOSF_PORT_PUNIT:
		case IOSF_PORT_CPUNIT:
		case IOSF_PORT_ITUNIT1:
		case IOSF_PORT_SAPms_BRIDGE:
		case IOSF_PORT_ITUNIT2:
		case IOSF_PORT_USB_AFE:
		case IOSF_PORT_ADAC:
		case IOSF_PORT_HDMI_TX_AFE:
		case IOSF_PORT_HDMI_RX_AFE:
		case IOSF_PORT_FPLL:
		case IOSF_PORT_HPLL:
		case IOSF_PORT_DPLL:
		case IOSF_PORT_APLL:
		case IOSF_PORT_PSF_0S:
		case IOSF_PORT_PSF_0N:
		case IOSF_PORT_PSF1:
		case IOSF_PORT_PSF3:
			wr_opcode = IOSF_OPCODE_CRWr;
			break;

		case IOSF_PORT_HUNIT:
		case IOSF_PORT_BUNIT:
		case IOSF_PORT_DDR_IO:
		case IOSF_PORT_REUT0:
		case IOSF_PORT_REUT1:
			wr_opcode = IOSF_OPCODE_RegWr;
			break;

		case IOSF_PORT_DFX_LAKEMORE:
		case IOSF_PORT_DFX_OMAR:
		case IOSF_PORT_SATA_AFE:
		case IOSF_PORT_PCIE_AFE:
			wr_opcode = IOSF_OPCODE_1_RegWr;
			break;

		default:
				ret = -EINVAL;

	}

	if (ret)
		return ret;

	return _common_iosf_reg_write32(host, dest_port,wr_opcode,  offset, value);

}

int _ce5300_iosf_reg_modify(struct iosf_host * host, uint8_t dest_port, uint32_t offset, uint32_t mask,  uint32_t value)
{
	uint8_t rd_opcode = 0;
	uint8_t wr_opcode = 0;
	int ret = 0;

	switch (dest_port)
	{
		case IOSF_PORT_VT:
		case IOSF_PORT_MCU:
		case IOSF_PORT_PUNIT:
		case IOSF_PORT_CPUNIT:
		case IOSF_PORT_ITUNIT1:
		case IOSF_PORT_SAPms_BRIDGE:
		case IOSF_PORT_ITUNIT2:
		case IOSF_PORT_USB_AFE:
		case IOSF_PORT_ADAC:
		case IOSF_PORT_HDMI_TX_AFE:
		case IOSF_PORT_HDMI_RX_AFE:
		case IOSF_PORT_FPLL:
		case IOSF_PORT_HPLL:
		case IOSF_PORT_DPLL:
		case IOSF_PORT_APLL:
		case IOSF_PORT_PSF_0S:
		case IOSF_PORT_PSF_0N:
		case IOSF_PORT_PSF1:
		case IOSF_PORT_PSF3:
			rd_opcode = IOSF_OPCODE_CRRd;
			wr_opcode = IOSF_OPCODE_CRWr;
			break;

		case IOSF_PORT_HUNIT:
		case IOSF_PORT_BUNIT:
		case IOSF_PORT_DDR_IO:
		case IOSF_PORT_REUT0:
		case IOSF_PORT_REUT1:
			rd_opcode = IOSF_OPCODE_RegRd;
			wr_opcode = IOSF_OPCODE_RegWr;
			break;

		case IOSF_PORT_DFX_LAKEMORE:
		case IOSF_PORT_DFX_OMAR:
		case IOSF_PORT_SATA_AFE:
		case IOSF_PORT_PCIE_AFE:
			rd_opcode = IOSF_OPCODE_0_RegRd;
			wr_opcode = IOSF_OPCODE_1_RegWr;
			break;

		default:
				ret = -EINVAL;
	}

	if (ret)
		return ret;
	return _common_iosf_reg_modify(host, dest_port, rd_opcode, wr_opcode, offset, mask, value);

}
