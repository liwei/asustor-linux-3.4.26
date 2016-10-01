
/*
 *  include/linux/ce_mailbox.h
 *
 *  GPL LICENSE SUMMARY
 *
 *  Copyright(c) 2012 Intel Corporation. All rights reserved.
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
#ifndef _CE_MBX_H_
#define _CE_MBX_H_

#include <linux/ioctl.h> 		/* needed for the _IOW etc stuff used later */

#define CE_MAILBOX_DEVICE_NAME "ce_mailbox"
#define DIGITS 80

/* RPC-IF structure */
struct npcpu_rpc_info
{
        unsigned int npcpu_ipv4_addr;
        unsigned int appcpu_ipv4_addr;
        unsigned int netmask;
        unsigned int vlan_id;
}__attribute__((packed));


struct npcpu_appcpu_mbx_user
{
	unsigned short  eventId;
	unsigned short  isParamRequired;
	struct npcpu_rpc_info parameter;
	unsigned int resv[2];		/* Reserved */
}__attribute__((packed));


enum npcpu_mbx_event_id
{
    NPCPU_EVENT_GPIO_INIT_EXIT   = 0x0001,
    NPCPU_EVENT_SPI_INIT_EXIT    = 0x0002,
    NPCPU_EVENT_EMMC_INIT_EXIT   = 0x0004,
    NPCPU_EVENT_RPC_IF_OBTAIN_ADDR = 0x0008,
    NPCPU_EVENT_EMMC_ADVANCE_INIT_EXIT = 0x0010
};

enum appcpu_mbx_event_id
{
    APPCPU_EVENT_RSVD   = 0x0001,
    APPCPU_EVENT_SPI_ADVANCE_EXIT    = 0x0002,
    APPCPU_EVENT_EMMC_ADVANCE_EXIT   = 0x0004
};

#define MBX_MODULE_ID 1
#define	MBX_SEND_EVENT_CMD           _IOW(MBX_MODULE_ID, 1, struct npcpu_appcpu_mbx_user )
#define	MBX_GET_EVENT_CMD            _IOR(MBX_MODULE_ID, 2, struct npcpu_appcpu_mbx_user )
#define	MBX_SEND_ACK_CMD             _IOW(MBX_MODULE_ID, 3,struct npcpu_appcpu_mbx_user )
#define	MBX_RECEIVE_ACK_CMD          _IOR(MBX_MODULE_ID, 4,struct npcpu_appcpu_mbx_user )
#define	MBX_REBOOT_EVENT_CMD          _IO(MBX_MODULE_ID, 5)

#define MBX_IOC_MAXNR	5

/* ATOM to DOCSIS interrupts */
#define BOOTCFG_REG_SW_INT_SET      (0x00000138)
#define BOOTCFG_REG_SW_INT_CLR      (0x0000013C)
#define BOOTCFG_REG_SW_INT_STAT     (0x00000140)
#define BOOTCFG_REG_SW_INT_ATOM_2_ARM11_INTC_MASK       be32_to_cpu(0x0000FFFF)
#define BOOTCFG_REG_SW_INT_ATOM_2_ARM11_INTC_REBOOT_ISR be32_to_cpu(0x00000001)
#define BOOTCFG_REG_SW_INT_ATOM_2_PP_COE_PrxPDSP_MASK   be32_to_cpu(0x00FF0000)
#define BOOTCFG_REG_SW_INT_ATOM_2_PP_COE_MASK          be32_to_cpu(0xFF000000)

/* DOCSIS to ATOM/PUnit interrupts */
#define BOOTCFG_REG_SW_INT1_STAT    (0x00000164)
#define BOOTCFG_REG_SW_INT1_SET     (0x00000168)
#define BOOTCFG_REG_SW_INT1_CLR     (0x0000016C)
#define BOOTCFG_REG_SW_INT1_ARM11_2_PUNIT_MASK     be32_to_cpu(0x000000FF)
#define BOOTCFG_REG_SW_INT1_ARM11_2_PUNIT_ISR      be32_to_cpu(0x00000001)
#define BOOTCFG_REG_SW_INT1_PP_2_PUNIT_MASK         be32_to_cpu(0x00000300)
#define BOOTCFG_REG_SW_INT1_ARM11_2_ATOM_MASK       be32_to_cpu(0xFFFF0000)
#define BOOTCFG_REG_SW_INT1_ARM11_2_ATOM_REBOOT_ISR be32_to_cpu(0x00010000)


#ifdef __KERNEL__

long npcpu_appcpu_mbx_receive_event_notification(unsigned short eventId, unsigned int *param);
long npcpu_appcpu_mbx_receive_specific_ack(unsigned short eventId);
long npcpu_appcpu_mbx_send_ack(unsigned short eventID);
long npcpu_appcpu_mbx_send_notification(unsigned short eventID, unsigned int *paramPtr);
void npcpu_bootcfg_ctrl_write_reg(uint32_t regOffset, uint32_t regVal);
uint32_t npcpu_bootcfg_ctrl_read_reg(uint32_t regOffset);


#endif
#endif	/* _NPCPU_APPCPU_MBX_H_ */
