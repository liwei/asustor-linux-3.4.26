/*
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

/* UDMA low level driver  */
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/device.h>
#include <asm/uaccess.h>
#include <linux/types.h>
#include <linux/errno.h>
#include <linux/pci.h>

#include "udma_hw.h"

#define SWITCH_RX_TX(direction,r,t) ((direction == UDMA_RX)?(r):(t))

#define UDMA_DRV_VERSION "1.0.0"
char udma_driver_name[] = "udma";
const char udma_driver_version[] = UDMA_DRV_VERSION;


static const struct __cntx_regset udma_cntx_rx_regset = {
		.cdesc = (UDMA_RX_REG_BASE + UDMA_CURR_DESC),
		.ndesc = (UDMA_RX_REG_BASE + UDMA_NEXT_DESC),
		.sdma  = (UDMA_RX_REG_BASE + UDMA_SRCDMA_START),
		.ddma  = (UDMA_RX_REG_BASE + UDMA_DSTDMA_START),
		.size  = (UDMA_RX_REG_BASE + UDMA_SRCDMA_SIZE),
		.flag  = UDMA_RX_REG_BASE + UDMA_FLAGS_MODE,
		.other = UDMA_RX_REG_BASE + UDMA_OTHER_MODE,
};

static const struct __cntx_regset udma_cntx_tx_regset = {
		.cdesc = UDMA_TX_REG_BASE + UDMA_CURR_DESC,
		.ndesc = UDMA_TX_REG_BASE + UDMA_NEXT_DESC,
		.sdma  = UDMA_TX_REG_BASE + UDMA_SRCDMA_START,
		.ddma  = UDMA_TX_REG_BASE + UDMA_DSTDMA_START,
		.size  = UDMA_TX_REG_BASE + UDMA_SRCDMA_SIZE,
		.flag  = UDMA_TX_REG_BASE + UDMA_FLAGS_MODE,
		.other = UDMA_TX_REG_BASE + UDMA_OTHER_MODE,
};

static const struct __intr_regset udma_intr_regset = {
		.int_mask = UDMA_INTR_MASK,
		.int_stat = UDMA_INTR_STATUS,
};


/* INTR enable register bits */
static const struct __port_regset udma_port_intr_unmask[] = {
	{
		UDMA_DST_INT0_EN, /* RX0, DMA channel 0 DST interrupt enable  when destination buffer is full */
		UDMA_SRC_INT1_EN, /* TX0, DMA channel 1 SRC interrupt enable when source buffer is empty */
	}, /* Port 0 */
	{
		UDMA_DST_INT2_EN, /* RX1, DMA channel 2 DST interrupt enable when destination buffer is full */
		UDMA_SRC_INT3_EN, /* TX1, DMA channel 3 SRC interrupt enable when source buffer is empty */
	} /* Port 1 */
};

static const struct __intr_status_bitset udma_intr_status[] = {
	{
		UDMA_DST_INT0_ACT_BIT, /* RX0, DMA channel 0 DST interrupt status bit offset when destination buffer is full */
		UDMA_SRC_INT1_ACT_BIT, /* TX0, DMA channel 1 SRC interrupt status bit offset when source buffer is empty */
	},
	{
		UDMA_DST_INT2_ACT_BIT, /* RX1, DMA channel 2 DST interrupt status bit offset when destination buffer is full */
		UDMA_SRC_INT3_ACT_BIT, /* TX1, DMA channel 3 SRC interrupt status bit offset when source buffer is empty */
	}
};

/* DMA address for port 0 & 1 */
static const struct __port_regset udma_port_dma[] = {
                {UDMA_CONTEXT0_DMA_ADDR,UDMA_CONTEXT1_DMA_ADDR}, /* Port 0 */
                {UDMA_CONTEXT2_DMA_ADDR,UDMA_CONTEXT3_DMA_ADDR}  /* Port 1 */
};
/* Port 0 & Port 1 shares the same cntx regs */
static const struct udma_cntx_regset udma_cntx_regs = {
	.rx 	= 	(struct __cntx_regset *)&udma_cntx_rx_regset,
	.tx 	= 	(struct __cntx_regset *)&udma_cntx_tx_regset,
	.intr 	= 	(struct __intr_regset *)&udma_intr_regset,
};





int  udma_setup_sw(void *dev);
void  udma_free_sw(void *dev);
struct udma_hw * udma_alloc_hw(size_t size);

static const struct pci_device_id udma_pci_tbl[] __devinitdata = {
        { PCI_DEVICE( 0x8086, UDMA_DEVICE_ID), .driver_data = 0 },
        {0},
};

MODULE_DEVICE_TABLE(pci, udma_pci_tbl);

void udma_desc_dump(struct udma_desc *desc)
{

	udma_info("\n	==========================\n");

	udma_info("\n	Current DESC 		0x%x\n", (u32)desc);
	udma_info("\n	Next DESC	 		0x%x\n", (u32)desc->next_desc);
	udma_info("\n	SRCDMA START	 	0x%x\n", desc->src);
	udma_info("\n	DSTDMA START	 	0x%x\n", desc->dst);
	udma_info("\n	SRCDMA SIZE		 	0x%x\n", desc->union_field.size);
	udma_info("\n	FLAGS MODE	 		0x%x\n", (u32)desc->flags);

}
void udma_regs_dump(struct udma_hw *hw, bool direction)
{
	struct __cntx_regset *regset = NULL;
	regset = SWITCH_RX_TX(direction,udma_cntx_regs.rx,udma_cntx_regs.tx);

	udma_info("	CONTEXT %s registers: ", SWITCH_RX_TX(direction,"Rx","Tx"));

	udma_info("		Current DESC(%x) 	0x%x\n", regset->cdesc, udma_readl(hw,regset->cdesc));
	udma_info("		Next DESC(%x)	 	0x%x\n", regset->ndesc, udma_readl(hw,regset->ndesc));
	udma_info("		SRCDMA START(%x) 	0x%x\n", regset->sdma, udma_readl(hw,regset->sdma));
	udma_info("		DSTDMA START(%x) 	0x%x\n", regset->ddma, udma_readl(hw,regset->ddma));
	udma_info("		SRCDMA SIZE(%x) 	0x%x\n", regset->size, udma_readl(hw,regset->size));
	udma_info("		FLAGS MODE(%x) 		0x%x\n", regset->flag, udma_readl(hw,regset->flag));
	udma_info("		OTHER MODE(%x) 		0x%x\n", regset->other, udma_readl(hw,regset->other));

	udma_info("	INTERRUPT registers: ");
	udma_info("		INTR MASK(%x) 		0x%x\n", udma_cntx_regs.intr->int_mask, udma_readl(hw,udma_cntx_regs.intr->int_mask));
	udma_info("		INTR STAT(%x) 		0x%x\n", udma_cntx_regs.intr->int_stat, udma_readl(hw,udma_cntx_regs.intr->int_stat));


}

/*
 * udma_is_active - check whether UDMA context is active or not
 * @direction: 0 for rx, 1 for tx
 *
 * return 1, when it's active;
 * return 0, when it's stop
*/

static  bool udma_hw_rx_is_active(struct udma_hw *hw)
{
	u32 value;

	value = udma_readl(hw, udma_cntx_regs.rx->flag);
	if (value & UDMA_DMA_IS_ACTIVE)
		return true;

	value = udma_readl(hw, udma_cntx_regs.rx->size);
	if ((value>>24) == DESC_HANDLING_FLAG)
		return true;
	return false;
}

static  bool udma_hw_tx_is_active(struct udma_hw *hw)
{
	u32 value;

	value = udma_readl(hw, udma_cntx_regs.tx->flag);
	if (value & UDMA_DMA_IS_ACTIVE)
		return true;

	value = udma_readl(hw, udma_cntx_regs.tx->size);
	if ((value>>24) == DESC_HANDLING_FLAG)
		return true;
	return false;
}


static u32 udma_hw_get_curr_rx_desc(struct udma_hw *hw)
{
	return udma_readl(hw, udma_cntx_regs.rx->cdesc);
}

static  u32 udma_hw_get_curr_tx_desc(struct udma_hw *hw)
{
	return udma_readl(hw, udma_cntx_regs.tx->cdesc);
}

static u32 udma_hw_get_next_rx_desc(struct udma_hw *hw)
{
	return udma_readl(hw, udma_cntx_regs.rx->ndesc);
}

static  u32 udma_hw_get_next_tx_desc(struct udma_hw *hw)
{
	return udma_readl(hw, udma_cntx_regs.tx->ndesc);
}



/*
 * udma_hw_irq_mask - mask interrupt
 * @direction - the transfer direction
 *    0 means RX, 1 means TX
*/
static int udma_hw_disable_rx_irq(struct udma_hw *hw)
{
	u32 value;

	value = udma_readl(hw, udma_cntx_regs.intr->int_mask);
	value &= ~udma_port_intr_unmask[hw->port].rx;
	udma_writel(hw, udma_cntx_regs.intr->int_mask, value);
	return 0;
}

static int udma_hw_disable_tx_irq(struct udma_hw *hw)
{
	u32 value;

	value = udma_readl(hw, udma_cntx_regs.intr->int_mask);
	value &= ~udma_port_intr_unmask[hw->port].tx;
	udma_writel(hw, udma_cntx_regs.intr->int_mask, value);

	return 0;
}

static int udma_hw_enable_rx_irq(struct udma_hw *hw)
{
	u32 value;

	value = udma_readl(hw, udma_cntx_regs.intr->int_mask);
	value |= udma_port_intr_unmask[hw->port].rx;
	udma_writel(hw, udma_cntx_regs.intr->int_mask, value);

	return 0;
}

static int udma_hw_enable_tx_irq(struct udma_hw *hw)
{
	u32 value;

	value = udma_readl(hw, udma_cntx_regs.intr->int_mask);
	value |= udma_port_intr_unmask[hw->port].tx;
	udma_writel(hw, udma_cntx_regs.intr->int_mask, value);

	return 0;
}

static int udma_hw_clear_rx_irq(struct udma_hw *hw)
{
	udma_writel(hw, udma_cntx_regs.intr->int_stat, udma_intr_status[hw->port].rx);
	return 0;
}

static int udma_hw_clear_tx_irq(struct udma_hw *hw)
{
	udma_writel(hw, udma_cntx_regs.intr->int_stat, udma_intr_status[hw->port].tx);
	return 0;
}

static  u32 udma_hw_get_irq_status(struct udma_hw *hw)
{
	 return udma_readl(hw,udma_cntx_regs.intr->int_stat);
}

static int udma_hw_enable_desc_rx_irq(struct udma_desc *desc)
{
	desc->flags |= UDMA_DST_INT_EN;
	return 0;
}

static int udma_hw_enable_desc_tx_irq(struct udma_desc *desc)
{
	desc->flags |= UDMA_SRC_INT_EN;
	return 0;
}

static int udma_hw_disable_desc_rx_irq(struct udma_desc *desc)
{
	desc->flags &= (~UDMA_DST_INT_EN);
	return 0;
}

static int  udma_hw_disable_desc_tx_irq(struct udma_desc *desc)
{
	desc->flags &= (~UDMA_SRC_INT_EN);
	return 0;
}


static inline int udma_hw_start_rx_transfer(struct udma_hw *hw, u32 desc_dma)
{
	udma_writel(hw, udma_cntx_regs.rx->other, UDMA_OTHER_MODE_DEFAULT_SETTING);
	udma_writel(hw, udma_cntx_regs.rx->ndesc, desc_dma);
	udma_writel(hw, udma_cntx_regs.rx->size, 0);
	udma_writel(hw, udma_cntx_regs.rx->flag, UDMA_FLAG_MODE_START_LL);
	return 0;
}

static inline int udma_hw_start_tx_transfer(struct udma_hw *hw, u32 desc_dma)
{
	udma_writel(hw, udma_cntx_regs.tx->other, UDMA_OTHER_MODE_DEFAULT_SETTING);
	udma_writel(hw, udma_cntx_regs.tx->ndesc, desc_dma);
	udma_writel(hw, udma_cntx_regs.tx->size, 0);
	udma_writel(hw, udma_cntx_regs.tx->flag, UDMA_FLAG_MODE_START_LL);
	return 0;
}

static inline int udma_hw_stop_rx_transfer(struct udma_hw *hw)
{
	u32 value;

	value = udma_readl(hw, udma_cntx_regs.rx->other);
	udma_writel(hw, udma_cntx_regs.rx->other, UDMA_STOP | value);
	return 0;
}

static inline int udma_hw_stop_tx_transfer(struct udma_hw *hw)
{
	u32 value;

	value = udma_readl(hw, udma_cntx_regs.tx->other);
	udma_writel(hw, udma_cntx_regs.tx->other, UDMA_STOP | value);
	return 0;
}


static inline bool udma_hw_rx_is_stopped(struct udma_hw *hw)
{
	u32 value;

	value = udma_readl(hw, udma_cntx_regs.rx->other);
	return !(value & UDMA_STOP);
}

static inline bool udma_hw_tx_is_stopped(struct udma_hw *hw)
{
	u32 value;

	value = udma_readl(hw, udma_cntx_regs.tx->other);
	return !(value & UDMA_STOP);
}

/* 	udma_hw_ll_desc_en : Enable a descriptor
 *
 * Steps:
 * 1. Update the src_dma & dst_dma field
 * 2. Update the size field
 **/
static void udma_hw_init_rx_desc(struct udma_hw *hw, struct udma_desc *desc)
{

	/* rx , enable interrupt at each descriptor */
	desc->src = udma_port_dma[hw->port].rx;
}

static void udma_hw_init_tx_desc(struct udma_hw *hw, struct udma_desc *desc)
{
	desc->dst = udma_port_dma[hw->port].tx;
}
static void udma_hw_update_rx_desc(struct udma_hw *hw, struct udma_desc *desc, u32 dma, u32 len)
{
	/* rx , enable interrupt at each descriptor */
	desc->dst = dma;
	desc->union_field.size = len & 0x00FFFFFF;
	desc->flags = UDMA_FLAG_MODE_RX_LL|UDMA_PACKET_POSITION_STARTING_ENDING;
}

static void udma_hw_update_tx_desc(struct udma_hw *hw, struct udma_desc *desc, u32 dma, u32 len)
{
	/* tx, do not set the interrupt, only enable at the last descriptor */
	desc->src = dma;
	desc->union_field.size = len & 0x00FFFFFF;
	desc->flags = UDMA_FLAG_MODE_TX_LL|UDMA_PACKET_POSITION_STARTING_ENDING;
}
static inline int  udma_hw_clear_desc(struct udma_desc *desc)
{
	memset((void *)((void *)desc + sizeof(u32)), 0, sizeof(struct udma_desc) - sizeof(u32));
	return 0;

}

static int udma_hw_init(struct udma_hw *hw)
{
	udma_hw_disable_tx_irq(hw);
	udma_hw_disable_rx_irq(hw);
	udma_writel(hw, udma_cntx_regs.rx->ndesc, 0);
	udma_writel(hw, udma_cntx_regs.tx->ndesc, 0);
	udma_hw_clear_tx_irq(hw);
	udma_hw_clear_rx_irq(hw);
	/* Set the other mode registers */
	udma_writel(hw, udma_cntx_regs.tx->other, UDMA_OTHER_MODE_DEFAULT_SETTING);
	udma_writel(hw, udma_cntx_regs.rx->other, UDMA_OTHER_MODE_DEFAULT_SETTING);


	return 0;
}

static int udma_hw_exit(struct udma_hw *hw)
{

	udma_hw_disable_tx_irq(hw);
	udma_hw_disable_rx_irq(hw);

	udma_hw_clear_tx_irq(hw);
	udma_hw_clear_rx_irq(hw);

	udma_writel(hw, udma_cntx_regs.rx->ndesc, 0);
	udma_writel(hw, udma_cntx_regs.tx->ndesc, 0);


	return 0;
}

/*****************************************************************************
 *
 * Initializaton/Exit
 *
*****************************************************************************/


static struct udma_hw_operations udma_hw_ops = {
	.rx_is_active				=	udma_hw_rx_is_active,
	.tx_is_active				=	udma_hw_tx_is_active,

	.get_curr_rx_desc			= 	udma_hw_get_curr_rx_desc,
	.get_curr_tx_desc			= 	udma_hw_get_curr_tx_desc,

	.get_next_rx_desc			= 	udma_hw_get_next_rx_desc,
	.get_next_tx_desc			= 	udma_hw_get_next_tx_desc,

	.start_rx_transfer			= 	udma_hw_start_rx_transfer,
	.start_tx_transfer			= 	udma_hw_start_tx_transfer,

	.stop_rx_transfer			=	udma_hw_stop_rx_transfer,
	.stop_tx_transfer			=	udma_hw_stop_tx_transfer,

	.rx_is_stopped				=	udma_hw_rx_is_stopped,
	.tx_is_stopped				=	udma_hw_tx_is_stopped,

	.init_rx_desc				=	udma_hw_init_rx_desc,
	.init_tx_desc				=	udma_hw_init_tx_desc,
	.update_rx_desc				=	udma_hw_update_rx_desc,
	.update_tx_desc				=	udma_hw_update_tx_desc,
	.clear_desc					=	udma_hw_clear_desc,

	.hw_init					= 	udma_hw_init,
	.hw_exit					=	udma_hw_exit,

	.enable_desc_rx_irq			=	udma_hw_enable_desc_rx_irq,
	.enable_desc_tx_irq			=	udma_hw_enable_desc_tx_irq,

	.disable_desc_rx_irq		=	udma_hw_disable_desc_rx_irq,
	.disable_desc_tx_irq		=	udma_hw_disable_desc_tx_irq,

	.get_irq_status				=	udma_hw_get_irq_status,

	.disable_rx_irq				=	udma_hw_disable_rx_irq,
	.enable_rx_irq				=	udma_hw_enable_rx_irq,
	.clear_rx_irq				=	udma_hw_clear_rx_irq,

	.disable_tx_irq				=	udma_hw_disable_tx_irq,
	.enable_tx_irq				=	udma_hw_enable_tx_irq,
	.clear_tx_irq				=	udma_hw_clear_tx_irq,
};

/**
 * udma_probe - Device Initialization Routine
 * Returns 0 on success, negative on failure
 *
 * udma_probe initializes an adapter identified by a pci_dev structure.
 * The OS initialization, configuring of the device private structure
 **/

static int __devinit udma_hw_probe(struct pci_dev *pdev,
				     const struct pci_device_id *ent)
{
	int ret = -ENODEV;
	struct udma_hw *hw = NULL;


	/* enable device */
	ret = pci_enable_device(pdev);
	if (ret) {
			dev_err(&pdev->dev, "pci_enable_device failed.\n");
			return ret;
	}

	hw = udma_alloc_hw(sizeof(struct udma_hw));
	if (IS_ERR(hw)) {
			dev_err(&pdev->dev, "Cannot allocate memory\n");
			ret = -ENOMEM;
			goto free_resource;
	}
	pci_request_region(pdev,0,(pdev->devfn>>3)?UDMA1_NAME:UDMA0_NAME);

	hw->ioaddr = (volatile void __iomem *)pci_ioremap_bar(pdev,0);
	if (!hw->ioaddr) {
		udma_err("error, failed to ioremap udma csr space\n");
		ret = -ENOMEM;
		goto free_mem;
	}
	/* DMA port map */
	hw->port = pdev->devfn>>3;
	hw->ops = &udma_hw_ops;
	hw->pdev = pdev;
	pci_set_drvdata(pdev, hw);


	//udma_dbg("udma port %d, ioaddr 0x%x, umdev 0x%x, pdev 0x%x\n", hw->port, hw->ioaddr, hw->private, hw->pdev);
	udma_hw_init(hw);
	ret = udma_setup_sw(udma_hw_priv(hw));
	if (ret)
		goto free_iomem;


	printk(KERN_INFO "Intel(R) UDMA Port %d Device Driver Init Done \n",hw->port);

	return 0;

free_iomem:
	iounmap(hw->ioaddr);
free_mem:
	kfree(hw);
free_resource:
	pci_release_regions(pdev);
	pci_disable_device(pdev);
	udma_err("udma HW probe failure \n");
	return ret;
}
/**
 * udma_remove - Device Removal Routine
 * udma_remove is called by the PCI subsystem to alert the driver
 * that it should release a PCI device.  The could be caused by a
 * Hot-Plug event, or because the driver is going to be removed from
 * memory.
 **/
void __devexit udma_hw_remove(struct pci_dev *pdev)
{
	struct udma_hw *hw = pci_get_drvdata(pdev);
	if (hw == NULL)
		return;
	udma_free_sw(udma_hw_priv(hw));
	udma_hw_exit(hw);
	pci_set_drvdata(pdev, NULL);
	iounmap(hw->ioaddr);
	kfree(hw);
	pci_release_region(pdev,0);
	pci_disable_device(pdev);
	printk(KERN_INFO "Intel(R) UDMA Device Driver Exit \n");
}

static struct pci_driver udma_driver = {
        .name           = udma_driver_name,
        .id_table       = udma_pci_tbl,
        .probe          = udma_hw_probe,
        .remove         = __devexit_p(udma_hw_remove),
};

/**
 * udma_init - Driver Registration Routine
 *
 **/
static int __init udma_drv_init(void)
{
	int ret;
	udma_info("Intel (R) UDMA Driver - %s\n", udma_driver_version);
	udma_info("Copyright (c) 2012 Intel Corperation. \n");
	ret = pci_register_driver(&udma_driver);

	return ret;
}


/**
 * udma_exit - Driver Exit Cleanup Routine
 *
 **/
static void __exit udma_drv_exit(void)
{
	pci_unregister_driver(&udma_driver);
}

module_init(udma_drv_init);
module_exit(udma_drv_exit);

MODULE_AUTHOR("Intel Corporation");
MODULE_DESCRIPTION("Intel(R) UDMA Driver");
MODULE_LICENSE("GPL");
MODULE_VERSION(UDMA_DRV_VERSION);




/* udma_hw.c */
