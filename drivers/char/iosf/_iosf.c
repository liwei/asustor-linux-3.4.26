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
 * File Name:_iosf.c
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
#include "_ce5300_iosf.h"
#include "_ce2600_iosf.h"
#include "_iosf.h"

int iosf_drv_init(void);
void iosf_drv_exit(void);

int  iosf_core_init(void);
void iosf_core_exit(void);


/* vendor, device, subvendor, subdevice, class, class_mask, driver_data */
static const struct pci_device_id iosf_id_tables[] __devinitdata = {
	{ PCI_DEVICE( PCI_VENDOR_ID_INTEL, 0x0C40), .driver_data = 0},
	{ PCI_DEVICE( PCI_VENDOR_ID_INTEL, 0x0931), .driver_data = 0},
	{0 },
};
MODULE_DEVICE_TABLE(pci, iosf_id_tables);


struct iosf {
	struct iosf_host	host;
	spinlock_t lock;
	struct pci_dev *dev;
};

static inline void iosf_config_write32(struct pci_dev *dev,unsigned offset, u32 value)
{
 // 		iosf_dbg("iosf pci config write offset 0x%x, value:%x\n",offset, value);
		pci_write_config_dword(dev, offset, value);
}

static inline u32 iosf_config_read32(struct pci_dev *dev, unsigned offset)
{
		u32 value;
 // 		iosf_dbg("iosf pci config read offset 0x%x\n",offset);
		pci_read_config_dword(dev, offset,&value);
		return value;
}

static inline struct iosf *to_iosf(const struct iosf_host *p)
{
	return container_of(p, struct iosf, host);
}

int  _common_iosf_reg_read32(struct iosf_host * host, uint8_t dest_port, uint8_t opcode, uint32_t offset, uint32_t *value)
{
	struct iosf *iosf;
	struct pci_dev *dev;
	u32 tmp;
	unsigned long flags;

	iosf = to_iosf(host);
	dev = iosf->dev;
	spin_lock_irqsave(&iosf->lock, flags);

	tmp = (offset & (~0xFF));
	iosf_config_write32(dev, CUNIT_SPRX, tmp);/* write offset[31:9] into CUNIT_SPRX[31:8] */
	smp_mb();

	tmp = ((opcode << 24)|(dest_port << 16)|((offset & 0xFF) << 8)|(0xF << 4));
	iosf_config_write32(dev, CUNIT_SPR, tmp);/* write dest_port into CUNIT_SPR[31:24] opcode into CUNIT_SPR[23:16] */
	smp_mb();/* offset[7:0] into CUNIT_SPR[15:8], byte_enable[3:0] into CUNIT_SPR[7:4] */
	*value = iosf_config_read32(dev,CUNIT_SDR); /*read data out of CUNIT_SDR */

	spin_unlock_irqrestore(&iosf->lock, flags);
	return 0;
}



int  _common_iosf_reg_write32(struct iosf_host * host, uint8_t dest_port, uint8_t opcode, uint32_t offset, uint32_t value)
{

	struct iosf *iosf;
	struct pci_dev *dev;
	u32 tmp;
	unsigned long flags;

	iosf = to_iosf(host);
	dev = iosf->dev;
	spin_lock_irqsave(&iosf->lock, flags);

	iosf_config_write32(dev, CUNIT_SDR, value); /* write the data into CUNIT_SDR register */
	smp_wmb();

	tmp = (offset & (~0xFF));
	iosf_config_write32(dev, CUNIT_SPRX, tmp); /* write offset[31:9] into CUNIT_SPRX[31:8] */
	smp_wmb();

	tmp = ((opcode << 24)|(dest_port << 16)|((offset & 0xFF) << 8)|(0xF << 4));
	iosf_config_write32(dev, CUNIT_SPR, tmp); /* write dest_port into CUNIT_SPR[31:24] opcode into CUNIT_SPR[23:16] */
	smp_wmb();							/* offset[7:0] into CUNIT_SPR[15:8], byte_enable[3:0] into CUNIT_SPR[7:4] */

	spin_unlock_irqrestore(&iosf->lock, flags);
	return 0;
}


int  _common_iosf_reg_modify(struct iosf_host * host, uint8_t dest_port, uint8_t rd_opcode, uint8_t wr_opcode,  uint32_t offset, uint32_t mask, uint32_t value)
{
	struct iosf *iosf;
	struct pci_dev *dev;
	u32 tmp;
	unsigned long flags;

	iosf = to_iosf(host);
	dev = iosf->dev;
	spin_lock_irqsave(&iosf->lock, flags);

	/* read operation */
	tmp = (offset & (~0xFF));
	iosf_config_write32(dev, CUNIT_SPRX, tmp);/* write offset[31:9] into CUNIT_SPRX[31:8] */
	smp_mb();

	tmp = ((rd_opcode << 24)|(dest_port << 16)|((offset & 0xFF) << 8)|(0xF << 4));
	iosf_config_write32(dev, CUNIT_SPR, tmp);/* write dest_port into CUNIT_SPR[31:24] opcode into CUNIT_SPR[23:16] */
	smp_mb();/* offset[7:0] into CUNIT_SPR[15:8], byte_enable[3:0] into CUNIT_SPR[7:4] */

	tmp = iosf_config_read32(dev,CUNIT_SDR); /*read data out of CUNIT_SDR */

	/* mask operation */
	value = ((value & mask) | (tmp & (~mask)));

	/*write operation */
	iosf_config_write32(dev, CUNIT_SDR, value); /* write the data into CUNIT_SDR register */
	smp_wmb();

	tmp = (offset & (~0xFF));
	iosf_config_write32(dev, CUNIT_SPRX, tmp); /* write offset[31:9] into CUNIT_SPRX[31:8] */
	smp_wmb();

	tmp = ((wr_opcode << 24)|(dest_port << 16)|((offset & 0xFF) << 8)|(0xF << 4));
	iosf_config_write32(dev, CUNIT_SPR, tmp); /* write dest_port into CUNIT_SPR[31:24] opcode into CUNIT_SPR[23:16] */
	smp_wmb();							/* offset[7:0] into CUNIT_SPR[15:8], byte_enable[3:0] into CUNIT_SPR[7:4] */

	spin_unlock_irqrestore(&iosf->lock, flags);
	return 0;
}

int  _common_iosf_msg(struct iosf_host * host, uint8_t dest_port, uint8_t opcode)
{

	struct iosf *iosf;
	struct pci_dev *dev;
	u32 tmp;
	unsigned long flags;

	iosf = to_iosf(host);
	dev = iosf->dev;
	spin_lock_irqsave(&iosf->lock,flags);

	tmp = ((opcode << 24)|(dest_port << 16));
	iosf_config_write32(dev, CUNIT_SPR, tmp); /* write dest_port into CUNIT_SPR[31:24] opcode into CUNIT_SPR[23:16] */
	smp_wmb();							/* offset[7:0] into CUNIT_SPR[15:8], byte_enable[3:0] into CUNIT_SPR[7:4] */

	spin_unlock_irqrestore(&iosf->lock, flags);
	return 0;
}

int  _common_iosf_msg_data(struct iosf_host * host, uint8_t dest_port, uint8_t opcode, uint32_t data)
{

	struct iosf *iosf;
	struct pci_dev *dev;
	u32 tmp;
	unsigned long flags;

	iosf = to_iosf(host);
	dev = iosf->dev;
	spin_lock_irqsave(&iosf->lock,flags);

	iosf_config_write32(dev, CUNIT_SDR, data); /* write the data into CUNIT_SDR register */
	smp_wmb();

	tmp = ((opcode << 24)|(dest_port << 16));
	iosf_config_write32(dev, CUNIT_SPR, tmp); /* write dest_port into CUNIT_SPR[31:24] opcode into CUNIT_SPR[23:16] */
	smp_wmb();							/* offset[7:0] into CUNIT_SPR[15:8], byte_enable[3:0] into CUNIT_SPR[7:4] */

	spin_unlock_irqrestore(&iosf->lock, flags);
	return 0;
}

#ifdef CONFIG_PM
static int iosf_device_suspend(struct device *dev)
{
	//struct pci_dev *pdev = to_pci_dev(dev);
	int ret = 0;

	return ret;
}

static int iosf_device_resume(struct device *dev)
{
	//struct pci_dev *pdev = to_pci_dev(dev);
	int ret = 0;

	return ret;
}
static const struct dev_pm_ops iosf_pm_ops = {
	.suspend    = iosf_device_suspend,
	.resume     = iosf_device_resume,
};
#endif


static int __devinit iosf_probe(struct pci_dev *pdev, const struct pci_device_id *id)
{

	struct iosf *iosf;
	int ret = 0;

	pci_request_regions(pdev, "iosf");

	iosf = kzalloc(sizeof *iosf, GFP_KERNEL);
	if (IS_ERR(iosf)) {
		dev_err(&pdev->dev, "failed to allocate memory\n");
		ret = -ENOMEM;
		goto free_mem_region;
	}


	spin_lock_init(&iosf->lock);
	iosf->dev = pdev;
	iosf->host.bus_id = (uint32_t)id->driver_data;
	iosf->host.owner = THIS_MODULE;

	switch (id->device) {
		case  CE5300_SOC_DEVICE_ID:
			iosf->host.port_is_valid = _ce5300_iosf_port_is_valid;
			iosf->host.msg_opcode_is_valid = _ce5300_iosf_msg_opcode_is_valid;
			iosf->host.msg_data_opcode_is_valid = _ce5300_iosf_msg_data_opcode_is_valid;
			iosf->host.reg_read32= _ce5300_iosf_reg_read32;
			iosf->host.reg_write32= _ce5300_iosf_reg_write32;
			iosf->host.reg_modify= _ce5300_iosf_reg_modify;
			iosf->host.msg= _common_iosf_msg;
			iosf->host.msg_data= _common_iosf_msg_data;
			break;

		case CE2600_SOC_DEVICE_ID:
			iosf->host.port_is_valid = _ce2600_iosf_port_is_valid;
			iosf->host.msg_opcode_is_valid = _ce2600_iosf_msg_opcode_is_valid;
			iosf->host.msg_data_opcode_is_valid = _ce2600_iosf_msg_data_opcode_is_valid;
			iosf->host.reg_read32= _ce2600_iosf_reg_read32;
			iosf->host.reg_write32= _ce2600_iosf_reg_write32;
			iosf->host.reg_modify= _ce2600_iosf_reg_modify;
			iosf->host.msg= _common_iosf_msg;
			iosf->host.msg_data= _common_iosf_msg_data;
			break;

		default:
			ret = -ENODEV;
	}

	if (ret)
		goto free_mem;

	ret =  iosf_register(&iosf->host);

	if (ret)
		goto free_mem;

	pci_set_drvdata(pdev, iosf);
	return 0;


free_mem:
	kfree(iosf);

free_mem_region:
	pci_release_regions(pdev);

	pci_set_drvdata(pdev, NULL);
	return ret;
}

static void __devexit iosf_remove(struct pci_dev *pdev)
{
	struct iosf *iosf = pci_get_drvdata(pdev);

	iosf_unregister(&iosf->host);
	pci_release_regions(pdev);
	kfree(iosf);
	pci_set_drvdata(pdev, NULL);

}

static struct pci_driver iosf_driver = {
	.name	= "iosf-sb",
	.probe		= iosf_probe,
	.remove		= __devexit_p(iosf_remove),
	.id_table	= iosf_id_tables,
#ifdef CONFIG_PM
	.driver.pm		= &iosf_pm_ops,
#endif
};

static int __init iosf_init(void)
{
	iosf_core_init();
	iosf_drv_init();
	return pci_register_driver(&iosf_driver);
}

static void __exit iosf_exit(void)
{
	pci_unregister_driver(&iosf_driver);
	iosf_drv_exit();
	iosf_core_exit();
}

module_init(iosf_init);
module_exit(iosf_exit);
