/*
 *  GPL LICENSE SUMMARY
 *
 *  Copyright(c) 2011-2013 Intel Corporation. All rights reserved.
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
 * Derived from
 * UIO Hilscher CIF card driver
 * (C) 2007 Hans J. Koch <hjk@hansjkoch.de>
 * Original code (C) 2005 Benedikt Spranger <b.spranger@linutronix.de>
 *
 */

/*
 * UIO Intel CE2600 driver for MoCA controller and L2 Switch
 */

#include <linux/device.h>
#include <linux/module.h>
#include <linux/pci.h>
#include <linux/slab.h>
#include <linux/uio_driver.h>


#define PCI_DEVICE_ID_INTEL_MOCA 0x08BE
#define	PCI_DEVICE_ID_INTEL_L2SW 0x08BD


static irqreturn_t ce2600_irqhandler(int irq, struct uio_info *dev_info)
{
	return IRQ_NONE;
}

static int __devinit ce2600_pci_probe(struct pci_dev *dev,
					const struct pci_device_id *id)
{
	struct uio_info *info;

	info = kzalloc(sizeof(struct uio_info), GFP_KERNEL);
	if (!info)
		return -ENOMEM;

	if (pci_enable_device(dev))
		goto out_free;

	if(dev->device == PCI_DEVICE_ID_INTEL_MOCA)
		info->name = "MOCA";
	else if (dev->device == PCI_DEVICE_ID_INTEL_L2SW)
		info->name = "L2SW";
	else
		goto out_disable;

	if (pci_request_regions(dev, info->name))
		goto out_disable;

	info->mem[0].addr = pci_resource_start(dev, 0);
	if (!info->mem[0].addr)
		goto out_release;

	info->mem[0].size = pci_resource_len(dev, 0);
	info->mem[0].memtype = UIO_MEM_LOGICAL;
	info->mem[0].name = "IRAM-DRAM";
	info->version = "0.0.1";
	info->irq = dev->irq;
	info->irq_flags = IRQF_SHARED;
	info->handler = ce2600_irqhandler;

	if (uio_register_device(&dev->dev, info))
		goto out_release;

	pci_set_drvdata(dev, info);
	info->priv = dev;

	return 0;
out_release:
	pci_release_regions(dev);
out_disable:
	pci_disable_device(dev);
out_free:
	kfree (info);
	return -ENODEV;
}

static void ce2600_pci_remove(struct pci_dev *dev)
{
	struct uio_info *info = pci_get_drvdata(dev);

	uio_unregister_device(info);
	pci_release_regions(dev);
	pci_disable_device(dev);
	pci_set_drvdata(dev, NULL);
#ifdef HAVE_ADDR
	iounmap(info->mem[0].internal_addr);
#endif

	kfree (info);
}

static struct pci_device_id ce2600_pci_ids[] __devinitdata = {
	{ PCI_VDEVICE(INTEL, PCI_DEVICE_ID_INTEL_MOCA) },
	{ PCI_VDEVICE(INTEL, PCI_DEVICE_ID_INTEL_L2SW) },
	{ 0, }
};

static struct pci_driver ce2600_pci_driver = {
	.name = "uio_ce2600",
	.id_table = ce2600_pci_ids,
	.probe = ce2600_pci_probe,
	.remove = ce2600_pci_remove,
};

static int __init uio_ce2600_init_module(void)
{
	return pci_register_driver(&ce2600_pci_driver);
}

static void __exit uio_ce2600_exit_module(void)
{
	pci_unregister_driver(&ce2600_pci_driver);
}

module_init(uio_ce2600_init_module);
module_exit(uio_ce2600_exit_module);

MODULE_DEVICE_TABLE(pci, ce2600_pci_ids);
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Intel");
