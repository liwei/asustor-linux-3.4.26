/*
 *  GPIO interface for Intel CE SoCs.
 *
 *  Copyright (c) 2010, 2012 Intel Corporation
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License 2 as published
 *  by the Free Software Foundation.
 *
 */

#include <linux/errno.h>
#include <linux/gpio.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/pci.h>
#include <linux/platform_device.h>

#include "intelce-gpio.h"
#include "ce4100-gpio.h"
#include "ce4200-gpio.h"
#include "ce5300-gpio.h"
#include "ce2600-gpio.h"

static int banks, gpios_per_bank;
static char *label[CE5300_PUB_GPIO_BANKS] = {
	"gpio_bank0",
	"gpio_bank1",
	"gpio_bank2",
	"gpio_bank3",
};

static struct resource sch_gpio_resource[] = {
	[0] = {
		.start = 1048,
		.end   = 1048 + 64 - 1,
		.flags = IORESOURCE_IO,
	},
	[1] = {
		.start = 0,
		.end   = 0,
		.flags = IORESOURCE_IRQ,
	},
};
static struct platform_device sch_device_gpio = {
	.name		  = "sch_gpio",
	.id		  = PCI_DEVICE_ID_INTEL_SCH_LPC,
	.num_resources	  = ARRAY_SIZE(sch_gpio_resource),
	.resource	  = sch_gpio_resource,
};

static uint16_t  legacy_iobase = 1048;
static int __devinit sch_gpio_setup(struct pci_dev *pdev, int gpio_base)
{
    struct pci_dev *lpc;

	sch_device_gpio.dev.parent = get_device(&pdev->dev);

    lpc = pci_get_bus_and_slot(0,PCI_DEVFN(31, 0));
    if (NULL == lpc) {
       printk(KERN_ERR "Can't detect the  LPC PCI device!!");
       return -ENODEV;
    }
    pci_read_config_word(lpc, 0x44, &legacy_iobase);
    pci_dev_put(lpc);
	legacy_iobase &= ~(64 - 1);
	sch_gpio_resource[0].start = legacy_iobase;
	sch_gpio_resource[0].end = legacy_iobase + 64 - 1;
	sch_gpio_resource[1].start = gpio_base;
	sch_gpio_resource[1].end = gpio_base;

	return platform_device_register(&sch_device_gpio);
}

static int (*intelce_gpio_suspend)(void *io_mem, unsigned short io_port);
static int (*intelce_gpio_resume)(void *io_mem, unsigned short io_port);

static int __devinit intelce_gpio_probe(struct pci_dev *pdev, const struct pci_device_id *pci_id)
{
	struct intelce_gpio_chip *c, *p;
	unsigned long paddr;
	void *vaddr;
	int i;
	unsigned int id;
	int gpio_base = 0;
	int mux_ctl_offset = CE4100_PUB_GPIO_MUX_CTL;
	int (*gpio_set_multi_function)(struct gpio_chip *chip, unsigned offset, int fn_num);
	int (*gpio_get_multi_function)(struct gpio_chip *chip, unsigned offset);
	void (*gpio_set)(struct gpio_chip *chip, unsigned offset, int value);
	int (*gpio_direction_output)(struct gpio_chip *chip, unsigned offset, int value);
	int (*gpio_direction_input)(struct gpio_chip *chip, unsigned offset);
	int (*gpio_irq_setup)(struct intelce_gpio_chip *c, struct pci_dev *pdev);
	int ret;


	intelce_get_soc_info(&id, NULL);
	switch(id) {
		case CE4100_SOC_DEVICE_ID:
			banks = CE4100_PUB_GPIO_BANKS;
			gpios_per_bank = CE4100_PUB_GPIOS_PER_BANK;
			mux_ctl_offset = CE4100_PUB_GPIO_MUX_CTL;
			gpio_set_multi_function = ce4100_set_multi_function;
			gpio_get_multi_function = ce4100_get_multi_function;
			gpio_set = ce4100_gpio_set;
			gpio_direction_output = ce4100_gpio_direction_output;
			gpio_direction_input = ce4100_gpio_direction_input;
			gpio_irq_setup = ce4100_gpio_irq_setup;
			dev_info(&pdev->dev, "CE4100 GPIO controller detected.\n");
			break;
		case CE4200_SOC_DEVICE_ID:
			banks = CE4200_PUB_GPIO_BANKS;
			gpios_per_bank = CE4200_PUB_GPIOS_PER_BANK;
			mux_ctl_offset = CE4200_PUB_GPIO_MUX_CTL;
			gpio_set_multi_function = ce4200_set_multi_function;
			gpio_get_multi_function = ce4200_get_multi_function;
			gpio_set = ce4100_gpio_set;
			gpio_direction_output = ce4100_gpio_direction_output;
			gpio_direction_input = ce4100_gpio_direction_input;
			gpio_irq_setup = ce4200_gpio_irq_setup;
			intelce_gpio_suspend = ce4200_gpio_suspend;
			intelce_gpio_resume =ce4200_gpio_resume;
			dev_info(&pdev->dev, "CE4200 GPIO controller detected.\n");
			break;
		case CE5300_SOC_DEVICE_ID:
			banks = CE5300_PUB_GPIO_BANKS;
			gpios_per_bank = CE5300_PUB_GPIOS_PER_BANK;
			mux_ctl_offset = CE5300_PUB_GPIO_MUX_CTL;
			gpio_set_multi_function = ce5300_set_multi_function;
			gpio_get_multi_function = ce5300_get_multi_function;
			gpio_set = ce4100_gpio_set;
			gpio_direction_output = ce4100_gpio_direction_output;
			gpio_direction_input = ce4100_gpio_direction_input;
			gpio_irq_setup = ce5300_gpio_irq_setup;
			intelce_gpio_suspend = ce5300_gpio_suspend;
			intelce_gpio_resume =ce5300_gpio_resume;
			dev_info(&pdev->dev, "CE5300 GPIO controller detected.\n");
			break;
		case CE2600_SOC_DEVICE_ID:
			banks = CE2600_PUB_GPIO_BANKS;
			gpios_per_bank = CE2600_PUB_GPIOS_PER_BANK;
			mux_ctl_offset = CE2600_PUB_GPIO_MUX_CTL;
			gpio_set_multi_function = ce2600_set_multi_function;
			gpio_get_multi_function = ce2600_get_multi_function;
			gpio_set = ce2600_gpio_set;
			gpio_direction_output = ce2600_gpio_direction_output;
			gpio_direction_input = ce2600_gpio_direction_input;
			gpio_irq_setup = ce2600_gpio_irq_setup;
			intelce_gpio_suspend = NULL;
			intelce_gpio_resume = NULL;
			dev_info(&pdev->dev, "CE2600 GPIO controller detected.\n");
			break;
		default:
		dev_err(&pdev->dev, "Does not support the platform\n");
			return -ENODEV;
	}

	c = kzalloc(sizeof(struct intelce_gpio_chip)*banks, GFP_KERNEL);
	if (!c)
		return -ENOMEM;
	ret = pci_enable_device(pdev);
	if (ret) {
		dev_err(&pdev->dev, "can't enable device.\n");
		goto done;
	}

	ret = pci_request_region(pdev, INTELCE_GPIO_BAR, INTELCE_GPIO_DRV_NAME);
	if (ret) {
		dev_err(&pdev->dev, "can't alloc PCI BAR #%d\n", INTELCE_GPIO_BAR);
		goto disable_pci;
	}

	paddr = pci_resource_start(pdev, INTELCE_GPIO_BAR);
	if (!paddr)
		goto release_reg;
	vaddr = ioremap(paddr, pci_resource_len(pdev, INTELCE_GPIO_BAR));

	for (i=0; i < banks; i++) {
		p = c + i;
		spin_lock_init(&p->lock);
		p->reg_base = vaddr + 0x20*i;
		p->high_base = vaddr + 0x80 + 0x10*i;
		p->mux_ctl_base = vaddr + mux_ctl_offset;
		if ((CE4200_SOC_DEVICE_ID == id) && ((CE4200_PUB_GPIO_BANKS-1)) == i) {

			p->chip.ngpio = gpios_per_bank - 18;
		}
		else {
			p->chip.ngpio = gpios_per_bank;
		}
		p->chip.set_multi_function = gpio_set_multi_function;
		p->chip.get_multi_function = gpio_get_multi_function;
		p->chip.set = gpio_set;
		p->chip.get = ce4100_gpio_get;
		p->chip.direction_output = gpio_direction_output;
		p->chip.direction_input = gpio_direction_input;
		p->chip.to_irq = ce4100_gpio_to_irq;
		p->chip.label = label[i];
		p->chip.base = gpio_base;
		ret = gpiochip_add(&p->chip);
		if (ret)
			goto unmap;
		ret = gpio_irq_setup(p, pdev);
		if (ret) {
			gpiochip_remove(&p->chip);
			goto unmap;
		}
		gpio_base += p->chip.ngpio;
	}
	sch_gpio_setup(pdev, gpio_base);
	pci_set_drvdata(pdev, c);
	return 0;

unmap:
	for (i--; i >= 0; i--) {
		p = c + i ;
		free_irq(pdev->irq, p);
		irq_free_descs(p->irq_base, gpios_per_bank);
		gpiochip_remove(&p->chip);
	}
	iounmap(c->reg_base);
release_reg:
	pci_release_region(pdev, INTELCE_GPIO_BAR);
disable_pci:
	pci_disable_device(pdev);
done:
	kfree(c);
	return ret;
}

static void intelce_gpio_remove(struct pci_dev *pdev)
{
	struct intelce_gpio_chip *c = pci_get_drvdata(pdev);
	struct intelce_gpio_chip *p;
	int i;

	platform_device_del(&sch_device_gpio);
	put_device(&pdev->dev);
	for (i=0; i < banks; i++) {
		p = c + i;
		free_irq(pdev->irq, p);
		irq_free_descs(p->irq_base, gpios_per_bank);
		if (gpiochip_remove(&p->chip))
			dev_err(&pdev->dev, "gpiochip_remove() gpio bank %d failed.\n", i);
	}
	pci_release_region(pdev, INTELCE_GPIO_BAR);
	iounmap(c->reg_base);
	pci_disable_device(pdev);
	kfree(c);
	pci_set_drvdata(pdev, NULL);
}

static struct pci_device_id intelce_gpio_pci_ids[] __devinitdata = {
	{ PCI_DEVICE(PCI_VENDOR_ID_INTEL, PCI_INTELCE_GPIO_DEVICE_ID) },
	{ 0, },
};

#ifdef CONFIG_PM

int intelce_gpio_device_suspend(struct device *dev)
{
	struct pci_dev *pdev = to_pci_dev(dev);
	struct intelce_gpio_chip *c = pci_get_drvdata(pdev);
	int ret = 0;

    /*gpio suspend */
	if (intelce_gpio_suspend) {
		ret = intelce_gpio_suspend(c->reg_base, legacy_iobase);
		if (ret)
			 return ret;
	}
	/*pci device save*/
	pci_save_state(pdev);
	pci_disable_device(pdev);
	pci_set_power_state(pdev, PCI_D3hot);
	return 0;
}

int intelce_gpio_device_resume(struct device *dev)
{
	struct pci_dev *pdev = to_pci_dev(dev);
	struct intelce_gpio_chip *c = pci_get_drvdata(pdev);
	int ret = 0;

	/*pci device restore*/
	pci_set_power_state(pdev, PCI_D0);
	pci_restore_state(pdev);
	ret = pci_enable_device(pdev);
	if (ret)
		return ret;
	/*gpio resume */
	if (intelce_gpio_resume) {
		return intelce_gpio_resume(c->reg_base, legacy_iobase);
	} else {
		return 0;
	}
}

static const struct dev_pm_ops intelce_gpio_pm_ops = {
	.suspend	= intelce_gpio_device_suspend,
	.resume		= intelce_gpio_device_resume,
};
#endif

static struct pci_driver intelce_gpio_driver = {
	.name = INTELCE_GPIO_DRV_NAME,
	.id_table = intelce_gpio_pci_ids,
	.probe = intelce_gpio_probe,
	.remove = intelce_gpio_remove,
#ifdef CONFIG_PM
	.driver.pm = &intelce_gpio_pm_ops,
#endif
};

static int __init intelce_gpio_init(void)
{
	return pci_register_driver(&intelce_gpio_driver);

}
module_init(intelce_gpio_init);

static void __exit intelce_gpio_exit(void)
{
	pci_unregister_driver(&intelce_gpio_driver);
}
module_exit(intelce_gpio_exit);

MODULE_DESCRIPTION("GPIO interface for Intel intelce SoCs");
MODULE_LICENSE("GPL v2");
