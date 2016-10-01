/*
 *  GPIO interface for Intel SoC CE5300.
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

#include "ce5300-gpio.h"
#include "ce4200-gpio.h"

int ce5300_set_multi_function(struct gpio_chip *chip, unsigned offset,  int fn_num)
{
    /*
	 *       0 --- UART1_TXD_GPIO_64 enable [1] /disable [0]
	 *       1 --- UART2_TXD_GPIO_66 enable [1] /disable [0]
	 *       2 --- Smart Card 1 [1] / GPIO [0]
	 *       3 --- Smart Card 0 [1] / GPIO [0]
	 *       4 --- GBE_LINK [1] / GPIO95 [0]
	 *       5-7 --- Reserved
	 *       8 --- Select Internal video sync 0 as trigger input [1] / Select a GPIO as trigger input
	 *       9 --- Select Internal video sync 0 as trigger input [1] / Select a GPIO as trigger input
	 *       10 --- Select Internal video sync 1 as trigger input [1] / Select a GPIO as trigger input
	 *       11 --- Select Internal video sync 1 as trigger input [1] / Select a GPIO as trigger input
	 *       12 --- GPIO 77 supplies the trigger input [1] / GPIO 38 supplies the trigger input [0]
	 *       13 --- GPIO 78 supplies the trigger input [1] / GPIO 37 supplies the trigger input [0]
	 *       14 --- GPIO 79 supplies the trigger input [1] / GPIO 36 supplies the trigger input [0]
	 *       15 --- GPIO 80 supplies the trigger input [1] / GPIO 35 supplies the trigger input [0]
	 *
	 */
	struct intelce_gpio_chip *c = to_intelce_gpio_chip(chip);
	void *reg_base = c->mux_ctl_base;
	uint32_t orig;
	uint32_t gpio = chip->base + offset;
	unsigned long flags;
	int bit_num = -1;
	int ret = 0;

	switch (gpio) {
		case 64:
			bit_num = 0;
			break;
		case 66:
			bit_num = 1;
			break;
		case 58 ... 63:
			bit_num = 2;
			break;
		case 33 ... 34:
		case 99 ... 102:
			bit_num = 3;
			break;
		case 95:
			bit_num = 4;
			break;
		case 38:
			spin_lock_irqsave(&c->lock, flags);
			orig = intelce_gpio_mmio_read32(reg_base);
			if (1 == fn_num) {
				intelce_gpio_mmio_write32(orig & ~((1<<8)|(1<<12)), reg_base);
			} else if (0 == fn_num) {
				intelce_gpio_mmio_write32(orig | (1<<8), reg_base);
			}else {
				ret = -EINVAL;
			}
			spin_unlock_irqrestore(&c->lock, flags);
			return ret;
		case 77:
			spin_lock_irqsave(&c->lock, flags);
			orig = intelce_gpio_mmio_read32(reg_base);
			if (1 == fn_num) {
				intelce_gpio_mmio_write32((orig & ~(1<<8)) | (1<<12), reg_base);
			} else if (0 == fn_num) {
				intelce_gpio_mmio_write32(orig | (1<<8), reg_base);
			} else {
				ret = -EINVAL;
			}
			spin_unlock_irqrestore(&c->lock, flags);
			return ret;
		case 37:
			spin_lock_irqsave(&c->lock, flags);
			orig = intelce_gpio_mmio_read32(reg_base);
			if (1 == fn_num) {
				intelce_gpio_mmio_write32(orig & ~((1<<9)|(1<<13)), reg_base);
			} else if (0 == fn_num) {
				intelce_gpio_mmio_write32(orig | (1<<9), reg_base);
			} else {
				ret = -EINVAL;
			}
			spin_unlock_irqrestore(&c->lock, flags);
			return ret;
		case 78:
			spin_lock_irqsave(&c->lock, flags);
			orig = intelce_gpio_mmio_read32(reg_base);
			if (1 == fn_num) {
				intelce_gpio_mmio_write32((orig & ~(1<<9)) | (1<<13), reg_base);
			} else if (0 == fn_num) {
				intelce_gpio_mmio_write32(orig | (1<<9), reg_base);
			} else {
				ret = -EINVAL;
			}
			spin_unlock_irqrestore(&c->lock, flags);
			return ret;
		case 36:
			spin_lock_irqsave(&c->lock, flags);
			orig = intelce_gpio_mmio_read32(reg_base);
			if (1 == fn_num) {
				intelce_gpio_mmio_write32(orig & ~((1<<10)|(1<<14)), reg_base);
			} else if (0 == fn_num) {
				intelce_gpio_mmio_write32(orig | (1<<10), reg_base);
			} else {
				ret =-EINVAL;
			}
			spin_unlock_irqrestore(&c->lock, flags);
			return ret;
		case 79:
			spin_lock_irqsave(&c->lock, flags);
			orig = intelce_gpio_mmio_read32(reg_base);
			if (1 == fn_num) {
				intelce_gpio_mmio_write32((orig & ~(1<<10)) | (1<<14), reg_base);
			} else if (0 == fn_num) {
				intelce_gpio_mmio_write32(orig | (1<<10), reg_base);
			} else {
				ret = -EINVAL;
			}
			spin_unlock_irqrestore(&c->lock, flags);
			return ret;
		case 35:
			spin_lock_irqsave(&c->lock, flags);
			orig = intelce_gpio_mmio_read32(reg_base);
			if (1 == fn_num) {
				intelce_gpio_mmio_write32(orig & ~((1<<11)|(1<<15)), reg_base);
			} else if (0 == fn_num) {
				intelce_gpio_mmio_write32(orig | (1<<11), reg_base);
			} else {
				ret = -EINVAL;
			}
			spin_unlock_irqrestore(&c->lock, flags);
			return ret;
		case 80:
			spin_lock_irqsave(&c->lock, flags);
			orig = intelce_gpio_mmio_read32(reg_base);
			if (1 == fn_num) {
				intelce_gpio_mmio_write32((orig & ~(1<<11)) | (1<<15), reg_base);
			} else if (0 == fn_num) {
				intelce_gpio_mmio_write32(orig | (1<<11), reg_base);
			} else {
				ret = -EINVAL;
			}
			spin_unlock_irqrestore(&c->lock, flags);
			return ret;
		default:
			break;
	}

	if (-1 == bit_num) return -EINVAL;

	spin_lock_irqsave(&c->lock, flags);
	orig = intelce_gpio_mmio_read32(reg_base);
	switch (fn_num) {
		case 0:
			intelce_gpio_mmio_write32(orig & ~(1 << bit_num), reg_base);
			break;
		case 1:
			intelce_gpio_mmio_write32(orig | (1 << bit_num), reg_base);
			break;
		default:
			ret = -EINVAL;
			break;
	}
	spin_unlock_irqrestore(&c->lock, flags);
	return ret;
}

int ce5300_get_multi_function(struct gpio_chip *chip, unsigned offset)
{
    /*
	 *       0 --- UART1_TXD_GPIO_64 enable [1] /disable [0]
	 *       1 --- UART2_TXD_GPIO_66 enable [1] /disable [0]
	 *       2 --- Smart Card 1 [1] / GPIO [0]
	 *       3 --- Smart Card 0 [1] / GPIO [0]
	 *       4 --- GBE_LINK [1] / GPIO95 [0]
	 *       5-7 --- Reserved
	 *       8 --- Select Internal video sync 0 as trigger input [1] / Select a GPIO as trigger input
	 *       9 --- Select Internal video sync 0 as trigger input [1] / Select a GPIO as trigger input
	 *       10 --- Select Internal video sync 1 as trigger input [1] / Select a GPIO as trigger input
	 *       11 --- Select Internal video sync 1 as trigger input [1] / Select a GPIO as trigger input
	 *       12 --- GPIO 77 supplies the trigger input [1] / GPIO 38 supplies the trigger input [0]
	 *       13 --- GPIO 78 supplies the trigger input [1] / GPIO 37 supplies the trigger input [0]
	 *       14 --- GPIO 79 supplies the trigger input [1] / GPIO 36 supplies the trigger input [0]
	 *       15 --- GPIO 80 supplies the trigger input [1] / GPIO 35 supplies the trigger input [0]
	 *
	 */
	struct intelce_gpio_chip *c = to_intelce_gpio_chip(chip);
	void *reg_base = c->mux_ctl_base;
	uint32_t gpio = chip->base + offset;
	int bit_num = -1;

	switch (gpio) {
		case 64:
			bit_num = 0;
			break;
		case 66:
			bit_num = 1;
			break;
		case 58 ... 63:
			bit_num = 2;
			break;
		case 33 ... 34:
		case 99 ... 102:
			bit_num = 3;
			break;
		case 95:
			bit_num = 4;
			break;
		case 35 ... 38:
		case 77 ... 80:
			bit_num = 5;
		default:
			break;
	}
	if (-1 == bit_num) return -EINVAL;

	return (intelce_gpio_mmio_read32(reg_base) >> bit_num) & 0x1;

}


static int ce5300_gpio_irq_set_type(struct irq_data *d, unsigned int type)
{
	struct intelce_gpio_chip *c = irq_data_get_irq_chip_data(d);
	void __iomem *mode_le_reg, *mode_rf_reg;
	uint32_t irq_offs = d->irq - c->irq_base;
	uint32_t reg1, reg2;

	if (irq_offs > 32)
		return -EINVAL;

	mode_le_reg = c->reg_base + CE5300_PUB_GPIO_INT_MODE_LE;
	mode_rf_reg = c->reg_base + CE5300_PUB_GPIO_INT_MODE_RF;

	reg1 = intelce_gpio_mmio_read32(mode_le_reg);
	reg2 = intelce_gpio_mmio_read32(mode_rf_reg);

	switch (type) {
	case IRQ_TYPE_EDGE_RISING:
		reg1 |= BIT(irq_offs);
		reg2 &= ~BIT(irq_offs);
		break;

	case IRQ_TYPE_EDGE_FALLING:
		reg1 |= BIT(irq_offs);
		reg2 |= BIT(irq_offs);
		break;

	case IRQ_TYPE_LEVEL_HIGH:
		reg1 &= ~BIT(irq_offs);
		reg2 &= ~BIT(irq_offs);
		break;

	case IRQ_TYPE_LEVEL_LOW:
		reg1 &= ~BIT(irq_offs);
		reg2 |= BIT(irq_offs);
		break;

	default:
		return -EINVAL;
	}
	intelce_gpio_mmio_write32(reg1, mode_le_reg);
	intelce_gpio_mmio_write32(reg2, mode_rf_reg);
	return 0;
}

static struct irq_chip ce5300_irq_chip = {
	.irq_mask = ce4200_gpio_irq_mask,
	.irq_unmask = ce4200_gpio_irq_unmask,
	.irq_eoi = ce4200_gpio_irq_eoi,
	.irq_set_type = ce5300_gpio_irq_set_type,
};

__devinit int ce5300_gpio_irq_setup(struct intelce_gpio_chip *c, struct pci_dev *pdev)
{
	int i;
	int irq;
	int ret;

	c->irq_base = irq_alloc_descs(-1, INTELCE_GPIO_IRQ_BASE, CE5300_PUB_GPIOS_PER_BANK, -1);
	if (c->irq_base < 0)
		return c->irq_base;

	/* mask + ACK all interrupt sources */
	intelce_gpio_mmio_write32(0, c->reg_base + CE5300_PUB_GPIO_INT_EN);
	intelce_gpio_mmio_write32(0xFFFFFFFF, c->reg_base + CE5300_PUB_GPIO_INT_STAT);

	ret = request_irq(pdev->irq, ce4200_gpio_irq_handler, IRQF_SHARED, "ce5300_gpio", c);
	if (ret)
		goto out_free_desc;

	/*
	 * This gpio irq controller latches level irqs. Testing shows that if
	 * we unmask & ACK the IRQ before the source of the interrupt is gone
	 * then the interrupt is active again.
	 */
	irq = c->irq_base;
	for (i=0; i < c->chip.ngpio; i++) {
		irq_set_chip_and_handler_name(irq, &ce5300_irq_chip, handle_fasteoi_irq, "gpio_irq");
		irq_set_chip_data(irq, c);
		irq++;
	}
	return 0;

out_free_desc:
	irq_free_descs(c->irq_base, CE5300_PUB_GPIOS_PER_BANK);
	return ret;
}

struct gpio_group {
	uint32_t input;
	uint32_t output;
	uint32_t output_enable;
	uint32_t int_status;
	uint32_t int_enable;
	uint32_t mode_le;
    uint32_t mode_rf;
};

struct _gpio {
	uint32_t cgen;
	uint32_t cgio;
	uint32_t cglv;
	uint32_t cgtpe;
	uint32_t cgtne;
	uint32_t cggpe;
	uint32_t cgsmi;
	uint32_t cgts;

	struct gpio_group group[4];
	uint32_t gpio_mux_ctl;
};

static struct _gpio  _gpio;

/*CE5300 gpio suspend routine */
int ce5300_gpio_suspend(void *io_mem, unsigned short io_port)
{
	char *virt_io_mem = (char *)io_mem;
	int i;

    /* Keep status of Core Well GPIO*/
	_gpio.cgen = intelce_gpio_port_read32(io_port + CE5300_CORE_WELL_GPIO_CGEN);
	_gpio.cgio = intelce_gpio_port_read32(io_port + CE5300_CORE_WELL_GPIO_CGIO);
	_gpio.cglv = intelce_gpio_port_read32(io_port + CE5300_CORE_WELL_GPIO_CGLV);
	_gpio.cgtpe = intelce_gpio_port_read32(io_port + CE5300_CORE_WELL_GPIO_CGTPE);
	_gpio.cgtne = intelce_gpio_port_read32(io_port + CE5300_CORE_WELL_GPIO_CGTNE);
	_gpio.cggpe = intelce_gpio_port_read32(io_port + CE5300_CORE_WELL_GPIO_CGGPE);
	_gpio.cgsmi = intelce_gpio_port_read32(io_port + CE5300_CORE_WELL_GPIO_CGSMI);

    /* Keep status of general purpose GPIO*/
	_gpio.gpio_mux_ctl = intelce_gpio_mmio_read32(virt_io_mem + CE5300_PUB_GPIO_MUX_CTL);
	for (i=0; i < 4; i++) {
		_gpio.group[i].output = intelce_gpio_mmio_read32(virt_io_mem + CE5300_PUB_GPIO_OUT);
		_gpio.group[i].output_enable = intelce_gpio_mmio_read32(virt_io_mem + CE5300_PUB_GPIO_OUT_EN);
		_gpio.group[i].int_enable = intelce_gpio_mmio_read32(virt_io_mem + CE5300_PUB_GPIO_INT_EN);
		_gpio.group[i].mode_le = intelce_gpio_mmio_read32(virt_io_mem + CE5300_PUB_GPIO_INT_MODE_LE);
		_gpio.group[i].mode_rf = intelce_gpio_mmio_read32(virt_io_mem + CE5300_PUB_GPIO_INT_MODE_RF);
		virt_io_mem += 0x20;
	}
	return 0;
}


/* CE5300 gpio resume routine */
int ce5300_gpio_resume(void *io_mem, unsigned short io_port)
{
	char *virt_io_mem = (char *)io_mem;
	int i;

    /* Restore status of general purpose GPIO*/
	intelce_gpio_mmio_write32( _gpio.gpio_mux_ctl, virt_io_mem + CE5300_PUB_GPIO_MUX_CTL);
	for (i=0; i < 4; i++) {
		intelce_gpio_mmio_write32(_gpio.group[i].output, virt_io_mem + CE5300_PUB_GPIO_OUT);
		intelce_gpio_mmio_write32(_gpio.group[i].output_enable, virt_io_mem + CE5300_PUB_GPIO_OUT_EN);
		intelce_gpio_mmio_write32(_gpio.group[i].int_enable, virt_io_mem + CE5300_PUB_GPIO_INT_EN);
		intelce_gpio_mmio_write32(_gpio.group[i].mode_le, virt_io_mem +CE5300_PUB_GPIO_INT_MODE_LE);
		intelce_gpio_mmio_write32(_gpio.group[i].mode_rf, virt_io_mem + CE5300_PUB_GPIO_INT_MODE_RF);
		virt_io_mem += 0x20;

	}

    /* Restore status of Core Well GPIO*/
	intelce_gpio_port_write32(_gpio.cgen & 0x3FF, io_port + CE5300_CORE_WELL_GPIO_CGEN);
	intelce_gpio_port_write32(_gpio.cgio & 0x3FF, io_port + CE5300_CORE_WELL_GPIO_CGIO);
	intelce_gpio_port_write32(_gpio.cgtpe & 0x3FF, io_port + CE5300_CORE_WELL_GPIO_CGLV);
	intelce_gpio_port_write32(_gpio.cgtpe & 0x3FF, io_port + CE5300_CORE_WELL_GPIO_CGTPE);
	intelce_gpio_port_write32(_gpio.cgtne & 0x3FF, io_port + CE5300_CORE_WELL_GPIO_CGTNE);
	intelce_gpio_port_write32(_gpio.cggpe & 0x3FF, io_port + CE5300_CORE_WELL_GPIO_CGGPE);
	intelce_gpio_port_write32( _gpio.cgsmi & 0x3FF, io_port + CE5300_CORE_WELL_GPIO_CGSMI);
	return 0;
}
