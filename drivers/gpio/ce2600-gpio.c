/*
 *  GPIO interface for Intel SOC CE2600.
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

#ifdef CONFIG_HW_MUTEXES
#include <linux/hw_mutex.h>
#endif

#include "ce2600-gpio.h"
#include "ce4200-gpio.h"

void ce2600_gpio_set(struct gpio_chip *chip, unsigned offset, int value)
{
	struct intelce_gpio_chip *c = to_intelce_gpio_chip(chip);
	void *reg_base = c->high_base;

	if (value ) {
		intelce_gpio_mmio_write32(1 << offset, reg_base + CE2600_PUB_GPIO_SET);
	} else {
		intelce_gpio_mmio_write32(1 << offset, reg_base + CE2600_PUB_GPIO_CLEAR);
	}
}

int ce2600_gpio_direction_input(struct gpio_chip *chip, unsigned offset)
{
	struct intelce_gpio_chip *c = to_intelce_gpio_chip(chip);
	void *reg_base = c->reg_base;
	uint32_t orig;
	unsigned long flags;

#ifdef CONFIG_HW_MUTEXES
	hw_mutex_lock(HW_MUTEX_GPIO);
#endif
	spin_lock_irqsave(&c->lock, flags);
	orig = intelce_gpio_mmio_read32(reg_base + CE2600_PUB_GPIO_OUT_EN);
	orig &= ~(1 << offset);
	intelce_gpio_mmio_write32(orig, reg_base + CE2600_PUB_GPIO_OUT_EN);
	spin_unlock_irqrestore(&c->lock, flags);
#ifdef CONFIG_HW_MUTEXES
	hw_mutex_unlock(HW_MUTEX_GPIO);
#endif
	return 0;
}

int ce2600_gpio_direction_output(struct gpio_chip *chip, unsigned offset, int value)
{
	struct intelce_gpio_chip *c = to_intelce_gpio_chip(chip);
	void *reg_base = c->reg_base;
	uint32_t orig;
	unsigned long flags;

#ifdef CONFIG_HW_MUTEXES
	hw_mutex_lock(HW_MUTEX_GPIO);
#endif
	spin_lock_irqsave(&c->lock, flags);
        orig = intelce_gpio_mmio_read32(reg_base + CE2600_PUB_GPIO_OUT);
        if (value) {
                        orig |= (1 << offset);
                } else {
                        orig &= ~(1 << offset);
        }
        intelce_gpio_mmio_write32(orig, reg_base + CE2600_PUB_GPIO_OUT);

	orig = intelce_gpio_mmio_read32(reg_base + CE2600_PUB_GPIO_OUT_EN);
	orig |= (1 << offset);
	intelce_gpio_mmio_write32(orig, reg_base + CE2600_PUB_GPIO_OUT_EN);

	spin_unlock_irqrestore(&c->lock, flags);
#ifdef CONFIG_HW_MUTEXES
	hw_mutex_unlock(HW_MUTEX_GPIO);
#endif
	return 0;
}

int ce2600_set_multi_function(struct gpio_chip *chip, unsigned offset,  int fn_num)
{
	struct intelce_gpio_chip *c = to_intelce_gpio_chip(chip);
	void *reg_base = c->mux_ctl_base;
	uint32_t curr_reg_value = 0;
	uint32_t new_reg_value = 0;
	uint32_t bit_num = 0;
	uint32_t revert = 0;
	uint32_t gpio = chip->base + offset;
	unsigned long flags;

	switch (gpio)
	{
		case 50:
			bit_num = 0;
			break;
		case 116:
		case 117:
		case 118:
		case 38:
		case 39:
		case 40:
		case 41:
			bit_num = 3;
			break;
		case 42:
		case 43:
			bit_num = 4;
			break;
		case 48:
			bit_num = 7;
			break;
		case 55:
			bit_num = 12;
			revert = 1;
			break;
		case 54:
			bit_num = 13;
			revert = 1;
			break;
		case 53:
			bit_num = 14;
			revert = 1;
			break;
		case 52:
			bit_num = 15;
			revert = 1;
			break;
		case 56:
			bit_num = 12;
			break;
		case 57:
			bit_num = 13;
			break;
		case 58:
			bit_num = 14;
			break;
		case 93:
			bit_num = 15;
			break;
		case 51:
			bit_num = 16;
			break;
		default:
			return -EINVAL;
	}
	if (revert)
		fn_num = !fn_num;

#ifdef CONFIG_HW_MUTEXES
	hw_mutex_lock(HW_MUTEX_GPIO);
#endif
	spin_lock_irqsave(&c->lock, flags);
	curr_reg_value = intelce_gpio_mmio_read32(reg_base);
	if (0 == fn_num)
		new_reg_value = curr_reg_value & ~(1 << bit_num);
	else
		new_reg_value = curr_reg_value | (1 << bit_num);
	intelce_gpio_mmio_write32(new_reg_value, reg_base);
	spin_unlock_irqrestore(&c->lock, flags);
#ifdef CONFIG_HW_MUTEXES
	hw_mutex_unlock(HW_MUTEX_GPIO);
#endif
	return 0;
}

int ce2600_get_multi_function(struct gpio_chip *chip, unsigned offset)
{
/*
 *  This function will set the pin associated with the gpio_num to it's alternate function.
 *  bit_num-----function [0] / [1]
 *  0 --- -disable / enable UART1_TXD_GPIO_50
 *  3 ---- GPIO fucntions /Smart Card0  GPIO 116, 117,118, 38, 39, 40, 41
 *  4 ---- gbe link assigned to 1 / GBE use LOS (loss of signal) pin as gbe link GPIO 42, 43
 *  7--- - disable / enable UART0_TXD_GPIO_48
 *  16---- disable  /eanble UART0 RTS GPIO 51
 */
	struct intelce_gpio_chip *c = to_intelce_gpio_chip(chip);
	void *reg_base = c->mux_ctl_base;
	uint32_t gpio = chip->base + offset;
	int bit_num = -1;
	int revert = 0;

	switch (gpio) {
		case 50:
			bit_num = 0;
			break;
		case 38 ... 41:
		case 116 ... 118:
			bit_num = 3;
			break;
		case 42 ... 43:
			bit_num = 4;
			break;
		case 48:
			bit_num = 7;
			break;
		case 51:
			bit_num = 16;
			break;
		case 52:
			bit_num = 15;
			revert = 1;
			break;
		case 53:
			bit_num = 14;
			revert = 1;
			break;
		case 54:
			bit_num = 13;
			revert = 1;
			break;
		case 55:
			bit_num = 12;
			revert = 1;
			break;
		case 56:
			bit_num = 12;
			break;
		case 57:
			bit_num = 13;
			break;
		case 58:
			bit_num = 14;
			break;
		case 93:
			bit_num = 15;
			break;
		default:
			break;
	}
	if (-1 == bit_num) return -EINVAL;

	if (revert)
		return !((intelce_gpio_mmio_read32(reg_base) >> bit_num) & 0x1);
	else
		return (intelce_gpio_mmio_read32(reg_base) >> bit_num) & 0x1;

}

static int ce2600_gpio_irq_set_type(struct irq_data *d, unsigned int type)
{
	struct intelce_gpio_chip *c = irq_data_get_irq_chip_data(d);
	void __iomem *mode_le_reg, *mode_rf_reg;
	uint32_t irq_offs = d->irq - c->irq_base;
	uint32_t reg1, reg2;
	int ret = 0;

	if (irq_offs > 32)
		return -EINVAL;

	mode_le_reg = c->reg_base + CE2600_PUB_GPIO_INT_MODE_LE;
	mode_rf_reg = c->reg_base + CE2600_PUB_GPIO_INT_MODE_RF;

#ifdef CONFIG_HW_MUTEXES
	hw_mutex_lock(HW_MUTEX_GPIO);
#endif
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
		ret =-EINVAL;
		break;
	}
	intelce_gpio_mmio_write32(reg1, mode_le_reg);
	intelce_gpio_mmio_write32(reg2, mode_rf_reg);
#ifdef CONFIG_HW_MUTEXES
	hw_mutex_unlock(HW_MUTEX_GPIO);
#endif
	return ret;
}

static struct irq_chip ce2600_irq_chip = {
	.irq_mask = ce4200_gpio_irq_mask,
	.irq_unmask = ce4200_gpio_irq_unmask,
	.irq_eoi = ce4200_gpio_irq_eoi,
	.irq_set_type = ce2600_gpio_irq_set_type,
};

__devinit int ce2600_gpio_irq_setup(struct intelce_gpio_chip *c, struct pci_dev *pdev)
{
	int i;
	int irq;
	int ret;

	c->irq_base = irq_alloc_descs(-1, INTELCE_GPIO_IRQ_BASE, CE2600_PUB_GPIOS_PER_BANK, -1);
	if (c->irq_base < 0)
		return c->irq_base;

	/* mask + ACK all interrupt sources */
	intelce_gpio_mmio_write32(0, c->reg_base + CE2600_PUB_GPIO_INT_EN);
	intelce_gpio_mmio_write32(0xFFFFFFFF, c->reg_base + CE2600_PUB_GPIO_INT_STAT);

	ret = request_irq(pdev->irq, ce4200_gpio_irq_handler, IRQF_SHARED, "ce2600_gpio", c);
	if (ret)
		goto out_free_desc;

	/*
	 * This gpio irq controller latches level irqs. Testing shows that if
	 * we unmask & ACK the IRQ before the source of the interrupt is gone
	 * then the interrupt is active again.
	 */
	irq = c->irq_base;
	for (i=0; i < c->chip.ngpio; i++) {
		irq_set_chip_and_handler_name(irq, &ce2600_irq_chip, handle_fasteoi_irq, "gpio_irq");
		irq_set_chip_data(irq, c);
		irq++;
	}
	return 0;

out_free_desc:
	irq_free_descs(c->irq_base, CE2600_PUB_GPIOS_PER_BANK);
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
	uint32_t polarity;
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
	uint32_t mux_ctl;
	uint32_t int_router;
};

static struct _gpio  _gpio;

/*CE2600 gpio suspend routine */
int ce2600_gpio_suspend(void *io_mem, unsigned short io_port)
{
	char *virt_io_mem = (char *)io_mem;
	int i;

    /* Keep status of Core Well GPIO*/
	_gpio.cgen = intelce_gpio_port_read32(io_port + CE2600_CORE_WELL_GPIO_CGEN);
	_gpio.cgio = intelce_gpio_port_read32(io_port + CE2600_CORE_WELL_GPIO_CGIO);
	_gpio.cglv = intelce_gpio_port_read32(io_port + CE2600_CORE_WELL_GPIO_CGLV);
	_gpio.cgtpe = intelce_gpio_port_read32(io_port + CE2600_CORE_WELL_GPIO_CGTPE);
	_gpio.cgtne = intelce_gpio_port_read32(io_port + CE2600_CORE_WELL_GPIO_CGTNE);
	_gpio.cggpe = intelce_gpio_port_read32(io_port + CE2600_CORE_WELL_GPIO_CGGPE);
	_gpio.cgsmi = intelce_gpio_port_read32(io_port + CE2600_CORE_WELL_GPIO_CGSMI);


	/* Keep status of general purpose GPIO*/
	_gpio.mux_ctl = intelce_gpio_mmio_read32(virt_io_mem + CE2600_PUB_GPIO_MUX_CTL);
	_gpio.group[0].polarity = intelce_gpio_mmio_read32(virt_io_mem + CE2600_PUB_GPIO_POLARITY0);
	_gpio.group[1].polarity = intelce_gpio_mmio_read32(virt_io_mem + CE2600_PUB_GPIO_POLARITY1);
	_gpio.group[2].polarity = intelce_gpio_mmio_read32(virt_io_mem + CE2600_PUB_GPIO_POLARITY2);
	_gpio.group[3].polarity = intelce_gpio_mmio_read32(virt_io_mem + CE2600_PUB_GPIO_POLARITY3);
	_gpio.int_router = intelce_gpio_mmio_read32(virt_io_mem + CE2600_PUB_GPIO_INT_ROUTER);
	for (i=0; i < 4; i++) {
		_gpio.group[i].output = intelce_gpio_mmio_read32(virt_io_mem + CE2600_PUB_GPIO_OUT);
		_gpio.group[i].output_enable = intelce_gpio_mmio_read32(virt_io_mem + CE2600_PUB_GPIO_OUT_EN);
		_gpio.group[i].int_enable = intelce_gpio_mmio_read32(virt_io_mem + CE2600_PUB_GPIO_INT_EN);
		_gpio.group[i].mode_le = intelce_gpio_mmio_read32(virt_io_mem + CE2600_PUB_GPIO_INT_MODE_LE);
		_gpio.group[i].mode_rf = intelce_gpio_mmio_read32(virt_io_mem + CE2600_PUB_GPIO_INT_MODE_RF);
		virt_io_mem += 0x20;
	}
	return 0;
}

/* CE2600 gpio resume routine */
int ce2600_gpio_resume(void *io_mem, unsigned short io_port)
{
	char *virt_io_mem = (char *)io_mem;
	int i;

    /* Restore status of general purpose GPIO*/
	intelce_gpio_mmio_write32(_gpio.mux_ctl, virt_io_mem + CE2600_PUB_GPIO_MUX_CTL);
	intelce_gpio_mmio_write32(_gpio.group[0].polarity, virt_io_mem + CE2600_PUB_GPIO_POLARITY0);
	intelce_gpio_mmio_write32(_gpio.group[1].polarity, virt_io_mem + CE2600_PUB_GPIO_POLARITY1);
	intelce_gpio_mmio_write32(_gpio.group[2].polarity, virt_io_mem + CE2600_PUB_GPIO_POLARITY2);
	intelce_gpio_mmio_write32(_gpio.group[3].polarity, virt_io_mem + CE2600_PUB_GPIO_POLARITY3);
	intelce_gpio_mmio_write32(_gpio.int_router, virt_io_mem + CE2600_PUB_GPIO_INT_ROUTER);
	for (i=0; i < 4; i++) {
		intelce_gpio_mmio_write32(_gpio.group[i].output, virt_io_mem + CE2600_PUB_GPIO_OUT);
		intelce_gpio_mmio_write32(_gpio.group[i].output_enable, virt_io_mem + CE2600_PUB_GPIO_OUT_EN);
		intelce_gpio_mmio_write32(_gpio.group[i].int_enable, virt_io_mem + CE2600_PUB_GPIO_INT_EN);
		intelce_gpio_mmio_write32(_gpio.group[i].mode_le, virt_io_mem + CE2600_PUB_GPIO_INT_MODE_LE);
		intelce_gpio_mmio_write32(_gpio.group[i].mode_rf, virt_io_mem + CE2600_PUB_GPIO_INT_MODE_RF);
		virt_io_mem += 0x20;

	}

    /* Restore status of Core Well GPIO*/
	intelce_gpio_port_write32(_gpio.cgen & 0x3FF, io_port + CE2600_CORE_WELL_GPIO_CGEN);
	intelce_gpio_port_write32(_gpio.cgio & 0x3FF, io_port + CE2600_CORE_WELL_GPIO_CGIO);
	intelce_gpio_port_write32(_gpio.cglv & 0x3FF, io_port + CE2600_CORE_WELL_GPIO_CGLV);
	intelce_gpio_port_write32(_gpio.cgtpe & 0x3FF, io_port + CE2600_CORE_WELL_GPIO_CGTPE);
	intelce_gpio_port_write32(_gpio.cgtne & 0x3FF, io_port + CE2600_CORE_WELL_GPIO_CGTNE);
	intelce_gpio_port_write32(_gpio.cggpe & 0x3FF, io_port + CE2600_CORE_WELL_GPIO_CGGPE);
	intelce_gpio_port_write32(_gpio.cgsmi & 0x3FF, io_port + CE2600_CORE_WELL_GPIO_CGSMI);
	return 0;
}
