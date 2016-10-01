/*
 *  GPIO interface for Intel SoC CE4100.
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

#include "ce4100-gpio.h"

int ce4100_gpio_get(struct gpio_chip *chip, unsigned offset)
{
	void *reg_base = to_intelce_gpio_chip(chip)->reg_base;

	return (intelce_gpio_mmio_read32(reg_base + CE4100_PUB_GPIO_INPUT) >> offset) & 0x1;
}

void ce4100_gpio_set(struct gpio_chip *chip, unsigned offset, int value)
{
	struct intelce_gpio_chip *c = to_intelce_gpio_chip(chip);
	void *reg_base = c->reg_base;
	uint32_t orig;
	unsigned long flags;

	spin_lock_irqsave(&c->lock, flags);
	orig = intelce_gpio_mmio_read32(reg_base + CE4100_PUB_GPIO_OUT);
	if (value) {
			orig |= (1 << offset);
		} else {
			orig &= ~(1 << offset);
	}
	intelce_gpio_mmio_write32(orig, reg_base + CE4100_PUB_GPIO_OUT);
	spin_unlock_irqrestore(&c->lock, flags);

}


int ce4100_gpio_direction_input(struct gpio_chip *chip, unsigned offset)
{
	struct intelce_gpio_chip *c = to_intelce_gpio_chip(chip);
	void *reg_base = c->reg_base;
	uint32_t orig;
	unsigned long flags;

	spin_lock_irqsave(&c->lock, flags);
	orig = intelce_gpio_mmio_read32(reg_base + CE4100_PUB_GPIO_OUT_EN);
	orig &= ~(1 << offset);
	intelce_gpio_mmio_write32(orig, reg_base + CE4100_PUB_GPIO_OUT_EN);
	spin_unlock_irqrestore(&c->lock, flags);
	return 0;
}

int ce4100_gpio_direction_output(struct gpio_chip *chip, unsigned offset, int value)
{
	struct intelce_gpio_chip *c = to_intelce_gpio_chip(chip);
	void *reg_base = c->reg_base;
	uint32_t orig;
	unsigned long flags;

	spin_lock_irqsave(&c->lock, flags);
	orig = intelce_gpio_mmio_read32(reg_base + CE4100_PUB_GPIO_OUT);
	if (value) {
			orig |= (1 << offset);
		} else {
			orig &= ~(1 << offset);
	}
	intelce_gpio_mmio_write32(orig, reg_base + CE4100_PUB_GPIO_OUT);

	orig = intelce_gpio_mmio_read32(reg_base + CE4100_PUB_GPIO_OUT_EN);
	orig |= (1 << offset);
	intelce_gpio_mmio_write32(orig, reg_base + CE4100_PUB_GPIO_OUT_EN);
	spin_unlock_irqrestore(&c->lock, flags);
	return 0;
}

int ce4100_set_multi_function(struct gpio_chip *chip, unsigned offset,  int fn_num)
{
	struct intelce_gpio_chip *c = to_intelce_gpio_chip(chip);
	void *reg_base = c->mux_ctl_base;
	uint32_t orig;
	unsigned long flags;
	uint32_t gpio = chip->base + offset;
	int bit_num = -1;
	int ret = 0;

	switch (gpio) {
		case 0 ... 5:
			bit_num = 0;
			break;
		case 6 ... 7:
			bit_num = 1;
			break;
		case 8 ... 11:
			bit_num = 2;
			break;
		case 15 ... 16:
			bit_num = 3;
			break;
		case 17 ... 19:
			bit_num = 4;
			break;
		case 20:
			bit_num = 5;
			break;
		case 21:
			bit_num = 6;
			break;
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

int ce4100_get_multi_function(struct gpio_chip *chip, unsigned offset)
{
	struct intelce_gpio_chip *c = to_intelce_gpio_chip(chip);
	void *reg_base = c->mux_ctl_base;
	uint32_t gpio = chip->base + offset;
	int bit_num = -1;

	switch (gpio) {
		case 0 ... 5:
			bit_num = 0;
			break;
		case 6 ... 7:
			bit_num = 1;
			break;
		case 8 ... 11:
			bit_num = 2;
			break;
		case 15 ... 16:
			bit_num = 3;
			break;
		case 17 ... 19:
			bit_num = 4;
			break;
		case 20:
			bit_num = 5;
			break;
		case 21:
			bit_num = 6;
			break;
		default:
			break;
	}
	if (-1 == bit_num) return -EINVAL;

	return (intelce_gpio_mmio_read32(reg_base) >> bit_num) & 0x1;
}

int ce4100_gpio_to_irq(struct gpio_chip *chip, unsigned offset)
{
	return to_intelce_gpio_chip(chip)->irq_base + offset;
}

static irqreturn_t ce4100_gpio_irq_handler(int irq, void *data)
{
	struct intelce_gpio_chip *c = data;
	uint32_t irq_stat = intelce_gpio_mmio_read32(c->reg_base + CE4100_PUB_GPIO_INT_STAT);

	irq_stat &= intelce_gpio_mmio_read32(c->reg_base + CE4100_PUB_GPIO_INT_EN);
	if (!irq_stat)
		return IRQ_NONE;

	while (irq_stat) {
		uint32_t irq_bit = __fls(irq_stat);

		irq_stat &= ~BIT(irq_bit);
		generic_handle_irq(c->irq_base + irq_bit);
	}

	return IRQ_HANDLED;
}

static void ce4100_gpio_irq_mask(struct irq_data *data)
{
	struct intelce_gpio_chip *c = irq_data_get_irq_chip_data(data);
	void *reg_base = c->reg_base;
	uint32_t orig;

	orig = intelce_gpio_mmio_read32(reg_base + CE4100_PUB_GPIO_INT_EN);
	orig &= ~(1 << (data->irq - c->irq_base));
	intelce_gpio_mmio_write32(orig, reg_base + CE4100_PUB_GPIO_INT_EN);
}

static void ce4100_gpio_irq_unmask(struct irq_data *data)
{
	struct intelce_gpio_chip *c = irq_data_get_irq_chip_data(data);
	void *reg_base = c->reg_base;
	uint32_t orig;

	orig = intelce_gpio_mmio_read32(reg_base + CE4100_PUB_GPIO_INT_EN);
	orig |= (1 << (data->irq - c->irq_base));
	intelce_gpio_mmio_write32(orig, reg_base + CE4100_PUB_GPIO_INT_EN);
}

static void ce4100_gpio_irq_eoi(struct irq_data *data)
{
	struct intelce_gpio_chip *c = irq_data_get_irq_chip_data(data);
	void *reg_base = c->reg_base;

	intelce_gpio_mmio_write32(1 << (data->irq -c->irq_base), reg_base + CE4100_PUB_GPIO_INT_STAT);
}

static int ce4100_gpio_irq_set_type(struct irq_data *d, unsigned int type)
{
	struct intelce_gpio_chip *c = irq_data_get_irq_chip_data(d);
	void __iomem *type_reg;
	uint32_t irq_offs = d->irq - c->irq_base;
	uint32_t bit_offs;
	uint32_t reg;

	if (irq_offs > 12)
		return -EINVAL;

	if (irq_offs < 8)
		type_reg = c->reg_base + CE4100_PUB_GPIO_INT_TYPE0;
	else
		type_reg = c->reg_base + CE4100_PUB_GPIO_INT_TYPE1;

	bit_offs = 4 *(irq_offs % 8);
	reg = intelce_gpio_mmio_read32(type_reg);

	reg &= ~(0xF << bit_offs);

	switch (type) {
		case IRQ_TYPE_LEVEL_HIGH:
			break;

		case IRQ_TYPE_LEVEL_LOW:
			reg |= (1 << bit_offs);
			break;

		default:
			return -EINVAL;
	}

	intelce_gpio_mmio_write32(reg, type_reg);
	return 0;
}

static struct irq_chip ce4100_irq_chip = {
	.irq_mask = ce4100_gpio_irq_mask,
	.irq_unmask = ce4100_gpio_irq_unmask,
	.irq_eoi = ce4100_gpio_irq_eoi,
	.irq_set_type = ce4100_gpio_irq_set_type,
};

__devinit int ce4100_gpio_irq_setup(struct intelce_gpio_chip *c, struct pci_dev *pdev)
{
	int i;
	int irq;
	int ret;

	c->irq_base = irq_alloc_descs(-1, INTELCE_GPIO_IRQ_BASE, CE4100_PUB_GPIOS_PER_BANK, -1);
	if (c->irq_base < 0)
		return c->irq_base;

	/* mask + ACK all interrupt sources */
	intelce_gpio_mmio_write32(0, c->reg_base + CE4100_PUB_GPIO_INT_EN);
	intelce_gpio_mmio_write32(0xFFF, c->reg_base + CE4100_PUB_GPIO_INT_STAT);

	ret = request_irq(pdev->irq, ce4100_gpio_irq_handler, IRQF_SHARED, "ce4100_gpio", c);
	if (ret)
		goto out_free_desc;

	/*
	 * This gpio irq controller latches level irqs. Testing shows that if
	 * we unmask & ACK the IRQ before the source of the interrupt is gone
	 * then the interrupt is active again.
	 */
	irq = c->irq_base;
	for (i=0; i < c->chip.ngpio; i++) {
		irq_set_chip_and_handler_name(irq, &ce4100_irq_chip, handle_fasteoi_irq, "gpio_irq");
		irq_set_chip_data(irq, c);
		irq++;
	}
	return 0;

out_free_desc:
	irq_free_descs(c->irq_base, CE4100_PUB_GPIOS_PER_BANK);
	return ret;
}
