/*
 *  GPIO interface for Intel CE4200.
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

#include "ce4200-gpio.h"
#include "ce4100-gpio.h"

int ce4200_set_multi_function(struct gpio_chip *chip, unsigned offset,  int fn_num)
{
    /*
	 *      0 --- GBE_LINK /GPIO_AUX[21]  -> GPIO[0]
	 *      1 --- Smart Card 0 / TSD_ICAM -> GPIO[1]
	 *      2 --- Smart Card 1 / NAND_CE_N, NAND_RE_N -> GPIO[2]
	 *      3 --- UART0_DSRB_I2S0_IN_BCK_GPIO_24 -> GPIO[3]
	 *      4 --- UART0_DTRB_I2S0_IN_MCLK_GPIO_25 -> GPIO[4]
	 *      5 --- UART0_DCDB_I2S1_IN_BCK_GPIO_26 -> GPIO[5]
	 *      6 --- UART0_RIB_SPDIF_IN_GPIO_27 -> GPIO[6]
	 *      7 --- UART0_RTSB_I2S1_IN_SDATA_3_GPIO_28 -> GPIO[7]
	 *      8 --- UART0_CTSB_I2S1_IN_SDATA_2_GPIO_29 -> GPIO[8]
	 *      9 --- UART1_RXD and UART1_TXD enable / UART1_RXD and UART1_TXD disable -> GPIO[9]
	 */
	struct intelce_gpio_chip *c = to_intelce_gpio_chip(chip);
	void *reg_base = c->mux_ctl_base;
	uint32_t orig;
	unsigned long flags;
	uint32_t gpio = chip->base + offset;
	int bit_num = -1;
	int ret = 0;

	switch(gpio) {
		case 0:
			bit_num = 4;
			break;
		case 1:
			bit_num = 3;
			break;
		case 2:
			bit_num = 2;
			break;
		case 3 ... 8:
			bit_num = 0;
			break;
		case 9:
			bit_num = 1;
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

int ce4200_get_multi_function(struct gpio_chip *chip, unsigned offset)
{
    /*
	 *      0 --- GBE_LINK /GPIO_AUX[21]  -> GPIO[0]
	 *      1 --- Smart Card 0 / TSD_ICAM -> GPIO[1]
	 *      2 --- Smart Card 1 / NAND_CE_N, NAND_RE_N -> GPIO[2]
	 *      3 --- UART0_DSRB_I2S0_IN_BCK_GPIO_24 -> GPIO[3]
	 *      4 --- UART0_DTRB_I2S0_IN_MCLK_GPIO_25 -> GPIO[4]
	 *      5 --- UART0_DCDB_I2S1_IN_BCK_GPIO_26 -> GPIO[5]
	 *      6 --- UART0_RIB_SPDIF_IN_GPIO_27 -> GPIO[6]
	 *      7 --- UART0_RTSB_I2S1_IN_SDATA_3_GPIO_28 -> GPIO[7]
	 *      8 --- UART0_CTSB_I2S1_IN_SDATA_2_GPIO_29 -> GPIO[8]
	 *      9 --- UART1_RXD and UART1_TXD enable / UART1_RXD and UART1_TXD disable -> GPIO[9]
	 */
	struct intelce_gpio_chip *c = to_intelce_gpio_chip(chip);
	void *reg_base = c->mux_ctl_base;
	uint32_t gpio = chip->base + offset;
	int bit_num = -1;

	switch(gpio) {
		case 0:
			bit_num = 4;
			break;
		case 1:
			bit_num = 3;
			break;
		case 2:
			bit_num = 2;
		case 3 ... 8:
			bit_num = 0;
			break;
		case 9:
			bit_num = 1;
			break;
		default:
			break;
	}
	if (-1 == bit_num) return -EINVAL;

	return (intelce_gpio_mmio_read32(reg_base) >> bit_num) & 0x1;

}


irqreturn_t ce4200_gpio_irq_handler(int irq, void *data)
{
	struct intelce_gpio_chip *c = data;
	u32 irq_stat = intelce_gpio_mmio_read32(c->reg_base + CE4200_PUB_GPIO_INT_STAT);

	irq_stat &= intelce_gpio_mmio_read32(c->reg_base + CE4200_PUB_GPIO_INT_EN);
	if (!irq_stat)
		return IRQ_NONE;

	while (irq_stat) {
		u32 irq_bit = __fls(irq_stat);

		irq_stat &= ~BIT(irq_bit);
		//printk("interterrupt %d coming\n", c->irq_base + irq_bit);
		generic_handle_irq(c->irq_base + irq_bit);
	}

	return IRQ_HANDLED;
}
void ce4200_gpio_irq_mask(struct irq_data *data)
{
	struct intelce_gpio_chip *c = irq_data_get_irq_chip_data(data);
	void *reg_base = c->reg_base;
	u32 orig;

	orig = intelce_gpio_mmio_read32(reg_base + CE4200_PUB_GPIO_INT_EN);
	orig &= ~(1 << (data->irq - c->irq_base));
	intelce_gpio_mmio_write32(orig, reg_base + CE4200_PUB_GPIO_INT_EN);
}

void ce4200_gpio_irq_unmask(struct irq_data *data)
{
	struct intelce_gpio_chip *c = irq_data_get_irq_chip_data(data);
	void *reg_base = c->reg_base;
	u32 orig;

	orig = intelce_gpio_mmio_read32(reg_base + CE4200_PUB_GPIO_INT_EN);
	orig |= (1 << (data->irq - c->irq_base));
	intelce_gpio_mmio_write32(orig, reg_base + CE4200_PUB_GPIO_INT_EN);
}

void ce4200_gpio_irq_eoi(struct irq_data *data)
{
	struct intelce_gpio_chip *c = irq_data_get_irq_chip_data(data);
	void *reg_base = c->reg_base;

	intelce_gpio_mmio_write32(1 << (data->irq -c->irq_base), reg_base + CE4200_PUB_GPIO_INT_STAT);
}

static int ce4200_gpio_irq_set_type(struct irq_data *d, unsigned int type)
{
	struct intelce_gpio_chip *c = irq_data_get_irq_chip_data(d);
	char *reg_base = c->reg_base;
	uint32_t irq_offs = d->irq - c->irq_base;
	uint32_t bit_offs;
	uint32_t reg;
	int ret = 0;

	if ((c->chip.base >= CE4200_PUB_GPIO_BANK1_BASE) || (irq_offs > 4)) {
		switch (type) {
			case IRQ_TYPE_LEVEL_HIGH:
				break;
			default:
				ret = -EINVAL;
				break;
		}
		return ret;
	}

	reg = intelce_gpio_mmio_read32(reg_base + CE4200_PUB_GPIO_INT_TYPE0);

	bit_offs = 4 *irq_offs;
	reg &= ~(0xF << bit_offs);

	switch (type) {
	case IRQ_TYPE_LEVEL_HIGH:
		reg |= (0x0 << bit_offs);
		break;
	case IRQ_TYPE_LEVEL_LOW:
		reg |= (0x1 << bit_offs);
		break;
	case IRQ_TYPE_EDGE_RISING:
		reg |= (0x2 << bit_offs);
		break;
	case IRQ_TYPE_EDGE_FALLING:
		reg |= (0x3 << bit_offs);
		break;
	default:
		return -EINVAL;
	}
	intelce_gpio_mmio_write32(reg, reg_base + CE4200_PUB_GPIO_INT_TYPE0);

	return 0;
}

static struct irq_chip ce4200_irq_chip = {
	.irq_mask = ce4200_gpio_irq_mask,
	.irq_unmask = ce4200_gpio_irq_unmask,
	.irq_eoi = ce4200_gpio_irq_eoi,
	.irq_set_type = ce4200_gpio_irq_set_type,
};

__devinit int ce4200_gpio_irq_setup(struct intelce_gpio_chip *c, struct pci_dev *pdev)
{
	int i;
	int irq;
	int ret;

	c->irq_base = irq_alloc_descs(-1, INTELCE_GPIO_IRQ_BASE, CE4200_PUB_GPIOS_PER_BANK, -1);
	if (c->irq_base < 0)
		return c->irq_base;

	/* mask + ACK all interrupt sources */
	intelce_gpio_mmio_write32(0, c->reg_base + CE4200_PUB_GPIO_INT_EN);
	intelce_gpio_mmio_write32(0xFFF, c->reg_base + CE4200_PUB_GPIO_INT_STAT);

	ret = request_irq(pdev->irq, ce4200_gpio_irq_handler, IRQF_SHARED, "ce4200_gpio", c);
	if (ret)
		goto out_free_desc;

	/*
	 * This gpio irq controller latches level irqs. Testing shows that if
	 * we unmask & ACK the IRQ before the source of the interrupt is gone
	 * then the interrupt is active again.
	 */
	irq = c->irq_base;
	for (i=0; i < c->chip.ngpio; i++) {
		irq_set_chip_and_handler_name(irq, &ce4200_irq_chip, handle_fasteoi_irq, "gpio_irq");
		irq_set_chip_data(irq, c);
		irq++;
	}
	return 0;

out_free_desc:
	irq_free_descs(c->irq_base, CE4200_PUB_GPIOS_PER_BANK);
	return ret;
}

struct gpio_group {
	uint32_t input;
	uint32_t output;
	uint32_t output_enable;
	uint32_t int_status;
	uint32_t int_enable;
	uint32_t int_mode;
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

struct gpio_group group[3];
uint32_t gpio_mux_ctl;
};

static struct _gpio  _gpio;

/*CE4200 gpio suspend routine */
int ce4200_gpio_suspend(void *io_mem, unsigned short io_port)
{
	char *virt_io_mem = (char *)io_mem;
	int i;

    /* keep status of Core Well GPIO */
	_gpio.cgen = intelce_gpio_port_read32(io_port + CE4200_CORE_WELL_GPIO_CGEN);
	_gpio.cgio = intelce_gpio_port_read32(io_port + CE4200_CORE_WELL_GPIO_CGIO);
	_gpio.cglv = intelce_gpio_port_read32(io_port + CE4200_CORE_WELL_GPIO_CGLV);
	_gpio.cgtpe = intelce_gpio_port_read32(io_port + CE4200_CORE_WELL_GPIO_CGTPE);
	_gpio.cgtne = intelce_gpio_port_read32(io_port + CE4200_CORE_WELL_GPIO_CGTNE);
	_gpio.cggpe = intelce_gpio_port_read32(io_port + CE4200_CORE_WELL_GPIO_CGGPE);
	_gpio.cgsmi = intelce_gpio_port_read32(io_port + CE4200_CORE_WELL_GPIO_CGSMI);

    /* keep status of general purpose GPIO */
	_gpio.group[0].int_mode = intelce_gpio_mmio_read32(virt_io_mem + CE4200_PUB_GPIO_INT_TYPE0);
	_gpio.gpio_mux_ctl = intelce_gpio_mmio_read32(virt_io_mem + CE4200_PUB_GPIO_MUX_CTL);
	for (i=0; i < 3; i++) {
		_gpio.group[i].output = intelce_gpio_mmio_read32(virt_io_mem + CE4200_PUB_GPIO_OUT);
		_gpio.group[i].output_enable = intelce_gpio_mmio_read32(virt_io_mem + CE4200_PUB_GPIO_OUT_EN);
		_gpio.group[i].int_enable = intelce_gpio_mmio_read32(virt_io_mem +CE4200_PUB_GPIO_INT_EN);
		virt_io_mem += 0x20;
	}
	return 0;
}

/* CE4200 gpio resume routine */
int ce4200_gpio_resume(void *io_mem, unsigned short io_port)
{
	char *virt_io_mem = (char *)io_mem;
	int i;

    /* restore general purpose GPIO */
	intelce_gpio_mmio_write32(_gpio.group[0].int_mode, virt_io_mem + CE4200_PUB_GPIO_INT_TYPE0);
	intelce_gpio_mmio_write32(_gpio.gpio_mux_ctl, virt_io_mem + CE4200_PUB_GPIO_MUX_CTL);
	for (i=0; i < 3; i++) {
		intelce_gpio_mmio_write32(_gpio.group[i].output, virt_io_mem + CE4200_PUB_GPIO_OUT);
		intelce_gpio_mmio_write32(_gpio.group[i].output_enable, virt_io_mem + CE4200_PUB_GPIO_OUT_EN);
		intelce_gpio_mmio_write32(_gpio.group[i].int_enable, virt_io_mem + CE4200_PUB_GPIO_INT_EN);
		virt_io_mem += 0x20;

	}
    /* restore Core Well GPIO */
	intelce_gpio_port_write32(_gpio.cgen & 0x3FF, io_port + CE4200_CORE_WELL_GPIO_CGEN);
	intelce_gpio_port_write32(_gpio.cgio & 0x3FF, io_port + CE4200_CORE_WELL_GPIO_CGIO);
	intelce_gpio_port_write32(_gpio.cglv & 0x3FF, io_port + CE4200_CORE_WELL_GPIO_CGLV);
	intelce_gpio_port_write32(_gpio.cgtpe & 0x3FF, io_port + CE4200_CORE_WELL_GPIO_CGTPE);
	intelce_gpio_port_write32(_gpio.cgtne & 0x3FF, io_port + CE4200_CORE_WELL_GPIO_CGTNE);
	intelce_gpio_port_write32(_gpio.cggpe & 0x3FF, io_port + CE4200_CORE_WELL_GPIO_CGGPE);
	intelce_gpio_port_write32(_gpio.cgsmi & 0x3FF, io_port + CE4200_CORE_WELL_GPIO_CGSMI);
	return 0;
}
