/*
 *  GPIO interface for Intel CE GPIO.
 *
 *  Copyright (c) 2010, 2012 Intel Corporation
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License 2 as published
 *  by the Free Software Foundation.
 *
 */
#ifndef _INTELCE_GPIO_H
#define _INTELCE_GPIO_H
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


#define INTELCE_GPIO_DRV_NAME		"intelce_gpio"
#define PCI_INTELCE_GPIO_DEVICE_ID	0x2e67

#define INTELCE_GPIO_BAR        0

#define INTELCE_GPIO_IRQ_BASE 128 /* Search free IRQS start from this number*/

struct intelce_gpio_chip{
	int irq_base;
	void __iomem *reg_base;
	void __iomem *high_base;
	void __iomem *mux_ctl_base;
	struct gpio_chip chip;
	struct spinlock lock;
};

static inline uint32_t intelce_gpio_mmio_read32(const volatile void  __iomem *addr)
{
	return readl(addr);
}

static inline void  intelce_gpio_mmio_write32(uint32_t value, volatile void __iomem *addr)
{
	writel(value, addr);
}

static inline uint32_t intelce_gpio_port_read32(int port)
{
	return inl(port);
}

static inline void  intelce_gpio_port_write32(uint32_t value, int port)
{
	outl(value, port);
}

static inline struct intelce_gpio_chip *to_intelce_gpio_chip(struct gpio_chip *c)
{
     return container_of(c, struct intelce_gpio_chip, chip);;
}
#endif
