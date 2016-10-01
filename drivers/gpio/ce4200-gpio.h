/*
 *  GPIO interface for Intel CE4200.
 *
 *  Copyright (c) 2010, 2011 Intel Corporation
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License 2 as published
 *  by the Free Software Foundation.
 *
 */
#ifndef _CE4200_GPIO_H
#define _CE4200_GPIO_H

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

#define CE4200_PUB_GPIOS_PER_BANK        32
#define CE4200_PUB_GPIO_BANKS            3

#define CE4200_PUB_GPIO_BANK0_BASE		0x0
#define CE4200_PUB_GPIO_BANK1_BASE		0x20
#define CE4200_PUB_GPIO_BANK2_BASE		0x40
#define CE4200_PUB_GPIO_BANK3_BASE		0x60

#define CE4200_PUB_GPIO_OUT				0x00
#define CE4200_PUB_GPIO_OUT_EN			0x04
#define CE4200_PUB_GPIO_INPUT			0x08
#define CE4200_PUB_GPIO_INT_STAT		0x0c

#define CE4200_PUB_GPIO_INT_EN			0x10

#define CE4200_PUB_GPIO_INT_TYPE0		0x14

#define CE4200_PUB_GPIO_MUX_CTL			0x18

/* Core Well GPIO Group in North brige */
/* Core Well GPIO[7:0] */
#define CE4200_CORE_WELL_GPIO_CGEN		0x00
#define CE4200_CORE_WELL_GPIO_CGIO		0x04
#define CE4200_CORE_WELL_GPIO_CGLV		0x08
#define CE4200_CORE_WELL_GPIO_CGTPE		0x0C
#define CE4200_CORE_WELL_GPIO_CGTNE		0x10
#define CE4200_CORE_WELL_GPIO_CGGPE		0x14
#define CE4200_CORE_WELL_GPIO_CGSMI		0x18
#define CE4200_CORE_WELL_GPIO_CGTS		0x1C

int ce4200_set_multi_function(struct gpio_chip *chip, unsigned offset,  int fn_num);
int ce4200_get_multi_function(struct gpio_chip *chip, unsigned offset);

irqreturn_t ce4200_gpio_irq_handler(int irq, void *data);
void ce4200_gpio_irq_mask(struct irq_data *data);
void ce4200_gpio_irq_unmask(struct irq_data *data);
void ce4200_gpio_irq_eoi(struct irq_data *data);

int ce4200_gpio_irq_setup(struct intelce_gpio_chip *c, struct pci_dev *pdev);

int ce4200_gpio_suspend(void *io_mem, unsigned short io_port);
int ce4200_gpio_resume(void *io_mem, unsigned short io_port);
#endif
