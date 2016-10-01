/*
 *  GPIO interface for Intel SoC CE4100.
 *
 *  file : ce4100-gpio.h
 *
 *  Copyright (c) 2010, 2012 Intel Corporation
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License 2 as published
 *  by the Free Software Foundation.
 *
 */
#ifndef _CE4100_GPIO_H
#define _CE4100_GPIO_H

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


#define CE4100_PUB_GPIO_OUT				0x00
#define CE4100_PUB_GPIO_OUT_EN			0x04
#define CE4100_PUB_GPIO_INPUT			0x08
#define CE4100_PUB_GPIO_INT_STAT		0x0c

#define CE4100_PUB_GPIO_INT_TYPE0		0x10
#define CE4100_PUB_GPIO_INT_EN			0x14
#define CE4100_PUB_GPIO_INT_TYPE1		0x18

#define CE4100_PUB_GPIO_MUX_CTL			0x1c

#define CE4100_PUB_GPIOS_PER_BANK        12
#define CE4100_PUB_GPIO_BANKS            1


int ce4100_gpio_get(struct gpio_chip *chip, unsigned offset);
void ce4100_gpio_set(struct gpio_chip *chip, unsigned offset, int value);

int ce4100_gpio_direction_input(struct gpio_chip *chip, unsigned offset);
int ce4100_gpio_direction_output(struct gpio_chip *chip, unsigned offset, int value);

int ce4100_set_multi_function(struct gpio_chip *chip, unsigned offset,  int fn_num);
int ce4100_get_multi_function(struct gpio_chip *chip, unsigned offset);


int ce4100_gpio_irq_setup(struct intelce_gpio_chip *c, struct pci_dev *pdev);
int ce4100_gpio_to_irq(struct gpio_chip *chip, unsigned offset);

#endif
