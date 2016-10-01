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

#ifndef _CE2600_GPIO_H
#define _CE2600_GPIO_H

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


#define CE2600_PUB_GPIOS_PER_BANK        32
#define CE2600_PUB_GPIO_BANKS            4

#define CE2600_PUB_GPIO_BANK0_BASE		0x0
#define CE2600_PUB_GPIO_BANK1_BASE		0x20
#define CE2600_PUB_GPIO_BANK2_BASE		0x40
#define CE2600_PUB_GPIO_BANK3_BASE		0x60

#define CE2600_PUB_GPIO_OUT				0x00
#define CE2600_PUB_GPIO_OUT_EN			0x04
#define CE2600_PUB_GPIO_INPUT			0x08
#define CE2600_PUB_GPIO_INT_STAT		0x0c

#define CE2600_PUB_GPIO_INT_EN			0x10
#define CE2600_PUB_GPIO_INT_MODE_LE		0x14
#define CE2600_PUB_GPIO_INT_MODE_RF		0x18

#define CE2600_PUB_GPIO_MUX_CTL			0x1c

#define CE2600_PUB_GPIO_BANK0_HIGH_BASE		0x80
#define CE2600_PUB_GPIO_BANK1_HIGH_BASE		0x90
#define CE2600_PUB_GPIO_BANK2_HIGH_BASE		0xA0
#define CE2600_PUB_GPIO_BANK3_HIGH_BASE		0xB0

#define CE2600_PUB_GPIO_CLEAR       		0x00
#define CE2600_PUB_GPIO_SET     		  	0x04


#define CE2600_PUB_GPIO_POLARITY0	0x88
#define CE2600_PUB_GPIO_POLARITY1	0x98
#define CE2600_PUB_GPIO_POLARITY2	0xa8
#define CE2600_PUB_GPIO_POLARITY3	0xb8

#define CE2600_PUB_GPIO_INT_ROUTER  0xbc

/* Core Well GPIO Group in North brige */
/* Core Well GPIO[7:0] */
#define CE2600_CORE_WELL_GPIO_CGEN		0x00
#define CE2600_CORE_WELL_GPIO_CGIO		0x04
#define CE2600_CORE_WELL_GPIO_CGLV		0x08
#define CE2600_CORE_WELL_GPIO_CGTPE		0x0C
#define CE2600_CORE_WELL_GPIO_CGTNE		0x10
#define CE2600_CORE_WELL_GPIO_CGGPE		0x14
#define CE2600_CORE_WELL_GPIO_CGSMI		0x18
#define CE2600_CORE_WELL_GPIO_CGTS		0x1C


void ce2600_gpio_set(struct gpio_chip *chip, unsigned offset, int value);
int ce2600_gpio_direction_output(struct gpio_chip *chip, unsigned offset, int value);
int ce2600_gpio_direction_input(struct gpio_chip *chip, unsigned offset);
int ce2600_set_multi_function(struct gpio_chip *chip, unsigned offset,  int fn_num);
int ce2600_get_multi_function(struct gpio_chip *chip, unsigned offset);
int ce2600_gpio_irq_setup(struct intelce_gpio_chip *c, struct pci_dev *pdev);

int ce2600_gpio_suspend(void *io_mem, unsigned short io_port);
int ce2600_gpio_resume(void *io_mem, unsigned short io_port);

#endif
