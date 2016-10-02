/*
 * xHCI host controller driver PCI Bus Glue.
 *
 * Copyright (C) 2008 Intel Corp.
 *
 * Author: Sarah Sharp
 * Some code borrowed from the Linux EHCI driver.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
 * or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License
 * for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software Foundation,
 * Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#include <linux/pci.h>

#include "etxhci.h"

struct table_item {
	int offset;
	u32 value;
};

static struct table_item cfg_s1_items[] = {
		{0x9b, 0xa1},
		{0x9c, 0x18},
		{0xec, 0x04},
		{0xf5, 0x40},
};
#define CFG_S1_ITEMS	((int)(sizeof(cfg_s1_items)/sizeof(cfg_s1_items[0])))

static struct table_item cfg_s2_items[] = {
		{0x44, 0x00000003},
		{0x2c, 0x70231b6f},
		{0x68, 0x19c001c1},
		{0x68, 0x198001d0},
		{0x68, 0x199001d1},
		{0x68, 0x19eb5003},
		{0x68, 0x19305004},
		{0x68, 0x19005005},
		{0x68, 0x19045007},
		{0x68, 0x19f05022},
		{0x68, 0x19609811},
		{0x68, 0x1950c000},
		{0x68, 0x1906c001},
		{0x68, 0x1950c002},
		{0x68, 0x1921c003},
		{0x68, 0x19ff8008},
		{0x68, 0x19a58010},
		{0x68, 0x19408011},
		{0x68, 0x1980c059},
		{0x68, 0x1904c05f},
		{0x68, 0x1901c063},
		{0x68, 0x1923c071},
		{0x68, 0x1976c072},
		{0x68, 0x1980c075},
		{0x68, 0x1920c07d},
		{0x68, 0x1980c07e},
		{0x68, 0x1900c090},
		{0x68, 0x1930c091},
		{0x68, 0x19a8c0ac},
		{0x68, 0x1961c0ad},
		{0x68, 0x19a8c0bc},
		{0x68, 0x1961c0bd},
		{0x68, 0x1905c0a4},
		{0x68, 0x1905c0b4},
		{0x68, 0x1908c200},
		{0x68, 0x1908c300},
		{0x68, 0x1958c202},
		{0x68, 0x1958c302},
		{0x68, 0x190ec0c0},
		{0x68, 0x1903c0f4},
		{0x68, 0x19619811},
		{0x68, 0x190000ec},
		{0x44, 0x00000000},
};
#define CFG_S2_ITEMS	((int)(sizeof(cfg_s2_items)/sizeof(cfg_s2_items[0])))

void xhci_init_ej168(struct xhci_hcd *xhci)
{
	int i;
	struct usb_hcd *hcd = xhci_to_hcd(xhci);
	struct pci_dev *pdev = to_pci_dev(hcd->self.controller);

	for (i = 0; i < CFG_S1_ITEMS; i++) {
		pci_write_config_byte(pdev, cfg_s1_items[i].offset, (u8)cfg_s1_items[i].value);
		mdelay(1);
	}

	for (i = 0; i < CFG_S2_ITEMS; i++) {
		pci_write_config_dword(pdev, cfg_s2_items[i].offset, cfg_s2_items[i].value);
		mdelay(1);
	}
}
