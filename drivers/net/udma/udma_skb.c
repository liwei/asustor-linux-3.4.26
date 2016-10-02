/*******************************************************************************

  Intel(R) UDMA Network Device Model sample code

  Copyright(c) 2012 Intel Corporation.

  This program is free software; you can redistribute it and/or modify it
  under the terms and conditions of the GNU General Public License,
  version 2, as published by the Free Software Foundation.

  This program is distributed in the hope it will be useful, but WITHOUT
  ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
  FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
  more details.

  You should have received a copy of the GNU General Public License along with
  this program; if not, write to the Free Software Foundation, Inc.,
  51 Franklin St - Fifth Floor, Boston, MA 02110-1301 USA.

  The full GNU General Public License is included in this distribution in
  the file called "COPYING".

*******************************************************************************/

/*
 * UDMA buffer management and skb interaction layer
*/


#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/module.h>
#include <linux/types.h>
#include <linux/init.h>
#include <linux/pci.h>
#include <linux/vmalloc.h>
#include <linux/pagemap.h>
#include <linux/delay.h>
#include <linux/netdevice.h>
#include <linux/tcp.h>
#include <linux/ipv6.h>
#include <linux/slab.h>
#include <net/checksum.h>
#include <net/ip6_checksum.h>
#include <linux/mii.h>
#include <linux/ethtool.h>
#include <linux/if_vlan.h>
#include <linux/cpu.h>
#include <linux/smp.h>
#include <linux/pm_runtime.h>
#include <linux/aer.h>
#include <linux/crc32.h>
#include <asm/io.h>

#include <linux/udma_api.h>
#include <linux/udma_skb.h>



//#define UDMA_LDML_DEBUG 1
#ifdef UDMA_LDML_DEBUG



#define udma_net_dbg(fmt, args...)  do \
				     { \
				           printk(KERN_INFO "\n%s " fmt "\n","udma_skb",##args); \
				     } while(0)
#else
#define udma_net_dbg(fmt, arg...)  do { } while (0)
#endif
#define udma_net_info(fmt, args...)  do \
				     { \
				           printk(KERN_INFO "\n%s " fmt "\n","udma_skb",##args); \
				     } while(0)

#define udma_net_err(fmt, args...)  do \
				     { \
				           printk(KERN_ERR "\n%s " fmt "\n","udma_skb",##args); \
				     } while(0)










int udma_xmit_skb(unsigned char port,struct sk_buff *skb)
{
	udma_result_t ret = UDMA_OK;
	int retried = 30;
	udma_net_dbg("In %s function port %d skb 0x%x\n", __func__,port, skb);

retry:
	ret = udma_send_packet(port, skb);
	if (likely(UDMA_OK == ret))
		return skb->len;
	else if (ret == UDMA_FULL) {
		udma_net_dbg("Tx Warnning: UDMA is full \n");
		if (!retried--) {
			printk("Tx ERROR, UDMA is always full \n");
			goto err_out;
		} else {
			udelay(30);
			goto retry;
		}
	}

err_out:
	dev_kfree_skb_any(skb);
	return 0;
}

int udma_register(uint8_t port, struct net_device *netdev,rx_callback_t rx_handler)
{
        int ret = 0;
        WARN_ON(!rx_handler);

        ret = udma_register_handler(port, netdev,rx_handler);
        if(ret)
                return ret;
        return 0;
}
EXPORT_SYMBOL_GPL(udma_register);



EXPORT_SYMBOL_GPL(udma_xmit_skb);
