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
#include <asm/io.h>

#include <linux/udma_skb.h>

#define DRV_VERSION "0.1"


#define MAX_TRY_TIMES           	10





/* Get MAC Adress like the same way as e1000 */
#define CONFIG_RAM_BASE         0x60000
#define GBE_CONFIG_OFFSET       0x0
#define NODE_ADDRESS_SIZE       6

#define GBE_CONFIG_RAM_BASE \
        ((unsigned int)(CONFIG_RAM_BASE + GBE_CONFIG_OFFSET))

#define GBE_CONFIG_BASE_VIRT \
        ((void __iomem *)phys_to_virt(GBE_CONFIG_RAM_BASE))

#define GBE_CONFIG_FLASH_READ(base, offset, count, data) \
        (ioread16_rep(base + (offset << 1), data, count))



struct udma_adapter{
	unsigned char udma_port;
	u8 mac_addr[NODE_ADDRESS_SIZE];
};
static struct net_device *udma_net_dev[UDMA_PORT_NUM_TOTAL];

/* Set MAC Address */
static void udma_set_mac_addr(unsigned char udma_port, struct net_device *netdev, struct udma_adapter *adapter){
       u16 eeprom_data, i, offset;

      for (i = 0; i < NODE_ADDRESS_SIZE; i += 2) {
               offset = i >> 1;
               GBE_CONFIG_FLASH_READ(GBE_CONFIG_BASE_VIRT, offset, 1,&eeprom_data);
               adapter->mac_addr[i] = (u8) (eeprom_data & 0x00FF);
               adapter->mac_addr[i + 1] = (u8) (eeprom_data >> 8);
       }
       adapter->mac_addr[NODE_ADDRESS_SIZE - 1] += udma_port + 1;

       memcpy(netdev->dev_addr, adapter->mac_addr, netdev->addr_len);
       memcpy(netdev->perm_addr, adapter->mac_addr, netdev->addr_len);
}


static netdev_tx_t udma_net_xmit(struct sk_buff *skb,
				    struct net_device *netdev)
{
	struct udma_adapter *adapter = netdev_priv(netdev);
	unsigned char port;
	size_t size = 0;
	netdev_tx_t ret ;

	port = adapter->udma_port;

	size =	udma_xmit_skb(port, skb);
	if (size) {
		//netdev->stats.tx_bytes += size;
		//netdev->stats.tx_packets++;
		ret = NETDEV_TX_OK;
	} else {
		netif_stop_queue(netdev);
		ret = NETDEV_TX_BUSY;
	}
	return ret;

}
static void udma_net_rx_callback(struct sk_buff *skb,
				    struct net_device *netdev)
{
	static int rx_cnt = 0;
	//int ret = 0;
	struct napi_struct *n = NULL;
	WARN_ON(!skb);
	WARN_ON(!netdev);
//	printk("Received skb from net %8x len %8d",netdev,skb->len);

	netdev->stats.rx_packets++;
	netdev->stats.rx_bytes += skb_headlen(skb);
	//n = list_first_entry(&netdev->napi_list, struct napi_struct, dev_list);
//	skb->ip_summed = CHECKSUM_UNNECESSARY;
	skb->protocol = eth_type_trans(skb, skb->dev);
	netif_receive_skb(skb);
//	ret = napi_gro_receive(n, skb);
//	if (ret == GRO_DROP)
	return;
}

static int udma_net_open(struct net_device *netdev)
{
	struct udma_adapter *adapter = netdev_priv(netdev);
	uint8_t port = adapter->udma_port;
	int ret = 0;

	ret = udma_register(port,netdev,&udma_net_rx_callback);
	if (ret)
		return ret;
	else
		netif_start_queue(netdev);
	return 0;
}

/**
 * udma_close - Disables a network interface
 * @netdev: network interface device structure
 *
 * Returns 0, this is not allowed to fail
 *
 **/

static int udma_net_close(struct net_device *netdev)
{
	struct udma_adapter *adapter = netdev_priv(netdev);

	netif_stop_queue(netdev);
	udma_flush(adapter->udma_port);
	return 0;
}

/* Netdevice get statistics request */

/*static struct rtnl_link_stats64 *
udma_net_get_stats64(struct net_device *dev, struct rtnl_link_stats64 *stats)
{
	return 0;
}*/


static const struct net_device_ops udma_netdev_ops = {
	.ndo_open				= udma_net_open,
	.ndo_stop				= udma_net_close,
	.ndo_start_xmit			= udma_net_xmit,
	.ndo_change_mtu			= eth_change_mtu,
	//.ndo_get_stats64	= udma_net_get_stats64,
};


/**
 * udma_net_init - Driver Registration Routine
 *
 * udma_net_init is the first routine called when the driver is
 * loaded.
 **/
 #define UDMA_NET "eth_udma"
static int __init udma_net_init(void)
{
	struct udma_adapter *adapter = NULL;
	struct net_device *netdev = NULL;
	int i,err;

	for (i = 0; i < UDMA_PORT_NUM_TOTAL; i++) {
		netdev = alloc_etherdev(sizeof(struct udma_adapter));
		if (!netdev){
			return -ENOMEM;
		}

		udma_net_dev[i] = netdev;
		adapter = netdev_priv(netdev);
		adapter->udma_port = i;

		netdev->netdev_ops = &udma_netdev_ops;
		//netdev->features |= NETIF_F_HW_CSUM;
		////NETIF_F_SG;
		sprintf(netdev->name, "%s%d", UDMA_NET, i);
		err = register_netdev(netdev);
		if (err){
			free_netdev(netdev);
			return err;
		}

		/* Currently the udma mac address is generated base on the GBE Mac address */
		udma_set_mac_addr(i, netdev, adapter);

	}

	printk(KERN_INFO "UDMA Network Device Driver init \n");
	return 0;
}

/**
 * udma_net_exit - Driver Exit Cleanup Routine
 *
 **/
static void __exit udma_net_exit(void)
{
	int i;
	struct udma_adapter *adapter;
	for (i = 0; i < UDMA_PORT_NUM_TOTAL; i++) {
		adapter = netdev_priv(udma_net_dev[i]);
		unregister_netdev(udma_net_dev[i]);
		free_netdev(udma_net_dev[i]);
		udma_net_dev[i] = NULL;
	}
	printk(KERN_INFO "UDMA Network Device Driver exit \n");
}
module_init(udma_net_init);
module_exit(udma_net_exit);


MODULE_AUTHOR("Intel Corporation");
MODULE_DESCRIPTION("Intel(R) UDMA Network Device Driver");
MODULE_LICENSE("GPL");
MODULE_VERSION(DRV_VERSION);

/* udma_net.c */
