#ifndef UDMA_SKB_H
#define UDMA_SKB_H

#include <linux/netdevice.h>

#include <linux/udma_api.h>

int udma_register(uint8_t port, struct net_device *netdev,rx_callback_t rx_callback);
int udma_register_netdev(uint8_t port, struct net_device *netdev);

int udma_xmit_skb(unsigned char port,struct sk_buff *skb);
/*
  * void udma_flush(unsigned char	port);
 */

#endif
