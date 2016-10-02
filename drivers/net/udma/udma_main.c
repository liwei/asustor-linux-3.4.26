/*
 * GPL LICENSE SUMMARY
 *
 *  Copyright(c) 2012 Intel Corporation. All rights reserved.
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of version 2 of the GNU General Public License as
 *  published by the Free Software Foundation.
 *
 *  This program is distributed in the hope that it will be useful, but
 *  WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *  General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 51 Franklin St - Fifth Floor, Boston, MA 02110-1301 USA.
 *  The full GNU General Public License is included in this distribution
 *  in the file called LICENSE.GPL.
 *
 *  Contact Information:
 *    Intel Corporation
 *    2200 Mission College Blvd.
 *    Santa Clara, CA  97052
 *
 */

/* UDMA  Driver main stack */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/device.h>
#include <asm/uaccess.h>
#include <linux/types.h>
#include <linux/errno.h>
#include <linux/pci.h>

#include <linux/spinlock.h>

#include <linux/interrupt.h>

#include <linux/udma_api.h>
#include <linux/delay.h>

#include "udma_hw.h"
#include "udma_main.h"
#include <linux/hrtimer.h>
#include <linux/ktime.h>

#include <linux/time.h>
#ifdef DEBUG
#include <linux/proc_fs.h>
#include <linux/inet.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/skbuff.h>
#include <linux/ethtool.h>
#include <net/sock.h>
#include <net/checksum.h>
#include <linux/if_ether.h>	/* For the statistics structure. */
#include <linux/if_arp.h>	/* For ARPHRD_ETHER */
#include <linux/ip.h>
#include <linux/tcp.h>
#include <linux/percpu.h>
#include <net/net_namespace.h>
#include <linux/u64_stats_sync.h>

#endif
unsigned int rx_drop_cnt = 0;
unsigned int rx_link_cnt = 0;
unsigned int hr_cnt = 0;

static u32 send_cnt = 0;
static u32 give_cnt = 0;
static u32 send_coming = 0;
static u32 send_period = 0;
static u32 udma_napi_done = 0;
static u32 send_period_old = 0;
static struct timeval tv;
static struct timeval tv1;
static struct timeval tv2;
static struct timeval tmp_tv1;
static struct timeval tmp_tv2;
static u32 timearray[1000] = {0};
static u32 timearray2[1000] = {0};
static u32 total_rx_packets_record[5] = {0,0,0,0,0};

/* This is the maximum VLAN size*/
#define UDMA_MIN_RX_BUFFER_SIZE 		(1522)


#define INTR_FREQ_IN_LOWEST_LATENCY 		70000
#define INTR_FREQ_IN_LOW_LATENCY 		16000
#define INTR_FREQ_IN_BULK_LATENCY 		8000


//#define UDMA_RX_IRQ_TUNING_ENABLED		1

#ifdef UDMA_RX_IRQ_TUNING_ENABLED
#define RX_PACKETS_PER_INTR		21 	//7 packets per interrupt
#else
#define RX_PACKETS_PER_INTR		1 	//7 packets per interrupt
#endif
static int udma_start_rx_transfer(struct udma_device *umdev);
static int udma_start_tx_transfer(struct udma_device *umdev);
static int udma_give_free_buffer(unsigned char port, int cleaned_count);
static int  udma_setup_all_ring_resources(struct udma_device *udma_dev);
static int  udma_free_all_ring_resources(struct udma_device *udma_dev);

static void  udma_init_tx_ring_descs(struct udma_device *umdev, struct udma_ring *tx);
static void  udma_init_rx_ring_descs(struct udma_device *umdev, struct udma_ring *tx);
static int 	udma_ring_clean_irq(struct udma_device *umdev, struct udma_ring *ring, const int budget, bool dir);
static inline u16 __get_using_desc(struct udma_ring *ring);
#ifdef DEBUG
#define UDMA_WARN_ON_RETURN(condition,ret) ({			\
		int __ret_warn_on = !!(condition);				\
		if (__ret_warn_on) return ret;					\
	})
#else
#define UDMA_WARN_ON_RETURN(condition,ret) do{}while(0)
#endif

static inline u16 __get_using_desc(struct udma_ring *ring)
{
	u16 i = 0;
	i = ring->to_be_use;
	/* Get to_be_used updated */
	while((i!=ring->tail) && DESC_IS_DONE(INDEX_TO_DESC(i,ring)))
		i = DESC_INDEX_INC(i);
	return i;
}


#ifdef DEBUG
static inline void udma_dump_a_ring(struct udma_device *umdev, struct udma_ring *ring, bool direction);
/*
 * Following code are for debug purpose
*/
u32 g_order[2];
#define ORDER_INC(o) ((o+1)%0xfffffff0)
void udma_dump_frame(void * buffer, size_t size, int direction)
{
	unsigned char *p = NULL;
	int i = 0;
	unsigned int size_1 = size/8;
	unsigned int size_2 = size%8;
	p = (unsigned char *)buffer;
	udma_dbg("\n==========================%s=============================\n",(direction == UDMA_TX)?"Tx":"Rx");
	udma_dbg(" Packet size %d bytes, size1 %d size2 %d\n",size,size_1,size_2);
	if (size_1) {
		for (i = 0;i< size_1;i++) {
			if (!(unsigned char *)(p+i*8))
				return;
			udma_dbg("[%d]	 %2x  %2x  %2x  %2x  %2x  %2x  %2x  %2x ",i*8+1,\
				*(unsigned char *)(p+i*8),*(unsigned char *)(p+i*8+1),\
				*(unsigned char *)(p+i*8+2),*(unsigned char *)(p+i*8+3), \
				*(unsigned char *)(p+i*8+4),*(unsigned char *)(p+i*8+5), \
				*(unsigned char *)(p+i*8+6),*(unsigned char *)(p+i*8+7));
		}
		p+=(size_1*8);
	}
	if (size_2) {
		for (i = 0;i< size_2;i++) {
			udma_dbg("[%d]	 %2x",8*size_1+i+1,*(unsigned char *)(p+i));
		}
	}
	udma_dbg("\n");
	udma_dbg("End Packet size %d bytes\n",size);

	return ;
}
#if 0
static void udma_dump_current_desc(struct udma_device *umdev, bool direction)
{
	struct udma_ring *ring = NULL;
	if (UDMA_TX == direction)
		ring = &umdev->tx_ring;
	else
		ring = &umdev->rx_ring;
	udma_dump_a_ring(umdev, ring, direction);
	return;
}
#endif
#endif
/**
 * udma_dump - Print registers, tx-ring and rx-ring
 **/
#ifdef DEBUG
#define udma_dump_a_ring_progress(ring)	do \
	{\
	udma_info("%8s clean[%2d] use[%2d] tail[%2d] new tail [%2d]\n",__FUNCTION__,ring->to_be_clean, ring->to_be_use, ring->tail,ring->new_tail); \
	} while (0)

static inline void udma_dump_a_ring(struct udma_device *umdev, struct udma_ring *ring, bool direction)
{
	int i;
	udma_info("\n ============Ring=======Stat================================= \n");
	udma_info("\n 		RingNo 	     		 dma 	0x%x  \n",  (u32)ring->ring_dma);
	udma_info("\n 		size 	    0x%x	 \n", ring->ring_dma_size);

	udma_info("\n 		cnt 	    %d 		 desc 0x%p     \n", ring->ring_entries, ring->desc);

	udma_info("\n 		clean[%d]	use[%d]	 tail[%d] newtail[%d]\n", ring->to_be_clean, ring->to_be_use, ring->tail,ring->new_tail);


	i = 0;
	while ( i< ring->ring_entries) {
		udma_info("\n  No. %d  \n", i);
		udma_desc_dump(INDEX_TO_DESC(i,ring));
		i++;
	}

	udma_info("\n =========================================================== \n");

}
#if 0
static void udma_dump_all_rings(u8 port)
{
	struct udma_device *umdev = umdev = udma_devs[port];
	struct udma_ring *tx= &umdev->tx_ring;
	struct udma_ring *rx= &umdev->rx_ring;
	udma_info("\n================================================================\n");
	udma_info("\n");
	udma_info("\n 		UDMA data structure dump 	\n");
	udma_info("\n");
	udma_info("\n 		desc size 0x%x \n", sizeof(struct udma_desc));

	udma_dump_a_ring(umdev, tx, UDMA_TX);
	udma_dump_a_ring(umdev, rx, UDMA_RX);

	udma_info("\n================================================================\n");

	udma_info("\n");
	return ;
}
#endif
#else

#define udma_dump_all_rings(port) \
do { \
} while(0)

#endif
static inline void udma_napi_schedule(struct udma_device *umdev)
{
	if (napi_schedule_prep(&umdev->napi)) {
		umdev->total_tx_bytes	 	= 0;
		umdev->total_tx_packets 	= 0;
		umdev->total_rx_bytes		= 0;
		umdev->total_rx_packets	 	= 0;
		__napi_schedule(&umdev->napi);
	}

}

static bool udma_rx_is_stopped(struct udma_device *umdev)
{
	struct udma_hw *hw = umdev->hw;
	struct udma_desc *udma_desc = NULL;
	struct udma_ring *rx = &umdev->rx_ring;
	u32 desc;
	int index;
	bool ret = false;

	if (hw->ops->rx_is_active(hw))
		return false;
	desc = hw->ops->get_next_rx_desc(hw);
	if (unlikely(desc == 0)) {
		return true;
	}
	desc = hw->ops->get_curr_rx_desc(hw);

	index = DESC_DMA_TO_DESC_INDEX(desc,rx);
	udma_desc = INDEX_TO_DESC(rx->tail,rx);
	if ((index == rx->tail) && DESC_IS_DONE(udma_desc)) {
			ret = true;
	}
	return ret;
}

static bool udma_tx_is_stopped(struct udma_device *umdev)
{
	struct udma_hw *hw = umdev->hw;
	struct udma_desc *udma_desc = NULL;
	struct udma_ring *tx = &umdev->tx_ring;
	u32 desc;
	int index;
	bool ret = false;

	if (hw->ops->tx_is_active(hw))
		return false;

	desc = hw->ops->get_next_tx_desc(hw);
	if (unlikely(desc == 0)) {
		return true;
	}
	desc = hw->ops->get_curr_tx_desc(hw);
	index = DESC_DMA_TO_DESC_INDEX(desc,tx);
	udma_desc = INDEX_TO_DESC(tx->tail,tx);
	if ((index == tx->tail) && DESC_IS_DONE(udma_desc)) {
		ret = true;
	}
	return ret;
}

/* Pop a descriptor in the ring and reclaim resource  */
static void udma_tx_ring_pop(struct udma_device *umdev, struct udma_ring *tx, const unsigned int i)
{
	struct udma_hw *hw = umdev->hw;
	struct udma_desc *desc = NULL;
	struct udma_buffer *buffer_info = NULL;

	desc = INDEX_TO_DESC(i,tx);
	buffer_info = &tx->buffer_info[i];

	umdev->total_tx_bytes += buffer_info->length - DESC_DATA_BUFFER_LEN(desc);

	if (likely(buffer_info->dma)) {
		dma_unmap_single(&hw->pdev->dev, buffer_info->dma,
			  buffer_info->length,DMA_TO_DEVICE);
		buffer_info->dma = 0;
	}
	if (likely(buffer_info->skb)) {
		dev_kfree_skb_any(buffer_info->skb);
		buffer_info->skb = 0;
	}

	return ;
}

/* Pop a descriptor in the ring and reclaim resource  */
static void udma_rx_ring_pop(struct udma_device *umdev, struct udma_ring *rx, const unsigned int i)
{
	struct udma_hw *hw = umdev->hw;
	struct udma_desc *desc = NULL;
	struct udma_buffer *buffer_info = NULL;
	u32 data_size = 0;

	buffer_info = &rx->buffer_info[i];
	desc = INDEX_TO_DESC(i,rx);
	data_size = UDMA_MIN_RX_BUFFER_SIZE - DESC_DATA_BUFFER_LEN(desc);
	umdev->total_rx_bytes += data_size;

	if (likely(buffer_info->dma)) {
		dma_unmap_single(&hw->pdev->dev, buffer_info->dma,
				 buffer_info->length,DMA_FROM_DEVICE);
		buffer_info->dma = 0;
	}

	/* Transfer to net stack */
	data_size -= ETH_FCS_LEN;
	skb_put(buffer_info->skb, data_size);
	umdev->rx_complete(buffer_info->skb, udma_netdevs[hw->port]);
	buffer_info->skb = NULL;

	return ;
}

/* Pop a descriptor in the ring and reclaim resource  */
static void udma_tx_ring_drop(struct udma_device *umdev, struct udma_ring *tx, const unsigned int i)
{
	struct udma_hw *hw = umdev->hw;
	struct udma_desc *desc = NULL;
	struct udma_buffer *buffer_info = NULL;

	desc = INDEX_TO_DESC(i,tx);
	buffer_info = &tx->buffer_info[i];

	if (likely(buffer_info->dma)) {
		dma_unmap_single(&hw->pdev->dev, buffer_info->dma,
			  buffer_info->length,DMA_TO_DEVICE);
		buffer_info->dma = 0;
	}
	if (likely(buffer_info->skb)) {
		dev_kfree_skb_any(buffer_info->skb);
		buffer_info->skb = 0;
	}

	return ;
}



/* Pop a descriptor in the ring and reclaim resource  */
static void udma_rx_ring_drop(struct udma_device *umdev, struct udma_ring *rx, const unsigned int i)
{
	struct udma_hw *hw = umdev->hw;
	struct udma_desc *desc = NULL;

	struct udma_buffer *buffer_info = NULL;

	buffer_info = &rx->buffer_info[i];
	desc = INDEX_TO_DESC(i,rx);

	if (buffer_info->dma) {
		dma_unmap_single(&hw->pdev->dev, buffer_info->dma,
				 buffer_info->length,DMA_FROM_DEVICE);
	}
	if (buffer_info->skb) {
		dev_kfree_skb_any(buffer_info->skb);
		buffer_info->skb = NULL;
	}


	return ;
}


uint32_t rx_cnt = 0;
uint32_t tx_cnt = 0;
static u32 inline avarage_rx_packets(void)
{
	return (total_rx_packets_record[0]+total_rx_packets_record[1]+total_rx_packets_record[2]+total_rx_packets_record[3]+total_rx_packets_record[4])/5;
}
static void inline udma_irq_enable(struct udma_device * umdev)
{
	struct udma_hw *hw = umdev->hw;
	hw->ops->enable_tx_irq(hw);
	hw->ops->enable_rx_irq(hw);
	return ;
}
static void inline udma_irq_disable(struct udma_device * umdev)
{
	struct udma_hw *hw = umdev->hw;
	hw->ops->disable_tx_irq(hw);
	hw->ops->disable_rx_irq(hw);
	return ;
}

/**
 * udma_intr - Interrupt Handler
 * @irq: interrupt number
 * @data: pointer to a private device structure
 **/

static irqreturn_t udma_intr(int irq, void *data)
{
	struct udma_device *umdev = (struct udma_device *)data;
	struct udma_hw *hw = umdev->hw;
	struct udma_ring *rx =&umdev->rx_ring;
	struct udma_ring *tx = &umdev->tx_ring;
	u32 status = hw->ops->get_irq_status(hw);
	u32 rx_free = UDMA_RING_ENTRY_NUM;
	u32 ava = 0;

	if (!(UDMA_HW_VALID_INTR_STATE(hw->port) & status)) {
		return IRQ_NONE;
	}
	/* Tx interrupt */
	if (status & UDMA_HW_VALID_TX_INTR_STATE(hw->port)) {
		udma_dbg("Tx udma_intr status 0x%x \n",status);
		/*Read the interrupt again before clearing it */
		status = hw->ops->get_irq_status(hw);
		tx_cnt ++;
		hw->ops->clear_tx_irq(hw);
		spin_lock(&tx->lock);
		tx->to_be_use	= __get_using_desc(tx);
		spin_unlock(&tx->lock);



	}

	if (status & UDMA_HW_VALID_RX_INTR_STATE(hw->port)) {
		udma_dbg("Rx udma_intr status 0x%x \n",status);
		/*Read the interrupt again before clearing it */
		rx_cnt ++;
		hw->ops->clear_rx_irq(hw);

		spin_lock(&rx->lock);
		rx->to_be_use =	__get_using_desc(rx);
		rx_free = (rx->tail + UDMA_RING_ENTRY_NUM - rx->to_be_use)%UDMA_RING_ENTRY_NUM;
		spin_unlock(&rx->lock);
		ava = avarage_rx_packets();
	}
	udma_irq_disable(umdev);
#if 0
	if (ava && (rx_free < ava))
		umdev->itr_ns = umdev->itr_ns/ava *rx_free;
	else if (unlikely(!ava)) {
		udma_napi_schedule(umdev);
		return IRQ_HANDLED;
	}
#endif
	if (rx_free < ava * 65/100)
		udma_napi_schedule(umdev);
	else if (!hrtimer_active(&umdev->itr_timer))
		hrtimer_start(&umdev->itr_timer, ns_to_ktime(umdev->itr_ns), HRTIMER_MODE_REL);
	return IRQ_HANDLED;
}

/*
 * udma_clean_ring: Clean the unused descriptors in a ring
*/
static int udma_clean_ring(struct udma_device *umdev, struct udma_ring *ring, bool dir)
{

	struct udma_desc *desc = NULL;
	int i = 0;
	unsigned long size = 0;
	int cleaned = 0;

	i = 0;
	for (i = 0; i< ring->ring_entries; i++) {
		desc = INDEX_TO_DESC(i,ring);
		if (UDMA_TX == dir)
			udma_tx_ring_drop(umdev,ring,i);
		else
			udma_rx_ring_drop(umdev,ring,i);
		cleaned ++;
	}
	size = sizeof(struct udma_buffer) * ring->ring_entries;
	memset(ring->buffer_info, 0, size);

	/* Zero out the descriptor ring */
	memset(ring->desc, 0, ring->ring_dma_size);

	ring->tail = 0;
	ring->new_tail = 0;
	ring->to_be_clean = 0;
	ring->to_be_use = 0;

	return cleaned;
}

/*
 * udma_ring_clean_irq: Clean the finished descriptors in a ring
*/
static int udma_ring_clean_tx_irq(struct udma_device *umdev, const int budget)
{
	int i,cleaned = 0;
	int next = 0;
	int using = 0;
	struct udma_desc *desc = NULL;
	struct udma_desc *next_desc = NULL;
	struct udma_ring *ring = &umdev->tx_ring;

	unsigned long flags = 0;

	spin_lock_irqsave(&ring->lock, flags);
	ring->to_be_use = __get_using_desc(ring);
	using = ring->to_be_use;
	spin_unlock_irqrestore(&ring->lock, flags);

	i = ring->to_be_clean;
	desc = INDEX_TO_DESC(i,ring);

	while (i != using) {
		if (cleaned >= budget)
			break;
		rmb();
		next = DESC_INDEX_INC(i);
		next_desc = INDEX_TO_DESC(next,ring);
		prefetch(next_desc);
#if 0
		if (unlikely(DESC_IS_IN_PROGRESS(desc) || \
			(DESC_IS_INITIAL(desc) && !DESC_IS_NULL(desc)))) {
			break;
		}
#endif
		if (likely(DESC_IS_DONE(desc))) {
			udma_tx_ring_pop(umdev,ring,i);
			cleaned ++;
		}
		i = next;
		desc = next_desc;
	}
	ring->to_be_clean = i;
	return cleaned;

}

static int udma_ring_clean_rx_irq(struct udma_device *umdev,const int budget)
{
	int i,cleaned = 0;

	int using = 0;
	static int j = 0;
	struct udma_desc *desc = NULL;

	struct udma_ring *ring = &umdev->rx_ring;

	unsigned long flags = 0;

	spin_lock_irqsave(&ring->lock, flags);
	ring->to_be_use = __get_using_desc(ring);
	using = ring->to_be_use;
	spin_unlock_irqrestore(&ring->lock, flags);


	i = ring->to_be_clean;
	desc = INDEX_TO_DESC(i,ring);

#if 1
	while (i != using) {
		if (cleaned >= budget)
			break;
		rmb();
		if (likely(DESC_IS_DONE(desc))) {
			udma_rx_ring_pop(umdev,ring,i);
			cleaned ++;
		}

		i = DESC_INDEX_INC(i);
		desc = INDEX_TO_DESC(i,ring);
	}
#endif
	ring->to_be_clean = i;
	if (cleaned)
		udma_give_free_buffer(umdev->hw->port,cleaned);
#if 0
	do_gettimeofday(&tv2);
	if ((tv1.tv_usec - tv.tv_usec) && (tv2.tv_usec - tv1.tv_usec)) {
		timearray[j%1000] = cleaned * 1453/(tv1.tv_usec - tv.tv_usec);
		timearray2[j++%1000] =cleaned * 1453/(tv2.tv_usec - tv.tv_usec);
	}
#endif
	return cleaned;

}


static int udma_clean_all_rings(struct udma_device *umdev,bool dir)
{
	struct udma_ring *ring = NULL;
	u32 cleaned = 0;
	if (UDMA_TX == dir)
		ring = &umdev->tx_ring;
	else
		ring = &umdev->rx_ring;
	cleaned	+= udma_clean_ring(umdev,ring,dir);
	return cleaned;
}
static int udma_clean_tx_irq(struct udma_device *umdev)
{
	struct udma_hw *hw = umdev->hw;
	unsigned long flags;
	int done = 0;

	hw->ops->clear_tx_irq(hw);
	done = udma_ring_clean_tx_irq(umdev,UDMA_RING_VALID_MAX_NUM);

	spin_lock_irqsave(&umdev->tx_ring.lock, flags);
	if (udma_tx_is_stopped(umdev)) {
		udma_start_tx_transfer(umdev);
	}
	spin_unlock_irqrestore(&umdev->tx_ring.lock, flags);
	send_cnt += done;
	umdev->total_tx_packets += done;
	return done < UDMA_RING_VALID_MAX_NUM;
}
static int udma_clean_rx_irq(struct udma_device *umdev, const int budget)
{
	struct udma_hw *hw = umdev->hw;
	unsigned long flags;
	int done = 0;

	hw->ops->clear_rx_irq(hw);
	done = udma_ring_clean_rx_irq(umdev,budget);

	spin_lock_irqsave(&umdev->rx_ring.lock, flags);
	if (udma_rx_is_stopped(umdev)) {
		udma_start_rx_transfer(umdev);
	}
	spin_unlock_irqrestore(&umdev->rx_ring.lock, flags);
	umdev->total_rx_packets += done;
	return done;
}
static void udma_watchdog(unsigned long data)
{
	struct udma_device * umdev = (struct udma_device *)data;
	struct udma_ring *rx = &umdev->rx_ring;

	int i = 0;
	struct udma_desc *desc = NULL;

	//printk("WATCH dog time out! jiffies %8d\n", jiffies);
	i = rx->to_be_clean;
	desc = INDEX_TO_DESC(i,rx);
	mod_timer(&umdev->watchdog_timer,jiffies+1);
	if (!DESC_IS_DONE(desc))
		return ;
	else {
		if (napi_schedule_prep(&umdev->napi))
			__napi_schedule(&umdev->napi);
	}
	return;

}
static unsigned int udma_update_itr(struct udma_device *umdev,
				     u16 itr_setting, int packets,
				     int bytes)
{
	unsigned int retval = itr_setting;
	static int i = 0;


	if (packets == 0)
		goto update_itr_done;

	switch (itr_setting) {
		case lowest_latency:
			/* handle TSO and jumbo frames */
			if (bytes/packets > 8000)
				retval = bulk_latency;
			else if ((packets < 5) && (bytes > 512))
				retval = low_latency;
			break;
		case low_latency:  /* 50 usec aka 20000 ints/s */
			if (bytes > 10000) {
				/* this if handles the TSO accounting */
				if (bytes/packets > 8000)
					retval = bulk_latency;
				else if ((packets < 10) || ((bytes/packets) > 1200))
					retval = bulk_latency;
				else if ((packets > 35))
					retval = lowest_latency;
			} else if (bytes/packets > 2000) {
				retval = bulk_latency;
			} else if (packets <= 2 && bytes < 512) {
				retval = lowest_latency;
			}
			break;
		case bulk_latency: /* 250 usec aka 4000 ints/s */
			if (bytes > 25000) {
				if (packets > 35)
					retval = low_latency;
			} else if (bytes < 6000) {
				retval = low_latency;
			}
			break;
		}

update_itr_done:
	return retval;
}

static void udma_set_itr(struct udma_device *umdev)
{
	static int i = 0;
	u16 current_itr;
	u32 new_itr = umdev->itr;

	umdev->tx_itr = udma_update_itr(umdev,
				    umdev->tx_itr,
				    umdev->total_tx_packets,
				    umdev->total_tx_bytes);

	umdev->rx_itr = udma_update_itr(umdev,
				    umdev->rx_itr,
				    umdev->total_rx_packets,
				    umdev->total_rx_bytes);
	total_rx_packets_record[i++%5] = umdev->total_rx_packets;

	current_itr = max(umdev->rx_itr, umdev->tx_itr);

	switch (current_itr) {
	/* counts and packets in update_itr are dependent on these numbers */
	case lowest_latency:
		new_itr = INTR_FREQ_IN_LOWEST_LATENCY;
		break;
	case low_latency:
		new_itr = INTR_FREQ_IN_LOW_LATENCY;
		break;
	case bulk_latency:
		new_itr = INTR_FREQ_IN_BULK_LATENCY;
		break;
	default:
		break;
	}

set_itr_now:
	if (new_itr != umdev->itr) {
		/*
		 * this attempts to bias the interrupt rate towards Bulk
		 * by adding intermediate steps when interrupt rate is
		 * increasing
		 */
		new_itr = new_itr > umdev->itr ?
			     min(umdev->itr + (new_itr >> 2), new_itr) :
			     new_itr;
		umdev->itr = new_itr;
		if (likely(new_itr))
			umdev->itr_ns = 1000000000/new_itr;
		else
			umdev->itr_ns = 0;
	}
}

static int udma_clean(struct napi_struct *napi, int budget)
{

	struct udma_device * umdev = container_of(napi, struct udma_device , napi);
	struct udma_hw *hw = umdev->hw;

	int tx_clean_complete = 0, work_done = 0;

	PRINT_FUNC

	tx_clean_complete = udma_clean_tx_irq(umdev);

	work_done = udma_clean_rx_irq(umdev,budget);
	give_cnt += work_done;


	if (!tx_clean_complete)
		work_done = budget;

	if (work_done < budget) {
		udma_set_itr(umdev);
		napi_complete(napi);
		udma_irq_enable(umdev);
		udma_napi_done ++;
	}

	return work_done;
}


/****************************************************************************
*
*
****************************************************************************/


static int udma_start_rx_transfer(struct udma_device *umdev)
{
	struct udma_hw *hw = umdev->hw;
	struct udma_ring *rx = &umdev->rx_ring;
	struct udma_desc *udma_desc_new_tail;



	PRINT_FUNC

	if (rx->tail == rx->new_tail) {
		return -1;
	}

	rx->to_be_use = NEXT_RX(rx->tail);
	udma_desc_new_tail = &rx->desc[rx->new_tail];

	SET_END_DESC_FLAG(udma_desc_new_tail);

	rx->tail = rx->new_tail;
	#if 0
	//rlink_cnt = min(rlink_cnt,(rx->tail + rx->ring_entries - rx->to_be_use + 1)%rx->ring_entries);
	len = TO_DESC_INDEX(rx->tail + rx->ring_entries- rx->to_be_use) +1;
	if (rx_link_cnt)
		rx_link_cnt = max(rx_link_cnt, len);
	else
		rx_link_cnt = len;
//	printk("start rx to_be_use %8d tail %8d, new tail %8d len %8d\n", rx->to_be_use, rx->tail, rx->new_tail,len);
	/* A link list start from to_be_use, end at tail */
	for (i = 1; i<= (len/RX_PACKETS_PER_INTR); i++) {
		desc = &rx->desc[TO_DESC_INDEX(i*RX_PACKETS_PER_INTR + rx->to_be_use -1)];
		hw->ops->enable_desc_rx_irq(desc);
	}
	if (len%RX_PACKETS_PER_INTR) {
		desc = &rx->desc[rx->tail];
		hw->ops->enable_desc_rx_irq(desc);
	}
	#ifdef UDMA_RX_IRQ_TUNING_ENABLED
	mod_timer(&umdev->watchdog_timer,jiffies+1);
	#endif
	#endif
	hw->ops->start_rx_transfer(hw, DESC_INDEX_TO_DESC_DMA(rx->to_be_use,rx));


	return 0;


}
static u32 link_cnt = 0;
static int udma_start_tx_transfer(struct udma_device *umdev)
{
	struct udma_hw *hw = umdev->hw;
	struct udma_desc *udma_desc_new_tail;
	struct udma_desc *udma_desc_to_be_use;
	struct udma_ring *tx = &umdev->tx_ring;


	if (tx->tail == tx->new_tail) {
		return -1;
	}
	tx->to_be_use = NEXT_TX(tx->tail);
	udma_desc_new_tail = &tx->desc[tx->new_tail];
	udma_desc_to_be_use = &tx->desc[tx->to_be_use];
	SET_END_DESC_FLAG(udma_desc_new_tail);
//	hw->ops->enable_desc_tx_irq(udma_desc_new_tail);
	tx->tail = tx->new_tail;
	hw->ops->start_tx_transfer(hw, DESC_INDEX_TO_DESC_DMA(tx->to_be_use,tx));


	return 0;

}


int udma_send_packet(unsigned char port, struct sk_buff *skb)
{
	struct udma_device *umdev = NULL;
	struct udma_hw *hw = NULL;
	struct udma_ring *tx = NULL;
	unsigned long flags;
	dma_addr_t dma;
	int ret = UDMA_OK;
	struct udma_buffer *buffer_info = NULL;

	u32 len = skb_headlen(skb);
	u32 i = 0;
	/* skb check*/

	if (unlikely((!skb) || (skb->len <= 0) || (skb->data_len !=0 ))) {
          dev_kfree_skb_any(skb);
          return 0;
	}

	if (skb->len < ETH_ZLEN) {
        WARN_ON(skb_pad(skb,ETH_ZLEN + ETH_FCS_LEN - skb->len));
		len = ETH_ZLEN + ETH_FCS_LEN;
    } else {
		WARN_ON(skb_pad(skb,ETH_FCS_LEN));
		len += ETH_FCS_LEN;
    }

	umdev = udma_devs[port];
	hw = umdev->hw;
	tx = &umdev->tx_ring;

	if (is_ring_full(tx)) {
		udma_ring_clean_tx_irq(umdev,UDMA_TASKLET_THRESHOLD);
		return UDMA_FULL;
	}
	dma = dma_map_single(&hw->pdev->dev, skb->data, len, DMA_TO_DEVICE);
	if (dma_mapping_error(&hw->pdev->dev, dma)) {
		dev_err(&umdev->hw->pdev->dev, "DMA map failed\n");
		return UDMA_ERR;
	}
	/* Fill in the buffer info to the descriptor */
	spin_lock_irqsave(&tx->lock, flags);
	i = NEXT_TX(tx->new_tail);
	tx->new_tail = i;

	buffer_info = &tx->buffer_info[i];

	buffer_info->skb = skb;
	buffer_info->dma = dma;
	buffer_info->length = len;

	hw->ops->update_tx_desc(hw, &tx->desc[i],dma,len);

	if (udma_tx_is_stopped(umdev)) {
		udma_start_tx_transfer(umdev);
	}
	spin_unlock_irqrestore(&tx->lock, flags);
	return ret;
}

/**  udma_give_free_buffer - Give a free buffer to UDMA driver, the buffer will be used for packet receive
 * @port - udma port number, could be 0 or 1
 * @buffer_desc - parameter to describe a coming buffer
 *
 * Note the upper layer should calls this functioin periodly to give enough free buffers to UDMA driver
 *
 * It's requried to allocate a single buffer for a packet.
 * Thus the coming free buffer size should be large enough, i.e larger than 1536 bytes. Otherwise, the buffer would be refused.
 *
 * return 0, UDMA driver took care of the buffer
 * return negative for failure
*/
/* Allocate rx buffers */
int udma_give_free_buffer(unsigned char port, int cleaned_count)
{
	struct udma_device *umdev = NULL;
	struct udma_hw *hw = NULL;
	struct udma_ring *rx = NULL;
	unsigned long flags;
	struct udma_buffer *buffer_info;
	dma_addr_t dma;
	int ret = UDMA_OK;
	struct sk_buff *skb = NULL;
	int i = 0;
	int sum = 0;
	static int j = 0;
	int cnt = cleaned_count;


	PRINT_FUNC
	umdev = udma_devs[port];
	hw = umdev->hw;
	rx = &umdev->rx_ring;



	while(cleaned_count-- ) {
	skb = netdev_alloc_skb_ip_align(udma_netdevs[port], UDMA_MIN_RX_BUFFER_SIZE);
	if(unlikely(!skb)){
		printk("allocate new skb failed in %s function\n", __func__);
		return UDMA_ERR;
	}
	sum += tmp_tv2.tv_usec - tmp_tv1.tv_usec;
map_skb:
	dma = dma_map_single(&hw->pdev->dev, skb->data, UDMA_MIN_RX_BUFFER_SIZE, DMA_FROM_DEVICE);
	if (dma_mapping_error(&hw->pdev->dev, dma)) {
		dev_err(&hw->pdev->dev, "DMA map failed\n");
		dev_kfree_skb_any(skb);
		return UDMA_ERR;
	}

	/* Fill in the buffer info to the descriptor */
	spin_lock_irqsave(&rx->lock, flags);
	i = NEXT_RX(rx->new_tail);
	rx->new_tail = i;

	buffer_info = &rx->buffer_info[i];
	buffer_info->skb = skb;
	buffer_info->dma = dma;

	hw->ops->update_rx_desc(hw, &rx->desc[i], dma, UDMA_MIN_RX_BUFFER_SIZE);
	spin_unlock_irqrestore(&rx->lock, flags);

	}

	return ret;
}


/**  udma_register_handler - register the Tx/Rx callback
 *
 * @port - udma port number, could be 0 or 1
 * @tx_handle - Tx callback. Once a buffer is send out, UDMA driver fills in UDMA_BUFFRE_TX_DONE to buffer descriptor information, and calls tx_handle
 * @rx_handle - Rx callback. Once a buffer is received, UDMA driver fills in UDMA_BUFFRE_RX_DONE and udma_packt_pos_t to buffer descriptor information,
			   and calls rx_handle
 *
 * Note that the context in which the tx_callback/rx_callback is called is that of a softIRQ
 *
 * A prototype of a udma_handle is:
 * 		int udma_handle(int port, udma_buffer_desc_t *buffer_desc);
 *
 * At UDMA driver exit,
 * 	it will mark the buffer descriptor as "UDMA_BUFFER_NULL", and send all of the unfinished Tx buffers to upper layer for clean by Tx_handle callback
 * 	it will mark the buffer descriptor as "UDMA_BUFFER_NULL", and send all of the received Rx free buffers to upper layer for clean by Rx_handle callback

 * return UDMA_OK, success
 * return UDMA_ERR, failure
*/
int udma_register_handler(unsigned char port, struct net_device *dev, rx_callback_t rx_handle)
{
	struct udma_device *umdev = NULL;
	struct udma_hw *hw = NULL;
	unsigned long flags = 0;
	int err = 0;

	UDMA_WARN_ON_RETURN((port >= UDMA_PORT_NUM_TOTAL),UDMA_INVALID_PARAM);
	UDMA_WARN_ON_RETURN(!rx_handle,UDMA_INVALID_PARAM);

	umdev = udma_devs[port];
	if (umdev == NULL) {
		udma_err("UDMA Driver is not installed\n");
		return UDMA_UNINITIALIZED;
	}
	hw = umdev->hw;
	if (umdev->rx_complete) {
		udma_err("err: UDMA device %d is already being used by others \n", port);
		return UDMA_BUSY;
	}

	udma_netdevs[port] = dev;

	umdev->rx_complete = rx_handle;
	hw->ops->hw_init(hw);

	udma_setup_all_ring_resources(umdev);


	err = request_irq(hw->pdev->irq, udma_intr, IRQF_SHARED,hw->port?UDMA1_NAME:UDMA0_NAME,umdev);
	if (err) {
		udma_err("Error to register UDMA interrupt for port %d, exit\n", hw->port);
		hw->ops->hw_exit(hw);

		umdev->rx_complete = NULL;
		return UDMA_ERR;
	}
	netif_napi_add(dev,&umdev->napi, udma_clean,64);
	napi_enable(&umdev->napi);
	udma_irq_enable(umdev);

	umdev->status = UDMA_DEV_IS_OPEN;


	/* Enable UDMA Rx */
	udma_give_free_buffer(umdev->hw->port,UDMA_RING_ENTRY_NUM-2);

	spin_lock_irqsave(&umdev->rx_ring.lock, flags);
	if (udma_rx_is_stopped(umdev))
                udma_start_rx_transfer(umdev);
	spin_unlock_irqrestore(&umdev->rx_ring.lock, flags);


	return UDMA_OK;
}

/**  udma_flush - Stop the UDMA and flush the pending requests in a UDMA port and recycling all of the buffers
 *
 * @port - udma port number, could be 0 or 1
 *
 * This function is expected to be called by upper layer when exit
 *
*/
void udma_flush(unsigned char port)
{
	struct udma_device *umdev = NULL;
	struct udma_hw *hw = NULL;
	int budget =  UDMA_RING_ENTRY_NUM;
#define  MAX_UDMA_STOP_DELAY      (200)
	int delay;

	if (WARN_ON(port >= UDMA_PORT_NUM_TOTAL)) {
		udma_err("%s invalid parameters \n",__FUNCTION__);
		return;
	}

	umdev = udma_devs[port];
	hw = umdev->hw;

	napi_disable(&umdev->napi);
	netif_napi_del(&umdev->napi);

	/* Stop Tx and clean */
	hw->ops->stop_tx_transfer(hw);
	delay = MAX_UDMA_STOP_DELAY;
	while (!hw->ops->tx_is_stopped(hw) && delay--) {
		mdelay(1);
	}

	hw->ops->clear_tx_irq(hw);


	/* Stop Rx and clean */
	hw->ops->stop_rx_transfer(hw);
	delay = MAX_UDMA_STOP_DELAY;
	while ((!hw->ops->rx_is_stopped(hw)) && (delay--)){
		mdelay(1);
	}
	hw->ops->clear_rx_irq(hw);

	udma_irq_disable(umdev);
	hw->ops->hw_exit(hw);

	del_timer_sync(&umdev->watchdog_timer);

	hrtimer_cancel(&umdev->itr_timer);



	udma_clean_all_rings(umdev, UDMA_TX);

	udma_clean_all_rings(umdev, UDMA_RX);

	free_irq(hw->pdev->irq, umdev);


	udma_free_all_ring_resources(umdev);

	umdev->rx_complete = NULL;
	umdev->status = UDMA_DEV_IS_CLOSED;

	return;
}

EXPORT_SYMBOL_GPL(udma_send_packet);
EXPORT_SYMBOL_GPL(udma_register_handler);
EXPORT_SYMBOL_GPL(udma_flush);


enum hrtimer_restart itr_timeout(struct hrtimer *timer)
{

	struct udma_device *umdev = container_of(timer, struct udma_device, itr_timer);
	hr_cnt++;
	udma_napi_schedule(umdev);
	return HRTIMER_NORESTART;
}

/****************************************************************************
*
* The UDMA SW stack setup/initialization routine
*
****************************************************************************/

/**
 * @udma_alloc_ring_dma - allocate DMA memory for a ring structure
 **/
static int  udma_alloc_ring_dma(struct udma_device *udma_dev,
					struct udma_ring *ring)
{
	struct pci_dev *pdev = udma_dev->hw->pdev;
	struct udma_desc* desc = NULL;
	int i = 0;
	int size = 0;
	BUG_ON(!udma_dev);
	BUG_ON(!ring);

#define UDMA_DESC_ALIGN  4
	ring->ring_dma_size = sizeof(struct udma_desc)*UDMA_RING_ENTRY_NUM;
	ring->ring_entries = UDMA_RING_ENTRY_NUM;

	size = sizeof(struct udma_buffer) * ring->ring_entries;
	ring->buffer_info = vzalloc(size);
	if (!ring->buffer_info) {
		printk(KERN_ERR "can not allocate memory for buffer_info\n");
		return -ENOMEM;
	}


	ring->desc = dma_alloc_coherent(&pdev->dev, ring->ring_dma_size, &ring->ring_dma,
					GFP_KERNEL);
	if (!ring->desc) {
		vfree(ring->buffer_info);
		return -ENOMEM;
	}
	desc = ring->desc;
	udma_dbg(" dma 0x%x\n",ring->ring_dma);

	/* Link the descripors one by one */
	while (i < UDMA_RING_ENTRY_NUM) {
		desc[i].next_desc = DESC_INDEX_TO_DESC_DMA((i+1)%UDMA_RING_ENTRY_NUM, ring);
		udma_dev->hw->ops->clear_desc(&desc[i]);
		i++;
	}
	return 0;
}

static int  udma_free_ring_dma(struct udma_device *udma_dev, struct udma_ring *ring)
{
	struct pci_dev *pdev = udma_dev->hw->pdev;
	if (ring->buffer_info)
		vfree(ring->buffer_info);
	if (ring->ring_dma)
		dma_free_coherent(&pdev->dev, ring->ring_dma_size, ring->desc, ring->ring_dma);
	ring->buffer_info = NULL;
	ring->desc = NULL;
	ring->ring_dma	=	0;
	ring->ring_dma_size	=	0;

	return 0 ;
}

static void  udma_init_tx_ring_descs(struct udma_device *umdev,
					struct udma_ring *tx)
{
	int i = 0;
	struct udma_desc* desc 			= NULL;
	/* Link the descripors one by one */
	while (i < UDMA_RING_ENTRY_NUM) {
		desc = &tx->desc[i];
		/* Pre config the Rx desc, all of the Rx buffer size are fixed */
		umdev->hw->ops->init_tx_desc(umdev->hw, desc);
		umdev->hw->ops->enable_desc_tx_irq(desc);

		i++;
	}
}

static void  udma_init_rx_ring_descs(struct udma_device *umdev,
					struct udma_ring *rx)
{
	int i = 0;
	struct udma_desc* desc 			= NULL;
	struct udma_buffer* buffer_info = NULL;
	/* Link the descripors one by one */
	while (i < UDMA_RING_ENTRY_NUM) {
		desc = &rx->desc[i];
		buffer_info = &rx->buffer_info[i];
		/* Pre config the Rx desc, all of the Rx buffer size are fixed */
		umdev->hw->ops->init_rx_desc(umdev->hw, desc);
		umdev->hw->ops->enable_desc_rx_irq(desc);
		/* Pre config the Rx buffer info */
		buffer_info->length = UDMA_MIN_RX_BUFFER_SIZE;
		i++;
	}
}
/**
 * udma_alloc_rings - Allocate memory for all rings
 * @udma_dev: board private structure to initialize
 */
static int  udma_setup_all_ring_resources(struct udma_device *udma_dev)
{

	memset(&udma_dev->rx_ring, 0, sizeof(struct udma_ring));
	memset(&udma_dev->tx_ring, 0, sizeof(struct udma_ring));
	spin_lock_init(&udma_dev->rx_ring.lock);
	spin_lock_init(&udma_dev->tx_ring.lock);
	udma_alloc_ring_dma(udma_dev, &udma_dev->rx_ring);
	udma_alloc_ring_dma(udma_dev, &udma_dev->tx_ring);

	udma_init_rx_ring_descs(udma_dev, &udma_dev->rx_ring);
	udma_init_tx_ring_descs(udma_dev, &udma_dev->tx_ring);

	return 0;
}
static int  udma_free_all_ring_resources(struct udma_device *umdev)
{


	udma_clean_all_rings(umdev, UDMA_TX);

	udma_clean_all_rings(umdev, UDMA_RX);

	udma_free_ring_dma(umdev, &umdev->rx_ring);
	udma_free_ring_dma(umdev, &umdev->tx_ring);

	memset(&umdev->rx_ring, 0, sizeof(struct udma_ring));
	memset(&umdev->tx_ring, 0, sizeof(struct udma_ring));
	return 0;
}
static int udma_dev_init(struct udma_device *umdev)
{



	init_timer(&umdev->watchdog_timer);
	umdev->watchdog_timer.function = udma_watchdog;
	umdev->watchdog_timer.data = (unsigned long) umdev;
	hrtimer_init(&umdev->itr_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	umdev->itr_timer.function = itr_timeout;

	umdev->tx_itr = lowest_latency;
	umdev->rx_itr = lowest_latency;

	umdev->itr = INTR_FREQ_IN_LOWEST_LATENCY;
	umdev->itr_ns = 1000000000/umdev->itr;
	umdev->total_rx_bytes = 0;
	umdev->total_rx_packets= 0;
	umdev->total_tx_bytes = 0;
	umdev->total_tx_packets= 0;

	umdev->status 	=	UDMA_DEV_IS_CLOSED;
	PRINT_FUNC
	return 0;
}

/*
 * Driver exit routine
 *
*/
static void udma_dev_exit(struct udma_device *umdev)
{
	BUG_ON(!umdev);


	del_timer_sync(&umdev->watchdog_timer);

	hrtimer_cancel(&umdev->itr_timer);

	return ;
}


struct udma_hw *udma_alloc_hw(size_t size)
{
	struct udma_device *umdev = NULL;
	struct udma_hw *hw = NULL;

	hw = kzalloc(sizeof(struct udma_device) + size, GFP_KERNEL);
	if (NULL == hw) {
			udma_err("Cannot allocate memory\n");
			return ERR_PTR(-ENOMEM);
	}

	umdev = (struct udma_device *)udma_hw_priv(hw);
	umdev->hw = hw;

	return hw;
}
EXPORT_SYMBOL_GPL(udma_alloc_hw);

#ifdef DEBUG
/* For debug purpose */
static int udma_open(struct inode *inode, struct file *filp)
{
        return 0;
}
static int udma_close(struct inode *inode, struct file *filp)
{
        return 0;
}
void udma_dump_time_coming()
{
	int i = 0;
	int sum = 0;
	int sum2 = 0;
	while(i++<1000) {
		if (i == 0)
			printk("\n");

		printk(" %4d/%4d",timearray[i],timearray2[i]);
		if (i<900) {
			sum+=timearray[i];
			sum2+=timearray2[i];
		}
		if (i%8 == 0)
			printk("\n");
	}
			printk("\n");
			printk("		avarage %4d/%4d    \n", sum/900,sum2/900);
			//printk("		avarage %8d    \n", sum/900);
}
void udma_clear_time_coming()
{
	int i = 0;
	while(i++<1000) {
		timearray[i] = 0;
		timearray2[i] = 0;
	}
}
static long udma_unlocked_ioctl(struct file *filp, unsigned int arg, unsigned long cmd)
{
	int i = 0;
	struct udma_device *umdev = NULL;
	struct udma_hw *hw = NULL;
	struct udma_ring *ring = NULL;
	PRINT_FUNC

	if (arg - UDMA_PORT_MAGIC_BASE >= UDMA_PORT_NUM_TOTAL)
		return -EIO;
	umdev = udma_devs[arg - UDMA_PORT_MAGIC_BASE];
	hw = umdev->hw;
	switch (cmd) {
    case UDMA_DUMP_TX_CURR_RING:
	udma_info("Tx send %8d, Tx INTR %8d, link size %8d, freq %8d , hrtimer %8d, hrsetting %8d, itr %8d\n, bytes %8d, packets %8d",send_cnt,tx_cnt,link_cnt,tx_cnt?(u8)(send_cnt/tx_cnt):0,hr_cnt, 1000*1000/umdev->itr_ns*1000, umdev->itr, umdev->total_tx_bytes, umdev->total_tx_packets);
	send_cnt = 0;
	tx_cnt = 0;
	link_cnt = 0;
	send_period = 0;
	send_period_old = 0;
	send_coming = 0;
	tv.tv_sec = 0;
	tv.tv_usec = 0;
	hr_cnt = 0;
	udma_info("Rx pkt %8d, Rx INTR %8d, freq %8d, rx napi complet %8d , max link size%8d, bytes %8d, packets %8d\n",give_cnt,rx_cnt,rx_cnt?(give_cnt/rx_cnt):0,udma_napi_done, rx_link_cnt, umdev->total_rx_bytes, umdev->total_rx_packets);
	rx_link_cnt = 0;
	give_cnt = 0;
	rx_cnt = 0;
	udma_napi_done = 0;
	rx_drop_cnt = 0;
	//udma_regs_dump(hw,UDMA_RX);
//		ring = &umdev->tx_ring;
//		udma_dump_a_ring(umdev, ring, UDMA_TX);
	udma_dump_time_coming();
	udma_clear_time_coming();
         break;

    case UDMA_DUMP_RX_CURR_RING:
		ring = &umdev->rx_ring;
		udma_dump_a_ring(umdev, ring, UDMA_RX);

         break;

    case UDMA_DUMP_TX_RING0:
		ring = &umdev->tx_ring;
		udma_dump_a_ring(umdev, ring, UDMA_TX);
        break;
	case UDMA_DUMP_TX_RING1:

		 break;

	case UDMA_DUMP_RX_RING0:
		ring = &umdev->rx_ring;
		udma_dump_a_ring(umdev, ring, UDMA_RX);
         break;
    case UDMA_DUMP_RX_RING1:
		break;
	case UDMA_DUMP_CURR_TX_REGS:
		udma_regs_dump(hw,UDMA_TX);
		break;
	case UDMA_DUMP_CURR_RX_REGS:
		udma_regs_dump(hw,UDMA_RX);
		break;
	case UDMA_DUMP_DBG_SPACE:
		//udma_dump_frame();
		break;
	case UDMA_DUMP_SND_PKT:
		//udma_dump_frame();
		break;
    default:
         printk(KERN_ERR "UDMA driver receive Wrong IOCTL command = 0x%x \n",cmd);
         return -EFAULT;
    }

	return 0;
}
static struct file_operations udma_fops = {
	.owner   		= THIS_MODULE,
	.unlocked_ioctl 	= udma_unlocked_ioctl,
	.open 			= udma_open,
	.release 		= udma_close,
};
#endif

int __devinit udma_setup_sw(void *dev)
{
	struct udma_device *umdev = (struct udma_device *)dev;
#ifdef DEBUG
	struct proc_dir_entry	*proc;
#endif
	int err;

	if (WARN_ON(NULL == umdev)) return -EINVAL;

	err = udma_dev_init(umdev);
	if (0 != err) {
		udma_err("UDMA SW layer setup failure\n");
		return -ENODEV;
	}
	udma_devs[umdev->hw->port] = umdev;


#ifdef DEBUG

	if (!umdev->hw->port) {
		proc = create_proc_entry( UDMA_PROC_FS, S_IRUSR|S_IWUSR | S_IRGRP |S_IWGRP |S_IROTH |S_IWOTH, NULL);
		if ( NULL != proc )
			proc->proc_fops = &udma_fops;
		else {
			printk("ERROR - DEVICE_NAME initialization- Can not create proc entry\n");
			return -EIO;
		}
	}
#endif
	return 0;
}

/**
 * udma_unregister - Called by hw layer as removal routine
 **/
void __devexit udma_free_sw(void *dev)
{
	struct udma_device *umdev = (struct udma_device *)dev;

	if (WARN_ON(umdev == NULL)) return ;

	udma_dev_exit(umdev);

	udma_devs[umdev->hw->port]  = NULL;

#ifdef DEBUG
	if (!umdev->hw->port)
		remove_proc_entry(UDMA_PROC_FS, NULL);
#endif
	return ;
}
EXPORT_SYMBOL_GPL(udma_setup_sw);
EXPORT_SYMBOL_GPL(udma_free_sw);


/* udma_main.c */
