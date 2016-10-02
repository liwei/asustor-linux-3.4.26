/*******************************************************************************

  Intel UDMA driver
  Copyright(c) 1999 - 2006 Intel Corporation.

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

  Contact Information:
  Linux NICS <linux.nics@intel.com>
  udma-devel Mailing List <udma-devel@lists.sourceforge.net>
  Intel Corporation, 5200 N.E. Elam Young Parkway, Hillsboro, OR 97124-6497

*******************************************************************************/


#ifndef _UDMA_MAIN_H_
#define _UDMA_MAIN_H_
#include <linux/netdevice.h>
#include <linux/interrupt.h>
#include "udma_hw.h"
#include <linux/spinlock.h>
#include <linux/hrtimer.h>
#include <linux/udma_skb.h>

/*
 * Tx : APPCPU 	--> 		L2SW
 * Rx:  APPCPU 	<-- 		L2SW
*/

/* wrapper around a pointer to a buffer,
 * so a DMA handle can be stored along with the buffer
 */
struct udma_buffer {
	struct sk_buff *skb;
	dma_addr_t dma;
	u16 length;
};

#define UDMA_RING_ENTRY_NUM   256
#define UDMA_RING_VALID_MAX_NUM    (UDMA_RING_ENTRY_NUM-2)
struct udma_ring {
	struct spinlock lock ____cacheline_aligned_in_smp;
	struct udma_buffer *buffer_info;
	struct udma_desc *desc;									/* Virtual address of desc. */
	dma_addr_t ring_dma ____cacheline_aligned_in_smp; 		/* phys address of desc. */
	/*
	Empty:		to_be_use == tail
	Full:			to_be_clean == tail+1
	A free descriptor means the descriptor with no packet buffer attached
	*/
	volatile u16  to_be_clean; 						/* The first free desc. brings nothing */
	volatile u16  to_be_use;							/* The desc is just started maybe being processed by UDMA */
	volatile u16  tail ;								/* The next avaliable desc. to be cleaned */
	volatile u16  new_tail;							/* The first free desc. brings nothing */

	int ring_dma_size;
	int ring_entries;

};

/* Left 1 slot empty, 256 is the maximum value */
#define UDMA_TASKLET_THRESHOLD				(UDMA_RING_ENTRY_NUM-2)

#define	 UDMA_DEV_IS_OPEN		1;
#define	 UDMA_DEV_IS_CLOSED		0;

typedef int (*udma_clean_op_t)(struct udma_device *, struct udma_ring *, const int , bool );

enum latency_range {
	lowest_latency = 0,
	low_latency = 1,
	bulk_latency = 2,
	latency_invalid = 255
};


/* UDMA device structure, includes HW, ring, buffer */
struct udma_device {
		/* structs defined in udma_hw.h */
		struct udma_hw				*hw;

		struct tasklet_struct clean_tasklet;
		struct tasklet_struct clean_tx_tasklet;
		struct napi_struct napi;
		struct timer_list watchdog_timer;

		udma_clean_op_t clean_unused;
		struct hrtimer itr_timer;
		u32 itr_ns;
		/* Interrupt Throttle Rate */
		u32 itr;
		u16 tx_itr;
		u16 rx_itr;

		unsigned int total_tx_bytes;
		unsigned int total_tx_packets;
		unsigned int total_rx_bytes;
		unsigned int total_rx_packets;
		/* Callbacks */
		rx_callback_t	rx_complete;
		int status;	/* 0: close, 1: open */
		struct udma_ring tx_ring ____cacheline_aligned_in_smp;
		struct udma_ring rx_ring ____cacheline_aligned_in_smp;
};
#define NEXT_TX(N)              (((N) + 1) & (UDMA_RING_ENTRY_NUM - 1))
#define NEXT_RX(N)              (((N) + 1) & (UDMA_RING_ENTRY_NUM - 1))

static inline bool is_ring_full(struct udma_ring *ring)
{
	return ((ring->new_tail + 2)%UDMA_RING_ENTRY_NUM == ring->to_be_clean);
}

struct udma_device *udma_devs[UDMA_PORT_NUM_TOTAL] = {NULL, NULL};
struct net_device *udma_netdevs[UDMA_PORT_NUM_TOTAL] = {NULL, NULL};


/* UDMA Ring related operation definitons */

#define TO_DESC_INDEX(i) 					((i)%UDMA_RING_ENTRY_NUM)

#define DESC_INDEX_DEC(i) 					TO_DESC_INDEX((i) + UDMA_RING_ENTRY_NUM -1)

#define DESC_INDEX_INC(i) 					TO_DESC_INDEX((i)+1)



/* Desc DMA address to desc index */
#define DESC_IS_IN_THE_RING(d,ring) 		((d > ring->dma) && (d <= INDEX_TO_DESC_DMA((ring->count - 1),ring)))

#define DESC_DMA_TO_DESC_INDEX(d,ring) 		(((u32)(d - ring->ring_dma))/sizeof(struct udma_desc))


/* Desc index to Desc DMA address */
#define DESC_INDEX_TO_DESC_DMA(i,ring)		((ring)->ring_dma + (i) * sizeof(struct udma_desc))

/* Ring index to the ring desc virtual address */
#define INDEX_TO_DESC(i,ring) 				(&(((struct udma_desc *)((ring)->desc))[i]))

/* Ring index to ring pointer */
#define	INDEX_TO_RING(umdev, r, direction) 	((UDMA_TX == direction)?(umdev)->tx_ring[r]:(umdev)->rx_ring[r])


#ifdef DEBUG

#define UDMA_PORT_MAGIC_BASE	100
#define UDMA_PORT0	100
#define UDMA_PORT1	101


#define UDMA_PROC_FS "udma_dbg"

#define UDMA_DUMP_TX_CURR_RING		121
#define UDMA_DUMP_RX_CURR_RING		122

#define UDMA_DUMP_TX_RING0			123
#define UDMA_DUMP_TX_RING1			124
#define UDMA_DUMP_RX_RING0			125
#define UDMA_DUMP_RX_RING1			126
#define UDMA_DUMP_CURR_TX_REGS		127
#define UDMA_DUMP_CURR_RX_REGS		128
#define UDMA_DUMP_DBG_SPACE			129
#define UDMA_DUMP_SND_PKT			130
#define UDMA_CLR_TX_RING			131
#define UDMA_CLR_RX_RING			132


#endif



#endif /* _UDMA_MAIN_H_ */
