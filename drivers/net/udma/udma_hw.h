/*******************************************************************************

  Intel UDMA driver
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

  Contact Information:
  Linux NICS <linux.nics@intel.com>
  e1000-devel Mailing List <e1000-devel@lists.sourceforge.net>
  Intel Corporation, 5200 N.E. Elam Young Parkway, Hillsboro, OR 97124-6497

*******************************************************************************/
/* udma_hw.h
 *  UDMA hardware layer
 */
#ifndef UDMA_HW_H
#define UDMA_HW_H

//
#define DEBUG 1
#ifdef DEBUG
//#define udma_warning(fmt, args...) do \
	{\
		printk(KERN_DEBUG "[%18s:%1d]" fmt "\n", __FUNCTION__,__LINE__,##args); \
	} while (0)
//#define WARN_FUNC printk(KERN_DEBUG " func[%s] line [%d] \n", __FUNCTION__, __LINE__);
#define udma_warning(fmt, args...) do { } while (0)
#define WARN_FUNC do { } while (0);
#define udma_dbg(format, arg...) do { } while (0)
//#define udma_dbg(fmt, args...) do \
	{\
		printk(KERN_INFO "[%s] " fmt "\n", __func__,##args); \
	} while (0)
//#define print_func printk(kern_debug " func[%s] line [%d] \n", __function__, __line__);
#define PRINT_FUNC  do { } while (0);
#define udma_info(format, arg...) do \
	{\
		printk(KERN_INFO "%s " format "\n","udma - ",##arg); \
	} while (0)

#else
#define udma_warning(fmt, args...) do { } while (0)
#define udma_dbg(format, arg...) do { } while (0)

#define PRINT_FUNC  do { } while (0);
//#define udma_info(format, arg...) do { } while (0)
#define udma_info(format, arg...) do \
	{\
		printk(KERN_INFO "	   %s " format "\n","UDMA - ",##arg); \
	} while (0)

#endif
#define udma_err(format, arg...) do \
	{\
		printk(KERN_DEBUG "	   %s " format "\n","UDMA error - ",##arg); \
	} while (0)


/* Total size for registers */
#define UDMA_PORT_REGS_SPACE_SIZE 				0x88

/* Generic UDMA Context Registers */

#define UDMA_CURR_DESC 							0x008
#define UDMA_NEXT_DESC							0x00C
#define UDMA_SRCDMA_START						0x010
#define UDMA_DSTDMA_START						0x014
#define UDMA_SRCDMA_SIZE						0x018
#define UDMA_FLAGS_MODE							0x01c
#define UDMA_OTHER_MODE							0x020


/* UDMA FLAGS MODE Register Attributes */
#define UDMA_DMA_IS_ACTIVE 						(1<<31)
#define UDMA_SRC_INT_EN							(1<<30)
#define UDMA_DST_INT_EN							(1<<29)
#define UDMA_TERM_EN							(1<<28)
#define UDMA_SRC_BIG_ENDIAN						(1<<25)
#define UDMA_DST_BIG_ENDIAN						(1<<24)
#define UDMA_PACKET_POSITION_STARTING			(1<<21)
#define UDMA_PACKET_POSITION_ENDING				(2<<21)
#define UDMA_PACKET_POSITION_STARTING_ENDING	(3<<21)

#define UDMA_XDMA_GAP_0_CLKS					(0<<16)
#define UDMA_XDMA_GAP_16_CLKS					(1<<16)
#define UDMA_XDMA_GAP_64_CLKS					(2<<16)
#define UDMA_XDMA_GAP_256_CLKS					(3<<16)
#define UDMA_XDMA_GAP_1024_CLKS					(4<<16)
#define UDMA_XDMA_GAP_2048_CLKS					(5<<16)
#define UDMA_XDMA_GAP_4096_CLKS					(6<<16)
#define UDMA_XDMA_GAP_8192_CLKS					(7<<16)

#define UDMA_XBURST_SZ_4_BYTES					(0<<12)
#define UDMA_XBURST_SZ_8_BYTES					(1<<12)
#define UDMA_XBURST_SZ_16_BYTES					(2<<12)
#define UDMA_XBURST_SZ_32_BYTES					(3<<12)
#define UDMA_XBURST_SZ_64_BYTES					(4<<12)

#define UDMA_READ_EN							(1<<7)	// Equal to TX from ATOM side point of view
#define UDMA_SRC_ADDR_MODE_LINEAR				(0<<5)
#define UDMA_SRC_ADDR_MODE_FIXED				(2<<5)
#define UDMA_SRC_ADDR_MODE_FIXED_CONTINUE		(3<<5)
#define UDMA_SRC_LINK_LIST_EN					(1<<4)

#define UDMA_WRITE_EN							(1<<3) // Equal to Rx from ATOM side point of view
#define UDMA_DST_ADDR_MODE_LINEAR				(0<<1)
#define UDMA_DST_ADDR_MODE_FIXED				(2<<1)
#define UDMA_DST_ADDR_MODE_FIXED_CONTINUE		(3<<1)
#define UDMA_DST_LINK_LIST_EN					(1<<0)



/* UDMA OTHER MODE Register Attributes */
#define UDMA_LL_OWNERSHIP_TAGS_EN				(1<<3)
#define UDMA_LL_PRE_FETCH_DISABLE				(1<<2)
#define UDMA_STOP								(1<<0)


/* Starting a linked-list transfer */
#define UDMA_FLAG_MODE_START_LL					(UDMA_SRC_ADDR_MODE_LINEAR | UDMA_DST_ADDR_MODE_LINEAR | \
												UDMA_SRC_LINK_LIST_EN | UDMA_DST_LINK_LIST_EN)

#define UDMA_FLAG_MODE_TX_LL 					(UDMA_XDMA_GAP_64_CLKS|UDMA_XBURST_SZ_64_BYTES | UDMA_READ_EN | \
												UDMA_SRC_ADDR_MODE_LINEAR | UDMA_SRC_LINK_LIST_EN |\
												UDMA_DST_ADDR_MODE_FIXED |UDMA_SRC_INT_EN)

#define UDMA_FLAG_MODE_TX_LL_ENDING 			(UDMA_XDMA_GAP_64_CLKS|UDMA_TERM_EN | UDMA_XBURST_SZ_64_BYTES | UDMA_READ_EN | \
												UDMA_SRC_ADDR_MODE_LINEAR | UDMA_SRC_LINK_LIST_EN \
												| UDMA_DST_ADDR_MODE_FIXED)

#define UDMA_FLAG_MODE_RX_LL 					(UDMA_XBURST_SZ_64_BYTES | UDMA_WRITE_EN | \
												UDMA_DST_ADDR_MODE_LINEAR | UDMA_DST_LINK_LIST_EN |\
												UDMA_SRC_ADDR_MODE_FIXED |UDMA_DST_INT_EN)

#define UDMA_FLAG_MODE_RX_LL_ENDING 			(UDMA_TERM_EN | UDMA_XBURST_SZ_64_BYTES | UDMA_WRITE_EN | \
												UDMA_DST_ADDR_MODE_LINEAR | UDMA_DST_LINK_LIST_EN |\
												UDMA_SRC_ADDR_MODE_FIXED)

#define UDMA_OTHER_MODE_DEFAULT_SETTING  		(UDMA_LL_OWNERSHIP_TAGS_EN | UDMA_LL_PRE_FETCH_DISABLE)


/* UDMA Interrupt Request Registers */
#define UDMA_INTR_MASK							0x80
	#define UDMA_STOP_INT5							(1<<17)
	#define UDMA_STOP_INT4							(1<<16)
	#define UDMA_STOP_INT3							(1<<15)
	#define UDMA_STOP_INT2							(1<<14)
	#define UDMA_STOP_INT1							(1<<13)
	#define UDMA_STOP_INT0							(1<<12)
	#define UDMA_DST_INT5_EN						(1<<11)
	#define UDMA_DST_INT4_EN						(1<<10)
	#define UDMA_DST_INT3_EN						(1<<9)
	#define UDMA_DST_INT2_EN						(1<<8)
	#define UDMA_DST_INT1_EN						(1<<7)
	#define UDMA_DST_INT0_EN						(1<<6)
	#define UDMA_SRC_INT5_EN 						(1<<5)
	#define UDMA_SRC_INT4_EN 						(1<<4)
	#define UDMA_SRC_INT3_EN 						(1<<3)
	#define UDMA_SRC_INT2_EN 						(1<<2)
	#define UDMA_SRC_INT1_EN 						(1<<1)
	#define UDMA_SRC_INT0_EN 						(1<<0)

#define UDMA_INTR_STATUS						0x84
	#define UDMA_STOP_INT5_ACT						(1<<17)
	#define UDMA_STOP_INT4_ACT						(1<<16)
	#define UDMA_STOP_INT3_ACT						(1<<15)
	#define UDMA_STOP_INT2_ACT						(1<<14)
	#define UDMA_STOP_INT1_ACT						(1<<13)
	#define UDMA_STOP_INT0_ACT						(1<<12)
	#define UDMA_DST_INT5_ACT						(1<<11)
	#define UDMA_DST_INT4_ACT						(1<<10)
	#define UDMA_DST_INT3_ACT						(1<<9)
	#define UDMA_DST_INT2_ACT						(1<<8)
	#define UDMA_DST_INT1_ACT						(1<<7)
	#define UDMA_DST_INT0_ACT						(1<<6)

	#define UDMA_DST_INT3_ACT_BIT					(9)
	#define UDMA_DST_INT2_ACT_BIT					(8)
	#define UDMA_DST_INT1_ACT_BIT					(7)
	#define UDMA_DST_INT0_ACT_BIT					(6)


	#define UDMA_SRC_INT5_ACT						(1<<5)
	#define UDMA_SRC_INT4_ACT 						(1<<4)
	#define UDMA_SRC_INT3_ACT 						(1<<3)
	#define UDMA_SRC_INT2_ACT 						(1<<2)
	#define UDMA_SRC_INT1_ACT						(1<<1)
	#define UDMA_SRC_INT0_ACT 						(1<<0)

	#define UDMA_SRC_INT3_ACT_BIT					(3)
	#define UDMA_SRC_INT2_ACT_BIT					(2)
	#define UDMA_SRC_INT1_ACT_BIT					(1)
	#define UDMA_SRC_INT0_ACT_BIT					(0)

#define UDMA_HW_VALID_TX_INTR_STATE(port) 			((port)?UDMA_SRC_INT3_ACT:UDMA_SRC_INT1_ACT)
#define UDMA_HW_VALID_RX_INTR_STATE(port) 			((port)?UDMA_DST_INT2_ACT:UDMA_DST_INT0_ACT)
#define UDMA_HW_VALID_INTR_STATE(port) 				(UDMA_HW_VALID_TX_INTR_STATE(port)|UDMA_HW_VALID_RX_INTR_STATE(port))

#define UDMA_RX_REG_BASE 							0x00
#define UDMA_TX_REG_BASE 							0x40


/* UDMA Port Address Map */
/*
 * port 0:
 * 		context 0: L2SW -> DDR
 * 		context 1: DDR  -> L2SW
 * port 1:
 * 		context 2: L2SW -> DDR
 * 		context 3: DDR  -> L2SW
 */
/* UDMA port 0 */
#define UDMA_CONTEXT0_DMA_ADDR						0xFFFA0000
#define UDMA_CONTEXT1_DMA_ADDR						0xFFFB0000

/* UDMA port 1 */
#define UDMA_CONTEXT2_DMA_ADDR						0xFFFC0000
#define UDMA_CONTEXT3_DMA_ADDR						0xFFFD0000

/* NPCPU port */
#define UDMA_CONTEXT4_DMA_ADDR						0xFFFE0000
#define UDMA_CONTEXT5_DMA_ADDR						0xFFFF0000

/* Debug port */
#define UDMA_CONTEXT_DUMP_DMA_ADDR					0xFFF00000



struct __port_regset {
	u32 rx;
	u32 tx;
};

struct __intr_regset {
	u32 int_mask;
	u32 int_stat;
};
struct __intr_status_bitset {
	u32 rx;
	u32 tx;
};

struct __cntx_regset {
	unsigned short cdesc;
	unsigned short ndesc;
	unsigned short sdma;
	unsigned short ddma;
	unsigned short size;
	unsigned short flag;
	unsigned short other;
};
struct udma_cntx_regset {
	struct __cntx_regset *rx;
	struct __cntx_regset *tx;
	struct __intr_regset *intr;
};



#define UDMA_DEVICE_ID									0x0947

#define UDMA0_NAME "udma0"
#define UDMA1_NAME "udma1"

#define UDMA_RX  										(0)
#define UDMA_TX  										(1)
#define UDMA_MAX_RX_BUFFER_SIZE 						BIT(24)
#define UDMA_MAX_XMIT_SIZE 								(1526)  /* The L2SW only support 1526 bytes as MTU */
//#define UDMA_DESC_MAX_XMIT_SIZE 					(1514)	/* header + data  */
#define UDMA_MIN_XMIT_SIZE 								(64)
#define UDMA_RX_MIN_BUFFER_SIZE							(2*1024)




/* UDMA Descriptor based oprations MACRO */
#define DESC_INITIAL_FLAG								0x00	/* Initial state, sw owns the descriptor */
#define DESC_HANDLING_FLAG								0x80	/* Descriptor is being processed by DMA */
#define DESC_DONE_FLAG									0xc0	/* Descriptor processed state */

#define SET_END_DESC_FLAG(d) 							((d)->flags |= UDMA_TERM_EN) 	/* Set the termination bit */
#define CLR_END_DESC_FLAG(d) 							((d)->flags &= (~UDMA_TERM_EN)) /* Clear the termination bit */

#define DESC_IS_INITIAL(d) 								((d)->union_field.fields.ownership == DESC_INITIAL_FLAG)
#define DESC_IS_IN_PROGRESS(d) 							((d)->union_field.fields.ownership == DESC_HANDLING_FLAG)
#define DESC_IS_DONE(d)									((d)->union_field.fields.ownership == DESC_DONE_FLAG)
#define DESC_IS_NULL(d)									(((d)->src == 0) || ((d)->dst == 0))

#define DESC_DATA_BUFFER_LEN(d)							(((d)->union_field.size<<8)>>8)

/* UDMA Descriptor */
struct udma_desc {
	__le32 next_desc;									/* Address of the next descriptor */
	union {
		__le32 size;									/* Data buffer length */
		struct {
		u8 length[3];									/* buffer lengh */
		u8 ownership;									/* owner ship field */
		} fields;
	} union_field;
	__le32 src;											/* Source Addr */
	__le32 dst;											/* Target Addr */
	__le32 flags;										/* Flag mode */
}__attribute__((aligned(4)));


/* Describe a UDMA port */
struct udma_hw {
	u8							port;					/* The port number of this UDMA HW */
	volatile void __iomem 		*ioaddr;				/* Virtual address of reg base address */
	struct udma_hw_operations 	*ops;
	struct pci_dev				*pdev;
	unsigned long private[0] ____cacheline_aligned;
};

struct udma_hw_operations {
	bool (*tx_is_active)(struct udma_hw *);
	bool (*rx_is_active)(struct udma_hw *);

	u32 (*get_curr_rx_desc)(struct udma_hw *);
	u32 (*get_curr_tx_desc)(struct udma_hw *);

	u32 (*get_next_rx_desc)(struct udma_hw *);
	u32 (*get_next_tx_desc)(struct udma_hw *);

	int (*start_rx_transfer)(struct udma_hw *, u32 );
	int (*start_tx_transfer)(struct udma_hw *, u32 );

	int  (*stop_rx_transfer)(struct udma_hw *);
	int  (*stop_tx_transfer)(struct udma_hw *);

	bool (*tx_is_stopped)(struct udma_hw *);
	bool (*rx_is_stopped)(struct udma_hw *);

	void (*init_rx_desc)(struct udma_hw *, struct udma_desc *);
	void (*init_tx_desc)(struct udma_hw *, struct udma_desc *);

	void (*update_rx_desc)(struct udma_hw *, struct udma_desc *, u32, u32);
	void (*update_tx_desc)(struct udma_hw *, struct udma_desc *, u32, u32);

	int (*clear_desc)(struct udma_desc *);

	int (*hw_init)(struct udma_hw *);
	int (*hw_exit)(struct udma_hw *);

	int (*enable_desc_rx_irq)(struct udma_desc *);
	int (*enable_desc_tx_irq)(struct udma_desc *);

	int (*disable_desc_rx_irq)(struct udma_desc *);
	int (*disable_desc_tx_irq)(struct udma_desc *);

	u32  (*get_irq_status)(struct udma_hw *);

	int (*disable_rx_irq)(struct udma_hw *);
	int (*enable_rx_irq)(struct udma_hw *);

	int (*disable_tx_irq)(struct udma_hw *);
	int (*enable_tx_irq)(struct udma_hw *);

	int (*clear_rx_irq)(struct udma_hw *);
	int (*clear_tx_irq)(struct udma_hw *);
};

static inline void udma_writel(struct udma_hw *hw, int reg, u32 val)
{
    __raw_writel(val, hw->ioaddr + reg);
}
static inline u32 udma_readl(struct udma_hw *hw,int reg)
{
    return __raw_readl(hw->ioaddr + reg);
}

static inline void *udma_hw_priv(struct udma_hw *hw)
{
	return (void *)hw->private;
}

static inline uint8_t udma_read_and_test_bits(struct udma_hw *hw,int reg, u32 val)
{
   return ((udma_readl(hw, reg) & (val)) > 0);
}

static inline void udma_read_and_set_bits(struct udma_hw *hw,int reg, u32 val)
{
    udma_writel(hw, reg, udma_readl(hw, reg) | (val));
}
static inline void udma_read_and_clr_bits(struct udma_hw *hw,int reg, u32 val)
{
    udma_writel(hw, reg, udma_readl(hw, reg) & (~(val)));
}

static inline void udma_set_bits_nr(struct udma_hw *hw,int reg, u32 nr)
{
	__set_bit(nr,hw->ioaddr + reg);
}
static inline void udma_clr_bits_nr(struct udma_hw *hw,int reg, u32 nr)
{
	__clear_bit(nr,hw->ioaddr + reg);
}




void udma_desc_dump(struct udma_desc *desc);

void udma_regs_dump(struct udma_hw *hw, bool direction);




#endif /* UDMA_HW_H */
