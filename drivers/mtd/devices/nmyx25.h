/*
  * MTD driver for  Numorix N25Q256/M25Px32 (and similar) serial flash chips
  * in Intel CE Media Processor 5300 and later
  *
  *
  *  GPL LICENSE SUMMARY
  *
  *  Copyright(c) 2011 Intel Corporation. All rights reserved.
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
  *    Santa Clara, CA	97052
  *
  */

#ifndef LINUX_DRIVERS_MTD_NMYX25H
#define LINUX_DRIVERS_MTD_NMYX25H
/* Flash opcodes. */
/* Legacy SPI mode command set */
#define	NMYX25_CMD_WREN				0x06		/* Write enable */
#define NMYX25_CMD_WRDI				0x04		/* Write disable */
#define	NMYX25_CMD_RDSR				0x05		/* Read status register */
#define	NMYX25_CMD_WRSR				0x01		/* Write status register 1 byte */
#define	NMYX25_CMD_NORM_READ		0x03		/* Read data bytes (low frequency) */
#define	NMYX25_CMD_FAST_READ		0x0b		/* Read data bytes (high frequency) */
#define	NMYX25_CMD_PP				0x02		/* Page program (up to 256 bytes) */
#define	NMYX25_CMD_BE_4K			0x20		/* Erase 4KiB block, SubSector Erase*/
#define	NMYX25_CMD_CHIP_ERASE		0xc7		/* Erase whole flash chip */
#define	NMYX25_CMD_SE				0xd8		/* Sector erase (usually 64KiB) */
#define	NMYX25_CMD_RDID				0x9f		/* Read JEDEC ID */
/* 4 Byte addressing command set */
#define NMYX25_CMD_READ4BYTE		0x13		/* Read Data Bytes using 4 byte address */
#define NMYX25_CMD_FAST_READ4BYTE 	0x0c		/* Fast Read Data Bytes using 4 byte address */

/* Used for SST flashes only. */
#define	OPCODE_BP		0x02	/* Byte program */
#define	OPCODE_WRDI		0x04	/* Write disable */
#define	OPCODE_AAI_WP		0xad	/* Auto address increment word program */

/* QUAD SPI mode command set, for N25Q256 */
#define NMYX25_CMD_MIORDID			0xAF		/*Multiple I/O read identification */
#define NMYX25_CMD_RDSFDF			0x5A		/* Read Serial Flash Discovery Parameter */
#define NMYX25_CMD_QCFR 			0x0b		/* Quad command fast read */
#define NMYX25_CMD_QCFR4BYTE 		0x0c		/* Quad command fast read using 4 byte address */

#define NMYX25_CMD_QCPP				0x02		/* Quad command page program */
#define NMYX25_CMD_RFSR				0x70		/* Read Flag status register */
#define NMYX25_CMD_CLFSR			0x50		/* Clear Flag status register */

#define NMYX25_CMD_RDNVCR			0xb5		/* Read NV Configuration Register */
#define NMYX25_CMD_WRNVCR			0xb1		/* Write NV Configuration Register */
#define NMYX25_CMD_RDVCR			0x85		/* Read volatile config register */
#define NMYX25_CMD_WRVCR			0x81		/* Write volatile config register */
#define NMYX25_CMD_RDVECR			0x65		/* Read volatile enhanced config register */
#define NMYX25_CMD_WRVECR			0x61		/* Write volatile enhanced config register */

#define NMYX25_CMD_EN4BYTEADDR		0xb7		/* Enter 4-byte address mode */
#define NMYX25_CMD_EX4BYTEADDR		0xe9		/* Exit 4-byte address mode */
#define	NMYX25_CMD_WREAR			0xc5		/* Write extended address register */
#define	NMYX25_CMD_RDEAR			0xc8		/* Read extended address register */
#define NMYX25_CMD_RSTEN			0x66		/* Reset Enable */
#define NMYX25_CMD_RST				0x99		/* Reset memory */

/* Non Volatile config register bits */
#define NVCR_4BYTE_ADDR_MODE	0			/* 4 byte address mode */
#define NVCR_3BYTE_ADDR_MODE	1			/* 3 byte address mode */

#define NVCR_DUAL_IO_CMD_DA		(1<<2)

#define NVCR_QUAD_IO_CMD_DA		(1<<3)		/* Diable Quad IO mode */

#define NVCR_PAD_HOLD_FUNC_EN	(1<<4)		/* enable pad hold/reset functionality*/
#define NVCR_OUT_DRV_LEN		(0b111<<6)	/* Default driver length */
#define NVCR_XIP_DISABLE		(0b111<<9)	/* Diable XIP */
#define NVCR_DUMMY_CLK_CYCLE	(0b1111<<12)/* Default dummy clock cycle */
#define NVCR_DEFALT_VALUE		(0xffff)	/* Default delivery value */
/* Volatile config register bits */
#define VCR_WRAP				(0b11)		/* Continuous reading */
#define VCR_XIP_DISABLE			(1<<3)		/* Disable XIP 	 */
#define VCR_DUMMY_CLK_CYCLE		(0b1111<<4)	/* Default dummy clock cycle */
#define VCR_DEFALT_VALUE		(0xfb)		/* Default delivery value */


/* Volatile Enhanced config register bits */
#define VECR_OUT_DRV_LEN		(0b111)		/* default output driver length */
#define VECR_ACCERATOR_PIN_EN	(~(1<<3))	/* Accellerator enabled */
#define VECR_ACCERATOR_PIN_DA	(1<<3)		/* Accellerator disabled */
#define VECR_PAD_HOLD_FUNC_EN	(1<<4)		/* enable pad hold/reset functionality*/
#define VECR_DUAL_IO_CMD_EN		(~(1<<6))
#define VECR_DUAL_IO_CMD_DA		(1<<6)		/* Disable dual io command */
#define VECR_QUAD_IO_CMD_DA		(1<<7) 		/* Disable quad IO command */

#define VECR_DEFALT_VALUE		(0xdf)		/* Default delivery value */

/* Legacy Status Register bits. */
#define	SR_WIP			1					/* Write in progress */
#define	SR_WEL			2					/* Write enable latch */
/* meaning of other SR_* bits may differ between vendors */
#define	SR_BP0			4					/* Block protect 0 */
#define	SR_BP1			8					/* Block protect 1 */
#define	SR_BP2			0x10				/* Block protect 2 */
#define	SR_SRWD			0x80				/* SR write protect */

/* Define max times to check status register before we give up. */
#define	MAX_READY_WAIT_JIFFIES	(40 * HZ)	/* 40s max chip erase */
#define	MAX_CMD_SIZE		5

#ifdef CONFIG_N25Q256_USE_FAST_READ
#define NMYX25_CMD_READ 	NMYX25_CMD_FAST_READ
#define FAST_READ_DUMMY_BYTE 1
#else
#define NMYX25_CMD_READ 	NMYX25_CMD_NORM_READ
#define FAST_READ_DUMMY_BYTE 0
#endif

//#define SPI_CE_DEBUG 1
#ifdef  SPI_CE_DEBUG
#define spi_dbg(fmt, args...) do \
                             { \
                               printk(KERN_INFO "\n	%s " fmt "\n","nmyx25",##args); \
                              } while(0)

#define spi_dbg_func	spi_dbg(" func %s, line %d\n",__FUNCTION__,__LINE__)

#else
#define spi_dbg(fmt,args...) do {} while(0)
#define spi_dbg_func	do {} while(0)

#endif

#define NMYX25_LOCK(flash) do {\
	if (flash->flags & NMYX25_SUPPORT_HW_MUTEX)\
		hw_mutex_lock_interruptible(HW_MUTEX_NOR_SPI);\
	else\
		mutex_lock(&flash->lock);\
} while(0)

#define NMYX25_UNLOCK(flash) do {\
		if (flash->flags & NMYX25_SUPPORT_HW_MUTEX)\
			hw_mutex_unlock(HW_MUTEX_NOR_SPI);\
		else\
			mutex_unlock(&flash->lock);\
} while(0)

/****************************************************************************/
/* serial flash command set */
struct sflash_cmdset {
	unsigned char r;  /* read */
	unsigned char fr; /* fast read */
	unsigned char rdid; /* read device id */
	unsigned char wren; /* write enable */
	unsigned char wrdi; /* write disable */
	unsigned char se; /* sector erase */
	unsigned char ce; /* chip erase */
	unsigned char pp; /* page program */
	unsigned char rdsr; /* read status reg */
	unsigned char wrsr; /* write status reg */
};


struct nmyx25 {
	struct spi_device	*spi;
	struct mutex		lock;
	struct mtd_info		mtd;
	unsigned			partitioned:1;
	unsigned			read_window:1;	/* 0: read data from CSR window; 1: read data from memory window */
	u16					page_size;
	u16					addr_width;
	u8					*command;
	u8					flags;
/* Two or more processors access the controller, HW Mutex is necessary to avoid confliction*/
#define NMYX25_SUPPORT_HW_MUTEX  (1<<0)
	struct flash_cs_info info;
	struct sflash_cmdset *cmdset;
};

typedef int (*nmyx25_op_type)(struct mtd_info *, loff_t , size_t , size_t *, const u_char *);

#endif
