/*
  * MTD driver for  Numonyx N25Q256/M25Px32 (and similar) serial flash chips
  * in Intel CE Media Processor 5300 and later
  *
  * Based on m25p80.c
  *
  *  GPL LICENSE SUMMARY
  *
  *  Copyright(c) 2011-2013 Intel Corporation. All rights reserved.
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
#include <linux/init.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/mutex.h>
#include <linux/math64.h>
#include <linux/slab.h>
#include <linux/mod_devicetable.h>
#include <linux/sched.h>

#include <linux/mtd/cfi.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/partitions.h>

#include <linux/spi/spi.h>
#include <linux/spi/flash.h>
#include <linux/pci.h>
#ifdef CONFIG_HW_MUTEXES
#include <linux/hw_mutex.h>
#endif
#include "nmyx25.h"
#include <linux/delay.h>

#define JEDEC_MFR(_jedec_id)	((_jedec_id) >> 16)


/* Legacy protocal command table */
static const struct sflash_cmdset legy_cmdset = {
	.r 	= NMYX25_CMD_READ,
	.fr	= NMYX25_CMD_FAST_READ,
	.rdid	= NMYX25_CMD_RDID,
	.wren	= NMYX25_CMD_WREN,
	.wrdi	= NMYX25_CMD_WRDI,
	.se	= NMYX25_CMD_SE,
	.ce	= NMYX25_CMD_CHIP_ERASE,
	.pp	= NMYX25_CMD_PP,
	.rdsr	= NMYX25_CMD_RDSR,
	.wrsr	= NMYX25_CMD_WRSR,
};

/* Quad mode command set */
static const struct sflash_cmdset quad_cmdset = {
	.r 	= NMYX25_CMD_QCFR,
	.fr	= NMYX25_CMD_FAST_READ,
	.rdid	= NMYX25_CMD_MIORDID,
	.wren	= NMYX25_CMD_WREN,
	.wrdi	= NMYX25_CMD_WRDI,
	.se	= NMYX25_CMD_SE,
	.ce	= NMYX25_CMD_CHIP_ERASE,
	.pp	= NMYX25_CMD_QCPP,
	.rdsr	= NMYX25_CMD_RDSR,
	.wrsr	= NMYX25_CMD_WRSR,
};

static inline struct nmyx25 *mtd_to_nmyx25(struct mtd_info *mtd)
{
	return container_of(mtd, struct nmyx25, mtd);
}

/****************************************************************************/
/*
 * Internal helper functions
 */


/*
 * Read the status register, returning its value in the location
 * Return the status register value.
 * Returns negative if error occurred.
 */
static int read_sr(struct nmyx25 *flash)
{
	ssize_t retval;
	u8 code = flash->cmdset->rdsr;
	u8 val;

	retval = spi_write_then_read(flash->spi, &code, 1, &val, 1);

	if (retval < 0) {
		dev_err(&flash->spi->dev, "error %d reading SR\n",
				(int) retval);
		return retval;
	}

	return val;
}

static int read_flg_sr(struct nmyx25 *flash, u8 cmd)
{
	ssize_t retval;
	u8 code = cmd;
	u8 val;

	retval = spi_write_then_read(flash->spi, &code, 1, &val, 1);

	if (retval < 0) {
		dev_err(&flash->spi->dev, "error %d reading SR\n",
				(int) retval);
		return retval;
	}

	return val;
}

/*
 * Write status register 1 byte
 * Returns negative if error occurred.
 */
static int write_sr(struct nmyx25 *flash, u8 val)
{
	flash->command[0] = flash->cmdset->wrsr;
	flash->command[1] = val;

	return spi_write(flash->spi, flash->command, 2);
}
static int wait_till_write_ready(struct nmyx25 *flash)
{
	unsigned long deadline;
	int sr;

	deadline = jiffies + MAX_READY_WAIT_JIFFIES;

	do {
		sr = read_sr(flash);
		if (sr < 0) {
			break;
		}else if (sr & SR_WEL){
			return 0;
		}
		cond_resched();

	} while (!time_after_eq(jiffies, deadline));
	printk(KERN_ERR "time out..\n");
	return 1;
}

/*
 * Set write enable latch with Write Enable command.
 * Returns negative if error occurred.
 */
static inline void write_enable(struct nmyx25 *flash)
{
	u8	code = flash->cmdset->wren;

	spi_write_then_read(flash->spi, &code, 1, NULL, 0);
	wait_till_write_ready(flash);
}

/*
 * Send write disble instruction to the chip.
 */
static inline int write_disable(struct nmyx25 *flash)
{
	u8	code = flash->cmdset->wrdi;

	return spi_write_then_read(flash->spi, &code, 1, NULL, 0);
}
/* Write a command to device
 * returns negative if error happens
 */
static inline int write_cmd(struct nmyx25 *flash,u8 cmd)
{
	spi_dbg( "%s: %s set to working mode \n",
			dev_name(&flash->spi->dev), __func__);

	flash->command[0] = cmd;
	/* Send write enable, then erase commands. */
	write_enable(flash);
	return spi_write(flash->spi, flash->command, 1);
}
/* write a register value which always including two byte
 * Returns negative if erros happen
 */
static inline int write_reg(struct nmyx25 *flash,u8 reg, u8 *val, u8 num)
{
	spi_dbg( "%s: %s set to working mode reg 0x%x, val 0x%x, len%d\n",
			dev_name(&flash->spi->dev), __func__,reg,
			*val, num);

	flash->command[0] = reg;
	memcpy(&flash->command[1], val, num);


	/* Send write enable, then erase commands. */
	write_enable(flash);
	return spi_write(flash->spi, flash->command, 1+num);
}

/*
 * Service routine to read status register until ready, or timeout occurs.
 * Returns non-zero if error.
 */
static int wait_till_ready(struct nmyx25 *flash)
{
	unsigned long deadline;
	int sr;

	deadline = jiffies + MAX_READY_WAIT_JIFFIES;

	do {
		sr = read_sr(flash);
		if (sr < 0) {
			break;
		} else if (!(sr & SR_WIP)) {
			return 0;
		}

		cond_resched();

	} while (!time_after_eq(jiffies, deadline));
	printk(KERN_ERR "time out..\n");
	return 1;
}

#ifdef SPI_CE_DEBUG
static void dump_reg(struct nmyx25 *flash)
{
	u8 val = 0;
	printk("\n");
	printk("Reg dump:\n");
	val = read_flg_sr(flash,NMYX25_CMD_RDSR);
	printk("	status :0x%x\n",val);
	val = 0;
	val = read_flg_sr(flash,NMYX25_CMD_RFSR);
	printk("	Flag status :0x%x\n",val);
	val = 0;
#if 0
	val = read_flg_sr(flash,NMYX25_CMD_RDNVCR);
	printk("	None volitile Reg status :0x%x\n",val);
	val = 0;
#endif
	val = read_flg_sr(flash,NMYX25_CMD_RDVCR);
	printk("	Volitile Reg status :0x%x\n",val);
	val = 0;

	val = read_flg_sr(flash,NMYX25_CMD_RDVECR);
	printk("	Volitile enhanced status :0x%x\n",val);
	val = 0;

}
#endif
/*
 * Erase the whole flash memory
 *
 * Returns 0 if successful, non-zero otherwise.
 */
static int erase_chip(struct nmyx25 *flash)
{
	pr_debug("%s: %s %lldKiB\n",
	      dev_name(&flash->spi->dev), __func__,
	      (long long)(flash->mtd.size >> 10));

	/* Wait until finished previous write command. */
	if (wait_till_ready(flash))
		return 1;

	/* Send write enable, then erase commands. */
	write_enable(flash);
//	wait_till_ready(flash);

	/* Set up command buffer. */
	flash->command[0] = flash->cmdset->ce;

	spi_write(flash->spi, flash->command, 1);

	return 0;
}

static void nmyx25_addr2cmd(struct nmyx25 *flash, unsigned int addr, u8 *cmd)
{
        /* opcode is in cmd[0] */
        cmd[1] = addr >> (flash->addr_width * 8 -  8);
        cmd[2] = addr >> (flash->addr_width * 8 - 16);
        cmd[3] = addr >> (flash->addr_width * 8 - 24);
        if (flash->addr_width == 4)
                cmd[4] = addr >> (flash->addr_width * 8 - 32);
}

static inline int nmyx25_cmdsz(struct nmyx25 *flash)
{
        return 1 + flash->addr_width;
}

/*
 * Erase one sector of flash memory at offset ``offset'' which is any
 * address within the sector which should be erased.
 *
 * Returns 0 if successful, non-zero otherwise.
 */
static int erase_sector(struct nmyx25 *flash, u32 offset)
{
	int ret = 0;
	pr_debug("%s: %s %dKiB at 0x%08x\n",
			dev_name(&flash->spi->dev), __func__,
			flash->mtd.erasesize / 1024, offset);

	/* Wait until finished previous write command. */
	if (wait_till_ready(flash))
		return 1;

	/* Send write enable, then erase commands. */
	write_enable(flash);

	/* Set up command buffer. */
	flash->command[0] = flash->cmdset->se;
	nmyx25_addr2cmd(flash, offset, flash->command);
	ret = spi_write(flash->spi, flash->command, nmyx25_cmdsz(flash));

	return 0;
}

/****************************************************************************/

/*
 * MTD implementation
 */

/*
 * Translation mtd address to physical address in two CS
 */
static inline void nmyx25_addr_translation(struct mtd_info *mtd, loff_t addr, size_t len,
			loff_t *cs0_addr, size_t *cs0_len, loff_t *cs1_addr, size_t *cs1_len)
{
	struct nmyx25 *flash = mtd_to_nmyx25(mtd);
	uint32_t cs0_size	= flash->info.cs0_size;


	if (addr >= cs0_size) {
		/* access CS1 */
		*cs0_addr = 0;
		*cs0_len	 = 0;
		*cs1_addr = addr - cs0_size;
		*cs1_len	 = len;
	}
	else if ((addr + len) > cs0_size) {
		/* access CS0 & CS1 */
		*cs0_addr	= addr;
		*cs0_len	= cs0_size - *cs0_addr;
		*cs1_addr	= 0;
		*cs1_len	= len - cs0_size;
	}
	else /* access CS0 */
	{
		*cs0_addr = addr;
		*cs0_len	 = len;
		*cs1_addr = 0;
		*cs1_len	 = 0;
	}
	spi_dbg("%s input addr 0x%x len 0x%x, translate to CS0: 0x%x with len 0x%x, CS1: 0x%x with len 0x%x\n",
		 __FUNCTION__,(uint32_t)addr, (uint32_t)len, (uint32_t)*cs0_addr, (uint32_t)*cs0_len, (uint32_t)*cs1_addr, (uint32_t)*cs1_len);
	return;
}

/*
 * Erase an address range on the flash chip.  The address range may extend
 * one or more erase sectors.  Return an error is there is a problem erasing.
 */
/*
 * Erase on only one chip
 */
static int nmyx25_erase(struct mtd_info *mtd, struct erase_info *instr)
{
	struct nmyx25 *flash = mtd_to_nmyx25(mtd);
	u32 addr,len;
	uint32_t rem;

	spi_dbg("%s: %s %s 0x%llx, len %lld\n",
	      dev_name(&flash->spi->dev), __func__, "at",
	      (long long)instr->addr, (long long)instr->len);

	/* sanity checks */
	if (instr->addr + instr->len > flash->info.cs0_size)
		return -EINVAL;
	div_u64_rem(instr->len, mtd->erasesize, &rem);
	if (rem)
		return -EINVAL;

	addr = instr->addr;
	len = instr->len;



	/* whole-chip erase? */
	if (len == flash->info.cs0_size) {
		if (erase_chip(flash)) {
			instr->state = MTD_ERASE_FAILED;
			return -EIO;
		}

	/* REVISIT in some cases we could speed up erasing large regions
	 * by using NMYX25_CMD_SE instead of NMYX25_CMD_BE_4K.  We may have set up
	 * to use "small sector erase", but that's not always optimal.
	 */
	/* "sector"-at-a-time erase */
	} else {
		while (len) {
			if (erase_sector(flash, addr)) {
				instr->state = MTD_ERASE_FAILED;
				return -EIO;
			}
			addr += mtd->erasesize;
			len -= mtd->erasesize;

		}
	}


	instr->state = MTD_ERASE_DONE;


	return 0;
}
/*
 * Read an address range from the flash chip.  The address range
 * may be any size provided it is within the physical boundaries.
 */
static int nmyx25_read_mem(struct mtd_info *mtd, loff_t from, size_t len,
		size_t *retlen, u_char *buf)
{
	struct nmyx25 *flash = mtd_to_nmyx25(mtd);
	struct spi_transfer t;
	struct spi_message m;

	spi_dbg("%s: %s %s 0x%08x, len %zd, to %08x\n",
			dev_name(&flash->spi->dev), __func__, "from",
			(u32)from, len, (u32)buf);

	/* sanity checks */
	if (!len)
		return 0;

	if (from + len > flash->mtd.size)
		return -EINVAL;
	spi_message_init(&m);
	memset(&t, 0, (sizeof t));

	/* Let controller know it's a memory access method*/
	m.is_dma_mapped	= 1;

	t.rx_buf = buf;

	t.rx_dma = (u32)from;
	t.len = len;
	spi_message_add_tail(&t, &m);

	/* Byte count starts at zero. */
	if (retlen)
		*retlen = 0;


	/* Wait till previous write/erase is done. */
	if (wait_till_ready(flash)) {
		/* REVISIT status return?? */
		return 1;
	}
	spi_dbg("{%s} m 0x%x, t 0x%x, to 0x%x, dma 0x%x, len 0x%x\n", __FUNCTION__,m,t,(u32)t.rx_buf, (u32)t.rx_dma, t.len);
	spi_sync(flash->spi, &m);

	*retlen = m.actual_length - FAST_READ_DUMMY_BYTE;



	m.is_dma_mapped	= 0;

	return 0;
}

static int nmyx25_read_csr(struct mtd_info *mtd, loff_t from, size_t len,
	size_t *retlen, u_char *buf)
{
	struct nmyx25 *flash = mtd_to_nmyx25(mtd);
	struct spi_transfer t[2];
	struct spi_message m;

	spi_dbg("%s: %s %s 0x%08x, len %zd\n",
			dev_name(&flash->spi->dev), __func__, "from",
			(u32)from, len);


	/* sanity checks */
	if (!len)
		return 0;

	if (from + len > flash->mtd.size)
		return -EINVAL;

	spi_message_init(&m);
	memset(t, 0, (sizeof t));

	/* NOTE:
	 * NMYX25_CMD_FAST_READ (if available) is faster.
	 * Should add 1 byte DUMMY_BYTE.
	 */
	t[0].tx_buf = flash->command;
	t[0].len = nmyx25_cmdsz(flash) + FAST_READ_DUMMY_BYTE;
	spi_message_add_tail(&t[0], &m);

	t[1].rx_buf = buf;
	t[1].len = len;
	spi_message_add_tail(&t[1], &m);

	/* Let controller know it's CSR access method */
	m.is_dma_mapped	= 0;

	/* Byte count starts at zero. */
	if (retlen)
		*retlen = 0;


	/* Wait till previous write/erase is done. */
	if (wait_till_ready(flash)) {
		/* REVISIT status return?? */
		return 1;
	}
	/* FIXME switch to NMYX25_CMD_FAST_READ.  It's required for higher
	 * clocks; and at this writing, every chip this driver handles
	 * supports that opcode.
	 */
	/* Set up the write data buffer. */
	flash->command[0] = flash->cmdset->r;
	nmyx25_addr2cmd(flash, from, flash->command);
	//printk("\nReading at chip No.%d from 0x%llx......\n",flash->spi->chip_select,from);
	spi_sync(flash->spi, &m);

	*retlen = m.actual_length - nmyx25_cmdsz(flash) - FAST_READ_DUMMY_BYTE;


	return 0;
}

/*
 * Write an address range to the flash chip.  Data must be written in
 * FLASH_PAGESIZE chunks.  The address range may be any size provided
 * it is within the physical boundaries.
 */
static int nmyx25_write(struct mtd_info *mtd, loff_t to, size_t len,
	size_t *retlen, const u_char *buf)
{
	struct nmyx25 *flash = mtd_to_nmyx25(mtd);
	u32 page_offset, page_size;
	struct spi_transfer t[2];
	struct spi_message m;

	spi_dbg("%s: %s %s 0x%08x, len %zd\n",
			dev_name(&flash->spi->dev), __func__, "to",
			(u32)to, len);

	if (retlen)
		*retlen = 0;

	/* sanity checks */
	if (!len)
		return(0);

	if (to + len > flash->info.cs0_size)
		return -EINVAL;

	spi_message_init(&m);
	memset(t, 0, (sizeof t));

	t[0].tx_buf = flash->command;
	t[0].rx_buf = NULL;
	t[0].len = nmyx25_cmdsz(flash);
	spi_message_add_tail(&t[0], &m);

	t[1].tx_buf = buf;
	t[1].rx_buf = NULL;
	spi_message_add_tail(&t[1], &m);



	/* Wait until finished previous write command. */
	if (wait_till_ready(flash)) {

		return 1;
	}

	write_enable(flash);

	/* Set up the opcode in the write buffer. */
	flash->command[0] = flash->cmdset->pp;
	nmyx25_addr2cmd(flash, to, flash->command);
	//printk("Programming chip No.%d at 0x%llx\n",flash->spi->chip_select,to);

	page_offset = to & (flash->page_size - 1);

	/* do all the bytes fit onto one page? */
	if (page_offset + len <= flash->page_size) {
		t[1].len = len;

		wait_till_ready(flash);


		spi_sync(flash->spi, &m);

		*retlen = m.actual_length - nmyx25_cmdsz(flash);
	} else {
		u32 i;

		/* the size of data remaining on the first page */
		page_size = flash->page_size - page_offset;

		t[1].len = page_size;

		wait_till_ready(flash);
		spi_sync(flash->spi, &m);

		*retlen = m.actual_length - nmyx25_cmdsz(flash);

		/* write everything in flash->page_size chunks */
		for (i = page_size; i < len; i += page_size) {
			page_size = len - i;
			if (page_size > flash->page_size)
				page_size = flash->page_size;

			/* write the next page to flash */
			flash->command[0] = flash->cmdset->pp;
			nmyx25_addr2cmd(flash, to + i, flash->command);
			//printk("Programming chip No.%d at 0x%llx\n",flash->spi->chip_select,to);

			t[0].tx_buf = flash->command;
			t[0].len = nmyx25_cmdsz(flash);
			t[1].tx_buf = buf + i;
			t[1].len = page_size;

			wait_till_ready(flash);
			write_enable(flash);

			//wait_till_ready(flash);

			spi_sync(flash->spi, &m);
			//printk("m.actual_length %d, i %x\n", m.actual_length, i);

			if (retlen)
				*retlen += m.actual_length - nmyx25_cmdsz(flash);
		}
	}


	return 0;
}


static int sst_write(struct mtd_info *mtd, loff_t to, size_t len,
		size_t *retlen, const u_char *buf)
{
	struct nmyx25 *flash = mtd_to_nmyx25(mtd);
	struct spi_transfer t[2];
	struct spi_message m;
	size_t actual;
	int cmd_sz, ret;

	spi_dbg("%s: %s %s 0x%08x, len %zd\n",
			dev_name(&flash->spi->dev), __func__, "to",
			(u32)to, len);

	*retlen = 0;

	/* sanity checks */
	if (!len)
		return 0;

	if (to + len > flash->mtd.size)
		return -EINVAL;

	spi_message_init(&m);
	memset(t, 0, (sizeof t));

	t[0].tx_buf = flash->command;
	t[0].len = nmyx25_cmdsz(flash);
	spi_message_add_tail(&t[0], &m);

	t[1].tx_buf = buf;
	spi_message_add_tail(&t[1], &m);

	/* Wait until finished previous write command. */
	ret = wait_till_ready(flash);
	if (ret)
		return ret;

	write_enable(flash);

	actual = to % 2;
	/* Start write from odd address. */
	if (actual) {
		flash->command[0] = OPCODE_BP;
		nmyx25_addr2cmd(flash, to, flash->command);

		/* write one byte. */
		t[1].len = 1;
		spi_sync(flash->spi, &m);
		ret = wait_till_ready(flash);
		if (ret)
			return ret;
		*retlen += m.actual_length - nmyx25_cmdsz(flash);
	}
	to += actual;

	flash->command[0] = OPCODE_AAI_WP;
	nmyx25_addr2cmd(flash, to, flash->command);

	/* Write out most of the data here. */
	cmd_sz = nmyx25_cmdsz(flash);
	for (; actual < len - 1; actual += 2) {
		t[0].len = cmd_sz;
		/* write two bytes. */
		t[1].len = 2;
		t[1].tx_buf = buf + actual;

		spi_sync(flash->spi, &m);
		ret = wait_till_ready(flash);
		if (ret)
			return ret;
		*retlen += m.actual_length - cmd_sz;
		cmd_sz = 1;
		to += 2;
	}
	write_disable(flash);
	ret = wait_till_ready(flash);
	if (ret)
		return ret;

	/* Write out trailing byte if it exists. */
	if (actual != len) {
		write_enable(flash);
		flash->command[0] = OPCODE_BP;
		nmyx25_addr2cmd(flash, to, flash->command);
		t[0].len = nmyx25_cmdsz(flash);
		t[1].len = 1;
		t[1].tx_buf = buf + actual;

		spi_sync(flash->spi, &m);
		ret = wait_till_ready(flash);
		if (ret)
			return ret;
		*retlen += m.actual_length - nmyx25_cmdsz(flash);
		write_disable(flash);
	}

	return 0;
}



/*
 * Translater mtd logical address to physical address in two flash chips
 */
static int nmyx25_mtd_ops(struct mtd_info *mtd, loff_t from, size_t len,
	size_t *retlen, const u_char *buf, nmyx25_op_type op)
{
	struct nmyx25 *flash = mtd_to_nmyx25(mtd);
	loff_t	from0 = 0, from1 = 0;
	size_t	len0 = 0, len1 = 0;
	size_t	tmplen = 0;
	u_char *target = (u_char *)buf;
	int ret = 0;

	/* sanity checks */
	if ((!len) || (from + len > flash->mtd.size))
		return -EINVAL;

	if (!retlen)
		return -EINVAL;
	else
		*retlen = 0;

	nmyx25_addr_translation(mtd, from, len, &from0, &len0, &from1, &len1);
	if (len0){
		flash->spi->chip_select = 0;

		ret = (*op)(mtd,from0,len0,&tmplen,target);
		*retlen += tmplen;
		target += tmplen;
		if (ret)
			return ret;
	}

	if (len1){

		flash->spi->chip_select = 1;
		ret = (*op)(mtd,from1,len1,&tmplen,target);
		*retlen += tmplen;
		if (ret)
			return ret;
	}

	return ret;
}
/*
 *  erase function to MTD interface
 * first, deliver erase request to two CS
 * then, execute erase operation on each CS
 */
static int nmyx25_mtd_erase(struct mtd_info *mtd, struct erase_info *instr)
{
	struct nmyx25 *flash = mtd_to_nmyx25(mtd);
	loff_t	from0 = 0, from1 = 0;
	size_t	len0 = 0, len1 = 0;
	int ret = 0;

	/* sanity checks */
	if ((!instr->len) || (instr->addr + instr->len > mtd->size))
		return -EINVAL;

	NMYX25_LOCK(flash);
	nmyx25_addr_translation(mtd, instr->addr, instr->len, &from0, &len0, &from1, &len1);

	if (len0){
		flash->spi->chip_select = 0;
		instr->addr = from0;
		instr->len = len0;
		ret = nmyx25_erase(mtd, instr);
		if (ret)
			goto err_out;
	}

	if (len1){
		flash->spi->chip_select = 1;
		instr->addr = from1;
		instr->len = len1;
		ret = nmyx25_erase(mtd, instr);
		if (ret)
			goto err_out;
	}

	/* Wait until finished previous write command. */
	if (wait_till_ready(flash))
			goto err_out;

	mtd_erase_callback(instr);
err_out:
	NMYX25_UNLOCK(flash);
	return ret;
}
static int nmyx25_mtd_write(struct mtd_info *mtd, loff_t to, size_t len,
	size_t *retlen, const u_char *buf)
{
	struct nmyx25 *flash = mtd_to_nmyx25(mtd);
	int ret = 0;
	NMYX25_LOCK(flash);
	ret = nmyx25_mtd_ops(mtd, to, len, retlen, buf, &nmyx25_write);


	/* Wait until finished previous write command. */
	if (wait_till_ready(flash)) {
		NMYX25_UNLOCK(flash);
		return 1;
	}

	if (spi_setup(flash->spi))
		dev_err(&flash->spi->dev, "controller working mode setup failure \n");

	NMYX25_UNLOCK(flash);
	return ret;
}

static int sst_mtd_write(struct mtd_info *mtd, loff_t to, size_t len,
	size_t *retlen, const u_char *buf)
{
	struct nmyx25 *flash = mtd_to_nmyx25(mtd);
	int ret = 0;
	NMYX25_LOCK(flash);
	ret = nmyx25_mtd_ops(mtd, to, len, retlen, buf, &sst_write);


	/* Wait until finished previous write command. */
	if (wait_till_ready(flash)) {
		NMYX25_UNLOCK(flash);
		return 1;
	}

	if (spi_setup(flash->spi))
		dev_err(&flash->spi->dev, "controller working mode setup failure \n");

	NMYX25_UNLOCK(flash);
	return ret;
}



static int nmyx25_mtd_read(struct mtd_info *mtd, loff_t from, size_t len,
	size_t *retlen, u_char *buf)
{
	struct nmyx25 *flash = mtd_to_nmyx25(mtd);
	int ret = 0;
	NMYX25_LOCK(flash);

	if (flash->read_window)
		ret =  nmyx25_read_mem(mtd ,from ,len ,retlen ,buf);
	else
		ret =  nmyx25_mtd_ops(mtd, from, len, retlen, buf, (nmyx25_op_type)(&nmyx25_read_csr));


	if (spi_setup(flash->spi))
		dev_err(&flash->spi->dev, "controller working mode setup failure ");

	NMYX25_UNLOCK(flash);

	return ret;
}
/****************************************************************************/

/*
 * SPI device driver setup and teardown
 */

struct flash_info {
	/* JEDEC id zero means "no ID" (most older chips); otherwise it has
	 * a high byte of zero plus three data bytes: the manufacturer id,
	 * then a two byte device id.
	 */
	u32		jedec_id;
	u16             ext_id;

	/* The size listed here is what works with NMYX25_CMD_SE, which isn't
	 * necessarily called a "sector" by the vendor.
	 */
	u32		sector_size;
	u16		n_sectors;

	u16		page_size;
	u16		addr_width;

	u16		flags;
#define	SECT_4K				(0x1<<1)		/* NMYX25_CMD_BE_4K works uniformly */
#define	N25Q_NO_ERASE		(0x1<<2)		/* No erase command needed */
#define	SPI_CAP_QUAD_IO		(0x1<<3)		/* Capable of Quad SPI mode */
#define	SPI_CAP_DUAL_IO		(0x1<<4)		/* Capable of Quad SPI mode */
#define READ_FROM_MEM_WIN	(0x1<<5)		/* Read data from memory window */

};

#define INFO(_jedec_id, _ext_id, _sector_size, _n_sectors, _page_size, _addr_width, _flags)	\
	((kernel_ulong_t)&(struct flash_info) {				\
		.jedec_id = (_jedec_id),				\
		.ext_id = (_ext_id),					\
		.sector_size = (_sector_size),				\
		.n_sectors = (_n_sectors),				\
		.page_size = (_page_size),					\
		.addr_width = (_addr_width),					\
		.flags = (_flags),					\
	})
/* NOTE: double check command sets and memory organization when you add
 * more flash chips.  This current list focusses on newer chips, which
 * have been converging on command sets which including JEDEC ID.
 */
static const struct spi_device_id nmyx25_ids[] = {
	/* Numorix 32MB device, work in 4-tyte addressing, Quad-IO mode */
	{ "n25q256",	INFO(0x20ba19, 0, 64*1024,  512, 256,	4, READ_FROM_MEM_WIN)},
	{ "m25px32", 	INFO(0x207116, 0, 64*1024,  64,  256,	3, SECT_4K)},
	{ "m25p32",  	INFO(0x202016, 0, 64*1024,  64,  256,   3, 0)},
	{ "m25p64",     INFO(0x202017, 0, 64*1024,  128, 256,   3, 0)},
	{ "w25q32bv",	INFO(0xef4016, 0, 64*1024,  64,  256,   3, 0)},
	{ "n25q64",     INFO(0x20ba17, 0, 64*1024,  128, 256,   3, 0)},
	{ "s25fl032a",  INFO(0x010215, 0, 64*1024,  64,  256,   3, 0)},
	{ "sst25vf032b",INFO(0xbf254a, 0, 64*1024,  64,  256,   3, SECT_4K)},
	{ },
};
MODULE_DEVICE_TABLE(spi, nmyx25_ids);

static const struct spi_device_id *__devinit jedec_probe(struct spi_device *spi)
{
	int			tmp;
	u8			code = NMYX25_CMD_RDID;
	u8			id[5];
	u32			jedec;
	u16                     ext_jedec;
	struct flash_info	*info;

	/* JEDEC also defines an optional "extended device information"
	 * string for after vendor-specific data, after the three bytes
	 * we use here.  Supporting some chips might require using it.
	 */

	tmp = spi_write_then_read(spi, &code, 1, id, 5);
	if (tmp < 0) {
		spi_dbg("%s: error %d reading JEDEC ID\n",
			dev_name(&spi->dev), tmp);
		return NULL;
	}
	jedec = id[0];
	jedec = jedec << 8;
	jedec |= id[1];
	jedec = jedec << 8;
	jedec |= id[2];

	/*
	 * Some chips (like Numonyx N25Q80) have JEDEC and non-JEDEC variants,
	 * which depend on technology process. Officially RDID command doesn't
	 * exist for non-JEDEC chips, but for compatibility they return ID 0.
	 */
	if (jedec == 0)
		return NULL;

	ext_jedec = id[3] << 8 | id[4];

	for (tmp = 0; tmp < ARRAY_SIZE(nmyx25_ids) - 1; tmp++) {
		info = (void *)nmyx25_ids[tmp].driver_data;
		if (info->jedec_id == jedec) {
			if (info->ext_id != 0 && info->ext_id != ext_jedec)
				continue;
			spi_dbg("func %s, sector size 0x%x, n_sectors 0x%x\n", __FUNCTION__,info->sector_size,info->n_sectors);
			return &nmyx25_ids[tmp];
		}
	}
	return NULL;
}

/*
 * Selecting and Enalbing a SPI protocal and addressing mode per devices' characters
 * Returns 0 if success
 */
static int  nmyx25_setup(struct nmyx25 *flash)
{
	int ret = 0;
	int val = 0;
	int tmp = 0;
	u8  code;
	spi_dbg("%s addr_width %d mode 0x%x\n", __FUNCTION__,flash->addr_width,flash->spi->mode);


	/* SPI protocol mode, quad or legacy */
	if (flash->spi->mode & SPI_MODE_QUAD_IO) {
		flash->cmdset	= (struct sflash_cmdset *)&quad_cmdset;
		code = NMYX25_CMD_RDVECR;
		wait_till_ready(flash);
		tmp = spi_write_then_read(flash->spi, &code, 1, (u8 *)&val, 1);
		if (tmp < 0) {
			return tmp;
		}
		if ((u8)val & VECR_QUAD_IO_CMD_DA) {
			val &= (~VECR_QUAD_IO_CMD_DA);
			wait_till_ready(flash);
			write_reg(flash, NMYX25_CMD_WRVECR, (u8 *)&val, 1);
		}
	}
	else
		flash->cmdset = (struct sflash_cmdset *)&legy_cmdset;


	/* addressing mode, 3 bytes or 4 bytes */
	if (flash->addr_width == 4) {
		spi_dbg("Enter 4 byte addressing mode \n");
		if (flash->info.cs0_size) {
			flash->spi->chip_select = 0;
			write_cmd(flash, NMYX25_CMD_EN4BYTEADDR);
			if (!(read_flg_sr(flash, NMYX25_CMD_RFSR)&0x01)) {
				printk(KERN_ERR "err: chip %d can not enter %d byte addressing mode, aborting... \n",flash->spi->chip_select,flash->addr_width);
				ret = -1;
			}
		}
		if (flash->info.cs1_size) {
			flash->spi->chip_select = 1;
			write_cmd(flash, NMYX25_CMD_EN4BYTEADDR);
			if (!(read_flg_sr(flash, NMYX25_CMD_RFSR)&0x01)) {
				printk(KERN_ERR "err: can not enter %d byte addressing mode, aborting... \n",flash->addr_width);
				ret = -1;
			}
		}
		flash->cmdset->r	= NMYX25_CMD_READ4BYTE;
	}
#ifdef SPI_CE_DEBUG
	dump_reg(flash);
#endif
	return ret;
}
/*
 * board specific setup should have ensured the SPI clock used here
 * matches what the READ command supports, at least until this driver
 * understands FAST_READ (for clocks over 25 MHz).
 */
static int __devinit nmyx25_probe(struct spi_device *spi)
{
	struct flash_platform_data	*data = NULL;
	struct nmyx25			*flash = NULL;
	struct flash_info	*info[2] = { NULL, NULL};
	struct pci_dev *tmp_dev = NULL;
	unsigned int		i = 0;
	struct mtd_part_parser_data	ppdata;

	info[0] = NULL;
	info[1] = NULL;
	data = spi->dev.platform_data;

	flash = kzalloc(sizeof *flash, GFP_KERNEL);
	if (!flash)
		return -ENOMEM;
	flash->command = kmalloc(MAX_CMD_SIZE + FAST_READ_DUMMY_BYTE, GFP_KERNEL);
	if (!flash->command) {
		kfree(flash);
		return -ENOMEM;
	}
#ifdef CONFIG_HW_MUTEXES
	/*
	 * If there's HW Mutex controller exist, then we'll need to use HW Mutex to make sure exclusive controller access from different processors
	 */
	tmp_dev = pci_get_device(0x8086, HW_MUTEX_DEV_ID,NULL);
	if (tmp_dev)
	{

		flash->flags |= NMYX25_SUPPORT_HW_MUTEX;
		pci_dev_put(tmp_dev);
	}
	spi_dbg("check whether there's HW Mutex needed, %s\n",(flash->flags & NMYX25_SUPPORT_HW_MUTEX)?"yes":"no");
#endif
	dev_set_drvdata(&spi->dev, flash);
	flash->spi = spi;
	mutex_init(&flash->lock);

	NMYX25_LOCK(flash);
	spi_dbg_func;
	/*
	 * Make sure CS0/CS1 being the same kind of flash device
	 * Here we did auto detection of the flash devices
	 */
	/* Workaround: do not enable CS1, need be updated later */
	for (i=0; i<1; i++){
		struct spi_device_id *jid;
		spi->chip_select	= i;

		jid = (struct spi_device_id *)jedec_probe(spi);
		if (!jid) {
			dev_dbg(&spi->dev, "non device in cs %d\n",i);
			info[i]= NULL;
		}
		else {
			info[i] = (void *)jid->driver_data;
			dev_info(&spi->dev, "Found device %s with size %dMbytes in chip select %d\n",
				 jid->name,(info[i]->sector_size*info[i]->n_sectors)>>20,i);

		}

	}

	/* No device in CS0, exit
	 * Make sure there's device in CS0
	 */
	if (!info[0]){
		printk(KERN_ERR "err: no device found in chip select 0\n");
		kfree(flash);
		NMYX25_UNLOCK(flash);

		return -ENODEV;
	}
	else if (info[1]&&(info[0]->jedec_id != info[1]->jedec_id)) {
		dev_err(&spi->dev, "err: two differenct kind of chips with ID cs0 0x%x, cs1 0x%x\n",
			 info[0]->jedec_id, info[1]->jedec_id);
		kfree(flash);
		NMYX25_UNLOCK(flash);

		return -ENODEV;
	}


	spi_dbg("sector 0x%x, n_sectors %x\n",info[0]->sector_size,info[0]->n_sectors);

	/* Fill in info structure */
	flash->cmdset	= (struct sflash_cmdset *)&legy_cmdset;	/* Default working mode */
	flash->info.cs0_size = info[0]->sector_size * info[0]->n_sectors;

	if (!info[1])
		flash->info.cs1_size = 0;
	else
		flash->info.cs1_size = info[1]->sector_size * info[1]->n_sectors;

	flash->page_size = info[0]->page_size;
	flash->addr_width = info[0]->addr_width;


	if (info[0]->flags & READ_FROM_MEM_WIN)
		flash->read_window	= 1;
	else
		flash->read_window	= 0;


	/* Fill in controller data */
	if (info[0]->flags & SPI_CAP_QUAD_IO)
		spi->mode = SPI_MODE_QUAD_IO;
	spi->bits_per_word = info[0]->addr_width<<3; /* Tell controller devices using 3 byte addressing mode */
	spi->controller_data	= &flash->info;

	spi_dbg("cs0 0x%x, cs1 0x%x\n",flash->info.cs0_size,flash->info.cs1_size);


	/* now set up flash device and controller's working mode per devices' info */

	if (nmyx25_setup(flash))
		dev_err(&spi->dev, "device working mode setup failure id 0x%x\n", info[0]->jedec_id);
	if (spi_setup(spi))
		dev_err(&spi->dev, "controller working mode setup failure id 0x%x\n", info[0]->jedec_id);

	/*
	 * Atmel, SST and Intel/Numonyx serial flash tend to power
	 * up with the software protection bits set
	 */

	if (JEDEC_MFR(info[0]->jedec_id) == CFI_MFR_ATMEL ||
	    JEDEC_MFR(info[0]->jedec_id) == CFI_MFR_INTEL ||
	    JEDEC_MFR(info[0]->jedec_id) == CFI_MFR_SST) {
		write_enable(flash);
		write_sr(flash, 0);
	}
	NMYX25_UNLOCK(flash);
	if (data && data->name)
		flash->mtd.name = data->name;
	else
		flash->mtd.name = "nmyx25";

	flash->mtd.type = MTD_NORFLASH;
	flash->mtd.flags = MTD_CAP_NORFLASH;
	flash->mtd.writesize = info[0]->page_size;

	flash->mtd.size = flash->info.cs0_size + flash->info.cs1_size;

	flash->mtd._erase = nmyx25_mtd_erase;
	flash->mtd._read = nmyx25_mtd_read;
	/* sst flash chips use AAI word program */
	if (JEDEC_MFR(info[0]->jedec_id) == CFI_MFR_SST)
		flash->mtd._write = sst_mtd_write;
	else
		flash->mtd._write = nmyx25_mtd_write;

	/* prefer "small sector" erase if possible */
	if (info[0]->flags & SECT_4K) {
		flash->cmdset->se= NMYX25_CMD_BE_4K;
		flash->mtd.erasesize = 4096;
	} else {
		flash->cmdset->se = NMYX25_CMD_SE;
		flash->mtd.erasesize = info[0]->sector_size;
	}

	if (info[0]->flags & N25Q_NO_ERASE)
		flash->mtd.flags |= MTD_NO_ERASE;

	ppdata.of_node = spi->dev.of_node;
	flash->mtd.dev.parent = &spi->dev;


	//dev_info(&spi->dev, "(%lld Mbytes, pagesize %dBytes, erasesize 0x%dKBytes)\n",
	dev_info(&spi->dev, "(%lld Mbytes, pagesize %dBytes, erasesize 0x%dKBytes)\n",
			(long long)flash->mtd.size >> 20, flash->mtd.writesize,flash->mtd.erasesize/1024);

	pr_debug("mtd .name = %s, .size = 0x%llx (%lldMiB) "
			".erasesize = 0x%.8x (%uKiB) .numeraseregions = %d\n",
		flash->mtd.name,
		(long long)flash->mtd.size, (long long)(flash->mtd.size >> 20),
		flash->mtd.erasesize, flash->mtd.erasesize / 1024,
		flash->mtd.numeraseregions);

	if (flash->mtd.numeraseregions)
		for (i = 0; i < flash->mtd.numeraseregions; i++)
			pr_debug("mtd.eraseregions[%d] = { .offset = 0x%llx, "
				".erasesize = 0x%.8x (%uKiB), "
				".numblocks = %d }\n",
				i, (long long)flash->mtd.eraseregions[i].offset,
				flash->mtd.eraseregions[i].erasesize,
				flash->mtd.eraseregions[i].erasesize / 1024,
				flash->mtd.eraseregions[i].numblocks);


	/* partitions should match sector boundaries; and it may be good to
	 * use readonly partitions for writeprotected sectors (BP2..BP0).
	 */
	return mtd_device_parse_register(&flash->mtd, NULL, &ppdata,
			data ? data->parts : NULL,
			data ? data->nr_parts : 0);
}


static int __devexit nmyx25_remove(struct spi_device *spi)
{
	struct nmyx25	*flash = dev_get_drvdata(&spi->dev);
	int		status;


	/* Clean up MTD stuff. */
	status = mtd_device_unregister(&flash->mtd);
	if (status == 0) {
		kfree(flash->command);
		kfree(flash);
	}
	return 0;
}

#ifdef CONFIG_PM
int nmyx25_device_suspend(struct device *dev)
{
	int ret = 0;

	return ret;
}

int nmyx25_device_resume(struct device *dev)
{
	struct nmyx25	*flash = dev_get_drvdata(dev);
	int ret = 0;
	unsigned int id;

	intelce_get_soc_info(&id, NULL);

	switch (id) {
	case CE2600_SOC_DEVICE_ID:
		break;
	default:
		nmyx25_setup(flash);
		spi_setup(flash->spi);
		break;
	}
	return ret;
}

static const struct dev_pm_ops nmyx25_pm_ops = {
	.suspend	= nmyx25_device_suspend,
	.resume		= nmyx25_device_resume,
};
#endif

static struct spi_driver nmyx25_driver = {
	.driver = {
		.name	= "nmyx25",
		.bus	= &spi_bus_type,
		.owner	= THIS_MODULE,
	},
	.probe	= nmyx25_probe,
	.remove	= __devexit_p(nmyx25_remove),
#ifdef CONFIG_PM
		.driver.pm 		= &nmyx25_pm_ops,
#endif
};


static int __init nmyx25_init(void)
{
	return spi_register_driver(&nmyx25_driver);
}


static void __exit nmyx25_exit(void)
{
	spi_unregister_driver(&nmyx25_driver);
}


module_init(nmyx25_init);
module_exit(nmyx25_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("SPI driver for Numnyx N25Q256/M25XP32 flash chips");
