/*
 *
 * punit_reboot_sync.c
 * Description:
 * power control unit device reboot sync driver
 *
 *  GPL LICENSE SUMMARY
 *
 *  Copyright(c) 2013 Intel Corporation. All rights reserved.
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

/*------------------------------------------------------------------------------
 * File Name: punit_reboot_sync.c
 *------------------------------------------------------------------------------
 */
#include <linux/kernel.h>
#include <linux/io.h>
#include <linux/version.h>
#include <linux/module.h>   /* for modules */
#include <linux/fs.h>       /* file_operations */
#include <linux/uaccess.h>  /* copy_(to,from)_user */
#include <linux/init.h>     /* module_init, module_exit */
#include <linux/errno.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/device.h>
#include <linux/ioctl.h>
#include <linux/proc_fs.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/workqueue.h>
#include <linux/sched.h>
#include <linux/pci.h>
#include <linux/ce_mailbox.h>
#include <linux/iosf_core.h>
#include <linux/punit_reboot_sync.h>

//#define P_UNIT_DEBUG
#ifdef  P_UNIT_DEBUG
/* note: prints function name for you */
#  define DPRINTK(fmt, args...) printk("%-40s:%5d " fmt, __FUNCTION__,__LINE__, ## args)
#else
#  define DPRINTK(fmt, args...)
#endif

DEFINE_MUTEX(p_unit_mutex);
EXPORT_SYMBOL(p_unit_mutex);

struct iosf_host *host;
/* Indication if reset event has been received from Punit. Default is FALSE */
static uint32_t              punit_reset_event = 0;
/* Indication if Punit has reboot sync IPC command with Atom. Default is FALSE */
static uint32_t              punit_valid_version = 0;

/*
 * Data for PCI driver interface
 */
static DEFINE_PCI_DEVICE_TABLE(p_unit_id_tables) = {
        { PCI_DEVICE(0x8086, 0x08bc), },
        { 0, },                 /* End of list */
};
MODULE_DEVICE_TABLE(pci, p_unit_id_tables);

#define ADDR_ATOM_PM_CMD        (0x000000DC)
#define ADDR_ATOM_PM_DATA       (0x000000DD)
#define ADDR_PM_ATOM_CMD        (0x000000DE)
#define ADDR_PM_ATOM_DATA       (0x000000DF)
#define ATOM_PM_INTR            (0x000000E0)
#define PUNIT_FW_VERSION        (0x00000087)

#define P_UNIT_CMD_TYPE_IPC         (0x0 << 8)
#define P_UNIT_CMD_TYPE_BBU         (0x1 << 8)
#define P_UNIT_CMD_TYPE_WATCHDOG    (0x2 << 8)
#define P_UNIT_CMD_TYPE_RESET       (0x3 << 8)

#define P_UNIT_CMD_DATA_ATTACHED    (0x1 << 14)
#define P_UNIT_CMD_DATA_EXPECTED    (0x1 << 15)

#define P_UNIT_CMD_IDLE             (0x0000)
#define P_UNIT_CMD_ACK              (0x0001)
#define ATOM_PM_INTR_DIS            (0x0000)
#define ATOM_PM_INTR_EN             (0x0001)


#define P_UNIT_CMD_RESET_REQ_COLD_RESET                         (P_UNIT_CMD_TYPE_RESET | 0x00) // Initiate a cold reset
#define P_UNIT_CMD_RESET_REQ_WARM_RESET                         (P_UNIT_CMD_TYPE_RESET | 0x01) // Initiate a warm reset
#define P_UNIT_CMD_RESET_GET_LAST_RESET_CAUSE                   (P_UNIT_CMD_TYPE_RESET | 0x04 | P_UNIT_CMD_DATA_EXPECTED) // Returns the last reset cause indicated by firmware. If a hardware reset(not under firmware control) occurs the state indicated here is a "cold boot"
#define P_UNIT_CMD_RESET_CLR_LAST_RESET_CAUSE                   (P_UNIT_CMD_TYPE_RESET | 0x05)

#define P_UNIT_CMD_RESET_SET_WARM_RESET_ON_BUTTON               (P_UNIT_CMD_TYPE_RESET | 0x08) // When set, on a reset button press the firmware will start the warm reset sequence (warm reset by default)
#define P_UNIT_CMD_RESET_SET_COLD_RESET_ON_BUTTON               (P_UNIT_CMD_TYPE_RESET | 0x09) // When set, on a reset button press the firmware will start the cold reset sequence (warm reset by default)

#define P_UNIT_CMD_RESET_EN_ATOM_RESET_INDICATION               (P_UNIT_CMD_TYPE_RESET | 0x0C | P_UNIT_CMD_DATA_ATTACHED) // When enabled, if a reset request is sent to Punit firmware, an IPC will be sent to ATOM to inform it a reset will occur. Data Sent: 0 = Disable (default), 1 = Enable
#define P_UNIT_CMD_RESET_ATOM_RESET_INDICATION_ACK              (P_UNIT_CMD_TYPE_RESET | 0x11) // After receiving the IPC of a reset request, the ATOM should respond with this ACK IPC when it gives the firmware the OK to continue with the reset.
#define P_UNIT_CMD_RESET_ATOM_RESET_INDICATION_ACK_TIMEOUT      (P_UNIT_CMD_TYPE_RESET | 0x14 | P_UNIT_CMD_DATA_ATTACHED) // Set the timeout time in milliseconds for the ATOM to ack, if this time expires the reset sequence will proceed without an ack. Unit is in milliseconds (valid range: 0-60,000 milliseconds) (default 2,000  milliseconds)
#define P_UNIT_CMD_RESET_ATOM_RESET_INDICATION                  (P_UNIT_CMD_TYPE_RESET | 0x10 | P_UNIT_CMD_DATA_ATTACHED) // If a reset request is sent to firmware and if enabled this IPC will be sent to ATOM to inform it a reset will occur.


#define P_UNIT_CMD_WATCHDOG_DO_COLD_RESET   (P_UNIT_CMD_TYPE_WATCHDOG | 0x4)
#define P_UNIT_CMD_WATCHDOG_DO_WARM_RESET   (P_UNIT_CMD_TYPE_WATCHDOG | 0x5)
#define P_UNIT_CMD_WATCHDOG_DO_CPU_RESET    (P_UNIT_CMD_TYPE_WATCHDOG | 0x6)
#define P_UNIT_CMD_WATCHDOG_DO_NOTHING      (P_UNIT_CMD_TYPE_WATCHDOG | 0x7)

#define IOSF_PORT_PUNIT 4
#define MAX_ACK_TIMEOUT 60000
#define MIN_ACK_TIMEOUT 0
#define PUNIT_FW_VERSION_A106 0x10006
#define ATOM_REBOOT_WAIT_TOTAL_TIME_MSEC    5000

/**
 * P-UNIT Low Level Building Blocks ....
 *
 **/
static int p_unit_cmd(unsigned int command)
{
	unsigned int data = command;

	DPRINTK("Enter cmd = 0x%08X \n", command);
	/* Send Command to Punit */
	if (kiosf_reg_modify(host,IOSF_PORT_PUNIT, ADDR_ATOM_PM_CMD, 0xFFFF, data)){
		return -EINVAL;
	}

	if (kiosf_reg_write32(host,IOSF_PORT_PUNIT, ATOM_PM_INTR, ATOM_PM_INTR_EN)){
		return -EINVAL;
	}

	/* Complete Initiator Handshake with Punit */
	do{
		if (kiosf_reg_read32(host,IOSF_PORT_PUNIT, ADDR_ATOM_PM_CMD, &data)){
			kiosf_reg_write32(host,IOSF_PORT_PUNIT, ATOM_PM_INTR, ATOM_PM_INTR_DIS);
			return -EINVAL;
		}
	}
	while (P_UNIT_CMD_ACK != data);

	if (kiosf_reg_write32(host,IOSF_PORT_PUNIT, ADDR_ATOM_PM_CMD, P_UNIT_CMD_IDLE)){
		npcpu_bootcfg_ctrl_write_reg(BOOTCFG_REG_SW_INT1_CLR, BOOTCFG_REG_SW_INT1_ARM11_2_PUNIT_ISR);
		return -EINVAL;
	}
	kiosf_reg_write32(host,IOSF_PORT_PUNIT, ATOM_PM_INTR, ATOM_PM_INTR_DIS);
	DPRINTK("Exit  cmd = 0x%08X \n", command);

	return 0;
}

static int p_unit_cmd_wr_data(unsigned int  command, unsigned int  data)
{
	int ret;

	DPRINTK("Enter cmd = 0x%08X data = 0x%08X\n", command, data);
	/* Send Data */
	if(kiosf_reg_write32(host, IOSF_PORT_PUNIT, ADDR_ATOM_PM_DATA, data)){
		return -EINVAL;
	}

	/* Send Command and Wait for ACK */
	ret = p_unit_cmd(command);
	DPRINTK("Exit  cmd = 0x%08X data = 0x%08X\n", command, data);

	return ret;
}

int p_unit_reset_soc( void )
{
	DPRINTK("Enter, punit_reset_event=%d\n", punit_reset_event);

	/* In case of reset event from Punit, we just need to ACK it. */
	if (punit_reset_event && punit_valid_version){
		mutex_lock(&p_unit_mutex);
		p_unit_cmd(P_UNIT_CMD_RESET_ATOM_RESET_INDICATION_ACK);
		mutex_unlock(&p_unit_mutex);
	}
	else{
		/* notify the reboot process of appcpu is done and then wait */
		npcpu_bootcfg_ctrl_write_reg(BOOTCFG_REG_SW_INT_CLR,BOOTCFG_REG_SW_INT_ATOM_2_ARM11_INTC_REBOOT_ISR);
		mdelay(ATOM_REBOOT_WAIT_TOTAL_TIME_MSEC);
		outb(0x8, 0xcf9);
	}
	DPRINTK("Exit, punit_reset_event=%d\n", punit_reset_event);
	return 0;
}
EXPORT_SYMBOL(p_unit_reset_soc);

uint32_t reset_from_punit( void )
{
	return punit_reset_event;
}
EXPORT_SYMBOL(reset_from_punit);

/*
 * brief proc file to configure P-Unit atom reset indiction ack timeout value
 *
 **/
static int p_unit_proc_control(struct file *fp, const char * buf, unsigned long count, void * data)
{
	unsigned char local_buf[20],header[20];
	int ret_val = 0;
	unsigned int ack_timeout;

	if (count > 20) {
		printk(KERN_ERR "Buffer Overflow\n");
		return -EFAULT;
	}

	if(copy_from_user(local_buf,buf,count))
		return -EFAULT;

	/* Ignoring last \n char */
	local_buf[count-1]='\0';
	ret_val = count;

	/* compare header with special header string*/
	sscanf(local_buf,"%s", header);

	if (!strcmp(header,"ack_timeout")) {
		sscanf(local_buf+11,"%d",&ack_timeout);
		if(ack_timeout <= MAX_ACK_TIMEOUT){
			mutex_lock(&p_unit_mutex);
			if(punit_valid_version){
				p_unit_cmd_wr_data(P_UNIT_CMD_RESET_ATOM_RESET_INDICATION_ACK_TIMEOUT, ack_timeout);
				printk(KERN_INFO "Set atom reset punit indiction ack timeout to %dms\n",ack_timeout);
			}
			mutex_unlock(&p_unit_mutex);
		}
		else{
			printk(KERN_INFO "Ack timeout value should be between 0-60000 ms\n");
			return -EFAULT;
		}
	}
	else{
		printk(KERN_INFO "Unknown operation\n");
		return -EFAULT;
	}

	return ret_val;
}

static irqreturn_t p_unit_isr(int irq, void *dev_id)
{
	unsigned int command;
	DPRINTK("In the punit irq handler\n");

	if (kiosf_reg_read32(host,IOSF_PORT_PUNIT, ADDR_PM_ATOM_CMD, &command)) {
		printk("Error in first kiosf_reg_read32 \n");
	}

	if (command == P_UNIT_CMD_RESET_ATOM_RESET_INDICATION) {
		DPRINTK("Interrupted - reset indication\n");
		punit_reset_event = 1;
		/* We will start reboot process by simulating NP->APP reboot interrupt */
		npcpu_bootcfg_ctrl_write_reg(BOOTCFG_REG_SW_INT1_SET, BOOTCFG_REG_SW_INT1_ARM11_2_ATOM_REBOOT_ISR);
	}
	else {
		printk("Interrupted - unknown command 0x%04X\n", command);
		return IRQ_NONE;
	}

	return IRQ_HANDLED;
}

/*
 * This function is the power control unit device reboot sync module init function.
 * @pdev: PCI device information struct
 * @ent: entry in p_unit_id_tables
 * return 0 on success else negative number.
 *
 **/
static int __devinit p_unit_probe(struct pci_dev *pdev, const struct pci_device_id *id)
{
	int ret,fw_version;
	struct proc_dir_entry * dir;


	host = iosf_request(0);
	if (NULL == host) {
		printk(KERN_ERR "Failed to open iosf device\n");
		return -ENODEV;
	}
	if (kiosf_reg_read32(host,IOSF_PORT_PUNIT, PUNIT_FW_VERSION, &fw_version)) {
		printk("%s:%d Error in kiosf_reg_read32 \n",__FUNCTION__,__LINE__);
		ret = -EINVAL;
		goto err_iosf_read;
	}

	punit_valid_version = ((fw_version & 0xffffff)>=PUNIT_FW_VERSION_A106);
	DPRINTK("fw_version is %x, punit_valid_version is %d\n",fw_version,punit_valid_version);

	/* The Punit reboot sync function is only available when fw version is equal or higher than A1.0.6*/
	if(!punit_valid_version){
		return 0;
	}
	ret = pci_enable_device(pdev);
	if(ret)
		goto err_iosf_read;

	/* Proc filesystem utilities.... */
	if (NULL == (dir = create_proc_entry("punit_control", 0, NULL))){
		printk("%s:%d ERROR ....\n",__FUNCTION__,__LINE__);
		ret = -EIO;
		goto err_iosf_read;
	}
	dir->write_proc = p_unit_proc_control;

	ret = request_irq(pdev->irq, p_unit_isr, IRQF_TRIGGER_RISING| IRQF_ONESHOT| IRQF_DISABLED, "punit_int", NULL );
	if (ret) {
		printk(KERN_ERR "PUNIT: Unable to allocate pUnit IRQ\n");
		goto err_iosf_read;
	}

	if(punit_valid_version){
		/* ACK timeout is 14 seconds */
		mutex_lock(&p_unit_mutex);
		ret = p_unit_cmd_wr_data(P_UNIT_CMD_RESET_ATOM_RESET_INDICATION_ACK_TIMEOUT, 14000);
		mutex_unlock(&p_unit_mutex);
		if(ret)
			goto err_punit_cmd;
		mutex_lock(&p_unit_mutex);
		ret = p_unit_cmd_wr_data(P_UNIT_CMD_RESET_EN_ATOM_RESET_INDICATION, 1);
		mutex_unlock(&p_unit_mutex);
		if(ret)
			goto err_punit_cmd;
	}
	return 0;

err_punit_cmd:
	free_irq(pdev->irq, NULL);
err_iosf_read:
	iosf_release(host);
	punit_valid_version = 0;
	return ret;
}


/**
 * p_unit_remove - Device Removal Routine
 * @pdev: PCI device information struct
 * This function is the power control unit device reboot sync module exit function.
 *
 */
static void __devexit p_unit_remove(struct pci_dev *pdev)
{
	iosf_release(host);

	/* The Punit reboot sync function is only available when fw version is higher than A1.0.6*/
	if(!punit_valid_version)
		return;

	free_irq(pdev->irq, NULL);
	punit_valid_version = 0;
	punit_reset_event = 0;
	printk(KERN_INFO "%s:%d device has been unregistered\n",__FUNCTION__,__LINE__);
}

static struct pci_driver p_unit_driver = {
	.name = "p_unit_drv",
	.probe          = p_unit_probe,
	.remove         = __devexit_p(p_unit_remove),
	.id_table       = p_unit_id_tables,
};

static int __init punit_reboot_sync_init(void)
{
	return pci_register_driver(&p_unit_driver);
}

static void __exit punit_reboot_sync_exit(void)
{
	pci_unregister_driver(&p_unit_driver);
}


module_init(punit_reboot_sync_init);
module_exit(punit_reboot_sync_exit);

/* Driver identification */
MODULE_DESCRIPTION("Power Control Unit Reboot Sync Driver");
MODULE_LICENSE("GPL");
