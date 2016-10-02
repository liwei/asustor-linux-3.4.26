/*
 * CE4100's SPI device is more or less the same one as found on PXA
 *
 */
#include <linux/pci.h>
#include <linux/platform_device.h>
#include <linux/of_device.h>
#include <linux/spi/spi.h>
#include <linux/module.h>
#include <linux/spi/pxa2xx_spi.h>

#define CE4200_NUM_SPI_MASTER 1
#define CE4200_NUM_CHIPSELECT 4
#define CE4X00_SPI_MAX_SPEED  1843200
#define CE5X00_SPI_MAX_SPEED  5000000

#define GEN3_BOARD_INFO_CONFIG

#ifdef  SPI_CE_DEBUG
#define spi_dbg(fmt, args...) do \
                             { \
                               printk(KERN_INFO fmt, ##args); \
                              } while(0)
#else
#define spi_dbg(fmt,args...) do {} while(0)
#endif

struct ce4100_info {
	struct ssp_device ssp;
	struct platform_device *spi_pdev;
};

static DEFINE_MUTEX(ssp_lock);
static LIST_HEAD(ssp_list);

#ifdef CONFIG_GEN3_SPI
#ifdef GEN3_BOARD_INFO_CONFIG
struct spi_board_info CE4X00_spi_devices[]={
  {
    .modalias = "spidev",
    .chip_select = 0,
    .max_speed_hz = CE4X00_SPI_MAX_SPEED,
    .bus_num = 0,
  },
  {
    .modalias = "spidev",
    .chip_select = 1,
    .max_speed_hz = CE4X00_SPI_MAX_SPEED,
    .bus_num = 0,
  },
  {
    .modalias = "spidev",
    .chip_select = 2,
    .max_speed_hz = CE4X00_SPI_MAX_SPEED,
    .bus_num = 0,
  },
  {
    .modalias = "spidev",
    .chip_select = 3,
    .max_speed_hz = CE4X00_SPI_MAX_SPEED,
    .bus_num = 0,
  },
};

struct spi_board_info CE5X00_spi_devices[]={
  {
    .modalias = "spidev",
    .chip_select = 0,
    .max_speed_hz = CE5X00_SPI_MAX_SPEED,
    .bus_num = 0,
  },
  {
    .modalias = "spidev",
    .chip_select = 1,
    .max_speed_hz = CE5X00_SPI_MAX_SPEED,
    .bus_num = 0,
  },
  {
    .modalias = "spidev",
    .chip_select = 2,
    .max_speed_hz = CE5X00_SPI_MAX_SPEED,
    .bus_num = 0,
  },
  {
    .modalias = "spidev",
    .chip_select = 3,
    .max_speed_hz = CE5X00_SPI_MAX_SPEED,
    .bus_num = 0,
  },
};

#endif
#endif

struct ssp_device *pxa_ssp_request(int port, const char *label)
{
	struct ssp_device *ssp = NULL;

	mutex_lock(&ssp_lock);

	list_for_each_entry(ssp, &ssp_list, node) {
		if (ssp->port_id == port && ssp->use_count == 0) {
			ssp->use_count++;
			ssp->label = label;
			break;
		}
	}

	mutex_unlock(&ssp_lock);

	if (&ssp->node == &ssp_list)
		return NULL;

	return ssp;
}
EXPORT_SYMBOL_GPL(pxa_ssp_request);

void pxa_ssp_free(struct ssp_device *ssp)
{
	mutex_lock(&ssp_lock);
	if (ssp->use_count) {
		ssp->use_count--;
		ssp->label = NULL;
	} else
		dev_err(&ssp->pdev->dev, "device already free\n");
	mutex_unlock(&ssp_lock);
}
EXPORT_SYMBOL_GPL(pxa_ssp_free);

#ifdef CONFIG_GEN3_SPI
#ifdef GEN3_BOARD_INFO_CONFIG
static void add_spi_dev_devices(void)
{
  unsigned int id;

	spi_dbg("add CE SPI board info\n");
	intelce_get_soc_info(&id, NULL);
	switch (id)
	{
		case CE5300_SOC_DEVICE_ID:
			spi_register_board_info(CE5X00_spi_devices,ARRAY_SIZE(CE5X00_spi_devices));
		break;
		case CE4200_SOC_DEVICE_ID:
		default:
			spi_register_board_info(CE4X00_spi_devices,ARRAY_SIZE(CE4X00_spi_devices));
			break;
	}
}
static void remove_spi_dev_devices(void)
{
  unsigned int id;

	spi_dbg("remove CE SPI board info\n");
	intelce_get_soc_info(&id, NULL);
	switch (id)
	{
		case CE5300_SOC_DEVICE_ID:
			spi_unregister_board_info(CE5X00_spi_devices, ARRAY_SIZE(CE5X00_spi_devices));
			break;
		case CE4200_SOC_DEVICE_ID:
		default:
			spi_unregister_board_info(CE4X00_spi_devices, ARRAY_SIZE(CE4X00_spi_devices));
			break;
	}
}
#endif
#endif

static int __devinit ce4100_spi_probe(struct pci_dev *dev,
		const struct pci_device_id *ent)
{
	int ret;
	resource_size_t phys_beg;
	resource_size_t phys_len;
	struct ce4100_info *spi_info = NULL;
	struct platform_device *pdev;
	struct pxa2xx_spi_master *spi_pdata = NULL;
	struct ssp_device *ssp;
	unsigned int id;

	ret = pci_enable_device(dev);
	if (ret)
		return ret;

	phys_beg = pci_resource_start(dev, 0);
	phys_len = pci_resource_len(dev, 0);

	if (!request_mem_region(phys_beg, phys_len,
				"CE4100 SPI")) {
		dev_err(&dev->dev, "Can't request register space.\n");
		ret = -EBUSY;
		return ret;
	}

	pdev = platform_device_alloc("pxa2xx-spi", dev->devfn);
	spi_info = kzalloc(sizeof(*spi_info), GFP_KERNEL);
	if (!pdev || !spi_info ) {
		ret = -ENOMEM;
		goto err_malloc_spi_info;
	}
	spi_pdata = kzalloc(sizeof(*spi_pdata), GFP_KERNEL);
	if (!spi_pdata) {
		ret = -ENOMEM;
		goto err_malloc_spi_pdata;

    }
    printk("spi_pdata is %p\n", spi_pdata);
	spi_pdata->num_chipselect = dev->devfn;
	spi_pdata->num_chipselect = CE4200_NUM_CHIPSELECT;
    pdev->dev.platform_data = spi_pdata;

	pdev->id = CE4200_NUM_SPI_MASTER-1;
	pdev->dev.parent = &dev->dev;
#ifdef CONFIG_OF
	pdev->dev.of_node = dev->dev.of_node;
#endif
	ssp = &spi_info->ssp;


	ssp->phys_base = pci_resource_start(dev, 0);
	ssp->mmio_base = ioremap(phys_beg, phys_len);
	if (!ssp->mmio_base) {
		dev_err(&pdev->dev, "failed to ioremap() registers\n");
		ret = -EIO;
		goto err_ioremap;
	}
	ssp->irq = dev->irq;
	ssp->port_id = pdev->id;
	intelce_get_soc_info(&id, NULL);
	switch (id)
	{
		case CE5300_SOC_DEVICE_ID:
			ssp->type = CE5X00_SSP;
			printk("ssp type is CE5X00 SSP\n");
			break;
		case CE4200_SOC_DEVICE_ID:
		default:
			ssp->type = CE4100_SSP;
			printk("ssp type is CE4100 SSP\n");
			break;
	}

	mutex_lock(&ssp_lock);
	list_add(&ssp->node, &ssp_list);
	mutex_unlock(&ssp_lock);

	pci_set_drvdata(dev, spi_info);

#ifdef CONFIG_GEN3_SPI
#ifdef GEN3_BOARD_INFO_CONFIG
        add_spi_dev_devices();
#endif
#endif
	spi_info->spi_pdev = pdev;
	ret = platform_device_add(pdev);
	if (ret)
		goto err_dev_add;

	return ret;

err_dev_add:
	pci_set_drvdata(dev, NULL);
	mutex_lock(&ssp_lock);
	list_del(&ssp->node);
	mutex_unlock(&ssp_lock);
	iounmap(ssp->mmio_base);

err_ioremap:

err_malloc_spi_pdata:
    kfree(spi_info);

err_malloc_spi_info:
	release_mem_region(phys_beg, phys_len);
	platform_device_put(pdev);
	return ret;
}

static void __devexit ce4100_spi_remove(struct pci_dev *dev)
{
	struct ce4100_info *spi_info;
	struct ssp_device *ssp;

	spi_info = pci_get_drvdata(dev);
	ssp = &spi_info->ssp;

#ifdef CONFIG_GEN3_SPI
#ifdef GEN3_BOARD_INFO_CONFIG
        remove_spi_dev_devices();
#endif
#endif

	platform_device_unregister(spi_info->spi_pdev);
	iounmap(ssp->mmio_base);
	release_mem_region(pci_resource_start(dev, 0),
			pci_resource_len(dev, 0));

	mutex_lock(&ssp_lock);
	list_del(&ssp->node);
	mutex_unlock(&ssp_lock);

	pci_set_drvdata(dev, NULL);
	pci_disable_device(dev);
	kfree(spi_info);
}

#ifdef CONFIG_PM
static int ce4XXX_spi_suspend(struct pci_dev *dev, pm_message_t state)
{
	spi_dbg("Start to do spi pci suspend\n");
	pci_save_state(dev);
	pci_set_power_state(dev, pci_choose_state(dev, state));
	return 0;
}

static int ce4XXX_spi_resume(struct pci_dev *dev)
{
	spi_dbg("Start to do spi pci resume\n");
	pci_set_power_state(dev, PCI_D0);
	pci_restore_state(dev);

	return 0;
}
#endif


static DEFINE_PCI_DEVICE_TABLE(ce4100_spi_devices) = {
	{ PCI_DEVICE(PCI_VENDOR_ID_INTEL, 0x2e6a) },
	{ },
};
MODULE_DEVICE_TABLE(pci, ce4100_spi_devices);

static struct pci_driver ce4100_spi_driver = {
	.name           = "ce4100_spi",
	.id_table       = ce4100_spi_devices,
	.probe          = ce4100_spi_probe,
	.remove         = __devexit_p(ce4100_spi_remove),
#ifdef CONFIG_PM
        .suspend        = ce4XXX_spi_suspend,
        .resume         = ce4XXX_spi_resume,
#endif

};

static int __init ce4100_spi_init(void)
{
	return pci_register_driver(&ce4100_spi_driver);
}
module_init(ce4100_spi_init);

static void __exit ce4100_spi_exit(void)
{
	pci_unregister_driver(&ce4100_spi_driver);
}
module_exit(ce4100_spi_exit);

MODULE_DESCRIPTION("CE4100 PCI-SPI glue code for PXA's driver");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Sebastian Andrzej Siewior <bigeasy@linutronix.de>");
