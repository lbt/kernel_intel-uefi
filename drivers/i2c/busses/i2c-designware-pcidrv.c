/*
 * Synopsys DesignWare I2C adapter driver (master only).
 *
 * Based on the TI DAVINCI I2C adapter driver.
 *
 * Copyright (C) 2006 Texas Instruments.
 * Copyright (C) 2007 MontaVista Software Inc.
 * Copyright (C) 2009 Provigent Ltd.
 * Copyright (C) 2011 Intel corporation.
 *
 * ----------------------------------------------------------------------------
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 * ----------------------------------------------------------------------------
 *
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/errno.h>
#include <linux/sched.h>
#include <linux/err.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/slab.h>
#include <linux/pci.h>
#include <linux/pm_runtime.h>
#include <linux/fs.h>
#include <linux/acpi.h>
#include "i2c-designware-core.h"

#define DRIVER_NAME "i2c-designware-pci"
#define DW_I2C_STATIC_BUS_NUM	10

#define DW_STD_SPEED	100000
#define DW_FAST_SPEED	400000
#define DW_HIGH_SPEED	3400000

enum dw_pci_ctl_id_t {
	moorestown_0,
	moorestown_1,
	moorestown_2,

	medfield_0,
	medfield_1,
	medfield_2,
	medfield_3,
	medfield_4,
	medfield_5,

	valleyview_4,
	valleyview_5,
};

struct dw_pci_controller {
	u32 bus_num;
	u32 bus_cfg;
	u32 tx_fifo_depth;
	u32 rx_fifo_depth;
	u32 clk_khz;
	int share_irq;
	char *acpi_name;
	int (*scl_cfg) (struct dw_i2c_dev *dev);
	void (*reset)(struct dw_i2c_dev *dev);
};

/* VLV2 PCI config space memio access to the controller is
* enabled by
* 1. Reset 0x804 and 0x808 offset from base address.
* 2. Set 0x804 offset from base address to 0x3.
*/
static void vlv2_reset(struct dw_i2c_dev *dev)
{
	int i;

	for (i = 0; i < 10; i++) {
		dw_writel(dev, 0, 0x804);
		dw_writel(dev, 0, 0x808);
		usleep_range(10, 100);

		dw_writel(dev, 3, 0x804);
		usleep_range(10, 100);

		if (dw_readl(dev, DW_IC_COMP_TYPE) != DW_IC_COMP_TYPE_VALUE)
			continue;

		return;
	}

	dev_warn(dev->dev, "vlv2 I2C reset failed\n");
}

static int mfld_i2c_scl_cfg(struct dw_i2c_dev *dev)
{
	dw_writel(dev, PNW_SS_SCLK_HCNT, DW_IC_SS_SCL_HCNT);
	dw_writel(dev, PNW_SS_SCLK_LCNT, DW_IC_SS_SCL_LCNT);

	dw_writel(dev, PNW_FS_SCLK_HCNT, DW_IC_FS_SCL_HCNT);
	dw_writel(dev, PNW_FS_SCLK_LCNT, DW_IC_FS_SCL_LCNT);

	return 0;
}

static int vlv2_i2c_scl_cfg(struct dw_i2c_dev *dev)
{
	dw_writel(dev, VLV2_SS_SCLK_HCNT, DW_IC_SS_SCL_HCNT);
	dw_writel(dev, VLV2_SS_SCLK_LCNT, DW_IC_SS_SCL_LCNT);

	dw_writel(dev, VLV2_FS_SCLK_HCNT, DW_IC_FS_SCL_HCNT);
	dw_writel(dev, VLV2_FS_SCLK_LCNT, DW_IC_FS_SCL_LCNT);

	dw_writel(dev, VLV2_HS_SCLK_HCNT, DW_IC_HS_SCL_HCNT);
	dw_writel(dev, VLV2_HS_SCLK_LCNT, DW_IC_HS_SCL_LCNT);

	return 0;
}

static struct  dw_pci_controller  dw_pci_controllers[] = {
	[moorestown_0] = {
		.bus_num     = 0,
		.bus_cfg   = INTEL_MID_STD_CFG | DW_IC_CON_SPEED_FAST,
		.tx_fifo_depth = 32,
		.rx_fifo_depth = 32,
		.clk_khz      = 25000,
	},
	[moorestown_1] = {
		.bus_num     = 1,
		.bus_cfg   = INTEL_MID_STD_CFG | DW_IC_CON_SPEED_FAST,
		.tx_fifo_depth = 32,
		.rx_fifo_depth = 32,
		.clk_khz      = 25000,
	},
	[moorestown_2] = {
		.bus_num     = 2,
		.bus_cfg   = INTEL_MID_STD_CFG | DW_IC_CON_SPEED_FAST,
		.tx_fifo_depth = 32,
		.rx_fifo_depth = 32,
		.clk_khz      = 25000,
	},
	[medfield_0] = {
		.bus_num     = 0,
		.bus_cfg   = INTEL_MID_STD_CFG | DW_IC_CON_SPEED_FAST,
		.tx_fifo_depth = 256,
		.rx_fifo_depth = 256,
		.clk_khz      = 17000,
		.scl_cfg = mfld_i2c_scl_cfg,
	},
	[medfield_1] = {
		.bus_num     = 1,
		.bus_cfg   = INTEL_MID_STD_CFG | DW_IC_CON_SPEED_STD,
		.tx_fifo_depth = 256,
		.rx_fifo_depth = 256,
		.clk_khz      = 20500,
		.scl_cfg = mfld_i2c_scl_cfg,
	},
	[medfield_2] = {
		.bus_num     = 2,
		.bus_cfg   = INTEL_MID_STD_CFG | DW_IC_CON_SPEED_FAST,
		.tx_fifo_depth = 256,
		.rx_fifo_depth = 256,
		.clk_khz      = 17000,
		.scl_cfg = mfld_i2c_scl_cfg,
	},
	[medfield_3] = {
		.bus_num     = 3,
		.bus_cfg   = INTEL_MID_STD_CFG | DW_IC_CON_SPEED_STD,
		.tx_fifo_depth = 256,
		.rx_fifo_depth = 256,
		.clk_khz      = 20500,
		.scl_cfg = mfld_i2c_scl_cfg,
	},
	[medfield_4] = {
		.bus_num     = 4,
		.bus_cfg   = INTEL_MID_STD_CFG | DW_IC_CON_SPEED_FAST,
		.tx_fifo_depth = 256,
		.rx_fifo_depth = 256,
		.clk_khz      = 17000,
		.scl_cfg = mfld_i2c_scl_cfg,
	},
	[medfield_5] = {
		.bus_num     = 5,
		.bus_cfg   = INTEL_MID_STD_CFG | DW_IC_CON_SPEED_FAST,
		.tx_fifo_depth = 256,
		.rx_fifo_depth = 256,
		.clk_khz      = 17000,
		.scl_cfg = mfld_i2c_scl_cfg,
	},
	[valleyview_4] = {
		.bus_num     = 5,
		.bus_cfg   = INTEL_MID_STD_CFG | DW_IC_CON_SPEED_FAST,
		.tx_fifo_depth = 64,
		.rx_fifo_depth = 64,
		.scl_cfg = vlv2_i2c_scl_cfg,
		.reset = vlv2_reset,
		.share_irq = 1,
		.acpi_name = "\\_SB.I2C5"
	},
	[valleyview_5] = {
		.bus_num     = 6,
		.bus_cfg   = INTEL_MID_STD_CFG | DW_IC_CON_SPEED_FAST,
		.tx_fifo_depth = 64,
		.rx_fifo_depth = 64,
		.scl_cfg = vlv2_i2c_scl_cfg,
		.reset = vlv2_reset,
		.share_irq = 1,
		.acpi_name = "\\_SB.I2C6"
	},
};

#ifdef CONFIG_ACPI
struct i2c_dw_board_info {
	struct i2c_adapter *adap;
	struct i2c_board_info info;
};

static int i2c_dw_find_irq(struct acpi_resource *ares, void *data)
{
	struct i2c_dw_board_info *dwinfo = data;

	if (dwinfo->info.irq < 0) {
		struct resource r;

		if (acpi_dev_resource_interrupt(ares, 0, &r))
			dwinfo->info.irq = r.start;
	}

	/* Tell the ACPI core to skip this resource */
	return 1;
}

static int i2c_dw_find_slaves(struct acpi_resource *ares, void *data)
{
	struct i2c_dw_board_info *dwinfo = data;
	struct device *dev = &dwinfo->adap->dev;
	struct dw_i2c_dev *i2c = i2c_get_adapdata(dwinfo->adap);
	unsigned int connection_speed;

	if (ares->type == ACPI_RESOURCE_TYPE_SERIAL_BUS) {
		struct acpi_resource_i2c_serialbus *sb;

		sb = &ares->data.i2c_serial_bus;
		if (sb->type == ACPI_RESOURCE_SERIAL_TYPE_I2C) {
			connection_speed = sb->connection_speed;
			if (connection_speed == DW_STD_SPEED) {
				i2c->master_cfg &= ~DW_IC_SPEED_MASK;
				i2c->master_cfg |= DW_IC_CON_SPEED_STD;
			} else if (connection_speed == DW_FAST_SPEED) {
				i2c->master_cfg &= ~DW_IC_SPEED_MASK;
				i2c->master_cfg |= DW_IC_CON_SPEED_FAST;
			} else if (connection_speed == DW_HIGH_SPEED) {
				i2c->master_cfg &= ~DW_IC_SPEED_MASK;
				i2c->master_cfg |= DW_IC_CON_SPEED_HIGH;
			}

			i2c_dw_init(i2c);

			dev_info(dev, "I2C speed get from acpi is %dKHz\n",
				connection_speed/1000);

			dwinfo->info.addr = sb->slave_address;
			if (sb->access_mode == ACPI_I2C_10BIT_MODE)
				dwinfo->info.flags |= I2C_CLIENT_TEN;
			dev_info(dev, "\t\tslave_addr 0x%x, irq %d\n",
				dwinfo->info.addr, dwinfo->info.irq);
			if (!i2c_new_device(dwinfo->adap, &dwinfo->info))
				dev_err(dev, "failed to add %s\n",
					dwinfo->info.type);
		}
	}

	/* Tell the ACPI core to skip this resource */
	return 1;
}

static acpi_status i2c_dw_add_device(acpi_handle handle, u32 level,
				       void *data, void **return_value)
{
	struct i2c_dw_board_info *dwinfo = data;
	struct device *dev = &dwinfo->adap->dev;
	struct list_head resource_list;
	struct acpi_device *adev;
	int ret;

	dev_info(dev, "\tCheck next device ...");
	ret = acpi_bus_get_device(handle, &adev);
	if (ret) {
		dev_info(dev, "\t err %d\n", ret);
		return AE_OK;
	}
	dev_info(dev, "\t%s\n", dev_name(&adev->dev));
	if (acpi_bus_get_status(adev) || !adev->status.present) {
		dev_err(dev, "\t\terror, present %d\n", adev->status.present);
		return AE_OK;
	}

	dwinfo->info.acpi_node.handle = handle;
	dwinfo->info.irq = -1;
	strlcpy(dwinfo->info.type, dev_name(&adev->dev),
			sizeof(dwinfo->info.type));

	INIT_LIST_HEAD(&resource_list);
	acpi_dev_get_resources(adev, &resource_list,
				     i2c_dw_find_irq, dwinfo);
	acpi_dev_get_resources(adev, &resource_list,
				     i2c_dw_find_slaves, dwinfo);
	acpi_dev_free_resource_list(&resource_list);

	return AE_OK;
}

static void i2c_dw_scan_devices(struct i2c_adapter *adapter, char *acpi_name)
{
	acpi_handle handle;
	struct i2c_dw_board_info dw_info;
	struct device *dev = &adapter->dev;

	dev_err(dev, "Scan devices on i2c-%d\n", adapter->nr);
	memset(&dw_info, 0, sizeof(dw_info));
	dw_info.adap = adapter;
	acpi_get_handle(NULL, acpi_name, &handle);
	acpi_walk_namespace(ACPI_TYPE_DEVICE, handle, 1,
				     i2c_dw_add_device, NULL,
				     &dw_info, NULL);
}
#else
static void i2c_dw_scan_devices(struct i2c_adapter *adapter, char *acpi_name) {}
#endif

static struct i2c_algorithm i2c_dw_algo = {
	.master_xfer	= i2c_dw_xfer,
	.functionality	= i2c_dw_func,
};

static int i2c_dw_pci_suspend(struct device *dev)
{
	struct pci_dev *pdev = container_of(dev, struct pci_dev, dev);
	struct dw_i2c_dev *i2c = pci_get_drvdata(pdev);
	int err;


	i2c_dw_disable(i2c);

	err = pci_save_state(pdev);
	if (err) {
		dev_err(&pdev->dev, "pci_save_state failed\n");
		return err;
	}

	err = pci_set_power_state(pdev, PCI_D3hot);
	if (err) {
		dev_err(&pdev->dev, "pci_set_power_state failed\n");
		return err;
	}

	return 0;
}

static int i2c_dw_pci_resume(struct device *dev)
{
	struct pci_dev *pdev = container_of(dev, struct pci_dev, dev);
	struct dw_i2c_dev *i2c = pci_get_drvdata(pdev);
	int err;
	u32 enabled;

	enabled = i2c_dw_is_enabled(i2c);
	if (enabled)
		return 0;

	err = pci_set_power_state(pdev, PCI_D0);
	if (err) {
		dev_err(&pdev->dev, "pci_set_power_state() failed\n");
		return err;
	}

	pci_restore_state(pdev);

	i2c_dw_init(i2c);
	return 0;
}

static int i2c_dw_pci_runtime_idle(struct device *dev)
{
	int err = pm_schedule_suspend(dev, 500);
	dev_dbg(dev, "runtime_idle called\n");

	if (err != 0)
		return 0;
	return -EBUSY;
}

static const struct dev_pm_ops i2c_dw_pm_ops = {
	.resume         = i2c_dw_pci_resume,
	.suspend        = i2c_dw_pci_suspend,
	SET_RUNTIME_PM_OPS(i2c_dw_pci_suspend, i2c_dw_pci_resume,
			   i2c_dw_pci_runtime_idle)
};

static u32 i2c_dw_get_clk_rate_khz(struct dw_i2c_dev *dev)
{
	if (dev->use_dyn_clk)
		return dev->clk_khz;
	else
		return dev->controller->clk_khz;
}

static ssize_t store_set_clk(struct device *dev, struct device_attribute *attr,
				const char *buf, size_t size)
{
	struct dw_i2c_dev *i2c = dev_get_drvdata(dev);

	if (sscanf(buf, "%u", &i2c->clk_khz) != 1) {
		dev_err(dev, "input one argument for I2C clock (kHz)\n");
		return -EINVAL;
	}

	return size;
}

static ssize_t show_get_clk(struct device *dev, struct device_attribute *attr,
							char *buf)
{
	struct dw_i2c_dev *i2c = dev_get_drvdata(dev);

	if (i2c->use_dyn_clk)
		return snprintf(buf, PAGE_SIZE, "%d\n", i2c->clk_khz);
	else
		return snprintf(buf, PAGE_SIZE, "%d\n",
				i2c->controller->clk_khz);
}

static ssize_t show_bus_num(struct device *dev, struct device_attribute *attr,
							char *buf)
{
	struct dw_i2c_dev *i2c = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE, "%d\n", i2c->controller->bus_num);
}

#define MODE_NAME_SIZE	10

static ssize_t store_set_mode(struct device *dev, struct device_attribute *attr,
				const char *buf, size_t size)
{
	struct dw_i2c_dev *i2c = dev_get_drvdata(dev);
	char mode[MODE_NAME_SIZE];

	memset(mode, 0, sizeof(mode));

	if (sscanf(buf, "%9s", mode) != 1) {
		dev_err(dev, "input one argument for I2C speed mode\n");
		return -EINVAL;
	}

	if (!strncmp("standard", mode, MODE_NAME_SIZE)
		|| !strncmp("std", mode, MODE_NAME_SIZE))
			i2c->speed_cfg = DW_IC_CON_SPEED_STD;
	else if (!strncmp("fast", mode, MODE_NAME_SIZE))
		i2c->speed_cfg = DW_IC_CON_SPEED_FAST;
	else if (!strncmp("high", mode, MODE_NAME_SIZE)) {
		i2c->master_cfg &= ~DW_IC_SPEED_MASK;
		i2c->master_cfg |= DW_IC_CON_SPEED_HIGH;
	} else
		return -EINVAL;

	return size;
}

static ssize_t show_get_mode(struct device *dev, struct device_attribute *attr,
							char *buf)
{
	int ret;
	u32 speed;
	struct dw_i2c_dev *i2c = dev_get_drvdata(dev);

	if (!i2c->use_dyn_clk)
		speed = i2c->master_cfg & DW_IC_SPEED_MASK;
	else
		speed = i2c->speed_cfg;

	switch (speed) {
	case DW_IC_CON_SPEED_STD:
		ret = snprintf(buf, PAGE_SIZE, "%s\n", "standard");
		break;
	case DW_IC_CON_SPEED_FAST:
		ret = snprintf(buf, PAGE_SIZE, "%s\n", "fast");
		break;
	case DW_IC_CON_SPEED_HIGH:
		ret = snprintf(buf, PAGE_SIZE, "%s\n", "high");
		break;
	default:
		ret = snprintf(buf, PAGE_SIZE, "%s\n", "not supported\n");
		break;
	}

	return ret;
}

static ssize_t store_use_dynamic_clk(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t size)
{
	struct dw_i2c_dev *i2c = dev_get_drvdata(dev);

	if (sscanf(buf, "%d", &i2c->use_dyn_clk) != 1) {
		dev_err(dev,
		"input one argument to decide whether to use dynamic clock\n");
		return -EINVAL;
	}

	return size;
}

static ssize_t show_use_dynamic_clk(struct device *dev,
				    struct device_attribute *attr,
				    char *buf)
{
	struct dw_i2c_dev *i2c = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE, "%d\n", i2c->use_dyn_clk);
}

static DEVICE_ATTR(bus_num, S_IRUGO, show_bus_num, NULL);
static DEVICE_ATTR(clk_khz, S_IRUGO | S_IWUSR, show_get_clk, store_set_clk);
static DEVICE_ATTR(mode, S_IRUGO | S_IWUSR, show_get_mode, store_set_mode);
static DEVICE_ATTR(use_dynamic_clk, S_IRUGO | S_IWUSR, show_use_dynamic_clk,
						store_use_dynamic_clk);

static struct attribute *designware_i2c_attrs[] = {
	&dev_attr_bus_num.attr,
	&dev_attr_clk_khz.attr,
	&dev_attr_mode.attr,
	&dev_attr_use_dynamic_clk.attr,
	NULL,
};

static struct attribute_group designware_i2c_attr_group = {
	.name = "i2c_sysfs",
	.attrs = designware_i2c_attrs,
};

static int i2c_dw_pci_probe(struct pci_dev *pdev,
			    const struct pci_device_id *id)
{
	struct dw_i2c_dev *dev;
	struct i2c_adapter *adap;
	int r;
	struct  dw_pci_controller *controller;

	controller = &dw_pci_controllers[id->driver_data];

	r = pcim_enable_device(pdev);
	if (r) {
		dev_err(&pdev->dev, "Failed to enable I2C PCI device (%d)\n",
			r);
		return r;
	}

	r = pcim_iomap_regions(pdev, 1 << 0, pci_name(pdev));
	if (r) {
		dev_err(&pdev->dev, "I/O memory remapping failed\n");
		return r;
	}

	dev = devm_kzalloc(&pdev->dev, sizeof(struct dw_i2c_dev), GFP_KERNEL);
	if (!dev)
		return -ENOMEM;

	init_completion(&dev->cmd_complete);
	mutex_init(&dev->lock);
	dev->clk = NULL;
	dev->controller = controller;
	dev->get_clk_rate_khz = i2c_dw_get_clk_rate_khz;
	dev->base = pcim_iomap_table(pdev)[0];
	dev->dev = &pdev->dev;
	dev->functionality =
		I2C_FUNC_I2C |
		I2C_FUNC_SMBUS_BYTE |
		I2C_FUNC_SMBUS_BYTE_DATA |
		I2C_FUNC_SMBUS_WORD_DATA |
		I2C_FUNC_SMBUS_I2C_BLOCK;
	dev->master_cfg =  controller->bus_cfg;
	dev->get_scl_cfg = controller->scl_cfg;
	dev->clk_khz = controller->clk_khz;
	dev->speed_cfg = dev->master_cfg & DW_IC_SPEED_MASK;
	dev->use_dyn_clk = 0;
	dev->reset = controller->reset;
	dev->share_irq = controller->share_irq;


	pci_set_drvdata(pdev, dev);

	dev->tx_fifo_depth = controller->tx_fifo_depth;
	dev->rx_fifo_depth = controller->rx_fifo_depth;
	r = i2c_dw_init(dev);
	if (r)
		return r;

	adap = &dev->adapter;
	i2c_set_adapdata(adap, dev);
	adap->owner = THIS_MODULE;
	adap->class = 0;
	adap->algo = &i2c_dw_algo;
	adap->dev.parent = &pdev->dev;
	adap->nr = controller->bus_num;
	snprintf(adap->name, sizeof(adap->name), "i2c-designware-pci-%d",
		adap->nr);

	r = devm_request_irq(&pdev->dev, pdev->irq, i2c_dw_isr, IRQF_SHARED,
			adap->name, dev);
	if (r) {
		dev_err(&pdev->dev, "failure requesting irq %i\n", dev->irq);
		return r;
	}

	i2c_dw_disable_int(dev);
	i2c_dw_clear_int(dev);
	r = i2c_add_numbered_adapter(adap);
	if (r) {
		dev_err(&pdev->dev, "failure adding adapter\n");
		return r;
	}

	r = sysfs_create_group(&pdev->dev.kobj, &designware_i2c_attr_group);
	if (r) {
		dev_err(&pdev->dev,
			"Unable to export sysfs interface, error: %d\n", r);
		goto err_del_adap;
	}
	pm_runtime_use_autosuspend(&pdev->dev);
	pm_runtime_allow(&pdev->dev);

	if (controller->acpi_name)
		i2c_dw_scan_devices(adap, controller->acpi_name);

	return 0;

err_del_adap:
	i2c_del_adapter(&dev->adapter);
	return r;
}

static void i2c_dw_pci_remove(struct pci_dev *pdev)
{
	struct dw_i2c_dev *dev = pci_get_drvdata(pdev);

	i2c_dw_disable(dev);
	pm_runtime_forbid(&pdev->dev);
	pm_runtime_get_noresume(&pdev->dev);

	sysfs_remove_group(&pdev->dev.kobj, &designware_i2c_attr_group);

	i2c_del_adapter(&dev->adapter);
}

/* work with hotplug and coldplug */
MODULE_ALIAS("i2c_designware-pci");

DEFINE_PCI_DEVICE_TABLE(i2c_designware_pci_ids) = {
	/* Moorestown */
	{ PCI_VDEVICE(INTEL, 0x0802), moorestown_0 },
	{ PCI_VDEVICE(INTEL, 0x0803), moorestown_1 },
	{ PCI_VDEVICE(INTEL, 0x0804), moorestown_2 },
	/* Medfield */
	{ PCI_VDEVICE(INTEL, 0x0817), medfield_0 },
	{ PCI_VDEVICE(INTEL, 0x0818), medfield_1 },
	{ PCI_VDEVICE(INTEL, 0x0819), medfield_2 },
	{ PCI_VDEVICE(INTEL, 0x082C), medfield_3 },
	{ PCI_VDEVICE(INTEL, 0x082D), medfield_4 },
	{ PCI_VDEVICE(INTEL, 0x082E), medfield_5 },
	/* Valleyview 2 */
	{ PCI_VDEVICE(INTEL, 0x0F45), valleyview_4 },
	{ PCI_VDEVICE(INTEL, 0x0F46), valleyview_5 },
	{ 0,}
};
MODULE_DEVICE_TABLE(pci, i2c_designware_pci_ids);

static struct pci_driver dw_i2c_driver = {
	.name		= DRIVER_NAME,
	.id_table	= i2c_designware_pci_ids,
	.probe		= i2c_dw_pci_probe,
	.remove		= i2c_dw_pci_remove,
	.driver         = {
		.pm     = &i2c_dw_pm_ops,
	},
};

module_pci_driver(dw_i2c_driver);

static int __init dw_i2c_reserve_static_bus(void)
{
	struct i2c_board_info dummy = {
		I2C_BOARD_INFO("dummy", 0xff),
	};

	i2c_register_board_info(DW_I2C_STATIC_BUS_NUM, &dummy, 1);
	return 0;
}
subsys_initcall(dw_i2c_reserve_static_bus);

MODULE_AUTHOR("Baruch Siach <baruch@tkos.co.il>");
MODULE_DESCRIPTION("Synopsys DesignWare PCI I2C bus adapter");
MODULE_LICENSE("GPL");
