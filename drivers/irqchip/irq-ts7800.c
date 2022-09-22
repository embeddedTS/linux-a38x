// SPDX-License-Identifier: GPL-2.0-only
/*
 * FPGA based IRQs contorller driver for TS-7800v1
 *
 * linux/drivers/mmc/host/ts7800v1_sdmmc.c
 *
 * Copyright (C) 2022 Savoir-faire Linux <contact@savoirfairelinux.com>
 *
 */

#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/irq.h>
#include <linux/irqchip.h>
#include <linux/irqchip/chained_irq.h>
#include <linux/irqdomain.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/platform_device.h>

#define DRIVER_NAME "ts7800-irqc"

#define IRQ_MASK_REG 0x4
#define IRQ_STATUS_REG 0x8
#define TS7800_IRQ_NUM 0x20

/* 6.2.7 FPGA IRQ Controller, map/enable only specific chained interrupts */
/*
 * IRQ 	Description
 * 0x0 	dma_cpu_pci_dack_o
 * 0x1 	dma_fpga_dack_o
 * 0x2 	SD Busy#
 * 0x3 	isa_irq3
 * 0x4 	isa_irq4
 * 0x5 	isa_irq5
 * 0x6 	isa_irq6
 * 0x7 	isa_irq7
 * 0x8 	Reserved
 * 0x9 	isa_irq9
 * 0xa 	isa_irq10
 * 0xb 	isa_irq11
 * 0xc 	isa_irq12
 * 0xd	isa_irq14
 * 0xe 	isa_irq15
 * 0x10:0x19    tsuart_irqs
*/

static irq_hw_number_t enabled_mappings[] = { 0x10, 0x11, 0x12, 0x13, 0x14,
					      0x15, 0x16, 0x17, 0x18, 0x19 };

struct ts7800_irq_data {
	int mpp7_virq;
	void __iomem *base;
	void __iomem *bridge;
	struct irq_domain *domain;
	struct irq_chip irq_chip;
};

static void ts7800_irq_mask(struct irq_data *d)
{
	struct ts7800_irq_data *data = irq_data_get_irq_chip_data(d);
	u32 fpga_mask_reg = readl(data->base + IRQ_MASK_REG);
	u32 mask_bits = 1 << d->hwirq;

	writel(fpga_mask_reg & ~mask_bits, data->base + IRQ_MASK_REG);
	writel(fpga_mask_reg & ~mask_bits, data->bridge + IRQ_MASK_REG);
}

static void ts7800_irq_unmask(struct irq_data *d)
{
	struct ts7800_irq_data *data = irq_data_get_irq_chip_data(d);
	u32 fpga_mask_reg = readl(data->base + IRQ_MASK_REG);
	u32 mask_bits = 1 << d->hwirq;

	writel(fpga_mask_reg | mask_bits, data->base + IRQ_MASK_REG);
	writel(fpga_mask_reg | mask_bits, data->bridge + IRQ_MASK_REG);
}

static int ts7800_irqdomain_map(struct irq_domain *d, unsigned int irq,
				irq_hw_number_t hwirq)
{
	struct ts7800_irq_data *data = d->host_data;

	irq_set_chip_and_handler(irq, &data->irq_chip, handle_simple_irq);
	irq_set_chip_data(irq, data);
	irq_set_noprobe(irq);

	return 0;
}

static const struct irq_domain_ops ts7800_ic_ops = {
	.map = ts7800_irqdomain_map,
	.xlate = irq_domain_xlate_onecell,
};

static void ts7800_ic_chained_handle_irq(struct irq_desc *desc)
{
	struct ts7800_irq_data *data = irq_desc_get_handler_data(desc);
	struct irq_chip *chip = irq_desc_get_chip(desc);
	u32 mask_bits = readl(data->base + IRQ_STATUS_REG);
	u32 status = mask_bits;

	chained_irq_enter(chip, desc);

	if (unlikely(status == 0)) {
		handle_bad_irq(desc);
		goto out;
	}

	do {
		unsigned int bit = __ffs(status);
		int irq = irq_find_mapping(data->domain, bit);

		status &= ~(1 << bit);

		if (irq && (mask_bits & BIT(bit))) {
			u32 x32 = readl(data->bridge);

			generic_handle_irq(irq);

			x32 &= ~(mask_bits & BIT(bit));
			writel(x32, data->bridge);
		}
	} while (status);

out:
	chained_irq_exit(chip, desc);
}

static int ts7800_ic_probe(struct platform_device *pdev)
{
	struct ts7800_irq_data *data = NULL;
	struct irq_chip *irq_chip = NULL;
	struct resource *mem_res = NULL, *brdg_res = NULL, *irq_res = NULL;
	unsigned int irqdomain;
	int i, ret;

	data = devm_kzalloc(&pdev->dev, sizeof(*data), GFP_KERNEL);
	if (IS_ERR_OR_NULL(data)) {
		dev_err(&pdev->dev,
			"Failed to allocate TS7800 data, error %ld \n",
			PTR_ERR(data));
		ret = PTR_ERR(data);
		goto devm_kzalloc_err;
	}

	mem_res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "ts_irqc");
	if (IS_ERR_OR_NULL(mem_res)) {
		dev_err(&pdev->dev,
			"Failed to get platform memory resource, error %ld \n",
			PTR_ERR(mem_res));
		ret = PTR_ERR(mem_res);
		goto pltfrm_get_res_mem_err;
	}

	data->base = devm_ioremap_resource(&pdev->dev, mem_res);
	if (IS_ERR_OR_NULL(data->base)) {
		dev_err(&pdev->dev,
			"Failed to IO map mem-region %s, error %ld \n",
			mem_res->name, PTR_ERR(data->base));
		ret = PTR_ERR(data->base);
		goto devm_ioremap_res_mem_err;
	}

	brdg_res =
		platform_get_resource_byname(pdev, IORESOURCE_MEM, "ts_bridge");
	if (IS_ERR_OR_NULL(brdg_res)) {
		dev_err(&pdev->dev,
			"Failed to get platform bridge resource, error %ld \n",
			PTR_ERR(brdg_res));
		ret = PTR_ERR(brdg_res);
		goto pltfrm_get_res_brdg_err;
	}

	data->bridge = devm_ioremap_resource(&pdev->dev, brdg_res);
	if (IS_ERR_OR_NULL(data->bridge)) {
		dev_err(&pdev->dev,
			"Failed to IO map bridge-region %s, error %ld \n",
			mem_res->name, PTR_ERR(data->bridge));
		ret = PTR_ERR(data->bridge);
		goto devm_ioremap_res_brdge_err;
	}

	irq_res = platform_get_resource_byname(pdev, IORESOURCE_IRQ,
					       "ts_irqc_parent");
	if (IS_ERR_OR_NULL(irq_res)) {
		dev_err(&pdev->dev,
			"Failed to get platform parent irq resource, error %ld \n",
			PTR_ERR(irq_res));
		ret = PTR_ERR(irq_res);
		goto pltfrm_get_res_irq_err;
	}

	data->mpp7_virq = irq_res->start;
	writel(0x0, data->base + IRQ_MASK_REG);
	writel(0x0, data->bridge + IRQ_MASK_REG);
	writel(0x0, data->bridge);

	irq_chip = &data->irq_chip;
	irq_chip->name = dev_name(&pdev->dev);
	irq_chip->irq_mask = ts7800_irq_mask;
	irq_chip->irq_unmask = ts7800_irq_unmask;

	data->domain = irq_domain_add_linear(pdev->dev.of_node, TS7800_IRQ_NUM,
					     &ts7800_ic_ops, data);
	if (IS_ERR_OR_NULL(data->domain)) {
		dev_err(&pdev->dev, "cannot add IRQ domain\n");
		ret = PTR_ERR(data->domain);
		goto irq_domain_add_linear_err;
	}

	for (i = 0; i < ARRAY_SIZE(enabled_mappings); ++i) {
		irqdomain =
			irq_create_mapping(data->domain, enabled_mappings[i]);
	}

	irq_set_chained_handler_and_data(data->mpp7_virq,
					 ts7800_ic_chained_handle_irq, data);

	irq_set_status_flags(data->mpp7_virq,
			     IRQ_DISABLE_UNLAZY | IRQ_LEVEL | IRQ_NOPROBE);

	platform_set_drvdata(pdev, data);

	return 0;

irq_domain_add_linear_err:
pltfrm_get_res_irq_err:
	devm_iounmap(&pdev->dev, data->bridge);

devm_ioremap_res_brdge_err:
pltfrm_get_res_brdg_err:
	devm_iounmap(&pdev->dev, data->base);

devm_ioremap_res_mem_err:
pltfrm_get_res_mem_err:
	devm_kfree(&pdev->dev, data);

devm_kzalloc_err:
	return ret;
}

static int ts7800_ic_remove(struct platform_device *pdev)
{
	struct ts7800_irq_data *data = platform_get_drvdata(pdev);
	int i;

	if (!IS_ERR_OR_NULL(data)) {
		irq_set_chained_handler_and_data(data->mpp7_virq, NULL, NULL);

		for (i = 0; i < ARRAY_SIZE(enabled_mappings); ++i)
			irq_dispose_mapping(irq_find_mapping(
				data->domain, enabled_mappings[i]));

		irq_domain_remove(data->domain);
	}

	return 0;
}

static const struct platform_device_id ts7800v1_ic_ids[] = {
	{
		.name = DRIVER_NAME,
	},
	{
		/* sentinel */
	}
};

MODULE_DEVICE_TABLE(platform, ts7800v1_ic_ids);

static struct platform_driver ts7800_ic_driver = {
	.probe  = ts7800_ic_probe,
	.remove = ts7800_ic_remove,
	.id_table	= ts7800v1_ic_ids,
	.driver = {
		.name = DRIVER_NAME,
		.probe_type = PROBE_PREFER_ASYNCHRONOUS,
	},
};
module_platform_driver(ts7800_ic_driver);

MODULE_ALIAS("platform:ts7800-irqc");
MODULE_DESCRIPTION("TS-7800v1 FPGA based IRQ controller Driver");
MODULE_AUTHOR("Firas Ashkar <firas.ashkar@savoirfairelinux.com>");
MODULE_LICENSE("GPL v2");
