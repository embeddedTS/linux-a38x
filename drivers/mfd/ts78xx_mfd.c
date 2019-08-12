#include <linux/delay.h>
#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/irq.h>
#include <linux/irqchip.h>
#include <linux/irqchip/chained_irq.h>
#include <linux/irqdomain.h>
#include <linux/kernel.h>
#include <linux/mfd/core.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/pci.h>
#include <linux/slab.h>

struct tsfpga_res {
	struct device	    *dev;
	void __iomem	    *base;
	struct irq_domain   *domain;
	struct irq_chip	    irq_chip;
};

static int ts7820_irqdomain_map(struct irq_domain *d, unsigned int irq,
				irq_hw_number_t hwirq)
{
	struct tsfpga_res *priv = d->host_data;

	dev_dbg(priv->dev, "Mapped IRQ IRQ %d\n", irq);

	irq_set_chip_and_handler(irq, &priv->irq_chip, handle_simple_irq);
	irq_set_chip_data(irq, priv);
	irq_set_noprobe(irq);

	return 0;
}

static const struct irq_domain_ops ts7820_ic_ops = {
	.map = ts7820_irqdomain_map,
	.xlate = irq_domain_xlate_onecell,
};

static void ts7820_handle_chained_irq(struct irq_desc *desc)
{
	struct tsfpga_res *priv = irq_desc_get_handler_data(desc);
	struct irq_chip *chip = irq_desc_get_chip(desc);
	u32 virq = irq_find_mapping(priv->domain, 0);

	dev_dbg(priv->dev, "Chained IRQ VIRQ %d\n", virq);

	chained_irq_enter(chip, desc);
	generic_handle_irq(virq);
	chained_irq_exit(chip, desc);
}

static void tsfpga_pcie_add_ranges(struct pci_dev *pdev, struct device_node *np)
{
	struct property *prop;
	u32 start, end;
	struct of_changeset ocs;
	uint32_t *val;

	start = pci_resource_start(pdev, 0);
	end = pci_resource_end(pdev, 0);

	prop = devm_kcalloc(&pdev->dev, 1, sizeof(*prop), GFP_KERNEL);
	val = devm_kcalloc(&pdev->dev, 1, sizeof(uint32_t)*3, GFP_KERNEL);
	prop->name = devm_kstrdup(&pdev->dev, "ranges", GFP_KERNEL);
	val[0] = 0x0;
	val[1] = cpu_to_be32(start);
	val[2] = cpu_to_be32(end - start);
	prop->length = sizeof(uint32_t) * 3;
	prop->value = val;

	of_changeset_init(&ocs);
	of_changeset_add_property(&ocs, np, prop);
	of_changeset_apply(&ocs);
}

static void ts7820_irq_mask(struct irq_data *d)
{
}

static void ts7820_irq_unmask(struct irq_data *d)
{
}

static int tsfpga_pci_probe(struct pci_dev *pdev,
			   const struct pci_device_id *ent)
{
	struct tsfpga_res *priv;
	struct device_node *np;
	struct irq_chip *irq_chip;
	int err = 0;
	unsigned long start, end;

	priv = devm_kzalloc(&pdev->dev, sizeof(struct tsfpga_res), GFP_KERNEL);
	if (!priv) {
		err = -ENOMEM;
		goto out;
	}

	if (pci_enable_device(pdev)) {
		err = -ENODEV;
		goto out;
	}

	pci_set_drvdata(pdev, priv);
	priv->dev = &pdev->dev;

	np = of_find_compatible_node(NULL, NULL, "technologic,ts78xx-mfd");
	if (np == NULL) {
		dev_err(&pdev->dev, "Couldn't find the device tree node!\n");
		err = -ENODEV;
		goto out_pci_disable_device;
	}
	pdev->dev.of_node = np;
	tsfpga_pcie_add_ranges(pdev, np);

	/* We only use BAR0 */
	start = pci_resource_start(pdev, 0);
	end = pci_resource_end(pdev, 0);
	if (!start || !end) {
		err = -ENODEV;
		goto out_pci_disable_device;
	}

	err = pci_alloc_irq_vectors(pdev, 1, 1, PCI_IRQ_MSI);
	if (err < 0) {
		err = -ENODEV;
		goto out_pci_disable_device;
	}
	priv->base = ioremap_nocache(start, 0x100);
	if (IS_ERR(priv->base)) {
		err = PTR_ERR(priv->base);
		dev_err(&pdev->dev, "ioremap failed for fpga mem\n");
		goto out_pci_disable_device;
	}

	dev_info(priv->dev, "FPGA base: 0x%p\n", (void *)start);

	irq_chip = &priv->irq_chip;
	irq_chip->name = pdev->dev.of_node->name;
	irq_chip->irq_mask = ts7820_irq_mask;
	irq_chip->irq_unmask = ts7820_irq_unmask;

	priv->domain = irq_domain_add_linear(np, 2, &ts7820_ic_ops, priv);
	if (!priv->domain) {
		dev_err(&pdev->dev, "cannot add IRQ domain\n");
		err = -ENOMEM;
		goto out_pci_disable_device;
	}

	irq_set_chained_handler_and_data(pci_irq_vector(pdev, 0),
		ts7820_handle_chained_irq, priv);

	devm_of_platform_populate(&pdev->dev);

	return 0;

out_pci_disable_device:
	pci_free_irq_vectors(pdev);
	pci_disable_device(pdev);
	pci_release_regions(pdev);

out:
	return err;
}

static void tsfpga_pci_remove(struct pci_dev *pdev)
{
}

static const struct pci_device_id tsfpga_pci_id_table[] = {
	{PCI_DEVICE(0x1e6d, 0x7840), 0},
	{0,}
};
MODULE_DEVICE_TABLE(pci, tsfpga_pci_id_table);

static struct pci_driver tsfpga_pci_driver = {
	.name = "ts78xx_mfd",
	.id_table = tsfpga_pci_id_table,
	.probe = tsfpga_pci_probe,
	.remove = tsfpga_pci_remove,
};

static int __init tsfpga_init(void)
{
	int ret;

	ret = pci_register_driver(&tsfpga_pci_driver);
	if (ret) {
		pr_err("Could not register pci driver\n");
		return ret;
	}
	return ret;
}
subsys_initcall(tsfpga_init);

MODULE_DESCRIPTION("TS-78XX Series FPGA MFD driver");
MODULE_LICENSE("GPL v2");
