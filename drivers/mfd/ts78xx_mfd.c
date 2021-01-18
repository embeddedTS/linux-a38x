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

struct tsfpga_priv {
	struct device	    *dev;
	void __iomem	    *base;
	int		    irq_parent;
};

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

static int tsfpga_pci_probe(struct pci_dev *pdev,
			   const struct pci_device_id *ent)
{
	struct tsfpga_priv *priv;
	struct device_node *np;
	int err = 0;
	unsigned long start, end;

	priv = devm_kzalloc(&pdev->dev, sizeof(struct tsfpga_priv), GFP_KERNEL);
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

	err = pci_alloc_irq_vectors(pdev, 1, 1, PCI_IRQ_MSI | PCI_IRQ_LEGACY);
	if (err < 0) {
		err = -ENODEV;
		goto out_pci_disable_device;
	}
	priv->irq_parent = pci_irq_vector(pdev, 0);

	dev_info(priv->dev, "FPGA base: 0x%px\n", (void *)start);
	dev_info(priv->dev, "FPGA IRQ: %d\n", priv->irq_parent);

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
