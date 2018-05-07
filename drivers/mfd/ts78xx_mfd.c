#include <linux/delay.h>
#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/kernel.h>
#include <linux/mfd/core.h>
#include <linux/mfd/tsfpga.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/pci.h>
#include <linux/regmap.h>
#include <linux/slab.h>

#define TS7820_GPIO_BANK0	0x0
#define TS7820_GPIO_SZ		0x10

static struct resource tsgpio_resources[] = {
	{
		.start = TS7820_GPIO_BANK0,
		.end = TS7820_GPIO_SZ,
		.flags = IORESOURCE_MEM,
	},
};

static const struct mfd_cell tsfpga_sysreg_cells[] = {
	{
		.name = "ts7820-gpio",
		.resources = &tsgpio_resources[0],
		.num_resources = 1,
		.of_compatible = "technologic,ts7820-gpio",
	},
};

static int tsfpga_pci_probe(struct pci_dev *pdev,
			   const struct pci_device_id *ent)
{
	struct tsfpga_res *priv;
	struct device_node *np;
	int err = 0;
	u32 reg;
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

	/* On the TS-78XX, the FPGA is always connected to BUS 3.
	 * If it is not BUS 3, it might be the TS-MINI PCIe
	 * device instead.  Only attach if it is bus 3.
	 */
	if (pdev->bus->number != 3) {
		err = -ENODEV;
		goto out_pci_disable_device;
	}

	pci_set_drvdata(pdev, priv);
	priv->dev = &pdev->dev;

	np = of_find_compatible_node(NULL, NULL, "technologic,ts78xx-mfd");
	if (np == NULL) {
		dev_err(&pdev->dev, "Couldn't find the device tree node!\n");
		reg = -ENODEV;
		goto out_pci_disable_device;
	}

	pdev->dev.of_node = of_node_get(np);

	/* We only use BAR0 */
	start = pci_resource_start(pdev, 0);
	end = pci_resource_end(pdev, 0);
	if (!start || !end) {
		err = -ENODEV;
		goto out_pci_disable_device;
	}

	/* Map to just read rev */
	priv->base = ioremap_nocache(start, TS7820_GPIO_SZ);
	if (IS_ERR(priv->base)) {
		err = PTR_ERR(priv->base);
		dev_err(&pdev->dev, "ioremap failed for fpga mem\n");
		goto out_pci_disable_device;
	}
	reg = readl(priv->base + 0x4);
	iounmap(priv->base);

	dev_info(&pdev->dev, "Detected model 0x%X, FPGA REV %d\n",
		reg >> 16, reg & 0xffff);

	devm_mfd_add_devices(&pdev->dev, -1,
			     tsfpga_sysreg_cells,
			     ARRAY_SIZE(tsfpga_sysreg_cells),
			     &pdev->resource[0],
			     0, NULL);

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
	{PCI_DEVICE(0x1172, 0x0004), 0},	/* TS-7820/TS-7840 */
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
