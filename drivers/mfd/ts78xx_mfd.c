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

static struct mfd_cell tsfpga_sysreg_cells[] = {
	{
		.name = "ts7800v2-gpio",
		.of_compatible = "technologic,ts7800v2-gpio",
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

	dev_info(&pdev->dev, "Driver loaded\n");
	priv = devm_kzalloc(&pdev->dev, sizeof(struct tsfpga_res), GFP_KERNEL);
	if (!priv){
		return -ENOMEM;
	}

	if (pci_enable_device(pdev)) {
		err = -ENODEV;
		goto out;
	}

	/* On the TS-78XX, the FPGA is always connected to BUS 3.  If it is not BUS 3, it might
	 * be the TS-MINI PCIe device instead.  Only attach if it is bus 3. */
	if(pdev->bus->number != 3)
	{
		err = -ENODEV;
		goto out_pci_disable_device;
	}

	err = pci_request_regions(pdev, "tsfpga");
	if (err)
		goto out_pci_disable_device;

	pci_set_drvdata(pdev, priv);
	priv->dev = &pdev->dev;

	np = of_find_compatible_node(NULL, NULL, "technologic,ts78xx-mfd");
	if(np == NULL) {
		dev_err(&pdev->dev, "Couldn't find the device tree node!\n");
		reg = -ENODEV;
		goto out_pci_disable_device;
	}

	pdev->dev.of_node = of_node_get(np);

	/* We only use BAR2 */
	start = pci_resource_start(pdev, 2);
	end = pci_resource_start(pdev, 2);
	if (!start || !end) {
		err = -ENODEV;
		goto out_pci_disable_device;
	}

	priv->base = devm_ioremap_nocache(&pdev->dev, start, 1024*1024);
	if (IS_ERR(priv->base)) {
		err = PTR_ERR(priv->base);
		dev_err(&pdev->dev, "ioremap failed for fpga mem\n");
		goto out_pci_disable_device;
	}

	priv->regmap = devm_regmap_init_mmio(&pdev->dev,priv->base,	&tsfpga_regmap_config);
	if (IS_ERR(priv->regmap)) {
		dev_err(&pdev->dev, "failed to initialise regmap\n");
		err = PTR_ERR(priv->regmap);
		goto out_pci_disable_device;
	}

	reg = readl(priv->base);

	dev_info(&pdev->dev, "Detected model 0x%X, FPGA REV %d\n", 
		reg >> 8, reg & 0xff);

	return devm_mfd_add_devices(&pdev->dev, PLATFORM_DEVID_NONE, tsfpga_sysreg_cells,
			ARRAY_SIZE(tsfpga_sysreg_cells), NULL, 0, NULL);

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
	{PCI_DEVICE(0x1172, 0x0004), 0},	/* TS-7840 */
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
		printk(KERN_ERR "Unable to initialize PCI module\n");
		return ret;
	}
	return ret;
}
subsys_initcall(tsfpga_init);

MODULE_DESCRIPTION("TS-78XX Series FPGA MFD driver");
MODULE_LICENSE("GPL v2");
