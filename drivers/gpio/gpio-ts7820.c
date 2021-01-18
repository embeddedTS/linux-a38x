/*
 * GPIO driver for the TS-7820
 *
 * Copyright (c) 2018 - Technologic Systems
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2. This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */

#include <linux/gpio.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/pm_runtime.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/spinlock.h>

/* Read Decodes */
#define TS7820_OE_IN		0x00
#define TS7820_OUT_DATA		0x08
#define TS7820_IN		0x0C
/*#define TS7820_IRQ_EN		0x10 */
#define TS7820_IRQSTATUS	0x14

/* Write Decodes */
#define TS7820_OE_SET		0x00
#define TS7820_OE_CLR		0x04
#define TS7820_DAT_SET		0x08
#define TS7820_DAT_CLR		0x0C
#define TS7820_IRQ_EN		0x10
#define TS7820_IRQ_NPOL		0x18 /* 1 = active low */

struct ts7820_gpio_priv {
	void __iomem *base;
	struct device *dev;
	struct gpio_chip gpio_chip;
	struct irq_chip irq_chip;
	spinlock_t lock;
	uint32_t irqen;
	uint32_t npol;
	unsigned int irq_parent;
};

static void ts7820_gpio_set(struct gpio_chip *chip, unsigned int pin, int val)
{
	struct ts7820_gpio_priv *priv = gpiochip_get_data(chip);

	if (val)
		writel(BIT(pin), priv->base + TS7820_DAT_SET);
	else
		writel(BIT(pin), priv->base + TS7820_DAT_CLR);
}

static void ts7820_gpio_set_multiple(struct gpio_chip *chip,
				     unsigned long *mask,
				     unsigned long *bits)
{
	struct ts7820_gpio_priv *priv = gpiochip_get_data(chip);

	writel(*mask & *bits, priv->base + TS7820_DAT_SET);
	writel(*mask & (~*bits), priv->base + TS7820_DAT_CLR);
}

static int ts7820_gpio_get(struct gpio_chip *chip, unsigned int pin)
{
	struct ts7820_gpio_priv *priv = gpiochip_get_data(chip);

	return !!(readl(priv->base + TS7820_IN) & BIT(pin));
}

static int ts7820_gpio_direction_input(struct gpio_chip *chip,
					unsigned int pin)
{
	struct ts7820_gpio_priv *priv = gpiochip_get_data(chip);

	writel(BIT(pin), priv->base + TS7820_OE_CLR);
	return 0;
}

static int ts7820_gpio_direction_output(struct gpio_chip *chip,
					unsigned int pin, int val)
{
	struct ts7820_gpio_priv *priv = gpiochip_get_data(chip);

	ts7820_gpio_set(chip, pin, val);
	writel(BIT(pin), priv->base + TS7820_OE_SET);
	return 0;
}

static int ts7820_gpio_direction_get(struct gpio_chip *chip,
				     unsigned int pin)
{
	struct ts7820_gpio_priv *priv = gpiochip_get_data(chip);

	return !(readl(priv->base + TS7820_OE_IN) & BIT(pin));
}

static void gpio_ts7820_irq_disable(struct irq_data *d)
{
	struct gpio_chip *gc = irq_data_get_irq_chip_data(d);
	struct ts7820_gpio_priv *priv = gpiochip_get_data(gc);
	unsigned long flags;

	spin_lock_irqsave(&priv->lock, flags);
	priv->irqen &= ~BIT(irqd_to_hwirq(d));
	writel(priv->irqen, priv->base + TS7820_IRQ_EN);
	spin_unlock_irqrestore(&priv->lock, flags);
	dev_dbg(priv->dev, "irqen set to 0x%X\n", priv->irqen);
}

static void gpio_ts7820_irq_enable(struct irq_data *d)
{
	struct gpio_chip *gc = irq_data_get_irq_chip_data(d);
	struct ts7820_gpio_priv *priv = gpiochip_get_data(gc);
	unsigned long flags;

	spin_lock_irqsave(&priv->lock, flags);
	priv->irqen |= BIT(irqd_to_hwirq(d));
	writel(priv->irqen, priv->base + TS7820_IRQ_EN);
	spin_unlock_irqrestore(&priv->lock, flags);
	dev_dbg(priv->dev, "irqen set to 0x%X\n", priv->irqen);
}

static int gpio_ts7820_irq_set_type(struct irq_data *d, unsigned int type)
{
	struct gpio_chip *gc = irq_data_get_irq_chip_data(d);
	struct ts7820_gpio_priv *priv = gpiochip_get_data(gc);
	unsigned int hwirq = irqd_to_hwirq(d);
	int ret = 0;
	unsigned long flags;

	dev_dbg(priv->dev, "set_type irq %d to %d\n", hwirq, type);

	spin_lock_irqsave(&priv->lock, flags);
	priv->npol &= ~hwirq;

	switch (type & IRQ_TYPE_SENSE_MASK) {
	case IRQ_TYPE_LEVEL_HIGH:
		priv->npol &= ~(1 << hwirq);
		writel(priv->npol, priv->base + TS7820_IRQ_NPOL);
		break;
	case IRQ_TYPE_LEVEL_LOW:
		priv->npol |= (1 << hwirq);
		writel(priv->npol, priv->base + TS7820_IRQ_NPOL);
		break;
	case IRQ_TYPE_EDGE_FALLING:
	default:
		ret = -EINVAL;
	}
	spin_unlock_irqrestore(&priv->lock, flags);

	dev_dbg(priv->dev, "set_type npol 0x%X\n", priv->npol);

	return ret;
}

static irqreturn_t gpio_ts7820_irq_handler(int irq, void *dev_id)
{
	struct ts7820_gpio_priv *priv = dev_id;
	unsigned long status, irqen;
	unsigned int irqs_handled = 0;
	u32 bit, virq;
	unsigned long flags;

	spin_lock_irqsave(&priv->lock, flags);
	status = readl(priv->base + TS7820_IRQSTATUS);
	irqen = priv->irqen;
	spin_unlock_irqrestore(&priv->lock, flags);

	status &= irqen;
	for_each_set_bit(bit, &status, 32) {
		virq = irq_find_mapping(priv->gpio_chip.irq.domain, bit);
		if (virq) {
			dev_dbg(priv->dev, "Dispatching IRQ: %d\n",
				virq);
			generic_handle_irq(virq);
			irqs_handled++;
		}
	}

	dev_dbg(priv->dev, "raw status: 0x%lX\n", status);
	dev_dbg(priv->dev, "raw irqen: 0x%lX\n", irqen);

	return IRQ_HANDLED;
}

static int ts7820_gpio_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	const char *name = dev_name(&pdev->dev);
	struct ts7820_gpio_priv *priv;
	struct resource *io, *irq;
	struct gpio_chip *gpio_chip;
	struct irq_chip *irq_chip;
	struct gpio_irq_chip *irqc;
	int err;

	priv = devm_kzalloc(dev, sizeof(struct ts7820_gpio_priv),
			  GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	irq_chip = devm_kzalloc(dev, sizeof(*irq_chip), GFP_KERNEL);
	if (!irq_chip)
		return -ENOMEM;

	priv->dev = dev;
	platform_set_drvdata(pdev, priv);
	pm_runtime_enable(dev);

	irq = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
	if (!irq) {
		dev_err(dev, "failed to find IRQ\n");
		return -ENXIO;
	}
	priv->base = devm_platform_ioremap_resource(pdev, 0);
	if (!priv->base) {
		dev_err(dev, "failed to remap I/O memory\n");
		return -ENXIO;
	}

	gpio_chip = &priv->gpio_chip;
	gpio_chip->label = name;
	gpio_chip->direction_input = ts7820_gpio_direction_input,
	gpio_chip->direction_output = ts7820_gpio_direction_output,
	gpio_chip->get_direction = ts7820_gpio_direction_get,
	gpio_chip->set = ts7820_gpio_set,
	gpio_chip->set_multiple = ts7820_gpio_set_multiple,
	gpio_chip->get = ts7820_gpio_get,
	gpio_chip->base = -1,
	gpio_chip->ngpio = 32,
	gpio_chip->owner = THIS_MODULE,
	gpio_chip->parent = dev;
	spin_lock_init(&priv->lock);

	irq_chip->name = name;
	irq_chip->parent_device = dev;
	irq_chip->irq_mask = gpio_ts7820_irq_disable;
	irq_chip->irq_unmask = gpio_ts7820_irq_enable;
	irq_chip->irq_set_type = gpio_ts7820_irq_set_type;
	irq_chip->flags	= IRQCHIP_SET_TYPE_MASKED;

	irqc = &gpio_chip->irq;
	irqc->chip = irq_chip;
	irqc->default_type = IRQ_TYPE_NONE;
	irqc->num_parents = 1;
	irqc->parents = &irq->start;
	irqc->handler = handle_bad_irq;

	err = gpiochip_add_data(gpio_chip, priv);
	if (err) {
		dev_err(dev, "failed to add GPIO controller\n");
		return err;
	}
	if (devm_request_irq(dev, irq->start, gpio_ts7820_irq_handler,
			     IRQF_SHARED, name, priv)) {
		dev_err(dev, "failed to request IRQ\n");
		return -ENOENT;
	}

	return 0;
}

static const struct of_device_id ts7820_gpio_of_match[] = {
	{ .compatible = "technologic,ts7820-gpio", },
	{},
};
MODULE_DEVICE_TABLE(of, ts7820_gpio_of_match);

static struct platform_driver ts7820_gpio_driver = {
	.probe = ts7820_gpio_probe,
	.driver = {
		.name = "ts7820-gpio",
		.of_match_table = ts7820_gpio_of_match,
		.suppress_bind_attrs = true,
	},
};
module_platform_driver(ts7820_gpio_driver);

MODULE_AUTHOR("Mark Featherston <mark@embeddedarm.com>");
MODULE_DESCRIPTION("TS-7820 FPGA GPIO driver");
MODULE_LICENSE("GPL v2");
