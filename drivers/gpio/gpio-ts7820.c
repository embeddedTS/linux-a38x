/*
 * GPIO driver for the TS-7820
 *
 * Copyright (c) 2018 - Technologic Systems
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2. This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */

#include <linux/gpio/driver.h>
#include <linux/module.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>

/*
 * 0: write=OE set, read=current OE
 * 4: write=OE clr, read=Model reg in upper 16, fpga rev in lower 16
 * 8: write=output set, read=current output
 * c: write=output clr, read=current inputs
 * This could almost be handled by the bgpio driver,
 * but no support for a separate OE_CLR/OE_SET
 */

#define TS7820_IN	0x0c
#define TS7820_OUT_SET	0x08
#define TS7820_OUT_CLR	0x0c
#define TS7820_OE_IN	0x00
#define TS7820_OE_SET	0x00
#define TS7820_OE_CLR	0x04

struct ts7820_gpio_chip {
	struct gpio_chip gpio;
	void __iomem *base;
};

static void ts7820_gpio_set(struct gpio_chip *chip, unsigned int pin, int val)
{
	struct ts7820_gpio_chip *gc = gpiochip_get_data(chip);

	if (val)
		writel(BIT(pin), gc->base + TS7820_OUT_SET);
	else
		writel(BIT(pin), gc->base + TS7820_OUT_CLR);
}

static void ts7820_gpio_set_multiple(struct gpio_chip *chip,
				     unsigned long *mask,
				     unsigned long *bits)
{
	struct ts7820_gpio_chip *gc = gpiochip_get_data(chip);

	writel(*mask & *bits, gc->base + TS7820_OUT_SET);
	writel(*mask & (~*bits), gc->base + TS7820_OUT_CLR);
}

static int ts7820_gpio_get(struct gpio_chip *chip, unsigned int pin)
{
	struct ts7820_gpio_chip *gc = gpiochip_get_data(chip);

	return !!(readl(gc->base + TS7820_IN) & BIT(pin));
}

static int ts7820_gpio_direction_input(struct gpio_chip *chip,
					unsigned int pin)
{
	struct ts7820_gpio_chip *gc = gpiochip_get_data(chip);

	writel(BIT(pin), gc->base + TS7820_OE_CLR);
	return 0;
}

static int ts7820_gpio_direction_output(struct gpio_chip *chip,
					unsigned int pin, int val)
{
	struct ts7820_gpio_chip *gc = gpiochip_get_data(chip);

	ts7820_gpio_set(chip, pin, val);
	writel(BIT(pin), gc->base + TS7820_OE_SET);
	return 0;
}

static int ts7820_gpio_direction_get(struct gpio_chip *chip,
				     unsigned int pin)
{
	struct ts7820_gpio_chip *gc = gpiochip_get_data(chip);

	return !!(readl(gc->base + TS7820_OE_IN) & BIT(pin));
}

static const struct gpio_chip ts7820_chip = {
	.label			= "ts7820-gpio",
	.direction_input	= ts7820_gpio_direction_input,
	.direction_output	= ts7820_gpio_direction_output,
	.get_direction		= ts7820_gpio_direction_get,
	.set			= ts7820_gpio_set,
	.set_multiple		= ts7820_gpio_set_multiple,
	.get			= ts7820_gpio_get,
	.base			= -1,
	.ngpio			= 32,
	.owner			= THIS_MODULE,
};

static int ts7820_gpio_probe(struct platform_device *pdev)
{
	struct ts7820_gpio_chip *gc;
	struct resource *res;

	gc = devm_kzalloc(&pdev->dev, sizeof(struct ts7820_gpio_chip),
			  GFP_KERNEL);
	if (!gc)
		return -ENOMEM;

	gc->gpio = ts7820_chip;
	gc->gpio.parent = &pdev->dev;
	platform_set_drvdata(pdev, gc);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	gc->base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(gc->base))
		return PTR_ERR(gc->base);

	return devm_gpiochip_add_data(&pdev->dev, &gc->gpio, gc);
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
