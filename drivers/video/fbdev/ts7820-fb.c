/*
 * FB driver for the TS-7820
 * based on drivers/video/fbdev/ocfb.c
 *
 * Copyright (c) 2018 - Technologic Systems
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2. This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */

#include <linux/module.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/platform_data/simplefb.h>
#include <linux/platform_device.h>
#include <linux/dma-mapping.h>

/* The TS-7820 supports a framebuffer through the FPGA.  The FPGA reads continuously from a 4*240*320 
 * from a contiguous block of physical memory, and outputs this to the LCD.  
 * This driver must allocate the contiguous 2MB region and write the physical address
 * to the FPGA, and this must be 4k aligned. 
 */

#define PSEUDO_PALETTE_SIZE 16
struct ts7820_fb_priv {
	u32 palette[PSEUDO_PALETTE_SIZE];
	struct fb_info info;
	void __iomem *regs;
	int width;
	int height;
	dma_addr_t fb_phys;
	void __iomem *fb_virt;
	struct simplefb_platform_data mode;
};

static int tsfb_setcolreg(u_int regno, u_int red, u_int green, u_int blue,
			      u_int transp, struct fb_info *info)
{
	u32 *pal = info->pseudo_palette;
	u32 cr = red >> (16 - info->var.red.length);
	u32 cg = green >> (16 - info->var.green.length);
	u32 cb = blue >> (16 - info->var.blue.length);
	u32 value;

	if (regno >= PSEUDO_PALETTE_SIZE)
		return -EINVAL;

	value = (cr << info->var.red.offset) |
		(cg << info->var.green.offset) |
		(cb << info->var.blue.offset);
	if (info->var.transp.length > 0) {
		u32 mask = (1 << info->var.transp.length) - 1;
		mask <<= info->var.transp.offset;
		value |= mask;
	}
	pal[regno] = value;

	return 0;
}

static struct fb_ops tsfb_ops = {
	.owner		= THIS_MODULE,
	.fb_setcolreg	= tsfb_setcolreg,
	.fb_fillrect	= cfb_fillrect,
	.fb_copyarea	= cfb_copyarea,
	.fb_imageblit	= cfb_imageblit,
};

static const char reg_resnmae[] = "TS7820FB";
static const char simplefb_resname[] = "TS7820FB";
static const char simplefb_mode[] = "a8r8g8b8";
static const struct simplefb_format formats[] = SIMPLEFB_FORMATS;

static int ts7820_fb_probe(struct platform_device *pdev)
{
	struct resource *res;
	struct device_node *np = pdev->dev.of_node;
	struct ts7820_fb_priv *priv;
	int fbsize, ret;

	priv = devm_kzalloc(&pdev->dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	platform_set_drvdata(pdev, priv);

	ret = of_property_read_u32(np, "width", &priv->width);
	if (ret) {
		dev_err(&pdev->dev, "Can't parse width property\n");
		return ret;
	}

	ret = of_property_read_u32(np, "height", &priv->height);
	if (ret) {
		dev_err(&pdev->dev, "Can't parse height property\n");
		return ret;
	}

	/* This is the address we need to inform of the physical address of
	 * the allocated framebuffer memory */
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	priv->regs = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(priv->regs))
		return PTR_ERR(priv->regs);

	fbsize = priv->width * priv->height * 2;
	priv->fb_virt = dma_alloc_coherent(&pdev->dev, PAGE_ALIGN(fbsize),
					    &priv->fb_phys, GFP_KERNEL);
	if (!priv->fb_virt) {
		dev_err(&pdev->dev,
			"Frame buffer memory allocation failed\n");
		return -ENOMEM;
	}

	priv->info.fbops = &tsfb_ops;
	priv->info.device = &pdev->dev;
	priv->info.par = priv;

	priv->info.fix.smem_start = priv->fb_phys;
	priv->info.fix.smem_len = fbsize;
	priv->info.screen_base = priv->fb_virt;
	priv->info.fix.line_length = 2 * priv->width;
	priv->info.fix.type = FB_TYPE_PACKED_PIXELS;
	priv->info.fix.visual = FB_VISUAL_TRUECOLOR;
	priv->info.fix.accel = FB_ACCEL_NONE;

	priv->info.var.width = -1;
	priv->info.var.height = -1;
	priv->info.var.activate = FB_ACTIVATE_NOW;
	priv->info.var.vmode = FB_VMODE_NONINTERLACED;
	priv->info.var.xres = priv->width;
	priv->info.var.yres = priv->height;
	priv->info.var.xres_virtual = priv->width;
	priv->info.var.yres_virtual = priv->height;
	
	priv->info.var.bits_per_pixel = 16;
	priv->info.var.red.offset = 11;
	priv->info.var.red.length = 5;
	priv->info.var.green.offset = 5;
	priv->info.var.green.length = 6;
	priv->info.var.blue.offset = 0;
	priv->info.var.blue.length = 5;

	/*priv->info.var.bits_per_pixel = 32;
	priv->info.var.red.offset = 19;
	priv->info.var.red.length = 5;
	priv->info.var.green.offset = 10;
	priv->info.var.green.length = 6;
	priv->info.var.blue.offset = 3;
	priv->info.var.blue.length = 5;*/

	priv->info.var.transp.offset = 0;
	priv->info.var.transp.length = 0;
	priv->info.pseudo_palette = priv->palette;

	/* Register framebuffer */
	ret = register_framebuffer(&priv->info);
	if (ret) {
		dev_err(&pdev->dev, "Framebuffer registration failed\n");
		return 1;
	}

	writel(priv->fb_phys, priv->regs);

	return 0;
}

static const struct of_device_id ts7820_fb_of_match[] = {
	{ .compatible = "technologic,ts7820-fb", },
	{},
};
MODULE_DEVICE_TABLE(of, ts7820_fb_of_match);

static struct platform_driver ts7820_fb_driver = {
	.probe = ts7820_fb_probe,
	.driver = {
		.name = "ts7820-fb",
		.of_match_table = ts7820_fb_of_match,
		.suppress_bind_attrs = true,
	},
};
module_platform_driver(ts7820_fb_driver);

MODULE_AUTHOR("Mark Featherston <mark@embeddedarm.com>");
MODULE_DESCRIPTION("TS-7820 LCD driver");
MODULE_LICENSE("GPL v2");
