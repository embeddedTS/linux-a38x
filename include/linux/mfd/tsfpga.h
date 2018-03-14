#ifndef __TSFPGA_H__
#define __TSFPGA_H__

#include <linux/regmap.h>

struct tsfpga_res
{
	struct device *dev;
	struct regmap *regmap;
	void __iomem *base;
	int irq;
};

static const struct regmap_config tsfpga_regmap_config = {
	.reg_bits = 32,
	.val_bits = 32,
	.reg_stride = 4,
};

#endif
