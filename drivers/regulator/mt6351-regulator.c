// SPDX-License-Identifier: GPL-2.0
/*
 * Minimal MediaTek MT6351 regulator driver: VLDO28 only.
 *
 * VLDO28 is a fixed 2.8 V rail (stock DTB: min = max = 2800000 uV) that
 * supplies the Meizu M5 Note touch panel. Its enable is bit 1 of PMIC
 * register 0x0aa2 (read from the stock kernel regulator table, flag 2165).
 * The 264 us enable delay is the stock "regulator-enable-ramp-delay".
 *
 * Bound by the "mt6351-regulator" cell of mt6397-core.c, so the PMIC node
 * needs a child with compatible = "mediatek,mt6351-regulator".
 */
#include <linux/err.h>
#include <linux/mfd/mt6397/core.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/regulator/driver.h>

#define MT6351_VLDO28_EN_REG	0x0aa2
#define MT6351_VLDO28_EN_MASK	BIT(1)

static const struct regulator_ops mt6351_fixed_ops = {
	.enable = regulator_enable_regmap,
	.disable = regulator_disable_regmap,
	.is_enabled = regulator_is_enabled_regmap,
};

static const struct regulator_desc mt6351_vldo28_desc = {
	.name = "vldo28",
	.of_match = "ldo_vldo28",
	.ops = &mt6351_fixed_ops,
	.type = REGULATOR_VOLTAGE,
	.owner = THIS_MODULE,
	.n_voltages = 1,
	.fixed_uV = 2800000,
	.enable_reg = MT6351_VLDO28_EN_REG,
	.enable_mask = MT6351_VLDO28_EN_MASK,
	.enable_time = 264,
};

static int mt6351_regulator_probe(struct platform_device *pdev)
{
	struct mt6397_chip *chip = dev_get_drvdata(pdev->dev.parent);
	struct regulator_config config = { };
	struct regulator_dev *rdev;

	if (!chip || !chip->regmap)
		return -ENODEV;

	config.dev = &pdev->dev;
	config.regmap = chip->regmap;

	rdev = devm_regulator_register(&pdev->dev, &mt6351_vldo28_desc, &config);
	if (IS_ERR(rdev))
		return dev_err_probe(&pdev->dev, PTR_ERR(rdev),
				     "failed to register vldo28\n");

	return 0;
}

static struct platform_driver mt6351_regulator_driver = {
	.driver = {
		.name = "mt6351-regulator",
	},
	.probe = mt6351_regulator_probe,
};
module_platform_driver(mt6351_regulator_driver);

MODULE_ALIAS("platform:mt6351-regulator");
MODULE_DESCRIPTION("Minimal MT6351 regulator driver (VLDO28 only)");
MODULE_LICENSE("GPL");
