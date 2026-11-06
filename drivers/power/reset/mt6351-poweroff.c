// SPDX-License-Identifier: GPL-2.0
/*
 * Power-off through the RTC block of the MediaTek MT6351 PMIC, over pwrap.
 * The sequence was read from the stock Meizu M5 Note kernel (rtc_bbpu_pwrdown).
 */
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pm.h>
#include <linux/reboot.h>
#include <linux/regmap.h>

#define MT6351_RTC_BBPU		0x4000
#define MT6351_RTC_WRTGR	0x403c
#define MT6351_BBPU_PWRDOWN	0x4309	/* key 0x43 | power-down bits */

static struct regmap *mt6351_map;

static void mt6351_power_off(void)
{
	int i;

	for (i = 0; i < 10; i++) {
		regmap_write(mt6351_map, MT6351_RTC_BBPU, MT6351_BBPU_PWRDOWN);
		regmap_write(mt6351_map, MT6351_RTC_WRTGR, 1);
		mdelay(100);
	}
	/* the stock kernel restarts if the PMIC did not cut the power */
	machine_restart(NULL);
}

static int mt6351_poweroff_probe(struct platform_device *pdev)
{
	mt6351_map = dev_get_regmap(pdev->dev.parent, NULL);
	if (!mt6351_map)
		return dev_err_probe(&pdev->dev, -ENODEV, "no PMIC regmap\n");

	pm_power_off = mt6351_power_off;
	dev_info(&pdev->dev, "registered power-off handler\n");
	return 0;
}

static const struct of_device_id mt6351_poweroff_of_match[] = {
	{ .compatible = "mediatek,mt6351-poweroff" },
	{ }
};
MODULE_DEVICE_TABLE(of, mt6351_poweroff_of_match);

static struct platform_driver mt6351_poweroff_driver = {
	.probe = mt6351_poweroff_probe,
	.driver = {
		.name = "mt6351-poweroff",
		.of_match_table = mt6351_poweroff_of_match,
	},
};
builtin_platform_driver(mt6351_poweroff_driver);

MODULE_DESCRIPTION("MT6351 PMIC power-off handler");
MODULE_LICENSE("GPL");
