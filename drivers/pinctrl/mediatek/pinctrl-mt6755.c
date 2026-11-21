// SPDX-License-Identifier: GPL-2.0
/*
 * Minimal MT6755 pinctrl/GPIO/EINT driver (Meizu M5 Note).
 *
 * Only what the touch panel needs: mode, direction, input and output
 * registers of the GPIO block at 0x10005000, plus the EINT block at
 * 0x1000b000. No pull, drive or iocfg support yet.
 *
 * Register layout read from the stock 3.18 kernel (mt_set_gpio_*_base,
 * mt_eint_*):
 *   dir 0x000, dout 0x100, din 0x200, stride 0x10, 32 pins per register
 *   mode 0x300 + 0x10 * (pin / 10): two groups of five 3-bit fields,
 *        at bits 0-14 and 16-30
 *   EINT: stat 0x000, ack 0x040, mask 0x080/0x0c0/0x100, sens 0x140/0x180/0x1c0,
 *        pol 0x340/0x380 (set/clear), 32 EINTs per register, 160 EINTs
 */
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>

#include "pinctrl-mtk-mt6755.h"
#include "pinctrl-paris.h"

static const struct mtk_pin_field_calc mt6755_pin_mode_range[] = {
	PIN_FIELD(0, 4, 0x300, 0x10, 0, 3),
	PIN_FIELD(5, 9, 0x300, 0x10, 16, 3),
	PIN_FIELD(10, 14, 0x310, 0x10, 0, 3),
	PIN_FIELD(15, 19, 0x310, 0x10, 16, 3),
	PIN_FIELD(20, 24, 0x320, 0x10, 0, 3),
	PIN_FIELD(25, 29, 0x320, 0x10, 16, 3),
	PIN_FIELD(30, 34, 0x330, 0x10, 0, 3),
	PIN_FIELD(35, 39, 0x330, 0x10, 16, 3),
	PIN_FIELD(40, 44, 0x340, 0x10, 0, 3),
	PIN_FIELD(45, 49, 0x340, 0x10, 16, 3),
	PIN_FIELD(50, 54, 0x350, 0x10, 0, 3),
	PIN_FIELD(55, 59, 0x350, 0x10, 16, 3),
	PIN_FIELD(60, 64, 0x360, 0x10, 0, 3),
	PIN_FIELD(65, 69, 0x360, 0x10, 16, 3),
	PIN_FIELD(70, 74, 0x370, 0x10, 0, 3),
	PIN_FIELD(75, 79, 0x370, 0x10, 16, 3),
	PIN_FIELD(80, 84, 0x380, 0x10, 0, 3),
	PIN_FIELD(85, 89, 0x380, 0x10, 16, 3),
	PIN_FIELD(90, 94, 0x390, 0x10, 0, 3),
	PIN_FIELD(95, 99, 0x390, 0x10, 16, 3),
	PIN_FIELD(100, 104, 0x3a0, 0x10, 0, 3),
	PIN_FIELD(105, 109, 0x3a0, 0x10, 16, 3),
	PIN_FIELD(110, 114, 0x3b0, 0x10, 0, 3),
	PIN_FIELD(115, 119, 0x3b0, 0x10, 16, 3),
	PIN_FIELD(120, 124, 0x3c0, 0x10, 0, 3),
	PIN_FIELD(125, 129, 0x3c0, 0x10, 16, 3),
	PIN_FIELD(130, 134, 0x3d0, 0x10, 0, 3),
	PIN_FIELD(135, 139, 0x3d0, 0x10, 16, 3),
	PIN_FIELD(140, 144, 0x3e0, 0x10, 0, 3),
	PIN_FIELD(145, 149, 0x3e0, 0x10, 16, 3),
	PIN_FIELD(150, 154, 0x3f0, 0x10, 0, 3),
	PIN_FIELD(155, 159, 0x3f0, 0x10, 16, 3),
	PIN_FIELD(160, 164, 0x400, 0x10, 0, 3),
	PIN_FIELD(165, 169, 0x400, 0x10, 16, 3),
	PIN_FIELD(170, 174, 0x410, 0x10, 0, 3),
	PIN_FIELD(175, 179, 0x410, 0x10, 16, 3),
	PIN_FIELD(180, 184, 0x420, 0x10, 0, 3),
	PIN_FIELD(185, 189, 0x420, 0x10, 16, 3),
};

static const struct mtk_pin_field_calc mt6755_pin_dir_range[] = {
	PIN_FIELD(0, 189, 0x0, 0x10, 0, 1),
};

static const struct mtk_pin_field_calc mt6755_pin_di_range[] = {
	PIN_FIELD(0, 189, 0x200, 0x10, 0, 1),
};

static const struct mtk_pin_field_calc mt6755_pin_do_range[] = {
	PIN_FIELD(0, 189, 0x100, 0x10, 0, 1),
};

static const struct mtk_pin_reg_calc mt6755_reg_cals[PINCTRL_PIN_REG_MAX] = {
	[PINCTRL_PIN_REG_MODE] = MTK_RANGE(mt6755_pin_mode_range),
	[PINCTRL_PIN_REG_DIR] = MTK_RANGE(mt6755_pin_dir_range),
	[PINCTRL_PIN_REG_DI] = MTK_RANGE(mt6755_pin_di_range),
	[PINCTRL_PIN_REG_DO] = MTK_RANGE(mt6755_pin_do_range),
};

static const char * const mt6755_pinctrl_register_base_names[] = {
	"gpio",
};

/*
 * 160 EINTs in 5 banks of 32 (stock DTB: max_eint_num = 0xa0). port_mask is
 * copied from mt6779 and the debounce table is left out: both unverified,
 * and not needed for the touch interrupt.
 */
static const struct mtk_eint_hw mt6755_eint_hw = {
	.port_mask = 7,
	.ports     = 5,
	.ap_num    = 160,
	.db_cnt    = 0,
};

static const struct mtk_pin_soc mt6755_data = {
	.reg_cal = mt6755_reg_cals,
	.pins = mtk_pins_mt6755,
	.npins = ARRAY_SIZE(mtk_pins_mt6755),
	.ngrps = ARRAY_SIZE(mtk_pins_mt6755),
	.eint_hw = &mt6755_eint_hw,
	.gpio_m = 0,
	.base_names = mt6755_pinctrl_register_base_names,
	.nbase_names = ARRAY_SIZE(mt6755_pinctrl_register_base_names),
};

static const struct of_device_id mt6755_pinctrl_of_match[] = {
	{ .compatible = "mediatek,mt6755-pinctrl", .data = &mt6755_data },
	{ }
};

static struct platform_driver mt6755_pinctrl_driver = {
	.driver = {
		.name = "mt6755-pinctrl",
		.of_match_table = mt6755_pinctrl_of_match,
	},
	.probe = mtk_paris_pinctrl_probe,
};

static int __init mt6755_pinctrl_init(void)
{
	return platform_driver_register(&mt6755_pinctrl_driver);
}
arch_initcall(mt6755_pinctrl_init);
