// SPDX-License-Identifier: GPL-2.0
/*
 * Panel driver for the Renesas R63350 FHD video-mode DSI panel (CSOT/TCL module) of the Meizu M5 Note.
 * Untested, not compiled.
 *
 * The init sequence, the power-up/power-down order and the timing values were decoded from the stock
 * bootloader (driver "r63350_fhd_dsi_vdo_tcl_csot"); the LCM reset line is MMSYS register 0x150 (not a GPIO).
 *
 * Install as drivers/gpu/drm/panel/panel-csot-r63350.c and add:
 *   Kconfig:   config DRM_PANEL_CSOT_R63350
 *                      tristate "CSOT R63350 FHD video mode DSI panel (Meizu M5 Note)"
 *                      depends on OF && DRM_MIPI_DSI && MFD_SYSCON
 *                      select DRM_KMS_HELPER
 *   Makefile:  obj-$(CONFIG_DRM_PANEL_CSOT_R63350) += panel-csot-r63350.o
 *
 * Device tree (inside the dsi0 node of mt6755-meizu-m5note-display.dtsi):
 *	panel@0 {
 *		compatible = "csot,r63350-fhd";
 *		reg = <0>;
 *		vsp-supply = <&reg_lcd_vsp>;		// regulator-fixed on GPIO12 (bias ENP), 5.5 V
 *		vsn-supply = <&reg_lcd_vsn>;		// regulator-fixed on GPIO7 (bias ENN), 5.5 V
 *		mediatek,lcm-reset = <&mmsys 0x150>;	// mmsys needs compatible = "mediatek,mt6755-mmsys", "syscon"
 *		backlight = <&backlight>;		// optional, the existing pwm-backlight node
 *		port { panel_in: endpoint { remote-endpoint = <&dsi0_out>; }; };
 *	};
 * The bias IC registers 0x00/0x01 (0x0f) are written by the bootloader and are not touched here.
 */

#include <linux/delay.h>
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>

#include <video/mipi_display.h>

#include <drm/drm_mipi_dsi.h>
#include <drm/drm_modes.h>
#include <drm/drm_panel.h>

struct r63350 {
	struct drm_panel panel;
	struct mipi_dsi_device *dsi;
	struct regulator *vsp;
	struct regulator *vsn;
	struct regmap *rst_map;
	unsigned int rst_off;
	bool prepared;
};

static inline struct r63350 *to_r63350(struct drm_panel *panel)
{
	return container_of(panel, struct r63350, panel);
}

/* MMSYS LCM_RST_B: 1 = released, 0 = reset asserted */
static void r63350_set_reset(struct r63350 *ctx, unsigned int val)
{
	regmap_write(ctx->rst_map, ctx->rst_off, val);
}

static int r63350_on(struct r63350 *ctx)
{
	struct mipi_dsi_device *dsi = ctx->dsi;

	dsi->mode_flags |= MIPI_DSI_MODE_LPM;

	/* bootloader sends sleep-out as a DCS short write with one parameter (0x15) */
	mipi_dsi_dcs_write_seq(dsi, MIPI_DCS_EXIT_SLEEP_MODE, 0x00);
	msleep(120);

	mipi_dsi_generic_write_seq(dsi, 0xb0, 0x04);
	mipi_dsi_generic_write_seq(dsi, 0xb3, 0x14, 0x00, 0x00, 0x00, 0x00, 0x00);
	mipi_dsi_generic_write_seq(dsi, 0xb4, 0x0c, 0x00);
	mipi_dsi_generic_write_seq(dsi, 0xb6, 0x4b, 0xdb, 0x16);
	mipi_dsi_generic_write_seq(dsi, 0xc1,
				   0x04, 0x60, 0x00, 0xbf, 0x0c, 0x74, 0x4c, 0xf8,
				   0xff, 0xff, 0xff, 0x5d, 0x63, 0xac, 0xb9, 0xff,
				   0xff, 0xff, 0x1f, 0x98, 0x83, 0x04, 0xf9, 0x00,
				   0x00, 0x00, 0x00, 0x00, 0x6a, 0x21, 0x02, 0x20,
				   0x00, 0x01);
	mipi_dsi_generic_write_seq(dsi, 0xc2,
				   0x31, 0xf7, 0x80, 0x08, 0x08, 0x00, 0x00, 0x08);
	mipi_dsi_generic_write_seq(dsi, 0xc3, 0x00, 0x00, 0x00);
	mipi_dsi_generic_write_seq(dsi, 0xc4,
				   0x70, 0x00, 0x00, 0x44, 0x44, 0x44, 0x44, 0x44,
				   0x44, 0x00, 0x03);
	mipi_dsi_generic_write_seq(dsi, 0xc6,
				   0xc8, 0x01, 0x31, 0x0f, 0x6a, 0x00, 0x00, 0x00,
				   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
				   0x00, 0x19, 0x16, 0x09, 0xc8);
	mipi_dsi_generic_write_seq(dsi, 0xc7,
				   0x0d, 0x1c, 0x23, 0x2c, 0x38, 0x45, 0x4e, 0x5d,
				   0x3f, 0x48, 0x52, 0x5f, 0x68, 0x6f, 0x78, 0x0d,
				   0x1c, 0x23, 0x2c, 0x38, 0x45, 0x4e, 0x5d, 0x45,
				   0x47, 0x52, 0x5f, 0x68, 0x6f, 0x78);
	mipi_dsi_generic_write_seq(dsi, 0xca,
				   0x1d, 0xfc, 0xfc, 0xfc, 0x00, 0xca, 0xca, 0x96,
				   0x00, 0x96, 0xc0, 0xc0, 0x00, 0x82, 0x00, 0x00,
				   0x00, 0x9e, 0x00, 0x00, 0x00, 0x82, 0xff, 0x9f,
				   0x83, 0xff, 0x9f, 0x83, 0x7d, 0x00, 0x00, 0x00,
				   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
				   0x00, 0x00, 0x00);
	mipi_dsi_generic_write_seq(dsi, 0xcb,
				   0x7e, 0xe0, 0x07, 0x7e, 0x08, 0x00, 0x00, 0x10,
				   0x56, 0xe0, 0x07, 0x6a, 0xe8, 0x00, 0x00);
	mipi_dsi_generic_write_seq(dsi, 0xcc, 0x0e);
	mipi_dsi_generic_write_seq(dsi, 0xd0,
				   0x11, 0x00, 0x00, 0x51, 0xd0, 0x40, 0x19, 0x19,
				   0x09, 0x00);
	mipi_dsi_generic_write_seq(dsi, 0xd1, 0x00, 0x48, 0x16, 0x0f);
	mipi_dsi_generic_write_seq(dsi, 0xd2, 0x5c, 0x00, 0x00);
	mipi_dsi_generic_write_seq(dsi, 0xd3,
				   0x1b, 0x33, 0xbb, 0xbb, 0xb3, 0x33, 0x33, 0x33,
				   0x33, 0x00, 0x01, 0x00, 0x00, 0xf8, 0xa0, 0x06,
				   0x53, 0x53, 0x33, 0x33, 0x72, 0x12, 0x8a, 0x57,
				   0x3d, 0xbc);
	mipi_dsi_generic_write_seq(dsi, 0xfd, 0x00, 0x00, 0x00, 0x00);
	mipi_dsi_generic_write_seq(dsi, 0xde, 0x00, 0x3f, 0xff, 0x10);
	mipi_dsi_generic_write_seq(dsi, 0xd6, 0x01);

	/* tear effect on, mode 0 (0x15 packet, same as the bootloader) */
	mipi_dsi_dcs_write_seq(dsi, MIPI_DCS_SET_TEAR_ON, 0x00);
	mipi_dsi_dcs_write_seq(dsi, MIPI_DCS_SET_DISPLAY_ON, 0x00);
	msleep(10);

	return 0;
}

static int r63350_off(struct r63350 *ctx)
{
	struct mipi_dsi_device *dsi = ctx->dsi;
	int ret;

	dsi->mode_flags &= ~MIPI_DSI_MODE_LPM;

	ret = mipi_dsi_dcs_set_display_off(dsi);
	if (ret < 0)
		return ret;
	msleep(20);

	/* re-send 0xd3 with two changed bytes (0x13, 0xb3) as the bootloader does before sleep-in */
	mipi_dsi_generic_write_seq(dsi, 0xd3,
				   0x13, 0x33, 0xbb, 0xb3, 0xb3, 0x33, 0x33, 0x33,
				   0x33, 0x00, 0x01, 0x00, 0x00, 0xf8, 0xa0, 0x06,
				   0x53, 0x53, 0x33, 0x33, 0x72, 0x12, 0x8a, 0x57,
				   0x3d, 0xbc);

	ret = mipi_dsi_dcs_enter_sleep_mode(dsi);
	if (ret < 0)
		return ret;
	msleep(120);
	msleep(6);

	mipi_dsi_generic_write_seq(dsi, 0xb0, 0x04);
	mipi_dsi_generic_write_seq(dsi, 0xb1, 0x01);
	msleep(20);

	return 0;
}

static int r63350_prepare(struct drm_panel *panel)
{
	struct r63350 *ctx = to_r63350(panel);
	struct device *dev = &ctx->dsi->dev;
	int ret;

	if (ctx->prepared)
		return 0;

	/* bias: ENP, 2 ms, ENN, 5 ms (bootloader init_power) */
	ret = regulator_enable(ctx->vsp);
	if (ret < 0) {
		dev_err(dev, "Failed to enable vsp: %d\n", ret);
		return ret;
	}
	usleep_range(2000, 3000);

	ret = regulator_enable(ctx->vsn);
	if (ret < 0) {
		dev_err(dev, "Failed to enable vsn: %d\n", ret);
		regulator_disable(ctx->vsp);
		return ret;
	}
	msleep(5);

	/* reset: 1, 1 ms, 0, 2 ms, 1, 6 ms (bootloader init) */
	r63350_set_reset(ctx, 1);
	usleep_range(1000, 2000);
	r63350_set_reset(ctx, 0);
	usleep_range(2000, 3000);
	r63350_set_reset(ctx, 1);
	usleep_range(6000, 7000);

	ret = r63350_on(ctx);
	if (ret < 0) {
		dev_err(dev, "Failed to initialize panel: %d\n", ret);
		r63350_set_reset(ctx, 0);
		regulator_disable(ctx->vsn);
		regulator_disable(ctx->vsp);
		return ret;
	}

	ctx->prepared = true;
	return 0;
}

static int r63350_unprepare(struct drm_panel *panel)
{
	struct r63350 *ctx = to_r63350(panel);
	struct device *dev = &ctx->dsi->dev;
	int ret;

	if (!ctx->prepared)
		return 0;

	ret = r63350_off(ctx);
	if (ret < 0)
		dev_err(dev, "Failed to un-initialize panel: %d\n", ret);

	/* reset low, 10 ms, ENN off, 5 ms, ENP off, 10 ms (bootloader suspend) */
	r63350_set_reset(ctx, 0);
	msleep(10);
	regulator_disable(ctx->vsn);
	msleep(5);
	regulator_disable(ctx->vsp);
	msleep(10);

	ctx->prepared = false;
	return 0;
}

/*
 * Timing totals from the bootloader parameters: 1080 + hfp 96 + hsync 10 + hbp 56 = 1242,
 * 1920 + vfp 16 + vsync 4 + vbp 4 = 1944. The pixel clock is chosen so that mtk_dsi derives the
 * bootloader's lane rate: 940 Mbit/s * 4 lanes / 24 bit = 156.667 MHz (the real refresh rate then comes out
 * near 65 Hz; the bootloader reports 60.1 Hz, which would need 870 Mbit/s - TO BE CHECKED on the panel).
 */
static const struct drm_display_mode r63350_mode = {
	.clock = 156667,
	.hdisplay = 1080,
	.hsync_start = 1080 + 96,
	.hsync_end = 1080 + 96 + 10,
	.htotal = 1080 + 96 + 10 + 56,
	.vdisplay = 1920,
	.vsync_start = 1920 + 16,
	.vsync_end = 1920 + 16 + 4,
	.vtotal = 1920 + 16 + 4 + 4,
	.width_mm = 68,
	.height_mm = 121,
};

static int r63350_get_modes(struct drm_panel *panel, struct drm_connector *connector)
{
	struct drm_display_mode *mode;

	mode = drm_mode_duplicate(connector->dev, &r63350_mode);
	if (!mode)
		return -ENOMEM;

	drm_mode_set_name(mode);

	mode->type = DRM_MODE_TYPE_DRIVER | DRM_MODE_TYPE_PREFERRED;
	connector->display_info.width_mm = mode->width_mm;
	connector->display_info.height_mm = mode->height_mm;
	drm_mode_probed_add(connector, mode);

	return 1;
}

static const struct drm_panel_funcs r63350_panel_funcs = {
	.prepare = r63350_prepare,
	.unprepare = r63350_unprepare,
	.get_modes = r63350_get_modes,
};

static int r63350_probe(struct mipi_dsi_device *dsi)
{
	struct device *dev = &dsi->dev;
	struct r63350 *ctx;
	int ret;

	ctx = devm_kzalloc(dev, sizeof(*ctx), GFP_KERNEL);
	if (!ctx)
		return -ENOMEM;

	ctx->vsp = devm_regulator_get(dev, "vsp");
	if (IS_ERR(ctx->vsp))
		return dev_err_probe(dev, PTR_ERR(ctx->vsp), "Failed to get vsp\n");

	ctx->vsn = devm_regulator_get(dev, "vsn");
	if (IS_ERR(ctx->vsn))
		return dev_err_probe(dev, PTR_ERR(ctx->vsn), "Failed to get vsn\n");

	ctx->rst_map = syscon_regmap_lookup_by_phandle_args(dev->of_node, "mediatek,lcm-reset",
							    1, &ctx->rst_off);
	if (IS_ERR(ctx->rst_map))
		return dev_err_probe(dev, PTR_ERR(ctx->rst_map), "Failed to get lcm-reset syscon\n");

	ctx->dsi = dsi;
	mipi_dsi_set_drvdata(dsi, ctx);

	dsi->lanes = 4;
	dsi->format = MIPI_DSI_FMT_RGB888;
	/*
	 * Measured on the running bootloader state: sync-event video mode (no BURST, no SYNC_PULSE),
	 * non-continuous clock (TXRX_CTRL bit 16 set), EOT packets enabled (TXRX_CTRL bit 6 clear).
	 * In this tree mtk_dsi.c sets DIS_EOT when MIPI_DSI_MODE_NO_EOT_PACKET is NOT set, so the flag
	 * is set here to keep the bit clear. Check TXRX_CTRL after bring-up.
	 */
	dsi->mode_flags = MIPI_DSI_MODE_VIDEO | MIPI_DSI_CLOCK_NON_CONTINUOUS |
			  MIPI_DSI_MODE_NO_EOT_PACKET;

	drm_panel_init(&ctx->panel, dev, &r63350_panel_funcs, DRM_MODE_CONNECTOR_DSI);

	ret = drm_panel_of_backlight(&ctx->panel);
	if (ret)
		return dev_err_probe(dev, ret, "Failed to get backlight\n");

	drm_panel_add(&ctx->panel);

	ret = mipi_dsi_attach(dsi);
	if (ret < 0) {
		dev_err(dev, "Failed to attach to DSI host: %d\n", ret);
		drm_panel_remove(&ctx->panel);
		return ret;
	}

	return 0;
}

static void r63350_remove(struct mipi_dsi_device *dsi)
{
	struct r63350 *ctx = mipi_dsi_get_drvdata(dsi);
	int ret;

	ret = mipi_dsi_detach(dsi);
	if (ret < 0)
		dev_err(&dsi->dev, "Failed to detach from DSI host: %d\n", ret);

	drm_panel_remove(&ctx->panel);
}

static const struct of_device_id r63350_of_match[] = {
	{ .compatible = "csot,r63350-fhd" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, r63350_of_match);

static struct mipi_dsi_driver r63350_driver = {
	.probe = r63350_probe,
	.remove = r63350_remove,
	.driver = {
		.name = "panel-csot-r63350",
		.of_match_table = r63350_of_match,
	},
};
module_mipi_dsi_driver(r63350_driver);

MODULE_DESCRIPTION("DRM driver for the CSOT R63350 FHD video mode DSI panel (Meizu M5 Note)");
MODULE_LICENSE("GPL");
