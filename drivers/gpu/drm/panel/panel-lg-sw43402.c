// SPDX-License-Identifier: GPL-2.0-only
/*
 * DRM driver for the LG SW43402 AMOLED panel found in the LG V30.
 *
 * Copyright (c) 2026 GuWolf <admin@guwolf.com>
 */

#include <linux/backlight.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/module.h>
#include <linux/of.h>

#include <video/mipi_display.h>

#include <drm/display/drm_dsc.h>
#include <drm/display/drm_dsc_helper.h>
#include <drm/drm_mipi_dsi.h>
#include <drm/drm_modes.h>
#include <drm/drm_panel.h>

struct sw43402 {
	struct drm_panel panel;
	struct mipi_dsi_device *dsi;
	struct gpio_desc *reset_gpio;
	struct gpio_desc *vddio_gpio;
	struct gpio_desc *vpnl_gpio;
	struct drm_dsc_config dsc;
	int error;
	bool prepared;
};

static inline struct sw43402 *to_sw43402(struct drm_panel *panel)
{
	return container_of(panel, struct sw43402, panel);
}

#define dsi_dcs_write_seq(ctx, seq...) do {				\
		static const u8 d[] = { seq };				\
									\
		sw43402_dcs_write(ctx, d, ARRAY_SIZE(d));		\
	} while (0)

static void sw43402_dcs_write(struct sw43402 *ctx, const void *data,
			      size_t len)
{
	int ret;

	if (ctx->error < 0)
		return;

	ret = mipi_dsi_dcs_write_buffer(ctx->dsi, data, len);
	if (ret < 0)
		ctx->error = ret;
}

static void sw43402_power_off(struct sw43402 *ctx)
{
	gpiod_set_value_cansleep(ctx->reset_gpio, 0);
	gpiod_set_value_cansleep(ctx->vddio_gpio, 0);
	usleep_range(1000, 1100);
	gpiod_set_value_cansleep(ctx->vpnl_gpio, 0);
	usleep_range(1000, 1100);
}

static void sw43402_power_on(struct sw43402 *ctx)
{
	gpiod_set_value_cansleep(ctx->vddio_gpio, 1);
	usleep_range(2000, 2100);
	gpiod_set_value_cansleep(ctx->vpnl_gpio, 1);
	usleep_range(2000, 2100);

	gpiod_set_value_cansleep(ctx->reset_gpio, 1);
	usleep_range(10000, 11000);
	gpiod_set_value_cansleep(ctx->reset_gpio, 0);
	usleep_range(2000, 2100);
	gpiod_set_value_cansleep(ctx->reset_gpio, 1);
	usleep_range(10000, 11000);
}

static int sw43402_on(struct sw43402 *ctx)
{
	struct drm_dsc_picture_parameter_set pps;
	struct mipi_dsi_device *dsi = ctx->dsi;
	int ret;

	dsi->mode_flags |= MIPI_DSI_MODE_LPM;
	ctx->error = 0;

	dsi_dcs_write_seq(ctx, 0xb0, 0x20, 0x43);
	dsi_dcs_write_seq(ctx, 0x3d, 0x00);
	dsi_dcs_write_seq(ctx, 0xf2, 0x00);
	dsi_dcs_write_seq(ctx, 0xff, 0x03, 0x00);
	dsi_dcs_write_seq(ctx, MIPI_DCS_SET_TEAR_ON);
	if (ctx->error < 0)
		return ctx->error;

	ret = mipi_dsi_dcs_exit_sleep_mode(dsi);
	if (ret < 0)
		return ret;
	msleep(60);

	dsi_dcs_write_seq(ctx, MIPI_DCS_SET_DISPLAY_BRIGHTNESS, 0x03);
	dsi_dcs_write_seq(ctx, MIPI_DCS_WRITE_CONTROL_DISPLAY, 0x07);
	dsi_dcs_write_seq(ctx, MIPI_DCS_WRITE_POWER_SAVE, 0x0c);
	dsi_dcs_write_seq(ctx, 0xb0, 0xa5, 0x00);
	dsi_dcs_write_seq(ctx, 0xb2, 0x5d, 0x41, 0x04, 0x8c, 0x00,
			  0xff, 0xff, 0x15, 0x00, 0x00, 0x00, 0x00);
	dsi_dcs_write_seq(ctx, 0xe8, 0x08, 0x90, 0x10, 0x25);
	dsi_dcs_write_seq(ctx, 0xd4, 0x10, 0x00, 0xff, 0x60, 0x30,
			  0x40, 0x50, 0x20, 0x20, 0x20, 0x20, 0xa0,
			  0x00, 0x20, 0x00, 0x34, 0xa0, 0x08, 0xda,
			  0xda, 0x4a);
	dsi_dcs_write_seq(ctx, 0xfb, 0x03, 0x77);
	dsi_dcs_write_seq(ctx, 0xed, 0x13, 0x00, 0x07, 0x00, 0x13);
	dsi_dcs_write_seq(ctx, 0xe2, 0x20, 0x0d, 0x08, 0xa8, 0x0a,
			  0xaa, 0x04, 0xa4, 0x80, 0x80, 0x80, 0x5c,
			  0x5c, 0x5c);
	dsi_dcs_write_seq(ctx, 0xe7, 0x00, 0x0d, 0x76, 0x1f, 0x00,
			  0x0d, 0x4a, 0x44, 0x0d, 0x76, 0x25, 0x00,
			  0x0d, 0x0d, 0x0d, 0x0d, 0x4a, 0x00);
	dsi_dcs_write_seq(ctx, 0xce, 0x81, 0x1f, 0x0f, 0x01, 0x24,
			  0x68, 0x22, 0x20, 0x04, 0x01, 0x00, 0x80,
			  0xff, 0x88, 0x08, 0x02, 0x00, 0x00);
	msleep(90);
	dsi_dcs_write_seq(ctx, 0xe7, 0x00, 0x0d, 0x76, 0x1f, 0x00,
			  0x0d, 0x0d, 0x44, 0x0d, 0x76, 0x25, 0x00,
			  0x0d, 0x0d, 0x0d, 0x0d, 0x4a, 0x00);
	msleep(70);
	if (ctx->error < 0)
		return ctx->error;

	drm_dsc_pps_payload_pack(&pps, &ctx->dsc);
	ret = mipi_dsi_picture_parameter_set(dsi, &pps);
	if (ret < 0)
		return ret;
	usleep_range(10000, 11000);

	/* Match the downstream post-panel-on sequence after one frame. */
	usleep_range(17000, 18000);
	dsi_dcs_write_seq(ctx, 0xb0, 0xa5, 0x00);
	msleep(60);
	if (ctx->error < 0)
		return ctx->error;

	return mipi_dsi_dcs_set_display_on(dsi);
}

static int sw43402_off(struct sw43402 *ctx)
{
	struct mipi_dsi_device *dsi = ctx->dsi;
	int ret;

	dsi->mode_flags |= MIPI_DSI_MODE_LPM;
	ctx->error = 0;

	dsi_dcs_write_seq(ctx, 0xca, 0x00, 0x06, 0x00, 0x06, 0x00,
			  0x16, 0x10);
	dsi_dcs_write_seq(ctx, 0xcb, 0x0b, 0x68, 0x00, 0x0b, 0x68,
			  0x00, 0x0b, 0x68, 0x00, 0x0b, 0x68, 0x00,
			  0x0b, 0x68, 0x00, 0x0b, 0x68, 0x00, 0x0b,
			  0x68, 0x00, 0x0b, 0x68, 0x00, 0x0b, 0x68,
			  0x00, 0x0b, 0x68, 0x00);
	dsi_dcs_write_seq(ctx, 0xcc, 0x0b, 0x68, 0x00, 0x0b, 0x68,
			  0x00, 0x0b, 0x68, 0x00, 0x05, 0xb4, 0x00,
			  0x05, 0xb4, 0x00, 0x55, 0x12, 0x13);
	dsi_dcs_write_seq(ctx, 0xe8, 0x08, 0x90, 0x10, 0x25);
	if (ctx->error < 0)
		return ctx->error;

	ret = mipi_dsi_dcs_set_display_off(dsi);
	if (ret < 0)
		return ret;

	ret = mipi_dsi_dcs_enter_sleep_mode(dsi);
	if (ret < 0)
		return ret;
	msleep(150);

	return 0;
}

static int sw43402_prepare(struct drm_panel *panel)
{
	struct sw43402 *ctx = to_sw43402(panel);
	struct device *dev = &ctx->dsi->dev;
	int ret;

	if (ctx->prepared)
		return 0;

	sw43402_power_on(ctx);

	ret = sw43402_on(ctx);
	if (ret < 0) {
		dev_err(dev, "failed to initialize panel: %d\n", ret);
		sw43402_power_off(ctx);
		return ret;
	}

	ctx->prepared = true;
	return 0;
}

static int sw43402_unprepare(struct drm_panel *panel)
{
	struct sw43402 *ctx = to_sw43402(panel);
	struct device *dev = &ctx->dsi->dev;
	int ret;

	if (!ctx->prepared)
		return 0;

	ret = sw43402_off(ctx);
	if (ret < 0)
		dev_err(dev, "failed to turn panel off: %d\n", ret);

	sw43402_power_off(ctx);
	ctx->prepared = false;

	return ret;
}

static const struct drm_display_mode sw43402_mode = {
	.clock = 282036,

	.hdisplay = 1440,
	.hsync_start = 1440 + 92,
	.hsync_end = 1440 + 92 + 32,
	.htotal = 1440 + 92 + 32 + 48,

	.vdisplay = 2880,
	.vsync_start = 2880 + 10,
	.vsync_end = 2880 + 10 + 1,
	.vtotal = 2880 + 10 + 1 + 25,

	.width_mm = 68,
	.height_mm = 136,
};

static int sw43402_get_modes(struct drm_panel *panel,
			     struct drm_connector *connector)
{
	struct drm_display_mode *mode;

	mode = drm_mode_duplicate(connector->dev, &sw43402_mode);
	if (!mode)
		return -ENOMEM;

	drm_mode_set_name(mode);
	mode->type = DRM_MODE_TYPE_DRIVER | DRM_MODE_TYPE_PREFERRED;
	connector->display_info.width_mm = mode->width_mm;
	connector->display_info.height_mm = mode->height_mm;
	drm_mode_probed_add(connector, mode);

	return 1;
}

static const struct drm_panel_funcs sw43402_panel_funcs = {
	.prepare = sw43402_prepare,
	.unprepare = sw43402_unprepare,
	.get_modes = sw43402_get_modes,
};

static int sw43402_bl_update_status(struct backlight_device *bl)
{
	struct mipi_dsi_device *dsi = bl_get_data(bl);
	u8 payload[] = {
		MIPI_DCS_SET_DISPLAY_BRIGHTNESS,
		backlight_get_brightness(bl),
	};
	int ret;

	dsi->mode_flags |= MIPI_DSI_MODE_LPM;
	ret = mipi_dsi_dcs_write_buffer(dsi, payload, sizeof(payload));

	return ret < 0 ? ret : 0;
}

static const struct backlight_ops sw43402_bl_ops = {
	.update_status = sw43402_bl_update_status,
};

static struct backlight_device *
sw43402_create_backlight(struct mipi_dsi_device *dsi)
{
	struct device *dev = &dsi->dev;
	const struct backlight_properties props = {
		.type = BACKLIGHT_RAW,
		.brightness = 158,
		.max_brightness = 255,
	};

	return devm_backlight_device_register(dev, dev_name(dev), dev, dsi,
					      &sw43402_bl_ops, &props);
}

static int sw43402_probe(struct mipi_dsi_device *dsi)
{
	struct device *dev = &dsi->dev;
	struct sw43402 *ctx;
	int ret;

	ctx = devm_kzalloc(dev, sizeof(*ctx), GFP_KERNEL);
	if (!ctx)
		return -ENOMEM;

	ctx->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_OUT_LOW);
	if (IS_ERR(ctx->reset_gpio))
		return dev_err_probe(dev, PTR_ERR(ctx->reset_gpio),
				     "failed to get reset GPIO\n");

	ctx->vddio_gpio = devm_gpiod_get(dev, "vddio", GPIOD_OUT_LOW);
	if (IS_ERR(ctx->vddio_gpio))
		return dev_err_probe(dev, PTR_ERR(ctx->vddio_gpio),
				     "failed to get VDDIO GPIO\n");

	ctx->vpnl_gpio = devm_gpiod_get(dev, "vpnl", GPIOD_OUT_LOW);
	if (IS_ERR(ctx->vpnl_gpio))
		return dev_err_probe(dev, PTR_ERR(ctx->vpnl_gpio),
				     "failed to get VPNL GPIO\n");

	ctx->dsi = dsi;
	mipi_dsi_set_drvdata(dsi, ctx);

	dsi->lanes = 4;
	dsi->format = MIPI_DSI_FMT_RGB888;
	dsi->mode_flags = MIPI_DSI_MODE_VIDEO_BURST | MIPI_DSI_MODE_LPM;

	ctx->dsc.dsc_version_major = 1;
	ctx->dsc.dsc_version_minor = 1;
	ctx->dsc.pic_width = 1440;
	ctx->dsc.pic_height = 2880;
	ctx->dsc.slice_height = 16;
	ctx->dsc.slice_width = 720;
	ctx->dsc.slice_count = 2;
	dsi->dsc_slice_per_pkt = 2;
	ctx->dsc.bits_per_component = 8;
	ctx->dsc.bits_per_pixel = 8 << 4;
	ctx->dsc.block_pred_enable = true;
	dsi->dsc = &ctx->dsc;

	drm_panel_init(&ctx->panel, dev, &sw43402_panel_funcs,
		       DRM_MODE_CONNECTOR_DSI);

	ctx->panel.backlight = sw43402_create_backlight(dsi);
	if (IS_ERR(ctx->panel.backlight))
		return dev_err_probe(dev, PTR_ERR(ctx->panel.backlight),
				     "failed to create backlight\n");

	drm_panel_add(&ctx->panel);

	ret = mipi_dsi_attach(dsi);
	if (ret < 0) {
		drm_panel_remove(&ctx->panel);
		return dev_err_probe(dev, ret, "failed to attach DSI host\n");
	}

	return 0;
}

static void sw43402_remove(struct mipi_dsi_device *dsi)
{
	struct sw43402 *ctx = mipi_dsi_get_drvdata(dsi);
	int ret;

	ret = mipi_dsi_detach(dsi);
	if (ret < 0)
		dev_err(&dsi->dev, "failed to detach DSI host: %d\n", ret);

	drm_panel_remove(&ctx->panel);
}

static const struct of_device_id sw43402_of_match[] = {
	{ .compatible = "lg,sw43402" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, sw43402_of_match);

static struct mipi_dsi_driver sw43402_driver = {
	.probe = sw43402_probe,
	.remove = sw43402_remove,
	.driver = {
		.name = "panel-lg-sw43402",
		.of_match_table = sw43402_of_match,
	},
};
module_mipi_dsi_driver(sw43402_driver);

MODULE_AUTHOR("GuWolf <admin@guwolf.com>");
MODULE_DESCRIPTION("DRM driver for the LG SW43402 AMOLED DSI panel");
MODULE_LICENSE("GPL");
