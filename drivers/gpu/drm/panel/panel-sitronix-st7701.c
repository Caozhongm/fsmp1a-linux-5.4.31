// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2019, Amarula Solutions.
 * Author: Jagan Teki <jagan@amarulasolutions.com>
 */

#include <drm/drmP.h>
#include <drm/drm_crtc.h>
#include <drm/drm_mipi_dsi.h>
#include <drm/drm_modes.h>
#include <drm/drm_panel.h>
#include <drm/drm_print.h>

#include <linux/gpio/consumer.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/regulator/consumer.h>

#include <video/mipi_display.h>
#include <linux/backlight.h>

#define  LCD_VERSION_V2		0x55

/* Command2 BKx selection command */
#define DSI_CMD2BKX_SEL			0xFF

/* Command2, BK0 commands */
#define DSI_CMD2_BK0_PVGAMCTRL		0xB0 /* Positive Voltage Gamma Control */
#define DSI_CMD2_BK0_NVGAMCTRL		0xB1 /* Negative Voltage Gamma Control */
#define DSI_CMD2_BK0_LNESET		0xC0 /* Display Line setting */
#define DSI_CMD2_BK0_PORCTRL		0xC1 /* Porch control */
#define DSI_CMD2_BK0_INVSEL		0xC2 /* Inversion selection, Frame Rate Control */

/* Command2, BK1 commands */
#define DSI_CMD2_BK1_VRHS		0xB0 /* Vop amplitude setting */
#define DSI_CMD2_BK1_VCOM		0xB1 /* VCOM amplitude setting */
#define DSI_CMD2_BK1_VGHSS		0xB2 /* VGH Voltage setting */
#define DSI_CMD2_BK1_TESTCMD		0xB3 /* TEST Command Setting */
#define DSI_CMD2_BK1_VGLS		0xB5 /* VGL Voltage setting */
#define DSI_CMD2_BK1_PWCTLR1		0xB7 /* Power Control 1 */
#define DSI_CMD2_BK1_PWCTLR2		0xB8 /* Power Control 2 */
#define DSI_CMD2_BK1_SPD1		0xC1 /* Source pre_drive timing set1 */
#define DSI_CMD2_BK1_SPD2		0xC2 /* Source EQ2 Setting */
#define DSI_CMD2_BK1_MIPISET1		0xD0 /* MIPI Setting 1 */

/**
 * Command2 with BK function selection.
 *
 * BIT[4, 0]: [CN2, BKXSEL]
 * 10 = CMD2BK0, Command2 BK0
 * 11 = CMD2BK1, Command2 BK1
 * 00 = Command2 disable
 */
#define DSI_CMD2BK1_SEL			0x11
#define DSI_CMD2BK0_SEL			0x10
#define DSI_CMD2BKX_SEL_NONE		0x00

/* Command2, BK0 bytes */
#define DSI_LINESET_LINE		0x69
#define DSI_LINESET_LDE_EN		BIT(7)
#define DSI_LINESET_LINEDELTA		GENMASK(1, 0)
#define DSI_CMD2_BK0_LNESET_B1		DSI_LINESET_LINEDELTA
#define DSI_CMD2_BK0_LNESET_B0		(DSI_LINESET_LDE_EN | DSI_LINESET_LINE)
#define DSI_INVSEL_DEFAULT		GENMASK(5, 4)
#define DSI_INVSEL_NLINV		GENMASK(2, 0)
#define DSI_INVSEL_RTNI			GENMASK(2, 1)
#define DSI_CMD2_BK0_INVSEL_B1		DSI_INVSEL_RTNI
#define DSI_CMD2_BK0_INVSEL_B0		(DSI_INVSEL_DEFAULT | DSI_INVSEL_NLINV)
#define DSI_CMD2_BK0_PORCTRL_B0(m)	((m)->vtotal - (m)->vsync_end)
#define DSI_CMD2_BK0_PORCTRL_B1(m)	((m)->vsync_start - (m)->vdisplay)

/* Command2, BK1 bytes */
// #define DSI_CMD2_BK1_VRHA_SET		0x45
#define DSI_CMD2_BK1_VRHA_SET		0x55
// #define DSI_CMD2_BK1_VCOM_SET		0x13
#define DSI_CMD2_BK1_VCOM_SET		0x38
// #define DSI_CMD2_BK1_VGHSS_SET		GENMASK(2, 0)
#define DSI_CMD2_BK1_VGHSS_SET		0x07
#define DSI_CMD2_BK1_TESTCMD_VAL	BIT(7)
#define DSI_VGLS_DEFAULT		BIT(6)
#define DSI_VGLS_SEL			GENMASK(2, 0)
#define DSI_CMD2_BK1_VGLS_SET		0x0e
#define DSI_PWCTLR1_AP			BIT(7) /* Gamma OP bias, max */
#define DSI_PWCTLR1_APIS		BIT(2) /* Source OP input bias, min */
#define DSI_PWCTLR1_APOS		BIT(0) /* Source OP output bias, min */
#define DSI_CMD2_BK1_PWCTLR1_SET	(DSI_PWCTLR1_AP | DSI_PWCTLR1_APIS | \
					DSI_PWCTLR1_APOS)
#define DSI_PWCTLR2_AVDD		BIT(5) /* AVDD 6.6v */
#define DSI_PWCTLR2_AVCL		0x0    /* AVCL -4.4v */
#define DSI_CMD2_BK1_PWCTLR2_SET	(DSI_PWCTLR2_AVDD | DSI_PWCTLR2_AVCL)
#define DSI_SPD1_T2D			BIT(3)
#define DSI_CMD2_BK1_SPD1_SET		(GENMASK(6, 4) | DSI_SPD1_T2D)
#define DSI_CMD2_BK1_SPD2_SET		DSI_CMD2_BK1_SPD1_SET
#define DSI_MIPISET1_EOT_EN		BIT(3)
#define DSI_CMD2_BK1_MIPISET1_SET	(BIT(7) | DSI_MIPISET1_EOT_EN)

struct st7701_panel_desc {
	const struct drm_display_mode *mode;
	unsigned int lanes;
	unsigned long flags;
	enum mipi_dsi_pixel_format format;
	const char *const *supply_names;
	unsigned int num_supplies;
	unsigned int panel_sleep_delay;
};

struct st7701 {
	struct device *dev;
	struct drm_panel panel;
	struct mipi_dsi_device *dsi;
	const struct st7701_panel_desc *desc;

	struct backlight_device *bl_dev;
	struct regulator_bulk_data *supplies;
	struct gpio_desc *reset;
	unsigned int sleep_delay;
};

static inline struct st7701 *panel_to_st7701(struct drm_panel *panel)
{
	return container_of(panel, struct st7701, panel);
}

static inline int st7701_dsi_write(struct st7701 *st7701, const void *seq,
				   size_t len)
{
	return mipi_dsi_dcs_write_buffer(st7701->dsi, seq, len);
}

#define ST7701_DSI(st7701, seq...)				\
	{							\
		const u8 d[] = { seq };				\
		st7701_dsi_write(st7701, d, ARRAY_SIZE(d));	\
	}

static void st7701_init_sequence(struct st7701 *st7701)
{
	const struct drm_display_mode *mode = st7701->desc->mode;

	ST7701_DSI(st7701, MIPI_DCS_SOFT_RESET, 0x00);

	/* We need to wait 5ms before sending new commands */
	msleep(5);

	ST7701_DSI(st7701, MIPI_DCS_EXIT_SLEEP_MODE, 0x00);

	msleep(st7701->sleep_delay);

	/* Command2, BK0 */
	ST7701_DSI(st7701, DSI_CMD2BKX_SEL,
		   0x77, 0x01, 0x00, 0x00, DSI_CMD2BK0_SEL);
	ST7701_DSI(st7701, DSI_CMD2_BK0_PVGAMCTRL, 0x00, 0x0E, 0x15, 0x0F,
		   0x11, 0x08, 0x08, 0x08, 0x08, 0x23, 0x04, 0x13, 0x12,
		   0x2B, 0x34, 0x1F);
	ST7701_DSI(st7701, DSI_CMD2_BK0_NVGAMCTRL, 0x00, 0x0E, 0x95, 0x0F,
		   0x13, 0x07, 0x09, 0x08, 0x08, 0x22, 0x04, 0x10, 0x0E,
		   0x2C, 0x34, 0x1F);
	ST7701_DSI(st7701, DSI_CMD2_BK0_LNESET,
		   DSI_CMD2_BK0_LNESET_B0, DSI_CMD2_BK0_LNESET_B1);
	ST7701_DSI(st7701, DSI_CMD2_BK0_PORCTRL,
		   DSI_CMD2_BK0_PORCTRL_B0(mode),
		   DSI_CMD2_BK0_PORCTRL_B1(mode));
	ST7701_DSI(st7701, DSI_CMD2_BK0_INVSEL,
		   DSI_CMD2_BK0_INVSEL_B0, DSI_CMD2_BK0_INVSEL_B1);

	/* Command2, BK1 */
	ST7701_DSI(st7701, DSI_CMD2BKX_SEL,
			0x77, 0x01, 0x00, 0x00, DSI_CMD2BK1_SEL);
	ST7701_DSI(st7701, DSI_CMD2_BK1_VRHS, DSI_CMD2_BK1_VRHA_SET);
	ST7701_DSI(st7701, DSI_CMD2_BK1_VCOM, DSI_CMD2_BK1_VCOM_SET);
	ST7701_DSI(st7701, DSI_CMD2_BK1_VGHSS, DSI_CMD2_BK1_VGHSS_SET);
	ST7701_DSI(st7701, DSI_CMD2_BK1_TESTCMD, DSI_CMD2_BK1_TESTCMD_VAL);
	ST7701_DSI(st7701, DSI_CMD2_BK1_VGLS, DSI_CMD2_BK1_VGLS_SET);
	ST7701_DSI(st7701, DSI_CMD2_BK1_PWCTLR1, DSI_CMD2_BK1_PWCTLR1_SET);
	ST7701_DSI(st7701, DSI_CMD2_BK1_PWCTLR2, DSI_CMD2_BK1_PWCTLR2_SET);
	ST7701_DSI(st7701, DSI_CMD2_BK1_SPD1, DSI_CMD2_BK1_SPD1_SET);
	ST7701_DSI(st7701, DSI_CMD2_BK1_SPD2, DSI_CMD2_BK1_SPD2_SET);
	ST7701_DSI(st7701, DSI_CMD2_BK1_MIPISET1, DSI_CMD2_BK1_MIPISET1_SET);

	/**
	 * ST7701_SPEC_V1.2 is unable to provide enough information above this
	 * specific command sequence, so grab the same from vendor BSP driver.
	 */
	ST7701_DSI(st7701,0xe0, 0x00, 0x00, 0x02);
	ST7701_DSI(st7701,0xe1, 0x05, 0x00, 0x00, 0x00, 0x04, 0x00, 0x00, 0x00, 0x00, 0x20, 0x20);
	ST7701_DSI(st7701,0xe2, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00);
	ST7701_DSI(st7701,0xe3, 0x00, 0x00, 0x33, 0x00);
	ST7701_DSI(st7701,0xe4, 0x22, 0x00);
	ST7701_DSI(st7701,0xe5, 0x07, 0x34, 0xa0, 0xa0, 0x05, 0x34, 0xa0, 0xa0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00);
	ST7701_DSI(st7701,0xe6, 0x00, 0x00, 0x33, 0x00);
	ST7701_DSI(st7701,0xe7, 0x22, 0x00);
	ST7701_DSI(st7701,0xe8, 0x06, 0x34, 0xa0, 0xa0, 0x04, 0x34, 0xa0, 0xa0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00);
	ST7701_DSI(st7701,0xeb, 0x02, 0x00, 0x10, 0x10, 0x00, 0x00, 0x00);
	ST7701_DSI(st7701,0xec, 0x02, 0x00);
	ST7701_DSI(st7701,0xed, 0xaa, 0x54, 0x0b, 0xbf, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfb, 0xb0, 0x45, 0xaa);
	/* disable Command2 */
	ST7701_DSI(st7701, DSI_CMD2BKX_SEL,
		   0x77, 0x01, 0x00, 0x00, DSI_CMD2BKX_SEL_NONE);
}

static void st7701_init_sequence_v2(struct st7701 *st7701)
{
	ST7701_DSI(st7701, MIPI_DCS_SOFT_RESET, 0x00);

	/* We need to wait 5ms before sending new commands */
	msleep(5);

	ST7701_DSI(st7701, MIPI_DCS_EXIT_SLEEP_MODE, 0x00);

	msleep(st7701->sleep_delay);
	ST7701_DSI(st7701,0xff, 0x77, 0x01, 0x00, 0x00, 0x13);
	ST7701_DSI(st7701,0xef, 0x08);
	
	ST7701_DSI(st7701,0xff, 0x77, 0x01, 0x00, 0x00, 0x10);
	ST7701_DSI(st7701,0xC0, 0xe9, 0x03);

	ST7701_DSI(st7701, 0xc1, 0x0c, 0x02);

	ST7701_DSI(st7701,0xc2, 0x10, 0x06);
	ST7701_DSI(st7701,0xcc, 0x38);
	ST7701_DSI(st7701,0xb0, 0x00, 0x07, 0x12, 0x14, 0x19, 0x0c, 0x14, 0x09, 0x06, 0x27, 0x04, 0x0c, 0x08, 0x11, 0x15, 0x1a);
	ST7701_DSI(st7701,0xb1, 0x00, 0x07, 0x11, 0x11, 0x18, 0x0a, 0x12, 0x0a, 0x0b, 0x27, 0x08, 0x17, 0x14, 0x1f, 0x23, 0x1a);
	ST7701_DSI(st7701,0xff, 0x77, 0x01, 0x00, 0x00, 0x11);

	ST7701_DSI(st7701,0xff, 0x77, 0x01, 0x00, 0x00, 0x11);
	ST7701_DSI(st7701,0xb0, 0x56);
	ST7701_DSI(st7701,0xb1, 0x74);
	ST7701_DSI(st7701,0xb2, 0x87);
	ST7701_DSI(st7701,0xb3, 0x80);
	ST7701_DSI(st7701,0xb5, 0x4d);
	ST7701_DSI(st7701,0xb7, 0x85);
	ST7701_DSI(st7701,0xb8, 0x10);
	ST7701_DSI(st7701,0xb9, 0x10);
	ST7701_DSI(st7701,0xbc, 0x03);
	ST7701_DSI(st7701,0xc0, 0x89);
	ST7701_DSI(st7701,0xc1, 0x78);
	ST7701_DSI(st7701,0xc2, 0x78);
	ST7701_DSI(st7701,0xd0, 0x88);
	msleep(10);
	/**
	 * ST7701_SPEC_V1.2 is unable to provide enough information above this
	 * specific command sequence, so grab the same from vendor BSP driver.
	 */
	ST7701_DSI(st7701,0xe0, 0x00, 0x00, 0x02);
	ST7701_DSI(st7701,0xe1, 0x04, 0x00, 0x00, 0x00, 0x05, 0x00, 0x00, 0x00, 0x00, 0x20, 0x20);
	ST7701_DSI(st7701,0xe2, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00);
	ST7701_DSI(st7701,0xe3, 0x00, 0x00, 0x33, 0x00);
	ST7701_DSI(st7701,0xe4, 0x22, 0x00);
	ST7701_DSI(st7701,0xe5, 0x04, 0x5c, 0xa0, 0xa0, 0x06, 0x5c, 0xa0, 0xa0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00);
	ST7701_DSI(st7701,0xe6, 0x00, 0x00, 0x33, 0x00);
	ST7701_DSI(st7701,0xe7, 0x22, 0x00);
	ST7701_DSI(st7701,0xe8, 0x05, 0x5c, 0xa0, 0xa0, 0x07, 0x5c, 0xa0, 0xa0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00);
	ST7701_DSI(st7701,0xeb, 0x02, 0x00, 0x40, 0x40, 0x00, 0x00, 0x00);
	ST7701_DSI(st7701,0xec, 0x00, 0x00);
	ST7701_DSI(st7701,0xed, 0xfa, 0x45, 0x0b, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xb0, 0x54, 0xaf);
	ST7701_DSI(st7701,0xef, 0x08, 0x08, 0x08, 0x42, 0x3f, 0x3f);
	/* disable Command2 */
	ST7701_DSI(st7701, DSI_CMD2BKX_SEL,
		   0x77, 0x01, 0x00, 0x00, DSI_CMD2BKX_SEL_NONE);

	ST7701_DSI(st7701,0x11);
	msleep(100);
	ST7701_DSI(st7701,0x29);
	msleep(20);
}


static int st7701_prepare(struct drm_panel *panel)
{
	struct st7701 *st7701 = panel_to_st7701(panel);
	char rddid[3] = {0};

	gpiod_set_value_cansleep(st7701->reset, 0);

	msleep(20);

	gpiod_set_value_cansleep(st7701->reset, 1);
	msleep(30);

	gpiod_set_value_cansleep(st7701->reset, 0);
	msleep(20);

	gpiod_set_value_cansleep(st7701->reset, 1);
	msleep(150);

	mipi_dsi_dcs_read(st7701->dsi, 0x04, rddid, 3);

	printk("******Benny Log %s: [%s %d] manufacturer ID==>%d******\n", __FILE__, __func__, __LINE__, rddid[0]);
	printk("******Benny Log %s: [%s %d] driver version ID==>%d******\n", __FILE__, __func__, __LINE__, rddid[1]);
	printk("******Benny Log %s: [%s %d] driver ID==>%d******\n", __FILE__, __func__, __LINE__, rddid[2]);

	if (rddid[0] == LCD_VERSION_V2) {
		st7701_init_sequence_v2(st7701);	
	} else {
		st7701_init_sequence(st7701);
	}

	return 0;
}

static int st7701_enable(struct drm_panel *panel)
{
	struct st7701 *st7701 = panel_to_st7701(panel);

	ST7701_DSI(st7701, MIPI_DCS_SET_DISPLAY_ON, 0x00);
	mipi_dsi_dcs_set_display_on(st7701->dsi);
	
	backlight_enable(st7701->bl_dev);

	return 0;
}

static int st7701_disable(struct drm_panel *panel)
{
	struct st7701 *st7701 = panel_to_st7701(panel);

	backlight_disable(st7701->bl_dev);

	ST7701_DSI(st7701, MIPI_DCS_SET_DISPLAY_OFF, 0x00);

	return mipi_dsi_dcs_set_display_off(st7701->dsi);
}

static int st7701_unprepare(struct drm_panel *panel)
{
	struct st7701 *st7701 = panel_to_st7701(panel);
	int ret;

	ST7701_DSI(st7701, MIPI_DCS_ENTER_SLEEP_MODE, 0x00);

	msleep(st7701->sleep_delay);

	/**
	 * During the Resetting period, the display will be blanked
	 * (The display is entering blanking sequence, which maximum
	 * time is 120 ms, when Reset Starts in Sleep Out –mode. The
	 * display remains the blank state in Sleep In –mode.) and
	 * then return to Default condition for Hardware Reset.
	 *
	 * So we need wait sleep_delay time to make sure reset completed.
	 */

    ret = mipi_dsi_dcs_set_display_off(st7701->dsi);
	if (ret < 0)
		dev_err(panel->dev, "failed to set display off: %d\n", ret);

	ret = mipi_dsi_dcs_enter_sleep_mode(st7701->dsi);
	if (ret < 0)
		dev_err(panel->dev, "failed to enter sleep mode: %d\n", ret);
	
	msleep(120);
	
	gpiod_set_value_cansleep(st7701->reset, 0);

	gpiod_set_value_cansleep(st7701->reset, 1);
	
	gpiod_set_value_cansleep(st7701->reset, 0);

	msleep(st7701->sleep_delay);

	regulator_bulk_disable(st7701->desc->num_supplies, st7701->supplies);

	return 0;
}

static const u32 rad_bus_formats[] = {
	MEDIA_BUS_FMT_RGB888_1X24,
};

static int st7701_get_modes(struct drm_panel *panel)
{
	struct st7701 *st7701 = panel_to_st7701(panel);
	struct drm_connector *connector = panel->connector;
	const struct drm_display_mode *desc_mode = st7701->desc->mode;
	struct drm_display_mode *mode;

	mode = drm_mode_duplicate(connector->dev, desc_mode);
	if (!mode) {
		DRM_DEV_ERROR(&st7701->dsi->dev,
			      "failed to add mode %ux%ux@%u\n",
			      desc_mode->hdisplay, desc_mode->vdisplay,
			      desc_mode->vrefresh);
		return -ENOMEM;
	}

	drm_mode_set_name(mode);
	drm_mode_probed_add(connector, mode);

	connector->display_info.width_mm = desc_mode->width_mm;
	connector->display_info.height_mm = desc_mode->height_mm;
	
	drm_display_info_set_bus_formats(&connector->display_info,
					 rad_bus_formats,
					 ARRAY_SIZE(rad_bus_formats));

	return 1;
}

static const struct drm_panel_funcs st7701_funcs = {
	.disable	= st7701_disable,
	.unprepare	= st7701_unprepare,
	.prepare	= st7701_prepare,
	.enable		= st7701_enable,
	.get_modes	= st7701_get_modes,
};

static const struct drm_display_mode st7701_mode = {
	.clock		= 27500,

	.hdisplay	= 480,
	.hsync_start	= 480 + 38,
	.hsync_end	= 480 + 38 + 12,
	.htotal		= 480 + 38 + 12 + 12,

	.vdisplay	= 854,
	.vsync_start	= 854 + 18,
	.vsync_end	= 854 + 18 + 18,
	.vtotal		= 854 + 18 + 18 + 11,

	.width_mm	= 69,
	.height_mm	= 139,

	.type = DRM_MODE_TYPE_DRIVER | DRM_MODE_TYPE_PREFERRED,
};

static const struct st7701_panel_desc st7701_desc = {
	.mode = &st7701_mode,
	.lanes = 2,
	.flags = MIPI_DSI_MODE_VIDEO | MIPI_DSI_MODE_VIDEO_BURST | MIPI_DSI_MODE_LPM | MIPI_DSI_CLOCK_NON_CONTINUOUS,
	.panel_sleep_delay = 80, /* panel need extra 80ms for sleep out cmd */
};

static int st7701_dsi_probe(struct mipi_dsi_device *dsi)
{
	struct device *dev = &dsi->dev;
	// struct device_node *backlight;
	const struct st7701_panel_desc *desc;
	struct st7701 *st7701;
	int ret, i;
	
	st7701 = devm_kzalloc(&dsi->dev, sizeof(*st7701), GFP_KERNEL);
	if (!st7701)
		return -ENOMEM;

	st7701->reset = devm_gpiod_get_optional(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(st7701->reset)) {
		ret = PTR_ERR(st7701->reset);
		dev_err(dev, "cannot get reset GPIO: %d\n", ret);
		return ret;
	}
	
	desc = of_device_get_match_data(&dsi->dev);
	dsi->mode_flags = desc->flags;
	dsi->format = desc->format;
	dsi->lanes = desc->lanes;

	st7701->supplies = devm_kcalloc(&dsi->dev, desc->num_supplies,
					sizeof(*st7701->supplies),
					GFP_KERNEL);
	if (!st7701->supplies)
		return -ENOMEM;

	for (i = 0; i < desc->num_supplies; i++)
		st7701->supplies[i].supply = desc->supply_names[i];

	ret = devm_regulator_bulk_get(&dsi->dev, desc->num_supplies,
				      st7701->supplies);
	if (ret < 0)
		return ret;

	drm_panel_init(&st7701->panel);
	st7701->panel.dev = &dsi->dev;
	st7701->panel.funcs = &st7701_funcs;

	st7701->sleep_delay = 120 + desc->panel_sleep_delay;

#if 0
	backlight = of_parse_phandle(dev->of_node, "backlight", 0);
	if (backlight) {
		st7701->bl_dev = of_find_backlight_by_node(backlight);
		of_node_put(backlight);

		if (!st7701->bl_dev)
			return -EPROBE_DEFER;
	}
#else

	st7701->bl_dev = devm_of_find_backlight(dev);
	if (IS_ERR(st7701->bl_dev))
		return PTR_ERR(st7701->bl_dev);

#endif


	ret = drm_panel_add(&st7701->panel);
	if (ret < 0)
		return ret;

	mipi_dsi_set_drvdata(dsi, st7701);
	st7701->dsi = dsi;
	st7701->desc = desc;

	ret = mipi_dsi_attach(dsi);
	
	return ret;
}

static int st7701_dsi_remove(struct mipi_dsi_device *dsi)
{
	struct st7701 *st7701 = mipi_dsi_get_drvdata(dsi);

	mipi_dsi_detach(dsi);
	drm_panel_remove(&st7701->panel);

	return 0;
}

static const struct of_device_id st7701_of_match[] = {
	{ .compatible = "sitronix,st7701", .data = &st7701_desc },
	{ }
};
MODULE_DEVICE_TABLE(of, st7701_of_match);

static struct mipi_dsi_driver st7701_dsi_driver = {
	.probe		= st7701_dsi_probe,
	.remove		= st7701_dsi_remove,
	.driver = {
		.name		= "st7701",
		.of_match_table	= st7701_of_match,
	},
};
module_mipi_dsi_driver(st7701_dsi_driver);

MODULE_AUTHOR("Jagan Teki <jagan@amarulasolutions.com>");
MODULE_DESCRIPTION("Sitronix ST7701 LCD Panel Driver");
MODULE_LICENSE("GPL");
