
// SPDX-License-Identifier: GPL-2.0
/*
 * i.MX drm driver - Sitronix MIPI-DSI panel driver
 * Author: Volker Peters
 * Based on work of:
 * Robert Chiras <robert.chiras@nxp.com>
 * Jagan Teki <jagan@amarulasolutions.com>
 *
 * Adapted by Goran Stankovic <goran.stankovic@magnosco.com> 20230324
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <drm/drm_crtc.h>
#include <drm/drm_mipi_dsi.h>
#include <drm/drm_modes.h>
#include <drm/drm_panel.h>

#include <linux/gpio/consumer.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/of.h>
#include <linux/regulator/consumer.h>

#include <video/mipi_display.h>
#include <video/of_videomode.h>
#include <video/videomode.h>

//from rm67191
#include <linux/of_platform.h>

/* Write Manufacture Command Set Control */
#define WRMAUCCTR 0xEF

/* Base Commands */
#define DSI_CMD_TEOFF			0x34
#define DSI_CMD_TEON			0x35
#define DSI_CMD_MADCTL			0x36
#define DSI_CMD_COLMOD			0x3A

/* Command2 BKx selection command */
#define DSI_CMD2BKX_SEL			0xFF

/* Command2, BK0 commands */
#define DSI_CMD2_BK0_PVGAMCTRL	0xB0 /* Positive Voltage Gamma Control */
#define DSI_CMD2_BK0_NVGAMCTRL	0xB1 /* Negative Voltage Gamma Control */
#define DSI_CMD2_BK0_LNESET		0xC0 /* Display Line setting */
#define DSI_CMD2_BK0_PORCTRL	0xC1 /* Porch control */
#define DSI_CMD2_BK0_INVSEL		0xC2 /* Inversion selection, Frame Rate Control */
#define DSI_CMD2_BK0_RGBCTRL	0xC3 /* RGB control */
#define DSI_CMD2_BK0_SDIR		0xC7 /* X-direction Control */

/* Command2, BK1 commands */
#define DSI_CMD2_BK1_VRHS		0xB0 /* Vop amplitude setting */
#define DSI_CMD2_BK1_VCOM		0xB1 /* VCOM amplitude setting */
#define DSI_CMD2_BK1_VGHSS		0xB2 /* VGH Voltage setting */
#define DSI_CMD2_BK1_TESTCMD	0xB3 /* TEST Command Setting */
#define DSI_CMD2_BK1_VGLS		0xB5 /* VGL Voltage setting */
#define DSI_CMD2_BK1_PWCTLR1	0xB7 /* Power Control 1 */
#define DSI_CMD2_BK1_PWCTLR2	0xB8 /* Power Control 2 */
#define DSI_CMD2_BK1_SPD1		0xC1 /* Source pre_drive timing set1 */
#define DSI_CMD2_BK1_SPD2		0xC2 /* Source EQ2 Setting - data sheet: Source pre_drive timing set2 */
#define DSI_CMD2_BK1_MIPISET1	0xD0 /* MIPI Setting 1 */

/*
 * Command2 with BK function selection.
 *
 * BIT[4, 0]: [CN2, BKXSEL]
 * 10 = CMD2BK0, Command2 BK0
 * 11 = CMD2BK1, Command2 BK1
 * 00 = Command2 disable
 */
#define DSI_CMD2BK1_SEL				0x11
#define DSI_CMD2BK0_SEL				0x10
#define DSI_CMD2BKX_SEL_NONE		0x00

/* Command2, BK0 bytes */
#define DSI_LINESET_LINE			0x69
#define DSI_LINESET_LDE_EN			BIT(7)
#define DSI_LINESET_LINEDELTA		GENMASK(1, 0)

#define DSI_CMD2_BK0_LNESET_B1		0x00
#define DSI_CMD2_BK0_LNESET_B0		0x3B
#define DSI_INVSEL_DEFAULT			GENMASK(5, 4)
#define DSI_INVSEL_NLINV			GENMASK(0, 0)
#define DSI_INVSEL_RTNI				0x05
#define DSI_CMD2_BK0_INVSEL_B1		DSI_INVSEL_RTNI
#define DSI_CMD2_BK0_INVSEL_B0		(DSI_INVSEL_DEFAULT | DSI_INVSEL_NLINV)
#define DSI_CMD2_BK0_PORCTRL_B0		0x10
/*gst: vtotal-vsync_end: .vsync_end	= 480 + 40 + 10,
	.vtotal		= 480 + 40 + 10 + 60,  --> 60 */
#define DSI_CMD2_BK0_PORCTRL_B1		0x0C 
/*gst: ((m)->vsync_start - (m)->vdisplay): vsync_start	= 480 + 40, -
	.vdisplay	= 480,	-->  40, */

/* Command2, BK1 bytes */
#define DSI_CMD2_BK1_VRHA_SET		0x5D
#define DSI_CMD2_BK1_VCOM_SET		0x34
#define DSI_CMD2_BK1_VGHSS_SET		GENMASK(2, 0)
#define DSI_CMD2_BK1_TESTCMD_VAL	BIT(7)
#define DSI_VGLS_DEFAULT			BIT(6)
#define DSI_VGLS_SEL				GENMASK(2, 0)
#define DSI_CMD2_BK1_VGLS_SET		0x4E
#define DSI_PWCTLR1_AP				BIT(7) /* Gamma OP bias, max */
#define DSI_PWCTLR1_APIS			BIT(2) /* Source OP input bias, min */
#define DSI_PWCTLR1_APOS			BIT(0) /* Source OP output bias, min */
#define DSI_CMD2_BK1_PWCTLR1_SET	(DSI_PWCTLR1_AP | DSI_PWCTLR1_APIS | \
									DSI_PWCTLR1_APOS)
#define DSI_PWCTLR2_AVDD			BIT(5) /* AVDD 6.6 V */
#define DSI_PWCTLR2_AVCL			0x01    /* AVCL ? V */
#define DSI_CMD2_BK1_PWCTLR2_SET	(DSI_PWCTLR2_AVDD | DSI_PWCTLR2_AVCL)
#define DSI_SPD1_T2D				BIT(4)
#define DSI_CMD2_BK1_SPD1_SET		(GENMASK(6, 4) | DSI_SPD1_T2D)
#define DSI_CMD2_BK1_SPD2_SET		DSI_CMD2_BK1_SPD1_SET
#define DSI_MIPISET1_EOT_EN			BIT(3)
#define DSI_CMD2_BK1_MIPISET1_SET	(BIT(7) | DSI_MIPISET1_EOT_EN)

/* Manufacturer Command Set pages (CMD2) */
struct cmd_set_entry {
	u8 cmd;
	u8 param;
};

static const u32 sit_bus_formats[] = {
	MEDIA_BUS_FMT_RGB888_1X24,
	MEDIA_BUS_FMT_RGB666_1X18,
	MEDIA_BUS_FMT_RGB565_1X16,
};

struct sit_panel {
	struct drm_panel base;
	struct mipi_dsi_device *dsi;

	struct gpio_desc *reset;

	bool prepared;
	bool enabled;

	struct videomode vm;
	u32 width_mm;
	u32 height_mm;
};

static inline struct sit_panel *to_sit_panel(struct drm_panel *panel)
{
	return container_of(panel, struct sit_panel, base);
}

static inline int dsi_write(struct sit_panel *panel, const void *seq, size_t len)
{
	return mipi_dsi_generic_write(panel->dsi, seq, len);
}

#define DSI_WRITE(panel, seq...)						\
	{													\
		int ret = 0;									\
		const u8 d[] = { seq };							\
		ret = dsi_write(panel, d, ARRAY_SIZE(d));		\
		if(ret < 0)										\
			return ret;									\
	}													\

/*
 *This function can be used to print integer in binary 8 bit
 *For 16bit: printf("m: "BYTE_TO_BINARY_PATTERN" "BYTE_TO_BINARY_PATTERN"\n",
 *  BYTE_TO_BINARY(m>>8), BYTE_TO_BINARY(m));
 */
#define BYTE_TO_BINARY_PATTERN "%c%c%c%c%c%c%c%c"
#define BYTE_TO_BINARY(byte)  \
  (byte & 0x80 ? '1' : '0'), \
  (byte & 0x40 ? '1' : '0'), \
  (byte & 0x20 ? '1' : '0'), \
  (byte & 0x10 ? '1' : '0'), \
  (byte & 0x08 ? '1' : '0'), \
  (byte & 0x04 ? '1' : '0'), \
  (byte & 0x02 ? '1' : '0'), \
  (byte & 0x01 ? '1' : '0') 

/*
 * HW resets display ST7701S
 */
static int reset_display(struct sit_panel *sit)
{
	if (sit->reset == NULL)
		return -1;

	gpiod_set_value(sit->reset, 1);
	udelay(10);
	/* reset the panel */
	gpiod_set_value(sit->reset, 0);
	/* assert reset */
	udelay(10);
	gpiod_set_value(sit->reset, 1);
	/* wait after releasing reset */
	msleep(200);    //from init docu
	sit->reset = NULL;

	return 0;
}

/*
 * manufacturer command set (WriteComm/WriteData) from oricdisplay.com
 * #defines from original display driver st7701 (Jagan Teki)
 */
static int init_sequence(struct sit_panel *panel)
{
	DSI_WRITE(panel, MIPI_DCS_EXIT_SLEEP_MODE, 0x00);

	msleep(120);

	/* CN2=’1’ enable the BK function of Command2 and select BK3 */
	DSI_WRITE(panel, DSI_CMD2BKX_SEL, 0x77, 0x01, 0x00, 0x00, 0x13);    // TODO check if it is necessary

	/* this is a confidential command from vendor */
	DSI_WRITE(panel, WRMAUCCTR, 0x08);									// TODO check if it is necessary

	/* select Command2, BK0 */
	DSI_WRITE(panel, DSI_CMD2BKX_SEL, 0x77, 0x01, 0x00, 0x00, DSI_CMD2BK0_SEL);
	DSI_WRITE(panel, DSI_CMD2_BK0_LNESET, DSI_CMD2_BK0_LNESET_B0, DSI_CMD2_BK0_LNESET_B1);
	DSI_WRITE(panel, DSI_CMD2_BK0_PORCTRL, DSI_CMD2_BK0_PORCTRL_B0, DSI_CMD2_BK0_PORCTRL_B1);
	DSI_WRITE(panel, DSI_CMD2_BK0_INVSEL, 0x07, 0x0A);
	DSI_WRITE(panel, DSI_CMD2_BK0_RGBCTRL, 0x02, 0x00, 0x00);
	DSI_WRITE(panel, DSI_CMD2_BK0_SDIR, 0x00);
	DSI_WRITE(panel, 0xCC, 0x10);										// TODO check if it is necessary
	DSI_WRITE(panel, DSI_CMD2_BK0_PVGAMCTRL, 0x05, 0x12, 0x98, 0x0E, 0x0F, 0x07, 0x07, 0x09, 0x09, 0x23, 0x05, 0x52, 0x0F,  0x67, 0x2C, 0x11);
	DSI_WRITE(panel, DSI_CMD2_BK0_NVGAMCTRL, 0x0B, 0x11, 0x97, 0x0C, 0x12, 0x06, 0x06, 0x08, 0x08, 0x22, 0x03, 0x51, 0x11,  0x66, 0x2B, 0x0F);

	/* select Command2, BK1 */
	DSI_WRITE(panel, DSI_CMD2BKX_SEL, 0x77, 0x01, 0x00, 0x00, DSI_CMD2BK1_SEL);
	DSI_WRITE(panel, DSI_CMD2_BK1_VRHS, DSI_CMD2_BK1_VRHA_SET);
	DSI_WRITE(panel, DSI_CMD2_BK1_VCOM, DSI_CMD2_BK1_VCOM_SET);
	DSI_WRITE(panel, DSI_CMD2_BK1_VGHSS, 0x81);
	DSI_WRITE(panel, DSI_CMD2_BK1_TESTCMD, DSI_CMD2_BK1_TESTCMD_VAL);
	DSI_WRITE(panel, DSI_CMD2_BK1_VGLS, DSI_CMD2_BK1_VGLS_SET);
	DSI_WRITE(panel, DSI_CMD2_BK1_PWCTLR1, 0x85);
	DSI_WRITE(panel, DSI_CMD2_BK1_PWCTLR2, 0x20);
	DSI_WRITE(panel, DSI_CMD2_BK1_SPD1, 0x78);
	DSI_WRITE(panel, DSI_CMD2_BK1_SPD2, 0x78);
	DSI_WRITE(panel, DSI_CMD2_BK1_MIPISET1, 0x88);

	/**
	 * ST7701_SPEC_V1.2 is unable to provide enough information above this
	 * specific command sequence, so grab the same from vendor BSP driver.
	 */
	DSI_WRITE(panel, 0xE0, 0x00, 0x00, 0x02);
	DSI_WRITE(panel, 0xE1, 0x06, 0x30, 0x08, 0x30, 0x05, 0x30, 
			0x07, 0x30, 0x00, 0x33, 0x33);
	DSI_WRITE(panel, 0xE2,  0x11, 0x11, 0x33, 0x33, 0xF4, 0x00, 
			0x00, 0x00, 0xF4, 0x00, 0x00, 0x00);
	DSI_WRITE(panel, 0xE3, 0x00, 0x00, 0x11, 0x11);
	DSI_WRITE(panel, 0xE4, 0x44, 0x44);
	DSI_WRITE(panel, 0xE5, 0x0D, 0xF5, 0x30, 0xF0, 0x0F, 0xF7, 
			0x30, 0xF0, 0x09, 0xF1, 0x30, 0xF0, 0x0B, 0xF3, 0x30, 0xF0);
	DSI_WRITE(panel, 0xE6,  0x00, 0x00, 0x11, 0x11);
	DSI_WRITE(panel, 0xE7, 0x44, 0x44);
	DSI_WRITE(panel, 0xE8, 0x0C, 0xF4, 0x30, 0xF0, 0x0E, 0xF6, 
			0x30, 0xF0, 0x08, 0xF0, 0x30, 0xF0, 0x0A, 0xF2, 0x30, 0xF0);
	DSI_WRITE(panel, 0xE9, 0x36, 0x01);
	DSI_WRITE(panel, 0xEB, 0x00, 0x01, 0xE4, 0xE4, 0x44, 0x88, 0x40);
	DSI_WRITE(panel, 0xED, 0xFF, 0x10, 0xAF, 0x76, 0x54, 0x2B, 0xCF, 
			0xFF, 0xFF, 0xFC, 0xB2, 0x45, 0x67, 0xFA, 0x01, 0xFF);
	DSI_WRITE(panel, 0xEF, 0x08, 0x08, 0x08, 0x45, 0x3F, 0x54);

	/* deactivate Command2 */
	DSI_WRITE(panel, DSI_CMD2BKX_SEL,0x77, 0x01, 0x00, 0x00, DSI_CMD2BKX_SEL_NONE); 
	DSI_WRITE(panel, 0x11, 0x00); // turns off sleep mode

	msleep (120);

	/* defines the pixel format for RGB interface --> 0x70 = 0111 0000 -> “111”=24-bit/pixel */
	DSI_WRITE(panel, DSI_CMD_COLMOD, 0x70);
	DSI_WRITE(panel, DSI_CMD_TEON, 0x00);					// TODO check if it is necessary or rather with DSI_CMD_TEOFF 
	/* scan direction normel and RGB */
	DSI_WRITE(panel, DSI_CMD_MADCTL, 0x00);


	DSI_WRITE(panel, MIPI_DCS_SET_DISPLAY_ON, 0x00);

	msleep(120);

	return 0;
}

static int sit_panel_prepare(struct drm_panel *panel)
{
	struct sit_panel *sit = to_sit_panel(panel);

	if (sit->prepared)
		return 0;

	if (sit->reset != NULL) {
		reset_display(sit);
		msleep(200);	//from init docu
	}

	sit->prepared = true;

	return 0;
}

static int sit_panel_enable(struct drm_panel *panel)
{
	struct sit_panel *sit = to_sit_panel(panel);
	struct mipi_dsi_device *dsi = sit->dsi;
	struct device *dev = &dsi->dev;
	int ret;

	if (sit->enabled)
		return 0;

	if (!sit->prepared) {
		dev_err(dev, "Panel not prepared!\n");
		return -EPERM;
	}

	reset_display(sit);

	sit->dsi->mode_flags |= MIPI_DSI_MODE_LPM;

	ret = init_sequence(sit);

	if (ret < 0) {
		dev_err(dev, "Failed to send MCS (%d)\n", ret);
		goto fail;
	}

	sit->enabled = true;

	return 0;

fail:
	if (sit->reset != NULL) {
		reset_display(sit);
	}

	return ret;
}

static int sit_panel_unprepare(struct drm_panel *panel)
{
	struct sit_panel *sit = to_sit_panel(panel);
	struct device *dev = &sit->dsi->dev;

	if (!sit->prepared)
		return 0;

	if (sit->enabled) {
		dev_err(dev, "Panel still enabled!\n");
		return -EPERM;
	}

	if (sit->reset != NULL)
		reset_display(sit);

	msleep(120);
	sit->prepared = false;

	return 0;
}

static int sit_panel_disable(struct drm_panel *panel)
{
	struct sit_panel *sit = to_sit_panel(panel);

	if (!sit->enabled)
		return 0;

	sit->dsi->mode_flags |= MIPI_DSI_MODE_LPM;

	msleep(120);
	DSI_WRITE(sit, MIPI_DCS_SET_DISPLAY_OFF, 0x00);
	msleep(120);
	DSI_WRITE(sit, MIPI_DCS_ENTER_SLEEP_MODE, 0x00);
	msleep(120);

	sit->enabled = false;

	return 0;
}

static int sit_panel_get_modes(struct drm_panel *panel, struct drm_connector *connector)
{
	struct sit_panel *sit = to_sit_panel(panel);
	struct device *dev = &sit->dsi->dev;
	struct drm_display_mode *mode;
	u32 *bus_flags = &connector->display_info.bus_flags;
	int ret;

	mode = drm_mode_create(connector->dev);
	if (!mode) {
		dev_err(dev, "Failed to create display mode!\n");
		return 0;
	}

	drm_display_mode_from_videomode(&sit->vm, mode);
	mode->width_mm = sit->width_mm;
	mode->height_mm = sit->height_mm;
	connector->display_info.width_mm = sit->width_mm;
	connector->display_info.height_mm = sit->height_mm;
	mode->type = DRM_MODE_TYPE_DRIVER | DRM_MODE_TYPE_PREFERRED;

	if (sit->vm.flags & DISPLAY_FLAGS_DE_HIGH)
		*bus_flags |= DRM_BUS_FLAG_DE_HIGH;
	if (sit->vm.flags & DISPLAY_FLAGS_DE_LOW)
		*bus_flags |= DRM_BUS_FLAG_DE_LOW;
	if (sit->vm.flags & DISPLAY_FLAGS_PIXDATA_NEGEDGE)
        *bus_flags |= DRM_BUS_FLAG_PIXDATA_DRIVE_NEGEDGE;
	if (sit->vm.flags & DISPLAY_FLAGS_PIXDATA_POSEDGE)
        *bus_flags |= DRM_BUS_FLAG_PIXDATA_DRIVE_POSEDGE;

	ret = drm_display_info_set_bus_formats(&connector->display_info, sit_bus_formats, ARRAY_SIZE(sit_bus_formats));

	if (ret)
		return ret;

	drm_mode_probed_add(connector, mode);

	return 1;
}

static const struct drm_panel_funcs sit_panel_funcs = {
	.prepare = sit_panel_prepare,
	.unprepare = sit_panel_unprepare,
	.enable = sit_panel_enable,
	.disable = sit_panel_disable,
	.get_modes = sit_panel_get_modes,
};

/*
 * refresh_rate = 60 Hz
 * pixelclock =
 *     (hactive  hfront_porch  hsync_len  hback_porch) * 
 *     (vactive  vfront_porch  vsync_len  vback_porch) * 
 *     refresh_rate = 17,47584×10⁶
 */
static const struct display_timing sit_default_timing = {
	//pixelclock = (480+20+10+54)*(480+4+10+60)*60 = 18,74736×10⁶
	//set by DMB:
	.pixelclock = { 20000000, 20000000, 20000000 },
	.hactive = { 480, 480, 480 },
	.hfront_porch = { 20, 20, 20 },
	.hsync_len = { 10, 10, 10 },
	.hback_porch = { 54, 54, 54 },
	.vactive = { 480, 480, 480 },
	.vfront_porch = { 40, 40, 40 },
	.vsync_len = { 10, 10, 10 },
	.vback_porch = { 60, 60, 60 },
	.flags = DISPLAY_FLAGS_HSYNC_LOW |
			 DISPLAY_FLAGS_VSYNC_LOW |
			 DISPLAY_FLAGS_DE_LOW |
			 DISPLAY_FLAGS_PIXDATA_NEGEDGE,
};

static int sit_panel_probe(struct mipi_dsi_device *dsi)
{
	struct device *dev = &dsi->dev;
	struct device_node *np = dev->of_node;
	struct device_node *timings;
	struct sit_panel *panel;
	int ret;
	u32 video_mode;

	panel = devm_kzalloc(&dsi->dev, sizeof(*panel), GFP_KERNEL);
	if (!panel)
		return -ENOMEM;

	mipi_dsi_set_drvdata(dsi, panel);

	panel->dsi = dsi;

	dsi->format = MIPI_DSI_FMT_RGB888;
	dsi->mode_flags =  MIPI_DSI_MODE_VIDEO_HSE | MIPI_DSI_MODE_VIDEO | MIPI_DSI_CLOCK_NON_CONTINUOUS;

	ret = of_property_read_u32(np, "video-mode", &video_mode);
	if (!ret) {
		switch (video_mode) {
		case 0:
			/* burst mode */
			dsi->mode_flags |= MIPI_DSI_MODE_VIDEO_BURST;
			break;
		case 1:
			/* non-burst mode with sync event */
			break;
		case 2:
			/* non-burst mode with sync pulse */
			dsi->mode_flags |= MIPI_DSI_MODE_VIDEO_SYNC_PULSE;
			break;
		default:
			dev_warn(dev, "invalid video mode %d\n", video_mode);
			break;

		}
	}

	ret = of_property_read_u32(np, "dsi-lanes", &dsi->lanes);
	if (ret < 0) {
		dev_err(dev, "Failed to get dsi-lanes property (%d)\n", ret);
		return ret;
	}

	/*
	 * 'display-timings' is optional, so verify if the node is present
	 * before calling of_get_videomode so we won't get console error
	 * messages
	 */
	timings = of_get_child_by_name(np, "display-timings");
	if (timings) {
		of_node_put(timings);
		ret = of_get_videomode(np, &panel->vm, 0);
	} else {
		videomode_from_timing(&sit_default_timing, &panel->vm);
	}

	if (ret < 0)
		return ret;

	of_property_read_u32(np, "panel-width-mm", &panel->width_mm);
	of_property_read_u32(np, "panel-height-mm", &panel->height_mm);

	panel->reset = devm_gpiod_get(dev, "reset", GPIOD_OUT_HIGH);

	if (IS_ERR(panel->reset)) {
		panel->reset = NULL;
	} else {
		gpiod_set_value(panel->reset, 1);
		udelay(10);
		/* reset the panel */
		gpiod_set_value(panel->reset, 0);
		/* assert reset */
		udelay(10);
		gpiod_set_value(panel->reset, 1);
		/* wait after releasing reset */
		usleep_range(5000, 10000);
	}

	drm_panel_init(&panel->base, dev, &sit_panel_funcs,	DRM_MODE_CONNECTOR_DSI);

	ret = drm_panel_of_backlight(&panel->base);
	if (ret) {
		dev_err(dev, "%s: error %d initializing backlight from dt",
				__func__, ret);
		return ret;
	}

	panel->base.funcs = &sit_panel_funcs;
	panel->base.dev = dev;
	dev_set_drvdata(dev, panel);

	drm_panel_add(&panel->base);

	if (ret < 0)
 		return ret;

	ret = mipi_dsi_attach(dsi);
	if (ret < 0)
		drm_panel_remove(&panel->base);

	return ret;
}

static int sit_panel_remove(struct mipi_dsi_device *dsi)
{
	struct sit_panel *sit = mipi_dsi_get_drvdata(dsi);
	struct device *dev = &dsi->dev;
	int ret;

	ret = mipi_dsi_detach(dsi);
	if (ret < 0)
		dev_err(dev, "Failed to detach from host (%d)\n", ret);

	drm_panel_remove(&sit->base);

	return 0;
}

static void sit_panel_shutdown(struct mipi_dsi_device *dsi)
{
	struct sit_panel *sit = mipi_dsi_get_drvdata(dsi);

	sit_panel_disable(&sit->base);
	sit_panel_unprepare(&sit->base);
}

#ifdef CONFIG_PM
static int sit_panel_suspend(struct device *dev)
{
	struct sit_panel *sit = dev_get_drvdata(dev);

	if (!sit->reset)
		return 0;

	devm_gpiod_put(dev, sit->reset);
	sit->reset = NULL;

	return 0;
}

static int sit_panel_resume(struct device *dev)
{
	struct sit_panel *sit = dev_get_drvdata(dev);

	if (sit->reset)
		return 0;

	sit->reset = devm_gpiod_get(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(sit->reset))
		sit->reset = NULL;

	return PTR_ERR_OR_ZERO(sit->reset);
}
#endif

static const struct dev_pm_ops sit_pm_ops = {
	SET_RUNTIME_PM_OPS(sit_panel_suspend, sit_panel_resume, NULL)
	SET_SYSTEM_SLEEP_PM_OPS(sit_panel_suspend, sit_panel_resume)
};

static const struct of_device_id sit_of_match[] = {
	{ .compatible = "sitronix,st7701", },
	{ }
};
MODULE_DEVICE_TABLE(of, sit_of_match);

static struct mipi_dsi_driver sit_panel_driver = {
	.driver = {
		.name = "panel-sitronix-st7701",
		.of_match_table = sit_of_match,
		.pm	= &sit_pm_ops,
	},
	.probe = sit_panel_probe,
	.remove = sit_panel_remove,
	.shutdown = sit_panel_shutdown,
};
module_mipi_dsi_driver(sit_panel_driver);

MODULE_AUTHOR("Goran Stankovic <goran.stankovic@magnosco.com>");
MODULE_DESCRIPTION("DRM Driver for Sitronix ST7701 MIPI DSI panel");
MODULE_LICENSE("GPL v2");
