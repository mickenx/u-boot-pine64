#define DEBUG 1

/*
 * Allwinner LCD driver
 *
 * (C) Copyright 2017 Vasily Khoruzhick <anarsoul@gmail.com>
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#include <common.h>
#include <display.h>
#include <dm.h>
#include <asm/io.h>
#include <asm/arch/clock.h>
#include <asm/arch/lcdc.h>
#include <asm/arch/gpio.h>
#include <asm/gpio.h>
#include "../anx9804.h"

static void sunxi_lcdc_panel_enable(void)
{
#if 0
	int pin, reset_pin;

	/*
	 * Start with backlight disabled to avoid the screen flashing to
	 * white while the lcd inits.
	 */
	pin = sunxi_name_to_gpio(CONFIG_VIDEO_LCD_BL_EN);
	if (pin >= 0) {
		gpio_request(pin, "lcd_backlight_enable");
		gpio_direction_output(pin, 0);
	}

	pin = sunxi_name_to_gpio(CONFIG_VIDEO_LCD_BL_PWM);
	if (pin >= 0) {
		gpio_request(pin, "lcd_backlight_pwm");
		gpio_direction_output(pin, PWM_OFF);
	}

	reset_pin = sunxi_name_to_gpio(CONFIG_VIDEO_LCD_RESET);
	if (reset_pin >= 0) {
		gpio_request(reset_pin, "lcd_reset");
		gpio_direction_output(reset_pin, 0); /* Assert reset */
	}

	/* Give the backlight some time to turn off and power up the panel. */
	mdelay(40);
	pin = sunxi_name_to_gpio(CONFIG_VIDEO_LCD_POWER);
	if (pin >= 0) {
		gpio_request(pin, "lcd_power");
		gpio_direction_output(pin, 1);
	}

	if (reset_pin >= 0)
		gpio_direction_output(reset_pin, 1); /* De-assert reset */
#endif
}

static void sunxi_lcdc_backlight_enable(void)
{
#if 0
	int pin;

	/*
	 * We want to have scanned out at least one frame before enabling the
	 * backlight to avoid the screen flashing to white when we enable it.
	 */
	mdelay(40);

	pin = sunxi_name_to_gpio(CONFIG_VIDEO_LCD_BL_EN);
	if (pin >= 0)
		gpio_direction_output(pin, 1);

	pin = sunxi_name_to_gpio(CONFIG_VIDEO_LCD_BL_PWM);
#ifdef SUNXI_PWM_PIN0
	if (pin == SUNXI_PWM_PIN0) {
		writel(SUNXI_PWM_CTRL_POLARITY0(PWM_ON) |
		       SUNXI_PWM_CTRL_ENABLE0 |
		       SUNXI_PWM_CTRL_PRESCALE0(0xf), SUNXI_PWM_CTRL_REG);
		writel(SUNXI_PWM_PERIOD_80PCT, SUNXI_PWM_CH0_PERIOD);
		sunxi_gpio_set_cfgpin(pin, SUNXI_PWM_MUX);
		return;
	}
#endif
	if (pin >= 0)
		gpio_direction_output(pin, PWM_ON);
#endif
}

static void sunxi_lcd_lcdc_init(const struct display_timing *edid, int bpp)
{
	struct sunxi_ccm_reg * const ccm =
	       (struct sunxi_ccm_reg *)SUNXI_CCM_BASE;
	struct sunxi_lcdc_reg * const lcdc =
	       (struct sunxi_lcdc_reg *)SUNXI_LCD0_BASE;

	/* Reset off */
	setbits_le32(&ccm->ahb_reset1_cfg, 1 << AHB_RESET_OFFSET_LCD0);

	/* Clock on */
	setbits_le32(&ccm->ahb_gate1, 1 << AHB_GATE_OFFSET_LCD0);

	/*
	lcdc_init(lcdc);
	lcdc_tcon0_mode_set(lcdc, edid, false, true);
	lcdc_enable(lcdc, bpp);
	sunxi_lcdc_backlight_enable();
	*/
}

static int sunxi_lcd_read_timing(struct udevice *dev,
                                struct display_timing *timing)
{
	timing->pixelclock.typ = 72000000;

	timing->hactive.typ = 1366;
	timing->hfront_porch.typ = 70;
	timing->hback_porch.typ = 10;
	timing->hsync_len.typ = 54;

	timing->vactive.typ = 768;
	timing->vfront_porch.typ = 12;
	timing->vback_porch.typ = 15;
	timing->vsync_len.typ = 5;
	
	timing->flags |= DISPLAY_FLAGS_HSYNC_HIGH;
	timing->flags |= DISPLAY_FLAGS_VSYNC_HIGH;

	return 0;
}

static int sunxi_lcd_enable(struct udevice *dev, int panel_bpp,
                           const struct display_timing *edid)
{
	int ret;
	struct udevice *bridge_dev;
	// enable panel
	//sunxi_lcdc_panel_enable();

	// enable bridge 
	ret = uclass_get_device(UCLASS_VIDEO_BRIDGE, 0, &bridge_dev);
	if (ret) {
		debug("video bridge init failed: %d\n", ret);
		return ret;
	}

	video_bridge_attach(bridge_dev);
	//anx9804_init(CONFIG_VIDEO_LCD_I2C_BUS, CONFIG_ANX_LANE_NUM,
	//	     ANX9804_DATA_RATE_1620M,
	//	     18);
	//sunxi_lcd_lcdc_init(edid, panel_bpp);

	return 0;
}

static int sunxi_lcd_probe(struct udevice *dev)
{
	/* make sure that clock is active */
	clock_set_pll10(432000000);

	return 0;
}

static const struct dm_display_ops sunxi_lcd_ops = {
       .read_timing = sunxi_lcd_read_timing,
       .enable = sunxi_lcd_enable,
};

U_BOOT_DRIVER(sunxi_lcd) = {
       .name   = "sunxi_lcd",
       .id     = UCLASS_DISPLAY,
       .ops    = &sunxi_lcd_ops,
       .probe  = sunxi_lcd_probe,
};

U_BOOT_DEVICE(sunxi_lcd) = {
       .name = "sunxi_lcd"
};
