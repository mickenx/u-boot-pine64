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
#include <video_bridge.h>
#include <dm.h>
#include <asm/io.h>
#include <asm/arch/clock.h>
#include <asm/arch/lcdc.h>
#include <asm/arch/gpio.h>
#include <asm/gpio.h>

static void sunxi_lcdc_backlight_enable(void)
{
	int pin;

#define CONFIG_VIDEO_LCD_BL_EN "PD23"
#define CONFIG_VIDEO_LCD_BL_PWM "PD22"
	/*
	 * We want to have scanned out at least one frame before enabling the
	 * backlight to avoid the screen flashing to white when we enable it.
	 */
	mdelay(40);

	pin = sunxi_name_to_gpio(CONFIG_VIDEO_LCD_BL_EN);
	gpio_request(pin, "bl enable");
	gpio_direction_output(pin, 0);
	if (pin >= 0)
		gpio_direction_output(pin, 1);

	pin = sunxi_name_to_gpio(CONFIG_VIDEO_LCD_BL_PWM);
	gpio_request(pin, "bl pwm");
	gpio_direction_output(pin, 0);
	if (pin >= 0)
		gpio_direction_output(pin, 1);
}

static void sunxi_lcdc_pll_set(int tcon, int dotclock,
			       int *clk_div, int *clk_double)
{
	struct sunxi_ccm_reg * const ccm =
		(struct sunxi_ccm_reg *)SUNXI_CCM_BASE;
	int value, n, m, min_m, max_m, diff;
	int best_n = 0, best_m = 0, best_diff = 0x0FFFFFFF;
	int best_double = 0;
	bool use_mipi_pll = false;

	if (tcon == 0) {
		min_m = 6;
		max_m = 127;
	} else {
		min_m = 1;
		max_m = 15;
	}

	/*
	 * Find the lowest divider resulting in a matching clock, if there
	 * is no match, pick the closest lower clock, as monitors tend to
	 * not sync to higher frequencies.
	 */
	for (m = min_m; m <= max_m; m++) {
#if 0
		n = (m * dotclock) / 3000;

		if ((n >= 9) && (n <= 127)) {
			value = (3000 * n) / m;
			diff = dotclock - value;
			if (diff < best_diff) {
				best_diff = diff;
				best_m = m;
				best_n = n;
				best_double = 0;
			}
		}

		/* These are just duplicates */
		if (!(m & 1))
			continue;
#endif

		/* No double clock on DE2 */
		n = (m * dotclock) / 6000;
		if ((n >= 9) && (n <= 127)) {
			value = (6000 * n) / m;
			diff = dotclock - value;
			if (diff < best_diff) {
				best_diff = diff;
				best_m = m;
				best_n = n;
				best_double = 1;
			}
		}
	}

#ifdef CONFIG_MACH_SUN6I
	/*
	 * Use the MIPI pll if we've been unable to find any matching setting
	 * for PLL3, this happens with high dotclocks because of min_m = 6.
	 */
	if (tcon == 0 && best_n == 0) {
		use_mipi_pll = true;
		best_m = 6;  /* Minimum m for tcon0 */
	}

	if (use_mipi_pll) {
		clock_set_pll3(297000000); /* Fix the video pll at 297 MHz */
		clock_set_mipi_pll(best_m * dotclock * 1000);
		debug("dotclock: %dkHz = %dkHz via mipi pll\n",
		      dotclock, clock_get_mipi_pll() / best_m / 1000);
	} else
#endif
	{
		clock_set_pll3(best_n * 3000000);
		debug("dotclock: %dkHz = %dkHz: (%d * 3MHz * %d) / %d\n",
		      dotclock,
		      (best_double + 1) * clock_get_pll3() / best_m / 1000,
		      best_double + 1, best_n, best_m);
	}

	if (tcon == 0) {
		u32 pll;

		if (use_mipi_pll)
			pll = CCM_LCD_CH0_CTRL_MIPI_PLL;
		else if (best_double)
			pll = CCM_LCD_CH0_CTRL_PLL3_2X;
		else
			pll = CCM_LCD_CH0_CTRL_PLL3;

		writel(CCM_LCD_CH0_CTRL_GATE | CCM_LCD_CH0_CTRL_RST | pll,
		       &ccm->lcd0_clk_cfg);
	} 
#if 0
	else {
		writel(CCM_LCD_CH1_CTRL_GATE |
		       (best_double ? CCM_LCD_CH1_CTRL_PLL3_2X :
				      CCM_LCD_CH1_CTRL_PLL3) |
		       CCM_LCD_CH1_CTRL_M(best_m), &ccm->lcd0_ch1_clk_cfg);
		if (sunxi_is_composite())
			setbits_le32(&ccm->lcd0_ch1_clk_cfg,
				     CCM_LCD_CH1_CTRL_HALF_SCLK1);
	}
#endif

	*clk_div = best_m;
	*clk_double = best_double;
}

static void sunxi_lcdc_tcon0_mode_set(const struct display_timing *edid)
{
	struct sunxi_lcdc_reg * const lcdc =
		(struct sunxi_lcdc_reg *)SUNXI_LCD0_BASE;
	int clk_div, clk_double, pin;

#if defined CONFIG_MACH_SUN8I && defined CONFIG_VIDEO_LCD_IF_LVDS
	for (pin = SUNXI_GPD(18); pin <= SUNXI_GPD(27); pin++) {
#elif defined CONFIG_MACH_SUN50I
	for (pin = SUNXI_GPD(0); pin <= SUNXI_GPD(21); pin++) {
#else
	for (pin = SUNXI_GPD(0); pin <= SUNXI_GPD(27); pin++) {
#endif
		sunxi_gpio_set_cfgpin(pin, SUNXI_GPD_LCD0);
		sunxi_gpio_set_drv(pin, 3);
	}

	debug("pixelclock: %d\n", edid->pixelclock.typ);
	sunxi_lcdc_pll_set(0, edid->pixelclock.typ / 1000, &clk_div, &clk_double);

	lcdc_tcon0_mode_set(lcdc, edid, clk_div, false,
			    18 /* bpp - FIXME */, 1 /*CONFIG_VIDEO_LCD_DCLK_PHASE*/);
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

	lcdc_init(lcdc);
	sunxi_lcdc_tcon0_mode_set(edid);
	lcdc_enable(lcdc, bpp);
	/* TODO: backlight is another driver */
	sunxi_lcdc_backlight_enable();
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

	// enable bridge 
	ret = uclass_get_device(UCLASS_VIDEO_BRIDGE, 0, &bridge_dev);
	if (ret) {
		debug("video bridge init failed: %d\n", ret);
		return ret;
	}

	video_bridge_attach(bridge_dev);
	printf("panel bpp: %d\n", panel_bpp);
	sunxi_lcd_lcdc_init(edid, 18);

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
