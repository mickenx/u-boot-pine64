/*
 * Copyright (c) 2017 Vasily Khoruzhick <anarsoul@gmail.com>
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#include <common.h>
#include <div64.h>
#include <dm.h>
#include <pwm.h>
#include <regmap.h>
#include <syscon.h>
#include <asm/io.h>
#include <asm/arch/pwm.h>
#include <asm/arch/gpio.h>
#include <power/regulator.h>

DECLARE_GLOBAL_DATA_PTR;

struct sunxi_pwm_priv {
	struct sunxi_pwm *regs;
	ulong freq;
	bool invert;
	uint32_t prescaler;
};

static const uint32_t prescaler_table[] = {
	120,	/* 0000 */
	180,	/* 0001 */
	240,	/* 0010 */
	360,	/* 0011 */
	480,	/* 0100 */
	0,	/* 0101 */
	0,	/* 0110 */
	0,	/* 0111 */
	12000,	/* 1000 */
	24000,	/* 1001 */
	36000,	/* 1010 */
	48000,	/* 1011 */
	72000,	/* 1100 */
	0,	/* 1101 */
	0,	/* 1110 */
	1,	/* 1111 */
};

static const uint64_t nsecs_per_sec = 1000000000L;

static int sunxi_pwm_config_pinmux(void)
{
#ifdef CONFIG_MACH_SUN50I
	sunxi_gpio_set_cfgpin(SUNXI_GPD(22), SUNXI_GPD_PWM);
#endif
	return 0;
}

static int sunxi_pwm_set_invert(struct udevice *dev, uint channel, bool polarity)
{
	struct sunxi_pwm_priv *priv = dev_get_priv(dev);

	debug("%s: polarity=%u\n", __func__, polarity);
	priv->invert = polarity;

	return 0;
}

static int sunxi_pwm_set_config(struct udevice *dev, uint channel, uint period_ns,
			     uint duty_ns)
{
	struct sunxi_pwm_priv *priv = dev_get_priv(dev);
	struct sunxi_pwm *regs = priv->regs;
	int prescaler;
	u32 v, period, duty;
	uint64_t div = 0, pval = 0, scaled_freq = 0;

	debug("%s: period_ns=%u, duty_ns=%u\n", __func__, period_ns, duty_ns);

	for (prescaler = 0; prescaler < SUNXI_PWM_CTRL_PRESCALE0_MASK; prescaler++) {
		if (!prescaler_table[prescaler])
			continue;
		div = priv->freq;
		pval = prescaler_table[prescaler];
		scaled_freq = lldiv(div, pval);
		div = scaled_freq * period_ns;
		div = lldiv(div, nsecs_per_sec);
		if (div - 1 <= SUNXI_PWM_CH0_PERIOD_MAX)
			break;
	}

	if (div - 1 > SUNXI_PWM_CH0_PERIOD_MAX) {
		debug("%s: failed to find prescaler value\n", __func__);
		return -EINVAL;
	}

	period = div;
	div = scaled_freq * duty_ns;
	div = lldiv(div, nsecs_per_sec);
	duty = div;

	if (priv->prescaler != prescaler) {
		/* Mask clock to update prescaler */
		v = readl(&regs->ctrl);
		v &= ~SUNXI_PWM_CTRL_CLK_GATE;
		writel(v, &regs->ctrl);
		v &= ~SUNXI_PWM_CTRL_PRESCALE0_MASK;
		v |= (priv->prescaler & SUNXI_PWM_CTRL_PRESCALE0_MASK);
		writel(v, &regs->ctrl);
		v |= SUNXI_PWM_CTRL_CLK_GATE;
		writel(v, &regs->ctrl);
		priv->prescaler = prescaler;
	}

	writel(SUNXI_PWM_CH0_PERIOD_PRD(period) |
	       SUNXI_PWM_CH0_PERIOD_DUTY(duty), &regs->ch0_period);

	debug("%s: prescaler: %d, period: %d, duty: %d\n", __func__, priv->prescaler,
	      period, duty);

	return 0;
}

static int sunxi_pwm_set_enable(struct udevice *dev, uint channel, bool enable)
{
	struct sunxi_pwm_priv *priv = dev_get_priv(dev);
	struct sunxi_pwm *regs = priv->regs;
	uint32_t v;

	debug("%s: Enable '%s'\n", __func__, dev->name);

	v = readl(&regs->ctrl);
	if (!enable) {
		v &= ~SUNXI_PWM_CTRL_ENABLE0;
		writel(v, &regs->ctrl);
		return 0;
	}

	sunxi_pwm_config_pinmux();

	v &= ~SUNXI_PWM_CTRL_POLARITY0_MASK;
	v |= priv->invert ? SUNXI_PWM_CTRL_POLARITY0(0) :
		      SUNXI_PWM_CTRL_POLARITY0(1);
	v |= SUNXI_PWM_CTRL_ENABLE0;
	writel(v, &regs->ctrl);

	return 0;
}

static int sunxi_pwm_ofdata_to_platdata(struct udevice *dev)
{
	struct sunxi_pwm_priv *priv = dev_get_priv(dev);

	priv->regs = (struct sunxi_pwm *)devfdt_get_addr(dev);

	return 0;
}

static int sunxi_pwm_probe(struct udevice *dev)
{
	struct sunxi_pwm_priv *priv = dev_get_priv(dev);

	priv->freq = 24000000;

	return 0;
}

static const struct pwm_ops sunxi_pwm_ops = {
	.set_invert	= sunxi_pwm_set_invert,
	.set_config	= sunxi_pwm_set_config,
	.set_enable	= sunxi_pwm_set_enable,
};

static const struct udevice_id sunxi_pwm_ids[] = {
	{ .compatible = "allwinner,sun8i-h3-pwm" },
	{ }
};

U_BOOT_DRIVER(sunxi_pwm) = {
	.name	= "sunxi_pwm",
	.id	= UCLASS_PWM,
	.of_match = sunxi_pwm_ids,
	.ops	= &sunxi_pwm_ops,
	.ofdata_to_platdata	= sunxi_pwm_ofdata_to_platdata,
	.probe		= sunxi_pwm_probe,
	.priv_auto_alloc_size	= sizeof(struct sunxi_pwm_priv),
};
