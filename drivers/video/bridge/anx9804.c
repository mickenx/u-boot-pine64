#define DEBUG 1
/*
 * Copyright (C) 2017 Vasily Khoruzhick <anarsoul@gmail.com>
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#include <common.h>
#include <dm.h>
#include <errno.h>
#include <i2c.h>
#include <video_bridge.h>

DECLARE_GLOBAL_DATA_PTR;

/* Registers at i2c address 0x38 */

#define ANX9804_HDCP_CONTROL_0_REG				0x01

#define ANX9804_SYS_CTRL1_REG					0x80
#define ANX9804_SYS_CTRL1_PD_IO					0x80
#define ANX9804_SYS_CTRL1_PD_VID				0x40
#define ANX9804_SYS_CTRL1_PD_LINK				0x20
#define ANX9804_SYS_CTRL1_PD_TOTAL				0x10
#define ANX9804_SYS_CTRL1_MODE_SEL				0x08
#define ANX9804_SYS_CTRL1_DET_STA				0x04
#define ANX9804_SYS_CTRL1_FORCE_DET				0x02
#define ANX9804_SYS_CTRL1_DET_CTRL				0x01

#define ANX9804_SYS_CTRL2_REG					0x81
#define ANX9804_SYS_CTRL2_CHA_STA				0x04

#define ANX9804_SYS_CTRL3_REG					0x82
#define ANX9804_SYS_CTRL3_VALID_CTRL				BIT(0)
#define ANX9804_SYS_CTRL3_F_VALID				BIT(1)
#define ANX9804_SYS_CTRL3_HPD_CTRL				BIT(4)
#define ANX9804_SYS_CTRL3_F_HPD					BIT(5)

#define ANX9804_LINK_BW_SET_REG					0xa0
#define ANX9804_LANE_COUNT_SET_REG				0xa1
#define ANX9804_TRAINING_PTN_SET_REG				0xa2
#define ANX9804_TRAINING_LANE0_SET_REG				0xa3
#define ANX9804_TRAINING_LANE1_SET_REG				0xa4
#define ANX9804_TRAINING_LANE2_SET_REG				0xa5
#define ANX9804_TRAINING_LANE3_SET_REG				0xa6

#define ANX9804_LINK_TRAINING_CTRL_REG				0xa8
#define ANX9804_LINK_TRAINING_CTRL_EN				BIT(0)

#define ANX9804_LINK_DEBUG_REG					0xb8
#define ANX9804_PLL_CTRL_REG					0xc7	
#define ANX9804_ANALOG_POWER_DOWN_REG				0xc8

/* Registers at i2c address 0x39 */

#define ANX9804_DEV_IDH_REG					0x03

#define ANX9804_POWERD_CTRL_REG					0x05
#define ANX9804_POWERD_AUDIO					BIT(4)

#define ANX9804_RST_CTRL_REG					0x06

#define ANX9804_RST_CTRL2_REG					0x07
#define ANX9804_RST_CTRL2_AUX					BIT(2)
#define ANX9804_RST_CTRL2_AC_MODE				BIT(6)

#define ANX9804_VID_CTRL1_REG					0x08
#define ANX9804_VID_CTRL1_VID_EN				BIT(7)
#define ANX9804_VID_CTRL1_EDGE					BIT(0)

#define ANX9804_VID_CTRL2_REG					0x09
#define ANX9804_ANALOG_DEBUG_REG1				0xdc
#define ANX9804_ANALOG_DEBUG_REG3				0xde
#define ANX9804_PLL_FILTER_CTRL1				0xdf
#define ANX9804_PLL_FILTER_CTRL3				0xe1
#define ANX9804_PLL_FILTER_CTRL					0xe2
#define ANX9804_PLL_CTRL3					0xe6

static int anx9804_write(struct udevice *dev, unsigned addr_off,
			 unsigned char reg_addr, unsigned char value)
{
	struct dm_i2c_chip *chip = dev_get_parent_platdata(dev);
	uint8_t buf[2];
	struct i2c_msg msg;
	int ret;

	msg.addr = addr_off;
	msg.flags = 0;
	buf[0] = reg_addr;
	buf[1] = value;
	msg.buf = buf;
	msg.len = 2;
	ret = dm_i2c_xfer(dev, &msg, 1);
	if (ret) {
		debug("%s: write failed, reg=%#x, value=%#x, ret=%d\n",
		      __func__, reg_addr, value, ret);
		return ret;
	}

	return 0;
}

static int anx9804_read(struct udevice *dev, unsigned addr_off,
			unsigned char reg_addr, unsigned char *value)
{
	struct dm_i2c_chip *chip = dev_get_parent_platdata(dev);
	uint8_t addr, val;
	struct i2c_msg msg[2];
	int ret;

	msg[0].addr = addr_off;
	msg[0].flags = 0;
	addr = reg_addr;
	msg[0].buf = &addr;
	msg[0].len = 1;
	msg[1].addr = addr_off;
	msg[1].flags = I2C_M_RD;
	msg[1].buf = &val;
	msg[1].len = 1;
	ret = dm_i2c_xfer(dev, msg, 2);
	if (ret) {
		debug("%s: read failed, reg=%.2x, value=%p, ret=%d\n",
		      __func__, (int)reg_addr, value, ret);
		return ret;
	}
	*value = val;

	return 0;
}

static int anx9804_set_backlight(struct udevice *dev, int percent)
{
	return 0;
}

static int anx9804_attach(struct udevice *dev)
{
	u8 chipid;
	int ret, i;
	u8 c, colordepth;

	u8 lanes = 1;
	u8 data_rate = 0x06; /* 1620M */
	u8 bpp = 18;

	/* Deassert reset and enable power */
	ret = video_bridge_set_active(dev, true);
	if (ret)
		return ret;

	/* Reset */
	anx9804_write(dev, 0x39, ANX9804_RST_CTRL_REG, 1);
	mdelay(100);
	anx9804_write(dev, 0x39, ANX9804_RST_CTRL_REG, 0);

	/* Write 0 to the powerdown reg (powerup everything) */
	anx9804_write(dev, 0x39, ANX9804_POWERD_CTRL_REG, 0);

	ret = anx9804_read(dev, 0x39, ANX9804_DEV_IDH_REG, &c);
	if (ret)
		debug("%s: read id failed: %d\n", __func__, ret);

	switch(c) {
	case 0x98:
		printf("ANX98xx detected.\n");
		break;
	case 0x63:
		printf("ANX63xx detected.\n");
		break;
	default:
		printf("Error anx9804 or anx6345 chipid mismatch: %.2x\n", (int)c);
		return -ENODEV;
	}
	chipid = c;

	for (i = 0; i < 100; i++) {
		anx9804_read(dev, 0x38, ANX9804_SYS_CTRL2_REG, &c);
		anx9804_write(dev, 0x38, ANX9804_SYS_CTRL2_REG, c);
		anx9804_read(dev, 0x38, ANX9804_SYS_CTRL2_REG, &c);
		if ((c & ANX9804_SYS_CTRL2_CHA_STA) == 0)
			break;

		mdelay(5);
	}
	if (i == 100)
		printf("Error anx9804 clock is not stable\n");

	if (bpp == 18)
		colordepth = 0x00; /* 6 bit */
	else
		colordepth = 0x10; /* 8 bit */
	anx9804_write(dev, 0x39, ANX9804_VID_CTRL2_REG, colordepth);
	
	/* Set a bunch of analog related register values */
	if (chipid == 0x98) {
		anx9804_write(dev, 0x38, ANX9804_PLL_CTRL_REG, 0x07);
		anx9804_write(dev, 0x39, ANX9804_PLL_FILTER_CTRL3, 0x19);
		anx9804_write(dev, 0x39, ANX9804_PLL_CTRL3, 0xd9);
		anx9804_write(dev, 0x39, ANX9804_RST_CTRL2_REG, ANX9804_RST_CTRL2_AC_MODE);
		anx9804_write(dev, 0x39, ANX9804_ANALOG_DEBUG_REG1, 0xf0);
		anx9804_write(dev, 0x39, ANX9804_ANALOG_DEBUG_REG3, 0x99);
		anx9804_write(dev, 0x39, ANX9804_PLL_FILTER_CTRL1, 0x7b);
		anx9804_write(dev, 0x39, ANX9804_PLL_FILTER_CTRL, 0x06);
	} else {
		anx9804_write(dev, 0x38, ANX9804_PLL_CTRL_REG, 0x00);
		anx9804_write(dev, 0x39, ANX9804_ANALOG_DEBUG_REG1, 0x70);
	}
	anx9804_write(dev, 0x38, ANX9804_LINK_DEBUG_REG, 0x30);

	/* Force HPD */
	anx9804_write(dev, 0x38, ANX9804_SYS_CTRL3_REG,
		      ANX9804_SYS_CTRL3_F_HPD | ANX9804_SYS_CTRL3_HPD_CTRL);

	/* Power up and configure lanes */
	anx9804_write(dev, 0x38, ANX9804_ANALOG_POWER_DOWN_REG, 0x00);
	anx9804_write(dev, 0x38, ANX9804_TRAINING_LANE0_SET_REG, 0x00);
	anx9804_write(dev, 0x38, ANX9804_TRAINING_LANE1_SET_REG, 0x00);
	anx9804_write(dev, 0x38, ANX9804_TRAINING_LANE2_SET_REG, 0x00);
	anx9804_write(dev, 0x38, ANX9804_TRAINING_LANE3_SET_REG, 0x00);

	/* Reset AUX CH */
	if (chipid == 0x98) {
		anx9804_write(dev, 0x39, ANX9804_RST_CTRL2_REG,
			      ANX9804_RST_CTRL2_AC_MODE |
			      ANX9804_RST_CTRL2_AUX);
		anx9804_write(dev, 0x39, ANX9804_RST_CTRL2_REG,
			      ANX9804_RST_CTRL2_AC_MODE);

	} else {
		anx9804_write(dev, 0x39, ANX9804_RST_CTRL2_REG,
			      ANX9804_RST_CTRL2_AUX);
		anx9804_write(dev, 0x39, ANX9804_RST_CTRL2_REG, 0);
	}

	/* Powerdown audio and some other unused bits */
	anx9804_write(dev, 0x39, ANX9804_POWERD_CTRL_REG, ANX9804_POWERD_AUDIO);
	anx9804_write(dev, 0x38, ANX9804_HDCP_CONTROL_0_REG, 0x00);
	anx9804_write(dev, 0x38, 0xa7, 0x00);

	/* Set data-rate / lanes */
	anx9804_write(dev, 0x38, ANX9804_LINK_BW_SET_REG, data_rate);
	anx9804_write(dev, 0x38, ANX9804_LANE_COUNT_SET_REG, lanes);

	/* Link training */	
	anx9804_write(dev, 0x38, ANX9804_LINK_TRAINING_CTRL_REG,
		      ANX9804_LINK_TRAINING_CTRL_EN);
	mdelay(5);
	for (i = 0; i < 100; i++) {
		anx9804_read(dev, 0x38, ANX9804_LINK_TRAINING_CTRL_REG, &c);
		if (((chipid == 0x98) && (c & 0x01) == 0) ||
		    ((chipid == 0x63) && (c & 0x80) == 0))
			break;

		mdelay(5);
	}
	if(i == 100) {
		printf("Error anx9804 link training timeout\n");
		return -ENODEV;
	}

	/* Enable */
	anx9804_write(dev, 0x39, ANX9804_VID_CTRL1_REG,
		      ANX9804_VID_CTRL1_VID_EN | ANX9804_VID_CTRL1_EDGE);
	/* Force stream valid */
	anx9804_write(dev, 0x38, ANX9804_SYS_CTRL3_REG,
		      ANX9804_SYS_CTRL3_F_HPD | ANX9804_SYS_CTRL3_HPD_CTRL |
		      ANX9804_SYS_CTRL3_F_VALID | ANX9804_SYS_CTRL3_VALID_CTRL);

	printf("%s: Init completed!!!\n", __func__);
	return 0;
}

static int anx9804_probe(struct udevice *dev)
{
	debug("%s\n", __func__);
	if (device_get_uclass_id(dev->parent) != UCLASS_I2C)
		return -EPROTONOSUPPORT;

	return 0;
}

struct video_bridge_ops anx9804_ops = {
	.attach = anx9804_attach,
	.set_backlight = anx9804_set_backlight,
};

static const struct udevice_id anx9804_ids[] = {
	{ .compatible = "analogix,anx9804", },
	{ .compatible = "analogix,anx6345", },
	{ }
};

U_BOOT_DRIVER(parade_anx9804) = {
	.name	= "analogix_anx9804",
	.id	= UCLASS_VIDEO_BRIDGE,
	.of_match = anx9804_ids,
	.probe	= anx9804_probe,
	.ops	= &anx9804_ops,
};
