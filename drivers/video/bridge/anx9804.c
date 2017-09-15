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

/* FIXME! */
#define ANX9804_AUX_CH_STA					0xe0
#define ANX9804_AUX_BUSY					(0x1 << 4)
#define ANX9804_AUX_STATUS_MASK					(0xf << 0)
#define ANX9804_DP_AUX_RX_COMM					0xe3
#define ANX9804_AUX_RX_COMM_I2C_DEFER				(0x2 << 2)
#define ANX9804_AUX_RX_COMM_AUX_DEFER				(0x2 << 0)
#define ANX9804_BUF_DATA_CTL					0xe4
#define ANX9804_BUF_CLR						(0x1 << 7)
#define ANX9804_DP_AUX_CH_CTL_1					0xe5
#define ANX9804_AUX_LENGTH(x)					(((x - 1) & 0xf) << 4)
#define ANX9804_AUX_TX_COMM_MASK				(0xf << 0)
#define ANX9804_AUX_TX_COMM_DP_TRANSACTION			(0x1 << 3)
#define ANX9804_AUX_TX_COMM_I2C_TRANSACTION			(0x0 << 3)
#define ANX9804_AUX_TX_COMM_MOT					(0x1 << 2)
#define ANX9804_AUX_TX_COMM_WRITE				(0x0 << 0)
#define ANX9804_AUX_TX_COMM_READ				(0x1 << 0)

#define ANX9804_DP_AUX_ADDR_7_0					0xe6
#define ANX9804_DP_AUX_ADDR_15_8				0xe7
#define ANX9804_DP_AUX_ADDR_19_16				0xe8

#define ANX9804_DP_AUX_CH_CTL_2					0xe9
#define ANX9804_ADDR_ONLY					(0x1 << 1)
#define ANX9804_AUX_EN						(0x1 << 0)

#define ANX9804_BUF_DATA_0					0xf0

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

// FIXME
#define ANX9804_DP_INT_STA					0xf7
#define ANX9804_RPLY_RECEIV					(0x1 << 1)
#define ANX9804_AUX_ERR						(0x1 << 0)
#define ANX9804_SP_COMMON_INT_MASK1				0xF8
#define ANX9804_SP_COMMON_INT_MASK2				0xF9
#define ANX9804_SP_COMMON_INT_MASK3				0xFA
#define ANX9804_SP_COMMON_INT_MASK4				0xFB
#define ANX9804_SP_INT_MASK					0xFE
#define ANX9804_INT_CTRL_REG					0xFF




static int anx9804_write(struct udevice *dev, unsigned addr_off,
			 unsigned char reg_addr, unsigned char value)
{
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


static int anx6345_start_aux_transaction(struct udevice *dev)
{
	u8 val;
	int retval = 0;
	int timeout_loop = 0;
	int aux_timeout = 0;
	

	anx9804_read(dev, 0x38, ANX9804_DP_AUX_CH_CTL_2, &val);
	val |= ANX9804_AUX_EN;
	anx9804_write(dev, 0x38, ANX9804_DP_AUX_CH_CTL_2, val);

	anx9804_read(dev, 0x38, ANX9804_DP_AUX_CH_CTL_2, &val);
	while (val & ANX9804_AUX_EN) {
		aux_timeout++;
		if ((100 * 10) < aux_timeout) {
			printf("AUX CH enable timeout!\n");
			return -ETIMEDOUT;
		}
		anx9804_read(dev, 0x38, ANX9804_DP_AUX_CH_CTL_2, &val);
		udelay(100);
	}

	/* Is AUX CH command redply received? */
	anx9804_read(dev, 0x39, ANX9804_DP_INT_STA, &val);
	while (!(val & ANX9804_RPLY_RECEIV)) {
		timeout_loop++;
		if (100 < timeout_loop) {
			printf("AUX CH command redply failed!\n");
			return -ETIMEDOUT;
		}
		anx9804_read(dev, 0x39, ANX9804_DP_INT_STA, &val);
		udelay(10);
	}

	/* Clear interrupt source for AUX CH command redply */
	anx9804_write(dev, 0x39, ANX9804_DP_INT_STA, val);

	/* Check AUX CH error access status */
	anx9804_read(dev, 0x38, ANX9804_AUX_CH_STA, &val);
	if ((val & ANX9804_AUX_STATUS_MASK) != 0) {
		printf("AUX CH error happens: %d\n\n",
			val & ANX9804_AUX_STATUS_MASK);
		return -EREMOTEIO;
	}

	return retval;
}

static int anx6345_select_i2c_device(struct udevice *dev,
			      unsigned int device_addr,
			      u8 val_addr)
{
	u8 val;
	int retval;

	/* Set normal AUX CH command */
	anx9804_read(dev, 0x38, ANX9804_DP_AUX_CH_CTL_2, &val);
	val &= ~ANX9804_ADDR_ONLY;
	anx9804_write(dev, 0x38, ANX9804_DP_AUX_CH_CTL_2, val);
	/* Set EDID device address */
	val = device_addr;
	anx9804_write(dev, 0x38, ANX9804_DP_AUX_ADDR_7_0, val);
	val = 0;
	anx9804_write(dev, 0x38, ANX9804_DP_AUX_ADDR_15_8, val);
	anx9804_write(dev, 0x38, ANX9804_DP_AUX_ADDR_19_16, val);

	/* Set offset from base address of EDID device */
	anx9804_write(dev, 0x38, ANX9804_BUF_DATA_0, val_addr);

	/*
	 * Set I2C transaction and write address
	 * If bit 3 is 1, DisplayPort transaction.
	 * If Bit 3 is 0, I2C transaction.
	 */
	val = ANX9804_AUX_TX_COMM_I2C_TRANSACTION | ANX9804_AUX_TX_COMM_MOT |
		ANX9804_AUX_TX_COMM_WRITE;
	anx9804_write(dev, 0x38, ANX9804_DP_AUX_CH_CTL_1, val);

	/* Start AUX transaction */
	retval = anx6345_start_aux_transaction(dev);
	if (retval != 0)
		debug("Aux Transaction fail!\n");

	return retval;
}

int anx6345_edid_read_bytes(struct udevice *dev,
				unsigned int device_addr,
				unsigned int val_addr,
				unsigned char count,
				unsigned char edid[])
{
	u8 val;
	unsigned int i;
	unsigned int start_offset;
	unsigned int cur_data_idx;
	unsigned int cur_data_cnt;
	unsigned int defer = 0;
	int retval = 0;

	for (i = 0; i < count; i += 16) {
		start_offset = i;
		if ((count - start_offset) > 16)
				cur_data_cnt = 16;
			else
				cur_data_cnt = count - start_offset;
		/*
		 * If Rx sends defer, Tx sends only reads
		 * request without sending addres
		 */
		if (!defer)
			retval = anx6345_select_i2c_device(dev,
					device_addr, val_addr + i);
		else
			defer = 0;

		/*
		 * Set I2C transaction and write data
		 * If bit 3 is 1, DisplayPort transaction.
		 * If Bit 3 is 0, I2C transaction.
		 */
		val = ANX9804_AUX_LENGTH(cur_data_cnt) | ANX9804_AUX_TX_COMM_I2C_TRANSACTION |
			ANX9804_AUX_TX_COMM_READ;
		anx9804_write(dev, 0x38, ANX9804_DP_AUX_CH_CTL_1, val);

		/* Start AUX transaction */
		retval = anx6345_start_aux_transaction(dev);
		if (retval < 0)
			debug("Aux Transaction fail!\n");

		/* Check if Rx sends defer */
		anx9804_read(dev, 0x38, ANX9804_DP_AUX_RX_COMM, &val);
		if (val == ANX9804_AUX_RX_COMM_AUX_DEFER ||
			val == ANX9804_AUX_RX_COMM_I2C_DEFER) {
			debug("Defer: %d\n\n", val);
			defer = 1;
		}
		

		for (cur_data_idx = 0; cur_data_idx < cur_data_cnt; cur_data_idx++) {
			anx9804_read(dev, 0x38, ANX9804_BUF_DATA_0 + cur_data_idx, &val);
			edid[i + cur_data_idx] = val;
			debug("0x%02x : 0x%02x\n", i + cur_data_idx, val);
		}
	}

	return retval;
}

static int anx9804_attach(struct udevice *dev)
{
	u8 chipid;
	int ret, i;
	u8 c, colordepth;
	u8 edid[128];

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

	memset(edid, 0xa5, 128);
	anx6345_edid_read_bytes(dev, 0x50, 0x0, 128, edid);
	printf("edid:\n");
	for (i = 0; i < 128; i+= 8) {
		printf("%.2x %.2x  %.2x  %.2x  %.2x  %.2x  %.2x  %.2x\n",
			edid[i + 0],
			edid[i + 1],
			edid[i + 2],
			edid[i + 3],
			edid[i + 4],
			edid[i + 5],
			edid[i + 6],
			edid[i + 7]);
	}

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
