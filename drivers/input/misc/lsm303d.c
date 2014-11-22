/* drivers/input/misc/lsm303d.c
 *
 * Copyright (C) 2012 XiaoMi, Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/device.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#ifdef CONFIG_SENSORS_LSM303D_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif
#include <linux/input/lsm303d.h>

/* lsm303d register list */
#define LSM303D_TEMP_OUT_L				0x05
#define LSM303D_TEMP_OUT_H				0x06
#define LSM303D_STATUS_REG_M				0x07
#define LSM303D_OUT_X_L_M				0x08
#define LSM303D_OUT_X_H_M				0x09
#define LSM303D_OUT_Y_L_M				0x0a
#define LSM303D_OUT_Y_H_M				0x0b
#define LSM303D_OUT_Z_L_M				0x0c
#define LSM303D_OUT_Z_H_M				0x0d
#define LSM303D_WHO_AM_I				0x0f
#define LSM303D_INT_CTRL_REG_M				0x12
#define LSM303D_INT_SRC_REG_M				0x13
#define LSM303D_INT_THS_L_M				0x14
#define LSM303D_INT_THS_H_M				0x15
#define LSM303D_OFFSET_X_L_M				0x16
#define LSM303D_OFFSET_X_H_M				0x17
#define LSM303D_OFFSET_Y_L_M				0x18
#define LSM303D_OFFSET_Y_H_M				0x19
#define LSM303D_OFFSET_Z_L_M				0x1a
#define LSM303D_OFFSET_Z_H_M				0x1b
#define LSM303D_REFERENCE_X				0x1c
#define LSM303D_REFERENCE_Y				0x1d
#define LSM303D_REFERENCE_Z				0x1e
#define LSM303D_CNTRL0					0x1f
#define LSM303D_CNTRL1					0x20
#define LSM303D_CNTRL2					0x21
#define LSM303D_CNTRL3					0x22
#define LSM303D_CNTRL4					0x23
#define LSM303D_CNTRL5					0x24
#define LSM303D_CNTRL6					0x25
#define LSM303D_CNTRL7					0x26
#define LSM303D_STATUS_REG_A				0x27
#define LSM303D_OUT_X_L_A				0x28
#define LSM303D_OUT_X_H_A				0x29
#define LSM303D_OUT_Y_L_A				0x2a
#define LSM303D_OUT_Y_H_A				0x2b
#define LSM303D_OUT_Z_L_A				0x2c
#define LSM303D_OUT_Z_H_A				0x2d
#define LSM303D_FIFO_CNTRL_REG				0x2e
#define LSM303D_FIFO_SRC_REG				0x2f
#define LSM303D_INT_GEN_1_REG				0x30
#define LSM303D_INT_GEN_1_SRC				0x31
#define LSM303D_INT_GEN_1_THS				0x32
#define LSM303D_INT_GEN_1_DURATION			0x33
#define LSM303D_INT_GEN_2_REG				0x34
#define LSM303D_INT_GEN_2_SRC				0x35
#define LSM303D_INT_GEN_2_THS				0x36
#define LSM303D_INT_GEN_2_DURATION			0x37
#define LSM303D_CLICK_CFG				0x38
#define LSM303D_CLICK_SRC				0x39
#define LSM303D_CLICK_THS				0x3a
#define LSM303D_TIME_LIMIT				0x3b
#define LSM303D_TIME_LATENCY				0x3c
#define LSM303D_TIME_WINDOW				0x3d
#define LSM303D_ACT_THS					0x3e
#define LSM303D_ACT_DUR					0x3f

#define LSM303D_REG_SIZE				0x40

/* enable address auto increment(or with address) */
#define LSM303D_AUTO_INC				0x80

/* lsm303d identifier */
#define LSM303D_CHIP_ID					0x49

/* magnetometer status flag */
#define LSM303D_ZYXMOR					0x80
#define LSM303D_ZMOR					0x40
#define LSM303D_YMOR					0x20
#define LSM303D_XMOR					0x10
#define LSM303D_ZYXMDA					0x08
#define LSM303D_ZMDA					0x04
#define LSM303D_YMDA					0x02
#define LSM303D_XMDA					0x01

/* accelerometer output data rate(block update and 3 axis) */
#define LSM303D_AODR_3_125HZ				0x1f
#define LSM303D_AODR_6_25HZ				0x2f
#define LSM303D_AODR_12_5HZ				0x3f
#define LSM303D_AODR_25HZ				0x4f
#define LSM303D_AODR_50HZ				0x5f
#define LSM303D_AODR_100HZ				0x6f
#define LSM303D_AODR_200HZ				0x7f
#define LSM303D_AODR_400HZ				0x8f
#define LSM303D_AODR_800HZ				0x9f
#define LSM303D_AODR_1600HZ				0xaf

#define LSM303D_APOWER_OFF				0x07

/* accelerometer full scale selection */
#define LSM303D_AFS_2G					0x00
#define LSM303D_AFS_4G					0x08
#define LSM303D_AFS_8G					0x10
#define LSM303D_AFS_16G					0x18

#define LSM303D_AST					0x02

/* interrupt enable bit for int1 */
#define LSM303D_P1_BOOT					0x80
#define LSM303D_P1_TAP					0x40
#define LSM303D_P1_INT1					0x20
#define LSM303D_P1_INT2					0x10
#define LSM303D_P1_INTM					0x08
#define LSM303D_P1_DRDYA				0x04
#define LSM303D_P1_DRDYM				0x02
#define LSM303D_P1_EMPTY				0x01

/* interrupt enable bit for int2 */
#define LSM303D_P2_TAP					0x80
#define LSM303D_P2_INT1					0x40
#define LSM303D_P2_INT2					0x20
#define LSM303D_P2_INTM					0x10
#define LSM303D_P2_DRDYA				0x08
#define LSM303D_P2_DRDYM				0x04
#define LSM303D_P2_OVERRUN				0x02
#define LSM303D_P2_WTM					0x01

/* magnetometer output data rate */
#define LSM303D_MODR_3_125HZ				0x00
#define LSM303D_MODR_6_25HZ				0x04
#define LSM303D_MODR_12_5HZ				0x08
#define LSM303D_MODR_25HZ				0x0c
#define LSM303D_MODR_50HZ				0x10
#define LSM303D_MODR_100HZ				0x14
#define LSM303D_MODR_200HZ				0x18

/* magnetometer full scale selection */
#define LSM303D_MFS_2GA					0x00
#define LSM303D_MFS_4GA					0x20
#define LSM303D_MFS_8GA					0x40
#define LSM303D_MFS_12GA				0x60

/* magnetometer mode selection */
#define LSM303D_MD_CONTINUOUS				0x00
#define LSM303D_MD_SINGLE				0x01
#define LSM303D_MD_POWERDOWN				0x03

/* accelerometer status flag */
#define LSM303D_ZYXAOR					0x80
#define LSM303D_ZAOR					0x40
#define LSM303D_YAOR					0x20
#define LSM303D_XAOR					0x10
#define LSM303D_ZYXADA					0x08
#define LSM303D_ZADA					0x04
#define LSM303D_YADA					0x02
#define LSM303D_XADA					0x01

/* to avoid the unstable result
   skip process output several time when scale change */
#define LSM303D_SKIPCOUNT				2

/* output overflow flag */
#define LSM303D_OVERFLOW				32767

/* functionality need to support by i2c adapter */
#define LSM303D_FUNC	\
	(I2C_FUNC_SMBUS_BYTE_DATA | I2C_FUNC_SMBUS_READ_I2C_BLOCK)

/* magnetometer supported range and resolution(unit is mga) */
#define LSM303D_MMIN					-12000
#define LSM303D_MMAX					12000
#define LSM303D_MRES					1

/* accelerometer supported range and resolution(unit is mg) */
#define LSM303D_AMIN					-16000
#define LSM303D_AMAX					16000
#define LSM303D_ARES					1

/* macro for define sys attribute */
#define LSM303D_ATTR(_prefix, _name, _mode, _show, _store) \
		static struct device_attribute \
		lsm303d_##_prefix##_##_name##_attr = \
		__ATTR(_name, _mode, _show, _store)

/* lsm303d device context
   if auto_scale != 0 then
      if max_output <= low_thres, then move to next small scale
      if max_output >= high_thres, then move to next big scale
   else always use the specified scale value */
struct lsm303d_data {
	/* shared state for magnetometer and accelerometer */
	struct mutex mutex;
	struct i2c_client *client;
	bool suspended;
#ifdef CONFIG_SENSORS_LSM303D_EARLYSUSPEND
	struct early_suspend early_suspend;
#endif
	/* state for magnetometer */
	struct input_dev *mag_input;
	bool mag_opened;
	bool mag_enabled;
	unsigned long mag_skipcount;
	unsigned long mag_dbgdump;
	unsigned long mag_period;	/* unit: ns */
	unsigned long mag_scale;	/* unit: mga */
	bool mag_autoscale;
	/* state for accelerometer */
	struct input_dev *acc_input;
	bool acc_opened;
	bool acc_enabled;
	unsigned long acc_skipcount;
	unsigned long acc_dbgdump;
	unsigned long acc_period;	/* unit: ns */
	unsigned long acc_scale;	/* unit: mg */
	bool acc_autoscale;
	bool acc_testmode;
};

/* small utility function */
static s32
i2c_smbus_write_bits_data(struct i2c_client *client,
			  u8 command, u8 value, u8 mask)
{
	s32 result;

	result = i2c_smbus_read_byte_data(client, command);
	if (result >= 0) {
		result &= ~mask, result |= value;
		result = i2c_smbus_write_byte_data(client, command, result);
	}

	return result;
}

static int
dump_hex_buffer(const char *prefix_str, int prefix_type, int rowsize,
		int groupsize, const void *buf, size_t len, bool ascii,
		char *output, size_t output_len)
{
	const u8 *ptr = buf;
	int i, linelen, count = 0, remaining = len;
	unsigned char linebuf[32 * 3 + 2 + 32 + 1];

	if (rowsize != 16 && rowsize != 32)
		rowsize = 16;

	for (i = 0; i < len; i += rowsize) {
		linelen = min(remaining, rowsize);
		remaining -= rowsize;

		hex_dump_to_buffer(ptr + i, linelen, rowsize, groupsize,
				   linebuf, sizeof(linebuf), ascii);

		switch (prefix_type) {
		case DUMP_PREFIX_ADDRESS:
			count += snprintf(output + count, output_len - count,
					  "%s%p: %s\n", prefix_str, ptr + i,
					  linebuf);
			break;
		case DUMP_PREFIX_OFFSET:
			count += snprintf(output + count, output_len - count,
					  "%s%.8x: %s\n", prefix_str, i,
					  linebuf);
			break;
		default:
			count += snprintf(output + count, output_len - count,
					  "%s%s\n", prefix_str, linebuf);
			break;
		}
	}

	return count;
}

/* magnetometer specified function */
static unsigned long lsm303d_mag_adjust_period(unsigned long period)
{
	period = max(period, 10000000UL);

	if (period >= 320000000)	/* 320ms(3.125Hz) */
		return 320000000;
	else if (period >= 160000000)	/* 160ms(6.25Hz) */
		return 160000000;
	else if (period >= 80000000)	/* 80ms(12.5Hz) */
		return 80000000;
	else if (period >= 40000000)	/* 40ms(25Hz) */
		return 40000000;
	else if (period >= 20000000)	/* 20ms(50Hz) */
		return 20000000;
	else if (period >= 10000000)	/* 10ms(100Hz) */
		return 10000000;
	else if (period >= 5000000)	/* 5ms(200Hz) */
		return 5000000;

	return 5000000;		/* out of range return as fast as we can */
}

static unsigned long lsm303d_mag_adjust_scale(unsigned long scale)
{
	if (scale == 0)		/* auto scale */
		return 4000;
	else if (scale <= 2000)	/* 2Ga */
		return 2000;
	else if (scale <= 4000)	/* 4Ga */
		return 4000;
	else if (scale <= 8000)	/* 8Ga */
		return 8000;
	else if (scale <= 12000)	/* 12Ga */
		return 12000;

	return 12000;		/* out of range, return as large as we can */
}

static unsigned long lsm303d_mag_get_next_small_scale(unsigned long scale)
{
	switch (scale) {
	case 2000:
		return 2000;
	case 4000:
		return 2000;
	case 8000:
		return 4000;
	case 12000:
		return 8000;
	default:
		BUG();
	}
}

static unsigned long lsm303d_mag_get_next_big_scale(unsigned long scale)
{
	switch (scale) {
	case 2000:
		return 4000;
	case 4000:
		return 8000;
	case 8000:
		return 12000;
	case 12000:
		return 12000;
	default:
		BUG();
	}
}

static unsigned long lsm303d_mag_get_sensitivity(unsigned long scale)
{
	/* multiply by 1000 to avoid float point */
	switch (scale) {
	case 2000:
		return 80;
	case 4000:
		return 160;
	case 8000:
		return 320;
	case 12000:
		return 480;
	default:
		BUG();
	}
}

static bool lsm303d_mag_is_overflow(s16 x, s16 y, s16 z)
{
	return abs(x) >= LSM303D_OVERFLOW ||
	    abs(y) >= LSM303D_OVERFLOW || abs(z) >= LSM303D_OVERFLOW;
}

static int lsm303d_mag_scale_output(s16 output, unsigned long scale)
{
	return (int)lsm303d_mag_get_sensitivity(scale) * output / 1000;
}

static u8 lsm303d_mag_get_ctrl5(unsigned long period)
{
	switch (period) {
	case 320000000:
		return LSM303D_MODR_3_125HZ;
	case 160000000:
		return LSM303D_MODR_6_25HZ;
	case 80000000:
		return LSM303D_MODR_12_5HZ;
	case 40000000:
		return LSM303D_MODR_25HZ;
	case 20000000:
		return LSM303D_MODR_50HZ;
	case 10000000:
		return LSM303D_MODR_100HZ;
	case 5000000:
		return LSM303D_MODR_200HZ;
	default:
		BUG();
	}
}

static u8 lsm303d_mag_get_ctrl6(unsigned long scale)
{
	switch (scale) {
	case 2000:
		return LSM303D_MFS_2GA;
	case 4000:
		return LSM303D_MFS_4GA;
	case 8000:
		return LSM303D_MFS_8GA;
	case 12000:
		return LSM303D_MFS_12GA;
	default:
		BUG();
	}
}

static bool lsm303d_mag_is_active(struct lsm303d_data *data)
{
	return data->mag_opened && data->mag_enabled && !data->suspended;
}

static int
lsm303d_mag_read_output(struct lsm303d_data *data, s16 *x, s16 *y, s16 *z)
{
	u8 buf[6];
	s32 status;
	int error = 0;
	struct i2c_client *client = data->client;

	status = i2c_smbus_read_byte_data(client, LSM303D_STATUS_REG_M);
	if (status < 0) {
		error = status;
		dev_err(&client->dev, "fail to read mag status register.");
		goto exit;
	}

	if ((status & LSM303D_ZYXMDA) == 0) {
		error = -EAGAIN;	/* output isn't ready, try later again. */
		goto exit;
	}

	error = i2c_smbus_read_i2c_block_data(client,
					      LSM303D_AUTO_INC |
					      LSM303D_OUT_X_L_M, sizeof(buf),
					      buf);
	if (error < 0) {
		dev_err(&client->dev, "fail to read mag output register.");
		goto exit;
	}

	*x = (buf[1] << 8) | buf[0];
	*y = (buf[3] << 8) | buf[2];
	*z = (buf[5] << 8) | buf[4];

exit:
	return error;
}

static int lsm303d_mag_update_device(struct lsm303d_data *data)
{
	int error = 0;
	struct i2c_client *client = data->client;

	/* disable sampling before update setting */
	error = i2c_smbus_write_byte_data(client,
					  LSM303D_CNTRL7, LSM303D_MD_POWERDOWN);
	if (error < 0) {
		dev_err(&client->dev, "fail to disable mag sampling.");
		goto exit;
	}

	/* update period register */
	error = i2c_smbus_write_byte_data(client, LSM303D_CNTRL5,
					  lsm303d_mag_get_ctrl5(data->
								mag_period));
	if (error < 0) {
		dev_err(&client->dev, "fail to set mag period register.");
		goto exit;
	}

	/* update scale register */
	error = i2c_smbus_write_byte_data(client, LSM303D_CNTRL6,
					  lsm303d_mag_get_ctrl6(data->
								mag_scale));
	if (error < 0) {
		dev_err(&client->dev, "fail to set mag scale register.");
		goto exit;
	}

	/* update mode register */
	if (lsm303d_mag_is_active(data)) {
		error = i2c_smbus_write_byte_data(client,
						  LSM303D_CNTRL7,
						  LSM303D_MD_CONTINUOUS);
		if (error < 0) {
			dev_err(&client->dev, "fail to set mag mode register.");
			goto exit;
		}
	}

exit:
	return error;
}

static void
lsm303d_mag_update_scale(struct lsm303d_data *data, s16 x, s16 y, s16 z)
{
	unsigned long next_scale = data->mag_scale;
	unsigned long max_output = max(max(abs(x), abs(y)), abs(z));
	struct lsm303d_platform_data *pdata = data->client->dev.platform_data;

	if (max_output >= pdata->mag_highthres) {
		next_scale = lsm303d_mag_get_next_big_scale(data->mag_scale);
		if (next_scale == data->mag_scale && data->mag_dbgdump)
			dev_info(&data->client->dev, "out of mag range.");
	} else if (max_output <= pdata->mag_lowthres) {
		next_scale = lsm303d_mag_get_next_small_scale(data->mag_scale);
	}

	if (next_scale != data->mag_scale) {
		if (data->mag_autoscale) {
			if (data->mag_dbgdump) {
				dev_info(&data->client->dev,
					 "update mag scale(%lu).", next_scale);
			}
			data->mag_skipcount = LSM303D_SKIPCOUNT;
			data->mag_scale = next_scale;
			lsm303d_mag_update_device(data);
		} else if (data->mag_dbgdump) {
			dev_info(&data->client->dev,
				 "out of mag range(auto scale disable).");
		}
	}
}

static ssize_t
lsm303d_mag_show_enable(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	unsigned long enabled;
	struct lsm303d_data *data;

	data = dev_get_drvdata(dev);

	mutex_lock(&data->mutex);
	enabled = data->mag_enabled;
	mutex_unlock(&data->mutex);

	return scnprintf(buf, PAGE_SIZE, "%lu\n", enabled);
}

static ssize_t
lsm303d_mag_store_enable(struct device *dev,
			 struct device_attribute *attr, const char *buf,
			 size_t cnt)
{
	int error;
	unsigned long enabled;
	struct lsm303d_data *data;

	data = dev_get_drvdata(dev);

	error = strict_strtoul(buf, 0, &enabled);
	if (error >= 0) {
		mutex_lock(&data->mutex);
		data->mag_enabled = enabled;
		error = lsm303d_mag_update_device(data);
		mutex_unlock(&data->mutex);
	}

	return error < 0 ? error : cnt;
}

static ssize_t
lsm303d_mag_show_dbgdump(struct device *dev,
			 struct device_attribute *attr, char *buf)
{
	int addr, error;
	u8 value[LSM303D_REG_SIZE];
	struct i2c_client *client;
	struct lsm303d_data *data;

	data = dev_get_drvdata(dev);
	client = data->client;

	for (addr = 0; addr < LSM303D_REG_SIZE; addr++) {
		error = i2c_smbus_read_byte_data(client, addr);
		if (error < 0)
			break;
		value[addr] = error;
	}

	if (error >= 0) {
		error = dump_hex_buffer("", DUMP_PREFIX_OFFSET, 16, 1,
					value, sizeof(value), false, buf,
					PAGE_SIZE);
	}

	return error;
}

static ssize_t
lsm303d_mag_store_dbgdump(struct device *dev,
			  struct device_attribute *attr, const char *buf,
			  size_t cnt)
{
	u8 addr;
	u8 value;
	int error;
	unsigned long dbgdump;
	struct i2c_client *client;
	struct lsm303d_data *data;

	data = dev_get_drvdata(dev);
	client = data->client;

	error = strict_strtoul(buf, 0, &dbgdump);
	if (error >= 0) {
		mutex_lock(&data->mutex);
		data->mag_dbgdump = dbgdump;
		mutex_unlock(&data->mutex);
	} else if (sscanf(buf, "%hhx=%hhx", &addr, &value) == 2) {
		error = i2c_smbus_write_byte_data(client, addr, value);
	} else {
		error = -EINVAL;
	}

	return error < 0 ? error : cnt;
}

static ssize_t
lsm303d_mag_show_period(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	unsigned long period;
	struct lsm303d_data *data;

	data = dev_get_drvdata(dev);

	mutex_lock(&data->mutex);
	period = data->mag_period;
	mutex_unlock(&data->mutex);

	return scnprintf(buf, PAGE_SIZE, "%lu\n", period);
}

static ssize_t
lsm303d_mag_store_period(struct device *dev,
			 struct device_attribute *attr, const char *buf,
			 size_t cnt)
{
	int error;
	unsigned long period;
	struct lsm303d_data *data;

	data = dev_get_drvdata(dev);

	error = strict_strtoul(buf, 0, &period);
	if (error >= 0) {
		mutex_lock(&data->mutex);
		data->mag_period = lsm303d_mag_adjust_period(period);
		error = lsm303d_mag_update_device(data);
		mutex_unlock(&data->mutex);
	}

	return error < 0 ? error : cnt;
}

static ssize_t
lsm303d_mag_show_scale(struct device *dev,
		       struct device_attribute *attr, char *buf)
{
	unsigned long scale;
	struct lsm303d_data *data;

	data = dev_get_drvdata(dev);

	mutex_lock(&data->mutex);
	scale = data->mag_scale;
	mutex_unlock(&data->mutex);

	return scnprintf(buf, PAGE_SIZE, "%lu\n", scale);
}

static ssize_t
lsm303d_mag_store_scale(struct device *dev,
			struct device_attribute *attr, const char *buf,
			size_t cnt)
{
	int error;
	unsigned long scale;
	struct lsm303d_data *data;

	data = dev_get_drvdata(dev);

	error = strict_strtoul(buf, 0, &scale);
	if (error >= 0) {
		mutex_lock(&data->mutex);
		data->mag_skipcount = LSM303D_SKIPCOUNT;
		data->mag_scale = lsm303d_mag_adjust_scale(scale);
		data->mag_autoscale = (scale == 0);
		error = lsm303d_mag_update_device(data);
		mutex_unlock(&data->mutex);
	}

	return error < 0 ? error : cnt;
}

LSM303D_ATTR(mag, enable, S_IWUSR | S_IRUGO,
	     lsm303d_mag_show_enable, lsm303d_mag_store_enable);
LSM303D_ATTR(mag, dbgdump, S_IWUSR | S_IRUGO,
	     lsm303d_mag_show_dbgdump, lsm303d_mag_store_dbgdump);
LSM303D_ATTR(mag, poll_delay, S_IWUSR | S_IRUGO,
	     lsm303d_mag_show_period, lsm303d_mag_store_period);
LSM303D_ATTR(mag, scale, S_IWUSR | S_IRUGO,
	     lsm303d_mag_show_scale, lsm303d_mag_store_scale);

static struct attribute *lsm303d_mag_attrs[] = {
	&lsm303d_mag_enable_attr.attr,
	&lsm303d_mag_dbgdump_attr.attr,
	&lsm303d_mag_poll_delay_attr.attr,
	&lsm303d_mag_scale_attr.attr,
	NULL
};

static struct attribute_group lsm303d_mag_attr_grp = {
	.attrs = lsm303d_mag_attrs,
};

static const struct attribute_group *lsm303d_mag_attr_grps[] = {
	&lsm303d_mag_attr_grp,
	NULL
};

static int lsm303d_mag_open(struct input_dev *dev)
{
	int error = 0;
	struct lsm303d_data *data = input_get_drvdata(dev);

	mutex_lock(&data->mutex);
	data->mag_opened = true;
	error = lsm303d_mag_update_device(data);
	mutex_unlock(&data->mutex);

	return error;
}

static void lsm303d_mag_close(struct input_dev *dev)
{
	struct lsm303d_data *data = input_get_drvdata(dev);

	mutex_lock(&data->mutex);
	data->mag_opened = false;
	lsm303d_mag_update_device(data);
	mutex_unlock(&data->mutex);
}

static int lsm303d_mag_interrupt(struct lsm303d_data *data)
{
	s16 x, y, z;
	int xmga, ymga, zmga, count = 0;
	struct lsm303d_platform_data *pdata;

	pdata = data->client->dev.platform_data;
	mutex_lock(&data->mutex);

	if (data->suspended) {
		mutex_unlock(&data->mutex);
		return count;
	}
	while (lsm303d_mag_read_output(data, &x, &y, &z) >= 0) {
		if (data->mag_skipcount == 0) {
			xmga = lsm303d_mag_scale_output(x, data->mag_scale);
			ymga = lsm303d_mag_scale_output(y, data->mag_scale);
			zmga = lsm303d_mag_scale_output(z, data->mag_scale);

			if (!lsm303d_mag_is_overflow(x, y, z)) {
				input_report_abs(data->mag_input,
						 pdata->mag_xcode,
						 pdata->mag_xdir * xmga);
				input_report_abs(data->mag_input,
						 pdata->mag_ycode,
						 pdata->mag_ydir * ymga);
				input_report_abs(data->mag_input,
						 pdata->mag_zcode,
						 pdata->mag_zdir * zmga);
				input_always_sync(data->mag_input);
			}

			lsm303d_mag_update_scale(data, x, y, z);

			if (data->mag_dbgdump) {
				dev_info(&data->client->dev, "x=%06hd(%06dmga),"
					 " y=%06hd(%06dmga), z=%06hd(%06dmga).",
					 x, xmga, y, ymga, z, zmga);
			}
		} else {
			data->mag_skipcount--;
			if (data->mag_dbgdump) {
				dev_info(&data->client->dev,
					 "skip process mag output.");
			}
		}
		count++;	/* how many sample we have read */
	}

	mutex_unlock(&data->mutex);
	return count;
}

static int lsm303d_mag_setup(struct lsm303d_data *data)
{
	int error;
	struct i2c_client *client;
	struct lsm303d_platform_data *pdata;

	client = data->client;
	pdata = client->dev.platform_data;

	data->mag_period = lsm303d_mag_adjust_period(pdata->mag_period);
	data->mag_scale = lsm303d_mag_adjust_scale(pdata->mag_scale);
	data->mag_autoscale = (pdata->mag_scale == 0);

	/* enable data ready interrupt */
	error = i2c_smbus_write_bits_data(data->client,
					  LSM303D_CNTRL3, LSM303D_P1_DRDYM,
					  LSM303D_P1_DRDYM);
	if (error < 0) {
		dev_err(&client->dev,
			"fail to enable mag data ready interrupt");
		goto exit;
	}

	/* register to input system */
	data->mag_input = input_allocate_device();
	if (data->mag_input == NULL) {
		dev_err(&client->dev, "fail to allocate mag input device.");
		error = -ENOMEM;
		goto exit;
	}
	input_set_drvdata(data->mag_input, data);

	data->mag_input->name = "compass";
	data->mag_input->open = lsm303d_mag_open;
	data->mag_input->close = lsm303d_mag_close;
	data->mag_input->dev.groups = lsm303d_mag_attr_grps;

	input_set_capability(data->mag_input, EV_ABS, pdata->mag_xcode);
	input_set_abs_params(data->mag_input, pdata->mag_xcode,
			     LSM303D_MMIN, LSM303D_MMAX, LSM303D_MRES, 0);

	input_set_capability(data->mag_input, EV_ABS, pdata->mag_ycode);
	input_set_abs_params(data->mag_input, pdata->mag_ycode,
			     LSM303D_MMIN, LSM303D_MMAX, LSM303D_MRES, 0);

	input_set_capability(data->mag_input, EV_ABS, pdata->mag_zcode);
	input_set_abs_params(data->mag_input, pdata->mag_zcode,
			     LSM303D_MMIN, LSM303D_MMAX, LSM303D_MRES, 0);

	error = input_register_device(data->mag_input);
	if (error < 0) {
		dev_err(&client->dev, "fail to register mag input device.");
		goto exit_free_device;
	}

	data->mag_input->phys =
	    kobject_get_path(&data->mag_input->dev.kobj, GFP_KERNEL);
	if (data->mag_input->phys == NULL) {
		dev_err(&client->dev, "fail to get mag input sysfs path.");
		error = -ENOMEM;
		goto exit_unregister_device;
	}

	goto exit;		/* all is fine */

exit_unregister_device:
	input_unregister_device(data->mag_input);
	goto exit;
exit_free_device:
	input_free_device(data->mag_input);
exit:
	return error;
}

static void lsm303d_mag_teardown(struct lsm303d_data *data)
{
	kfree(data->mag_input->phys);
	input_unregister_device(data->mag_input);
}

/* accelerometer specified function */
static unsigned long lsm303d_acc_adjust_period(unsigned long period)
{
	period = max(period, 10000000UL);

	if (period >= 320000000)	/* 320ms(3.125Hz) */
		return 320000000;
	else if (period >= 160000000)	/* 160ms(6.25Hz) */
		return 160000000;
	else if (period >= 80000000)	/* 80ms(12.5Hz) */
		return 80000000;
	else if (period >= 40000000)	/* 40ms(25Hz) */
		return 40000000;
	else if (period >= 20000000)	/* 20ms(50Hz) */
		return 20000000;
	else if (period >= 10000000)	/* 10ms(100Hz) */
		return 10000000;
	else if (period >= 5000000)	/* 5ms(200Hz) */
		return 5000000;
	else if (period >= 2500000)	/* 2.5ms(400Hz) */
		return 2500000;
	else if (period >= 1250000)	/* 1.25ms(800Hz) */
		return 1250000;
	else if (period >= 625000)	/* 0.625ms(1600Hz) */
		return 625000;

	return 625000;		/* out of range return as fast as we can */
}

static unsigned long lsm303d_acc_adjust_scale(unsigned long scale)
{
	if (scale == 0)		/* auto scale */
		return 2000;
	else if (scale <= 2000)	/* 2G */
		return 2000;
	else if (scale <= 4000)	/* 4G */
		return 4000;
	else if (scale <= 8000)	/* 8G */
		return 8000;
	else if (scale <= 16000)	/* 16G */
		return 16000;

	return 16000;		/* out of range, return as large as we can */
}

static unsigned long lsm303d_acc_get_next_small_scale(unsigned long scale)
{
	switch (scale) {
	case 2000:
		return 2000;
	case 4000:
		return 2000;
	case 8000:
		return 4000;
	case 16000:
		return 8000;
	default:
		BUG();
	}
}

static unsigned long lsm303d_acc_get_next_big_scale(unsigned long scale)
{
	switch (scale) {
	case 2000:
		return 4000;
	case 4000:
		return 8000;
	case 8000:
		return 16000;
	case 16000:
		return 16000;
	default:
		BUG();
	}
}

static unsigned long lsm303d_acc_get_sensitivity(unsigned long scale)
{
	/* multiply by 1000 to avoid float point */
	switch (scale) {
	case 2000:
		return 1;
	case 4000:
		return 2;
	case 8000:
		return 4;
	case 16000:
		return 12;
	default:
		BUG();
	}
}

static bool lsm303d_acc_is_overflow(s16 x, s16 y, s16 z)
{
	return abs(x) >= LSM303D_OVERFLOW ||
	    abs(y) >= LSM303D_OVERFLOW || abs(z) >= LSM303D_OVERFLOW;
}

static int lsm303d_acc_scale_output(s16 output, unsigned long scale)
{
	return (int)lsm303d_acc_get_sensitivity(scale) * (output >> 4);
}

static u8 lsm303d_acc_get_ctrl1(unsigned long period)
{
	switch (period) {
	case 320000000:
		return LSM303D_AODR_3_125HZ;
	case 160000000:
		return LSM303D_AODR_6_25HZ;
	case 80000000:
		return LSM303D_AODR_12_5HZ;
	case 40000000:
		return LSM303D_AODR_25HZ;
	case 20000000:
		return LSM303D_AODR_50HZ;
	case 10000000:
		return LSM303D_AODR_100HZ;
	case 5000000:
		return LSM303D_AODR_200HZ;
	case 2500000:
		return LSM303D_AODR_400HZ;
	case 1250000:
		return LSM303D_AODR_800HZ;
	case 625000:
		return LSM303D_AODR_1600HZ;
	default:
		BUG();
	}
}

static u8 lsm303d_acc_get_ctrl2(unsigned long scale, bool testmode)
{
	if (testmode) {
		return LSM303D_AST;
	} else {
		switch (scale) {
		case 2000:
			return LSM303D_AFS_2G;
		case 4000:
			return LSM303D_AFS_4G;
		case 8000:
			return LSM303D_AFS_8G;
		case 16000:
			return LSM303D_AFS_16G;
		default:
			BUG();
		}
	}
}

static int
lsm303d_acc_read_output(struct lsm303d_data *data, s16 *x, s16 *y, s16 *z)
{
	u8 buf[6];
	s32 status;
	int error = 0;
	struct i2c_client *client = data->client;

	status = i2c_smbus_read_byte_data(client, LSM303D_STATUS_REG_A);
	if (status < 0) {
		error = status;
		dev_err(&client->dev, "fail to read acc status register.");
		goto exit;
	}

	if ((status & LSM303D_ZYXADA) == 0) {
		error = -EAGAIN;	/* output isn't ready, try later again. */
		goto exit;
	}

	error = i2c_smbus_read_i2c_block_data(client,
					      LSM303D_AUTO_INC |
					      LSM303D_OUT_X_L_A, sizeof(buf),
					      buf);
	if (error < 0) {
		dev_err(&client->dev, "fail to read acc output register.");
		goto exit;
	}

	*x = (buf[1] << 8) | buf[0];
	*y = (buf[3] << 8) | buf[2];
	*z = (buf[5] << 8) | buf[4];

exit:
	return error;
}

static bool lsm303d_acc_is_active(struct lsm303d_data *data)
{
	return data->acc_opened && data->acc_enabled && !data->suspended;
}

static int lsm303d_acc_update_device(struct lsm303d_data *data)
{
	int error = 0;
	struct i2c_client *client = data->client;

	/* disable sampling before update setting */
	error = i2c_smbus_write_byte_data(client,
					  LSM303D_CNTRL1, LSM303D_APOWER_OFF);
	if (error < 0) {
		dev_err(&client->dev, "fail to disable acc sampling.");
		goto exit;
	}

	/* update scale register */
	error = i2c_smbus_write_byte_data(client, LSM303D_CNTRL2,
					  lsm303d_acc_get_ctrl2(data->acc_scale,
								data->
								acc_testmode));
	if (error < 0) {
		dev_err(&client->dev, "fail to set acc scale register.");
		goto exit;
	}

	/* update period register */
	if (lsm303d_acc_is_active(data)) {
		error = i2c_smbus_write_byte_data(client, LSM303D_CNTRL1,
						  lsm303d_acc_get_ctrl1(data->
									acc_period));
		if (error < 0) {
			dev_err(&client->dev,
				"fail to set acc period register.");
			goto exit;
		}
	}

exit:
	return error;
}

static void
lsm303d_acc_update_scale(struct lsm303d_data *data, s16 x, s16 y, s16 z)
{
	unsigned long next_scale = data->acc_scale;
	unsigned long max_output = max(max(abs(x), abs(y)), abs(z));
	struct lsm303d_platform_data *pdata = data->client->dev.platform_data;

	if (max_output >= pdata->acc_highthres) {
		next_scale = lsm303d_acc_get_next_big_scale(data->acc_scale);
		if (next_scale == data->acc_scale && data->acc_dbgdump)
			dev_info(&data->client->dev, "out of acc range.");
	} else if (max_output <= pdata->acc_lowthres) {
		next_scale = lsm303d_acc_get_next_small_scale(data->acc_scale);
	}

	if (next_scale != data->acc_scale) {
		if (data->acc_autoscale) {
			if (data->acc_dbgdump) {
				dev_info(&data->client->dev,
					 "update acc scale(%lu).", next_scale);
			}
			data->acc_skipcount = LSM303D_SKIPCOUNT;
			data->acc_scale = next_scale;
			lsm303d_acc_update_device(data);
		} else if (data->acc_dbgdump) {
			dev_info(&data->client->dev,
				 "out of acc range(auto scale disable).");
		}
	}
}

static ssize_t
lsm303d_acc_show_enable(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	unsigned long enabled;
	struct lsm303d_data *data;

	data = dev_get_drvdata(dev);

	mutex_lock(&data->mutex);
	enabled = data->acc_enabled;
	mutex_unlock(&data->mutex);

	return scnprintf(buf, PAGE_SIZE, "%lu\n", enabled);
}

static ssize_t
lsm303d_acc_store_enable(struct device *dev,
			 struct device_attribute *attr, const char *buf,
			 size_t cnt)
{
	int error;
	unsigned long enabled;
	struct lsm303d_data *data;

	data = dev_get_drvdata(dev);

	error = strict_strtoul(buf, 0, &enabled);
	if (error >= 0) {
		mutex_lock(&data->mutex);
		data->acc_enabled = enabled;
		error = lsm303d_acc_update_device(data);
		mutex_unlock(&data->mutex);
	}

	return error < 0 ? error : cnt;
}

static ssize_t
lsm303d_acc_show_dbgdump(struct device *dev,
			 struct device_attribute *attr, char *buf)
{
	int addr, error;
	u8 value[LSM303D_REG_SIZE];
	struct i2c_client *client;
	struct lsm303d_data *data;

	data = dev_get_drvdata(dev);
	client = data->client;

	for (addr = 0; addr < LSM303D_REG_SIZE; addr++) {
		error = i2c_smbus_read_byte_data(client, addr);
		if (error < 0)
			break;
		value[addr] = error;
	}

	if (error >= 0) {
		error = dump_hex_buffer("", DUMP_PREFIX_OFFSET, 16, 1,
					value, sizeof(value), false, buf,
					PAGE_SIZE);
	}

	return error;
}

static ssize_t
lsm303d_acc_store_dbgdump(struct device *dev,
			  struct device_attribute *attr, const char *buf,
			  size_t cnt)
{
	u8 addr;
	u8 value;
	int error;
	unsigned long dbgdump;
	struct i2c_client *client;
	struct lsm303d_data *data;

	data = dev_get_drvdata(dev);
	client = data->client;

	error = strict_strtoul(buf, 0, &dbgdump);
	if (error >= 0) {
		mutex_lock(&data->mutex);
		data->acc_dbgdump = dbgdump;
		mutex_unlock(&data->mutex);
	} else if (sscanf(buf, "%hhx=%hhx", &addr, &value) == 2) {
		error = i2c_smbus_write_byte_data(client, addr, value);
	} else {
		error = -EINVAL;
	}

	return error < 0 ? error : cnt;
}

static ssize_t
lsm303d_acc_show_period(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	unsigned long period;
	struct lsm303d_data *data;

	data = dev_get_drvdata(dev);

	mutex_lock(&data->mutex);
	period = data->acc_period;
	mutex_unlock(&data->mutex);

	return scnprintf(buf, PAGE_SIZE, "%lu\n", period);
}

static ssize_t
lsm303d_acc_store_period(struct device *dev,
			 struct device_attribute *attr, const char *buf,
			 size_t cnt)
{
	int error;
	unsigned long period;
	struct lsm303d_data *data;

	data = dev_get_drvdata(dev);

	error = strict_strtoul(buf, 0, &period);
	if (error >= 0) {
		mutex_lock(&data->mutex);
		data->acc_period = lsm303d_acc_adjust_period(period);
		error = lsm303d_acc_update_device(data);
		mutex_unlock(&data->mutex);
	}

	return error < 0 ? error : cnt;
}

static ssize_t
lsm303d_acc_show_scale(struct device *dev,
		       struct device_attribute *attr, char *buf)
{
	unsigned long scale;
	struct lsm303d_data *data;

	data = dev_get_drvdata(dev);

	mutex_lock(&data->mutex);
	scale = data->acc_scale;
	mutex_unlock(&data->mutex);

	return scnprintf(buf, PAGE_SIZE, "%lu\n", scale);
}

static ssize_t
lsm303d_acc_store_scale(struct device *dev,
			struct device_attribute *attr, const char *buf,
			size_t cnt)
{
	int error;
	unsigned long scale;
	struct lsm303d_data *data;

	data = dev_get_drvdata(dev);

	error = strict_strtoul(buf, 0, &scale);
	if (error >= 0) {
		mutex_lock(&data->mutex);
		data->acc_skipcount = LSM303D_SKIPCOUNT;
		data->acc_scale = lsm303d_acc_adjust_scale(scale);
		data->acc_autoscale = (scale == 0);
		error = lsm303d_acc_update_device(data);
		mutex_unlock(&data->mutex);
	}

	return error < 0 ? error : cnt;
}

static ssize_t
lsm303d_acc_show_selftest(struct device *dev,
			  struct device_attribute *attr, char *buf)
{
	unsigned long testmode;
	struct lsm303d_data *data;

	data = dev_get_drvdata(dev);

	mutex_lock(&data->mutex);
	testmode = data->acc_testmode;
	mutex_unlock(&data->mutex);

	return scnprintf(buf, PAGE_SIZE, "%lu\n", testmode);
}

static ssize_t
lsm303d_acc_store_selftest(struct device *dev,
			   struct device_attribute *attr, const char *buf,
			   size_t cnt)
{
	int error;
	unsigned long testmode;
	struct lsm303d_data *data;

	data = dev_get_drvdata(dev);

	error = strict_strtoul(buf, 0, &testmode);
	if (error >= 0) {
		mutex_lock(&data->mutex);
		data->acc_testmode = testmode;
		error = lsm303d_acc_update_device(data);
		mutex_unlock(&data->mutex);
	}

	return error < 0 ? error : cnt;
}

LSM303D_ATTR(acc, enable, S_IWUSR | S_IRUGO,
	     lsm303d_acc_show_enable, lsm303d_acc_store_enable);
LSM303D_ATTR(acc, dbgdump, S_IWUSR | S_IRUGO,
	     lsm303d_acc_show_dbgdump, lsm303d_acc_store_dbgdump);
LSM303D_ATTR(acc, poll_delay, S_IWUSR | S_IRUGO,
	     lsm303d_acc_show_period, lsm303d_acc_store_period);
LSM303D_ATTR(acc, scale, S_IWUSR | S_IRUGO,
	     lsm303d_acc_show_scale, lsm303d_acc_store_scale);
LSM303D_ATTR(acc, selftest, S_IWUSR | S_IRUGO,
	     lsm303d_acc_show_selftest, lsm303d_acc_store_selftest);

static struct attribute *lsm303d_acc_attrs[] = {
	&lsm303d_acc_enable_attr.attr,
	&lsm303d_acc_dbgdump_attr.attr,
	&lsm303d_acc_poll_delay_attr.attr,
	&lsm303d_acc_scale_attr.attr,
	&lsm303d_acc_selftest_attr.attr,
	NULL
};

static struct attribute_group lsm303d_acc_attr_grp = {
	.attrs = lsm303d_acc_attrs,
};

static const struct attribute_group *lsm303d_acc_attr_grps[] = {
	&lsm303d_acc_attr_grp,
	NULL
};

static int lsm303d_acc_open(struct input_dev *dev)
{
	int error = 0;
	struct lsm303d_data *data = input_get_drvdata(dev);

	mutex_lock(&data->mutex);
	data->acc_opened = true;
	error = lsm303d_acc_update_device(data);
	mutex_unlock(&data->mutex);

	return error;
}

static void lsm303d_acc_close(struct input_dev *dev)
{
	struct lsm303d_data *data = input_get_drvdata(dev);

	mutex_lock(&data->mutex);
	data->acc_opened = false;
	lsm303d_acc_update_device(data);
	mutex_unlock(&data->mutex);
}

static int lsm303d_acc_interrupt(struct lsm303d_data *data)
{
	s16 x, y, z;
	int xmg, ymg, zmg, count = 0;
	struct lsm303d_platform_data *pdata;

	pdata = data->client->dev.platform_data;
	mutex_lock(&data->mutex);

	if (data->suspended) {
		mutex_unlock(&data->mutex);
		return count;
	}
	while (lsm303d_acc_read_output(data, &x, &y, &z) >= 0) {
		if (data->acc_skipcount == 0) {
			xmg = lsm303d_acc_scale_output(x, data->acc_scale);
			ymg = lsm303d_acc_scale_output(y, data->acc_scale);
			zmg = lsm303d_acc_scale_output(z, data->acc_scale);

			if (!lsm303d_acc_is_overflow(x, y, z)) {
				input_report_abs(data->acc_input,
						 pdata->acc_xcode,
						 pdata->acc_xdir * xmg);
				input_report_abs(data->acc_input,
						 pdata->acc_ycode,
						 pdata->acc_ydir * ymg);
				input_report_abs(data->acc_input,
						 pdata->acc_zcode,
						 pdata->acc_zdir * zmg);
				input_always_sync(data->acc_input);
			}

			lsm303d_acc_update_scale(data, x, y, z);

			if (data->acc_dbgdump) {
				dev_info(&data->client->dev, "x=%06hd(%06dmg),"
					 " y=%06hd(%06dmg), z=%06hd(%06dmg).",
					 x, xmg, y, ymg, z, zmg);
			}
		} else {
			data->acc_skipcount--;
			if (data->acc_dbgdump) {
				dev_info(&data->client->dev,
					 "skip process acc output.");
			}
		}
		count++;	/* how many sample we have read */
	}

	mutex_unlock(&data->mutex);
	return count;
}

static int lsm303d_acc_setup(struct lsm303d_data *data)
{
	int error;
	struct i2c_client *client;
	struct lsm303d_platform_data *pdata;

	client = data->client;
	pdata = client->dev.platform_data;

	data->acc_period = lsm303d_acc_adjust_period(pdata->acc_period);
	data->acc_scale = lsm303d_acc_adjust_scale(pdata->acc_scale);
	data->acc_autoscale = (pdata->acc_scale == 0);

	/* enable data ready interrupt */
	error = i2c_smbus_write_bits_data(data->client,
					  LSM303D_CNTRL3, LSM303D_P1_DRDYA,
					  LSM303D_P1_DRDYA);
	if (error < 0) {
		dev_err(&client->dev,
			"fail to enable acc data ready interrupt");
		goto exit;
	}

	/* register to input system */
	data->acc_input = input_allocate_device();
	if (data->acc_input == NULL) {
		dev_err(&client->dev, "fail to allocate acc input device.");
		error = -ENOMEM;
		goto exit;
	}
	input_set_drvdata(data->acc_input, data);

	data->acc_input->name = "accelerometer";
	data->acc_input->open = lsm303d_acc_open;
	data->acc_input->close = lsm303d_acc_close;
	data->acc_input->dev.groups = lsm303d_acc_attr_grps;

	input_set_capability(data->acc_input, EV_ABS, pdata->acc_xcode);
	input_set_abs_params(data->acc_input, pdata->acc_xcode,
			     LSM303D_AMIN, LSM303D_AMAX, LSM303D_ARES, 0);

	input_set_capability(data->acc_input, EV_ABS, pdata->acc_ycode);
	input_set_abs_params(data->acc_input, pdata->acc_ycode,
			     LSM303D_AMIN, LSM303D_AMAX, LSM303D_ARES, 0);

	input_set_capability(data->acc_input, EV_ABS, pdata->acc_zcode);
	input_set_abs_params(data->acc_input, pdata->acc_zcode,
			     LSM303D_AMIN, LSM303D_AMAX, LSM303D_ARES, 0);

	error = input_register_device(data->acc_input);
	if (error < 0) {
		dev_err(&client->dev, "fail to register acc input device.");
		goto exit_free_device;
	}

	data->acc_input->phys =
	    kobject_get_path(&data->acc_input->dev.kobj, GFP_KERNEL);
	if (data->acc_input->phys == NULL) {
		dev_err(&client->dev, "fail to get acc input sysfs path.");
		error = -ENOMEM;
		goto exit_unregister_device;
	}

	goto exit;		/* all is fine */

exit_unregister_device:
	input_unregister_device(data->acc_input);
	goto exit;
exit_free_device:
	input_free_device(data->acc_input);
exit:
	return error;
}

static void lsm303d_acc_teardown(struct lsm303d_data *data)
{
	kfree(data->acc_input->phys);
	input_unregister_device(data->acc_input);
}

/* i2c client driver interface */
static irqreturn_t lsm303d_interrupt(int irq, void *dev)
{
	int mag_count, acc_count;

	do {
		mag_count = lsm303d_mag_interrupt(dev);
		acc_count = lsm303d_acc_interrupt(dev);
	} while (mag_count || acc_count);

	return IRQ_HANDLED;
}

#if defined(CONFIG_PM) || defined(CONFIG_SENSORS_LSM303D_EARLYSUSPEND)
static int lsm303d_suspend(struct device *dev)
{
	int error = 0;
	struct lsm303d_data *data = dev_get_drvdata(dev);

	mutex_lock(&data->mutex);
	data->suspended = true;
	error = lsm303d_mag_update_device(data);
	if (error >= 0)
		error = lsm303d_acc_update_device(data);

	disable_irq_nosync(data->client->irq);

	mutex_unlock(&data->mutex);

	return error;
}

static int lsm303d_resume(struct device *dev)
{
	int error = 0;
	struct lsm303d_data *data = dev_get_drvdata(dev);

	mutex_lock(&data->mutex);
	data->suspended = false;
	error = lsm303d_mag_update_device(data);
	if (error >= 0)
		error = lsm303d_acc_update_device(data);

	enable_irq(data->client->irq);

	mutex_unlock(&data->mutex);

	return error;
}
#endif

#ifdef CONFIG_PM
static const struct dev_pm_ops lsm303d_pm_ops = {
#ifndef CONFIG_SENSORS_LSM303D_EARLYSUSPEND
	.suspend = lsm303d_suspend,
	.resume = lsm303d_resume,
#endif
};
#endif

#ifdef CONFIG_SENSORS_LSM303D_EARLYSUSPEND
static void lsm303d_early_suspend(struct early_suspend *h)
{
	struct lsm303d_data *data = container_of(h,
						 struct lsm303d_data,
						 early_suspend);
	lsm303d_suspend(&data->client->dev);
}

static void lsm303d_early_resume(struct early_suspend *h)
{
	struct lsm303d_data *data = container_of(h,
						 struct lsm303d_data,
						 early_suspend);
	lsm303d_resume(&data->client->dev);
}
#endif

static int
lsm303d_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	s32 chip;
	int error;
	struct lsm303d_data *data;

	/* verify device */
	if (i2c_check_functionality(client->adapter, LSM303D_FUNC) == 0) {
		dev_err(&client->dev, "incompatible adapter.");
		error = -ENODEV;
		goto exit;
	}

	chip = i2c_smbus_read_byte_data(client, LSM303D_WHO_AM_I);
	if (chip != LSM303D_CHIP_ID) {
		dev_err(&client->dev, "fail to detect LSM303D chip.");
		error = -ENODEV;
		goto exit;
	}

	/* hack for devices which have the same i2c address as lsm303d */
	if (id->driver_data) {	/* use default platform data instead */
		client->dev.platform_data = (void *)id->driver_data;
		dev_info(&client->dev, "%s use default platform data.",
			 id->name);
	} else if (client->dev.platform_data == NULL) {
		dev_err(&client->dev, "invalid platform data.");
		error = -EINVAL;
		goto exit;
	}

	/* allocate device status */
	data = kzalloc(sizeof(*data), GFP_KERNEL);
	if (data == NULL) {
		dev_err(&client->dev, "fail to allocate lsm303d_data.");
		error = -ENOMEM;
		goto exit;
	}

	i2c_set_clientdata(client, data);
	mutex_init(&data->mutex);
	data->client = client;

	/* run device setup */
	error = lsm303d_mag_setup(data);
	if (error < 0) {
		dev_err(&client->dev, "fail to perform mag setup.");
		goto exit_free_data;
	}

	error = lsm303d_acc_setup(data);
	if (error < 0) {
		dev_err(&client->dev, "fail to perform acc setup.");
		goto exit_mag_teardown;
	}

	/* setup interrupt */
	error = request_threaded_irq(client->irq, NULL, lsm303d_interrupt,
				     IRQF_TRIGGER_HIGH | IRQF_ONESHOT,
				     "lsm303d", data);
	if (error < 0) {
		dev_err(&client->dev, "fail to request interrupt.");
		goto exit_acc_teardown;
	}
#ifdef CONFIG_SENSORS_LSM303D_EARLYSUSPEND
	data->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	data->early_suspend.suspend = lsm303d_early_suspend;
	data->early_suspend.resume = lsm303d_early_resume;
	register_early_suspend(&data->early_suspend);
#endif

	goto exit;		/* all is fine */

exit_acc_teardown:
	lsm303d_acc_teardown(data);
exit_mag_teardown:
	lsm303d_mag_teardown(data);
exit_free_data:
	kfree(data);
exit:
	return error;
}

static int lsm303d_remove(struct i2c_client *client)
{
	struct lsm303d_data *data;
	data = i2c_get_clientdata(client);

#ifdef CONFIG_SENSORS_LSM303D_EARLYSUSPEND
	unregister_early_suspend(&data->early_suspend);
#endif

	free_irq(client->irq, data);

	lsm303d_mag_teardown(data);
	lsm303d_acc_teardown(data);

	kfree(data);
	return 0;
}

static struct lsm303d_platform_data lsm303d_default_pdata = {
	.mag_xdir = -1,
	.mag_ydir = 1,
	.mag_zdir = 1,
	.mag_xcode = ABS_Y,
	.mag_ycode = ABS_X,
	.mag_zcode = ABS_Z,
	.mag_period = 200000000,
	.mag_lowthres = 0x3e00,
	.mag_highthres = 0x7e00,
	.acc_xdir = -1,
	.acc_ydir = 1,
	.acc_zdir = 1,
	.acc_xcode = ABS_Y,
	.acc_ycode = ABS_X,
	.acc_zcode = ABS_Z,
	.acc_period = 200000000,
	.acc_lowthres = 0x3e00,
	.acc_highthres = 0x7e00,
};

static const struct i2c_device_id lsm303d_ids[] = {
	{"lsm303d", (kernel_ulong_t) NULL},
	{"hmc5883l", (kernel_ulong_t) &lsm303d_default_pdata},
	{ /* list end */ },
};

MODULE_DEVICE_TABLE(i2c, lsm303d_ids);

static struct i2c_driver lsm303d_driver = {
	.probe = lsm303d_probe,
	.remove = lsm303d_remove,
	.driver = {
		   .name = "lsm303d",
		   .pm = &lsm303d_pm_ops,
		   },
	.id_table = lsm303d_ids,
};

static int __init lsm303d_init(void)
{
	return i2c_add_driver(&lsm303d_driver);
}

static void __exit lsm303d_exit(void)
{
	i2c_del_driver(&lsm303d_driver);
}

module_init(lsm303d_init);
module_exit(lsm303d_exit);

MODULE_AUTHOR("Xiang Xiao <xiaoxiang@xiaomi.com>");
MODULE_DESCRIPTION("LSM303D input driver");
MODULE_LICENSE("GPL");
