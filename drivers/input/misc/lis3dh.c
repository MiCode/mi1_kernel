/* drivers/input/misc/lis3dh.c
 *
 * Copyright (C) 2011 XiaoMi, Inc.
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
#include <linux/input/lis3dh.h>
#ifdef CONFIG_SENSORS_LIS3DH_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif

/* lis3dh register list */
#define LIS3DH_STATUS_REG_AUX				0x07
#define LIS3DH_OUT_ADC1_L				0x08
#define LIS3DH_OUT_ADC1_H				0x09
#define LIS3DH_OUT_ADC2_L				0x0a
#define LIS3DH_OUT_ADC2_H				0x0b
#define LIS3DH_OUT_ADC3_L				0x0c
#define LIS3DH_OUT_ADC3_H				0x0d
#define LIS3DH_INT_COUNTER_REG				0x0e
#define LIS3DH_WHO_AM_I					0x0f
#define LIS3DH_TEMP_CFG_REG				0x1f
#define LIS3DH_CTRL_REG1				0x20
#define LIS3DH_CTRL_REG2				0x21
#define LIS3DH_CTRL_REG3				0x22
#define LIS3DH_CTRL_REG4				0x23
#define LIS3DH_CTRL_REG5				0x24
#define LIS3DH_CTRL_REG6				0x25
#define LIS3DH_REFERENCE				0x26
#define LIS3DH_STATUS_REG2				0x27
#define LIS3DH_OUT_X_L					0x28
#define LIS3DH_OUT_X_H					0x29
#define LIS3DH_OUT_Y_L					0x2a
#define LIS3DH_OUT_Y_H					0x2b
#define LIS3DH_OUT_Z_L					0x2c
#define LIS3DH_OUT_Z_H					0x2d
#define LIS3DH_FIFO_CTRL_REG				0x2e
#define LIS3DH_FIFO_SRC_REG				0x2f
#define LIS3DH_INT1_CFG					0x30
#define LIS3DH_INT1_SOURCE				0x31
#define LIS3DH_INT1_THS					0x32
#define LIS3DH_INT1_DURATION				0x33
#define LIS3DH_CLICK_CFG				0x38
#define LIS3DH_CLICK_SRC				0x39
#define LIS3DH_CLICK_THS				0x3a
#define LIS3DH_TIME_LIMIT				0x3b
#define LIS3DH_TIME_LATENCY				0x3c
#define LIS3DH_TIME_WINDOW				0x3d

/* enable address auto increment(or with address) */
#define LIS3DH_AUTO_INC					0x80

/* lis3dh identifier */
#define LIS3DH_CHIP_ID					0x33

/* output data rate(normal mode and 3 axis) */
#define LIS3DH_ODR_1HZ					0x17
#define LIS3DH_ODR_10HZ					0x27
#define LIS3DH_ODR_25HZ					0x37
#define LIS3DH_ODR_50HZ					0x47
#define LIS3DH_ODR_100HZ				0x57
#define LIS3DH_ODR_200HZ				0x67
#define LIS3DH_ODR_400HZ				0x77
#define LIS3DH_ODR_1250HZ				0x97

#define LIS3DH_POWER_OFF				0x00

/* interrupt enable bit for int1 */
#define LIS3DH_I1_CLICK					0x80
#define LIS3DH_I1_AOI1					0x40
#define LIS3DH_I1_AOI2					0x20
#define LIS3DH_I1_DRDY1					0x10
#define LIS3DH_I1_DRDY2					0x08
#define LIS3DH_I1_WTM					0x04
#define LIS3DH_I1_OVERRUN				0x02

/* full scale selection(block update, little endian and high resolution) */
#define LIS3DH_FS_2G					0x88
#define LIS3DH_FS_4G					0x98
#define LIS3DH_FS_8G					0xa8
#define LIS3DH_FS_16G					0xb8

/* self test mode selection */
#define LIS3DH_ST_NORMAL				0x00
#define LIS3DH_ST_MODE0					0x02
#define LIS3DH_ST_MODE1					0x04

/* interrupt status flag */
#define LIS3DH_ZYXOR					0x80
#define LIS3DH_ZOR					0x40
#define LIS3DH_YOR					0x20
#define LIS3DH_XOR					0x10
#define LIS3DH_ZYXDA					0x08
#define LIS3DH_ZDA					0x04
#define LIS3DH_YDA					0x02
#define LIS3DH_XDA					0x01

/* interrupt configuration */
#define LIS3DH_AOI					0x80
#define LIS3DH_6D					0x40
#define LIS3DH_ZHIE					0x20
#define LIS3DH_ZLIE					0x10
#define LIS3DH_YHIE					0x08
#define LIS3DH_YLIE					0x04
#define LIS3DH_XHIE					0x02
#define LIS3DH_XLIE					0x01

/* to avoid the unstable result
   skip process output several time when scale change */
#define LIS3DH_SKIPCOUNT				2

/* output overflow flag */
#define LIS3DH_OVERFLOW					32767

/* functionality need to support by i2c adapter */
#define LIS3DH_FUNC	\
	(I2C_FUNC_SMBUS_BYTE_DATA | I2C_FUNC_SMBUS_READ_I2C_BLOCK)

/* supported acceleration range and resolution(unit is mg) */
#define LIS3DH_MIN					-16000
#define LIS3DH_MAX					16000
#define LIS3DH_RES					12

/* lis3dh device context
   if auto_scale != 0 then
      if max_output <= low_thres, then move to next small scale
      if max_output >= high_thres, then move to next big scale
   else always use the specified scale value */
struct lis3dh_data {
	struct mutex		mutex;
	struct input_dev	*input;
	struct i2c_client	*client;
#ifdef CONFIG_SENSORS_LIS3DH_EARLYSUSPEND
	struct early_suspend	early_suspend;
#endif
	bool			opened;
	bool			enabled;
	bool			mode_6d;
	bool			suspended;
	unsigned long		skipcount;
	unsigned long		period;		/* unit: ns */
	unsigned long		scale;		/* unit: mg */
	bool			auto_scale;
	/* 0:normal, 1:mode 0, 2:mode 1 */
	unsigned long		test_mode;
};

/* lis3dh utility function */
static unsigned long adjust_period(unsigned long period)
{
	period = max(period, 10000000UL);

	if (period >= 1000000000)	/* 1s(1Hz) */
		return 1000000000;
	else if (period >= 100000000)	/* 100ms(10Hz) */
		return 100000000;
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
	else if (period >= 800000)	/* 0.8ms(1250Hz) */
		return 800000;

	return 800000; /* out of range return as fast as we can */
}

static unsigned long adjust_scale(unsigned long scale)
{
	if (scale == 0)			/* auto scale */
		return 2000;
	else if (scale <= 2000)		/* 2G */
		return 2000;
	else if (scale <= 4000)		/* 4G */
		return 4000;
	else if (scale <= 8000)		/* 8G */
		return 8000;
	else if (scale <= 16000)	/* 16G */
		return 16000;

	return 16000; /* out of range, return as large as we can */
}

static unsigned long get_next_small_scale(unsigned long scale)
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
		BUG(); /* scale should already adjust by adjust_scale */
	}
}

static unsigned long get_next_big_scale(unsigned long scale)
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
		BUG(); /* scale should already adjust by adjust_scale */
	}
}

static unsigned long get_sensitivity(unsigned long scale)
{
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

static bool output_overflow(s16 x, s16 y, s16 z)
{
	return  abs(x) >= LIS3DH_OVERFLOW ||
		abs(y) >= LIS3DH_OVERFLOW ||
		abs(z) >= LIS3DH_OVERFLOW;
}

static int scale_output(s16 output, unsigned long scale)
{
	return (int)get_sensitivity(scale) * (output >> 4);
}

static u8 get_ctrl_reg1(bool active, unsigned long period)
{
	if (active) {
		switch (period) {
		case 1000000000:
			return LIS3DH_ODR_1HZ;
		case 100000000:
			return LIS3DH_ODR_10HZ;
		case 40000000:
			return LIS3DH_ODR_25HZ;
		case 20000000:
			return LIS3DH_ODR_50HZ;
		case 10000000:
			return LIS3DH_ODR_100HZ;
		case 5000000:
			return LIS3DH_ODR_200HZ;
		case 2500000:
			return LIS3DH_ODR_400HZ;
		case 800000:
			return LIS3DH_ODR_1250HZ;
		default:
			BUG(); /* should already adjust by adjust_period */
		}
	} else {
		return LIS3DH_POWER_OFF;
	}
}

static u8 get_ctrl_reg4(unsigned long scale, unsigned long mode)
{
	u8 reg4 = 0;

	switch (scale) {
	case 2000:
		reg4 |= LIS3DH_FS_2G;
		break;
	case 4000:
		reg4 |= LIS3DH_FS_4G;
		break;
	case 8000:
		reg4 |= LIS3DH_FS_8G;
		break;
	case 16000:
		reg4 |= LIS3DH_FS_16G;
		break;
	default:
		BUG();/* scale should already adjust by adjust_scale */
	}

	switch (mode) {
	case 1:
		reg4 |= LIS3DH_ST_MODE0;
		break;
	case 2:
		reg4 |= LIS3DH_ST_MODE1;
		break;
	}

	return reg4;
}

static int read_output(struct lis3dh_data *data, s16 *x, s16 *y, s16 *z)
{
	u8			buf[6];
	s32			status;
	int			error	= 0;
	struct i2c_client	*client	= data->client;

	status = i2c_smbus_read_byte_data(client, LIS3DH_STATUS_REG2);
	if (status < 0) {
		error = status;
		dev_err(&client->dev, "fail to read status register.");
		goto exit;
	}

	if ((status & LIS3DH_ZYXDA) == 0) {
		error = -EAGAIN; /* output isn't ready, try later again. */
		goto exit;
	}

	error = i2c_smbus_read_i2c_block_data(client,
			LIS3DH_AUTO_INC | LIS3DH_OUT_X_L,
			sizeof(buf), buf);
	if (error < 0) {
		dev_err(&client->dev, "fail to read output register.");
		goto exit;
	}

	*x = (buf[1] << 8) | buf[0];
	*y = (buf[3] << 8) | buf[2];
	*z = (buf[5] << 8) | buf[4];

exit:
	return error;
}

static bool device_active(struct lis3dh_data *data)
{
	return data->opened && data->enabled && !data->suspended;
}

static int update_device(struct lis3dh_data *data)
{
	int				error	= 0;
	struct i2c_client		*client	= data->client;
	struct lis3dh_platform_data	*pdata	= client->dev.platform_data;

	/* disable sensor before update setting */
	error = i2c_smbus_write_byte_data(client,
			LIS3DH_CTRL_REG1, LIS3DH_POWER_OFF);
	if (error < 0) {
		dev_err(&client->dev, "fail to disable sensor.");
		goto exit;
	}

	/* update 6d threshold register */
	error = i2c_smbus_write_byte_data(client, LIS3DH_INT1_THS,
			(128 * pdata->thres_6d) / data->scale);
	if (error < 0) {
		dev_err(&client->dev, "fail to set 6d threshold.");
		goto exit;
	}

	/* update scale register */
	error = i2c_smbus_write_byte_data(client, LIS3DH_CTRL_REG4,
			get_ctrl_reg4(data->scale, data->test_mode));
	if (error < 0) {
		dev_err(&client->dev, "fail to set scale range.");
		goto exit;
	}

	/* reset interrupt register */
	error = i2c_smbus_write_byte_data(client, LIS3DH_CTRL_REG3,
			data->mode_6d ? LIS3DH_I1_AOI1 : LIS3DH_I1_DRDY1);
	if (error < 0) {
		dev_err(&client->dev, "fail to reset interrupt.");
		goto exit;
	}

	/* update period register */
	error = i2c_smbus_write_byte_data(client, LIS3DH_CTRL_REG1,
			get_ctrl_reg1(device_active(data), data->period));
	if (error < 0) {
		dev_err(&client->dev, "fail to set period.");
		goto exit;
	}

exit:
	return error;
}

static void update_scale(struct lis3dh_data *data, s16 x, s16 y, s16 z)
{
	unsigned long next_scale = data->scale;
	unsigned long max_output = max(max(abs(x), abs(y)), abs(z));
	struct lis3dh_platform_data *pdata = data->client->dev.platform_data;

	if (max_output >= pdata->high_thres) {
		next_scale = get_next_big_scale(data->scale);
		if (next_scale == data->scale)
			dev_dbg(&data->client->dev, "out of scale.");
	} else if (max_output <= pdata->low_thres) {
		next_scale = get_next_small_scale(data->scale);
	}

	if (next_scale != data->scale) {
		if (data->auto_scale) {
			dev_dbg(&data->client->dev,
				"update scale range(%lu).", next_scale);
			data->skipcount = LIS3DH_SKIPCOUNT;
			data->scale = next_scale;
			update_device(data);
		} else {
			dev_dbg(&data->client->dev,
				"out of range(auto scale disable).");
		}
	}
}

/* sysfs interface */
static ssize_t lis3dh_show_enable(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	unsigned long		enabled;
	struct lis3dh_data	*data;

	data = dev_get_drvdata(dev);

	mutex_lock(&data->mutex);
	enabled = data->enabled;
	mutex_unlock(&data->mutex);

	return scnprintf(buf, PAGE_SIZE, "%lu\n", enabled);
}

static ssize_t lis3dh_store_enable(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t cnt)
{
	int			error;
	unsigned long		enabled;
	struct lis3dh_data	*data;

	data = dev_get_drvdata(dev);

	error = strict_strtoul(buf, 0, &enabled);
	if (error >= 0) {
		mutex_lock(&data->mutex);
		data->enabled = enabled;
		error = update_device(data);
		mutex_unlock(&data->mutex);
	}

	return error < 0 ? error : cnt;
}

static ssize_t lis3dh_show_mode_6d(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct lis3dh_data	*data;
	unsigned long		mode_6d;

	data = dev_get_drvdata(dev);

	mutex_lock(&data->mutex);
	mode_6d = data->mode_6d;
	mutex_unlock(&data->mutex);

	return scnprintf(buf, PAGE_SIZE, "%lu\n", mode_6d);
}

static ssize_t lis3dh_store_mode_6d(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t cnt)
{
	int			error;
	struct lis3dh_data	*data;
	unsigned long		mode_6d;

	data = dev_get_drvdata(dev);

	error = strict_strtoul(buf, 0, &mode_6d);
	if (error >= 0) {
		mutex_lock(&data->mutex);
		data->mode_6d = mode_6d;
		error = update_device(data);
		mutex_unlock(&data->mutex);
	}

	return error < 0 ? error : cnt;
}

static ssize_t lis3dh_show_period(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	unsigned long		period;
	struct lis3dh_data	*data;

	data = dev_get_drvdata(dev);

	mutex_lock(&data->mutex);
	period = data->period;
	mutex_unlock(&data->mutex);

	return scnprintf(buf, PAGE_SIZE, "%lu\n", period);
}

static ssize_t lis3dh_store_period(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t cnt)
{
	int			error;
	unsigned long		period;
	struct lis3dh_data	*data;

	data = dev_get_drvdata(dev);

	error = strict_strtoul(buf, 0, &period);
	if (error >= 0) {
		mutex_lock(&data->mutex);
		data->period = adjust_period(period);
		error = update_device(data);
		mutex_unlock(&data->mutex);
	}

	return error < 0 ? error : cnt;
}

static ssize_t lis3dh_show_scale(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	unsigned long		scale;
	struct lis3dh_data	*data;

	data = dev_get_drvdata(dev);

	mutex_lock(&data->mutex);
	scale = data->scale;
	mutex_unlock(&data->mutex);

	return scnprintf(buf, PAGE_SIZE, "%lu\n", scale);
}

static ssize_t lis3dh_store_scale(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t cnt)
{
	int			error;
	unsigned long		scale;
	struct lis3dh_data	*data;

	data = dev_get_drvdata(dev);

	error = strict_strtoul(buf, 0, &scale);
	if (error >= 0) {
		mutex_lock(&data->mutex);
		data->skipcount  = LIS3DH_SKIPCOUNT;
		data->scale      = adjust_scale(scale);
		data->auto_scale = (scale == 0);
		error = update_device(data);
		mutex_unlock(&data->mutex);
	}

	return error < 0 ? error : cnt;
}

static ssize_t lis3dh_show_selftest(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	unsigned long		test_mode;
	struct lis3dh_data	*data;

	data = dev_get_drvdata(dev);

	mutex_lock(&data->mutex);
	test_mode = data->test_mode;
	mutex_unlock(&data->mutex);

	return scnprintf(buf, PAGE_SIZE, "%lu\n", test_mode);
}

static ssize_t lis3dh_store_selftest(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t cnt)
{
	int			error;
	unsigned long		test_mode;
	struct lis3dh_data	*data;

	data = dev_get_drvdata(dev);

	error = strict_strtoul(buf, 0, &test_mode);
	if (error >= 0) {
		mutex_lock(&data->mutex);
		data->test_mode = test_mode;
		error = update_device(data);
		mutex_unlock(&data->mutex);
	}

	return error < 0 ? error : cnt;
}

static DEVICE_ATTR(enable, S_IWUSR | S_IRUGO,
		lis3dh_show_enable, lis3dh_store_enable);
static DEVICE_ATTR(mode_6d, S_IWUSR | S_IRUGO,
		lis3dh_show_mode_6d, lis3dh_store_mode_6d);
static DEVICE_ATTR(poll_delay, S_IWUSR | S_IRUGO,
		lis3dh_show_period, lis3dh_store_period);
static DEVICE_ATTR(scale, S_IWUSR | S_IRUGO,
		lis3dh_show_scale, lis3dh_store_scale);
static DEVICE_ATTR(selftest, S_IWUSR | S_IRUGO,
		lis3dh_show_selftest, lis3dh_store_selftest);

static struct attribute *lis3dh_attrs[] = {
	&dev_attr_enable.attr    ,
	&dev_attr_mode_6d.attr   ,
	&dev_attr_poll_delay.attr,
	&dev_attr_scale.attr     ,
	&dev_attr_selftest.attr  ,
	NULL
};

static struct attribute_group lis3dh_attr_grp = {
	.attrs = lis3dh_attrs,
};

static const struct attribute_group *lis3dh_attr_grps[] = {
	&lis3dh_attr_grp,
	NULL
};

/* input device driver interface */
static int lis3dh_open(struct input_dev *dev)
{
	int			error = 0;
	struct lis3dh_data	*data = input_get_drvdata(dev);

	mutex_lock(&data->mutex);
	data->opened = true;
	error = update_device(data);
	mutex_unlock(&data->mutex);

	return error;
}

static void lis3dh_close(struct input_dev *dev)
{
	struct lis3dh_data *data = input_get_drvdata(dev);

	mutex_lock(&data->mutex);
	data->opened = false;
	update_device(data);
	mutex_unlock(&data->mutex);
}

/* interrupt handler */
static irqreturn_t lis3dh_interrupt(int irq, void *dev)
{
	s16				x, y, z;
	int				xmg, ymg, zmg;
	struct lis3dh_data		*data;
	struct lis3dh_platform_data	*pdata;

	data = dev;
	pdata = data->client->dev.platform_data;
	mutex_lock(&data->mutex);

	while (read_output(data, &x, &y, &z) >= 0) {
		if (data->skipcount == 0) {
			xmg = scale_output(x, data->scale);
			ymg = scale_output(y, data->scale);
			zmg = scale_output(z, data->scale);

			if (device_active(data) && !output_overflow(x, y, z)) {
				input_report_abs(data->input,
					pdata->xcode, pdata->xdir*xmg);
				input_report_abs(data->input,
					pdata->ycode, pdata->ydir*ymg);
				input_report_abs(data->input,
					pdata->zcode, pdata->zdir*zmg);
				input_always_sync(data->input);
			}

			update_scale(data, x, y, z);

			dev_dbg(&data->client->dev, "x=%06hd(%06dmg),"
				" y=%06hd(%06dmg), z=%06hd(%06dmg).",
				x, xmg, y, ymg, z, zmg);
		} else {
			data->skipcount--;
			dev_dbg(&data->client->dev, "skip process output.");
		}
	}

	mutex_unlock(&data->mutex);
	return IRQ_HANDLED;
}

/* i2c client driver interface */
static int lis3dh_setup(struct lis3dh_data *data)
{
	struct lis3dh_platform_data *pdata = data->client->dev.platform_data;
	if (pdata->setup)
		return pdata->setup(data->client, pdata);
	else
		return 0;
}

static int lis3dh_teardown(struct lis3dh_data *data)
{
	struct lis3dh_platform_data *pdata = data->client->dev.platform_data;
	if (pdata->teardown)
		return pdata->teardown(data->client, pdata);
	else
		return 0;
}

#if defined(CONFIG_PM) || defined(CONFIG_SENSORS_LIS3DH_EARLYSUSPEND)
static int lis3dh_suspend(struct device *dev)
{
	int			error = 0;
	struct lis3dh_data	*data = dev_get_drvdata(dev);

	mutex_lock(&data->mutex);
	data->suspended = true;
	error = update_device(data);
	mutex_unlock(&data->mutex);

	return error;
}

static int lis3dh_resume(struct device *dev)
{
	int			error = 0;
	struct lis3dh_data	*data = dev_get_drvdata(dev);

	mutex_lock(&data->mutex);
	data->suspended = false;
	error = update_device(data);
	mutex_unlock(&data->mutex);

	return error;
}
#endif

#ifdef CONFIG_PM
static const struct dev_pm_ops lis3dh_pm_ops = {
#ifndef CONFIG_SENSORS_LIS3DH_EARLYSUSPEND
	.suspend	= lis3dh_suspend,
	.resume		= lis3dh_resume,
#endif
};
#endif

#ifdef CONFIG_SENSORS_LIS3DH_EARLYSUSPEND
static void lis3dh_early_suspend(struct early_suspend *h)
{
	struct lis3dh_data *data = container_of(h,
			struct lis3dh_data, early_suspend);
	lis3dh_suspend(&data->client->dev);
}

static void lis3dh_early_resume(struct early_suspend *h)
{
	struct lis3dh_data *data = container_of(h,
			struct lis3dh_data, early_suspend);
	lis3dh_resume(&data->client->dev);
}
#endif

static int lis3dh_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	s32				chip;
	int				error;
	struct lis3dh_data		*data;
	struct lis3dh_platform_data	*pdata;

	pdata = client->dev.platform_data;
	if (pdata == NULL) {
		dev_err(&client->dev, "invalid platform data.");
		error = -EINVAL;
		goto exit;
	}

	/* allocate device status */
	data = kzalloc(sizeof(*data), GFP_KERNEL);
	if (data == NULL) {
		dev_err(&client->dev, "fail to allocate lis3dh_data.");
		error = -ENOMEM;
		goto exit;
	}
	i2c_set_clientdata(client, data);

	mutex_init(&data->mutex);

	data->client		= client;
	data->period		= adjust_period(pdata->period);
	data->scale		= adjust_scale(pdata->scale);
	data->auto_scale	= (pdata->scale == 0);

	/* run platform setup */
	error = lis3dh_setup(data);
	if (error < 0) {
		dev_err(&client->dev, "fail to perform platform setup.");
		goto exit_free_data;
	}

	/* verify device */
	if (i2c_check_functionality(client->adapter, LIS3DH_FUNC) == 0) {
		dev_err(&client->dev, "incompatible adapter.");
		error = -ENODEV;
		goto exit_teardown;
	}

	chip = i2c_smbus_read_byte_data(client, LIS3DH_WHO_AM_I);
	if (chip != LIS3DH_CHIP_ID) {
		dev_err(&client->dev, "fail to detect LIS3DH chip.");
		error = -ENODEV;
		goto exit_teardown;
	}

	/* initialize the fixed register setting */
	error = i2c_smbus_write_byte_data(client,
			LIS3DH_INT1_DURATION, pdata->count_6d);
	if (error < 0) {
		dev_err(&client->dev, "fail to set 6d duration.");
		goto exit;
	}

	error = i2c_smbus_write_byte_data(client,
			LIS3DH_INT1_CFG, LIS3DH_ZHIE|LIS3DH_YHIE|LIS3DH_XHIE);
	if (error < 0) {
		dev_err(&client->dev, "fail to set 6d config.");
		goto exit;
	}

	/* register to input system */
	data->input = input_allocate_device();
	if (data->input == NULL) {
		dev_err(&client->dev, "fail to allocate input device.");
		error = -ENOMEM;
		goto exit_teardown;
	}
	input_set_drvdata(data->input, data);

	data->input->name	= "accelerometer";
	data->input->open	= lis3dh_open;
	data->input->close	= lis3dh_close;
	data->input->dev.groups	= lis3dh_attr_grps;

	input_set_capability(data->input, EV_ABS, pdata->xcode);
	input_set_abs_params(data->input, pdata->xcode,
			LIS3DH_MIN, LIS3DH_MAX, LIS3DH_RES, 0);

	input_set_capability(data->input, EV_ABS, pdata->ycode);
	input_set_abs_params(data->input, pdata->ycode,
			LIS3DH_MIN, LIS3DH_MAX, LIS3DH_RES, 0);

	input_set_capability(data->input, EV_ABS, pdata->zcode);
	input_set_abs_params(data->input, pdata->zcode,
			LIS3DH_MIN, LIS3DH_MAX, LIS3DH_RES, 0);

	error = input_register_device(data->input);
	if (error < 0) {
		dev_err(&client->dev, "fail to register input device.");
		goto exit_free_device;
	}

	data->input->phys =
		kobject_get_path(&data->input->dev.kobj, GFP_KERNEL);
	if (data->input->phys == NULL) {
		dev_err(&client->dev, "fail to get input sysfs path.");
		error = -ENOMEM;
		goto exit_unregister_device;
	}

	/* setup interrupt */
	error = request_threaded_irq(client->irq, NULL, lis3dh_interrupt,
			IRQF_TRIGGER_RISING, "lis3dh", data);
	if (error < 0) {
		dev_err(&client->dev, "fail to request irq.");
		goto exit_free_phys;
	}

#ifdef CONFIG_SENSORS_LIS3DH_EARLYSUSPEND
	data->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	data->early_suspend.suspend = lis3dh_early_suspend;
	data->early_suspend.resume = lis3dh_early_resume;
	register_early_suspend(&data->early_suspend);
#endif

	goto exit; /* all is fine */

exit_free_phys:
	kfree(data->input->phys);
exit_unregister_device:
	input_unregister_device(data->input);
	goto exit_teardown;
exit_free_device:
	input_free_device(data->input);
exit_teardown:
	lis3dh_teardown(data);
exit_free_data:
	kfree(data);
exit:
	return error;
}

static int lis3dh_remove(struct i2c_client *client)
{
	struct lis3dh_data *data;
	data = i2c_get_clientdata(client);

#ifdef CONFIG_SENSORS_LIS3DH_EARLYSUSPEND
	unregister_early_suspend(&data->early_suspend);
#endif

	free_irq(client->irq, data);

	kfree(data->input->phys);
	input_unregister_device(data->input);

	lis3dh_teardown(data);
	kfree(data);

	return 0;
}

static const struct i2c_device_id lis3dh_ids[] = {
	{"lis3dh", 0   },
	{/* list end */},
};

MODULE_DEVICE_TABLE(i2c, lis3dh_ids);

static struct i2c_driver lis3dh_driver = {
	.probe		= lis3dh_probe,
	.remove		= lis3dh_remove,
	.driver = {
		.name	= "lis3dh",
		.pm	= &lis3dh_pm_ops,
	},
	.id_table	= lis3dh_ids,
};

/* module initialization and termination */
static int __init lis3dh_init(void)
{
	return i2c_add_driver(&lis3dh_driver);
}

static void __exit lis3dh_exit(void)
{
	i2c_del_driver(&lis3dh_driver);
}

module_init(lis3dh_init);
module_exit(lis3dh_exit);

MODULE_AUTHOR("Xiang Xiao <xiaoxiang@xiaomi.com>");
MODULE_DESCRIPTION("LIS3DH accelerometer input driver");
MODULE_LICENSE("GPL");
