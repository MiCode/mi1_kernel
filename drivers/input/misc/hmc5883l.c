/* drivers/input/misc/hmc5883l.c
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
#include <linux/workqueue.h>
#include <linux/input/hmc5883l.h>
#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif

/* hmc5883l register list */
#define HMC5883L_REG_A					0
#define HMC5883L_REG_B					1
#define HMC5883L_REG_MODE				2
#define HMC5883L_REG_XMSB				3
#define HMC5883L_REG_XLSB				4
#define HMC5883L_REG_ZMSB				5
#define HMC5883L_REG_ZLSB				6
#define HMC5883L_REG_YMSB				7
#define HMC5883L_REG_YLSB				8
#define HMC5883L_REG_STATUS				9
#define HMC5883L_REG_IDA				10
#define HMC5883L_REG_IDB				11
#define HMC5883L_REG_IDC				12

/* sample averaged per measurement output */
#define HMC5883L_SAMPLE_1				0x00
#define HMC5883L_SAMPLE_2				0x20
#define HMC5883L_SAMPLE_4				0x40
#define HMC5883L_SAMPLE_8				0x60

/* gain configuration for all channels */
#define HMC5883L_GAIN0_88GA				0x00
#define HMC5883L_GAIN1_3GA				0x20
#define HMC5883L_GAIN1_9GA				0x40
#define HMC5883L_GAIN2_5GA				0x60
#define HMC5883L_GAIN4_0GA				0x80
#define HMC5883L_GAIN4_7GA				0xa0
#define HMC5883L_GAIN5_6GA				0xc0
#define HMC5883L_GAIN8_1GA				0xe0

/* operation mode */
#define HMC5883L_MODE_SINGLE				0x01
#define HMC5883L_MODE_SLEEP				0x03

/* output overflow */
#define HMC5883L_OVERFLOW				-4096

/* status bit */
#define HMC5883L_STATUS_READY				0x01
#define HMC5883L_STATUS_LOCK				0x02

/* hmc5883l identifier */
#define HMC5883L_CHIP_IDA				0x48
#define HMC5883L_CHIP_IDB				0x34
#define HMC5883L_CHIP_IDC				0x33

/* to avoid the unstable result
   skip process output several time when scale change */
#define HMC5883L_SKIPCOUNT				2

/* functionality need to support by i2c adapter */
#define HMC5883L_FUNC	\
	(I2C_FUNC_SMBUS_BYTE_DATA | I2C_FUNC_SMBUS_READ_I2C_BLOCK)

/* supported range and resolution(unit is mga) */
#define HMC5883L_MIN					-8100
#define HMC5883L_MAX					8100
#define HMC5883L_RES					5

/* hmc5883l device context
   if auto_scale != 0 then
      if output overflow, then move to next big scale
      otherwise select scale according datasheet(p12, table 9)
   else always use the specified scale value */
struct hmc5883l_data {
	struct workqueue_struct	*wq;
	struct delayed_work	work;
	struct mutex		mutex;
	struct input_dev	*input;
	struct i2c_client	*client;
#ifdef CONFIG_HAS_EARLYSUSPEND
	struct early_suspend	early_suspend;
#endif
	bool			opened;
	bool			enabled;
	bool			suspended;
	unsigned long		skipcount;
	unsigned long		dbgdump;
	unsigned long		period;		/* unit: ns */
	unsigned long		scale;		/* unit: mga */
	bool			auto_scale;
	/* 0:no bias, 1:pos bias, 2:neg bias */
	u8			test_mode;
};

/* hmc5883l utility function */
static unsigned long adjust_period(unsigned long period)
{
	unsigned long jiffies;

	period = max(period, 10000000UL);
	jiffies = nsecs_to_jiffies(period);

	return 1000*jiffies_to_usecs(jiffies);
}

static unsigned long adjust_scale(unsigned long scale)
{
	if (scale == 0)			/* auto scale */
		return 1300;
	else if (scale <= 880)		/* 0.88Ga */
		return 880;
	else if (scale <= 1300)		/* 1.3Ga */
		return 1300;
	else if (scale <= 1900)		/* 1.9Ga */
		return 1900;
	else if (scale <= 2500)		/* 2.5Ga */
		return 2500;
	else if (scale <= 4000)		/* 4.0Ga */
		return 4000;
	else if (scale <= 4700)		/* 4.7Ga */
		return 4700;
	else if (scale <= 5600)		/* 5.6Ga */
		return 5600;
	else if (scale <= 8100)		/* 8.1Ga */
		return 8100;

	return 8100; /* out of range */
}

static unsigned long get_recommended_scale(unsigned long output,
			unsigned long scale, unsigned long threshold)
{
	if (abs(output-scale) <= threshold)
		return scale;
	else if (output <= 880)
		return 880;
	else if (output <= 1300)
		return 1300;
	else if (output <= 1900)
		return 1900;
	else if (output <= 2500)
		return 2500;
	else if (output <= 4000)
		return 4000;
	else if (output <= 4700)
		return 4700;
	else if (output <= 5600)
		return 5600;
	else
		return 8100;
}

static unsigned long get_next_big_scale(unsigned long scale)
{
	switch (scale) {
	case 880:
		return 1300;
	case 1300:
		return 1900;
	case 1900:
		return 2500;
	case 2500:
		return 4000;
	case 4000:
		return 4700;
	case 4700:
		return 5600;
	case 5600:
		return 8100;
	case 8100:
		return 8100;
	default:
		BUG(); /* scale should already adjust by adjust_scale */
	}
}

static int scale_output(s16 output, unsigned long scale)
{
	int sign = (output >= 0 ? 1 : -1), gain;

	switch (scale) {
	case 880:
		gain = 1370; break;
	case 1300:
		gain = 1090; break;
	case 1900:
		gain = 820; break;
	case 2500:
		gain = 660; break;
	case 4000:
		gain = 440; break;
	case 4700:
		gain = 390; break;
	case 5600:
		gain = 330; break;
	case 8100:
		gain = 230; break;
	default:
		BUG(); /* scale should already adjust by adjust_scale */
	}

	return (1000 * output + sign * (gain / 2)) / gain;
}

static bool output_overflow(s16 x, s16 y, s16 z)
{
	return  x == HMC5883L_OVERFLOW ||
		y == HMC5883L_OVERFLOW ||
		z == HMC5883L_OVERFLOW;
}

static u8 get_rega(u8 test_mode)
{
	return HMC5883L_SAMPLE_8 | test_mode;
}

static u8 get_regb(unsigned long scale)
{
	switch (scale) {
	case 880:
		return HMC5883L_GAIN0_88GA;
	case 1300:
		return HMC5883L_GAIN1_3GA;
	case 1900:
		return HMC5883L_GAIN1_9GA;
	case 2500:
		return HMC5883L_GAIN2_5GA;
	case 4000:
		return HMC5883L_GAIN4_0GA;
	case 4700:
		return HMC5883L_GAIN4_7GA;
	case 5600:
		return HMC5883L_GAIN5_6GA;
	case 8100:
		return HMC5883L_GAIN8_1GA;
	default:
		BUG();/* scale should already adjust by adjust_scale */
	}
}

static int read_output(struct hmc5883l_data *data, s16 *x, s16 *y, s16 *z)
{
	u8			buf[6];
	s32			status;
	int			error	= 0;
	struct i2c_client	*client	= data->client;

	status = i2c_smbus_read_byte_data(client, HMC5883L_REG_STATUS);
	if (status < 0) {
		error = status;
		dev_err(&client->dev, "fail to read status register.");
		goto exit;
	}

	if ((status & HMC5883L_STATUS_READY) == 0) {
		dev_err(&client->dev, "x, y, z axis output isn't ready.");
		error = -EAGAIN;
		goto exit;
	}

	if (status & HMC5883L_STATUS_LOCK) {
		dev_err(&client->dev, "x, y, z axis output is locked.");
		error = -EBUSY;
		goto exit;
	}

	error = i2c_smbus_read_i2c_block_data(client,
			HMC5883L_REG_XMSB, sizeof(buf), buf);
	if (error < 0) {
		dev_err(&client->dev, "fail to read output register.");
		goto exit;
	}

	*x = (buf[0] << 8) | buf[1];
	*y = (buf[4] << 8) | buf[5];
	*z = (buf[2] << 8) | buf[3];

exit:
	return error;
}

static bool device_active(struct hmc5883l_data *data)
{
	return data->opened && data->enabled && !data->suspended;
}

static int update_device(struct hmc5883l_data *data)
{
	int			error	= 0;
	struct i2c_client	*client	= data->client;

	cancel_delayed_work(&data->work);

	if (device_active(data)) {
		error = i2c_smbus_write_byte_data(client,
				HMC5883L_REG_A, get_rega(data->test_mode));
		if (error < 0) {
			dev_err(&client->dev, "fail to set test mode.");
			goto exit;
		}

		error = i2c_smbus_write_byte_data(client,
				HMC5883L_REG_B, get_regb(data->scale));
		if (error < 0) {
			dev_err(&client->dev, "fail to set scale range.");
			goto exit;
		}

		/* start work immediately */
		queue_delayed_work(data->wq, &data->work, 0);
	}

exit:
	return error;
}

static void update_scale(struct hmc5883l_data *data,
		bool overflow, int x, int y, int z)
{
	unsigned long next_scale = data->scale;

	if (overflow) { /* move to next big scale level */
		next_scale = get_next_big_scale(data->scale);
		if (data->dbgdump >= 1 && next_scale == data->scale)
			dev_info(&data->client->dev, "out of scale.");
	} else if (data->test_mode == 0) { /* just work in no test mode */
		/* get recommended scale level from datasheet */
		struct hmc5883l_platform_data *pdata =
				data->client->dev.platform_data;
		unsigned long output = max(max(abs(x), abs(y)), abs(z));

		next_scale = get_recommended_scale(
				output, next_scale, pdata->threshold);
	}

	if (next_scale != data->scale) {
		if (data->auto_scale) {
			if (data->dbgdump >= 1)
				dev_info(&data->client->dev,
					"update scale range(%lu).", next_scale);
			data->skipcount = HMC5883L_SKIPCOUNT;
			data->scale = next_scale;
			update_device(data);
		} else if (data->dbgdump >= 1) {
			dev_info(&data->client->dev,
				"out of range(auto scale disable).");
		}
	}
}

/* sysfs interface */
static ssize_t hmc5883l_show_enable(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	unsigned long		enabled;
	struct hmc5883l_data	*data;

	data = dev_get_drvdata(dev);

	mutex_lock(&data->mutex);
	enabled = data->enabled;
	mutex_unlock(&data->mutex);

	return scnprintf(buf, PAGE_SIZE, "%lu\n", enabled);
}

static ssize_t hmc5883l_store_enable(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t cnt)
{
	int			error;
	unsigned long		enabled;
	struct hmc5883l_data	*data;

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

static ssize_t hmc5883l_show_dbgdump(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	unsigned long		dbgdump;
	struct hmc5883l_data	*data;

	data = dev_get_drvdata(dev);

	mutex_lock(&data->mutex);
	dbgdump = data->dbgdump;
	mutex_unlock(&data->mutex);

	return scnprintf(buf, PAGE_SIZE, "%lu\n", dbgdump);
}

static ssize_t hmc5883l_store_dbgdump(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t cnt)
{
	int			error;
	unsigned long		dbgdump;
	struct hmc5883l_data	*data;

	data = dev_get_drvdata(dev);

	error = strict_strtoul(buf, 0, &dbgdump);
	if (error >= 0) {
		mutex_lock(&data->mutex);
		data->dbgdump = dbgdump;
		mutex_unlock(&data->mutex);
	}

	return error < 0 ? error : cnt;
}

static ssize_t hmc5883l_show_period(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	unsigned long		period;
	struct hmc5883l_data	*data;

	data = dev_get_drvdata(dev);

	mutex_lock(&data->mutex);
	period = data->period;
	mutex_unlock(&data->mutex);

	return scnprintf(buf, PAGE_SIZE, "%lu\n", period);
}

static ssize_t hmc5883l_store_period(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t cnt)
{
	int			error;
	unsigned long		period;
	struct hmc5883l_data	*data;

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

static ssize_t hmc5883l_show_scale(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	unsigned long		scale;
	struct hmc5883l_data	*data;

	data = dev_get_drvdata(dev);

	mutex_lock(&data->mutex);
	scale = data->scale;
	mutex_unlock(&data->mutex);

	return scnprintf(buf, PAGE_SIZE, "%lu\n", scale);
}

static ssize_t hmc5883l_store_scale(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t cnt)
{
	int			error;
	unsigned long		scale;
	struct hmc5883l_data	*data;

	data = dev_get_drvdata(dev);

	error = strict_strtoul(buf, 0, &scale);
	if (error >= 0) {
		mutex_lock(&data->mutex);
		data->skipcount  = HMC5883L_SKIPCOUNT;
		data->scale      = adjust_scale(scale);
		data->auto_scale = (scale == 0);
		error = update_device(data);
		mutex_unlock(&data->mutex);
	}

	return error < 0 ? error : cnt;
}

static ssize_t hmc5883l_show_selftest(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	unsigned long		test_mode;
	struct hmc5883l_data	*data;

	data = dev_get_drvdata(dev);

	mutex_lock(&data->mutex);
	test_mode = data->test_mode;
	mutex_unlock(&data->mutex);

	return scnprintf(buf, PAGE_SIZE, "%lu\n", test_mode);
}

static ssize_t hmc5883l_store_selftest(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t cnt)
{
	int			error;
	unsigned long		test_mode;
	struct hmc5883l_data	*data;

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
		hmc5883l_show_enable, hmc5883l_store_enable);
static DEVICE_ATTR(dbgdump, S_IWUSR | S_IRUGO,
		hmc5883l_show_dbgdump, hmc5883l_store_dbgdump);
static DEVICE_ATTR(poll_delay, S_IWUSR | S_IRUGO,
		hmc5883l_show_period, hmc5883l_store_period);
static DEVICE_ATTR(scale, S_IWUSR | S_IRUGO,
		hmc5883l_show_scale, hmc5883l_store_scale);
static DEVICE_ATTR(selftest, S_IWUSR | S_IRUGO,
		hmc5883l_show_selftest, hmc5883l_store_selftest);

static struct attribute *hmc5883l_attrs[] = {
	&dev_attr_enable.attr    ,
	&dev_attr_dbgdump.attr   ,
	&dev_attr_poll_delay.attr,
	&dev_attr_scale.attr     ,
	&dev_attr_selftest.attr  ,
	NULL
};

static struct attribute_group hmc5883l_attr_grp = {
	.attrs = hmc5883l_attrs,
};

static const struct attribute_group *hmc5883l_attr_grps[] = {
	&hmc5883l_attr_grp,
	NULL
};

/* input device driver interface */
static int hmc5883l_open(struct input_dev *dev)
{
	int			error = 0;
	struct hmc5883l_data	*data = input_get_drvdata(dev);

	mutex_lock(&data->mutex);
	data->opened = true;
	error = update_device(data);
	mutex_unlock(&data->mutex);

	return error;
}

static void hmc5883l_close(struct input_dev *dev)
{
	struct hmc5883l_data *data = input_get_drvdata(dev);

	mutex_lock(&data->mutex);
	data->opened = false;
	update_device(data);
	mutex_unlock(&data->mutex);
}

/* work and interrupt handler */
static void hmc5883l_oneshot(struct work_struct *work)
{
	int			error;
	struct hmc5883l_data	*data;

	data = container_of(to_delayed_work(work), struct hmc5883l_data, work);
	mutex_lock(&data->mutex);

	if (device_active(data)) {
		error = i2c_smbus_write_byte_data(data->client,
				HMC5883L_REG_MODE, HMC5883L_MODE_SINGLE);
		if (error < 0) {
			dev_err(&data->client->dev,
				"fail to start HMC5883L.");
		}

		queue_delayed_work(data->wq, &data->work,
				nsecs_to_jiffies(data->period));
	}

	mutex_unlock(&data->mutex);
}

static irqreturn_t hmc5883l_interrupt(int irq, void *dev)
{
	s16				x, y, z;
	int				xmga, ymga, zmga;
	bool				overflow;
	struct hmc5883l_data		*data;
	struct hmc5883l_platform_data	*pdata;

	data = dev;
	pdata = data->client->dev.platform_data;
	mutex_lock(&data->mutex);

	if (read_output(data, &x, &y, &z) < 0)
		goto exit;

	if (data->skipcount != 0) {
		data->skipcount--;
		if (data->dbgdump >= 1)
			dev_info(&data->client->dev, "skip process output.");
		goto exit;
	}

	xmga = scale_output(x, data->scale);
	ymga = scale_output(y, data->scale);
	zmga = scale_output(z, data->scale);

	overflow = output_overflow(x, y, z);
	if (!overflow) {
		if (device_active(data)) {
			input_report_abs(data->input,
				pdata->xcode, pdata->xdir*xmga);
			input_report_abs(data->input,
				pdata->ycode, pdata->ydir*ymga);
			input_report_abs(data->input,
				pdata->zcode, pdata->zdir*zmga);
			input_always_sync(data->input);
		}
	} else if (data->dbgdump >= 1)
		dev_info(&data->client->dev, "output overflow.");

	update_scale(data, overflow, xmga, ymga, zmga);

	if (data->dbgdump >= 2)
		dev_info(&data->client->dev, "x=%05hd(%05dmga),"
			" y=%05hd(%05dmga), z=%05hd(%05dmga).",
			 x, xmga, y, ymga, z, zmga);

exit:
	mutex_unlock(&data->mutex);
	return IRQ_HANDLED;
}

/* i2c client driver interface */
static int hmc5883l_setup(struct hmc5883l_data *data)
{
	struct hmc5883l_platform_data *pdata = data->client->dev.platform_data;
	if (pdata->setup)
		return pdata->setup(data->client, pdata);
	else
		return 0;
}

static int hmc5883l_teardown(struct hmc5883l_data *data)
{
	struct hmc5883l_platform_data *pdata = data->client->dev.platform_data;
	if (pdata->teardown)
		return pdata->teardown(data->client, pdata);
	else
		return 0;
}

#if defined(CONFIG_PM) || defined(CONFIG_HAS_EARLYSUSPEND)
static int hmc5883l_suspend(struct device *dev)
{
	int			error = 0;
	struct hmc5883l_data	*data = dev_get_drvdata(dev);

	mutex_lock(&data->mutex);
	data->suspended = true;
	error = update_device(data);
	mutex_unlock(&data->mutex);

	return error;
}

static int hmc5883l_resume(struct device *dev)
{
	int			error = 0;
	struct hmc5883l_data	*data = dev_get_drvdata(dev);

	mutex_lock(&data->mutex);
	data->suspended = false;
	error = update_device(data);
	mutex_unlock(&data->mutex);

	return error;
}
#endif

#ifdef CONFIG_PM
static const struct dev_pm_ops hmc5883l_pm_ops = {
#ifndef CONFIG_HAS_EARLYSUSPEND
	.suspend	= hmc5883l_suspend,
	.resume		= hmc5883l_resume,
#endif
};
#endif

#ifdef CONFIG_HAS_EARLYSUSPEND
static void hmc5883l_early_suspend(struct early_suspend *h)
{
	struct hmc5883l_data *data = container_of(h,
			struct hmc5883l_data, early_suspend);
	hmc5883l_suspend(&data->client->dev);
}

static void hmc5883l_early_resume(struct early_suspend *h)
{
	struct hmc5883l_data *data = container_of(h,
			struct hmc5883l_data, early_suspend);
	hmc5883l_resume(&data->client->dev);
}
#endif

static int hmc5883l_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	s32				chip;
	int				error;
	struct hmc5883l_data		*data;
	struct hmc5883l_platform_data	*pdata;

	pdata = client->dev.platform_data;
	if (pdata == NULL) {
		dev_err(&client->dev, "invalid platform data.");
		error = -EINVAL;
		goto exit;
	}

	/* allocate device status */
	data = kzalloc(sizeof(*data), GFP_KERNEL);
	if (data == NULL) {
		dev_err(&client->dev, "fail to allocate hmc5883l_data.");
		error = -ENOMEM;
		goto exit;
	}
	i2c_set_clientdata(client, data);

	INIT_DELAYED_WORK(&data->work, hmc5883l_oneshot);
	mutex_init(&data->mutex);

	data->client		= client;
	data->period		= adjust_period(pdata->period);
	data->scale		= adjust_scale(pdata->scale);
	data->auto_scale	= (pdata->scale == 0);

	/* run platform setup */
	error = hmc5883l_setup(data);
	if (error < 0) {
		dev_err(&client->dev, "fail to perform platform setup.");
		goto exit_free_data;
	}

	/* verify device */
	if (i2c_check_functionality(client->adapter, HMC5883L_FUNC) == 0) {
		dev_err(&client->dev, "incompatible adapter.");
		error = -ENODEV;
		goto exit_teardown;
	}

	chip = i2c_smbus_read_byte_data(client, HMC5883L_REG_IDA);
	if (chip != HMC5883L_CHIP_IDA) {
		dev_err(&client->dev, "fail to detect HMC5883L chip.");
		error = -ENODEV;
		goto exit_teardown;
	}

	/* create dedicated work queue */
	data->wq = create_singlethread_workqueue("hmc5883l");
	if (data->wq == NULL) {
		dev_err(&client->dev, "fail to create work queue.");
		error = -ENOMEM;
		goto exit_teardown;
	}

	/* register to input system */
	data->input = input_allocate_device();
	if (data->input == NULL) {
		dev_err(&client->dev, "fail to allocate input device.");
		error = -ENOMEM;
		goto exit_destroy_workqueue;
	}
	input_set_drvdata(data->input, data);

	data->input->name	= "compass";
	data->input->open	= hmc5883l_open;
	data->input->close	= hmc5883l_close;
	data->input->dev.groups	= hmc5883l_attr_grps;

	input_set_capability(data->input, EV_ABS, pdata->xcode);
	input_set_abs_params(data->input, pdata->xcode,
			HMC5883L_MIN, HMC5883L_MAX, HMC5883L_RES, 0);

	input_set_capability(data->input, EV_ABS, pdata->ycode);
	input_set_abs_params(data->input, pdata->ycode,
			HMC5883L_MIN, HMC5883L_MAX, HMC5883L_RES, 0);

	input_set_capability(data->input, EV_ABS, pdata->zcode);
	input_set_abs_params(data->input, pdata->zcode,
			HMC5883L_MIN, HMC5883L_MAX, HMC5883L_RES, 0);

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
	error = request_threaded_irq(client->irq, NULL, hmc5883l_interrupt,
			IRQF_TRIGGER_RISING, "hmc5883l", data);
	if (error < 0) {
		dev_err(&client->dev, "fail to request irq.");
		goto exit_free_phys;
	}

#ifdef CONFIG_HAS_EARLYSUSPEND
	data->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	data->early_suspend.suspend = hmc5883l_early_suspend;
	data->early_suspend.resume = hmc5883l_early_resume;
	register_early_suspend(&data->early_suspend);
#endif

	goto exit; /* all is fine */

exit_free_phys:
	kfree(data->input->phys);
exit_unregister_device:
	input_unregister_device(data->input);
	goto exit_destroy_workqueue;
exit_free_device:
	input_free_device(data->input);
exit_destroy_workqueue:
	destroy_workqueue(data->wq);
exit_teardown:
	hmc5883l_teardown(data);
exit_free_data:
	kfree(data);
exit:
	return error;
}

static int hmc5883l_remove(struct i2c_client *client)
{
	struct hmc5883l_data *data;
	data = i2c_get_clientdata(client);

#ifdef CONFIG_HAS_EARLYSUSPEND
	unregister_early_suspend(&data->early_suspend);
#endif

	cancel_delayed_work(&data->work);
	destroy_workqueue(data->wq);

	free_irq(client->irq, data);

	kfree(data->input->phys);
	input_unregister_device(data->input);

	hmc5883l_teardown(data);
	kfree(data);

	return 0;
}

static const struct i2c_device_id hmc5883l_ids[] = {
	{"hmc5883l", 0 },
	{/* list end */},
};

MODULE_DEVICE_TABLE(i2c, hmc5883l_ids);

static struct i2c_driver hmc5883l_driver = {
	.probe		= hmc5883l_probe,
	.remove		= hmc5883l_remove,
	.driver = {
		.name	= "hmc5883l",
#ifdef CONFIG_PM
		.pm	= &hmc5883l_pm_ops,
#endif
	},
	.id_table	= hmc5883l_ids,
};

/* module initialization and termination */
static int __init hmc5883l_init(void)
{
	return i2c_add_driver(&hmc5883l_driver);
}

static void __exit hmc5883l_exit(void)
{
	i2c_del_driver(&hmc5883l_driver);
}

module_init(hmc5883l_init);
module_exit(hmc5883l_exit);

MODULE_AUTHOR("Xiang Xiao <xiaoxiang@xiaomi.com>");
MODULE_DESCRIPTION("HMC5883L compass input driver");
MODULE_LICENSE("GPL");
