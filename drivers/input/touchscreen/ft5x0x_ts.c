/*
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

#include <linux/delay.h>
#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif
#include <linux/firmware.h>
#include <linux/gpio.h>
#include <linux/init.h>
#include <linux/input.h>
#ifdef CONFIG_TOUCHSCREEN_FT5X0X_TYPEB
#include <linux/input/mt.h>
#endif
#include <linux/input/ft5x0x_ts.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/power_supply.h>
#include "ft5x0x_ts.h"

/* ft5x0x register list */
#define FT5X0X_DEVICE_MODE		0x00
#define FT5X0X_TD_STATUS		0x02

#define FT5X0X_TOUCH_START		0x03
#define FT5X0X_TOUCH_LENGTH		6

#define FT5X0X_TOUCH_XH			0x00 /* offset from each touch */
#define FT5X0X_TOUCH_XL			0x01
#define FT5X0X_TOUCH_YH			0x02
#define FT5X0X_TOUCH_YL			0x03
#define FT5X0X_TOUCH_PRESSURE		0x04
#define FT5X0X_TOUCH_SIZE		0x05

#define FT5X0X_ID_G_PMODE		0xa5
#define FT5X0X_ID_G_FIRMID		0xa6
#define FT5X0X_ID_G_VENDORID		0xa8

#define FT5X0X_NOISE_FILTER		0xb5

#define FT5X0X_RESET			0xfc

/* ft5x0x bit field definition */
#define FT5X0X_MODE_NORMAL		0x00
#define FT5X0X_MODE_SYSINFO		0x10
#define FT5X0X_MODE_TEST		0x40
#define FT5X0X_MODE_MASK		0x70

#define FT5X0X_EVENT_DOWN		0x00
#define FT5X0X_EVENT_UP			0x40
#define FT5X0X_EVENT_CONTACT		0x80
#define FT5X0X_EVENT_MASK		0xc0

#define FT5X0X_POWER_ACTIVE		0x00
#define FT5X0X_POWER_MONITOR		0x01
#define FT5X0X_POWER_HIBERNATE		0x03

/* ft5x0x firmware upgrade definition */
#define FT5X0X_FIRMWARE_TAIL		-8 /* base on the end of firmware */
#define FT5X0X_FIRMWARE_VERION		-2
#define FT5X0X_PACKET_HEADER		6
#define FT5X0X_PACKET_LENGTH		128

/* ft5x0x absolute value */
#define FT5X0X_MAX_FINGER		0x10
#define FT5X0X_MAX_SIZE			0xff
#define FT5X0X_MAX_PRESSURE		0xff

struct ft5x0x_packet {
	u8  magic1;
	u8  magic2;
	u16 offset;
	u16 length;
	u8  payload[FT5X0X_PACKET_LENGTH];
};

struct ft5x0x_finger {
	int x, y;
	int size;
	int pressure;
	bool detect;
};

struct ft5x0x_tracker {
	int x, y;
	bool detect;
	bool moving;
	unsigned long jiffies;
};

struct ft5x0x_data {
	struct mutex mutex;
	struct device *dev;
	struct input_dev *input;
	struct kobject *vkeys_dir;
	struct kobj_attribute vkeys_attr;
	const struct ft5x0x_bus_ops *bops;
#ifdef CONFIG_HAS_EARLYSUSPEND
	struct early_suspend early_suspend;
#endif
	struct ft5x0x_tracker tracker[FT5X0X_MAX_FINGER];
	int  irq;
	bool dbgdump;
};

static int ft5x0x_recv_byte(struct ft5x0x_data *ft5x0x, u8 len, ...)
{
	int error;
	va_list varg;
	u8 i, buf[len];

	error = ft5x0x->bops->recv(ft5x0x->dev, buf, len);
	if (error)
		return error;

	va_start(varg, len);
	for (i = 0; i < len; i++)
		*va_arg(varg, u8 *) = buf[i];
	va_end(varg);

	return 0;
}

static int ft5x0x_send_block(struct ft5x0x_data *ft5x0x,
				const void *buf, int len)
{
	return ft5x0x->bops->send(ft5x0x->dev, buf, len);
}

static int ft5x0x_send_byte(struct ft5x0x_data *ft5x0x, u8 len, ...)
{
	va_list varg;
	u8 i, buf[len];

	va_start(varg, len);
	for (i = 0; i < len; i++)
		buf[i] = va_arg(varg, int); /* u8 promote to int */
	va_end(varg);

	return ft5x0x_send_block(ft5x0x, buf, len);
}

static int ft5x0x_read_block(struct ft5x0x_data *ft5x0x,
				u8 addr, void *buf, u8 len)
{
	return ft5x0x->bops->read(ft5x0x->dev, addr, buf, len);
}

static int ft5x0x_read_byte(struct ft5x0x_data *ft5x0x, u8 addr, u8 *data)
{
	return ft5x0x_read_block(ft5x0x, addr, data, sizeof(*data));
}

static int ft5x0x_write_byte(struct ft5x0x_data *ft5x0x, u8 addr, u8 data)
{
	return ft5x0x->bops->write(ft5x0x->dev, addr, &data, sizeof(data));
}

static int reset_delay[] = {
	30, 33, 36, 39, 42, 45, 27, 24, 21, 18, 15
};

static int ft5x0x_load_firmware(struct ft5x0x_data *ft5x0x,
		const struct ft5x0x_firmware_data *firmware, bool *upgraded)
{
	struct ft5x0x_packet packet;
	int i, j, length, error = 0;
	u8 val1, val2, id, ecc = 0;

	/* step 0a: check and init argument */
	if (upgraded)
		*upgraded = false;

	if (firmware == NULL)
		return 0;

	/* step 0b: find the right firmware for touch screen */
	error = ft5x0x_read_byte(ft5x0x, FT5X0X_ID_G_VENDORID, &id);
	if (error)
		return error;
	dev_info(ft5x0x->dev, "firmware vendor is %02x\n", id);

	for (; firmware->size != 0; firmware++) {
		if (id == firmware->vendor)
			break;
	}

	if (firmware->size == 0) {
		dev_err(ft5x0x->dev, "unknown touch screen vendor\n");
		return -ENOENT;
	}

	/* step 1: check firmware id is different */
	error = ft5x0x_read_byte(ft5x0x, FT5X0X_ID_G_FIRMID, &id);
	if (error)
		return error;
	dev_info(ft5x0x->dev, "firmware version is %02x\n", id);

	if (id == firmware->data[firmware->size+FT5X0X_FIRMWARE_VERION])
		return 0;
	dev_info(ft5x0x->dev, "upgrade firmware to %02x\n",
		firmware->data[firmware->size+FT5X0X_FIRMWARE_VERION]);

	for (i = 0, error = -1; i < ARRAY_SIZE(reset_delay) && error; i++) {
		/* step 2: reset device */
		error = ft5x0x_write_byte(ft5x0x, FT5X0X_RESET, 0xaa);
		if (error)
			continue;
		msleep(50);

		error = ft5x0x_write_byte(ft5x0x, FT5X0X_RESET, 0x55);
		if (error)
			continue;
		msleep(reset_delay[i]);

		/* step 3: enter upgrade mode */
		for (i = 0; i < 10; i++) {
			error = ft5x0x_send_byte(ft5x0x, 2, 0x55, 0xaa);
			msleep(5);
			if (!error)
				break;
		}
		if (error)
			continue;

		/* step 4: check device id */
		error = ft5x0x_send_byte(ft5x0x, 4, 0x90, 0x00, 0x00, 0x00);
		if (error)
			continue;

		error = ft5x0x_recv_byte(ft5x0x, 2, &val1, &val2);
		if (error)
			continue;

		if (val1 != 0x79 || val2 != 0x03)
			error = -ENODEV;
	}

	if (error) /* check the final result */
		return error;

	/* step 5: erase device */
	error = ft5x0x_send_byte(ft5x0x, 1, i > 0 ? 0x61 : 0x60);
	if (error)
		return error;
	msleep(1500);

	/* step 6: flash firmware to device */
	packet.magic1 = 0xbf;
	packet.magic2 = 0x00;

	/* step 6a: send data in 128 bytes chunk each time */
	for (i = 0; i < firmware->size+FT5X0X_FIRMWARE_TAIL; i += length) {
		length = min(FT5X0X_PACKET_LENGTH,
				firmware->size+FT5X0X_FIRMWARE_TAIL-i);

		packet.offset = cpu_to_be16(i);
		packet.length = cpu_to_be16(length);

		for (j = 0; j < length; j++) {
			packet.payload[j] = firmware->data[i+j];
			ecc ^= firmware->data[i+j];
		}

		error = ft5x0x_send_block(ft5x0x, &packet,
					FT5X0X_PACKET_HEADER+length);
		if (error)
			return error;

		msleep(FT5X0X_PACKET_LENGTH/6);
	}

	/* step 6b: send one byte each time for last six bytes */
	for (j = 0; i < firmware->size+FT5X0X_FIRMWARE_VERION; i++, j++) {
		packet.offset = cpu_to_be16(0x6ffa+j);
		packet.length = cpu_to_be16(1);

		packet.payload[0] = firmware->data[i];
		ecc ^= firmware->data[i];

		error = ft5x0x_send_block(ft5x0x, &packet,
					FT5X0X_PACKET_HEADER+1);
		if (error)
			return error;

		msleep(20);
	}

	/* step 7: verify checksum */
	error = ft5x0x_send_byte(ft5x0x, 1, 0xcc);
	if (error)
		return error;

	error = ft5x0x_recv_byte(ft5x0x, 1, &val1);
	if (error)
		return error;

	if (val1 != ecc)
		return -ERANGE;

	/* step 8: reset to new firmware */
	error = ft5x0x_send_byte(ft5x0x, 1, 0x07);
	if (error)
		return error;
	msleep(300);

#ifdef CONFIG_TOUCHSCREEN_FT5X0X_CALIBRATE
	/* step 9: calibrate the reference value */
	error = ft5x0x_write_byte(ft5x0x, /* enter factory mode */
				FT5X0X_DEVICE_MODE, 0x40);
	if (error)
		return error;
	msleep(100);

	error = ft5x0x_write_byte(ft5x0x, /* start calibration */
				FT5X0X_TD_STATUS, 0x04);
	if (error)
		return error;
	msleep(300);

	for (i = 0; i < 100; i++) {
		error = ft5x0x_read_byte(ft5x0x, FT5X0X_DEVICE_MODE, &val1);
		if (error)
			return error;
		if ((val1&0x70) == 0) /* return to normal mode? */
			break;
		msleep(200); /* not yet, wait and try again later */
	}

	msleep(300); /* enter factory mode again */
	error = ft5x0x_write_byte(ft5x0x, FT5X0X_DEVICE_MODE, 0x40);
	if (error)
		return error;
	msleep(100);

	error = ft5x0x_write_byte(ft5x0x, /* save calibration result */
				FT5X0X_TD_STATUS, 0x05);
	if (error)
		return error;
	msleep(300);

	error = ft5x0x_write_byte(ft5x0x, /* return to normal mode */
				FT5X0X_DEVICE_MODE, 0x00);
	if (error)
		return error;
	msleep(300);
#endif

	if (upgraded)
		*upgraded = true;

	return 0;
}

static int ft5x0x_collect_finger(struct ft5x0x_data *ft5x0x,
				struct ft5x0x_finger *finger, int count)
{
	u8 number, buf[256];
	int i, error;

	error = ft5x0x_read_byte(ft5x0x, FT5X0X_TD_STATUS, &number);
	if (error)
		return error;
	number &= 0x0f;

	error = ft5x0x_read_block(ft5x0x, FT5X0X_TOUCH_START,
				buf, FT5X0X_TOUCH_LENGTH*number);
	if (error)
		return error;

	/* clear the finger buffer */
	memset(finger, 0, sizeof(*finger)*count);

	for (i = 0; i < number; i++) {
		u8 xh = buf[FT5X0X_TOUCH_LENGTH*i+FT5X0X_TOUCH_XH];
		u8 xl = buf[FT5X0X_TOUCH_LENGTH*i+FT5X0X_TOUCH_XL];
		u8 yh = buf[FT5X0X_TOUCH_LENGTH*i+FT5X0X_TOUCH_YH];
		u8 yl = buf[FT5X0X_TOUCH_LENGTH*i+FT5X0X_TOUCH_YL];

		u8 size     = buf[FT5X0X_TOUCH_LENGTH*i+FT5X0X_TOUCH_SIZE];
		u8 pressure = buf[FT5X0X_TOUCH_LENGTH*i+FT5X0X_TOUCH_PRESSURE];

		u8 id = (yh&0xf0)>>4;

		finger[id].x        = ((xh&0x0f)<<8)|xl;
		finger[id].y        = ((yh&0x0f)<<8)|yl;
		finger[id].size     = size;
		finger[id].pressure = pressure;
		finger[id].detect   = (xh&FT5X0X_EVENT_MASK) != FT5X0X_EVENT_UP;

		if (ft5x0x->dbgdump)
			dev_info(ft5x0x->dev,
				"fig(%02u): %d %04d %04d %03d %03d\n", id,
				finger[i].detect, finger[i].x, finger[i].y,
				finger[i].pressure, finger[i].size);
	}

	return 0;
}

static void ft5x0x_apply_filter(struct ft5x0x_data *ft5x0x,
				struct ft5x0x_finger *finger, int count)
{
	struct ft5x0x_platform_data *pdata = ft5x0x->dev->platform_data;
	int i;

	for (i = 0; i < count; i++) {
		if (!finger[i].detect) /* finger release */
			ft5x0x->tracker[i].detect = false;
		else if (!ft5x0x->tracker[i].detect) { /* initial touch */
			ft5x0x->tracker[i].x = finger[i].x;
			ft5x0x->tracker[i].y = finger[i].y;
			ft5x0x->tracker[i].detect  = true;
			ft5x0x->tracker[i].moving  = false;
			ft5x0x->tracker[i].jiffies = jiffies;
		} else { /* the rest report until finger lift */
			unsigned long landed_jiffies;
			int delta_x, delta_y, threshold;

			landed_jiffies  = ft5x0x->tracker[i].jiffies;
			landed_jiffies += pdata->landing_jiffies;

			/* no significant movement yet */
			if (!ft5x0x->tracker[i].moving) {
				/* use the big threshold for landing period */
				if (time_before(jiffies, landed_jiffies))
					threshold = pdata->landing_threshold;
				else /* use the middle jitter threshold */
					threshold = pdata->staying_threshold;
			} else { /* use the small threshold during movement */
				threshold = pdata->moving_threshold;
			}

			delta_x = finger[i].x - ft5x0x->tracker[i].x;
			delta_y = finger[i].y - ft5x0x->tracker[i].y;

			delta_x *= delta_x;
			delta_y *= delta_y;

			/* use the saved value for small change */
			if (delta_x + delta_y <= threshold * threshold)	{
				finger[i].x = ft5x0x->tracker[i].x;
				finger[i].y = ft5x0x->tracker[i].y;
			} else {/* save new location */
				ft5x0x->tracker[i].x = finger[i].x;
				ft5x0x->tracker[i].y = finger[i].y;
				ft5x0x->tracker[i].moving = true;
			}
		}
	}
}

static void ft5x0x_report_touchevent(struct ft5x0x_data *ft5x0x,
				struct ft5x0x_finger *finger, int count)
{
	bool mt_sync_sent = false;
	int i;

	for (i = 0; i < count; i++) {
#ifdef CONFIG_TOUCHSCREEN_FT5X0X_TYPEB
		input_mt_slot(ft5x0x->input, i);
#endif
		if (!finger[i].detect) {
#ifdef CONFIG_TOUCHSCREEN_FT5X0X_TYPEB
			input_mt_report_slot_state(ft5x0x->input,
							MT_TOOL_FINGER, false);
#endif
			continue;
		}
#ifdef CONFIG_TOUCHSCREEN_FT5X0X_TYPEB
		input_mt_report_slot_state(ft5x0x->input, MT_TOOL_FINGER, true);
#endif
		input_report_abs(ft5x0x->input, ABS_MT_TRACKING_ID, i);
		input_report_abs(ft5x0x->input, ABS_MT_POSITION_X ,
			max(1, finger[i].x)); /* for fruit ninja */
		input_report_abs(ft5x0x->input, ABS_MT_POSITION_Y ,
			max(1, finger[i].y)); /* for fruit ninja */
		input_report_abs(ft5x0x->input, ABS_MT_PRESSURE,
			max(1, finger[i].pressure));
		input_report_abs(ft5x0x->input, ABS_MT_WIDTH_MAJOR,
			max(1, finger[i].size));
#ifndef CONFIG_TOUCHSCREEN_FT5X0X_TYPEB
		input_mt_sync(ft5x0x->input);
#endif
		mt_sync_sent = true;

		if (ft5x0x->dbgdump)
			dev_info(ft5x0x->dev,
				"tch(%02d): %04d %04d %03d %03d\n",
				i, finger[i].x, finger[i].y,
				finger[i].pressure, finger[i].size);
	}

	if (!mt_sync_sent) {
#ifndef CONFIG_TOUCHSCREEN_FT5X0X_TYPEB
		input_mt_sync(ft5x0x->input);
#endif
		if (ft5x0x->dbgdump)
			dev_info(ft5x0x->dev, "tch(xx): no touch contact\n");
	}

	input_sync(ft5x0x->input);
}

static irqreturn_t ft5x0x_interrupt(int irq, void *dev_id)
{
	struct ft5x0x_finger finger[FT5X0X_MAX_FINGER];
	struct ft5x0x_data *ft5x0x = dev_id;
	int error;

	mutex_lock(&ft5x0x->mutex);
	error = ft5x0x_collect_finger(ft5x0x, finger, FT5X0X_MAX_FINGER);
	if (error >= 0) {
		ft5x0x_apply_filter(ft5x0x, finger, FT5X0X_MAX_FINGER);
		ft5x0x_report_touchevent(ft5x0x, finger, FT5X0X_MAX_FINGER);
	} else
		dev_err(ft5x0x->dev, "fail to collect finger(%d)\n", error);
	mutex_unlock(&ft5x0x->mutex);

	return IRQ_HANDLED;
}

int ft5x0x_suspend(struct ft5x0x_data *ft5x0x)
{
	int error;

	mutex_lock(&ft5x0x->mutex);
	memset(ft5x0x->tracker, 0, sizeof(ft5x0x->tracker));
	error = ft5x0x_write_byte(ft5x0x,
			FT5X0X_ID_G_PMODE, FT5X0X_POWER_HIBERNATE);
	mutex_unlock(&ft5x0x->mutex);

	return error;
}
EXPORT_SYMBOL_GPL(ft5x0x_suspend);

int ft5x0x_resume(struct ft5x0x_data *ft5x0x)
{
	struct ft5x0x_platform_data *pdata = ft5x0x->dev->platform_data;
	int error;

	mutex_lock(&ft5x0x->mutex);
	/* reset device */
	gpio_set_value_cansleep(pdata->resetpin, 0);
	msleep(1);
	gpio_set_value_cansleep(pdata->resetpin, 1);
	msleep(50);

	/* enable noise filter when the charger plug in */
	error = ft5x0x_write_byte(ft5x0x, FT5X0X_NOISE_FILTER,
				power_supply_is_system_supplied());
	mutex_unlock(&ft5x0x->mutex);

	return error;
}
EXPORT_SYMBOL_GPL(ft5x0x_resume);

#ifdef CONFIG_HAS_EARLYSUSPEND
static void ft5x0x_early_suspend(struct early_suspend *h)
{
	struct ft5x0x_data *ft5x0x = container_of(h,
					struct ft5x0x_data, early_suspend);
	ft5x0x_suspend(ft5x0x);
}

static void ft5x0x_early_resume(struct early_suspend *h)
{
	struct ft5x0x_data *ft5x0x = container_of(h,
					struct ft5x0x_data, early_suspend);
	ft5x0x_resume(ft5x0x);
}
#endif

static ssize_t ft5x0x_vkeys_show(struct kobject *kobj,
				struct kobj_attribute *attr, char *buf)
{
	struct ft5x0x_data *ft5x0x =
		container_of(attr, struct ft5x0x_data, vkeys_attr);
	struct ft5x0x_platform_data *pdata = ft5x0x->dev->platform_data;
	const struct ft5x0x_keypad_data *keypad = pdata->keypad;
	int i, count = 0;

	for (i = 0; keypad && i < keypad->length; i++) {
		int width  = keypad->button[i].width;
		int height = keypad->button[i].height;
		int midx   = keypad->button[i].left+width/2;
		int midy   = keypad->button[i].top+height/2;

		count += snprintf(buf+count, PAGE_SIZE-count,
				"0x%02x:%d:%d:%d:%d:%d:",
				EV_KEY, keypad->keymap[i],
				midx, midy, width, height);
	}

	count -= 1; /* remove the last colon */
	count += snprintf(buf+count, PAGE_SIZE-count, "\n");
	return count;
}

static ssize_t ft5x0x_object_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	static struct {
		u8 addr;
		const char *fmt;
	} reg_list[] = {
		/* threshold setting */
		{0x80, "THGROUP          %3d\n"  },
		{0x81, "THPEAK           %3d\n"  },
		{0x82, "THCAL            %3d\n"  },
		{0x83, "THWATER          %3d\n"  },
		{0x84, "THTEMP           %3d\n"  },
		{0x85, "THDIFF           %3d\n"  },
		{0xae, "THBAREA          %3d\n"  },
		/* mode setting */
		{0x86, "CTRL              %02x\n"},
		{0xa0, "AUTOCLB           %02x\n"},
		{0xa4, "MODE              %02x\n"},
		{0xa5, "PMODE             %02x\n"},
		{0xa7, "STATE             %02x\n"},
		{0xa9, "ERR               %02x\n"},
		/* timer setting */
		{0x87, "TIME2MONITOR     %3d\n"  },
		{0x88, "PERIODACTIVE     %3d\n"  },
		{0x89, "PERIODMONITOR    %3d\n"  },
		/* version info */
		{0xa1, "LIBVERH           %02x\n"},
		{0xa2, "LIBVERL           %02x\n"},
		{0xa3, "CIPHER            %02x\n"},
		{0xa6, "FIRMID            %02x\n"},
		{0xa8, "FT5201ID          %02x\n"},
		{/* end of the list */},
	};

	struct ft5x0x_data *ft5x0x = dev_get_drvdata(dev);
	int i, error, count = 0;
	u8 val;

	mutex_lock(&ft5x0x->mutex);
	for (i = 0; reg_list[i].addr != 0; i++) {
		error = ft5x0x_read_byte(ft5x0x, reg_list[i].addr, &val);
		if (error)
			break;

		count += snprintf(buf+count, PAGE_SIZE-count,
				reg_list[i].fmt, val);
	}
	mutex_unlock(&ft5x0x->mutex);

	return error ? : count;
}

static ssize_t ft5x0x_object_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct ft5x0x_data *ft5x0x = dev_get_drvdata(dev);
	u8 addr, val;
	int error;

	mutex_lock(&ft5x0x->mutex);
	if (sscanf(buf, "%hhx=%hhx", &addr, &val) == 2)
		error = ft5x0x_write_byte(ft5x0x, addr, val);
	else
		error = -EINVAL;
	mutex_unlock(&ft5x0x->mutex);

	return error ? : count;
}

static ssize_t ft5x0x_dbgdump_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct ft5x0x_data *ft5x0x = dev_get_drvdata(dev);
	int count;

	mutex_lock(&ft5x0x->mutex);
	count = sprintf(buf, "%d\n", ft5x0x->dbgdump);
	mutex_unlock(&ft5x0x->mutex);

	return count;
}

static ssize_t ft5x0x_dbgdump_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct ft5x0x_data *ft5x0x = dev_get_drvdata(dev);
	unsigned long dbgdump;
	int error;

	mutex_lock(&ft5x0x->mutex);
	error = strict_strtoul(buf, 0, &dbgdump);
	if (!error)
		ft5x0x->dbgdump = dbgdump;
	mutex_unlock(&ft5x0x->mutex);

	return error ? : count;
}

static ssize_t ft5x0x_updatefw_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct ft5x0x_data *ft5x0x = dev_get_drvdata(dev);
	struct ft5x0x_firmware_data firmware;
	const struct firmware *fw;
	bool upgraded;
	int error;

	error = request_firmware(&fw, "ft5x0x.bin", dev);
	if (!error) {
		firmware.data = fw->data;
		firmware.size = fw->size;

		mutex_lock(&ft5x0x->mutex);
		error = ft5x0x_load_firmware(ft5x0x, &firmware, &upgraded);
		mutex_unlock(&ft5x0x->mutex);

		release_firmware(fw);
	}

	return error ? : count;
}

static DEVICE_ATTR(object, 0644, ft5x0x_object_show, ft5x0x_object_store);
static DEVICE_ATTR(dbgdump, 0644, ft5x0x_dbgdump_show, ft5x0x_dbgdump_store);
static DEVICE_ATTR(updatefw, 0200, NULL, ft5x0x_updatefw_store);

static struct attribute *ft5x0x_attrs[] = {
	&dev_attr_object.attr,
	&dev_attr_dbgdump.attr,
	&dev_attr_updatefw.attr,
	NULL
};

static const struct attribute_group ft5x0x_attr_group = {
	.attrs = ft5x0x_attrs
};

struct ft5x0x_data *ft5x0x_probe(struct device *dev, int irq,
				const struct ft5x0x_bus_ops *bops)
{
	u8 mode;
	int error;
	struct ft5x0x_data *ft5x0x;
	struct ft5x0x_platform_data *pdata;

	/* check input argument */
	if (irq <= 0) {
		dev_err(dev, "irq isn't configured\n");
		error = -EINVAL;
		goto err;
	}

	pdata = dev->platform_data;
	if (pdata == NULL) {
		dev_err(dev, "platform data doesn't exist\n");
		error = -EINVAL;
		goto err;
	}

	/* init platform stuff */
	if (pdata->setup) {
		error = pdata->setup(pdata);
		if (error) {
			dev_err(dev, "fail to setup platform\n");
			goto err;
		}
	}

	error = gpio_request(pdata->resetpin, "ft5x0x_reset");
	if (error) {
		dev_err(dev, "fail to request rest pin\n");
		goto err_teardown;
	}

	error = gpio_direction_output(pdata->resetpin, 1);
	if (error) {
		dev_err(dev, "fail to change rest pin direction\n");
		goto err_free_resetpin;
	}

	/* alloc and init data object */
	ft5x0x = kzalloc(sizeof(struct ft5x0x_data), GFP_KERNEL);
	if (ft5x0x == NULL) {
		dev_err(dev, "fail to allocate data object\n");
		error = -ENOMEM;
		goto err_free_resetpin;
	}

	mutex_init(&ft5x0x->mutex);

	ft5x0x->dev  = dev;
	ft5x0x->irq  = irq;
	ft5x0x->bops = bops;

	/* alloc and init input device */
	ft5x0x->input = input_allocate_device();
	if (ft5x0x->input == NULL) {
		dev_err(dev, "fail to allocate input device\n");
		error = -ENOMEM;
		goto err_free_data;
	}

	input_set_drvdata(ft5x0x->input, ft5x0x);
	ft5x0x->input->name       = "ft5x0x";
	ft5x0x->input->id.bustype = bops->bustype;
	ft5x0x->input->id.vendor  = 0x4654; /* FocalTech */
	ft5x0x->input->id.product = 0x5000; /* ft5x0x    */
	ft5x0x->input->id.version = 0x0100; /* 1.0       */
	ft5x0x->input->dev.parent = dev;

	/* init touch parameter */
	input_set_capability(ft5x0x->input, EV_ABS, ABS_MT_TRACKING_ID);
	input_set_capability(ft5x0x->input, EV_ABS, ABS_MT_POSITION_X);
	input_set_capability(ft5x0x->input, EV_ABS, ABS_MT_POSITION_Y);
	input_set_capability(ft5x0x->input, EV_ABS, ABS_MT_PRESSURE);
	input_set_capability(ft5x0x->input, EV_ABS, ABS_MT_WIDTH_MAJOR);
	set_bit(INPUT_PROP_DIRECT, ft5x0x->input->propbit);

	input_set_abs_params(ft5x0x->input, ABS_MT_TRACKING_ID,
			0, FT5X0X_MAX_FINGER, 0, 0);
	input_set_abs_params(ft5x0x->input,
			ABS_MT_POSITION_X, pdata->tcharea.left,
			pdata->tcharea.left+pdata->tcharea.width, 0, 0);
	input_set_abs_params(ft5x0x->input,
			ABS_MT_POSITION_Y, pdata->tcharea.top,
			pdata->tcharea.top+pdata->tcharea.height, 0, 0);
	input_set_abs_params(ft5x0x->input, ABS_MT_PRESSURE,
			0, FT5X0X_MAX_PRESSURE, 0, 0);
	input_set_abs_params(ft5x0x->input, ABS_MT_WIDTH_MAJOR,
			0, FT5X0X_MAX_SIZE, 0, 0);
#ifdef CONFIG_TOUCHSCREEN_FT5X0X_TYPEB
	input_mt_init_slots(ft5x0x->input, FT5X0X_MAX_FINGER);
#endif

	input_set_events_per_packet(ft5x0x->input, 64);

	/* init touch device */
	error = ft5x0x_read_byte(ft5x0x, FT5X0X_DEVICE_MODE, &mode);
	if (error) {
		dev_err(dev, "fail to read device mode register\n");
		goto err_free_input;
	}
	if ((mode&FT5X0X_MODE_MASK) != FT5X0X_MODE_NORMAL) {
		dev_err(dev, "device isn't in normal operation mode\n");
		error = -ENODEV;
		goto err_free_input;
	}

	error = ft5x0x_load_firmware(ft5x0x, pdata->firmware, NULL);
	if (error) {
		dev_err(dev, "fail to load firmware\n");
		goto err_free_input;
	}

	/* register input device */
	error = input_register_device(ft5x0x->input);
	if (error) {
		dev_err(dev, "fail to register input device\n");
		goto err_free_input;
	}

	ft5x0x->input->phys =
		kobject_get_path(&ft5x0x->input->dev.kobj, GFP_KERNEL);
	if (ft5x0x->input->phys == NULL) {
		dev_err(dev, "fail to get input device path\n");
		error = -ENOMEM;
		goto err_unregister_input;
	}

	/* start interrupt process */
	error = request_threaded_irq(irq, NULL, ft5x0x_interrupt,
				IRQF_TRIGGER_FALLING, "ft5x0x", ft5x0x);
	if (error) {
		dev_err(dev, "fail to request interrupt\n");
		goto err_free_phys;
	}

	/* export sysfs entries */
	ft5x0x->vkeys_dir = kobject_create_and_add("board_properties", NULL);
	if (ft5x0x->vkeys_dir == NULL) {
		error = -ENOMEM;
		dev_err(dev, "fail to create board_properties entry\n");
		goto err_free_irq;
	}

	sysfs_attr_init(&ft5x0x->vkeys_attr.attr);
	ft5x0x->vkeys_attr.attr.name = "virtualkeys.ft5x0x";
	ft5x0x->vkeys_attr.attr.mode = (S_IRUSR|S_IRGRP|S_IROTH);
	ft5x0x->vkeys_attr.show      = ft5x0x_vkeys_show;

	error = sysfs_create_file(ft5x0x->vkeys_dir, &ft5x0x->vkeys_attr.attr);
	if (error) {
		dev_err(dev, "fail to create virtualkeys entry\n");
		goto err_put_vkeys;
	}

	error = sysfs_create_group(&dev->kobj, &ft5x0x_attr_group);
	if (error) {
		dev_err(dev, "fail to export sysfs entires\n");
		goto err_put_vkeys;
	}

#ifdef CONFIG_HAS_EARLYSUSPEND
	ft5x0x->early_suspend.level   = EARLY_SUSPEND_LEVEL_BLANK_SCREEN+1;
	ft5x0x->early_suspend.suspend = ft5x0x_early_suspend;
	ft5x0x->early_suspend.resume  = ft5x0x_early_resume;
	register_early_suspend(&ft5x0x->early_suspend);
#endif
	return ft5x0x;

err_put_vkeys:
	kobject_put(ft5x0x->vkeys_dir);
err_free_irq:
	free_irq(irq, ft5x0x);
err_free_phys:
	kfree(ft5x0x->input->phys);
err_unregister_input:
	input_unregister_device(ft5x0x->input);
	ft5x0x->input = NULL;
err_free_input:
	input_free_device(ft5x0x->input);
err_free_data:
	kfree(ft5x0x);
err_free_resetpin:
	gpio_free(pdata->resetpin);
err_teardown:
	if (pdata->teardown)
		pdata->teardown(pdata);
err:
	return ERR_PTR(error);
}
EXPORT_SYMBOL_GPL(ft5x0x_probe);

void ft5x0x_remove(struct ft5x0x_data *ft5x0x)
{
	struct ft5x0x_platform_data *pdata = ft5x0x->dev->platform_data;

#ifdef CONFIG_HAS_EARLYSUSPEND
	unregister_early_suspend(&ft5x0x->early_suspend);
#endif
	sysfs_remove_group(&ft5x0x->dev->kobj, &ft5x0x_attr_group);
	kobject_put(ft5x0x->vkeys_dir);
	free_irq(ft5x0x->irq, ft5x0x);
	kfree(ft5x0x->input->phys);
	input_unregister_device(ft5x0x->input);
	kfree(ft5x0x);
	gpio_free(pdata->resetpin);
	if (pdata->teardown)
		pdata->teardown(pdata);
}
EXPORT_SYMBOL_GPL(ft5x0x_remove);

MODULE_AUTHOR("Xiang Xiao <xiaoxiang@xiaomi.com>");
MODULE_DESCRIPTION("ft5x0x touchscreen input driver");
MODULE_LICENSE("GPL");
