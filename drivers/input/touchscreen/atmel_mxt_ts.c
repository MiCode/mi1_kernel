/*
 * Atmel maXTouch Touchscreen driver
 *
 * Copyright (C) 2010 Samsung Electronics Co.Ltd
 * Author: Joonyoung Shim <jy0922.shim@samsung.com>
 *
 * Copyright (C) 2011-2014 Xiaomi Ltd.
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 *
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/firmware.h>
#include <linux/i2c.h>
#include <linux/i2c/atmel_mxt_ts.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/completion.h>
#include <linux/gpio.h>
#include <linux/power_supply.h>
#include <linux/workqueue.h>
#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif

/* Slave addresses */
#define MXT_APP_LOW			0x4a
#define MXT_APP_HIGH			0x4b
#define MXT_BOOT_LOW			0x24
#define MXT_BOOT_HIGH			0x25

/* Firmware */
#define MXT_FW_NAME			"maxtouch.fw"

/* Registers */
#define MXT_FAMILY_ID			0x00
#define MXT_VARIANT_ID			0x01
#define MXT_VERSION			0x02
#define MXT_BUILD			0x03
#define MXT_MATRIX_X_SIZE		0x04
#define MXT_MATRIX_Y_SIZE		0x05
#define MXT_OBJECT_NUM			0x06

/* Object field */
#define MXT_OBJECT_START		0x07
#define MXT_OBJECT_LENGTH		6

#define MXT_OBJECT_TYPE			0
#define MXT_OBJECT_ADDRLSB		1
#define MXT_OBJECT_ADDRMSB		2
#define MXT_OBJECT_SIZE			3
#define MXT_OBJECT_INST			4
#define MXT_OBJECT_IDNUM		5

#define MXT_MAX_BLOCK_WRITE	256

/* Object types */
#define MXT_GEN_MESSAGE			5
#define MXT_GEN_COMMAND			6
#define MXT_GEN_POWER			7
#define MXT_GEN_ACQUIRE			8
#define MXT_TOUCH_MULTI			9
#define MXT_TOUCH_KEYARRAY		15
#define MXT_PROCI_GRIPFACE		20
#define MXT_PROCG_NOISE			22
#define MXT_SPT_SELFTEST		25
#define MXT_SPT_USERDATA		38

/* MXT_GEN_COMMAND field */
#define MXT_COMMAND_RESET		0
#define MXT_COMMAND_BACKUPNV		1
#define MXT_COMMAND_CALIBRATE		2
#define MXT_COMMAND_REPORTALL		3
#define MXT_COMMAND_DIAGNOSTIC		5

/* MXT_GEN_POWER field */
#define MXT_POWER_IDLEACQINT		0
#define MXT_POWER_ACTVACQINT		1
#define MXT_POWER_ACTV2IDLETO		2

/* MXT_GEN_ACQUIRE field */
#define MXT_ACQUIRE_CHRGTIME		0
#define MXT_ACQUIRE_TCHDRIFT		2
#define MXT_ACQUIRE_DRIFTST		3
#define MXT_ACQUIRE_TCHAUTOCAL		4
#define MXT_ACQUIRE_SYNC		5
#define MXT_ACQUIRE_ATCHCALST		6
#define MXT_ACQUIRE_ATCHCALSTHR		7
#define MXT_ACQUIRE_ATCHFRCCALTHR	8
#define MXT_ACQUIRE_ATCHFRCCALRATIO	9

/* MXT_TOUCH_MULTI field */
#define MXT_TOUCH_CTRL			0
#define MXT_TOUCH_XORIGIN		1
#define MXT_TOUCH_YORIGIN		2
#define MXT_TOUCH_XSIZE			3
#define MXT_TOUCH_YSIZE			4
#define MXT_TOUCH_AKSCFG		5
#define MXT_TOUCH_BLEN			6
#define MXT_TOUCH_TCHTHR		7
#define MXT_TOUCH_TCHDI			8
#define MXT_TOUCH_ORIENT		9
#define MXT_TOUCH_MRGTIMEOUT		10
#define MXT_TOUCH_MOVHYSTI		11
#define MXT_TOUCH_MOVHYSTN		12
#define MXT_TOUCH_MOVFILTER		13
#define MXT_TOUCH_NUMTOUCH		14
#define MXT_TOUCH_MRGHYST		15
#define MXT_TOUCH_MRGTHR		16
#define MXT_TOUCH_AMPHYST		17
#define MXT_TOUCH_XRANGE_LSB		18
#define MXT_TOUCH_XRANGE_MSB		19
#define MXT_TOUCH_YRANGE_LSB		20
#define MXT_TOUCH_YRANGE_MSB		21
#define MXT_TOUCH_XLOCLIP		22
#define MXT_TOUCH_XHICLIP		23
#define MXT_TOUCH_YLOCLIP		24
#define MXT_TOUCH_YHICLIP		25
#define MXT_TOUCH_XEDGECTRL		26
#define MXT_TOUCH_XEDGEDIST		27
#define MXT_TOUCH_YEDGECTRL		28
#define MXT_TOUCH_YEDGEDIST		29
#define MXT_TOUCH_JUMPLIMIT		30
#define MXT_TOUCH_TCHHYST		31

/* MXT_TOUCH_KEYARRAY field */
#define MXT_KEYARRAY_CTRL		0
#define MXT_KEYARRAY_XORIGIN		1
#define MXT_KEYARRAY_YORIGIN		2
#define MXT_KEYARRAY_XSIZE		3
#define MXT_KEYARRAY_YSIZE		4
#define MXT_KEYARRAY_AKSCFG		5
#define MXT_KEYARRAY_BLEN		6
#define MXT_KEYARRAY_TCHTHR		7
#define MXT_KEYARRAY_TCHDI		8

/* MXT_PROCG_NOISE field */
#define MXT_NOISE_CTRL			0
#define MXT_NOISE_GCAFUL_LSB		3
#define MXT_NOISE_GCAFUL_MSB		4
#define MXT_NOISE_GCAFLL_LSB		5
#define MXT_NOISE_GCAFLL_MSB		6
#define MXT_NOISE_ACTVGCAFVALID		7
#define MXT_NOISE_NOISETHR		8
#define MXT_NOISE_FREQHOPSCALE		10
#define MXT_NOISE_FREQ0			11
#define MXT_NOISE_FREQ1			12
#define MXT_NOISE_FREQ2			13
#define MXT_NOISE_FREQ3			14
#define MXT_NOISE_FREQ4			15
#define MXT_NOISE_IDLEGCAFVALID		16

/* MXT_SPT_SELFTEST field */
#define MXT_SELFTEST_CTRL		0
#define MXT_SELFTEST_CMD		1
#define MXT_SELFTEST_HISIGLIM		2	/* 2 bytes */
#define MXT_SELFTEST_LOSIGLIM		4	/* 2 bytes */

/* Predefined report ids */
#define MXT_MSG_RESERVED		0x00
#define MXT_MSG_FIRST			0x01
#define MXT_MSG_LAST			0xfe
#define MXT_MSG_INVALID			0xff

/* MXT_GEN_COMMAND message field */
#define MXT_COMMAND_STATUS		0
#define MXT_COMMAND_CHECKSUM		1	/* 3 bytes */

/* MXT_TOUCH_MULTI message field */
#define MXT_TOUCH_STATUS		0
#define MXT_TOUCH_XPOSMSB		1
#define MXT_TOUCH_YPOSMSB		2
#define MXT_TOUCH_XYPOSLSB		3
#define MXT_TOUCH_TCHAREA		4
#define MXT_TOUCH_TCHAMPLITUDE		5
#define MXT_TOUCH_TCHVECTOR		6

/* MXT_TOUCH_KEYARRAY message field */
#define MXT_KEYARRAY_STATUS		0
#define MXT_KEYARRAY_KEYSTATE		1	/* 4 bytes */

/* MXT_PROCI_GRIPFACE message field */
#define MXT_GRIPFACE_STATUS		0

/* MXT_PROCG_NOISE message field */
#define MXT_NOISE_STATUS		0
#define MXT_NOISE_GCAFDEPTH		1
#define MXT_NOISE_FREQINDEX		2

/* MXT_SPT_SELFTEST message field */
#define MXT_SELFTEST_STATUS		0
#define MXT_SELFTEST_INFO		1	/* 4 bytes */

/* Define for MXT_GEN_COMMAND */
#define MXT_RESET_VALUE			0x01	/* except 0x00 and 0xa5 */
#define MXT_BOOT_VALUE			0xa5
#define MXT_BACKUP_VALUE		0x55
#define MXT_CALIBRATE_VALUE		0x01

#define MXT_BACKUP_TIME			25	/* msec */
#define MXT_RESET_TIME			65	/* msec */
#define MXT_FWRESET_TIME		175	/* msec */

/* Command to unlock bootloader */
#define MXT_UNLOCK_CMD_MSB		0xaa
#define MXT_UNLOCK_CMD_LSB		0xdc

/* Bootloader mode status */
#define MXT_WAITING_BOOTLOAD_CMD	0xc0	/* valid 7 6 bit only */
#define MXT_WAITING_FRAME_DATA		0x80	/* valid 7 6 bit only */
#define MXT_FRAME_CRC_CHECK		0x02
#define MXT_FRAME_CRC_FAIL		0x03
#define MXT_FRAME_CRC_PASS		0x04
#define MXT_APP_CRC_FAIL		0x40	/* valid 7 8 bit only */
#define MXT_BOOT_STATUS_MASK		0x3f

/* Bit field for MXT_TOUCH_CTRL */
#define MXT_TOUCH_ENABLE		0x01
#define MXT_TOUCH_RPTEN			0x02
#define MXT_TOUCH_DISAMP		0x04
#define MXT_TOUCH_DISVECT		0x08
#define MXT_TOUCH_DISMOVE		0x10
#define MXT_TOUCH_DISREL		0x20
#define MXT_TOUCH_DISPRSS		0x40
#define MXT_TOUCH_SCANEN		0x80

/* Bit field for MXT_KEYARRAY_CTRL */
#define MXT_KEYARRAY_ENABLE		0x01
#define MXT_KEYARRAY_RPTEN		0x02
#define MXT_KEYARRAY_INTAKSEN		0x80

/* Bit field for MXT_PROCG_NOISE */
#define MXT_NOISE_ENABLE		0x01
#define MXT_NOISE_RPTEN			0x02
#define MXT_NOISE_FREQHEN		0x04
#define MXT_NOISE_MEDIANEN		0x08
#define MXT_NOISE_GCAFEN		0x10
#define MXT_NOISE_DISGCAFD		0x80

/* Bit field for MXT_SELFTEST_CTRL */
#define MXT_SELFTEST_ENABLE		0x01
#define MXT_SELFTEST_RPTEN		0x02

/* Bit field for MXT_SELFTEST_CMD */
#define MXT_SELFTEST_DONE		0x00
#define MXT_SELFTEST_AVDD		0x01
#define MXT_SELTTEST_PIN		0x11
#define MXT_SELTTEST_SIGNAL		0x17
#define MXT_SELTTEST_GAIN		0x20
#define MXT_SELTTEST_SPECIAL		0xf0
#define MXT_SELTTEST_ALL		0xfe

/* Bit field for MXT_COMMAND_STATUS */
#define MXT_COMMAND_COMSERR		(1 << 2)
#define MXT_COMMAND_CFGERR		(1 << 3)
#define MXT_COMMAND_CAL			(1 << 4)
#define MXT_COMMAND_SIGERR		(1 << 5)
#define MXT_COMMAND_OFL			(1 << 6)
#define MXT_COMMAND_RST			(1 << 7)

/* Bit field for MXT_TOUCH_STATUS */
#define MXT_TOUCH_UNGRIP		(1 << 0)
#define MXT_TOUCH_SUPPRESS		(1 << 1)
#define MXT_TOUCH_AMP			(1 << 2)
#define MXT_TOUCH_VECTOR		(1 << 3)
#define MXT_TOUCH_MOVE			(1 << 4)
#define MXT_TOUCH_RELEASE		(1 << 5)
#define MXT_TOUCH_PRESS			(1 << 6)
#define MXT_TOUCH_DETECT		(1 << 7)

/* Bit field for MXT_KEYARRAY_STATUS */
#define MXT_KEYARRAY_DETECT		(1 << 7)

/* Bit field for MXT_GRIPFACE_STATUS */
#define MXT_GRIPFACE_FACESUP		(1 << 0)

/* Bit field for MXT_NOISE_STATUS */
#define MXT_NOISE_FHCHG			(1 << 0)
#define MXT_NOISE_GCAFERR		(1 << 2)
#define MXT_NOISE_FHERR			(1 << 3)
#define MXT_NOISE_GCAFCHG		(1 << 4)

/* Touchscreen absolute values */
#define MXT_TOUCH_MAX_AMP		0xff
#define MXT_TOUCH_MAX_AREA		0x20
#define MXT_TOUCH_MAX_FINGER		0xff

/* MXT_GEN_MESSAGE_T5 object */
#define MXT_RPTID_NOMSG                0xff
#define MXT_FORCE_CALIBRATE_DELAY       (HZ / 2)

struct mxt_info {
	u8 family_id;
	u8 variant_id;
	u8 version;
	u8 build;
	u8 matrix_xsize;
	u8 matrix_ysize;
	u8 object_num;
	u8 info_crc[3];
	u8 config_crc[3];
};

struct mxt_object {
	u8 type;
	u16 start_address;
	u8 size;
	u8 instances;
	u8 num_report_ids;
};

struct mxt_message {
	u8 reportid;
	u8 message[0];		/* variable length at least 7 bytes */
	/*u8 checksum; */
};

struct mxt_finger {
	bool detect;
	int x;
	int y;
	int amp;
	int area;
};

/* Each client has this additional data */
struct mxt_data {
	struct i2c_client *client;
	struct i2c_client *bootloader;
	struct input_dev *input_touch;
	struct input_dev *input_keypad;
	struct mxt_object *object_table;
	u16 mem_size;
	struct mxt_info info;
	struct kobject *vkeys_dir;
	struct kobj_attribute vkeys_attr;
	struct mutex mutex;
	unsigned long dbgdump;
	struct delayed_work force_calibrate_delayed_work;
#ifdef CONFIG_HAS_EARLYSUSPEND
	struct early_suspend early_suspend;
#endif
	bool suspended;
	bool antipalm_disabled;
	bool medfilter_enabled;
	/* some saved config */
	u8 idleacqint;
	u8 actvacqint;
	u8 antipalm[2];
	/* message object location */
	u16 msgobj_addr;
	u8 msgobj_size;
	/* touch data */
	struct mxt_finger *finger;	/* size: touch_reportid_num */
	struct mxt_finger first_pressed;
	bool touch_opened;
	bool touch_x12bits;
	bool touch_y12bits;
	u8 touch_reportid;
	u8 touch_reportid_num;
	/* keypad data */
	unsigned int *keypad_map;	/* size: pdata->keypad.length */
	bool keypad_opened;
	u8 keypad_reportid;
	/* self test */
	struct completion test_done;
	u8 test_result[6];
	struct bin_attribute mem_access_attr;
	bool debug_enabled;
	bool driver_paused;
};

static bool mxt_is_irq_assert(struct mxt_data *data)
{
	struct mxt_platform_data *pdata = data->client->dev.platform_data;
	if (pdata->irqflags & (IRQF_TRIGGER_RISING | IRQF_TRIGGER_HIGH))
		return gpio_get_value_cansleep(pdata->irqpin) == 1;
	else
		return gpio_get_value_cansleep(pdata->irqpin) == 0;
}

/* note: irq must disable before call this function */
static int mxt_wait_irq_assert(struct mxt_data *data)
{
	int i;

	for (i = 0; i < 10; i++) {
		if (mxt_is_irq_assert(data))
			return 0;
		msleep(10);
	}

	return -EBUSY;
}

static int mxt_check_bootloader(struct i2c_client *client, unsigned int state)
{
	u8 val;

recheck:
	if (i2c_master_recv(client, &val, 1) != 1) {
		dev_err(&client->dev, "%s: i2c recv failed\n", __func__);
		return -EIO;
	}

	switch (state) {
	case MXT_WAITING_BOOTLOAD_CMD:
	case MXT_WAITING_FRAME_DATA:
		val &= ~MXT_BOOT_STATUS_MASK;
		break;
	case MXT_FRAME_CRC_PASS:
		if (val == MXT_FRAME_CRC_CHECK)
			goto recheck;
		break;
	default:
		return -EINVAL;
	}

	if (val != state) {
		dev_err(&client->dev, "Unvalid bootloader mode state\n");
		return -EINVAL;
	}

	return 0;
}

static int mxt_unlock_bootloader(struct i2c_client *client)
{
	u8 buf[2];

	buf[0] = MXT_UNLOCK_CMD_LSB;
	buf[1] = MXT_UNLOCK_CMD_MSB;

	if (i2c_master_send(client, buf, 2) != 2) {
		dev_err(&client->dev, "%s: i2c send failed\n", __func__);
		return -EIO;
	}

	return 0;
}

static int mxt_fw_write(struct i2c_client *client,
			const void *data, unsigned int frame_size)
{
	if (i2c_master_send(client, data, frame_size) != frame_size) {
		dev_err(&client->dev, "%s: i2c send failed\n", __func__);
		return -EIO;
	}

	return 0;
}

static int mxt_read_regbuf(struct i2c_client *client,
			   u16 reg, u16 len, void *val)
{
	struct i2c_msg xfer[2];
	u8 buf[2];

	buf[0] = reg & 0xff;
	buf[1] = (reg >> 8) & 0xff;

	/* Write register */
	xfer[0].addr = client->addr;
	xfer[0].flags = 0;
	xfer[0].len = 2;
	xfer[0].buf = buf;

	/* Read data */
	xfer[1].addr = client->addr;
	xfer[1].flags = I2C_M_RD;
	xfer[1].len = len;
	xfer[1].buf = val;

	if (i2c_transfer(client->adapter, xfer, 2) != 2) {
		dev_err(&client->dev, "%s: i2c transfer failed "
			"at %d size %d\n", __func__, reg, len);
		return -EIO;
	}

	return 0;
}

static int mxt_read_reg(struct i2c_client *client, u16 reg, u8 *val)
{
	return mxt_read_regbuf(client, reg, 1, val);
}

static int mxt_write_regbuf(struct i2c_client *client,
			    u16 reg, u16 len, const void *val)
{
	u8 buf[2 + len];	/* no problem since len<=256 */

	buf[0] = reg & 0xff;
	buf[1] = (reg >> 8) & 0xff;
	memcpy(buf + 2, val, len);

	if (i2c_master_send(client, buf, sizeof(buf)) != sizeof(buf)) {
		dev_err(&client->dev, "%s: i2c send failed "
			"at %d size %d\n", __func__, reg, len);
		return -EIO;
	}

	return 0;
}

static int mxt_read_object_table(struct i2c_client *client,
				 int index, struct mxt_object *object)
{
	int error;
	u8 buf[MXT_OBJECT_LENGTH];

	error = mxt_read_regbuf(client,
				MXT_OBJECT_START + MXT_OBJECT_LENGTH * index,
				MXT_OBJECT_LENGTH, buf);
	if (!error) {
		object->type = buf[MXT_OBJECT_TYPE];
		object->start_address = buf[MXT_OBJECT_ADDRLSB] |
		    (buf[MXT_OBJECT_ADDRMSB] << 8);
		object->size = buf[MXT_OBJECT_SIZE];
		object->instances = buf[MXT_OBJECT_INST];
		object->num_report_ids = buf[MXT_OBJECT_IDNUM];
	}

	return error;
}

static struct mxt_object *mxt_get_object(struct mxt_data *data, u8 type)
{
	struct mxt_object *object;
	int i;

	for (i = 0; i < data->info.object_num; i++) {
		object = data->object_table + i;
		if (object->type == type)
			return object;
	}

	dev_err(&data->client->dev, "Invalid object type(%d)\n", type);
	return NULL;
}

static int mxt_get_reportid(struct mxt_data *data, u8 type)
{
	int i;
	int reportid = MXT_MSG_FIRST;

	for (i = 0; i < data->info.object_num; i++) {
		if (data->object_table[i].type == type)
			return reportid;

		reportid += data->object_table[i].num_report_ids;
	}

	return -ENOENT;
}

static int mxt_read_message(struct mxt_data *data,
			    struct mxt_message *message, int number)
{
	char *buf = (char *)message;
	int count;

	for (count = 0; count < number; count++) {
		int ret;

		/* check irq still assert first */
		if (!mxt_is_irq_assert(data))
			break;

		/* then issue read transfer
		   just send address in first transfer to save bandwidth */
		if (count == 0)
			ret = mxt_read_regbuf(data->client,
					      data->msgobj_addr,
					      data->msgobj_size,
					      buf + data->msgobj_size * count);
		else
			ret = i2c_master_recv(data->client,
					      buf + data->msgobj_size * count,
					      data->msgobj_size);

		if (ret < 0)
			return ret;
		if (data->debug_enabled) {
			print_hex_dump(KERN_DEBUG, "MXT MSG:", DUMP_PREFIX_NONE,
				       16, 1, buf + data->msgobj_size * count,
				       sizeof(struct mxt_message), false);
		}
	}

	return count;
}

static int mxt_read_objbuf(struct mxt_data *data,
			   u8 type, u8 offset, u16 len, void *val)
{
	struct mxt_object *object;
	u16 reg;

	object = mxt_get_object(data, type);
	if (!object)
		return -EINVAL;

	reg = object->start_address;
	return mxt_read_regbuf(data->client, reg + offset, len, val);
}

static int mxt_read_object(struct mxt_data *data, u8 type, u8 offset, u8 *val)
{
	return mxt_read_objbuf(data, type, offset, 1, val);
}

static int mxt_write_objbuf(struct mxt_data *data,
			    u8 type, u8 offset, u16 len, const void *val)
{
	struct mxt_object *object;
	u16 reg;

	object = mxt_get_object(data, type);
	if (!object)
		return -EINVAL;

	reg = object->start_address;
	return mxt_write_regbuf(data->client, reg + offset, len, val);
}

static int mxt_write_object(struct mxt_data *data, u8 type, u8 offset, u8 val)
{
	return mxt_write_objbuf(data, type, offset, 1, &val);
}

static int mxt_write_objbits(struct mxt_data *data,
			     u8 type, u8 offset, u8 bits, bool set)
{
	u8 temp;
	int error;

	error = mxt_read_object(data, type, offset, &temp);
	if (!error) {
		if (set)
			temp |= bits;
		else
			temp &= ~bits;

		error = mxt_write_object(data, type, offset, temp);
	}

	return error;
}

static int mxt_reset(struct mxt_data *data, bool upgrade)
{
	int error;

	error = mxt_write_object(data, MXT_GEN_COMMAND, MXT_COMMAND_RESET,
				 upgrade ? MXT_BOOT_VALUE : MXT_RESET_VALUE);
	if (!error) {
		/* wait until reset complete and irq assert since
		   maxTouch doesn't response i2c access during reset */
		msleep(MXT_RESET_TIME);
		error = mxt_wait_irq_assert(data);
	}

	if (error)
		dev_err(&data->client->dev, "fail to issue reset command\n");

	return error;
}

static int mxt_backup(struct mxt_data *data)
{
	int error;

	error = mxt_write_object(data, MXT_GEN_COMMAND,
				 MXT_COMMAND_BACKUPNV, MXT_BACKUP_VALUE);
	if (!error) {
		/* wait until backup complete and irq assert since
		   reset command always follow backup command */
		msleep(MXT_BACKUP_TIME);
		error = mxt_wait_irq_assert(data);
	}

	if (error)
		dev_err(&data->client->dev, "fail to issue backup command\n");

	return error;
}

static int mxt_set_antipalm(struct mxt_data *data, bool enable)
{
	int error = 0;

	if (data->antipalm_disabled != !enable) {
		static const u8 val[] = { 0, 0 };

		error = mxt_write_objbuf(data, MXT_GEN_ACQUIRE,
					 MXT_ACQUIRE_ATCHFRCCALTHR, sizeof(val),
					 enable ? data->antipalm : val);
		if (!error) {
			data->antipalm_disabled = !enable;
			if (data->dbgdump >= 1)
				dev_info(&data->client->dev, "anti-palm %s\n",
					 enable ? "enabled" : "disabled");
		} else
			dev_err(&data->client->dev, "fail to set anti-palm");
	}

	return error;
}

static int mxt_set_medfilter(struct mxt_data *data, bool enable)
{
	int error = 0;

	if (data->medfilter_enabled != enable) {
		error = mxt_write_objbits(data, MXT_PROCG_NOISE,
					  MXT_NOISE_CTRL, MXT_NOISE_MEDIANEN,
					  enable);
		if (!error) {
			data->medfilter_enabled = enable;
			if (data->dbgdump >= 1)
				dev_info(&data->client->dev, "med-filter %s\n",
					 enable ? "enabled" : "disabled");
		} else
			dev_err(&data->client->dev, "fail to set med-filter");
	}

	return error;
}

static bool mxt_in_rect(const struct mxt_rect *rect, int x, int y)
{
	return x >= rect->left && x < rect->left + rect->width &&
	    y >= rect->top && y < rect->top + rect->height;
}

static bool mxt_in_surface(struct mxt_data *data, struct mxt_finger *finger)
{
	struct mxt_platform_data *pdata = data->client->dev.platform_data;
	const struct mxt_keypad_data *keypad = pdata->keypad;

	if (!finger->detect)
		return false;

	if (mxt_in_rect(&pdata->tcharea, finger->x, finger->y))
		return true;

	if (keypad && mxt_in_rect(&keypad->kparea, finger->x, finger->y))
		return true;

	return false;
}

static void mxt_report_touchevent(struct mxt_data *data)
{
	struct device *dev = &data->client->dev;
	struct mxt_finger *finger = data->finger;
	struct input_dev *input_dev = data->input_touch;
	int id, finger_num = 0;

	if (input_dev) {
		for (id = 0; id < data->touch_reportid_num; id++) {
			if (!mxt_in_surface(data, &finger[id]))
				continue;

			input_report_abs(input_dev, ABS_MT_TRACKING_ID, id);
			input_report_abs(input_dev, ABS_MT_POSITION_X, max(1, finger[id].x));	/* for fruit ninja */
			input_report_abs(input_dev, ABS_MT_POSITION_Y, max(1, finger[id].y));	/* for fruit ninja */
			input_report_abs(input_dev, ABS_MT_PRESSURE,
					 max(1, finger[id].amp));
			input_report_abs(input_dev, ABS_MT_WIDTH_MAJOR,
					 max(1, finger[id].area));
			input_mt_sync(input_dev);

			finger_num++;

			if (data->dbgdump >= 2)
				dev_info(dev,
					 "mth(%02u): %04d %04d %03d %03d\n",
					 id, finger[id].x, finger[id].y,
					 finger[id].amp, finger[id].area);
		}

		if (finger_num == 0)
			input_mt_sync(input_dev);

		input_sync(input_dev);
	}
}

static void mxt_clear_touchevent(struct mxt_data *data)
{
	struct mxt_finger *finger = data->finger;
	bool has_touch = false;
	unsigned int i;

	data->first_pressed.detect = false;
	for (i = 0; i < data->touch_reportid_num; i++) {
		if (!finger[i].detect)
			continue;
		finger[i].detect = false;
		has_touch = true;
	}

	if (has_touch)
		mxt_report_touchevent(data);
}

static int mxt_update_device(struct mxt_data *data)
{
	int error;

	error = mxt_write_object(data,
				 MXT_GEN_POWER, MXT_POWER_IDLEACQINT,
				 data->suspended ? 0 : data->idleacqint);
	if (error)
		return error;

	error = mxt_write_object(data,
				 MXT_GEN_POWER, MXT_POWER_ACTVACQINT,
				 data->suspended ? 0 : data->actvacqint);
	if (error)
		return error;

	error = mxt_write_objbits(data,
				  MXT_TOUCH_MULTI, MXT_TOUCH_CTRL,
				  MXT_TOUCH_RPTEN | MXT_TOUCH_ENABLE,
				  !data->suspended && data->touch_opened);
	if (error)
		return error;

	error = mxt_write_objbits(data,
				  MXT_TOUCH_KEYARRAY, MXT_KEYARRAY_CTRL,
				  MXT_KEYARRAY_RPTEN | MXT_KEYARRAY_ENABLE,
				  !data->suspended && data->keypad_opened);
	if (error)
		return error;

	if (data->suspended || !data->touch_opened) {
		error = mxt_set_antipalm(data, true);
		if (error)
			return error;

		mxt_clear_touchevent(data);
	}

	return 0;
}

static void mxt_command_event(struct mxt_data *data,
			      struct mxt_message *message)
{
	bool rst, ofl, sigerr, cal, cfgerr, comserr;

	rst = !!(message->message[MXT_COMMAND_STATUS] & MXT_COMMAND_RST);
	ofl = !!(message->message[MXT_COMMAND_STATUS] & MXT_COMMAND_OFL);
	sigerr = !!(message->message[MXT_COMMAND_STATUS] & MXT_COMMAND_SIGERR);
	cal = !!(message->message[MXT_COMMAND_STATUS] & MXT_COMMAND_CAL);
	cfgerr = !!(message->message[MXT_COMMAND_STATUS] & MXT_COMMAND_CFGERR);
	comserr =
	    !!(message->message[MXT_COMMAND_STATUS] & MXT_COMMAND_COMSERR);

	/* save config CRC */
	memcpy(data->info.config_crc,
	       &message->message[MXT_COMMAND_CHECKSUM],
	       sizeof(data->info.config_crc));

	/* sync internal state with device */
	if (rst) {
		mxt_clear_touchevent(data);
		mxt_update_device(data);
	}

	if (data->dbgdump >= 1)
		dev_info(&data->client->dev,
			 "cmd(%02u): %02x %02x %02x%s%s%s%s%s%s\n",
			 message->reportid,
			 message->message[MXT_COMMAND_CHECKSUM],
			 message->message[MXT_COMMAND_CHECKSUM + 1],
			 message->message[MXT_COMMAND_CHECKSUM + 2],
			 rst ? " RST" : "",
			 ofl ? " OFL" : "",
			 sigerr ? " SIGERR" : "",
			 cal ? " CAL" : "",
			 cfgerr ? " CFGERR" : "", comserr ? " COMSERR" : "");
}

static int mxt_finger_count(struct mxt_data *data)
{
	struct mxt_finger *finger = data->finger;
	unsigned int i, count = 0;

	for (i = 0; i < data->touch_reportid_num; i++)
		count += finger[i].detect;

	return count;
}

static int mxt_finger_distance(struct mxt_finger *f1, struct mxt_finger *f2)
{
	return max(abs(f1->x - f2->x), abs(f1->y - f2->y));
}

static void mxt_input_touchevent(struct mxt_data *data,
				 struct mxt_message *message, int id)
{
	struct mxt_finger *finger = &data->finger[id];
	struct mxt_finger *pressed = &data->first_pressed;
	int new_count, old_count = mxt_finger_count(data);
	struct device *dev = &data->client->dev;

	u8 status = message->message[MXT_TOUCH_STATUS];
	u8 xmsb = message->message[MXT_TOUCH_XPOSMSB];
	u8 ymsb = message->message[MXT_TOUCH_YPOSMSB];
	u8 xylsb = message->message[MXT_TOUCH_XYPOSLSB];

	if (data->touch_x12bits)
		finger->x = (xmsb << 4) | ((xylsb & 0xf0) >> 4);
	else
		finger->x = (xmsb << 2) | ((xylsb & 0xc0) >> 6);

	if (data->touch_y12bits)
		finger->y = (ymsb << 4) | ((xylsb & 0x0f) >> 0);
	else
		finger->y = (ymsb << 2) | ((xylsb & 0x0c) >> 2);

	finger->detect = !!(status & MXT_TOUCH_DETECT);
	finger->amp = message->message[MXT_TOUCH_TCHAMPLITUDE];
	finger->area = message->message[MXT_TOUCH_TCHAREA];

	new_count = mxt_finger_count(data);
	if (new_count >= 2)
		pressed->detect = false;
	else if (old_count == 0 && new_count == 1)
		*pressed = *finger;
	else if (new_count == 0 && pressed->detect) {
		pressed->detect = false;

		/* disable anti-palm algorithm when the movement
		   is bigger enough to improve water resistance */
		if (mxt_finger_distance(finger, pressed) >= 100)
			mxt_set_antipalm(data, false);

		/* enable the median filter algorithm when the external
		   power supply exist to reduce the noise influence */
		mxt_set_medfilter(data, power_supply_is_system_supplied());
	}

	if (data->dbgdump >= 2)
		dev_info(dev, "tch(%02u): %d %04d %04d %03d %03d %02x\n",
			 message->reportid, finger->detect, finger->x,
			 finger->y, finger->amp, finger->area, status);
}

static void mxt_input_keyevent(struct mxt_data *data,
			       struct mxt_message *message)
{
	unsigned int i;
	struct device *dev = &data->client->dev;
	struct input_dev *input_dev = data->input_keypad;
	struct mxt_platform_data *pdata = data->client->dev.platform_data;
	const struct mxt_keypad_data *keypad = pdata->keypad;
	u8 *keystate = &message->message[MXT_KEYARRAY_KEYSTATE];

	if (input_dev) {
		for (i = 0; i < keypad->length; i++) {
			unsigned int j = (i >> 3), k = (i & 7);
			input_report_key(input_dev,
					 data->keypad_map[i],
					 (keystate[j] >> k) & 1);

			if (data->dbgdump >= 2)
				dev_info(dev, "key(%02u): %04x %d\n", i,
					 data->keypad_map[i],
					 (keystate[j] >> k) & 1);
		}

		input_sync(input_dev);
	}
}

static void mxt_gripface_event(struct mxt_data *data,
			       struct mxt_message *message)
{
	bool facesup
	    = !!(message->message[MXT_GRIPFACE_STATUS] & MXT_GRIPFACE_FACESUP);

	/* sync internal state with device */
	if (facesup)
		mxt_clear_touchevent(data);

	if (data->dbgdump >= 1)
		dev_info(&data->client->dev, "sup(%02u): %s\n",
			 message->reportid, facesup ? "active" : "inactive");
}

static void mxt_noise_event(struct mxt_data *data, struct mxt_message *message)
{
	bool gcafchg, fherr, gcaferr, fhchg;

	gcafchg = !!(message->message[MXT_NOISE_STATUS] & MXT_NOISE_GCAFCHG);
	fherr = !!(message->message[MXT_NOISE_STATUS] & MXT_NOISE_FHERR);
	gcaferr = !!(message->message[MXT_NOISE_STATUS] & MXT_NOISE_GCAFERR);
	fhchg = !!(message->message[MXT_NOISE_STATUS] & MXT_NOISE_FHCHG);

	if (data->dbgdump >= 1)
		dev_info(&data->client->dev,
			 "nse(%02u): %02d %d%s%s%s%s\n",
			 message->reportid,
			 message->message[MXT_NOISE_GCAFDEPTH],
			 message->message[MXT_NOISE_FREQINDEX],
			 gcafchg ? " GCAFCHG" : "",
			 fherr ? " FHERR" : "",
			 gcaferr ? " GCAFERR" : "", fhchg ? " FHCHG" : "");
}

static void mxt_test_event(struct mxt_data *data, struct mxt_message *message)
{
	memcpy(data->test_result, message->message, sizeof(data->test_result));
	complete_all(&data->test_done);

	if (data->dbgdump >= 1)
		dev_info(&data->client->dev,
			 "tst(%02u): %02x %02x %02x %02x %02x %02x\n",
			 message->reportid, message->message[0],
			 message->message[1], message->message[2],
			 message->message[3], message->message[4],
			 message->message[5]);
}

static irqreturn_t mxt_interrupt(int irq, void *dev_id)
{
	struct mxt_data *data = dev_id;
	const unsigned int max_count = 8;
	char buf[data->msgobj_size * max_count];
	struct device *dev = &data->client->dev;
	u8 touch_reportid_end = data->touch_reportid + data->touch_reportid_num;
	bool has_touch = false;
	int id, i, count;

	mutex_lock(&data->mutex);
	do {
		count = mxt_read_message(data,
					 (struct mxt_message *)buf, max_count);
		if (count < 0) {
			dev_err(dev, "Failed to read message\n");
			break;
		}

		if (data->driver_paused) {
			dev_dbg(dev, "Driver is paused\n");
			continue;
		}

		for (i = 0; i < count; i++) {
			struct mxt_message *message =
			    (struct mxt_message *)&buf[data->msgobj_size * i];

			if (message->reportid >= data->touch_reportid &&
			    message->reportid < touch_reportid_end) {
				id = message->reportid - data->touch_reportid;
				mxt_input_touchevent(data, message, id);
				has_touch = true;
			} else if (message->reportid == data->keypad_reportid)
				mxt_input_keyevent(data, message);
			else if (message->reportid ==
				 mxt_get_reportid(data, MXT_GEN_COMMAND))
				mxt_command_event(data, message);
			else if (message->reportid ==
				 mxt_get_reportid(data, MXT_PROCG_NOISE))
				mxt_noise_event(data, message);
			else if (message->reportid ==
				 mxt_get_reportid(data, MXT_PROCI_GRIPFACE))
				mxt_gripface_event(data, message);
			else if (message->reportid ==
				 mxt_get_reportid(data, MXT_SPT_SELFTEST))
				mxt_test_event(data, message);
		}
	} while (count != 0);

	/* report touch event in batch for efficiency */
	if (has_touch)
		mxt_report_touchevent(data);

	mutex_unlock(&data->mutex);
	return IRQ_HANDLED;
}

static int mxt_check_reg_init(struct mxt_data *data, bool *backup)
{
	struct mxt_platform_data *pdata = data->client->dev.platform_data;
	const struct mxt_config_data *config = pdata->config;
	struct device *dev = &data->client->dev;
	char vendor[8];
	int i, error;

	*backup = false;	/* assume no backup is needed */

	if (!config) {
		dev_dbg(dev, "No cfg data defined, skipping reg init\n");
		return 0;
	}

	error = mxt_read_objbuf(data, MXT_SPT_USERDATA,
				0, sizeof(vendor), vendor);
	if (error) {
		dev_err(dev, "Fail to read user data\n");
		return error;
	}
	dev_info(dev, "vendor: %s\n", vendor[0] ? vendor : "Wintek");

	for (; config->vendor != NULL; config++) {
		if (strcmp(config->vendor, vendor) == 0)
			break;
	}

	if (config->vendor == NULL) {
		dev_err(dev, "Unknown touch screen vendor\n");
		return -ENOENT;
	}

	if (memcmp(data->info.config_crc, config->checksum,
		   sizeof(data->info.config_crc)) == 0) {
		dev_dbg(dev, "Checksum match, skipping reg init\n");
		return 0;
	}

	for (i = 0; config->object[i].size != 0; i++) {
		error = mxt_write_objbuf(data, config->object[i].type, 0,
					 config->object[i].size,
					 config->object[i].data);
		if (error) {
			dev_err(dev, "Fail to write config data\n");
			return error;
		}
	}

	*backup = true;		/* tell caller to backup config */
	return 0;
}

static int mxt_make_highchg(struct mxt_data *data)
{
	/* utilize interrupt handler to drain and
	   process pending messages until CHG become high */
	mxt_interrupt(data->client->irq, data);
	return 0;
}

static int mxt_get_info(struct mxt_data *data)
{
	struct i2c_client *client = data->client;
	struct mxt_info *info = &data->info;
	int error;
	u8 val;

	error = mxt_read_reg(client, MXT_FAMILY_ID, &val);
	if (error)
		return error;
	info->family_id = val;

	error = mxt_read_reg(client, MXT_VARIANT_ID, &val);
	if (error)
		return error;
	info->variant_id = val;

	error = mxt_read_reg(client, MXT_VERSION, &val);
	if (error)
		return error;
	info->version = val;

	error = mxt_read_reg(client, MXT_BUILD, &val);
	if (error)
		return error;
	info->build = val;

	error = mxt_read_reg(client, MXT_OBJECT_NUM, &val);
	if (error)
		return error;
	info->object_num = val;

	return 0;
}

static int mxt_get_object_table(struct mxt_data *data)
{
	int error;
	int i;
	u8 reportid = MXT_MSG_FIRST;
	u16 end_address;

	data->mem_size = 0;

	for (i = 0; i < data->info.object_num; i++) {
		struct mxt_object *object = data->object_table + i;

		error = mxt_read_object_table(data->client, i, object);
		if (error)
			return error;

		/* update cached info */
		switch (object->type) {
		case MXT_GEN_MESSAGE:
			data->msgobj_addr = object->start_address;
			data->msgobj_size = object->size + 1;
			break;
		case MXT_TOUCH_MULTI:
			data->touch_reportid = reportid;
			data->touch_reportid_num =
			    object->num_report_ids * (object->instances + 1);
			break;
		case MXT_TOUCH_KEYARRAY:
			data->keypad_reportid = reportid;
			break;
		}

		if (object->num_report_ids) {
			reportid += object->num_report_ids *
			    (object->instances + 1);
		}

		end_address = object->start_address
		    + (object->size + 1) * (object->instances + 1) - 1;

		if (end_address >= data->mem_size)
			data->mem_size = end_address + 1;
	}

	return 0;
}

static int mxt_get_misc_info(struct mxt_data *data)
{
	int error;
	u8 val;

	/* Update matrix size at info struct */
	error = mxt_read_reg(data->client, MXT_MATRIX_X_SIZE, &val);
	if (error)
		return error;
	data->info.matrix_xsize = val;

	error = mxt_read_reg(data->client, MXT_MATRIX_Y_SIZE, &val);
	if (error)
		return error;
	data->info.matrix_ysize = val;

	/* Save info block crc */
	error = mxt_read_regbuf(data->client,
				MXT_OBJECT_START +
				MXT_OBJECT_SIZE * data->info.object_num,
				sizeof(data->info.info_crc),
				data->info.info_crc);
	if (error)
		return error;

	/* Save some config value */
	error = mxt_read_object(data, MXT_GEN_POWER,
				MXT_POWER_IDLEACQINT, &val);
	if (error)
		return error;
	data->idleacqint = val;

	error = mxt_read_object(data, MXT_GEN_POWER,
				MXT_POWER_ACTVACQINT, &val);
	if (error)
		return error;
	data->actvacqint = val;

	error = mxt_read_objbuf(data,
				MXT_GEN_ACQUIRE, MXT_ACQUIRE_ATCHFRCCALTHR,
				sizeof(data->antipalm), data->antipalm);
	if (error)
		return error;

	/* Determine position format */
	error = mxt_read_object(data, MXT_TOUCH_MULTI,
				MXT_TOUCH_XRANGE_MSB, &val);
	if (error)
		return error;
	data->touch_x12bits = (val >= 4);

	error = mxt_read_object(data, MXT_TOUCH_MULTI,
				MXT_TOUCH_YRANGE_MSB, &val);
	if (error)
		return error;
	data->touch_y12bits = (val >= 4);

	return 0;
}

static int mxt_initialize(struct mxt_data *data)
{
	struct i2c_client *client = data->client;
	struct mxt_info *info = &data->info;
	bool backup;
	int error;

	/* Wait maxTouch fully initialization */
	error = mxt_wait_irq_assert(data);
	if (error)
		return error;

	/* Get information block */
	error = mxt_get_info(data);
	if (error)
		return error;

	kfree(data->object_table);
	data->object_table = kcalloc(info->object_num,
				     sizeof(struct mxt_object), GFP_KERNEL);
	if (!data->object_table) {
		dev_err(&client->dev, "Failed to allocate memory\n");
		return -ENOMEM;
	}

	/* Get object table information */
	error = mxt_get_object_table(data);
	if (error)
		return error;

	kfree(data->finger);
	data->finger = kcalloc(data->touch_reportid_num,
			       sizeof(struct mxt_finger), GFP_KERNEL);
	if (!data->finger) {
		dev_err(&client->dev, "Failed to allocate memory\n");
		return -ENOMEM;
	}

	/* Get misc info */
	error = mxt_get_misc_info(data);
	if (error)
		return error;

	/* Check register init values */
	error = mxt_make_highchg(data);	/* receive config crc here */
	if (error)
		return error;

	error = mxt_check_reg_init(data, &backup);
	if (error)
		return error;

	if (backup) {
		dev_info(&client->dev, "Backup config to NV\n");

		error = mxt_backup(data);
		if (error)
			return error;

		error = mxt_reset(data, false);
		if (error)
			return error;

		/* Reload misc info */
		error = mxt_get_misc_info(data);
		if (error)
			return error;
	}

	return 0;
}

static ssize_t mxt_vkeys_show(struct kobject *kobj,
			      struct kobj_attribute *attr, char *buf)
{
	struct mxt_data *data = container_of(attr, struct mxt_data, vkeys_attr);
	struct mxt_platform_data *pdata = data->client->dev.platform_data;
	const struct mxt_keypad_data *keypad = pdata->keypad;
	int i, count = 0;

	for (i = 0; keypad && i < keypad->length; i++) {
		int width = keypad->button[i].width;
		int height = keypad->button[i].height;
		int midx = keypad->button[i].left + width / 2;
		int midy = keypad->button[i].top + height / 2;

		count += snprintf(buf + count, PAGE_SIZE - count,
				  "0x%02x:%d:%d:%d:%d:%d:",
				  EV_KEY, keypad->keymap[i],
				  midx, midy, width, height);
	}

	count -= 1;		/* remove the latest colon */
	count += snprintf(buf + count, PAGE_SIZE - count, "\n");
	return count;
}

static ssize_t mxt_object_show(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	struct mxt_data *data = dev_get_drvdata(dev);
	struct mxt_object *object;
	int count = 0;
	int i, j;
	int error;
	u8 val;

	mutex_lock(&data->mutex);

	/* print information block */
	count += snprintf(buf + count, PAGE_SIZE - count, "Family ID: %02x"
			  " Variant ID: %02x Version: %02x Build: %u\n",
			  data->info.family_id, data->info.variant_id,
			  data->info.version, data->info.build);

	count += snprintf(buf + count, PAGE_SIZE - count,
			  "Matrix X Size: %d Matrix Y Size: %d Object Num: %d\n",
			  data->info.matrix_xsize, data->info.matrix_ysize,
			  data->info.object_num);

	count += snprintf(buf + count, PAGE_SIZE - count,
			  "Info CRC: %02x %02x %02x Config CRC: %02x %02x %02x\n",
			  data->info.info_crc[0], data->info.info_crc[1],
			  data->info.info_crc[2], data->info.config_crc[0],
			  data->info.config_crc[1], data->info.config_crc[2]);

	/* then detailed object information */
	for (i = 0; i < data->info.object_num; i++) {
		object = data->object_table + i;

		count += snprintf(buf + count, PAGE_SIZE - count, "\nType: %02u"
				  " Addr: %04x Size: %03u Inst: %u Ids: %02u",
				  object->type, object->start_address,
				  object->size + 1, object->instances + 1,
				  object->num_report_ids);

		for (j = 0; j < object->size + 1; j++) {
			if (j % 16 == 0)
				count +=
				    snprintf(buf + count, PAGE_SIZE - count,
					     "\n%03u:", j);

			error = mxt_read_object(data, object->type, j, &val);
			if (error)
				break;

			count += snprintf(buf + count, PAGE_SIZE - count,
					  " %02x", val);
		}

		count += snprintf(buf + count, PAGE_SIZE - count, "\n");
	}

	mutex_unlock(&data->mutex);
	return count;
}

static ssize_t mxt_object_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct mxt_data *data = dev_get_drvdata(dev);
	u8 type, offset, val;
	int error;

	mutex_lock(&data->mutex);

	if (sscanf(buf, "%hhu:%hhu=%hhx", &type, &offset, &val) == 3) {
		error = mxt_write_object(data, type, offset, val);
		if (error)
			count = error;
	} else
		count = -EINVAL;

	mutex_unlock(&data->mutex);
	return count;
}

static ssize_t mxt_dbgdump_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct mxt_data *data = dev_get_drvdata(dev);
	struct mxt_finger *finger = data->finger;
	int count = 0;
	int id;

	mutex_lock(&data->mutex);

	for (id = 0; id < data->touch_reportid_num; id++) {
		count += snprintf(buf + count, PAGE_SIZE - count,
				  "fig(%02u): %d %04d %04d %03d %03d\n",
				  id, finger[id].detect, finger[id].x,
				  finger[id].y, finger[id].amp,
				  finger[id].area);
	}

	mutex_unlock(&data->mutex);
	return count;
}

static ssize_t mxt_dbgdump_store(struct device *dev,
				 struct device_attribute *attr,
				 const char *buf, size_t count)
{
	struct mxt_data *data = dev_get_drvdata(dev);
	unsigned long dbgdump;
	int error;

	mutex_lock(&data->mutex);

	error = strict_strtoul(buf, 0, &dbgdump);
	if (!error)
		data->dbgdump = dbgdump;

	mutex_unlock(&data->mutex);
	return error ? error : count;
}

static ssize_t mxt_selftest_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	struct mxt_data *data = dev_get_drvdata(dev);
	int count;

	mutex_lock(&data->mutex);
	count = sprintf(buf, "%02x, %02x, %02x, %02x, %02x, %02x\n",
			data->test_result[0], data->test_result[1],
			data->test_result[2], data->test_result[3],
			data->test_result[4], data->test_result[5]);

	mutex_unlock(&data->mutex);
	return count;
}

static int mxt_run_selftest(struct mxt_data *data)
{
	int error;
	u8 val;

	mutex_lock(&data->mutex);

	/* self test is running? */
	error = mxt_read_object(data, MXT_SPT_SELFTEST, MXT_SELFTEST_CMD, &val);
	if (error)
		goto exit;

	if (val != MXT_SELFTEST_DONE)
		goto exit;

	/* no, start self test */
	error = mxt_write_object(data, MXT_SPT_SELFTEST, MXT_SELFTEST_CTRL,
				 MXT_SELFTEST_RPTEN | MXT_SELFTEST_ENABLE);
	if (error)
		goto exit;

	error = mxt_write_object(data, MXT_SPT_SELFTEST,
				 MXT_SELFTEST_CMD, MXT_SELTTEST_ALL);
	if (error)
		goto exit;

	INIT_COMPLETION(data->test_done);

exit:
	mutex_unlock(&data->mutex);
	return error;
}

static ssize_t mxt_selftest_store(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf, size_t count)
{
	struct mxt_data *data = dev_get_drvdata(dev);
	int error;

	error = mxt_run_selftest(data);
	if (!error)
		error = wait_for_completion_interruptible(&data->test_done);
	if (error)
		count = error;

	return count;
}

static int mxt_load_fw(struct device *dev, const char *fn)
{
	struct mxt_data *data = dev_get_drvdata(dev);
	struct i2c_client *client = data->bootloader;
	const struct firmware *fw = NULL;
	unsigned int frame_size;
	unsigned int pos = 0;
	int ret;

	ret = request_firmware(&fw, fn, dev);
	if (ret) {
		dev_err(dev, "Unable to open firmware %s\n", fn);
		return ret;
	}

	/* Change to the bootloader mode */
	ret = mxt_reset(data, true);
	if (ret)
		goto out;

	ret = mxt_check_bootloader(client, MXT_WAITING_BOOTLOAD_CMD);
	if (ret)
		goto out;

	/* Unlock bootloader */
	mxt_unlock_bootloader(client);

	while (pos < fw->size) {
		ret = mxt_check_bootloader(client, MXT_WAITING_FRAME_DATA);
		if (ret)
			goto out;

		frame_size = ((*(fw->data + pos) << 8) | *(fw->data + pos + 1));

		/* We should add 2 at frame size as the the firmware data is not
		 * included the CRC bytes.
		 */
		frame_size += 2;

		/* Write one frame to device */
		mxt_fw_write(client, fw->data + pos, frame_size);

		ret = mxt_check_bootloader(client, MXT_FRAME_CRC_PASS);
		if (ret)
			goto out;

		pos += frame_size;

		dev_dbg(dev, "Updated %d bytes / %zd bytes\n", pos, fw->size);
	}

out:
	release_firmware(fw);
	return ret;
}

static ssize_t mxt_update_fw_store(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t count)
{
	struct mxt_data *data = dev_get_drvdata(dev);
	int error;

	mutex_lock(&data->mutex);
	disable_irq(data->client->irq);

	error = mxt_load_fw(dev, MXT_FW_NAME);
	if (error) {
		dev_err(dev, "The firmware update failed(%d)\n", error);
		count = error;
	} else {
		dev_dbg(dev, "The firmware update succeeded\n");

		/* Wait for reset */
		msleep(MXT_FWRESET_TIME);
		mxt_initialize(data);
	}

	enable_irq(data->client->irq);
	mutex_unlock(&data->mutex);

	return count;
}

static ssize_t mxt_pause_show(struct device *dev,
			      struct device_attribute *attr, char *buf)
{
	struct mxt_data *data = dev_get_drvdata(dev);
	ssize_t count;
	char c;

	c = data->driver_paused ? '1' : '0';
	count = sprintf(buf, "%c\n", c);

	return count;
}

static ssize_t mxt_pause_store(struct device *dev,
			       struct device_attribute *attr, const char *buf,
			       size_t count)
{
	struct mxt_data *data = dev_get_drvdata(dev);
	int i;

	if (sscanf(buf, "%u", &i) == 1 && i < 2) {
		data->driver_paused = (i == 1);
		dev_dbg(dev, "%s\n", i ? "paused" : "unpaused");
		return count;
	} else {
		dev_dbg(dev, "pause_driver write error\n");
		return -EINVAL;
	}
}

static ssize_t mxt_debug_enable_show(struct device *dev,
				     struct device_attribute *attr, char *buf)
{
	struct mxt_data *data = dev_get_drvdata(dev);
	int count;
	char c;

	c = data->debug_enabled ? '1' : '0';
	count = sprintf(buf, "%c\n", c);

	return count;
}

static ssize_t mxt_debug_enable_store(struct device *dev,
				      struct device_attribute *attr,
				      const char *buf, size_t count)
{
	struct mxt_data *data = dev_get_drvdata(dev);
	int i;

	if (sscanf(buf, "%u", &i) == 1 && i < 2) {
		data->debug_enabled = (i == 1);

		dev_dbg(dev, "%s\n", i ? "debug enabled" : "debug disabled");
		return count;
	} else {
		dev_dbg(dev, "debug_enabled write error\n");
		return -EINVAL;
	}
}

static DEVICE_ATTR(object, 0644, mxt_object_show, mxt_object_store);
static DEVICE_ATTR(dbgdump, 0644, mxt_dbgdump_show, mxt_dbgdump_store);
static DEVICE_ATTR(selftest, 0644, mxt_selftest_show, mxt_selftest_store);
static DEVICE_ATTR(update_fw, 0200, NULL, mxt_update_fw_store);
static DEVICE_ATTR(debug_enable, S_IWUSR | S_IRUSR, mxt_debug_enable_show,
		   mxt_debug_enable_store);
static DEVICE_ATTR(pause_driver, S_IWUSR | S_IRUSR, mxt_pause_show,
		   mxt_pause_store);

static int mxt_check_mem_access_params(struct mxt_data *data, loff_t off,
				       size_t *count)
{
	if (off >= data->mem_size)
		return -EIO;

	if (off + *count > data->mem_size)
		*count = data->mem_size - off;

	if (*count > MXT_MAX_BLOCK_WRITE)
		*count = MXT_MAX_BLOCK_WRITE;

	return 0;
}

static ssize_t mxt_mem_access_read(struct file *filp, struct kobject *kobj,
				   struct bin_attribute *bin_attr, char *buf,
				   loff_t off, size_t count)
{
	struct device *dev = container_of(kobj, struct device, kobj);
	struct mxt_data *data = dev_get_drvdata(dev);
	int ret = 0;

	ret = mxt_check_mem_access_params(data, off, &count);
	if (ret < 0)
		return ret;

	if (count > 0)
		ret = mxt_read_regbuf(data->client, off, count, buf);

	return ret == 0 ? count : ret;
}

static ssize_t mxt_mem_access_write(struct file *filp, struct kobject *kobj,
				    struct bin_attribute *bin_attr, char *buf,
				    loff_t off, size_t count)
{
	struct device *dev = container_of(kobj, struct device, kobj);
	struct mxt_data *data = dev_get_drvdata(dev);
	int ret = 0;

	ret = mxt_check_mem_access_params(data, off, &count);
	if (ret < 0)
		return ret;

	if (count > 0)
		ret = mxt_write_regbuf(data->client, off, count, buf);

	return ret == 0 ? count : 0;
}

static struct attribute *mxt_attrs[] = {
	&dev_attr_object.attr,
	&dev_attr_dbgdump.attr,
	&dev_attr_selftest.attr,
	&dev_attr_update_fw.attr,
	&dev_attr_debug_enable.attr,
	&dev_attr_pause_driver.attr,
	NULL
};

static const struct attribute_group mxt_attr_group = {
	.attrs = mxt_attrs,
};

static int mxt_touch_open(struct input_dev *dev)
{
	struct mxt_data *data = input_get_drvdata(dev);
	int error;

	mutex_lock(&data->mutex);
	data->touch_opened = true;
	error = mxt_update_device(data);
	mutex_unlock(&data->mutex);

	return error;
}

static void mxt_touch_close(struct input_dev *dev)
{
	struct mxt_data *data = input_get_drvdata(dev);

	mutex_lock(&data->mutex);
	data->touch_opened = false;
	mxt_update_device(data);
	mutex_unlock(&data->mutex);
}

#define CALIB_TIMEOUT 1000
static void mxt_do_force_calibration(struct mxt_data *data)
{
	int error;
	u8 val;
	int i = 0;

	error = mxt_write_object(data,
				 MXT_GEN_COMMAND, MXT_COMMAND_CALIBRATE, 0x01);
	if (error < 0) {
		goto error_seg;
	}

	while (i < CALIB_TIMEOUT) {
		error = mxt_read_object(data,
					MXT_GEN_COMMAND,
					MXT_COMMAND_CALIBRATE, &val);
		if (error < 0) {
			goto error_seg;
		}

		if ((val & 0x1) == 0)
			break;

		i++;
		mdelay(1);
	}

	return;
error_seg:
	dev_err(&data->client->dev, "calibration failed in %s\n", __func__);
}

static void mxt_force_calibrate_delayed_work(struct work_struct *work)
{
	struct delayed_work *delayed_work = to_delayed_work(work);
	struct mxt_data *data =
	    container_of(delayed_work, struct mxt_data,
			 force_calibrate_delayed_work);

	mxt_do_force_calibration(data);
}

static int mxt_touch_event(struct input_dev *dev,
			   unsigned int type, unsigned int code, int value)
{
	struct mxt_data *data = input_get_drvdata(dev);

	if (type == EV_SYN && code == SYN_CONFIG) {
		if (data->dbgdump >= 1) {
			dev_info(&data->client->dev,
				 "calibrate since device turn on with shelter\n");
		}
		schedule_delayed_work(&data->force_calibrate_delayed_work,
				      MXT_FORCE_CALIBRATE_DELAY);
	}

	return 0;
}

static int __devinit mxt_touch_setup(struct mxt_data *data)
{
	struct i2c_client *client = data->client;
	struct mxt_platform_data *pdata = client->dev.platform_data;
	struct input_dev *input_dev;
	int error;

	input_dev = input_allocate_device();
	if (!input_dev) {
		dev_err(&client->dev, "Failed to allocate touch dev\n");
		error = -ENOMEM;
		goto err;
	}

	input_dev->name = "mXT-touch";
	input_dev->id.bustype = BUS_I2C;
	input_dev->id.vendor = 0x4154;	/* ATmel  */
	input_dev->id.product = 0x0224;	/* mXT224 */
	input_dev->id.version = 0x0200;	/* 2.0    */
	input_dev->dev.parent = &client->dev;
	input_dev->open = mxt_touch_open;
	input_dev->close = mxt_touch_close;
	input_dev->event = mxt_touch_event;

	__set_bit(INPUT_PROP_DIRECT, input_dev->propbit);

	input_set_capability(input_dev, EV_ABS, ABS_MT_TRACKING_ID);
	input_set_capability(input_dev, EV_ABS, ABS_MT_POSITION_X);
	input_set_capability(input_dev, EV_ABS, ABS_MT_POSITION_Y);
	input_set_capability(input_dev, EV_ABS, ABS_MT_PRESSURE);
	input_set_capability(input_dev, EV_ABS, ABS_MT_WIDTH_MAJOR);

	input_set_abs_params(input_dev,
			     ABS_MT_TRACKING_ID, 0, MXT_TOUCH_MAX_FINGER, 0, 0);
	input_set_abs_params(input_dev,
			     ABS_MT_POSITION_X, pdata->tcharea.left,
			     pdata->tcharea.left + pdata->tcharea.width, 0, 0);
	input_set_abs_params(input_dev,
			     ABS_MT_POSITION_Y, pdata->tcharea.top,
			     pdata->tcharea.top + pdata->tcharea.height, 0, 0);
	input_set_abs_params(input_dev,
			     ABS_MT_PRESSURE, 0, MXT_TOUCH_MAX_AMP, 0, 0);
	input_set_abs_params(input_dev,
			     ABS_MT_WIDTH_MAJOR, 0, MXT_TOUCH_MAX_AREA, 0, 0);

	input_set_events_per_packet(input_dev, 64);

	data->input_touch = input_dev;
	input_set_drvdata(input_dev, data);

	error = input_register_device(input_dev);
	if (error) {
		dev_err(&client->dev, "Failed to register touch dev\n");
		goto err_free_device;
	}

	input_dev->phys = kobject_get_path(&input_dev->dev.kobj, GFP_KERNEL);
	if (input_dev->phys == NULL) {
		dev_err(&client->dev, "fail to get touch sysfs path.");
		error = -ENOMEM;
		goto err_unregister_device;
	}

	return 0;

err_unregister_device:
	input_unregister_device(input_dev);
	goto err;
err_free_device:
	input_free_device(input_dev);
err:
	return error;
}

static int mxt_keypad_open(struct input_dev *dev)
{
	struct mxt_data *data = input_get_drvdata(dev);
	int error;

	mutex_lock(&data->mutex);
	data->keypad_opened = true;
	error = mxt_update_device(data);
	mutex_unlock(&data->mutex);

	return error;
}

static void mxt_keypad_close(struct input_dev *dev)
{
	struct mxt_data *data = input_get_drvdata(dev);

	mutex_lock(&data->mutex);
	data->keypad_opened = false;
	mxt_update_device(data);
	mutex_unlock(&data->mutex);
}

static int __devinit mxt_keypad_setup(struct mxt_data *data)
{
	struct mxt_platform_data *pdata = data->client->dev.platform_data;
	const struct mxt_keypad_data *keypad = pdata->keypad;
	struct i2c_client *client = data->client;
	struct input_dev *input_dev;
	int i, error;

	if (keypad && keypad->length && !keypad->button) {
		data->keypad_map = kcalloc(keypad->length,
					   sizeof(keypad->keymap[0]),
					   GFP_KERNEL);
		if (!data->keypad_map) {
			dev_err(&client->dev, "Failed to allocate keymap\n");
			error = -ENOMEM;
			goto err;
		}
		memcpy(data->keypad_map, keypad->keymap,
		       sizeof(keypad->keymap[0]) * keypad->length);

		input_dev = input_allocate_device();
		if (!input_dev) {
			dev_err(&client->dev, "Failed to alloc keypad dev\n");
			error = -ENOMEM;
			goto err_free_keymap;
		}

		input_dev->name = "mXT-keypad";
		input_dev->id.bustype = BUS_I2C;
		input_dev->id.vendor = 0x4154;	/* ATmel  */
		input_dev->id.product = 0x0224;	/* mXT224 */
		input_dev->id.version = 0x0200;	/* 2.0    */
		input_dev->dev.parent = &client->dev;
		input_dev->open = mxt_keypad_open;
		input_dev->close = mxt_keypad_close;
		input_dev->keycodemax = keypad->length;
		input_dev->keycodesize = sizeof(data->keypad_map[0]);
		input_dev->keycode = data->keypad_map;

		if (keypad->repeat)
			set_bit(EV_REP, input_dev->evbit);

		for (i = 0; i < keypad->length; i++)
			input_set_capability(input_dev,
					     EV_KEY, data->keypad_map[i]);

		data->input_keypad = input_dev;
		input_set_drvdata(input_dev, data);

		error = input_register_device(input_dev);
		if (error) {
			dev_err(&client->dev, "Failed register keypad dev\n");
			goto err_free_device;
		}

		input_dev->phys =
		    kobject_get_path(&input_dev->dev.kobj, GFP_KERNEL);
		if (input_dev->phys == NULL) {
			dev_err(&client->dev, "fail to get keypad sysfs path.");
			error = -ENOMEM;
			goto err_unregister_device;
		}
	}

	return 0;

err_unregister_device:
	input_unregister_device(input_dev);
	goto err_free_keymap;
err_free_device:
	input_free_device(input_dev);
err_free_keymap:
	kfree(data->keypad_map);
err:
	return error;
}

static int __devinit mxt_setup(struct mxt_data *data)
{
	struct mxt_platform_data *pdata = data->client->dev.platform_data;
	if (pdata->setup)
		return pdata->setup(data->client, pdata);
	else
		return 0;
}

static int mxt_teardown(struct mxt_data *data)
{
	struct mxt_platform_data *pdata = data->client->dev.platform_data;
	if (pdata->teardown)
		return pdata->teardown(data->client, pdata);
	else
		return 0;
}

#if defined(CONFIG_PM) || defined(CONFIG_HAS_EARLYSUSPEND)
static int mxt_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct mxt_data *data = i2c_get_clientdata(client);
	int error;

	mutex_lock(&data->mutex);
	data->suspended = true;
	error = mxt_update_device(data);
	mutex_unlock(&data->mutex);

	cancel_delayed_work_sync(&data->force_calibrate_delayed_work);
	return error;
}

static int mxt_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct mxt_data *data = i2c_get_clientdata(client);
	int error;

	mutex_lock(&data->mutex);
	data->suspended = false;
	error = mxt_update_device(data);
	mutex_unlock(&data->mutex);

	return error;
}
#endif

#ifdef CONFIG_PM
static const struct dev_pm_ops mxt_pm_ops = {
#ifndef CONFIG_HAS_EARLYSUSPEND
	.suspend = mxt_suspend,
	.resume = mxt_resume,
#endif
};
#endif

#ifdef CONFIG_HAS_EARLYSUSPEND
static void mxt_early_suspend(struct early_suspend *h)
{
	struct mxt_data *data = container_of(h, struct mxt_data, early_suspend);
	mxt_suspend(&data->client->dev);
}

static void mxt_early_resume(struct early_suspend *h)
{
	struct mxt_data *data = container_of(h, struct mxt_data, early_suspend);
	mxt_resume(&data->client->dev);
}
#endif

static int __devinit mxt_probe(struct i2c_client *client,
			       const struct i2c_device_id *id)
{
	struct mxt_platform_data *pdata = client->dev.platform_data;
	struct mxt_data *data;
	int error;

	if (!pdata)
		return -EINVAL;

	data = kzalloc(sizeof(struct mxt_data), GFP_KERNEL);
	if (!data) {
		dev_err(&client->dev, "Failed to allocate memory\n");
		return -ENOMEM;
	}

	data->dbgdump = 1;
	data->client = client;
	mutex_init(&data->mutex);
	init_completion(&data->test_done);
	INIT_DELAYED_WORK(&data->force_calibrate_delayed_work,
			  mxt_force_calibrate_delayed_work);
	i2c_set_clientdata(client, data);

	data->bootloader = i2c_new_dummy(client->adapter,
					 client->addr ==
					 MXT_APP_LOW ? MXT_BOOT_LOW :
					 MXT_BOOT_HIGH);
	if (!data->bootloader) {
		dev_err(&client->dev, "Failed to new i2c dev for bootloader\n");
		error = -ENODEV;
		goto err_free_mem;
	}

	error = mxt_setup(data);
	if (error) {
		dev_err(&client->dev, "Failed to perform platform setup\n");
		goto err_unregister_bootloader;
	}

	error = gpio_request(pdata->irqpin, "mxt_intr");
	if (error) {
		dev_err(&client->dev, "Failed to request interrupt pin\n");
		goto err_teardown;
	}

	error = gpio_direction_input(pdata->irqpin);
	if (error) {
		dev_err(&client->dev, "Failed to change interrupt directon\n");
		goto err_free_intrpin;
	}

	error = mxt_initialize(data);
	if (error)
		goto err_free_intrpin;

	error = mxt_touch_setup(data);
	if (error)
		goto err_free_object;

	error = mxt_keypad_setup(data);
	if (error)
		goto err_unregister_touch;

	error = request_threaded_irq(client->irq, NULL, mxt_interrupt,
				     pdata->irqflags, client->dev.driver->name,
				     data);
	if (error) {
		dev_err(&client->dev, "Failed to register interrupt\n");
		goto err_unregister_keypad;
	}

	data->vkeys_dir = kobject_create_and_add("board_properties", NULL);
	if (data->vkeys_dir == NULL) {
		error = -ENOMEM;
		goto err_free_irq;
	}

	sysfs_attr_init(&data->vkeys_attr.attr);
	data->vkeys_attr.attr.name = "virtualkeys.mXT-touch";
	data->vkeys_attr.attr.mode = (S_IRUSR | S_IRGRP | S_IROTH);
	data->vkeys_attr.show = mxt_vkeys_show;

	error = sysfs_create_file(data->vkeys_dir, &data->vkeys_attr.attr);
	if (error) {
		dev_err(&client->dev, "Failed to create sysfs entry\n");
		goto err_put_vkeys;
	}

	error = sysfs_create_group(&client->dev.kobj, &mxt_attr_group);
	if (error)
		goto err_put_vkeys;

#ifdef CONFIG_HAS_EARLYSUSPEND
	data->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	data->early_suspend.suspend = mxt_early_suspend;
	data->early_suspend.resume = mxt_early_resume;
	register_early_suspend(&data->early_suspend);
#endif

	sysfs_bin_attr_init(&data->mem_access_attr);
	data->mem_access_attr.attr.name = "mem_access";
	data->mem_access_attr.attr.mode = S_IRUGO | S_IWUSR;
	data->mem_access_attr.read = mxt_mem_access_read;
	data->mem_access_attr.write = mxt_mem_access_write;
	data->mem_access_attr.size = data->mem_size;

	if (sysfs_create_bin_file(&client->dev.kobj,
				  &data->mem_access_attr) < 0) {
		dev_err(&client->dev, "Failed to create %s\n",
			data->mem_access_attr.attr.name);
		goto err_remove_sysfs_group;
	}

	return 0;

err_put_vkeys:
	kobject_put(data->vkeys_dir);

err_remove_sysfs_group:
	sysfs_remove_group(&client->dev.kobj, &mxt_attr_group);
err_free_irq:
	free_irq(client->irq, data);
err_unregister_keypad:
	input_unregister_device(data->input_keypad);
	kfree(data->keypad_map);
err_unregister_touch:
	input_unregister_device(data->input_touch);
err_free_object:
	kfree(data->object_table);
	kfree(data->finger);
err_free_intrpin:
	gpio_free(pdata->irqpin);
err_teardown:
	mxt_teardown(data);
err_unregister_bootloader:
	i2c_unregister_device(data->bootloader);
err_free_mem:
	kfree(data);
	return error;
}

static int __devexit mxt_remove(struct i2c_client *client)
{
	struct mxt_data *data = i2c_get_clientdata(client);
	struct mxt_platform_data *pdata = client->dev.platform_data;

#ifdef CONFIG_HAS_EARLYSUSPEND
	unregister_early_suspend(&data->early_suspend);
#endif

	sysfs_remove_bin_file(&client->dev.kobj, &data->mem_access_attr);
	sysfs_remove_group(&client->dev.kobj, &mxt_attr_group);
	cancel_delayed_work_sync(&data->force_calibrate_delayed_work);
	kobject_put(data->vkeys_dir);
	free_irq(client->irq, data);
	input_unregister_device(data->input_keypad);
	kfree(data->keypad_map);
	input_unregister_device(data->input_touch);
	kfree(data->object_table);
	kfree(data->finger);
	gpio_free(pdata->irqpin);
	mxt_teardown(data);
	i2c_unregister_device(data->bootloader);
	kfree(data);

	return 0;
}

static const struct i2c_device_id mxt_id[] = {
	{"qt602240_ts", 0},
	{"atmel_mxt_ts", 0},
	{"mXT224", 0},
	{}
};

MODULE_DEVICE_TABLE(i2c, mxt_id);

static struct i2c_driver mxt_driver = {
	.driver = {
		   .name = "atmel_mxt_ts",
		   .owner = THIS_MODULE,
#ifdef CONFIG_PM
		   .pm = &mxt_pm_ops,
#endif
		   },
	.probe = mxt_probe,
	.remove = __devexit_p(mxt_remove),
	.id_table = mxt_id,
};

static int __init mxt_init(void)
{
	return i2c_add_driver(&mxt_driver);
}

static void __exit mxt_exit(void)
{
	i2c_del_driver(&mxt_driver);
}

module_init(mxt_init);
module_exit(mxt_exit);

/* Module information */
MODULE_AUTHOR("Joonyoung Shim <jy0922.shim@samsung.com>");
MODULE_DESCRIPTION("Atmel maXTouch Touchscreen driver");
MODULE_LICENSE("GPL");
