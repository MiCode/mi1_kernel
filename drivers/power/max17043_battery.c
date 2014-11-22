/*
 *  max17043_battery.c
 *  fuel-gauge systems for lithium-ion (Li+) batteries
 *
 *  Copyright (C) 2009 Samsung Electronics
 *  Minkyu Kang <mk7.kang@samsung.com>
 *  Copyright (C) 2009 Xiaomi Corporation
 *  Lin Liu <liulin@xiaomi.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include <linux/module.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/mutex.h>
#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/power_supply.h>
#include <linux/slab.h>
#include <linux/time.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/max17043_battery.h>
#include <linux/msm-charger.h>

#define MAX17043_VCELL_MSB	0x02
#define MAX17043_VCELL_LSB	0x03
#define MAX17043_SOC_MSB	0x04
#define MAX17043_SOC_LSB	0x05
#define MAX17043_MODE_MSB	0x06
#define MAX17043_MODE_LSB	0x07
#define MAX17043_VER_MSB	0x08
#define MAX17043_VER_LSB	0x09
#define MAX17043_RCOMP_CFG	0x0C
#define MAX17043_ATHD		0x0D
#define MAX17043_CMD_MSB	0xFE
#define MAX17043_CMD_LSB	0xFF

#define BATT_TEMP_UNKNOW	-300
#define BATT_ALERT_SOC		10
#define BATT_OFF_SOC		4
/* ATHD: 0x1F means 1%, 0x00 means 32% */
#define BATT_ALERT_ATHD		(32 - BATT_ALERT_SOC)

struct max17043_chip {
	struct i2c_client *client;
	struct max17043_platform_data *pdata;
	struct timespec next_update_time;
	struct wake_lock wlock;
	/* battery voltage */
	int vcell;
	/* battery capacity */
	int soc;
	/* the lastest soc */
	int last_soc;
	/* battery alert threshold */
	int athd;
	/* battery temperature last */
	int last_temp;
	int model_loaded;
	struct mutex max17043_lock;
};

static struct delayed_work bootup_work;
static bool system_is_bootup = true;
static u8 vcell_msb, vcell_lsb;	/* Used as the base adjustment */
/* Save soc each time it's read out */
static int ocv_saved;

struct max17043_chip *batt_chip;
static int max17043_load_model(struct max17043_chip *chip);
static irqreturn_t max17043_battery_short(int irq, void *dev);
extern int msm_charger_update_heartbeat(void);

static int max17043_read_reg(struct i2c_client *client, int reg)
{
	int ret;

	ret = i2c_smbus_read_byte_data(client, reg);

	if (ret < 0)
		dev_err(&client->dev, "%s: err %d\n", __func__, ret);

	return ret;
}

static void max17043_set_athd(struct max17043_chip *chip, int psoc)
{
	int soc;
	u8 msb, lsb, rcomp;
	u16 config;

	if (psoc == 0) {
		msb = max17043_read_reg(chip->client, MAX17043_SOC_MSB);
		lsb = max17043_read_reg(chip->client, MAX17043_SOC_LSB);
		soc = ((msb * 256 + lsb) / 512);
	} else {
		soc = psoc;
	}

	/* The alert window is between 0% and 16% */
	if (soc <= 0)
		soc = 1;
	if (soc > 16)
		soc = 16;

	rcomp = max17043_read_reg(chip->client, MAX17043_RCOMP_CFG);

	/* set battery short alert to 30%/2 or current soc - 2 */
	config = rcomp | ((32 - soc * 2) << 8);

	/* clear ATHD and set new ATHD value */
	i2c_smbus_write_word_data(chip->client, MAX17043_RCOMP_CFG, config);
	chip->athd = soc;
	pr_info("max17043 set athd:0x%04x soc:%d\n", config, soc);
}

static void max17043_get_soc_local(struct i2c_client *client)
{
	struct max17043_chip *chip = i2c_get_clientdata(client);
	struct timespec now;
	static int vcell_min;
	int soc;
	int charging = battery_charging();
	u8 msb, lsb;

	ktime_get_ts(&now);
	monotonic_to_bootbased(&now);
	if (timespec_compare(&now, &chip->next_update_time) < 0)
		return;

	/* Verify if gauce ic is workable */
	if (max17043_read_reg(client, MAX17043_SOC_MSB) < 0) {
		pr_err("%s Gauge IC error! Force soc to 1\n", __func__);
		chip->soc = 1;
		chip->vcell = 0;
		return;
	}

	if (!chip->model_loaded) {
		if (max17043_load_model(chip) < 0)
			pr_warn("%s load model ERROR\n", __func__);
	}

	msb = max17043_read_reg(client, MAX17043_SOC_MSB);
	lsb = max17043_read_reg(client, MAX17043_SOC_LSB);
	soc = ((msb * 256 + lsb) / 512);
	chip->soc = soc > 100 ? 100 : soc;

	msb = max17043_read_reg(client, MAX17043_VCELL_MSB);
	lsb = max17043_read_reg(client, MAX17043_VCELL_LSB);
	chip->vcell = ((msb * 256 + lsb) * 5) / 4 / 16;

	/* Avoid reporting soc=0 during bootup */
	if (system_is_bootup && soc == 0 && chip->vcell > 3000 && !charging)
		chip->soc = 1;

	/* Shut down if voltage is kept below for a while */
	if (!system_is_bootup && !charging && chip->vcell < 3400)
		vcell_min++;
	else
		vcell_min = 0;

	/* soc is zero if voltage is below 3400mV for a while */
	if (!system_is_bootup
	    && !charging && chip->vcell < 3400 && vcell_min >= 5) {
		pr_info("max17043 voltage is low\n");
		chip->soc = 0;
	}

	/* Report zero if soc is below 3 and voltage is below 3500 */
	if (!system_is_bootup
	    && !charging && chip->vcell < 3500 && chip->soc < 4) {
		pr_info("max17043 soc is low:%d\n", chip->soc);
		chip->soc = 0;
	}

	/* Avoid soc is up when discharging */
	if (!charging
	    && !system_is_bootup
	    && chip->last_soc >= 0 && chip->last_soc < chip->soc)
		chip->soc = chip->last_soc;
	else
		chip->last_soc = chip->soc;

	/* wake lock */
	if (chip->soc == 0)
		wake_lock_timeout(&chip->wlock, 20 * HZ);

	/* Update battery short alarm */
	if ((chip->soc <= BATT_ALERT_SOC) && (chip->soc > 0)
	    && (abs(chip->soc - chip->athd) >= 1)) {
		pr_info("max17043 update new athd\n");
		max17043_set_athd(chip, chip->soc);
	}

	/* next update must be at least 1 second later */
	ktime_get_ts(&chip->next_update_time);
	monotonic_to_bootbased(&chip->next_update_time);
	chip->next_update_time.tv_sec++;

	if (!chip->model_loaded) {
		if (request_threaded_irq(client->irq, NULL,
					 max17043_battery_short,
					 IRQF_TRIGGER_LOW | IRQF_ONESHOT,
					 "max17043", chip) < 0)
			dev_err(&client->dev, "fail to request irq.");
		chip->model_loaded = 1;
		irq_set_irq_wake(client->irq, 1);
	}

	pr_info("max17043 soc:%d %d v:%d\n", soc, chip->soc, chip->vcell);
}

void max17043_temperature_compensation(int temp)
{
	struct max17043_chip *chip;
	struct max17043_platform_data *pdata;
	u16 config;
	u8 athd;
	int ncomp;

	if (batt_chip == NULL) {
		pr_warn("Fuel Gauge Not Ready!\n");
		return;
	}
	chip = batt_chip;
	pdata = chip->pdata;

	if (chip->last_temp == BATT_TEMP_UNKNOW)
		chip->last_temp = temp;
	else if (abs(temp - chip->last_temp) < 2)
		return;
	else
		chip->last_temp = temp;

	/* Update RCOMP */

	if (temp > 20)
		ncomp = pdata->rcomp_value +
		    (((temp - 20) * pdata->temp_cold_up) / 1000);
	else if (temp < 20)
		ncomp = pdata->rcomp_value +
		    (((temp - 20) * pdata->temp_cold_down) / 1000);
	else
		ncomp = pdata->rcomp_value;

	if (ncomp > 255)
		ncomp = 255;
	else if (ncomp < 0)
		ncomp = 0;

	athd = max17043_read_reg(chip->client, MAX17043_ATHD);
	/* set next alert threshold and clear alert and sleep bit */
	config = ncomp | athd << 8;
	i2c_smbus_write_word_data(chip->client, MAX17043_RCOMP_CFG, config);

	pr_info("max17043 compensation temp:%d config:0x%04x\n", temp, config);
}

EXPORT_SYMBOL(max17043_temperature_compensation);

int max17043_get_batt_soc(void)
{
	struct max17043_chip *chip;

	chip = batt_chip;

	if (batt_chip == NULL) {
		pr_warn("Gauge IC Not Ready!\n");
		return 1;
	}

	/* 17043 is suspended */
	if (!chip->pdata->ready)
		return chip->soc;

	mutex_lock(&chip->max17043_lock);
	max17043_get_soc_local(chip->client);
	mutex_unlock(&chip->max17043_lock);

	return chip->soc;
}

EXPORT_SYMBOL(max17043_get_batt_soc);

int max17043_get_batt_mvolts(void)
{
	struct max17043_chip *chip;

	chip = batt_chip;

	if (batt_chip == NULL) {
		pr_warn("Fuel Gauge Not Ready!\n");
		return 0;
	}

	/* 17043 is suspended */
	if (!chip->pdata->ready)
		return chip->vcell;

	mutex_lock(&chip->max17043_lock);
	max17043_get_soc_local(chip->client);
	mutex_unlock(&chip->max17043_lock);

	return chip->vcell;
}

EXPORT_SYMBOL(max17043_get_batt_mvolts);

static int max17043_get_version(struct i2c_client *client)
{
	int ret;
	u8 msb;
	u8 lsb;

	msb = ret = max17043_read_reg(client, MAX17043_VER_MSB);
	lsb = max17043_read_reg(client, MAX17043_VER_LSB);

	dev_info(&client->dev, "max17043 Fuel-Gauge Ver %d%d\n", msb, lsb);
	return ret;
}

/* thread interrupt handler */
static irqreturn_t max17043_battery_short(int irq, void *dev)
{
	struct max17043_chip *chip;
	int i;

	chip = (struct max17043_chip *)dev;

	/* make resume update battery capacity */
	wake_lock_timeout(&chip->wlock, HZ);
	pr_info("max17043 irq \n");

	for (i = 0; i < 5; i++) {
		if (chip->pdata->ready)
			break;
		else
			msleep(100);
	}
	if (!chip->pdata->ready) {
		pr_err("max17043 battery short irq not handled\n");
	} else {
		msm_charger_update_heartbeat();
		max17043_set_athd(chip, 0);
	}

	return IRQ_HANDLED;
}

static int max17043_load_model(struct max17043_chip *chip)
{
	int i, ret;
	struct max17043_platform_data *pdata;
	struct i2c_client *client;
	u8 rcomp1, rcomp2, ocv1, ocv2, lsb, msb, vlsb_tmp, vmsb_tmp;
	u16 soc, ocv, vcell, v_tmp;
	int changed = 0;

	pdata = chip->pdata;
	client = chip->client;

	/* The vcell is read during loading model */
	vmsb_tmp = max17043_read_reg(client, MAX17043_VCELL_MSB);
	vlsb_tmp = max17043_read_reg(client, MAX17043_VCELL_LSB);
	v_tmp = ((vmsb_tmp * 256 + vlsb_tmp) * 5) / 4 / 16;

	/* Unlock Model Access */
	i2c_smbus_write_word_data(client, 0x3E, swab16(pdata->unlock));
	/* Read original RCOMP and OCV */
	rcomp1 = max17043_read_reg(chip->client, 0x0C);
	rcomp2 = max17043_read_reg(chip->client, 0x0D);
	ocv1 = max17043_read_reg(chip->client, 0x0E);
	ocv2 = max17043_read_reg(chip->client, 0x0F);
	pr_info("max17043 load_model 0x%02x 0x%02x 0x%02x 0x%02x\n",
		rcomp1, rcomp2, ocv1, ocv2);
	/* Write OCV Test Value */
	i2c_smbus_write_word_data(client, 0x0E, swab16(pdata->ocv_test));
	/* Write RCOMP to a Maximum value */
	i2c_smbus_write_word_data(client, 0x0C, 0x00FF);
	/* Write the Model */
	for (i = 0; i < 64; i += 2) {
		soc = pdata->model[i] | pdata->model[i + 1] << 8;
		i2c_smbus_write_word_data(chip->client, 0x40 + i, soc);
	}
	msleep(160);
	/* Write OCV Test Value */
	i2c_smbus_write_word_data(client, 0x0E, swab16(pdata->ocv_test));
	/* Wait for verification */
	msleep(160);
	/* Read SOC */
	msb = max17043_read_reg(chip->client, 0x04);
	lsb = max17043_read_reg(chip->client, 0x05);
	soc = msb << 8 | lsb;
	if (msb >= pdata->soc_checkA && msb <= pdata->soc_checkB)
		ret = 0;
	else
		ret = -1;

	pr_info("max17043 soc:0x%2x 0x%2x in 0x%x,0x%x chk:%d\n",
		msb, lsb, pdata->soc_checkA, pdata->soc_checkB, ret);

	/* The vcell during initialization */
	vcell = ((vcell_msb * 256 + vcell_msb) * 5) / 4 / 16;
	ocv = ((ocv1 * 256 + ocv2) * 5) / 4 / 16;
	ocv_saved = ocv;

	pr_info("max17043 vcell:%d v_tmp:%d ocv:%d\n", vcell, v_tmp, ocv);

	/* Average(v_tmp, vcell) */
	if (!battery_charging() && v_tmp > vcell) {
		vcell = v_tmp;
		vcell_msb = vmsb_tmp;
		vcell_lsb = vlsb_tmp;
	}
	/* The difference is based on the experiment */
	if ((ocv > (vcell + 200)) || (vcell > (ocv + 250))) {
		/* voltage compensation */
		ocv1 = vcell_msb + 2;
		ocv2 = vcell_lsb;
		ocv_saved = vcell + 40;
		changed = 1;
	}

	/* Restore RCOMP and OCV */
	i2c_smbus_write_word_data(client, 0x0C, (rcomp1 | rcomp2 << 8));
	i2c_smbus_write_word_data(client, 0x0E, (ocv1 | ocv2 << 8));
	/* Lock Model Access */
	i2c_smbus_write_word_data(client, 0x3E, 0x0000);
	if (changed)
		/* wait 500ms for next operation */
		msleep(500);
	else
		msleep(200);
	return ret;
}

static void max17043_bootup(struct work_struct *work)
{
	pr_info("%s\n", __func__);
	system_is_bootup = false;
}

static int __devinit max17043_probe(struct i2c_client *client,
				    const struct i2c_device_id *id)
{
	int ret;
	struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);
	struct max17043_chip *chip;

	if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_BYTE))
		return -EIO;

	chip = kzalloc(sizeof(*chip), GFP_KERNEL);
	if (!chip)
		return -ENOMEM;

	batt_chip = chip;
	chip->client = client;
	chip->pdata = client->dev.platform_data;
	mutex_init(&chip->max17043_lock);

	i2c_set_clientdata(client, chip);

	ret = max17043_get_version(client);
	if (ret < 0) {
		ret = -ENODEV;
		goto err;
	}

	vcell_msb = max17043_read_reg(chip->client, 0x02);
	vcell_lsb = max17043_read_reg(chip->client, 0x03);

	chip->last_temp = BATT_TEMP_UNKNOW;
	chip->model_loaded = 0;

	/* next update must be at least 1 second later */
	ktime_get_ts(&chip->next_update_time);
	monotonic_to_bootbased(&chip->next_update_time);

	wake_lock_init(&chip->wlock, WAKE_LOCK_SUSPEND, "batt_short");
	INIT_DELAYED_WORK(&bootup_work, max17043_bootup);
	schedule_delayed_work(&bootup_work,
			      round_jiffies_relative(msecs_to_jiffies(65000)));

	chip->pdata->ready = true;

	return 0;
err:
	batt_chip = NULL;
	kfree(chip);
	return ret;
}

static int __devexit max17043_remove(struct i2c_client *client)
{
	struct max17043_chip *chip = i2c_get_clientdata(client);

	batt_chip = NULL;
	free_irq(client->irq, chip);
	wake_lock_destroy(&chip->wlock);
	kfree(chip);
	return 0;
}

#ifdef CONFIG_PM

static int max17043_suspend(struct i2c_client *client, pm_message_t state)
{
	struct max17043_chip *chip = i2c_get_clientdata(client);

	chip->pdata->ready = false;
	pr_debug("max17043 suspend\n");
	return 0;
}

static int max17043_resume(struct i2c_client *client)
{
	struct max17043_chip *chip = i2c_get_clientdata(client);

	chip->pdata->ready = true;
	pr_debug("max17043 resume\n");
	return 0;
}

#else

#define max17040_suspend NULL
#define max17040_resume NULL

#endif /* CONFIG_PM */

static const struct i2c_device_id max17043_id[] = {
	{"max17043", 0},
	{}
};

MODULE_DEVICE_TABLE(i2c, max17043_id);

static struct i2c_driver max17043_i2c_driver = {
	.driver = {
		   .name = "max17043",
		   },
	.probe = max17043_probe,
	.remove = __devexit_p(max17043_remove),
	.suspend = max17043_suspend,
	.resume = max17043_resume,
	.id_table = max17043_id,
};

static int __init max17043_init(void)
{
	return i2c_add_driver(&max17043_i2c_driver);
}

late_initcall(max17043_init);

static void __exit max17043_exit(void)
{
	i2c_del_driver(&max17043_i2c_driver);
}

module_exit(max17043_exit);

MODULE_AUTHOR("Liu Lin <liulin@xiaomi.com>");
MODULE_DESCRIPTION("MAX17043 Fuel Gauge");
MODULE_LICENSE("GPL");
