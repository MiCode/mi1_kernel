/*
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
#include <linux/device.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/rfkill.h>
#include <asm/gpio.h>
#include <asm/mach-types.h>
#include <linux/skbuff.h>

#include <mach/gpiomux.h>
#include "board-xiaomi.h"

#include <linux/mfd/pmic8058.h>

static struct rfkill *bt_rfk;
static const char bt_name[] = "bcm4329";

static int bluetooth_set_power(void *data, bool blocked)
{
	if (!blocked) {
		gpio_direction_output(XIAOMI_GPIO_BT_SHUTDOWN_N, 1);
		gpio_direction_output(XIAOMI_GPIO_BT_RESET_N, 1);
		msleep(200);
	} else {
		gpio_direction_output(XIAOMI_GPIO_BT_SHUTDOWN_N, 0);
		gpio_direction_output(XIAOMI_GPIO_BT_RESET_N, 0);
	}

	return 0;
}

static struct rfkill_ops xiaomi_rfkill_ops = {
	.set_block = bluetooth_set_power,
};

static int xiaomi_rfkill_probe(struct platform_device *pdev)
{
	int rc = 0;
	bool default_state = true;	/* off */

	rc = gpio_request(XIAOMI_GPIO_BT_SHUTDOWN_N, "bt_shutdown");
	if (rc) {
		printk(" %s:%d BT_SHUTDOWN request failed\n", __func__,
		       __LINE__);
		goto err_gpio_shutdown;
	}

	rc = gpio_request(XIAOMI_GPIO_BT_RESET_N, "bt_reset");
	if (rc) {
		printk(" %s:%d BT_RESET request failed\n", __func__, __LINE__);
		goto err_gpio_reset;
	}

	bluetooth_set_power(NULL, default_state);

	bt_rfk = rfkill_alloc(bt_name, &pdev->dev, RFKILL_TYPE_BLUETOOTH,
			      &xiaomi_rfkill_ops, NULL);
	if (!bt_rfk) {
		rc = -ENOMEM;
		goto err_rfkill_alloc;
	}

	rfkill_set_states(bt_rfk, default_state, false);

	/* userspace cannot take exclusive control */

	rc = rfkill_register(bt_rfk);
	if (rc)
		goto err_rfkill_reg;

	return 0;

err_rfkill_reg:
	rfkill_destroy(bt_rfk);
err_rfkill_alloc:
	gpio_free(XIAOMI_GPIO_BT_SHUTDOWN_N);
err_gpio_shutdown:
	gpio_free(XIAOMI_GPIO_BT_RESET_N);
err_gpio_reset:
	return rc;
}

static int xiaomi_rfkill_remove(struct platform_device *dev)
{
	rfkill_unregister(bt_rfk);
	rfkill_destroy(bt_rfk);
	gpio_free(XIAOMI_GPIO_BT_SHUTDOWN_N);
	gpio_free(XIAOMI_GPIO_BT_RESET_N);

	return 0;
}

static struct platform_driver xiaomi_rfkill_driver = {
	.probe = xiaomi_rfkill_probe,
	.remove = xiaomi_rfkill_remove,
	.driver = {
		   .name = "xiaomi_rfkill",
		   .owner = THIS_MODULE,
		   },
};

static int __init xiaomi_rfkill_init(void)
{
	return platform_driver_register(&xiaomi_rfkill_driver);
}

static void __exit xiaomi_rfkill_exit(void)
{
	platform_driver_unregister(&xiaomi_rfkill_driver);
}

module_init(xiaomi_rfkill_init);
module_exit(xiaomi_rfkill_exit);
MODULE_DESCRIPTION("xiaomi rfkill");
MODULE_AUTHOR("Nick Pelly <npelly@google.com>");
MODULE_LICENSE("GPL");
