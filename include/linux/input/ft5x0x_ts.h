/*
 * Copyright (C) 2011-2014 XiaoMi, Inc.
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

#ifndef LINUX_FT5X0X_TS_H
#define LINUX_FT5X0X_TS_H

#include <linux/types.h>

/* platform data for Focaltech touchscreen */
struct ft5x0x_firmware_data {
	u8        vendor;
	const u8 *data;
	int       size;
};

struct ft5x0x_rect { /* rectangle on the touch screen */
	u16 left , top;
	u16 width, height;
};

struct ft5x0x_keypad_data {
	/* two cases could happen:
	   1.if length == 0, disable keypad functionality.
	   2.else convert touch in kparea to key event. */
	unsigned int              length; /* for keymap and button */
	const unsigned int       *keymap; /* scancode==>keycode map */
	const struct ft5x0x_rect *button; /* define button location */
};

struct ft5x0x_platform_data {
	const struct ft5x0x_firmware_data *firmware; /* terminated by 0 size */
	const struct ft5x0x_keypad_data   *keypad;
	const struct ft5x0x_rect           tcharea; /* exclude keypad area */
	unsigned long                      resetpin;
	unsigned long                      landing_jiffies;
	int                                landing_threshold;
	int                                staying_threshold;
	int                                moving_threshold;
	/* optional callback for platform needs */
	int (*setup)(struct ft5x0x_platform_data *pdata);
	int (*teardown)(struct ft5x0x_platform_data *pdata);
	void *context;
};

#endif /* LINUX_FT5X0X_TS_H */
