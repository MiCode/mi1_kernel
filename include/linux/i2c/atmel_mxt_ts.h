/*
 * Atmel maXTouch Touchscreen driver
 *
 * Copyright (C) 2010 Samsung Electronics Co.Ltd
 * Author: Joonyoung Shim <jy0922.shim@samsung.com>
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#ifndef __LINUX_ATMEL_MXT_TS_H
#define __LINUX_ATMEL_MXT_TS_H

#include <linux/types.h>

/* The platform data for the Atmel maXTouch touchscreen driver */
struct mxt_object_data {
	u8        type;
	u16       size;
	const u8 *data;
};

struct mxt_config_data {
	const char *vendor; /* touch screen identifier */
	const u8 checksum[3]; /* checksum for config */
	const struct mxt_object_data *object; /* terminated by zero size */
};

struct mxt_rect { /* rectangle on the touch screen */
	u16 left , top;
	u16 width, height;
};

struct mxt_keypad_data {
	/* three cases could happen:
	   1.if length == 0, disable keypad functionality.
	   2.else if button == NULL, use key array built-in object.
	   3.otherwise, convert touch in kparea to key. */
	bool                   repeat; /* enable key repeat */
	unsigned int           length; /* for keymap and button */
	const unsigned int    *keymap; /* scancode==>keycode map */
	const struct mxt_rect *button; /* define button location */
	const struct mxt_rect  kparea; /* valid keypad area */
};

struct i2c_client; /* forward declaration */

struct mxt_platform_data {
	const struct mxt_config_data *config; /* terminated by null vendor */
	const struct mxt_keypad_data *keypad;
	const struct mxt_rect         tcharea; /* exclude keypad area */
	unsigned long                 irqpin;
	unsigned long                 irqflags;
	/* optional callback for platform needs */
	int (*setup)(struct i2c_client *client,
			struct mxt_platform_data *pdata);
	int (*teardown)(struct i2c_client *client,
			struct mxt_platform_data *pdata);
	void *context;
};

#endif /* __LINUX_ATMEL_MXT_TS_H */
