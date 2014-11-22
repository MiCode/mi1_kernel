/**
 *
 * Synaptics Register Mapped Interface (RMI4) I2C Physical Layer Driver.
 * Copyright (c) 2007-2010, Synaptics Incorporated
 *
 * Author: Js HA <js.ha@stericsson.com> for ST-Ericsson
 * Author: Naveen Kumar G <naveen.gaddipati@stericsson.com> for ST-Ericsson
 * Copyright 2010 (c) ST-Ericsson AB
 */
/*
 * This file is licensed under the GPL2 license.
 *
 *#############################################################################
 * GPL
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published
 * by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
 * or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License
 * for more details.
 *
 *#############################################################################
 */

#ifndef _SYNAPTICS_RMI4_H_INCLUDED_
#define _SYNAPTICS_RMI4_H_INCLUDED_

struct synaptics_rmi4_rect { /* rectangle on the touch screen */
	u16 left , top;
	u16 width, height;
};

struct synaptics_rmi4_keypad_data {
	/* two cases could happen:
	   1.if length == 0, disable keypad functionality.
	   2.else convert touch in kparea to key event. */
	unsigned int                      length; /* for keymap and button */
	const unsigned int               *keymap; /* scancode==>keycode map */
	const struct synaptics_rmi4_rect *button; /* define button location */
	const struct synaptics_rmi4_rect  kparea; /* valid keypad area */
};

struct synaptics_rmi4_firmware_data {
	const unsigned char *data;
	int                  size;
};

/**
 * struct synaptics_rmi4_platform_data - contains the rmi4 platform data
 * @irq_number: irq number
 * @irq_type: irq type
 * @x flip: x flip flag
 * @y flip: y flip flag
 *
 * This structure gives platform data for rmi4.
 */

struct i2c_client; /* forward declaration */

struct synaptics_rmi4_platform_data {
	int irq_number;
	int irq_type;
	int irq_pin;
	bool x_flip;
	bool y_flip;
	bool regulator_en;
	int sensor_max_x;
	int sensor_max_y;
	int moving_threshold;
	int staying_threshold;
	int landing_threshold;
	unsigned long landing_jiffies;
	const struct synaptics_rmi4_rect tcharea;
	const struct synaptics_rmi4_keypad_data *keypad;
	const struct synaptics_rmi4_firmware_data *firmware;
	/* optional callback for platform needs */
	int (*setup)(struct i2c_client *client,
			struct synaptics_rmi4_platform_data *pdata);
	int (*teardown)(struct i2c_client *client,
			struct synaptics_rmi4_platform_data *pdata);
	void *context;
};

#endif
