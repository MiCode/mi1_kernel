/* include/linux/input/lis3dh.h
 *
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

#ifndef LINUX_INPUT_LIS3DH_H
#define LINUX_INPUT_LIS3DH_H

struct i2c_client; /* forward declaration */

struct lis3dh_platform_data {
	int xdir, ydir, zdir; /* can be +1 or -1 */
	int xcode, ycode, zcode; /* can be ABS_? */
	unsigned long period; /* default sample period */
	unsigned long scale; /* 2/4/8/16G, 0 mean auto scale */
	/* configuration for 6d detection */
	unsigned long thres_6d; /* the smallest value to recognize */
	u8 count_6d; /* count to stay in the new position */
	/* threshold for switch scale value */
	u16 low_thres;
	u16 high_thres;
	/* optional callback for platform needs */
	int (*setup)(struct i2c_client *client,
			struct lis3dh_platform_data *pdata);
	int (*teardown)(struct i2c_client *client,
			struct lis3dh_platform_data *pdata);
	void *context;
};

#endif
