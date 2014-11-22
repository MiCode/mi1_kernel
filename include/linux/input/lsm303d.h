/* include/linux/input/lsm303d.h
 *
 * Copyright (C) 2012-2014 XiaoMi, Inc.
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

#ifndef LINUX_INPUT_LSM303D_H
#define LINUX_INPUT_LSM303D_H

struct i2c_client; /* forward declaration */

struct lsm303d_platform_data {
	/* for magnetometer sensor */
	int mag_xdir, mag_ydir, mag_zdir; /* can be +1 or -1 */
	int mag_xcode, mag_ycode, mag_zcode; /* can be ABS_? */
	unsigned long mag_period; /* default sample period */
	unsigned long mag_scale; /* 2/4/8/12Ga, 0 mean auto scale */
	u16 mag_lowthres; /* threshold for switch scale value */
	u16 mag_highthres; /* threshold for switch scale value */
	/* for accelerometer sensor */
	int acc_xdir, acc_ydir, acc_zdir; /* can be +1 or -1 */
	int acc_xcode, acc_ycode, acc_zcode; /* can be ABS_? */
	unsigned long acc_period; /* default sample period */
	unsigned long acc_scale; /* 2/4/8/16G, 0 mean auto scale */
	u16 acc_lowthres; /* threshold for switch scale value */
	u16 acc_highthres; /* threshold for switch scale value */
};

#endif
