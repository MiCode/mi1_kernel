/*
 *  Copyright (C) 2009 Samsung Electronics
 *  Minkyu Kang <mk7.kang@samsung.com>
 *  Copyright (C) 2011 XiaoMi Corporation
 *  Lin Liu <liulin@xiaomi.com>

 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __MAX17043_BATTERY_H_
#define __MAX17043_BATTERY_H_

struct max17043_platform_data {
	int (*battery_online)(void);
	int (*charger_online)(void);
	int (*charger_enable)(void);
	int (*power_supply_register)(struct device *parent,
		struct power_supply *psy);
	void (*power_supply_unregister)(struct power_supply *psy);
	int (*get_batt_soc)(void);
	int (*get_batt_mvolts)(void);
	bool ready;
	u16 rcomp_value;
	u16 bit;
	u16 soc_checkA;
	u16 soc_checkB;
	u16 unlock;
	int ocv_test;
	int temp_cold_up;
	int temp_cold_down;
	const u8 *model;
};

#endif
