/* include/linux/input/max1187x.h
 *
 * Copyright (c)2013 Maxim Integrated Products, Inc.
 * Copyright (C) 2013-2014 Sony Mobile Communications AB.
 * Copyright (c) 2021 fowof.
 *
 * Driver Version: x.x.x
 * Release Date: 2021-00-00T00:00:00Z
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

#ifndef __MAX1187X_H
#define __MAX1187X_H

#define MAX1187X_NAME   "max1187x"

struct max1187x_pdata {
	char *vdd_supply_name;

	u32	gpio_tirq;
	u32	gpio_reset;
	u32	gpio_reset_l2h;

	u32	lcd_x;
	u32	lcd_y;

	u32	num_sensor_x;
	u32	num_sensor_y;

  u32 swap_xy;
	u32 reverse_x;
	u32 reverse_y;

	u32	report_type;
	u32	report_pressure;
	u32	report_size;
	u32 ignore_pen;
	u32	enable_glove;
};

#endif /* __MAX1187X_H */
