/* drivers/input/touchscreen/max1187x_ts.h
 *
 * Copyright (c) 2013 Maxim Integrated Products, Inc.
 * Copyright (c) 2013-2014 Sony Mobile Communications AB.
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

#ifndef __MAX1187X_TS_H
#define __MAX1187X_TS_H

#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/mutex.h>
#include <linux/regulator/consumer.h>
#include <linux/types.h>
#include <linux/wait.h>

#include <linux/input/max1187x.h>

#ifdef CONFIG_FB
#include <linux/notifier.h>
#endif

#define MXM_TOUCH_COUNT_MAX             10

#define MXM_VREG_LOAD_UA_LOW          2000
#define MXM_VREG_LOAD_UA_HIGH        15000
#define MXM_VREG_MAX_UV            3000000

#define MXM_LCD_X_MIN                  480
#define MXM_LCD_Y_MIN                  240
#define MXM_LCD_SIZE_MAX            0x7FFF

#define MXM_NUM_SENSOR_MAX              40

#define MXM_WAIT_MIN_US               1000
#define MXM_WAIT_MAX_US               2000
#define MXM_PWR_SET_WAIT_MS            100
#define MXM_CHIP_RESET_US             6000
#define MXM_RPT_WAIT_TICKS        (HZ >> 2)

struct max1187x_touchscreen {
	struct max1187x_pdata *pdata;
	struct input_dev *input_dev;
	char phys[32];

#ifdef CONFIG_FB
	struct notifier_block fb_notif;
#endif

	bool is_suspended;

	struct regulator *vdd_supply;

  struct {
		struct i2c_client * client;
		struct mutex mutex;
	} i2c;

	struct {
		struct mutex mutex;
	} gpio;

	struct {
		u16 status;
		u16 power_mode;
		u16 config_id;
		u8 chip_id;
		struct {
			u8 major;
			u8 minor;
			u16 revision;
//			char branch[4];
		} fwver;
	} system;

	struct {
		struct {
			u16 id;
			wait_queue_head_t queue;
			struct mutex      mutex;
		} wait;
    // struct {
    //   u16 seq;
    //   u16 segs;
    //   u16 * head;
    //   struct mxm_message msg;
    // } buf;
	} report;

	u16 framecounter;
};

#endif /* __MAX1187X_TS_H */
