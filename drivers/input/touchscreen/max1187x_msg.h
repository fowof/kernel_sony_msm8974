/* drivers/input/touchscreen/max1187x_msg.h
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

#ifndef __MAX1187X_MSG_H
#define __MAX1187X_MSG_H

#include <linux/types.h>

#include "max1187x_ts.h"

#define MXM_OFFSET_CMD       0x0000
#define MXM_OFFSET_RPT       0x000A

#define MXM_PKT_HDR_WLEN 1
#define MXM_MSG_HDR_WLEN 2

#define MXM_MSG_MAX_WLEN     1024
#define MXM_MSGPL_MAX_WLEN   (MXM_MSG_MAX_WLEN - MXM_MSG_HDR_WLEN)

struct mxm_packet_header {
	u16 len:8;    // payload length
	u16 seq:4;		// sequence number in segments
	u16 segs:4;   // total segments
}; // 1 ward

struct mxm_packet_buffer {
	u16 offset;
	struct mxm_packet_header hdr;
	u16 payload[255];
}; // 257 words

struct mxm_message_header {
  u16 id;
  u16 size;
}; // 2 words

struct mxm_message {
  struct mxm_message_header hdr;
  u16 payload[MXM_MSGPL_MAX_WLEN];
};

#define	MXM_CMD_ID_GET_CONFIG_INFO         0x0002
#define	MXM_CMD_ID_GET_PRIVATE_CONFIG      0x0004
#define	MXM_CMD_ID_GET_CALIB_TABLE         0x0011
#define	MXM_CMD_ID_SET_TOUCH_RPT_MODE      0x0018
#define	MXM_CMD_ID_SET_POWER_MODE          0x0020
#define	MXM_CMD_ID_SET_TOUCH_FARME         0x0026
#define	MXM_CMD_ID_SET_BASELINE_MODE       0x0028
#define MXM_CMD_ID_GET_LOOKUP_TABLE        0x0031
#define	MXM_CMD_ID_SET_BASELINE            0x0034
#define	MXM_CMD_ID_GET_FIRMWARE_VERSION    0x0040
#define	MXM_CMD_ID_SET_IMAGE_FACTOR_DATA   0x0046
#define	MXM_CMD_ID_GET_IMAGE_FACTOR_TABLE  0x0047
#define	MXM_CMD_ID_SET_GLOVE_MODE          0x0083
#define	MXM_CMD_ID_SET_COVER_MODE          0x0088
#define	MXM_CMD_ID_SET_EDGE_FILTER         0x0089
#define	MXM_CMD_ID_RESET_SYSTEM            0x00E9

#define	MXM_RPT_ID_CONFIG_INFO             0x0102
#define	MXM_RPT_ID_PRIVATE_CONFIG          0x0104
#define	MXM_RPT_ID_CALIB_TABLE             0x0111
#define	MXM_RPT_ID_POWER_MODE              0x0121
#define	MXM_RPT_ID_BASELINE                0x0134
#define	MXM_RPT_ID_FIRMWARE_VERSION        0x0140
#define	MXM_RPT_ID_IMAGE_FACTOR_DATA       0x0146
#define	MXM_RPT_ID_IMAGE_FACTOR_TABLE      0x0147
#define	MXM_RPT_ID_SYSTEM_STATUS           0x01A0
#define	MXM_RPT_ID_TOUCH_RAW_IMAGE         0x0800
#define	MXM_RPT_ID_TOUCH_INFO_BASIC        0x0801
#define	MXM_RPT_ID_TOUCH_INFO_EXTENDED     0x0802
#define	MXM_RPT_ID_BAD_COMMAND             0xBADC

#define MXM_TOUCH_REPORT_MODE_RAW          0x0000
#define MXM_TOUCH_REPORT_MODE_BSC          0x0001
#define MXM_TOUCH_REPORT_MODE_EXT          0x0002

#define	MXM_PWR_SLEEP                      0x0000
#define	MXM_PWR_AUTO_ACTIVE                0x0002
#define	MXM_PWR_FORCE_ACTIVE               0x0003
#define	MXM_PWR_DRIVE                      0x0004
#define	MXM_PWR_SENSE                      0x0005
#define MXM_PWR_WAKEUP                     0x0006

#define	MXM_STATUS_FAIL_CMD                0x0001
#define	MXM_STATUS_POWER_RESET             0x0008
#define	MXM_STATUS_EXT_RESET               0x0010
#define	MXM_STATUS_WD_RESET                0x0020
#define	MXM_STATUS_SOFT_RESET              0x0040

#define MXM_TOOL_PEN                       0x0001
#define MXM_TOOL_FINGER                    0x0002
#define MXM_TOOL_GLOVE                     0x0003

#define MXM_BASELINE_MODE_NONE             0x0000
#define MXM_BASELINE_MODE_FIXED            0x0001
#define MXM_BASELINE_MODE_AUTO             0x0002

struct mxm_power_mode {
  u16 value;
};

struct mxm_system_status {
	u16 value;
};

struct mxm_touch_header {
	u16 touch_count:4;
	u16 touch_status:4;
	u16 reserved0:8;
	u16 reserved1;
	u16 framecounter;
}; // 3 words

struct mxm_touch_extended {
	u16 finger_id:4;
	u16 reserved0:4;
	u16 tool_type:4;
	u16 reserved1:4;
	u16 x:12;
	u16 reserved2:4;
	u16 y:12;
	u16 reserved3:4;
	u16 z;
	s16 xspeed;
	s16 yspeed;
	s16 xpixel:8;
	s16 ypixel:8;
	u16 area;
	u16 xmin;
	u16 xmax;
	u16 ymin;
	u16 ymax;
}; // 12 words

struct mxm_touch_info_extended {
	struct mxm_touch_header   header;
	struct mxm_touch_extended body[MXM_TOUCH_COUNT_MAX];
};

struct mxm_firmware_version {
  u16 minor:8;
  u16 major:8;
  u16 branch_char:6;
	u16 branch_num:2;
  u16 chip_id:8;
  u16 revision;
};

struct mxm_config_info {
  u16 id;
};

enum mxm_command {
	MXM_CMD_NONE = 0,
  MXM_CMD_GET_FIRMWARE_VERSION,
	MXM_CMD_GET_CONFIG_INFO,
	MXM_CMD_SET_TOUCH_REPORT_EXTENDED,
	MXM_CMD_AUTO_ACTIVE,
	MXM_CMD_SLEEP,
	MXM_CMD_RESET_SYSTEM,
	MXM_CMD_RESET_BASELINE,
	MXM_CMD_ENABLE_GLOVE,
	MXM_CMD_DISABLE_GLOVE,
	MXM_CMD_SET_BASELINE_MODE_AUTO,
	MXM_CMD_MAX,
};

// struct max1187x_touchscreen;

int mxm_read_packet(struct max1187x_touchscreen *ts,
	struct mxm_packet_buffer * pkt);

int mxm_send_packet(struct max1187x_touchscreen *ts,
	struct mxm_packet_buffer *pkt);

int mxm_send_command(struct max1187x_touchscreen * ts,
  enum mxm_command cmd);

void mxm_handle_message(struct max1187x_touchscreen * ts,
  struct mxm_message * msg);

int mxm_wait_report(struct max1187x_touchscreen *ts, u16 id);

#endif
