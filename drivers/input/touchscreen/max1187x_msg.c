/* drivers/input/touchscreen/max1187x_msg.c
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

#include "max1187x_msg.h"

#include <asm/byteorder.h>
#include <linux/input/mt.h>
#include <linux/jiffies.h>

#define MXM_RPT_PKTPL_MAX_WLEN 245
#define MXM_CMD_PKTPL_MAX_WLEN 255

//
//
//

static int mxm_i2c_rx_words(struct max1187x_touchscreen *ts, u16 *buf, u16 len)
{
	int i, rc;
	struct device *dev = &ts->i2c.client->dev;

	do {
		rc = i2c_master_recv(ts->i2c.client,
			(char *)buf, (int)(len * 2));
	} while (rc == -EAGAIN);

	if (rc < 0) {
		dev_err(dev, "%s: failed with error code %d.\n", __func__, rc);
		return rc;
	} else if (rc % 2) {
		dev_err(dev, "%s: odd number of bytes (%d)\n", __func__, rc);
		return -EIO;
	}
	rc /= 2;

	for (i = 0; i < len; i++)
		buf[i] = le16_to_cpu(buf[i]);

	if (rc != len) {
		dev_err(dev, "%s: failed to rx specified length (%d/%d)\n", __func__, rc, len);
		return -EIO;
	}

	return 0;
}

static int mxm_i2c_tx_words(struct max1187x_touchscreen *ts, u16 *buf, u16 len)
{
	int i, rc;
	struct device *dev = &ts->i2c.client->dev;

	for (i = 0; i < len; i++)
		buf[i] = cpu_to_le16(buf[i]);

	do {
		rc = i2c_master_send(ts->i2c.client,
			(char *)buf, (int)(2 * len));
	} while (rc == -EAGAIN);

	if (rc < 0) {
		dev_err(dev, "%s: failed with error code %d\n", __func__, rc);
		return rc;
	} else if (rc % 2) {
		dev_err(dev, "%s: odd number of bytes (%d)\n", __func__, rc);
		return -EIO;
	}
	rc /= 2;

	if (rc != len) {
		dev_err(dev, "%s: failed to tx specified length (%d/%d)\n", __func__, rc, len);
		return -EIO;
	}

	return 0;
}

//
//
//

static int mxm_i2c_read_words(struct max1187x_touchscreen *ts, u16 *buf, u16 addr, u16 len)
{
	if (mxm_i2c_tx_words(ts, &addr, 1))
		return -EIO;
  if (mxm_i2c_rx_words(ts, buf, len))
		return -EIO;
  return 0;
}

//
//
//

int mxm_read_packet(struct max1187x_touchscreen *ts, struct mxm_packet_buffer * pkt)
{
  struct device *dev = &ts->i2c.client->dev;
	int rc = -EIO;

	mutex_lock(&ts->i2c.mutex);
  do {
    if (mxm_i2c_read_words(ts, (u16 *)&(pkt->hdr), pkt->offset, 1))
      break;
    if (pkt->hdr.len > MXM_RPT_PKTPL_MAX_WLEN) {
      dev_err(dev, "%s: too long rpt payload (%d/%d)\n", __func__,
        pkt->hdr.len, MXM_RPT_PKTPL_MAX_WLEN);
			rc = -EINVAL;
      break;
    }
    rc = mxm_i2c_read_words(ts,
			(u16 *)&(pkt->payload), pkt->offset+1, pkt->hdr.len);
  } while(0);
  mutex_unlock(&ts->i2c.mutex);

	return rc;
}

int mxm_send_packet(struct max1187x_touchscreen *ts, struct mxm_packet_buffer *pkt)
{
	int rc;
	mutex_lock(&ts->i2c.mutex);
  rc = mxm_i2c_tx_words(ts, (u16 *)pkt, pkt->hdr.len+2);
  mutex_unlock(&ts->i2c.mutex);
	return rc;
}

//
//
//

int mxm_send_command(struct max1187x_touchscreen * ts, enum mxm_command cmd)
{
  struct {
  	u16 addr;
    struct mxm_packet_header pkthdr;
    struct mxm_message_header msghdr;
    u16 value[2];
  } buf = {
    MXM_OFFSET_CMD,
    {2,1,1},
    {0,0},
    {0xFFFF,0xFFFF}
  };
	u16 reply_id = 0;
  int len;
  int rc = -EINVAL;

  switch(cmd){
    case MXM_CMD_GET_FIRMWARE_VERSION:
      buf.msghdr.id   = MXM_CMD_ID_GET_FIRMWARE_VERSION;
      reply_id        = MXM_RPT_ID_FIRMWARE_VERSION;
      break;
    case MXM_CMD_GET_CONFIG_INFO:
      buf.msghdr.id   = MXM_CMD_ID_GET_CONFIG_INFO;
      reply_id        = MXM_RPT_ID_CONFIG_INFO;
      break;
		case MXM_CMD_SET_TOUCH_REPORT_EXTENDED:
      buf.msghdr.id   = MXM_CMD_ID_SET_TOUCH_RPT_MODE;
      buf.value[0]    = MXM_TOUCH_REPORT_MODE_EXT;
      break;
    case MXM_CMD_RESET_SYSTEM:
      buf.msghdr.id   = MXM_CMD_ID_RESET_SYSTEM;
			reply_id        = MXM_RPT_ID_SYSTEM_STATUS;
      break;
    case MXM_CMD_SLEEP:
      buf.msghdr.id   = MXM_CMD_ID_SET_POWER_MODE;
      buf.value[0]    = MXM_PWR_SLEEP;
      break;
    case MXM_CMD_AUTO_ACTIVE:
      buf.msghdr.id   = MXM_CMD_ID_SET_POWER_MODE;
      buf.value[0]    = MXM_PWR_AUTO_ACTIVE;
      break;
		case MXM_CMD_RESET_BASELINE:
			buf.msghdr.id   = MXM_CMD_ID_SET_BASELINE;
			buf.value[0]    = 0;
			reply_id        = MXM_RPT_ID_BASELINE;
		  break;
		case MXM_CMD_ENABLE_GLOVE:
			buf.msghdr.id   = MXM_CMD_ID_SET_GLOVE_MODE;
			buf.value[0]    = 0;
			buf.value[1]    = 1;
		  break;
		case MXM_CMD_DISABLE_GLOVE:
			buf.msghdr.id   = MXM_CMD_ID_SET_GLOVE_MODE;
			buf.value[0]    = 0;
			buf.value[1]    = 0;
		  break;
    default:
      dev_err(&ts->i2c.client->dev, "%s: unknown command! (0x%04X)\n",
			__func__, cmd);
      return rc;
  }

	buf.msghdr.size = buf.value[0] != 0xFFFF ? (buf.value[1] != 0xFFFF ? 2: 1): 0;
  buf.pkthdr.len  = buf.msghdr.size + MXM_MSG_HDR_WLEN;
  len             = buf.pkthdr.len  + MXM_PKT_HDR_WLEN + 1;

  if (reply_id)
    mutex_lock(&ts->report.wait.mutex);

	mutex_lock(&ts->i2c.mutex);
  rc = mxm_i2c_tx_words(ts, (u16 *)&buf, len);
	ts->report.wait.id = reply_id;
  mutex_unlock(&ts->i2c.mutex);

  if (reply_id) {
		if (wait_event_interruptible_timeout(ts->report.wait.queue,
				ts->report.wait.id == 0, MXM_RPT_WAIT_TICKS) == 0)
		  dev_warn(&ts->i2c.client->dev,
				"%s: wait reply timeout! (cmd: 0x%04X, reply: 0x%04X)\n",
			  __func__, buf.msghdr.id, reply_id);
		ts->report.wait.id = 0;
    mutex_unlock(&ts->report.wait.mutex);
  }

	return rc;
}

//
//
//
//
//

static void handle_system_status(struct max1187x_touchscreen * ts, struct mxm_system_status * status)
{
	//if (status->value != ts->system.status)
		dev_info(&(ts->i2c.client->dev), "system status: 0x%04X -> 0x%04X\n",
			ts->system.status, status->value);
	ts->system.status = status->value;
}

static void handle_power_mode(struct max1187x_touchscreen * ts, struct mxm_power_mode * pwr)
{
	//if (pwr->value != ts->system.power_mode)
		dev_info(&(ts->i2c.client->dev), "power mode: 0x%04X -> 0x%04X\n",
			ts->system.power_mode, pwr->value);
	ts->system.power_mode = pwr->value;
}

static void handle_firmware_version(struct max1187x_touchscreen * ts, struct mxm_firmware_version * fwver)
{
	dev_info(&(ts->i2c.client->dev), "chip id: 0x%02X\n",
	  fwver->chip_id);
	dev_info(&(ts->i2c.client->dev), "fw ver: %u.%u.%u\n",
		fwver->major, fwver->minor, fwver->revision);

	ts->system.chip_id = fwver->chip_id;
	ts->system.fwver.major = fwver->major;
	ts->system.fwver.minor = fwver->minor;
	ts->system.fwver.revision = fwver->revision;
	// snprintf(ts->system.fwver.branch, 3,
	//   "p%u%c", fwver->branch_num, fwver->branch_char);
}

static void handle_config_info(struct max1187x_touchscreen * ts, struct mxm_config_info * cfg)
{
  ts->system.config_id = cfg->id;
	dev_info(&(ts->i2c.client->dev), "config id: 0x%04X\n", ts->system.config_id);
}

static void handle_touch(struct max1187x_touchscreen * ts, struct mxm_touch_info_extended * info)
{
  struct input_dev *idev = ts->input_dev;
	struct device * dev = &ts->i2c.client->dev;
	struct mxm_touch_extended * e = info->body;
  u16 x, y, tool_type;
  int i, n_touch=0;

	if (ts->is_suspended) {
		dev_warn(dev, "receive touch report while system is suspended.\n");
	}
  if (idev->users == 0) {
    dev_warn(dev, "no user.");
    return;
  }
  if (info->header.framecounter <= ts->framecounter) {
    dev_dbg(dev, "framecounter decremented! (prev: %u, recv: %u)\n",
		  ts->framecounter, info->header.framecounter);
  }
	ts->framecounter = info->header.framecounter;

  for (i = 0; i < info->header.touch_count; i++,e++) {
  	tool_type = e->tool_type;
    switch (tool_type) {
  		case MXM_TOOL_PEN:
  			if (ts->pdata->ignore_pen)
  				continue;
  			tool_type = MT_TOOL_PEN;
  			break;
  		case MXM_TOOL_GLOVE:
				continue;
  		case MXM_TOOL_FINGER:
  		  tool_type = MT_TOOL_FINGER;
  			break;
      default:
  		  dev_warn(dev, "Unknown tool type(%u)!\n", tool_type);
        continue;
    }
    n_touch++;

  	if (ts->pdata->swap_xy)
		  {x = e->y; y = e->x;}
		else
		  {x = e->x; y = e->y;}
  	if (ts->pdata->reverse_x)
  		x = ts->pdata->lcd_x - 1 - x;
  	if (ts->pdata->reverse_y)
  		y = ts->pdata->lcd_y - 1 - y;

		input_report_abs(idev, ABS_MT_TRACKING_ID, e->finger_id);
		if (ts->pdata->report_type)
		  input_report_abs(idev, ABS_MT_TOOL_TYPE, tool_type);
		input_report_abs(idev, ABS_MT_POSITION_X, x);
		input_report_abs(idev, ABS_MT_POSITION_Y, y);
		if (ts->pdata->report_pressure)
			input_report_abs(idev, ABS_MT_PRESSURE, e->z);
		if (ts->pdata->report_size)
			input_report_abs(idev, ABS_MT_TOUCH_MAJOR, e->area);
		input_mt_sync(idev);

		// dev_info(dev, "XYZA[%u][%d]=(%4u, %4u, %04X, %4u)\n",
		//            info->header.framecounter, i, x, y, e->z, e->area);
  }

  if (n_touch == 0)
    input_mt_sync(idev);
  input_sync(idev);
}

void mxm_handle_message(struct max1187x_touchscreen * ts, struct mxm_message * msg)
{
  switch (msg->hdr.id) {
    case MXM_RPT_ID_CONFIG_INFO:
      handle_config_info(ts,
        (struct mxm_config_info *)&(msg->payload));
      break;
    case MXM_RPT_ID_POWER_MODE:
      handle_power_mode(ts,
        (struct mxm_power_mode *)&(msg->payload));
      break;
    case MXM_RPT_ID_FIRMWARE_VERSION:
      handle_firmware_version(ts,
        (struct mxm_firmware_version *)&(msg->payload));
      break;
    case MXM_RPT_ID_SYSTEM_STATUS:
      handle_system_status(ts,
        (struct mxm_system_status *)&(msg->payload));
      break;
    case MXM_RPT_ID_TOUCH_INFO_EXTENDED:
      handle_touch(ts,
        (struct mxm_touch_info_extended *)&(msg->payload));
      break;
		case MXM_RPT_ID_BASELINE:
		  break;
		case MXM_RPT_ID_BAD_COMMAND:
			dev_err(&ts->i2c.client->dev,
				"%s: received BAD_COMMAND error.\n", __func__);
			break;
    default:
      dev_warn(&ts->i2c.client->dev,
				"%s: unhandled message (id=0x%04X, len=%u, val[0]=0x%04X).\n",
        __func__, msg->hdr.id, msg->hdr.size,
				msg->hdr.size ? msg->payload[0]: 0);
  }

  if (msg->hdr.id == ts->report.wait.id
		  || msg->hdr.id == MXM_RPT_ID_BAD_COMMAND) {
    wake_up_interruptible(&ts->report.wait.queue);
		ts->report.wait.id = 0;
	}
}

int mxm_wait_report(struct max1187x_touchscreen *ts, u16 id)
{
	int rc;
  ts->report.wait.id = id;
	rc = wait_event_interruptible_timeout(ts->report.wait.queue,
		     ts->report.wait.id == 0, usecs_to_jiffies(MXM_WAIT_MAX_US));
  if (rc == 0)
    dev_err(&ts->i2c.client->dev, "%s: wait reply timeout! (reply = 0x%04X)\n",
      __func__, id);
	return rc;
}
