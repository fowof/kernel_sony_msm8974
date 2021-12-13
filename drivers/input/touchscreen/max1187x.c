/* drivers/input/touchscreen/max1187x.c
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

#include <linux/async.h>
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/of.h>

#ifdef CONFIG_FB
#include <linux/fb.h>
#include <linux/notifier.h>
#endif

#include "max1187x_ts.h"
#include "max1187x_msg.h"

static int mxm_of_property_read_u32(struct max1187x_touchscreen * ts, const char * propname, u32 * out, u32 min, u32 max) {
  struct device * dev = &(ts->i2c.client->dev);
  struct device_node *np = dev->of_node;
  u32 val;
  if (!np)
    return -EINVAL;
  if (of_property_read_u32(np, propname, &val)) {
    dev_err(dev, "failed to read property %s from device tree\n", propname);
    return -EIO;
  }
  if ((0 < min && val < min) || (0 < max && max < val)) {
    dev_err(dev, "property %s (%u) out of range [%d,%d]\n", propname, val, min, max);
    return 1;
  }
  *out = val;
  return 0;
}

static int mxm_of_property_read_string(struct max1187x_touchscreen * ts, const char * propname, const char ** out) {
  struct device * dev = &(ts->i2c.client->dev);
  struct device_node *np = dev->of_node;
  if (!np)
    return -EINVAL;
  if (of_property_read_string(np, propname, out)) {
    dev_err(dev, "failed to read property %s from device tree\n", propname);
    return -EIO;
  }
  return 0;
}

//
//
//

static irqreturn_t irq_handler_hard(int irq, void *context)
{
	struct max1187x_touchscreen *ts = (struct max1187x_touchscreen *)context;
  int rc;

	mutex_lock(&ts->gpio.mutex);
	rc = gpio_get_value(ts->pdata->gpio_tirq) ? IRQ_HANDLED: IRQ_WAKE_THREAD;
	mutex_unlock(&ts->gpio.mutex);

	return rc;
}

static irqreturn_t irq_handler_soft(int irq, void *context)
{
	struct max1187x_touchscreen *ts = (struct max1187x_touchscreen *)context;
  struct device *dev = &(ts->i2c.client->dev);

  struct mxm_packet_buffer pkt = {MXM_OFFSET_RPT,};

  if (mxm_read_packet(ts, &pkt))
    dev_err(dev, "%s: read packet failed\n", __func__);
  else if (pkt.hdr.segs > 1)
    dev_warn(dev, "%s: received multi packet report, ignored.\n", __func__);
  else if (pkt.hdr.seq == 1 && pkt.hdr.segs == 1)
		mxm_handle_message(ts, (struct mxm_message *)&pkt.payload);
  else
    dev_err(dev, "%s: unknown error\n", __func__);

	return IRQ_HANDLED;
}

/****************************************
 *
 * MAX1187X TouchScreen
 *
 ****************************************/

static int mxm_ts_init_platdata(struct max1187x_touchscreen * ts)
{
	struct device *dev = &ts->i2c.client->dev;
	struct max1187x_pdata *pdata;
	int ret;

	/* If pdata is missing, try to get pdata from device tree (dts) */
	pdata = devm_kzalloc(dev, sizeof(*pdata), GFP_KERNEL);
	if (!pdata) {
		dev_err(dev, "failed to alloc mem for pdata\n");
		return -ENOMEM;
	}

	ret = -EPERM;
	do {
		if (mxm_of_property_read_string(ts, "touch_vdd-supply_name",
		    (const char **)&pdata->vdd_supply_name)) break;
		if (mxm_of_property_read_u32(ts, "gpio_tirq",
		    &(pdata->gpio_tirq), 1, 0)) break;
		if (mxm_of_property_read_u32(ts, "gpio_reset",
		    &(pdata->gpio_reset), 0, 0)) break;
		if (mxm_of_property_read_u32(ts, "gpio_reset_l2h",
		    &(pdata->gpio_reset_l2h), 0, 0)) break;
		if (mxm_of_property_read_u32(ts, "lcd_x",
		    &(pdata->lcd_x), MXM_LCD_X_MIN, MXM_LCD_SIZE_MAX)) break;
		if (mxm_of_property_read_u32(ts, "lcd_y",
		    &(pdata->lcd_y), MXM_LCD_Y_MIN, MXM_LCD_SIZE_MAX)) break;
		if (mxm_of_property_read_u32(ts, "num_sensor_x",
		    &(pdata->num_sensor_x), 0, MXM_NUM_SENSOR_MAX)) break;
		if (mxm_of_property_read_u32(ts, "num_sensor_y",
		    &(pdata->num_sensor_y), 0, MXM_NUM_SENSOR_MAX)) break;
		if (mxm_of_property_read_u32(ts, "swap_xy",
		    &(pdata->swap_xy), 0, 0)) break;
		if (mxm_of_property_read_u32(ts, "reverse_x",
		    &(pdata->reverse_x), 0, 0)) break;
		if (mxm_of_property_read_u32(ts, "reverse_y",
		    &(pdata->reverse_y), 0, 0)) break;
		if (mxm_of_property_read_u32(ts, "ignore_pen",
		    &(pdata->ignore_pen), 0, 0)) break;
		if (mxm_of_property_read_u32(ts, "report_type",
		    &(pdata->report_type), 0, 0)) break;
		if (mxm_of_property_read_u32(ts, "report_pressure",
		    &(pdata->report_pressure), 0, 0)) break;
		if (mxm_of_property_read_u32(ts, "report_size",
		    &(pdata->report_size), 0, 0)) break;
		if (mxm_of_property_read_u32(ts, "enable_glove",
		    &(pdata->enable_glove), 0, 0)) break;
		ret = 0;
	} while(0);

	if (ret)
		devm_kfree(dev, pdata);
	else
		ts->pdata = pdata;

	return ret;
}

static void mxm_ts_deinit_platdata(struct max1187x_touchscreen * ts)
{
	if (!IS_ERR_OR_NULL(ts->pdata)) {
		devm_kfree(&ts->i2c.client->dev, ts->pdata);
	}
	ts->pdata = NULL;
}

static int mxm_ts_init_gpio(struct max1187x_touchscreen *ts)
{
	int  rc = -EINVAL;

	mutex_init(&ts->gpio.mutex);

	mutex_lock(&ts->gpio.mutex);
	do {
		if (!gpio_is_valid(ts->pdata->gpio_tirq))
			break;
		rc = gpio_request(ts->pdata->gpio_tirq, "max1187x_tirq");
		if (rc) {
			dev_err(&ts->i2c.client->dev,
				"GPIO request failed for %s (%u)\n",
				"max1187x_tirq", ts->pdata->gpio_tirq);
			break;
		}
		rc = gpio_direction_input(ts->pdata->gpio_tirq);
		if (rc) {
			dev_err(&ts->i2c.client->dev,
				"GPIO set input direction failed for " \
				"max1187x_tirq (%d)\n", ts->pdata->gpio_tirq);
			gpio_free(ts->pdata->gpio_tirq);
			break;
		}
		if (!gpio_is_valid(ts->pdata->gpio_reset))
			break;
		rc = gpio_request(ts->pdata->gpio_reset, "max1187x_reset");
		if (rc) {
			dev_err(&ts->i2c.client->dev,
				"GPIO request failed for %s (%u)\n",
				"max1187x_reset", ts->pdata->gpio_reset);
			gpio_free(ts->pdata->gpio_tirq);
			break;
		}
		rc = gpio_direction_output(ts->pdata->gpio_reset,
							ts->pdata->gpio_reset_l2h ? 1 : 0);
		if (rc) {
			dev_err(&ts->i2c.client->dev,
				"GPIO set output direction failed for " \
				"max1187x_gpio_reset (%d)\n",
				ts->pdata->gpio_reset);
			gpio_free(ts->pdata->gpio_tirq);
			gpio_free(ts->pdata->gpio_reset);
			break;
		}
	} while(0);
	mutex_unlock(&ts->gpio.mutex);

	return rc;
}

static void mxm_ts_deinit_gpio(struct max1187x_touchscreen * ts)
{
	if (gpio_is_valid(ts->pdata->gpio_tirq))
		gpio_free(ts->pdata->gpio_tirq);
	if (gpio_is_valid(ts->pdata->gpio_reset)) {
		gpio_set_value(ts->pdata->gpio_reset,
							ts->pdata->gpio_reset_l2h ? 0: 1);
		gpio_free(ts->pdata->gpio_reset);
	}
}

static int mxm_ts_init_regulator(struct max1187x_touchscreen * ts)
{
  struct device * dev = &ts->i2c.client->dev;
	struct regulator * vreg = regulator_get(dev,
					ts->pdata->vdd_supply_name);
	int rc;

	if (IS_ERR(vreg)) {
		dev_err(dev, "%s: get vdd failed\n", __func__);
		return -ENODEV;
	}

	do {
		rc = regulator_set_voltage(vreg,
					MXM_VREG_MAX_UV, MXM_VREG_MAX_UV);
		if (rc) {
			dev_err(dev, "%s: set voltage vdd failed, rc=%d\n",
								__func__, rc);
			break;
		}
		rc = regulator_enable(vreg);
		if (rc) {
			dev_err(dev, "Failed to configure VREG");
			break;
		}
		msleep(MXM_PWR_SET_WAIT_MS);
		rc = 0;
	} while(0);

	if (rc) {
		regulator_put(vreg);
	} else {
		ts->vdd_supply = vreg;
	}

	return rc;
}

static void mxm_ts_deinit_regulator(struct max1187x_touchscreen * ts)
{
  struct device * dev = &ts->i2c.client->dev;
  struct regulator * vreg = regulator_get(dev,
          ts->pdata->vdd_supply_name);

	if (!IS_ERR_OR_NULL(vreg)) {
		regulator_set_voltage(vreg, 0, MXM_VREG_MAX_UV);
		regulator_disable(ts->vdd_supply);
		regulator_put(ts->vdd_supply);
	}
	ts->vdd_supply = NULL;
}

static int mxm_ts_init_input_device(struct max1187x_touchscreen * ts)
{
	struct i2c_client *client = ts->i2c.client;
  struct device * dev = &client->dev;
	struct input_dev *idev = input_allocate_device();
	if (!idev) {
		dev_err(dev, "Failed to allocate touch input device");
		return -ENOMEM;
	}

  snprintf(ts->phys, sizeof(ts->phys),
    "%s/input0", dev_name(dev));

	idev->name = client->driver->driver.name;
	idev->phys = ts->phys;
	idev->id.bustype = BUS_I2C;

  set_bit(EV_SYN, idev->evbit);
	set_bit(EV_ABS, idev->evbit);

	input_set_abs_params(idev, ABS_MT_TRACKING_ID,
		0, MXM_TOUCH_COUNT_MAX, 0, 0);
	input_set_abs_params(idev, ABS_MT_POSITION_X,
		0, ts->pdata->lcd_x - 1, 0, 0);
	input_set_abs_params(idev, ABS_MT_POSITION_Y,
		0, ts->pdata->lcd_y - 1, 0, 0);
	if (ts->pdata->report_type)
		input_set_abs_params(idev, ABS_MT_TOOL_TYPE,
			0, ts->pdata->ignore_pen ? MT_TOOL_FINGER : MT_TOOL_PEN, 0, 0);
	if (ts->pdata->report_pressure)
		input_set_abs_params(idev, ABS_MT_PRESSURE,
			0, 0xFFFF, 0xFF, 0);
	if (ts->pdata->report_size)
		input_set_abs_params(idev, ABS_MT_TOUCH_MAJOR,
			0, ts->pdata->num_sensor_x * ts->pdata->num_sensor_y, 0, 0);

	if (input_register_device(idev)) {
		dev_err(dev, "Failed to register touch input device");
		input_free_device(idev);
		return -EPERM;
	}
	ts->input_dev = idev;

	return 0;
}

static void mxm_ts_deinit_input_device(struct max1187x_touchscreen * ts)
{
	if (!IS_ERR_OR_NULL(ts->input_dev)) {
		input_unregister_device(ts->input_dev);
		input_free_device(ts->input_dev);
	}
	ts->input_dev = NULL;
}

static int mxm_ts_init_irq(struct max1187x_touchscreen * ts)
{
  struct i2c_client *client = ts->i2c.client;
  struct device * dev = &client->dev;

	int rc = request_threaded_irq(
		  client->irq,
			irq_handler_hard, irq_handler_soft,
			IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
			client->name, ts);
	if (rc) {
		dev_err(dev, "Failed to setup IRQ handler");
		return -EIO;
	}

	rc = -EIO;
	do {
		if (mxm_send_command(ts, MXM_CMD_RESET_SYSTEM))
		  break;
		if (mxm_send_command(ts, MXM_CMD_GET_FIRMWARE_VERSION))
		  break;
		if (mxm_send_command(ts, MXM_CMD_GET_CONFIG_INFO))
		  break;
		rc = 0;
	} while(0);

	if (rc) {
		disable_irq(client->irq);
		free_irq(client->irq, ts);
		dev_err(dev, "Failed to get response from chip.");
	}

	return rc;
}

static void mxm_ts_deinit_irq(struct max1187x_touchscreen * ts)
{
	disable_irq(ts->i2c.client->irq);
	free_irq(ts->i2c.client->irq, ts);
}

#ifdef CONFIG_FB
static int fb_notifier_callback(struct notifier_block *self,
				unsigned long event, void *data)
{
	struct fb_event *evdata = data;
	struct max1187x_touchscreen *ts;
	struct device *dev;
	int *blank;

	do {
		if (event != FB_EVENT_BLANK)
		  break;

		ts = container_of(self, struct max1187x_touchscreen, fb_notif);
		if (!(ts && ts->i2c.client))
		  break;
		dev = &ts->i2c.client->dev;

		if (!(evdata && evdata->data))
		  break;

		blank = evdata->data;
		switch (*blank) {
			case FB_BLANK_UNBLANK:
			//	 dev_dbg(&ts->i2c.client->dev, "FB_BLANK_UNBLANK\n");
				dev->driver->pm->resume(&(ts->i2c.client->dev));
				break;
			// case FB_BLANK_NORMAL:
			//   dev_dbg(&ts->i2c.client->dev, "FB_BLANK_NORMAL\n");
			//   break;
			// case FB_BLANK_VSYNC_SUSPEND:
			//   dev_dbg(&ts->i2c.client->dev, "FB_BLANK_VSYNC_SUSPEND\n");
			//   break;
			// case FB_BLANK_HSYNC_SUSPEND:
		  //   dev_dbg(&ts->i2c.client->dev, "FB_BLANK_HSYNC_SUSPEND\n");
			//   break;
			case FB_BLANK_POWERDOWN:
			//	dev_dbg(&ts->i2c.client->dev, "FB_BLANK_POWERDOWN\n");
				dev->driver->pm->suspend(&(ts->i2c.client->dev));
				break;
			// default:
			//   dev_warn(&ts->i2c.client->dev, "Unknown blank state(%d)!\n", *blank);
		}
	} while(0);

	return 0;
}

static int mxm_ts_init_notifier_callback(struct max1187x_touchscreen * ts)
{
	ts->fb_notif.notifier_call = fb_notifier_callback;
	if (fb_register_client(&ts->fb_notif)) {
		dev_err(&ts->i2c.client->dev, "Unable to register fb_notifier");
		return -EPERM;
	}
	return 0;
}

static void mxm_ts_deinit_notifier_callback(struct max1187x_touchscreen * ts)
{
	fb_unregister_client(&ts->fb_notif);
}

#endif /* CONFIG_FB */

//
//
//
//
//

struct max1187x_touchscreen * max1187x_touchscreen_create(struct i2c_client *client)
{
	struct max1187x_touchscreen * ts;

	if (IS_ERR_OR_NULL(client)) {
		dev_err(&ts->i2c.client->dev, "no i2c client.");
		return NULL;
	}

	ts = kzalloc(sizeof(struct max1187x_touchscreen), GFP_KERNEL);
	if (!ts) {
		dev_err(&ts->i2c.client->dev, "Failed to allocate control block memory");
		return NULL;
	}
	ts->i2c.client = client;
	i2c_set_clientdata(client, ts);

	return ts;
}

void max1187x_touchscreen_release(struct max1187x_touchscreen * ts)
{
	i2c_set_clientdata(ts->i2c.client, NULL);
	kzfree(ts);
}

int max1187x_touchscreen_init(struct max1187x_touchscreen * ts)
{
  struct device * dev = &ts->i2c.client->dev;
  int rc = 0;

	mutex_init(&ts->i2c.mutex);

  ts->report.wait.id = 0;
	init_waitqueue_head(&ts->report.wait.queue);
	mutex_init(&ts->report.wait.mutex);

	ts->is_suspended = false;
  ts->framecounter = 0;

	do {
		if(mxm_ts_init_platdata(ts))
			break;
		rc++;
		dev_info(dev, "(INIT): Platform data OK\n");

		if(mxm_ts_init_regulator(ts))
			break;
		rc++;
		dev_info(dev, "(INIT): VREG OK\n");

		if (mxm_ts_init_gpio(ts))
			break;
		rc++;
		dev_info(dev, "(INIT): GPIO OK\n");

    if (mxm_ts_init_irq(ts))
			break;
		rc++;
		dev_info(dev, "(INIT): IRQ handler OK\n");

		if (mxm_ts_init_input_device(ts))
			break;
		rc++;
		dev_info(dev, "(INIT): Input device OK\n");

#ifdef CONFIG_FB
		if (mxm_ts_init_notifier_callback(ts))
		  break;
		rc++;
		dev_info(dev, "(INIT): fb callback OK\n");
#endif /* CONFIG_FB */

    rc = 0;
	} while(0);

	if (rc) {
#ifdef CONFIG_FB
		if (rc > 4)
			mxm_ts_deinit_input_device(ts);
#endif /* CONFIG_FB */
    if (rc > 3)
			mxm_ts_deinit_irq(ts);
		if (rc > 2)
			mxm_ts_deinit_gpio(ts);
		if (rc > 1)
			mxm_ts_deinit_regulator(ts);
		mxm_ts_deinit_platdata(ts);
		rc = -EPERM;
	}

	return rc;
}

void max1187x_touchscreen_deinit(struct max1187x_touchscreen * ts)
{
#ifdef CONFIG_FB
	mxm_ts_deinit_notifier_callback(ts);
#endif
	mxm_ts_deinit_irq(ts);
	mxm_ts_deinit_input_device(ts);
	mxm_ts_deinit_gpio(ts);
	mxm_ts_deinit_regulator(ts);
	mxm_ts_deinit_platdata(ts);
}

//
//
//
//
//

int max1187x_touchscreen_reset(struct max1187x_touchscreen * ts)
{
	// int rc = -EINVAL;

	if (!gpio_is_valid(ts->pdata->gpio_reset)) {
		return -EINVAL;
	}

  dev_info(&ts->i2c.client->dev, "RESET\n");

	disable_irq(ts->i2c.client->irq);
	mutex_lock(&ts->gpio.mutex);
	mutex_lock(&ts->i2c.mutex);

	gpio_set_value(ts->pdata->gpio_reset,
    ts->pdata->gpio_reset_l2h ? 0 : 1);
	usleep_range(MXM_WAIT_MIN_US, MXM_WAIT_MAX_US);
	gpio_set_value(ts->pdata->gpio_reset,
				    ts->pdata->gpio_reset_l2h ? 1 : 0);
	usleep_range(MXM_CHIP_RESET_US,
		MXM_CHIP_RESET_US + MXM_WAIT_MIN_US);

	mutex_unlock(&ts->i2c.mutex);
	mutex_unlock(&ts->gpio.mutex);
	enable_irq(ts->i2c.client->irq);

	// if (rc == 0)
	// 	dev_dbg(&ts->i2c.client->dev, "%s: hard reset complete\n", __func__);
	// else
	// 	dev_err(&ts->i2c.client->dev, "hard reset failed!\n");

	return 0;
}

/****************************************
 *
 * Standard Driver Structures/Functions
 *
 ****************************************/

 static int max1187x_probe(struct i2c_client * client, const struct i2c_device_id * dev_id)
 {
  struct device * dev = &client->dev;
	struct max1187x_touchscreen * ts;
 	// struct kobject *parent;

 	dev_info(dev, "(INIT): Start");

 	/* if I2C functionality is not present we are done */
 	if (!IS_ERR_OR_NULL(client)
      && !i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
 		dev_err(dev, "I2C core driver does not support I2C functionality\n");
 		return -ENXIO;
 	}
 	dev_info(dev, "(INIT): I2C functionality OK\n");

 	/* allocate control block; nothing more to do if we can't */
	ts = max1187x_touchscreen_create(client);
 	if (!ts)
 		return -ENOMEM;
 	dev_info(dev, "(INIT): Memory allocation OK\n");

	if (max1187x_touchscreen_init(ts)) {
		max1187x_touchscreen_release(ts);
		return -EPERM;
	}

 	// /* set up debug interface */
 	// ret = create_sysfs_entries(ts);
 	// if (ret) {
 	// 	dev_err(dev, "failed to create sysfs file");
 	// 	goto err_device_init_irq;
 	// }

 	/* create symlink */
 	// parent = ts->input_dev->dev.kobj.parent;
 	if (sysfs_create_link(ts->input_dev->dev.kobj.parent,
    &client->dev.kobj, MAX1187X_NAME))
 		dev_err(dev, "sysfs_create_link error\n");

 	dev_info(dev, "(INIT): Done\n");
 	return 0;
}

static void max1187x_shutdown(struct i2c_client * client)
{
 	struct max1187x_touchscreen *ts = i2c_get_clientdata(client);

 	if (ts == NULL)
 		return;

 	// remove_sysfs_entries(ts);
 	sysfs_remove_link(ts->input_dev->dev.kobj.parent,
 						MAX1187X_NAME);

	max1187x_touchscreen_deinit(ts);
	max1187x_touchscreen_release(ts);
}

static int max1187x_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct max1187x_touchscreen *ts = i2c_get_clientdata(client);
  int rc;

	if (!ts->is_suspended) {

		if (mxm_send_command(ts, MXM_CMD_SLEEP))
			dev_err(dev, "Failed to set sleep mode\n");

		disable_irq(client->irq);
		rc = regulator_set_optimum_mode(ts->vdd_supply,
							MXM_VREG_LOAD_UA_LOW);
		usleep_range(MXM_WAIT_MIN_US, MXM_WAIT_MAX_US);

		ts->is_suspended = true;

    input_mt_sync(ts->input_dev);
    input_sync(ts->input_dev);

    dev_dbg(dev, "suspend end\n");
	}

	return 0;
}

static int max1187x_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct max1187x_touchscreen *ts = i2c_get_clientdata(client);
	int rc;

  if (ts->is_suspended) {

		rc = regulator_set_optimum_mode(ts->vdd_supply, MXM_VREG_LOAD_UA_HIGH);
		usleep_range(MXM_WAIT_MIN_US, MXM_WAIT_MAX_US);

		enable_irq(client->irq);

		if (mxm_send_command(ts, MXM_CMD_AUTO_ACTIVE))
			dev_err(dev, "Failed to wake up");
    if (mxm_send_command(ts, MXM_CMD_RESET_SYSTEM))
      dev_err(dev, "Failed to reset system");
    if (mxm_send_command(ts, MXM_CMD_RESET_BASELINE))
      dev_err(dev, "Failed to reset baseline");
		if (mxm_send_command(ts, MXM_CMD_SET_TOUCH_REPORT_EXTENDED))
			dev_err(dev, "Failed to set touch report mode");
		// if (ts->pdata->enable_glove)
		//   rc = mxm_send_command(ts, MXM_CMD_ENABLE_GLOVE);
		// else
		//   rc = mxm_send_command(ts, MXM_CMD_DISABLE_GLOVE);

    input_mt_sync(ts->input_dev);
    input_sync(ts->input_dev);

		ts->is_suspended = false;

    dev_dbg(dev, "resume end\n");
	}

	return 0;
}

//
//
//

static const struct i2c_device_id max1187x_id_table[] = {
	{MAX1187X_NAME, 0},
	{}
};
MODULE_DEVICE_TABLE(i2c, max1187x_id);

static struct of_device_id max1187x_of_match_table[] = {
	{.compatible = "maxim,max1187x_tsc"},
	{}
};

static const struct dev_pm_ops max1187x_pm_ops = {
	.resume = max1187x_resume,
	.suspend = max1187x_suspend,
};

static struct i2c_driver driver = {
	.id_table = max1187x_id_table,
  .probe = max1187x_probe,
  .shutdown = max1187x_shutdown,
	.driver = {
		.name = MAX1187X_NAME,
		.owner = THIS_MODULE,
		.of_match_table = max1187x_of_match_table,
    .pm = &max1187x_pm_ops,
	},
};

#ifndef MODULE
void __devinit max1187x_init_async(void *unused, async_cookie_t cookie)
{
	int ret;
	ret = i2c_add_driver(&driver);
	if (ret != 0)
		pr_err("Maxim I2C registration failed ret = %d\n", ret);
}
#endif

static int __devinit max1187x_init(void)
{
#ifdef MODULE
	return i2c_add_driver(&driver);
#else
	async_schedule(max1187x_init_async, NULL);
	return 0;
#endif
}

static void __exit max1187x_exit(void)
{
	i2c_del_driver(&driver);
}

module_init(max1187x_init);
module_exit(max1187x_exit);

MODULE_AUTHOR("Maxim Integrated Products, Inc. et al.");
MODULE_DESCRIPTION("Simplified MAX1187X Touchscreen Driver");
MODULE_LICENSE("GPL v2");
// MODULE_VERSION("3.3.2");
