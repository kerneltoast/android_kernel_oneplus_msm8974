/*
 * Synaptics DSX touchscreen driver
 *
 * Copyright (C) 2012 Synaptics Incorporated
 *
 * Copyright (C) 2012 Alexandra Chin <alexandra.chin@tw.synaptics.com>
 * Copyright (C) 2012 Scott Lin <scott.lin@tw.synaptics.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#define pr_fmt(fmt) "synaptics-rmi-ts: " fmt

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/gpio.h>
#include <linux/regulator/consumer.h>
#include <linux/of_gpio.h>
#include <linux/rtc.h>
#include <linux/fb.h>
#include <linux/proc_fs.h>
#include <linux/input/mt.h>

#include "synaptics_dsx.h"
#include "synaptics_dsx_i2c.h"
#include "synaptics_test_rawdata_14001.h"

char *tp_firmware_strings[TP_TYPE_MAX][LCD_TYPE_MAX] = {
	{"WINTEK", "WINTEK", "WINTEK"},
	{"TPK", "TPK", "TPK"}
};

#define RPT_TYPE	(1 << 0)
#define RPT_X_LSB	(1 << 1)
#define RPT_X_MSB	(1 << 2)
#define RPT_Y_LSB	(1 << 3)
#define RPT_Y_MSB	(1 << 4)
#define RPT_Z		(1 << 5)
#define RPT_WX		(1 << 6)
#define RPT_WY		(1 << 7)
#define RPT_DEFAULT	(RPT_TYPE | RPT_X_LSB | RPT_X_MSB | RPT_Y_LSB | RPT_Y_MSB)

#define SYN_I2C_RETRY_TIMES 10
#define MAX_F11_TOUCH_WIDTH 15

#define CHECK_STATUS_TIMEOUT_MS 100

#define F01_STD_QUERY_LEN	21
#define F01_BUID_ID_OFFSET	18
#define F11_STD_QUERY_LEN	9
#define F11_STD_CTRL_LEN	10
#define F11_STD_DATA_LEN	12

#define STATUS_NO_ERROR			0x00
#define STATUS_RESET_OCCURRED		0x01
#define STATUS_INVALID_CONFIG		0x02
#define STATUS_DEVICE_FAILURE		0x03
#define STATUS_CONFIG_CRC_FAILURE	0x04
#define STATUS_FIRMWARE_CRC_FAILURE	0x05
#define STATUS_CRC_IN_PROGRESS		0x06

#define NORMAL_OPERATION	(0 << 0)
#define SENSOR_SLEEP		(1 << 0)
#define NO_SLEEP_OFF		(0 << 2)
#define NO_SLEEP_ON		(1 << 2)
#define CONFIGURED		(1 << 7)

static int synaptics_rmi4_i2c_read(struct synaptics_rmi4_data *rmi4_data,
		unsigned short addr, unsigned char *data,
		unsigned short length);

static int synaptics_rmi4_i2c_write(struct synaptics_rmi4_data *rmi4_data,
		unsigned short addr, unsigned char *data,
		unsigned short length);

static int synaptics_rmi4_f12_set_enables(struct synaptics_rmi4_data *rmi4_data,
		unsigned short ctrl28);

static void synaptics_rmi4_reinit_device(struct synaptics_rmi4_data *rmi4_data);
static void synaptics_rmi4_reset_device(struct synaptics_rmi4_data *rmi4_data,
		unsigned short f01_cmd_base_addr);

struct synaptics_rmi4_f01_device_status {
	union {
		struct {
			unsigned char status_code:4;
			unsigned char reserved:2;
			unsigned char flash_prog:1;
			unsigned char unconfigured:1;
		} __packed;
		unsigned char data[1];
	};
};

struct synaptics_rmi4_f12_query_5 {
	union {
		struct {
			unsigned char size_of_query6;
			struct {
				unsigned char ctrl0_is_present:1;
				unsigned char ctrl1_is_present:1;
				unsigned char ctrl2_is_present:1;
				unsigned char ctrl3_is_present:1;
				unsigned char ctrl4_is_present:1;
				unsigned char ctrl5_is_present:1;
				unsigned char ctrl6_is_present:1;
				unsigned char ctrl7_is_present:1;
			} __packed;
			struct {
				unsigned char ctrl8_is_present:1;
				unsigned char ctrl9_is_present:1;
				unsigned char ctrl10_is_present:1;
				unsigned char ctrl11_is_present:1;
				unsigned char ctrl12_is_present:1;
				unsigned char ctrl13_is_present:1;
				unsigned char ctrl14_is_present:1;
				unsigned char ctrl15_is_present:1;
			} __packed;
			struct {
				unsigned char ctrl16_is_present:1;
				unsigned char ctrl17_is_present:1;
				unsigned char ctrl18_is_present:1;
				unsigned char ctrl19_is_present:1;
				unsigned char ctrl20_is_present:1;
				unsigned char ctrl21_is_present:1;
				unsigned char ctrl22_is_present:1;
				unsigned char ctrl23_is_present:1;
			} __packed;
			struct {
				unsigned char ctrl24_is_present:1;
				unsigned char ctrl25_is_present:1;
				unsigned char ctrl26_is_present:1;
				unsigned char ctrl27_is_present:1;
				unsigned char ctrl28_is_present:1;
				unsigned char ctrl29_is_present:1;
				unsigned char ctrl30_is_present:1;
				unsigned char ctrl31_is_present:1;
			} __packed;
		};
		unsigned char data[5];
	};
};

struct synaptics_rmi4_f12_query_8 {
	union {
		struct {
			unsigned char size_of_query9;
			struct {
				unsigned char data0_is_present:1;
				unsigned char data1_is_present:1;
				unsigned char data2_is_present:1;
				unsigned char data3_is_present:1;
				unsigned char data4_is_present:1;
				unsigned char data5_is_present:1;
				unsigned char data6_is_present:1;
				unsigned char data7_is_present:1;
			} __packed;
			struct {
				unsigned char data8_is_present:1;
				unsigned char data9_is_present:1;
				unsigned char data10_is_present:1;
				unsigned char data11_is_present:1;
				unsigned char data12_is_present:1;
				unsigned char data13_is_present:1;
				unsigned char data14_is_present:1;
				unsigned char data15_is_present:1;
			} __packed;
		};
		unsigned char data[3];
	};
};

struct synaptics_rmi4_f12_ctrl_8 {
	union {
		struct {
			unsigned char max_x_coord_lsb;
			unsigned char max_x_coord_msb;
			unsigned char max_y_coord_lsb;
			unsigned char max_y_coord_msb;
			unsigned char rx_pitch_lsb;
			unsigned char rx_pitch_msb;
			unsigned char tx_pitch_lsb;
			unsigned char tx_pitch_msb;
			unsigned char low_rx_clip;
			unsigned char high_rx_clip;
			unsigned char low_tx_clip;
			unsigned char high_tx_clip;
			unsigned char num_of_rx;
			unsigned char num_of_tx;
		};
		unsigned char data[14];
	};
};

struct synaptics_rmi4_f12_ctrl_23 {
	union {
		struct {
			unsigned char obj_type_enable;
			unsigned char max_reported_objects;
		};
		unsigned char data[2];
	};
};

struct synaptics_rmi4_f12_finger_data {
	unsigned char object_type_and_status;
	unsigned char x_lsb;
	unsigned char x_msb;
	unsigned char y_lsb;
	unsigned char y_msb;
	unsigned char z;
	unsigned char wx;
	unsigned char wy;
};

struct synaptics_rmi4_exp_fn {
	enum exp_fn fn_type;
	bool inserted;
	int (*func_init)(struct synaptics_rmi4_data *rmi4_data);
	void (*func_remove)(struct synaptics_rmi4_data *rmi4_data);
	void (*func_attn)(struct synaptics_rmi4_data *rmi4_data,
			unsigned char intr_mask);
	struct list_head link;
};

struct synaptics_rmi4_exp_fn_data {
	bool initialized;
	bool queue_work;
	struct mutex mutex;
	struct list_head list;
	struct delayed_work work;
	struct workqueue_struct *workqueue;
	struct synaptics_rmi4_data *rmi4_data;
};

static struct synaptics_rmi4_exp_fn_data exp_data;


/**
 * synaptics_rmi4_set_page()
 *
 * Called by synaptics_rmi4_i2c_read() and synaptics_rmi4_i2c_write().
 *
 * This function writes to the page select register to switch to the
 * assigned page.
 */
static int synaptics_rmi4_set_page(struct synaptics_rmi4_data *rmi4_data,
		unsigned int address)
{
	int ret = 0, rc;
	unsigned char retry;
	unsigned char buf[PAGE_SELECT_LEN];
	unsigned char page;
	struct i2c_client *i2c = rmi4_data->i2c_client;

	page = ((address >> 8) & MASK_8BIT);
	if (page != rmi4_data->current_page) {
		buf[0] = MASK_8BIT;
		buf[1] = page;
		for (retry = 0; retry < SYN_I2C_RETRY_TIMES; retry++) {
			rc = i2c_master_send(i2c, buf, PAGE_SELECT_LEN);
			if (rc != PAGE_SELECT_LEN) {
				msleep(20);
			} else {
				rmi4_data->current_page = page;
				break;
			}
		}
		if (retry == SYN_I2C_RETRY_TIMES) {
			dev_err(&rmi4_data->i2c_client->dev,
					"%s: I2C read over retry limit\n",
					__func__);
			synaptics_rmi4_reset_device(rmi4_data,rmi4_data->f01_cmd_base_addr);
			ret = -EIO;
		} else {
			rmi4_data->reset_count = 0;
		}

	}

	return ret;
}

/**
 * synaptics_rmi4_i2c_read()
 *
 * Called by various functions in this driver, and also exported to
 * other expansion Function modules such as rmi_dev.
 *
 * This function reads data of an arbitrary length from the sensor,
 * starting from an assigned register address of the sensor, via I2C
 * with a retry mechanism.
 */
static int synaptics_rmi4_i2c_read(struct synaptics_rmi4_data *rmi4_data,
		unsigned short addr, unsigned char *data, unsigned short length)
{
	int ret;
	unsigned char retry;
	unsigned char buf;
	struct i2c_msg msg[] = {
		{
			.addr = rmi4_data->i2c_client->addr,
			.flags = 0,
			.len = 1,
			.buf = &buf,
		},
		{
			.addr = rmi4_data->i2c_client->addr,
			.flags = I2C_M_RD,
			.len = length,
			.buf = data,
		},
	};

	buf = addr & MASK_8BIT;

	mutex_lock(&rmi4_data->rmi4_io_ctrl_mutex);

	ret = synaptics_rmi4_set_page(rmi4_data, addr);
	if (ret)
		goto exit;

	for (retry = 0; retry < SYN_I2C_RETRY_TIMES; retry++) {
		if (i2c_transfer(rmi4_data->i2c_client->adapter, msg, 2) == 2)
			break;

		msleep(20);
	}

	if (retry == SYN_I2C_RETRY_TIMES) {
		dev_err(&rmi4_data->i2c_client->dev,
				"%s: I2C read over retry limit\n",
				__func__);
		rmi4_data->current_page = MASK_8BIT;
		ret = -EIO;
	}

exit:
	mutex_unlock(&rmi4_data->rmi4_io_ctrl_mutex);

	return ret;
}

/**
 * synaptics_rmi4_i2c_write()
 *
 * Called by various functions in this driver, and also exported to
 * other expansion Function modules such as rmi_dev.
 *
 * This function writes data of an arbitrary length to the sensor,
 * starting from an assigned register address of the sensor, via I2C with
 * a retry mechanism.
 */
static int synaptics_rmi4_i2c_write(struct synaptics_rmi4_data *rmi4_data,
		unsigned short addr, unsigned char *data, unsigned short length)
{
	int ret;
	unsigned char retry;
	unsigned char buf[length + 1];
	struct i2c_msg msg[] = {
		{
			.addr = rmi4_data->i2c_client->addr,
			.flags = 0,
			.len = length + 1,
			.buf = buf,
		}
	};

	mutex_lock(&rmi4_data->rmi4_io_ctrl_mutex);

	ret = synaptics_rmi4_set_page(rmi4_data, addr);
	if (ret)
		goto exit;

	buf[0] = addr & MASK_8BIT;
	memcpy(&buf[1], &data[0], length);

	for (retry = 0; retry < SYN_I2C_RETRY_TIMES; retry++) {
		if (i2c_transfer(rmi4_data->i2c_client->adapter, msg, 1) == 1)
			break;

		msleep(20);
	}

	if (retry == SYN_I2C_RETRY_TIMES) {
		dev_err(&rmi4_data->i2c_client->dev,
				"%s: I2C write over retry limit\n",
				__func__);
		rmi4_data->current_page = MASK_8BIT;
		ret = -EIO;
	}

exit:
	mutex_unlock(&rmi4_data->rmi4_io_ctrl_mutex);

	return ret;
}

/* Analog voltage @3.0 V */
#define SYNA_VTG_MIN_UV		3000000
#define SYNA_VTG_MAX_UV		3000000
#define SYNA_ACTIVE_LOAD_UA	15000
#define SYNA_LPM_LOAD_UA	10

/* i2c voltage @1.8 V */
#define SYNA_I2C_VTG_MIN_UV	1800000
#define SYNA_I2C_VTG_MAX_UV	1800000
#define SYNA_I2C_LOAD_UA	10000
#define SYNA_I2C_LPM_LOAD_UA	10

#define SYNA_NO_GESTURE			0x00
#define SYNA_ONE_FINGER_DOUBLE_TAP	0x03
#define SYNA_TWO_FINGER_SWIPE		0x07
#define SYNA_ONE_FINGER_CIRCLE		0x08
#define SYNA_ONE_FINGER_DIRECTION	0x0a
#define SYNA_ONE_FINGER_W_OR_M		0x0b

#define KEY_F3	61
#define KEY_F4	62
#define KEY_F5	63
#define KEY_F6	64
#define KEY_F7	65
#define KEY_F8	66
#define KEY_F9	67

#define UnknownGesture      0
#define DouTap              1   // double tap
#define UpVee               2   // V
#define DownVee             3   // ^
#define LeftVee             4   // >
#define RightVee            5   // <
#define Circle              6   // O
#define DouSwip             7   // ||
#define Left2RightSwip      8   // -->
#define Right2LeftSwip      9   // <--
#define Up2DownSwip         10  // |v
#define Down2UpSwip         11  // |^
#define Mgestrue            12  // M
#define Wgestrue            13  // W

#define SYNA_ADDR_REPORT_FLAG        0x1b  //report mode register
#define SYNA_ADDR_GESTURE_FLAG       0x20  //gesture enable register
#define SYNA_ADDR_GESTURE_OFFSET     0x08  //gesture register addr=0x08
#define SYNA_ADDR_GESTURE_EXT        0x402  //gesture ext data
#define SYNA_ADDR_TOUCH_FEATURE      0x1E  //ThreeD Touch Features
#define SYNA_ADDR_F12_2D_CTRL23      0x1D
#define SYNA_ADDR_F12_2D_CTRL10      0x16

#define SYNA_GESTURE_DELAY_MS 1500 // time to wait in between gestures

extern int rmi4_fw_module_init(bool insert);

static struct synaptics_rmi4_data *syna_rmi4_data;
static struct regulator *vdd_regulator;
static struct regulator *vdd_regulator_i2c;
static char synaptics_vendor_str[32];
static unsigned int syna_lcd_ratio1;
static unsigned int syna_lcd_ratio2;

/***** For virtual key definition begin *******************/
enum tp_vkey_enum {
	TP_VKEY_MENU,
	TP_VKEY_HOME,
	TP_VKEY_BACK,
	TP_VKEY_NONE,
	TP_VKEY_COUNT = TP_VKEY_NONE,
};

static struct tp_vkey_button {
	int x;
	int y;
	int width;
	int height;
} vkey_buttons[TP_VKEY_COUNT];

#define LCD_SENSOR_X  (1080)
#define LCD_SENSOR_Y  (2040)
#define LCD_MAX_X  (1080)
#define LCD_MAX_Y  (1920)
#define LCD_MULTI_RATIO(m)   (((syna_lcd_ratio1)*(m))/(syna_lcd_ratio2))
#define VK_LCD_WIDTH  LCD_MULTI_RATIO(LCD_MAX_X/TP_VKEY_COUNT)   // 3 keys
static void vk_calculate_area(void)  //added by liujun
{
	struct synaptics_rmi4_data *syna_ts_data = syna_rmi4_data;
	int i;
	int tp_max_x = syna_ts_data->sensor_max_x;
	int tp_max_y = syna_ts_data->sensor_max_y;
	int vk_height = syna_ts_data->virtual_key_height;
	int vk_width = tp_max_x/TP_VKEY_COUNT;
	int margin_x = 85;

	syna_ts_data->vk_prop_width = LCD_MULTI_RATIO(190);
	syna_ts_data->vk_prop_center_y = LCD_MULTI_RATIO(1974);
	syna_ts_data->vk_prop_height = LCD_MULTI_RATIO(120);

	for (i = 0; i < TP_VKEY_COUNT; ++i) {
		vkey_buttons[i].width = vk_width - margin_x*2;
		vkey_buttons[i].height = vk_height - 10;
		vkey_buttons[i].x = vk_width*i + margin_x;
		vkey_buttons[i].y = tp_max_y - vkey_buttons[i].height;
	}
	vkey_buttons[TP_VKEY_BACK].x += 20;
}

static ssize_t vk_syna_show(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf)
{
	struct synaptics_rmi4_data *ts = syna_rmi4_data;
	int len;
	len = sprintf(buf,
		__stringify(EV_KEY) ":" __stringify(KEY_MENU)  ":%d:%d:%d:%d"
		":" __stringify(EV_KEY) ":" __stringify(KEY_HOMEPAGE)  ":%d:%d:%d:%d"
		":" __stringify(EV_KEY) ":" __stringify(KEY_BACK)  ":%d:%d:%d:%d" "\n",
		VK_LCD_WIDTH/2 + 0,   ts->vk_prop_center_y, ts->vk_prop_width, ts->vk_prop_height,
		VK_LCD_WIDTH*3/2, ts->vk_prop_center_y, ts->vk_prop_width, ts->vk_prop_height,
		VK_LCD_WIDTH*5/2+20 , ts->vk_prop_center_y, ts->vk_prop_width, ts->vk_prop_height);

	return len;
}

static struct kobj_attribute vk_syna_attr = {
	.attr = {
		.name = "virtualkeys.synaptics-rmi-ts",
		.mode = S_IRUGO,
	},
	.show = &vk_syna_show,
};

static struct attribute *syna_properties_attrs[] = {
	&vk_syna_attr.attr,
	NULL
};

static struct attribute_group syna_properties_attr_group = {
	.attrs = syna_properties_attrs,
};

static void synaptics_ts_init_area(struct synaptics_rmi4_data *ts)
{
	ts->snap_top = 0;
	ts->snap_left = 0;
	ts->snap_right = 0;
	ts->snap_bottom = 0;
}

static int synaptics_ts_init_virtual_key(struct synaptics_rmi4_data *ts )
{
	int ret = 0;

	vk_calculate_area();
	if (ts->properties_kobj || ts->sensor_max_x <= 0 || ts->sensor_max_y <= 0)
		return 0;

	ts->properties_kobj = kobject_create_and_add("board_properties", NULL);
	if (ts->properties_kobj)
		ret = sysfs_create_group(ts->properties_kobj,
				&syna_properties_attr_group);

	if (!ts->properties_kobj || ret)
		pr_err("%s: failed to create board_properties\n", __func__);

	return ret;
}

static int get_virtual_key_button(int x, int y)
{
	int i;

	if (y <= LCD_MAX_Y)
		return 0;

	for (i = 0; i < TP_VKEY_NONE; ++i) {
		struct tp_vkey_button* button = &vkey_buttons[i];
		if ((x >= button->x) && (x <= button->x + button->width))
			break;
	}

	return i;
}
/***** For virtual key definition end *********************/

static void synaptics_update_gesture_status(struct synaptics_rmi4_data *ts)
{
	atomic_set(&ts->syna_use_gesture,
			atomic_read(&ts->double_tap_enable) ||
			atomic_read(&ts->camera_enable) ||
			atomic_read(&ts->music_enable) ||
			atomic_read(&ts->flashlight_enable) ? 1 : 0);
}

static int synaptics_enable_gesture(struct synaptics_rmi4_data *rmi4_data, bool enable)
{
	unsigned short reportaddr = SYNA_ADDR_REPORT_FLAG;
	unsigned char reportbuf[3];
	unsigned char val[4];
	int ret;

	ret = synaptics_rmi4_i2c_read(rmi4_data,
			reportaddr,
			reportbuf,
			sizeof(reportbuf));
	if (ret) {
		dev_err(&rmi4_data->input_dev->dev,
				"%s: Failed to get report buffer\n",
				__func__);
		return ret;
	}

	ret = synaptics_rmi4_i2c_read(syna_rmi4_data,
				SYNA_ADDR_GESTURE_FLAG,
				val,
				sizeof(val));
	if (ret) {
		dev_err(&rmi4_data->input_dev->dev,
				"%s: Failed to get gesture buffer\n",
				__func__);
		return ret;
	}

	if (enable) {
		val[0] = 0x6b;
		reportbuf[2] |= 0x02;
	} else {
		val[0] = 0x00;
		reportbuf[2] &= 0xfd;
	}

	ret = synaptics_rmi4_i2c_write(syna_rmi4_data,
				SYNA_ADDR_GESTURE_FLAG,
				val,
				sizeof(val));
	if (ret) {
		dev_err(&rmi4_data->input_dev->dev,
				"%s: Failed to write gesture buffer\n",
				__func__);
		return ret;
	}

	ret = synaptics_rmi4_i2c_write(rmi4_data,
			reportaddr,
			reportbuf,
			sizeof(reportbuf));
	if (ret)
		dev_err(&rmi4_data->input_dev->dev,
				"%s: Failed to write report buffer\n",
				__func__);

	return ret;
}

static void synaptics_enable_irqwake(struct synaptics_rmi4_data *rmi4_data, bool enable)
{
	if (enable)
		enable_irq_wake(rmi4_data->i2c_client->irq);
	else
		disable_irq_wake(rmi4_data->i2c_client->irq);
}

static int synaptics_rmi4_proc_double_tap_read(char *page, char **start, off_t off,
		int count, int *eof, void *data)
{
	return sprintf(page, "%d\n", atomic_read(&syna_rmi4_data->double_tap_enable));
}

static int synaptics_rmi4_proc_double_tap_write(struct file *filp, const char __user *buff,
		unsigned long len, void *data)
{
	int enable;
	char buf[2];

	if (len > 2)
		return 0;

	if (copy_from_user(buf, buff, len)) {
		pr_debug("Read proc input error.\n");
		return -EFAULT;
	}

	enable = (buf[0] == '0') ? 0 : 1;

	atomic_set(&syna_rmi4_data->double_tap_enable, enable);
	synaptics_update_gesture_status(syna_rmi4_data);

	return len;
}

static int synaptics_rmi4_proc_camera_read(char *page, char **start, off_t off,
		int count, int *eof, void *data)
{
	return sprintf(page, "%d\n", atomic_read(&syna_rmi4_data->camera_enable));
}

static int synaptics_rmi4_proc_camera_write(struct file *filp, const char __user *buff,
		unsigned long len, void *data)
{
	int enable;
	char buf[2];

	if (len > 2)
		return 0;

	if (copy_from_user(buf, buff, len)) {
		pr_debug("Read proc input error.\n");
		return -EFAULT;
	}

	enable = (buf[0] == '0') ? 0 : 1;

	atomic_set(&syna_rmi4_data->camera_enable, enable);
	synaptics_update_gesture_status(syna_rmi4_data);

	return len;
}

static int synaptics_rmi4_proc_music_read(char *page, char **start, off_t off,
		int count, int *eof, void *data)
{
	return sprintf(page, "%d\n", atomic_read(&syna_rmi4_data->music_enable));
}

static int synaptics_rmi4_proc_music_write(struct file *filp, const char __user *buff,
		unsigned long len, void *data)
{
	int enable;
	char buf[2];

	if (len > 2)
		return 0;

	if (copy_from_user(buf, buff, len)) {
		pr_debug("Read proc input error.\n");
		return -EFAULT;
	}

	enable = (buf[0] == '0') ? 0 : 1;

	atomic_set(&syna_rmi4_data->music_enable, enable);
	synaptics_update_gesture_status(syna_rmi4_data);

	return len;
}

static int synaptics_rmi4_proc_flashlight_read(char *page, char **start, off_t off,
		int count, int *eof, void *data)
{
	return sprintf(page, "%d\n", atomic_read(&syna_rmi4_data->flashlight_enable));
}

static int synaptics_rmi4_proc_flashlight_write(struct file *filp, const char __user *buff,
		unsigned long len, void *data)
{
	int enable;
	char buf[2];

	if (len > 2)
		return 0;

	if (copy_from_user(buf, buff, len)) {
		pr_debug("Read proc input error.\n");
		return -EFAULT;
	}

	enable = (buf[0] == '0') ? 0 : 1;

	atomic_set(&syna_rmi4_data->flashlight_enable, enable);
	synaptics_update_gesture_status(syna_rmi4_data);

	return len;
}

static int keypad_enable_proc_read(char *page, char **start, off_t off,
		int count, int *eof, void *data)
{
	struct synaptics_rmi4_data *ts = data;
	return sprintf(page, "%d\n", atomic_read(&ts->keypad_enable));
}

static int keypad_enable_proc_write(struct file *file, const char __user *buffer,
		unsigned long count, void *data)
{
	struct synaptics_rmi4_data *ts = data;
	char buf[2];
	unsigned int val = 0;

	if (count > 2)
		return count;

	if (copy_from_user(buf, buffer, count)) {
		printk(KERN_ERR "%s: read proc input error.\n", __func__);
		return count;
	}

	val = (buf[0] == '0' ? 0 : 1);
	atomic_set(&ts->keypad_enable, val);
	if (val) {
		set_bit(KEY_BACK, ts->input_dev->keybit);
		set_bit(KEY_MENU, ts->input_dev->keybit);
		set_bit(KEY_HOMEPAGE, ts->input_dev->keybit);
	} else {
		clear_bit(KEY_BACK, ts->input_dev->keybit);
		clear_bit(KEY_MENU, ts->input_dev->keybit);
		clear_bit(KEY_HOMEPAGE, ts->input_dev->keybit);
	}
	input_sync(ts->input_dev);

	return count;
}

static int synaptics_rmi4_init_touchpanel_proc(void)
{
	struct proc_dir_entry *proc_entry;
	struct proc_dir_entry *procdir = proc_mkdir("touchpanel", NULL);

	// double tap to wake
	proc_entry = create_proc_entry("double_tap_enable", 0664, procdir);
	if (proc_entry) {
		proc_entry->write_proc = synaptics_rmi4_proc_double_tap_write;
		proc_entry->read_proc = synaptics_rmi4_proc_double_tap_read;
	}

	// wake to camera
	proc_entry = create_proc_entry("camera_enable", 0664, procdir);
	if (proc_entry) {
		proc_entry->write_proc = synaptics_rmi4_proc_camera_write;
		proc_entry->read_proc = synaptics_rmi4_proc_camera_read;
	}

	// wake to music
	proc_entry = create_proc_entry("music_enable", 0664, procdir);
	if (proc_entry) {
		proc_entry->write_proc = synaptics_rmi4_proc_music_write;
		proc_entry->read_proc = synaptics_rmi4_proc_music_read;
	}

	// wake to flashlight
	proc_entry = create_proc_entry("flashlight_enable", 0664, procdir);
	if (proc_entry) {
		proc_entry->write_proc = synaptics_rmi4_proc_flashlight_write;
		proc_entry->read_proc = synaptics_rmi4_proc_flashlight_read;
	}

	proc_entry = create_proc_entry("keypad_enable", 0664, procdir);
	if (proc_entry) {
		proc_entry->write_proc = keypad_enable_proc_write;
		proc_entry->read_proc = keypad_enable_proc_read;
		proc_entry->data = syna_rmi4_data;
	}

	return 0;
}

static struct synaptics_dsx_cap_button_map button_map = {
	.nbuttons = 0,
	.map = "NULL",
};

static struct synaptics_dsx_platform_data dsx_platformdata = {
	.irq_flags = IRQF_ONESHOT | IRQF_TRIGGER_FALLING,
	.reset_delay_ms = 100,
	.cap_button_map = &button_map,
	.regulator_en = 1,
};

static void synaptics_init_gpio(struct synaptics_rmi4_data *ts)
{
	int i;
	struct gpio synaptics_all_gpio[] = {
		{ts->irq_gpio, GPIO_CFG(ts->irq_gpio, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_UP, GPIO_CFG_2MA), "synaptics_irq_gpio"},
		{ts->reset_gpio, GPIO_CFG(ts->reset_gpio, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), "synaptics_reset_gpio"},
		{ts->wakeup_gpio, GPIO_CFG(ts->wakeup_gpio, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_UP, GPIO_CFG_2MA), "synaptics_wakeup_gpio"},
		{ts->id_gpio, GPIO_CFG(ts->id_gpio, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_UP, GPIO_CFG_2MA), "synaptics_id_gpio"},
	};

	for (i = 0; i < sizeof(synaptics_all_gpio)/sizeof(struct gpio); i++) {
		gpio_tlmm_config(synaptics_all_gpio[i].flags, GPIO_CFG_ENABLE);
		gpio_request(synaptics_all_gpio[i].gpio,synaptics_all_gpio[i].label);
	}
}

static void synaptics_parse_dt(struct device *dev, struct synaptics_rmi4_data *ts)
{
	struct device_node *np = dev->of_node;

	/* reset, irq gpio info */
	ts->irq_gpio    = of_get_named_gpio(np, "synaptics,irq-gpio", 0);
	ts->reset_gpio  = of_get_named_gpio(np, "synaptics,reset-gpio", 0);
	ts->wakeup_gpio = of_get_named_gpio(np, "synaptics,wakeup-gpio", 0);  //gpio 57
	ts->id_gpio     = of_get_named_gpio(np, "synaptics,id-gpio", 0);  //gpio 62
}

static int synaptics_regulator_configure(void)
{
	int rc;

	if (!vdd_regulator) {
		vdd_regulator = regulator_get(NULL, "8941_l22");
		if (IS_ERR(vdd_regulator)) {
			rc = PTR_ERR(vdd_regulator);
			printk("Regulator get failed vcc_ana rc=%d\n", rc);
			return rc;
		}
	}

	if (regulator_count_voltages(vdd_regulator)) {
		rc = regulator_set_voltage(vdd_regulator, SYNA_VTG_MIN_UV,
				SYNA_VTG_MAX_UV);
		if (rc) {
			printk("regulator set_vtg failed rc=%d\n", rc);
			goto error_set_vtg_vcc_ana;
		}
		regulator_enable(vdd_regulator);
	}

	vdd_regulator_i2c = regulator_get(NULL, "8941_s3");
	if (IS_ERR(vdd_regulator_i2c)) {
		rc = PTR_ERR(vdd_regulator_i2c);
		printk("Regulator get failed rc=%d\n", rc);
		goto error_get_vtg_i2c;
	}
	if (regulator_count_voltages(vdd_regulator_i2c)) {
		rc = regulator_set_voltage(vdd_regulator_i2c,
				SYNA_I2C_VTG_MIN_UV, SYNA_I2C_VTG_MAX_UV);
		if (rc) {
			printk("regulator set_vtg failed rc=%d\n", rc);
			goto error_set_vtg_i2c;
		}
	}

	return 0;

error_set_vtg_i2c:
	regulator_put(vdd_regulator_i2c);
error_get_vtg_i2c:
	if (regulator_count_voltages(vdd_regulator))
		regulator_set_voltage(vdd_regulator, 0, 0);
error_set_vtg_vcc_ana:
	regulator_put(vdd_regulator);
	return rc;
}

static unsigned char synaptics_rmi4_update_gesture2(unsigned char *gesture,
		unsigned char *gestureext)
{
	int i;
	unsigned char keyvalue = 0;
	unsigned char gesturemode;
	unsigned short points[16];

	for (i = 0; i < 12; i++) {
		points[i] = ((unsigned short)gestureext[i * 2]) |
			(((unsigned short)gestureext[i * 2 + 1]) << 8);
	}

	// 1--clockwise, 0--anticlockwise, not circle, report 2
	points[i] = (gestureext[24] == 0x10) ? 1 :
		(gestureext[24] == 0x20) ? 0 : 2;

	switch (gesture[0]) {
		case SYNA_ONE_FINGER_CIRCLE:
			gesturemode = Circle;
			if (atomic_read(&syna_rmi4_data->camera_enable))
				keyvalue = KEY_GESTURE_CIRCLE;
			break;

		case SYNA_TWO_FINGER_SWIPE:
			gesturemode =
				(gestureext[24] == 0x41) ? Left2RightSwip   :
				(gestureext[24] == 0x42) ? Right2LeftSwip   :
				(gestureext[24] == 0x44) ? Up2DownSwip      :
				(gestureext[24] == 0x48) ? Down2UpSwip      :
				(gestureext[24] == 0x80) ? DouSwip          :
				UnknownGesture;
			if (gesturemode == DouSwip ||
					gesturemode == Down2UpSwip ||
					gesturemode == Up2DownSwip) {
				if (abs(points[3] - points[1]) <= 800)
					gesturemode=UnknownGesture;
			}
			if (gesturemode == DouSwip) {
				if (atomic_read(&syna_rmi4_data->music_enable))
					keyvalue = KEY_GESTURE_SWIPE_DOWN;
			}
			break;

		case SYNA_ONE_FINGER_DOUBLE_TAP:
			gesturemode = DouTap;
			if (atomic_read(&syna_rmi4_data->double_tap_enable))
				keyvalue = KEY_WAKEUP;
			break;

		case SYNA_ONE_FINGER_DIRECTION:
			switch (gesture[2]) {
				case 0x01:  //UP
					gesturemode = DownVee;
					break;
				case 0x02:  //DOWN
					gesturemode = UpVee;
					if (atomic_read(&syna_rmi4_data->flashlight_enable))
						keyvalue = KEY_GESTURE_V;
					break;
				case 0x04:  //LEFT
					gesturemode = RightVee;
					if (atomic_read(&syna_rmi4_data->music_enable))
						keyvalue = KEY_GESTURE_LTR;
					break;
				case 0x08:  //RIGHT
					gesturemode = LeftVee;
					if (atomic_read(&syna_rmi4_data->music_enable))
						keyvalue = KEY_GESTURE_GTR;
					break;
			}
			break;

		case SYNA_ONE_FINGER_W_OR_M:
			gesturemode =
				(gesture[2] == 0x77 && gesture[3] == 0x00) ? Wgestrue :
				(gesture[2] == 0x6d && gesture[3] == 0x00) ? Mgestrue :
				UnknownGesture;

			keyvalue = KEY_F9;
			break;
	}

	return keyvalue;
}

/**
 * synaptics_rmi4_f12_abs_report()
 *
 * Called by synaptics_rmi4_report_touch() when valid Function $12
 * finger data has been detected.
 *
 * This function reads the Function $12 data registers, determines the
 * status of each finger supported by the Function, processes any
 * necessary coordinate manipulation, reports the finger data to
 * the input subsystem, and returns the number of fingers detected.
 */
static int synaptics_rmi4_f12_abs_report(struct synaptics_rmi4_data *rmi4_data,
		struct synaptics_rmi4_fn *fhandler)
{
	int ret;
	unsigned char touch_count = 0; /* number of touch points */
	unsigned char finger;
	unsigned char fingers_to_process;
	unsigned char finger_status;
	unsigned char size_of_2d_data;
	unsigned short data_addr;
	int x;
	int y;
	int wx;
	int wy;
	int temp;
	int z;
	struct synaptics_rmi4_f12_extra_data *extra_data;
	struct synaptics_rmi4_f12_finger_data *data;
	struct synaptics_rmi4_f12_finger_data *finger_data;
	unsigned char gesture[5];
	unsigned char gestureext[25];
	unsigned char keyvalue;
	unsigned int  finger_info = 0;
	u64 now = ktime_to_ms(ktime_get());

	fingers_to_process = fhandler->num_of_data_points;
	data_addr = fhandler->full_addr.data_base;
	extra_data = (struct synaptics_rmi4_f12_extra_data *)fhandler->extra;
	size_of_2d_data = sizeof(struct synaptics_rmi4_f12_finger_data);

	if (atomic_read(&rmi4_data->syna_use_gesture) &&
		(now - rmi4_data->last_gesture_time > SYNA_GESTURE_DELAY_MS) &&
		!atomic_read(&rmi4_data->resume_suspend)) {
		rmi4_data->last_gesture_time = now;

		synaptics_rmi4_i2c_read(rmi4_data,
				SYNA_ADDR_GESTURE_OFFSET,
				gesture,
				sizeof(gesture));
		synaptics_rmi4_i2c_read(rmi4_data,
				SYNA_ADDR_GESTURE_EXT,
				gestureext,
				sizeof(gestureext));
		if (gesture[0]) {
			keyvalue = synaptics_rmi4_update_gesture2(gesture, gestureext);
			if (keyvalue && keyvalue != KEY_F9) {
				input_report_key(rmi4_data->input_dev, keyvalue, 1);
				input_sync(rmi4_data->input_dev);
				input_report_key(rmi4_data->input_dev, keyvalue, 0);
				input_sync(rmi4_data->input_dev);
			}
		}
	}

	ret = synaptics_rmi4_i2c_read(rmi4_data,
			data_addr + extra_data->data1_offset,
			(unsigned char *)fhandler->data,
			fingers_to_process * size_of_2d_data);
	if (ret)
		return 0;

	data = (struct synaptics_rmi4_f12_finger_data *)fhandler->data;

	for (finger = 0; finger < fingers_to_process; finger++) {
		finger_data = data + finger;
		finger_status = finger_data->object_type_and_status & MASK_2BIT;
		finger_info <<= 1;

		if (finger_status) {
			x = (finger_data->x_msb << 8) | (finger_data->x_lsb);
			y = (finger_data->y_msb << 8) | (finger_data->y_lsb);
			// The presssure from the sensor is weak, *4
			z = finger_data->z << 2;
			wx = finger_data->wx;
			wy = finger_data->wy;

			if (rmi4_data->board->swap_axes) {
				temp = x;
				x = y;
				y = temp;
				temp = wx;
				wx = wy;
				wy = temp;
			}

			if (rmi4_data->board->x_flip)
				x = rmi4_data->sensor_max_x - x;

			if (rmi4_data->board->y_flip)
				y = rmi4_data->sensor_max_y - y;

			if (y > rmi4_data->sensor_max_y -
				rmi4_data->snap_top -
				rmi4_data->snap_bottom-rmi4_data->virtual_key_height) {
				if (!atomic_read(&rmi4_data->keypad_enable))
					continue;
			}

			if (get_virtual_key_button(x, y) == TP_VKEY_NONE)
				continue;

			input_mt_slot(rmi4_data->input_dev, finger);
			input_mt_report_slot_state(rmi4_data->input_dev,
					MT_TOOL_FINGER, finger_status);
			input_report_key(rmi4_data->input_dev,
					BTN_TOUCH, 1);
			input_report_key(rmi4_data->input_dev,
					BTN_TOOL_FINGER, 1);
			input_report_abs(rmi4_data->input_dev,
					ABS_MT_POSITION_X, x);
			input_report_abs(rmi4_data->input_dev,
					ABS_MT_POSITION_Y, y);
			input_report_abs(rmi4_data->input_dev,
					ABS_MT_PRESSURE, z);

			input_report_abs(rmi4_data->input_dev,
					ABS_MT_TOUCH_MAJOR, max(wx, wy));
			input_report_abs(rmi4_data->input_dev,
					ABS_MT_TOUCH_MINOR, min(wx, wy));
			touch_count++;
			finger_info |= 1;
		}
	}

	for (finger = 0; finger < fingers_to_process; finger++) {
		finger_status = (finger_info >> (fingers_to_process - finger - 1)) & 1;
		if (!finger_status) {
			input_mt_slot(rmi4_data->input_dev, finger);
			input_mt_report_slot_state(rmi4_data->input_dev,
					MT_TOOL_FINGER, finger_status);
		}
	}

	if (!touch_count) {
		input_report_key(rmi4_data->input_dev,
				BTN_TOUCH, 0);
		input_report_key(rmi4_data->input_dev,
				BTN_TOOL_FINGER, 0);
	}

	input_sync(rmi4_data->input_dev);

	return touch_count;
}

/**
 * synaptics_rmi4_report_touch()
 *
 * Called by synaptics_rmi4_sensor_report().
 *
 * This function calls the appropriate finger data reporting function
 * based on the function handler it receives and returns the number of
 * fingers detected.
 */
static void synaptics_rmi4_report_touch(struct synaptics_rmi4_data *rmi4_data,
		struct synaptics_rmi4_fn *fhandler, const ktime_t timestamp)
{
	input_event(rmi4_data->input_dev, EV_SYN, SYN_TIME_SEC,
			ktime_to_timespec(timestamp).tv_sec);
	input_event(rmi4_data->input_dev, EV_SYN, SYN_TIME_NSEC,
			ktime_to_timespec(timestamp).tv_nsec);
	synaptics_rmi4_f12_abs_report(rmi4_data, fhandler);
}

/**
 * synaptics_rmi4_sensor_report()
 *
 * Called by synaptics_rmi4_irq().
 *
 * This function determines the interrupt source(s) from the sensor
 * and calls synaptics_rmi4_report_touch() with the appropriate
 * function handler for each function with valid data inputs.
 */
static void synaptics_rmi4_sensor_report(struct synaptics_rmi4_data *rmi4_data, const ktime_t timestamp)
{
	int ret;
	unsigned char data[MAX_INTR_REGISTERS + 1];
	unsigned char *intr = &data[1];
	struct synaptics_rmi4_f01_device_status status;
	struct synaptics_rmi4_fn *fhandler;
	struct synaptics_rmi4_exp_fn *exp_fhandler;
	struct synaptics_rmi4_device_info *rmi = &rmi4_data->rmi4_mod_info;

	/*
	 * Get interrupt status information from F01 Data1 register to
	 * determine the source(s) that are flagging the interrupt.
	 */
	ret = synaptics_rmi4_i2c_read(rmi4_data,
			rmi4_data->f01_data_base_addr,
			data,
			rmi4_data->num_of_intr_regs + 1);
	if (ret) {
		dev_err(&rmi4_data->i2c_client->dev,
				"%s: Failed to read interrupt status\n",
				__func__);
		return;
	}

	status.data[0] = data[0];
	if (status.unconfigured && !status.flash_prog) {
		pr_notice("%s: spontaneous reset detected\n", __func__);
		synaptics_rmi4_reinit_device(rmi4_data);
		return;
	}

	/*
	 * Traverse the function handler list and service the source(s)
	 * of the interrupt accordingly.
	 */
	if (!list_empty(&rmi->support_fn_list)) {
		list_for_each_entry(fhandler, &rmi->support_fn_list, link) {
			if (fhandler == NULL)
				continue;
			if (fhandler->num_of_data_sources) {
				if (fhandler->intr_mask &
						intr[fhandler->intr_reg_num]) {
					synaptics_rmi4_report_touch(rmi4_data,
							fhandler, timestamp);
				}
			}
		}
	}

	mutex_lock(&exp_data.mutex);
	if (!list_empty(&exp_data.list)) {
		list_for_each_entry(exp_fhandler, &exp_data.list, link) {
			if (exp_fhandler == NULL)
				continue;
			if (exp_fhandler->inserted &&
					(exp_fhandler->func_attn != NULL))
				exp_fhandler->func_attn(rmi4_data, intr[0]);
		}
	}
	mutex_unlock(&exp_data.mutex);
}

/**
 * synaptics_rmi4_irq()
 *
 * Called by the kernel when an interrupt occurs (when the sensor
 * asserts the attention irq).
 *
 * This function is the ISR thread and handles the acquisition
 * and the reporting of finger data when the presence of fingers
 * is detected.
 */
static irqreturn_t synaptics_rmi4_irq(int irq, void *data)
{
	struct synaptics_rmi4_data *rmi4_data = data;
	unsigned long flags;
	bool i2c_active;

	if (!atomic_read(&rmi4_data->ts_awake)) {
		spin_lock_irqsave(&rmi4_data->isr_lock, flags);
		i2c_active = rmi4_data->i2c_awake;
		spin_unlock_irqrestore(&rmi4_data->isr_lock, flags);

		/* I2C bus must be active */
		if (!i2c_active) {
			__pm_stay_awake(&rmi4_data->syna_isr_ws);
			/* Wait for I2C to resume before proceeding */
			INIT_COMPLETION(rmi4_data->i2c_resume);
			wait_for_completion_timeout(&rmi4_data->i2c_resume,
							msecs_to_jiffies(30));
		}
	}

	synaptics_rmi4_sensor_report(rmi4_data, ktime_get());

	if (rmi4_data->syna_isr_ws.active)
		__pm_relax(&rmi4_data->syna_isr_ws);

	return IRQ_HANDLED;
}

/**
 * synaptics_rmi4_irq_enable()
 *
 * Called by synaptics_rmi4_probe() and the power management functions
 * in this driver and also exported to other expansion Function modules
 * such as rmi_dev.
 *
 * This function handles the enabling and disabling of the attention
 * irq.
 */
static int synaptics_rmi4_irq_enable(struct synaptics_rmi4_data *rmi4_data,
		bool enable)
{
	int ret = 0;
	unsigned char intr_status[MAX_INTR_REGISTERS];

	if ((enable && atomic_read(&rmi4_data->irq_enabled)) ||
		(!enable && !atomic_read(&rmi4_data->irq_enabled)))
		return ret;

	if (enable) {
		/* Clear interrupts first */
		ret = synaptics_rmi4_i2c_read(rmi4_data,
				rmi4_data->f01_data_base_addr + 1,
				intr_status,
				rmi4_data->num_of_intr_regs);
		if (ret) {
			dev_err(&rmi4_data->i2c_client->dev,
				"%s: Failed to read interrupt status %d\n",
				__func__, __LINE__);
			return ret;
		}

		enable_irq(rmi4_data->irq);
		atomic_set(&rmi4_data->irq_enabled, 1);
	} else {
		disable_irq(rmi4_data->irq);
		atomic_set(&rmi4_data->irq_enabled, 0);
	}

	return ret;
}

static int synaptics_rmi4_f12_set_enables(struct synaptics_rmi4_data *rmi4_data,
		unsigned short ctrl28)
{
	static unsigned short ctrl_28_address;

	if (ctrl28)
		ctrl_28_address = ctrl28;

	return synaptics_rmi4_i2c_write(rmi4_data,
			ctrl_28_address,
			&rmi4_data->report_enable,
			sizeof(rmi4_data->report_enable));
}

/**
 * synaptics_rmi4_f12_init()
 *
 * Called by synaptics_rmi4_query_device().
 *
 * This funtion parses information from the Function 12 registers and
 * determines the number of fingers supported, offset to the data1
 * register, x and y data ranges, offset to the associated interrupt
 * status register, interrupt bit mask, and allocates memory resources
 * for finger data acquisition.
 */
static int synaptics_rmi4_f12_init(struct synaptics_rmi4_data *rmi4_data,
		struct synaptics_rmi4_fn *fhandler,
		struct synaptics_rmi4_fn_desc *fd,
		unsigned int intr_count)
{
	int ret;
	unsigned char ii;
	unsigned char intr_offset;
	unsigned char size_of_2d_data;
	unsigned char size_of_query8;
	unsigned char ctrl_8_offset;
	unsigned char ctrl_23_offset;
	unsigned char ctrl_28_offset;
	unsigned char num_of_fingers;
	struct synaptics_rmi4_f12_extra_data *extra_data;
	struct synaptics_rmi4_f12_query_5 query_5;
	struct synaptics_rmi4_f12_query_8 query_8;
	struct synaptics_rmi4_f12_ctrl_8 ctrl_8;
	struct synaptics_rmi4_f12_ctrl_23 ctrl_23;

	fhandler->fn_number = fd->fn_number;
	fhandler->num_of_data_sources = fd->intr_src_count;
	fhandler->extra = kmalloc(sizeof(*extra_data), GFP_KERNEL);
	extra_data = (struct synaptics_rmi4_f12_extra_data *)fhandler->extra;
	size_of_2d_data = sizeof(struct synaptics_rmi4_f12_finger_data);

	ret = synaptics_rmi4_i2c_read(rmi4_data,
			fhandler->full_addr.query_base + 5,
			query_5.data,
			sizeof(query_5.data));
	if (ret)
		return ret;

	ctrl_8_offset = query_5.ctrl0_is_present +
		query_5.ctrl1_is_present +
		query_5.ctrl2_is_present +
		query_5.ctrl3_is_present +
		query_5.ctrl4_is_present +
		query_5.ctrl5_is_present +
		query_5.ctrl6_is_present +
		query_5.ctrl7_is_present;

	ctrl_23_offset = ctrl_8_offset +
		query_5.ctrl8_is_present +
		query_5.ctrl9_is_present +
		query_5.ctrl10_is_present +
		query_5.ctrl11_is_present +
		query_5.ctrl12_is_present +
		query_5.ctrl13_is_present +
		query_5.ctrl14_is_present +
		query_5.ctrl15_is_present +
		query_5.ctrl16_is_present +
		query_5.ctrl17_is_present +
		query_5.ctrl18_is_present +
		query_5.ctrl19_is_present +
		query_5.ctrl20_is_present +
		query_5.ctrl21_is_present +
		query_5.ctrl22_is_present;

	ctrl_28_offset = ctrl_23_offset +
		query_5.ctrl23_is_present +
		query_5.ctrl24_is_present +
		query_5.ctrl25_is_present +
		query_5.ctrl26_is_present +
		query_5.ctrl27_is_present;

	ret = synaptics_rmi4_i2c_read(rmi4_data,
			fhandler->full_addr.ctrl_base + ctrl_23_offset,
			ctrl_23.data,
			sizeof(ctrl_23.data));
	if (ret)
		return ret;

	/* Maximum number of fingers supported */
	fhandler->num_of_data_points = min(ctrl_23.max_reported_objects,
			(unsigned char)F12_FINGERS_TO_SUPPORT);

	num_of_fingers = fhandler->num_of_data_points;
	rmi4_data->num_of_fingers = num_of_fingers;

	ret = synaptics_rmi4_i2c_read(rmi4_data,
			fhandler->full_addr.query_base + 7,
			&size_of_query8,
			sizeof(size_of_query8));
	if (ret)
		return ret;

	ret = synaptics_rmi4_i2c_read(rmi4_data,
			fhandler->full_addr.query_base + 8,
			query_8.data,
			size_of_query8);
	if (ret)
		return ret;

	/* Determine the presence of the Data0 register */
	extra_data->data1_offset = query_8.data0_is_present;

	if ((size_of_query8 >= 3) && (query_8.data15_is_present)) {
		extra_data->data15_offset = query_8.data0_is_present +
			query_8.data1_is_present +
			query_8.data2_is_present +
			query_8.data3_is_present +
			query_8.data4_is_present +
			query_8.data5_is_present +
			query_8.data6_is_present +
			query_8.data7_is_present +
			query_8.data8_is_present +
			query_8.data9_is_present +
			query_8.data10_is_present +
			query_8.data11_is_present +
			query_8.data12_is_present +
			query_8.data13_is_present +
			query_8.data14_is_present;
		extra_data->data15_size = (num_of_fingers + 7) / 8;
	} else {
		extra_data->data15_size = 0;
	}

	rmi4_data->report_enable = RPT_DEFAULT;
	rmi4_data->report_enable |= RPT_Z;
	rmi4_data->report_enable |= (RPT_WX | RPT_WY);

	ret = synaptics_rmi4_f12_set_enables(rmi4_data,
			fhandler->full_addr.ctrl_base + ctrl_28_offset);
	if (ret)
		return ret;

	ret = synaptics_rmi4_i2c_read(rmi4_data,
			fhandler->full_addr.ctrl_base + ctrl_8_offset,
			ctrl_8.data,
			sizeof(ctrl_8.data));
	if (ret)
		return ret;

	/* Maximum x and y */
	rmi4_data->sensor_max_x =
		((unsigned short)ctrl_8.max_x_coord_lsb << 0) |
		((unsigned short)ctrl_8.max_x_coord_msb << 8);
	rmi4_data->sensor_max_y =
		((unsigned short)ctrl_8.max_y_coord_lsb << 0) |
		((unsigned short)ctrl_8.max_y_coord_msb << 8);
	dev_dbg(&rmi4_data->i2c_client->dev,
			"%s: Function %02x max x = %d max y = %d\n",
			__func__, fhandler->fn_number,
			rmi4_data->sensor_max_x,
			rmi4_data->sensor_max_y);

	rmi4_data->num_of_rx = ctrl_8.num_of_rx;
	rmi4_data->num_of_tx = ctrl_8.num_of_tx;
	rmi4_data->max_touch_width = max(rmi4_data->num_of_rx,
			rmi4_data->num_of_tx);

	fhandler->intr_reg_num = (intr_count + 7) / 8;
	if (fhandler->intr_reg_num != 0)
		fhandler->intr_reg_num -= 1;

	/* Set an enable bit for each data source */
	intr_offset = intr_count % 8;
	fhandler->intr_mask = 0;
	for (ii = intr_offset;
			ii < ((fd->intr_src_count & MASK_3BIT) +
				intr_offset);
			ii++)
		fhandler->intr_mask |= 1 << ii;

	/* Allocate memory for finger data storage space */
	fhandler->data_size = num_of_fingers * size_of_2d_data;
	fhandler->data = kmalloc(fhandler->data_size, GFP_KERNEL);

	return ret;
}

static void synaptics_rmi4_empty_fn_list(struct synaptics_rmi4_data *rmi4_data)
{
	struct synaptics_rmi4_fn *fhandler;
	struct synaptics_rmi4_fn *fhandler_temp;
	struct synaptics_rmi4_device_info *rmi = &rmi4_data->rmi4_mod_info;

	if (!list_empty(&rmi->support_fn_list)) {
		list_for_each_entry_safe(fhandler,
				fhandler_temp,
				&rmi->support_fn_list,
				link) {
			kfree(fhandler->extra);
			kfree(fhandler->data);
			list_del(&fhandler->link);
			kfree(fhandler);
		}
	}
	INIT_LIST_HEAD(&rmi->support_fn_list);
}

static int synaptics_rmi4_check_status(struct synaptics_rmi4_data *rmi4_data,
		bool *was_in_bl_mode)
{
	int ret;
	int timeout = CHECK_STATUS_TIMEOUT_MS;
	unsigned char command = 0x01;
	unsigned char intr_status;
	struct synaptics_rmi4_f01_device_status status;

	/* Do a device reset first */
	ret = synaptics_rmi4_i2c_write(rmi4_data,
			rmi4_data->f01_cmd_base_addr,
			&command,
			sizeof(command));
	if (ret)
		return ret;

	msleep(rmi4_data->board->reset_delay_ms);

	ret = synaptics_rmi4_i2c_read(rmi4_data,
			rmi4_data->f01_data_base_addr,
			status.data,
			sizeof(status.data));
	if (ret)
		return ret;

	while (status.status_code == STATUS_CRC_IN_PROGRESS) {
		if (timeout > 0)
			msleep(20);
		else
			return -1;

		ret = synaptics_rmi4_i2c_read(rmi4_data,
				rmi4_data->f01_data_base_addr,
				status.data,
				sizeof(status.data));
		if (ret)
			return ret;

		timeout -= 20;
	}

	if (timeout != CHECK_STATUS_TIMEOUT_MS)
		*was_in_bl_mode = true;

	if (status.flash_prog == 1) {
		rmi4_data->flash_prog_mode = true;
		pr_notice("%s: In flash prog mode, status = 0x%02x\n",
				__func__,
				status.status_code);
	} else {
		rmi4_data->flash_prog_mode = false;
	}

	ret = synaptics_rmi4_i2c_read(rmi4_data,
			rmi4_data->f01_data_base_addr + 1,
			&intr_status,
			sizeof(intr_status));
	if (ret) {
		dev_err(&rmi4_data->i2c_client->dev,
				"%s: Failed to read interrupt status\n",
				__func__);
		return ret;
	}

	return 0;
}

static void synaptics_rmi4_set_configured(struct synaptics_rmi4_data *rmi4_data)
{
	int ret;
	unsigned char device_ctrl;

	ret = synaptics_rmi4_i2c_read(rmi4_data,
			rmi4_data->f01_ctrl_base_addr,
			&device_ctrl,
			sizeof(device_ctrl));
	if (ret) {
		dev_err(&rmi4_data->input_dev->dev,
				"%s: Failed to set configured\n",
				__func__);
		return;
	}

	rmi4_data->no_sleep_setting = device_ctrl & NO_SLEEP_ON;
	device_ctrl |= CONFIGURED;

	ret = synaptics_rmi4_i2c_write(rmi4_data,
			rmi4_data->f01_ctrl_base_addr,
			&device_ctrl,
			sizeof(device_ctrl));
	if (ret) {
		dev_err(&rmi4_data->input_dev->dev,
				"%s: Failed to set configured\n",
				__func__);
	}
}

static int synaptics_rmi4_alloc_fh(struct synaptics_rmi4_fn **fhandler,
		struct synaptics_rmi4_fn_desc *rmi_fd, int page_number)
{
	*fhandler = kmalloc(sizeof(**fhandler), GFP_KERNEL);
	if (!(*fhandler))
		return -ENOMEM;

	(*fhandler)->full_addr.data_base =
		(rmi_fd->data_base_addr |
		 (page_number << 8));
	(*fhandler)->full_addr.ctrl_base =
		(rmi_fd->ctrl_base_addr |
		 (page_number << 8));
	(*fhandler)->full_addr.cmd_base =
		(rmi_fd->cmd_base_addr |
		 (page_number << 8));
	(*fhandler)->full_addr.query_base =
		(rmi_fd->query_base_addr |
		 (page_number << 8));

	return 0;
}

/**
 * synaptics_rmi4_query_device()
 *
 * Called by synaptics_rmi4_probe().
 *
 * This funtion scans the page description table, records the offsets
 * to the register types of Function $01, sets up the function handlers
 * for Function $11 and Function $12, determines the number of interrupt
 * sources from the sensor, adds valid Functions with data inputs to the
 * Function linked list, parses information from the query registers of
 * Function $01, and enables the interrupt sources from the valid Functions
 * with data inputs.
 */
static int synaptics_rmi4_query_device(struct synaptics_rmi4_data *rmi4_data)
{
	int ret;
	unsigned char ii;
	unsigned char page_number;
	unsigned char intr_count;
	unsigned char f01_query[F01_STD_QUERY_LEN];
	unsigned short pdt_entry_addr;
	unsigned short intr_addr;
	bool was_in_bl_mode;
	struct synaptics_rmi4_fn_desc rmi_fd;
	struct synaptics_rmi4_fn *fhandler;
	struct synaptics_rmi4_device_info *rmi = &rmi4_data->rmi4_mod_info;

rescan_pdt:
	was_in_bl_mode = false;
	intr_count = 0;
	INIT_LIST_HEAD(&rmi->support_fn_list);

	/* Scan the page description tables of the pages to service */
	for (page_number = 0; page_number < PAGES_TO_SERVICE; page_number++) {
		for (pdt_entry_addr = PDT_START; pdt_entry_addr > PDT_END;
				pdt_entry_addr -= PDT_ENTRY_SIZE) {
			pdt_entry_addr |= (page_number << 8);

			ret = synaptics_rmi4_i2c_read(rmi4_data,
					pdt_entry_addr,
					(unsigned char *)&rmi_fd,
					sizeof(rmi_fd));
			if (ret)
				return ret;

			fhandler = NULL;

			if (!rmi_fd.fn_number) {
				dev_dbg(&rmi4_data->i2c_client->dev,
						"%s: Reached end of PDT\n",
						__func__);
				break;
			}

			dev_dbg(&rmi4_data->i2c_client->dev,
					"%s: F%02x found (page %d)\n",
					__func__, rmi_fd.fn_number,
					page_number);

			switch (rmi_fd.fn_number) {
				case SYNAPTICS_RMI4_F01:
					rmi4_data->f01_query_base_addr =
						rmi_fd.query_base_addr;
					rmi4_data->f01_ctrl_base_addr =
						rmi_fd.ctrl_base_addr;
					rmi4_data->f01_data_base_addr =
						rmi_fd.data_base_addr;
					rmi4_data->f01_cmd_base_addr =
						rmi_fd.cmd_base_addr;

					ret = synaptics_rmi4_check_status(rmi4_data,
							&was_in_bl_mode);
					if (ret) {
						dev_err(&rmi4_data->i2c_client->dev,
								"%s: Failed to check status\n",
								__func__);
						return ret;
					}

					if (was_in_bl_mode)
						goto rescan_pdt;

					if (rmi4_data->flash_prog_mode)
						goto flash_prog_mode;

					break;
				case SYNAPTICS_RMI4_F12:
					if (rmi_fd.intr_src_count == 0)
						break;

					ret = synaptics_rmi4_alloc_fh(&fhandler,
							&rmi_fd, page_number);
					if (ret) {
						dev_err(&rmi4_data->i2c_client->dev,
								"%s: Failed to alloc for F%d\n",
								__func__,
								rmi_fd.fn_number);
						return ret;
					}

					ret = synaptics_rmi4_f12_init(rmi4_data,
							fhandler, &rmi_fd, intr_count);
					if (ret)
						return ret;
					break;
			}

			/* Accumulate the interrupt count */
			intr_count += (rmi_fd.intr_src_count & MASK_3BIT);

			if (fhandler && rmi_fd.intr_src_count) {
				list_add_tail(&fhandler->link,
						&rmi->support_fn_list);
			}
		}
	}

flash_prog_mode:
	rmi4_data->num_of_intr_regs = (intr_count + 7) / 8;
	dev_dbg(&rmi4_data->i2c_client->dev,
			"%s: Number of interrupt registers = %d\n",
			__func__, rmi4_data->num_of_intr_regs);

	ret = synaptics_rmi4_i2c_read(rmi4_data,
			rmi4_data->f01_query_base_addr,
			f01_query,
			sizeof(f01_query));
	if (ret)
		return ret;

	/* RMI Version 4.0 currently supported */
	rmi->version_major = 4;
	rmi->version_minor = 0;

	rmi->manufacturer_id = f01_query[0];
	rmi->product_props = f01_query[1];
	rmi->product_info[0] = f01_query[2] & MASK_7BIT;
	rmi->product_info[1] = f01_query[3] & MASK_7BIT;
	rmi->date_code[0] = f01_query[4] & MASK_5BIT;
	rmi->date_code[1] = f01_query[5] & MASK_4BIT;
	rmi->date_code[2] = f01_query[6] & MASK_5BIT;
	rmi->tester_id = ((f01_query[7] & MASK_7BIT) << 8) |
		(f01_query[8] & MASK_7BIT);
	rmi->serial_number = ((f01_query[9] & MASK_7BIT) << 8) |
		(f01_query[10] & MASK_7BIT);
	memcpy(rmi->product_id_string, &f01_query[11], 10);

	ret = synaptics_rmi4_i2c_read(rmi4_data,
			rmi4_data->f01_query_base_addr + F01_BUID_ID_OFFSET,
			rmi->build_id,
			sizeof(rmi->build_id));
	if (ret)
		return ret;

	rmi4_data->firmware_id = (unsigned int)rmi->build_id[0] +
		(unsigned int)rmi->build_id[1] * 0x100 +
		(unsigned int)rmi->build_id[2] * 0x10000;

	memset(rmi4_data->intr_mask, 0x00, sizeof(rmi4_data->intr_mask));

	/*
	 * Map out the interrupt bit masks for the interrupt sources
	 * from the registered function handlers.
	 */
	if (!list_empty(&rmi->support_fn_list)) {
		list_for_each_entry(fhandler, &rmi->support_fn_list, link) {
			if (fhandler == NULL)
				continue;
			if (fhandler->num_of_data_sources) {
				rmi4_data->intr_mask[fhandler->intr_reg_num] |=
					fhandler->intr_mask;
			}
		}
	}

	/* Enable the interrupt sources */
	for (ii = 0; ii < rmi4_data->num_of_intr_regs; ii++) {
		if (rmi4_data->intr_mask[ii] != 0x00) {
			dev_dbg(&rmi4_data->i2c_client->dev,
					"%s: Interrupt enable mask %d = 0x%02x\n",
					__func__, ii, rmi4_data->intr_mask[ii]);
			intr_addr = rmi4_data->f01_ctrl_base_addr + 1 + ii;
			ret = synaptics_rmi4_i2c_write(rmi4_data,
					intr_addr,
					&rmi4_data->intr_mask[ii],
					sizeof(rmi4_data->intr_mask[ii]));
			if (ret)
				return ret;
		}
	}

	synaptics_rmi4_set_configured(rmi4_data);

	return 0;
}

static void synaptics_rmi4_set_params(struct synaptics_rmi4_data *rmi4_data)
{
	set_bit(KEY_BACK, rmi4_data->input_dev->keybit);
	set_bit(KEY_MENU, rmi4_data->input_dev->keybit);
	set_bit(KEY_HOMEPAGE, rmi4_data->input_dev->keybit);
	set_bit(KEY_F3, rmi4_data->input_dev->keybit);
	set_bit(KEY_WAKEUP, rmi4_data->input_dev->keybit);
	set_bit(KEY_GESTURE_CIRCLE, rmi4_data->input_dev->keybit);
	set_bit(KEY_GESTURE_SWIPE_DOWN, rmi4_data->input_dev->keybit);
	set_bit(KEY_GESTURE_V, rmi4_data->input_dev->keybit);
	set_bit(KEY_GESTURE_LTR, rmi4_data->input_dev->keybit);
	set_bit(KEY_GESTURE_GTR, rmi4_data->input_dev->keybit);
	synaptics_ts_init_virtual_key(rmi4_data);

	input_set_abs_params(rmi4_data->input_dev,
			ABS_MT_POSITION_X, rmi4_data->snap_left,
			rmi4_data->sensor_max_x-rmi4_data->snap_right, 0, 0);
	input_set_abs_params(rmi4_data->input_dev,
			ABS_MT_POSITION_Y, rmi4_data->snap_top,
			rmi4_data->sensor_max_y-rmi4_data->virtual_key_height -
			rmi4_data->snap_bottom, 0, 0);
	input_set_abs_params(rmi4_data->input_dev,
			ABS_MT_PRESSURE, 0, 255, 0, 0);
	input_set_abs_params(rmi4_data->input_dev,
			ABS_MT_TOUCH_MAJOR, 0,
			rmi4_data->max_touch_width, 0, 0);
	input_set_abs_params(rmi4_data->input_dev,
			ABS_MT_TOUCH_MINOR, 0,
			rmi4_data->max_touch_width, 0, 0);
	input_mt_init_slots(rmi4_data->input_dev, rmi4_data->num_of_fingers);
}

static int synaptics_rmi4_set_input_dev(struct synaptics_rmi4_data *rmi4_data)
{
	int ret;
	int temp;

	rmi4_data->input_dev = input_allocate_device();
	if (rmi4_data->input_dev == NULL) {
		dev_err(&rmi4_data->i2c_client->dev,
				"%s: Failed to allocate input device\n",
				__func__);
		ret = -ENOMEM;
		goto err_input_device;
	}

	rmi4_data->input_dev->name = "synaptics-rmi-ts";
	rmi4_data->input_dev->phys = "synaptics-rmi-ts/input0";
	rmi4_data->input_dev->id.product = SYNAPTICS_DSX_DRIVER_PRODUCT;
	rmi4_data->input_dev->id.version = SYNAPTICS_DSX_DRIVER_VERSION;
	rmi4_data->input_dev->id.bustype = BUS_I2C;
	rmi4_data->input_dev->dev.parent = &rmi4_data->i2c_client->dev;
	input_set_drvdata(rmi4_data->input_dev, rmi4_data);

	set_bit(EV_SYN, rmi4_data->input_dev->evbit);
	set_bit(EV_KEY, rmi4_data->input_dev->evbit);
	set_bit(EV_ABS, rmi4_data->input_dev->evbit);
	set_bit(BTN_TOUCH, rmi4_data->input_dev->keybit);
	set_bit(BTN_TOOL_FINGER, rmi4_data->input_dev->keybit);

	atomic_set(&rmi4_data->keypad_enable, 1);
	atomic_set(&rmi4_data->syna_use_gesture, 1);
	atomic_set(&rmi4_data->double_tap_enable, 1);
	atomic_set(&rmi4_data->camera_enable, 0);
	atomic_set(&rmi4_data->music_enable, 0);
	atomic_set(&rmi4_data->flashlight_enable, 0);

	set_bit(INPUT_PROP_DIRECT, rmi4_data->input_dev->propbit);

	ret = synaptics_rmi4_query_device(rmi4_data);
	if (ret) {
		dev_err(&rmi4_data->i2c_client->dev,
				"%s: Failed to query device\n",
				__func__);
		goto err_query_device;
	}

	if (rmi4_data->board->swap_axes) {
		temp = rmi4_data->sensor_max_x;
		rmi4_data->sensor_max_x = rmi4_data->sensor_max_y;
		rmi4_data->sensor_max_y = temp;
	}

	synaptics_rmi4_set_params(rmi4_data);

	ret = input_register_device(rmi4_data->input_dev);
	if (ret) {
		dev_err(&rmi4_data->i2c_client->dev,
				"%s: Failed to register input device\n",
				__func__);
		goto err_query_device;
	}

	return 0;

err_query_device:
	synaptics_rmi4_empty_fn_list(rmi4_data);
	input_free_device(rmi4_data->input_dev);

err_input_device:
	return ret;
}

static void synaptics_rmi4_free_fingers(struct synaptics_rmi4_data *rmi4_data)
{
	unsigned char ii;

	for (ii = 0; ii < rmi4_data->num_of_fingers; ii++) {
		input_mt_slot(rmi4_data->input_dev, ii);
		input_mt_report_slot_state(rmi4_data->input_dev,
				MT_TOOL_FINGER, 0);
	}

	input_report_key(rmi4_data->input_dev,
			BTN_TOUCH, 0);
	input_report_key(rmi4_data->input_dev,
			BTN_TOOL_FINGER, 0);
	input_sync(rmi4_data->input_dev);
}

static void synaptics_rmi4_reinit_device(struct synaptics_rmi4_data *rmi4_data)
{
	mutex_lock(&rmi4_data->rmi4_reset_mutex);
	synaptics_rmi4_free_fingers(rmi4_data);
	synaptics_rmi4_set_configured(rmi4_data);
	mutex_unlock(&rmi4_data->rmi4_reset_mutex);
}

static void synaptics_rmi4_reset_device(struct synaptics_rmi4_data *rmi4_data,
		unsigned short f01_cmd_base_addr)
{
	int ret;
	int temp;

	rmi4_data->reset_count ++;
	if (rmi4_data->reset_count >= 2)
		return;

	mutex_lock(&rmi4_data->rmi4_reset_mutex);

	//power off device
	regulator_disable(rmi4_data->regulator);
	msleep(30);
	rmi4_data->current_page = MASK_8BIT;
	if (atomic_read(&rmi4_data->irq_enabled)) {
		disable_irq(rmi4_data->irq);
		atomic_set(&rmi4_data->irq_enabled, 0);
	}

	synaptics_rmi4_free_fingers(rmi4_data);

	synaptics_rmi4_empty_fn_list(rmi4_data);

	//power on device
	regulator_enable(rmi4_data->regulator);
	msleep(180);

	ret = synaptics_rmi4_query_device(rmi4_data);
	if (ret) {
		dev_err(&rmi4_data->i2c_client->dev,
				"%s: Failed to query device\n",
				__func__);
		mutex_unlock(&rmi4_data->rmi4_reset_mutex);
		return;
	}

	if (rmi4_data->board->swap_axes) {
		temp = rmi4_data->sensor_max_x;
		rmi4_data->sensor_max_x = rmi4_data->sensor_max_y;
		rmi4_data->sensor_max_y = temp;
	}

	synaptics_rmi4_set_params(rmi4_data);

	mutex_unlock(&rmi4_data->rmi4_reset_mutex);

	//reinit device
	msleep(10);
	synaptics_rmi4_i2c_read(rmi4_data,rmi4_data->f01_data_base_addr + 1,
			(unsigned char *)&temp, 1);
	if (!atomic_read(&rmi4_data->irq_enabled)) {
		enable_irq(rmi4_data->irq);
		atomic_set(&rmi4_data->irq_enabled, 1);
	}
}

/**
 * synaptics_rmi4_exp_fn_work()
 *
 * Called by the kernel at the scheduled time.
 *
 * This function is a work thread that checks for the insertion and
 * removal of other expansion Function modules such as rmi_dev and calls
 * their initialization and removal callback functions accordingly.
 */
static void synaptics_rmi4_exp_fn_work(struct work_struct *work)
{
	struct synaptics_rmi4_exp_fn *exp_fhandler;
	struct synaptics_rmi4_exp_fn *exp_fhandler_temp;
	struct synaptics_rmi4_data *rmi4_data = exp_data.rmi4_data;

	mutex_lock(&exp_data.mutex);
	if (!list_empty(&exp_data.list)) {
		list_for_each_entry_safe(exp_fhandler,
				exp_fhandler_temp,
				&exp_data.list,
				link) {
			if (exp_fhandler == NULL)
				continue;
			if ((exp_fhandler->func_init != NULL) &&
					(exp_fhandler->inserted == false)) {
				if (exp_fhandler->func_init(rmi4_data) < 0) {
					//retry init
					queue_delayed_work(exp_data.workqueue,
							&exp_data.work,
							msecs_to_jiffies(500));
					mutex_unlock(&exp_data.mutex);
					return;
				}
				exp_fhandler->inserted = true;
			} else if ((exp_fhandler->func_init == NULL) &&
					(exp_fhandler->inserted == true)) {
				exp_fhandler->func_remove(rmi4_data);
				list_del(&exp_fhandler->link);
				kfree(exp_fhandler);
			}
		}
	}
	mutex_unlock(&exp_data.mutex);
}

/**
 * synaptics_rmi4_new_function()
 *
 * Called by other expansion Function modules in their module init and
 * module exit functions.
 *
 * This function is used by other expansion Function modules such as
 * rmi_dev to register themselves with the driver by providing their
 * initialization and removal callback function pointers so that they
 * can be inserted or removed dynamically at module init and exit times,
 * respectively.
 */
void synaptics_rmi4_new_function(enum exp_fn fn_type, bool insert,
		int (*func_init)(struct synaptics_rmi4_data *rmi4_data),
		void (*func_remove)(struct synaptics_rmi4_data *rmi4_data),
		void (*func_attn)(struct synaptics_rmi4_data *rmi4_data,
			unsigned char intr_mask))
{
	struct synaptics_rmi4_exp_fn *exp_fhandler;

	if (!exp_data.initialized) {
		mutex_init(&exp_data.mutex);
		INIT_LIST_HEAD(&exp_data.list);
		exp_data.initialized = true;
	}

	mutex_lock(&exp_data.mutex);
	if (insert) {
		exp_fhandler = kzalloc(sizeof(*exp_fhandler), GFP_KERNEL);
		if (!exp_fhandler) {
			pr_err("%s: Failed to alloc mem for expansion function\n",
					__func__);
			goto exit;
		}
		exp_fhandler->fn_type = fn_type;
		exp_fhandler->func_init = func_init;
		exp_fhandler->func_attn = func_attn;
		exp_fhandler->func_remove = func_remove;
		exp_fhandler->inserted = false;
		list_add_tail(&exp_fhandler->link, &exp_data.list);
	} else if (!list_empty(&exp_data.list)) {
		list_for_each_entry(exp_fhandler, &exp_data.list, link) {
            if (exp_fhandler == NULL)
                continue;
			if (exp_fhandler->fn_type == fn_type) {
				exp_fhandler->func_init = NULL;
				exp_fhandler->func_attn = NULL;
				goto exit;
			}
		}
	}

exit:
	mutex_unlock(&exp_data.mutex);

	if (exp_data.queue_work) {
		queue_delayed_work(exp_data.workqueue,
				&exp_data.work,
				msecs_to_jiffies(200));
	}
}
EXPORT_SYMBOL(synaptics_rmi4_new_function);

static int synaptics_rmi4_get_vendorid1(int id1, int id2, int id3)
{
	if (id1 == 0 && id2 == 0 && id3 == 0)
		return TP_VENDOR_TPK;
	else if (id1 == 0 && id2 == 1 && id3 == 0)
		return TP_VENDOR_WINTEK;

	return 0;
}

/* return firmware version and string */
extern int synaptics_rmi4_get_firmware_version(int vendor, int lcd_type);

static char * synaptics_rmi4_get_vendorstring(int tp_type, int lcd_type) {
	char *pconst = "UNKNOWN";

	pconst = tp_firmware_strings[tp_type - 1][lcd_type];
	sprintf(synaptics_vendor_str, "%s(%x)", pconst,
			synaptics_rmi4_get_firmware_version(tp_type, lcd_type));

	return synaptics_vendor_str;
}

int lcd_type_id;

static int __init lcd_type_id_setup(char *str)
{
	if (!strcmp("1:dsi:0:qcom,mdss_dsi_jdi_1080p_cmd", str))
		lcd_type_id = LCD_VENDOR_JDI;
	else if (!strcmp("1:dsi:0:qcom,mdss_dsi_truly_1080p_cmd", str))
		lcd_type_id = LCD_VENDOR_TRULY;
	else if (!strcmp("1:dsi:0:qcom,mdss_dsi_sharp_1080p_cmd", str))
		lcd_type_id = LCD_VENDOR_SHARP;
	else
		lcd_type_id = -1;

	return 1;
}
__setup("mdss_mdp.panel=", lcd_type_id_setup);

static void synaptics_rmi4_get_vendorid(struct synaptics_rmi4_data *rmi4_data)
{
	int vendor_id;

	vendor_id = synaptics_rmi4_get_vendorid1(gpio_get_value(rmi4_data->id_gpio),
			gpio_get_value(rmi4_data->wakeup_gpio), 0);

	rmi4_data->vendor_id = vendor_id;
	synaptics_rmi4_get_vendorstring(rmi4_data->vendor_id, lcd_type_id);
	pr_err("vendor id: %x\n", vendor_id);
}

/**
 * synaptics_rmi4_sensor_sleep()
 *
 * Called by synaptics_rmi4_early_suspend() and synaptics_rmi4_suspend().
 *
 * This function stops finger data acquisition and puts the sensor to sleep.
 */
static void synaptics_rmi4_sensor_sleep(struct synaptics_rmi4_data *rmi4_data)
{
	unsigned char device_ctrl;
	int ret;

	if (!atomic_read(&rmi4_data->sensor_awake))
		return;

	ret = synaptics_rmi4_i2c_read(rmi4_data,
			rmi4_data->f01_ctrl_base_addr,
			&device_ctrl,
			sizeof(device_ctrl));
	if (ret) {
		dev_err(&rmi4_data->input_dev->dev,
				"%s: Failed to enter sleep mode\n",
				__func__);
		return;
	}

	device_ctrl = (device_ctrl & ~MASK_3BIT);
	device_ctrl = (NO_SLEEP_OFF | SENSOR_SLEEP);

	ret = synaptics_rmi4_i2c_write(rmi4_data,
			rmi4_data->f01_ctrl_base_addr,
			&device_ctrl,
			sizeof(device_ctrl));
	if (ret) {
		dev_err(&rmi4_data->input_dev->dev,
				"%s: Failed to enter sleep mode\n",
				__func__);
		return;
	}

	atomic_set(&rmi4_data->sensor_awake, 0);
}

/**
 * synaptics_rmi4_sensor_wake()
 *
 * Called by synaptics_rmi4_resume() and synaptics_rmi4_late_resume().
 *
 * This function wakes the sensor from sleep.
 */
static void synaptics_rmi4_sensor_wake(struct synaptics_rmi4_data *rmi4_data)
{
	unsigned char device_ctrl;
	unsigned char no_sleep_setting = rmi4_data->no_sleep_setting;
	int ret;

	if (atomic_read(&rmi4_data->sensor_awake))
		return;

	ret = synaptics_rmi4_i2c_read(rmi4_data,
			rmi4_data->f01_ctrl_base_addr,
			&device_ctrl,
			sizeof(device_ctrl));
	if (ret) {
		dev_err(&rmi4_data->input_dev->dev,
				"%s: Failed to wake from sleep mode\n",
				__func__);
		return;
	}

	device_ctrl = (device_ctrl & ~MASK_3BIT);
	device_ctrl = (device_ctrl | no_sleep_setting | NORMAL_OPERATION);

	ret = synaptics_rmi4_i2c_write(rmi4_data,
			rmi4_data->f01_ctrl_base_addr,
			&device_ctrl,
			sizeof(device_ctrl));
	if (ret) {
		dev_err(&rmi4_data->input_dev->dev,
				"%s: Failed to wake from sleep mode\n",
				__func__);
		return;
	}

	atomic_set(&rmi4_data->sensor_awake, 1);
}

/**
 * synaptics_rmi4_suspend()
 *
 * Called by the kernel during the suspend phase when the system
 * enters suspend.
 *
 * This function stops finger data acquisition and puts the sensor to
 * sleep (if not already done so during the early suspend phase),
 * disables the interrupt, and turns off the power to the sensor.
 */
static void synaptics_rmi4_suspend(struct synaptics_rmi4_data *rmi4_data)
{
	synaptics_rmi4_irq_enable(rmi4_data, false);
	synaptics_rmi4_free_fingers(rmi4_data);

	if (atomic_read(&rmi4_data->syna_use_gesture)) {
		synaptics_enable_gesture(rmi4_data, true);
		synaptics_enable_irqwake(rmi4_data, true);
		synaptics_rmi4_irq_enable(rmi4_data, true);
	} else {
		synaptics_rmi4_sensor_sleep(rmi4_data);
	}

	atomic_set(&rmi4_data->ts_awake, 0);

	pm_relax(&rmi4_data->i2c_client->dev);
}

/**
 * synaptics_rmi4_resume()
 *
 * Called by the kernel during the resume phase when the system
 * wakes up from suspend.
 *
 * This function turns on the power to the sensor, wakes the sensor
 * from sleep, enables the interrupt, and starts finger data
 * acquisition.
 */
static void synaptics_rmi4_resume(struct synaptics_rmi4_data *rmi4_data)
{
	if (atomic_read(&rmi4_data->irq_enabled)) {
		synaptics_rmi4_irq_enable(rmi4_data, false);
		synaptics_enable_irqwake(rmi4_data, false);
		synaptics_enable_gesture(rmi4_data, false);
		synaptics_rmi4_free_fingers(rmi4_data);
	} else {
		synaptics_rmi4_sensor_wake(rmi4_data);
		synaptics_rmi4_reinit_device(rmi4_data);
	}

	synaptics_rmi4_irq_enable(rmi4_data, true);
	atomic_set(&rmi4_data->ts_awake, 1);
}

static void synaptics_rmi4_pm_main(struct work_struct *work)
{
	struct synaptics_rmi4_data *rmi4_data =
			container_of(work, struct synaptics_rmi4_data, syna_pm_work);

	if (atomic_read(&rmi4_data->resume_suspend))
		synaptics_rmi4_resume(rmi4_data);
	else
		synaptics_rmi4_suspend(rmi4_data);
}

static int fb_notifier_callback(struct notifier_block *nb,
		unsigned long event, void *data)
{
	struct synaptics_rmi4_data *rmi4_data =
			container_of(nb, struct synaptics_rmi4_data, fb_notif);
	struct fb_event *evdata = data;
	int *blank = evdata->data;

	if (event != FB_EVENT_BLANK)
		return NOTIFY_OK;

	if (rmi4_data->old_status == *blank)
		return NOTIFY_OK;

	switch (*blank) {
	case FB_BLANK_UNBLANK:
	case FB_BLANK_VSYNC_SUSPEND:
		if (!atomic_read(&rmi4_data->ts_awake)) {
			cancel_work_sync(&rmi4_data->syna_pm_work);
			atomic_set(&rmi4_data->resume_suspend, 1);
			queue_work(rmi4_data->syna_pm_wq, &rmi4_data->syna_pm_work);
		}
		break;
	default:
		if (atomic_read(&rmi4_data->ts_awake)) {
			/* Don't allow device to sleep while suspend worker is running */
			pm_stay_awake(&rmi4_data->i2c_client->dev);
			cancel_work_sync(&rmi4_data->syna_pm_work);
			atomic_set(&rmi4_data->resume_suspend, 0);
			queue_work(rmi4_data->syna_pm_wq, &rmi4_data->syna_pm_work);
		}
	}

	rmi4_data->old_status = *blank;

	return NOTIFY_OK;
}

static int synaptics_i2c_resume(struct device *dev)
{
	unsigned long flags;

	spin_lock_irqsave(&syna_rmi4_data->isr_lock, flags);
	syna_rmi4_data->i2c_awake = true;
	spin_unlock_irqrestore(&syna_rmi4_data->isr_lock, flags);

	complete(&syna_rmi4_data->i2c_resume);

	return 0;
}

static int synaptics_i2c_suspend(struct device *dev)
{
	unsigned long flags;

	spin_lock_irqsave(&syna_rmi4_data->isr_lock, flags);
	syna_rmi4_data->i2c_awake = false;
	spin_unlock_irqrestore(&syna_rmi4_data->isr_lock, flags);

	return 0;
}

static const struct dev_pm_ops synaptics_i2c_pm_ops = {
	.resume  = synaptics_i2c_resume,
	.suspend = synaptics_i2c_suspend,
};

/**
 * synaptics_rmi4_probe()
 *
 * Called by the kernel when an association with an I2C device of the
 * same name is made (after doing i2c_add_driver).
 *
 * This funtion allocates and initializes the resources for the driver
 * as an input driver, turns on the power to the sensor, queries the
 * sensor for its supported Functions and characteristics, registers
 * the driver to the input subsystem, sets up the interrupt, handles
 * the registration of the early_suspend and late_resume functions,
 * and creates a work queue for detection of other expansion Function
 * modules.
 */
static int __devinit synaptics_rmi4_probe(struct i2c_client *client,
		const struct i2c_device_id *dev_id)
{
	int ret = 0;
	struct synaptics_rmi4_data *rmi4_data;
	const struct synaptics_dsx_platform_data *platform_data =&dsx_platformdata;

	if (!i2c_check_functionality(client->adapter,
				I2C_FUNC_SMBUS_BYTE_DATA)) {
		dev_err(&client->dev,
				"%s: SMBus byte data not supported\n",
				__func__);
		return -EIO;
	}

	if (!platform_data) {
		dev_err(&client->dev,
				"%s: No platform data found\n",
				__func__);
		return -EINVAL;
	}

	rmi4_data = kzalloc(sizeof(*rmi4_data), GFP_KERNEL);
	if (!rmi4_data) {
		dev_err(&client->dev,
				"%s: Failed to alloc mem for rmi4_data\n",
				__func__);
		return -ENOMEM;
	}

	rmi4_data->i2c_client = client;
	rmi4_data->current_page = MASK_8BIT;
	rmi4_data->board = platform_data;

	rmi4_data->i2c_read = synaptics_rmi4_i2c_read;
	rmi4_data->i2c_write = synaptics_rmi4_i2c_write;
	rmi4_data->irq_enable = synaptics_rmi4_irq_enable;
	rmi4_data->reset_device = synaptics_rmi4_reset_device;

	//init sensor size
	rmi4_data->virtual_key_height = 114;
	rmi4_data->sensor_max_x = LCD_MAX_X;
	rmi4_data->sensor_max_y = LCD_MAX_Y + 120;
	syna_lcd_ratio1 = 100;
	syna_lcd_ratio2 = 100;

	mutex_init(&rmi4_data->rmi4_io_ctrl_mutex);
	mutex_init(&rmi4_data->rmi4_reset_mutex);

	syna_rmi4_data = rmi4_data;
	client->dev.platform_data = &dsx_platformdata;
	synaptics_parse_dt(&client->dev, rmi4_data);

	if (synaptics_regulator_configure())
		goto err_set_input_dev;

	rmi4_data->regulator = vdd_regulator;

	synaptics_init_gpio(rmi4_data);

	dsx_platformdata.irq_gpio = rmi4_data->irq_gpio;
	dsx_platformdata.reset_gpio = rmi4_data->reset_gpio;
	rmi4_data->irq = rmi4_data->i2c_client->irq;
	msleep(500);
	synaptics_rmi4_get_vendorid(rmi4_data);
	msleep(150);
	synaptics_ts_init_area(rmi4_data);
	i2c_set_clientdata(client, rmi4_data);

	ret = synaptics_rmi4_set_input_dev(rmi4_data);
	if (ret) {
		dev_err(&client->dev,
				"%s: Failed to set up input device\n",
				__func__);
		goto err_set_input_dev;
	}

	atomic_set(&rmi4_data->ts_awake, 1);
	rmi4_data->fb_notif.notifier_call = fb_notifier_callback;
	ret = fb_register_client(&rmi4_data->fb_notif);
	if (ret)
		goto err_enable_irq;

	if (platform_data->gpio_config) {
		ret = platform_data->gpio_config(
				platform_data->irq_gpio,
				true);
		if (ret) {
			dev_err(&client->dev,
					"%s: Failed to configure attention GPIO\n",
					__func__);
			goto err_gpio_attn;
		}

		if (platform_data->reset_gpio >= 0) {
			ret = platform_data->gpio_config(
					platform_data->reset_gpio,
					true);
			if (ret) {
				dev_err(&client->dev,
						"%s: Failed to configure reset GPIO\n",
						__func__);
				goto err_gpio_reset;
			}
		}
	}

	synaptics_ts_init_virtual_key(rmi4_data);
	synaptics_rmi4_init_touchpanel_proc();
	atomic_set(&rmi4_data->sensor_awake, 1);

	init_completion(&rmi4_data->i2c_resume);
	spin_lock_init(&rmi4_data->isr_lock);
	wakeup_source_init(&rmi4_data->syna_isr_ws, "synaptics-isr");
	rmi4_data->i2c_awake = true;

	ret = request_threaded_irq(rmi4_data->irq, NULL,
			synaptics_rmi4_irq, platform_data->irq_flags,
			"synaptics-rmi-ts", rmi4_data);
	if (ret)
		dev_err(&client->dev,
				"%s: Failed to register irq\n",
				__func__);

	atomic_set(&rmi4_data->irq_enabled, 1);

	if (!exp_data.initialized) {
		mutex_init(&exp_data.mutex);
		INIT_LIST_HEAD(&exp_data.list);
		exp_data.initialized = true;
	}

	exp_data.workqueue = create_singlethread_workqueue("dsx_exp_workqueue");
	INIT_DELAYED_WORK(&exp_data.work, synaptics_rmi4_exp_fn_work);
	exp_data.rmi4_data = rmi4_data;
	exp_data.queue_work = true;
	queue_delayed_work(exp_data.workqueue,
			&exp_data.work,
			msecs_to_jiffies(200));

	rmi4_data->syna_pm_wq = alloc_workqueue("synaptics_pm_wq",
					WQ_HIGHPRI | WQ_NON_REENTRANT, 0);
	INIT_WORK(&rmi4_data->syna_pm_work, synaptics_rmi4_pm_main);

	rmi4_fw_module_init(true);
	while(1) {
		msleep(50);
		if (rmi4_data->bcontinue) {
			rmi4_fw_module_init(false);
			break;
		}
	}

	return ret;

err_enable_irq:
	if (platform_data->gpio_config && (platform_data->reset_gpio >= 0))
		platform_data->gpio_config(platform_data->reset_gpio, false);

err_gpio_reset:
	if (platform_data->gpio_config)
		platform_data->gpio_config(platform_data->irq_gpio, false);

err_gpio_attn:
	synaptics_rmi4_empty_fn_list(rmi4_data);
	input_unregister_device(rmi4_data->input_dev);
	rmi4_data->input_dev = NULL;

err_set_input_dev:
	if (platform_data->regulator_en) {
		regulator_disable(rmi4_data->regulator);
		regulator_put(rmi4_data->regulator);
	}

	return ret;
}

/**
 * synaptics_rmi4_remove()
 *
 * Called by the kernel when the association with an I2C device of the
 * same name is broken (when the driver is unloaded).
 *
 * This funtion terminates the work queue, stops sensor data acquisition,
 * frees the interrupt, unregisters the driver from the input subsystem,
 * turns off the power to the sensor, and frees other allocated resources.
 */
static int __devexit synaptics_rmi4_remove(struct i2c_client *client)
{
	struct synaptics_rmi4_data *rmi4_data = i2c_get_clientdata(client);
	const struct synaptics_dsx_platform_data *platform_data = rmi4_data->board;

	synaptics_rmi4_irq_enable(rmi4_data, false);

	if (platform_data->gpio_config) {
		platform_data->gpio_config(platform_data->irq_gpio, false);

		if (platform_data->reset_gpio >= 0) {
			platform_data->gpio_config(
					platform_data->reset_gpio,
					false);
		}
	}

	cancel_delayed_work_sync(&exp_data.work);
	flush_workqueue(exp_data.workqueue);
	destroy_workqueue(exp_data.workqueue);

	synaptics_rmi4_empty_fn_list(rmi4_data);

	input_unregister_device(rmi4_data->input_dev);

	if (platform_data->regulator_en) {
		regulator_disable(rmi4_data->regulator);
		regulator_put(rmi4_data->regulator);
	}

	kfree(rmi4_data);

	return 0;
}

static const struct i2c_device_id synaptics_rmi4_id_table[] = {
	{"synaptics-rmi-ts", 0},
	{ },
};
MODULE_DEVICE_TABLE(i2c, synaptics_rmi4_id_table);

static struct of_device_id synaptics_of_match_table[] = {
	{ .compatible = "synaptics,rmi-ts",},
	{ },
};

static struct i2c_driver synaptics_rmi4_driver = {
	.driver = {
		.name = "synaptics-rmi-ts",
		.owner = THIS_MODULE,
		.of_match_table = synaptics_of_match_table,
		.pm = &synaptics_i2c_pm_ops,
	},
	.probe = synaptics_rmi4_probe,
	.remove = __devexit_p(synaptics_rmi4_remove),
	.id_table = synaptics_rmi4_id_table,
};

/**
 * synaptics_rmi4_init()
 *
 * Called by the kernel during do_initcalls.
 *
 * This function registers the driver to the I2C subsystem.
 *
 */
static int __init synaptics_rmi4_init(void)
{
	return i2c_add_driver(&synaptics_rmi4_driver);
}
device_initcall(synaptics_rmi4_init);

MODULE_DESCRIPTION("Synaptics DSX I2C Touch Driver");
MODULE_LICENSE("GPL");
