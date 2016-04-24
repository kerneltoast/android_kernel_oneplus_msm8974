/*
 * Copyright (C) 2008 Rodolfo Giometti <giometti@linux.it>
 * Copyright (C) 2008 Eurotech S.p.A. <info@eurotech.it>
 * Based on a previous work by Copyright (C) 2008 Texas Instruments, Inc.
 *
 * Copyright (c) 2011, The Linux Foundation. All rights reserved.
 *
 * Copyright (C) 2016, Sultanxda <sultanxda@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <asm/unaligned.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/qpnp-charger.h>
#include <linux/slab.h>

#define DRIVER_VERSION			"1.1.0-BACON_MSM"

/* BQ27541 standard data commands */
#define BQ27541_REG_CNTL		0x00
#define BQ27541_REG_AR			0x02
#define BQ27541_REG_ARTTE		0x04
#define BQ27541_REG_TEMP		0x06
#define BQ27541_REG_VOLT		0x08
#define BQ27541_REG_FLAGS		0x0A
#define BQ27541_REG_NAC			0x0C
#define BQ27541_REG_FAC			0x0e
#define BQ27541_REG_RM			0x10
#define BQ27541_REG_FCC			0x12
#define BQ27541_REG_AI			0x14
#define BQ27541_REG_TTE			0x16
#define BQ27541_REG_TTF			0x18
#define BQ27541_REG_SI			0x1a
#define BQ27541_REG_STTE		0x1c
#define BQ27541_REG_MLI			0x1e
#define BQ27541_REG_MLTTE		0x20
#define BQ27541_REG_AE			0x22
#define BQ27541_REG_AP			0x24
#define BQ27541_REG_TTECP		0x26
#define BQ27541_REG_SOH			0x28
#define BQ27541_REG_SOC			0x2c
#define BQ27541_REG_NIC			0x2e
#define BQ27541_REG_ICR			0x30
#define BQ27541_REG_LOGIDX		0x32
#define BQ27541_REG_LOGBUF		0x34

#define BQ27541_FLAG_DSC		BIT(0)
#define BQ27541_FLAG_FC			BIT(9)

#define BQ27541_CS_DLOGEN		BIT(15)
#define BQ27541_CS_SS		    BIT(13)

/* Control subcommands */
#define BQ27541_SUBCMD_CNTL_STATUS  0x0000
#define BQ27541_SUBCMD_DEVICE_TYPE  0x0001
#define BQ27541_SUBCMD_FW_VER  0x0002
#define BQ27541_SUBCMD_HW_VER  0x0003
#define BQ27541_SUBCMD_DF_CSUM  0x0004
#define BQ27541_SUBCMD_PREV_MACW   0x0007
#define BQ27541_SUBCMD_CHEM_ID   0x0008
#define BQ27541_SUBCMD_BD_OFFSET   0x0009
#define BQ27541_SUBCMD_INT_OFFSET  0x000a
#define BQ27541_SUBCMD_CC_VER   0x000b
#define BQ27541_SUBCMD_OCV  0x000c
#define BQ27541_SUBCMD_BAT_INS   0x000d
#define BQ27541_SUBCMD_BAT_REM   0x000e
#define BQ27541_SUBCMD_SET_HIB   0x0011
#define BQ27541_SUBCMD_CLR_HIB   0x0012
#define BQ27541_SUBCMD_SET_SLP   0x0013
#define BQ27541_SUBCMD_CLR_SLP   0x0014
#define BQ27541_SUBCMD_FCT_RES   0x0015
#define BQ27541_SUBCMD_ENABLE_DLOG  0x0018
#define BQ27541_SUBCMD_DISABLE_DLOG 0x0019
#define BQ27541_SUBCMD_SEALED   0x0020
#define BQ27541_SUBCMD_ENABLE_IT    0x0021
#define BQ27541_SUBCMD_DISABLE_IT   0x0023
#define BQ27541_SUBCMD_CAL_MODE  0x0040
#define BQ27541_SUBCMD_RESET   0x0041
#define ZERO_DEGREES_CELSIUS_IN_TENTH_KELVIN   2731
#define BQ27541_INIT_DELAY   ((HZ)*1)

#define BQ27541_CHG_CALIB_CNT   3 /* Num of calibration cycles after charging */
#define BQ27541_SOC_CRIT   41 /* SOC threshold to stop limiting SOC drop rate */

static DEFINE_MUTEX(i2c_read_mutex);
static DEFINE_SPINLOCK(i2c_pm_lock);
static DECLARE_COMPLETION(i2c_resume_done);

/* Back up and use old data if raw measurement fails */
struct bq27541_old_data {
	int curr;
	int is_charging;
	int mvolts;
	int soc;
	int temp;
};

struct bq27541_device_info {
	struct device			*dev;
	struct i2c_client		*client;
	struct delayed_work		hw_config;
	struct bq27541_old_data		old_data;
	bool suspended;
};

static struct bq27541_device_info *bq27541_di;

static bool bq27541_get_suspend_state(struct bq27541_device_info *di)
{
	unsigned long flags;
	bool suspended;

	spin_lock_irqsave(&i2c_pm_lock, flags);
	suspended = di->suspended;
	spin_unlock_irqrestore(&i2c_pm_lock, flags);

	return suspended;
}

static void bq27541_set_suspend_state(struct bq27541_device_info *di,
	bool suspended)
{
	unsigned long flags;

	spin_lock_irqsave(&i2c_pm_lock, flags);
	di->suspended = suspended;
	spin_unlock_irqrestore(&i2c_pm_lock, flags);
}

static void bq27541_wait_for_i2c(void)
{
	INIT_COMPLETION(i2c_resume_done);
	wait_for_completion_timeout(&i2c_resume_done,
					msecs_to_jiffies(100));
}

static int bq27541_read_i2c(u8 reg, int *rt_value,
			struct bq27541_device_info *di)
{
	struct i2c_client *client = di->client;
	struct i2c_msg msg[1];
	unsigned char data[2];
	int err;

	if (!client->adapter)
		return -ENODEV;

	msg->addr = client->addr;
	msg->flags = 0;
	msg->len = 1;
	msg->buf = data;

	data[0] = reg;
	err = i2c_transfer(client->adapter, msg, 1);

	if (err >= 0) {
		msg->len = 2;
		msg->flags = I2C_M_RD;

		err = i2c_transfer(client->adapter, msg, 1);
		if (err >= 0) {
			*rt_value = get_unaligned_le16(data);
			return 0;
		}
	}

	return err;
}

static int bq27541_read(u8 reg, int *rt_value,
			struct bq27541_device_info *di)
{
	bool suspended;
	int ret;

	/* Make sure I2C bus is active */
	suspended = bq27541_get_suspend_state(di);
	if (suspended) {
		pm_stay_awake(&di->client->dev);
		bq27541_wait_for_i2c();
	}

	mutex_lock(&i2c_read_mutex);
	ret = bq27541_read_i2c(reg, rt_value, di);
	mutex_unlock(&i2c_read_mutex);

	if (suspended)
		pm_relax(&di->client->dev);

	return ret;
}

/* Return the battery temperature in tenths of degrees Celsius */
static int bq27541_battery_temperature(struct bq27541_device_info *di)
{
	int ret, temp;

	ret = bq27541_read(BQ27541_REG_TEMP, &temp, di);
	if (ret) {
		dev_err(di->dev, "error reading temperature, ret: %d\n", ret);
		return di->old_data.temp;
	}

	di->old_data.temp = temp - ZERO_DEGREES_CELSIUS_IN_TENTH_KELVIN;

	return di->old_data.temp;
}

static int bq27541_battery_voltage(struct bq27541_device_info *di)
{
	int ret, volt;

	ret = bq27541_read(BQ27541_REG_VOLT, &volt, di);
	if (ret) {
		dev_err(di->dev, "error reading voltage, ret: %d\n", ret);
		return di->old_data.mvolts;
	}

	di->old_data.mvolts = volt * 1000;

	return di->old_data.mvolts;
}

static int bq27541_average_current(struct bq27541_device_info *di)
{
	int ret, curr;

	ret = bq27541_read(BQ27541_REG_AI, &curr, di);
	if (ret) {
		dev_err(di->dev, "error reading current, ret: %d\n", ret);
		return di->old_data.curr;
	}

	/* Negative current */
	if (curr & 0x8000)
		curr = -((~(curr - 1)) & 0xFFFF);

	di->old_data.curr = -curr;

	return di->old_data.curr;
}

static int bq27541_battery_soc(struct bq27541_device_info *di)
{
	int ret, soc;

	ret = bq27541_read(BQ27541_REG_SOC, &soc, di);
	if (ret) {
		dev_err(di->dev, "error reading SOC, ret: %d\n", ret);
		return di->old_data.soc;
	}

	if (soc) {
		/* Prevent SOC from quickly dropping to 99% */
		if (soc > 90 && soc < 100)
			soc++;
		/* Reserve 1% charge to be safe */
		if (soc < 50)
			soc--;
	}

	/* Double check before reporting 0% SOC */
	if (di->old_data.soc && !soc) {
		ret = di->old_data.soc;
		di->old_data.soc = soc;
		return ret;
	}

	if (!di->old_data.soc)
		di->old_data.soc = soc;

	if (soc > di->old_data.soc) {
		/*
		 * Don't raise SOC while discharging, unless this is
		 * a calibration cycle.
		 */
		int chg_status = qpnp_get_charging_status();
		if (chg_status == POWER_SUPPLY_STATUS_DISCHARGING) {
			if (di->old_data.is_charging)
				di->old_data.is_charging--;
			else
				soc = di->old_data.soc;
		} else {
			di->old_data.is_charging = BQ27541_CHG_CALIB_CNT;
		}
	} else if (soc < di->old_data.soc && (soc > BQ27541_SOC_CRIT)) {
		/*
		 * Don't force SOC to scale down by 1% during first
		 * BQ27541_CHG_CALIB_CNT discharge heartbeats after
		 * charging. This will allow SOC to quickly drop to
		 * its true value if needed.
		 */
		if (di->old_data.is_charging)
			di->old_data.is_charging--;
		else
			/* Scale down 1% at a time when above SOC crit */
			soc = di->old_data.soc - 1;
	}

	di->old_data.soc = soc;

	return di->old_data.soc;
}

/* I2C-specific code */
static int bq27541_i2c_txsubcmd(u8 reg, unsigned short subcmd,
		struct bq27541_device_info *di)
{
	struct i2c_msg msg;
	unsigned char data[3];
	int ret;

	if (!di->client)
		return -ENODEV;

	memset(data, 0, sizeof(data));
	data[0] = reg;
	data[1] = subcmd & 0x00FF;
	data[2] = (subcmd & 0xFF00) >> 8;

	msg.addr = di->client->addr;
	msg.flags = 0;
	msg.len = 3;
	msg.buf = data;

	ret = i2c_transfer(di->client->adapter, &msg, 1);
	if (ret < 0)
		return -EIO;

	return 0;
}

static void bq27541_cntl_cmd(struct bq27541_device_info *di,
				int subcmd)
{
	bq27541_i2c_txsubcmd(BQ27541_REG_CNTL, subcmd, di);
}

static int bq27541_chip_config(struct bq27541_device_info *di)
{
	int flags, ret;

	bq27541_cntl_cmd(di, BQ27541_SUBCMD_CNTL_STATUS);
	udelay(66);
	ret = bq27541_read(BQ27541_REG_CNTL, &flags, di);
	if (ret) {
		dev_err(di->dev, "error reading register %02x, ret: %d\n",
			 BQ27541_REG_CNTL, ret);
		return ret;
	}
	udelay(66);

	bq27541_cntl_cmd(di, BQ27541_SUBCMD_ENABLE_IT);
	udelay(66);

	if (!(flags & BQ27541_CS_DLOGEN)) {
		bq27541_cntl_cmd(di, BQ27541_SUBCMD_ENABLE_DLOG);
		udelay(66);
	}

	return 0;
}

static int bq27541_get_battery_mvolts(void)
{
	return bq27541_battery_voltage(bq27541_di);
}

static int bq27541_get_battery_temperature(void)
{
	return bq27541_battery_temperature(bq27541_di);
}

static int bq27541_get_battery_soc(void)
{
	return bq27541_battery_soc(bq27541_di);
}

static int bq27541_get_average_current(void)
{
	return bq27541_average_current(bq27541_di);
}

static struct qpnp_battery_gauge bq27541_batt_gauge = {
	.get_battery_mvolts		= bq27541_get_battery_mvolts,
	.get_battery_temperature	= bq27541_get_battery_temperature,
	.get_battery_soc		= bq27541_get_battery_soc,
	.get_average_current		= bq27541_get_average_current,
};

static void bq27541_hw_config(struct work_struct *work)
{
	int ret, flags = 0, type = 0, fw_ver = 0;
	struct bq27541_device_info *di;

	di = container_of(work, struct bq27541_device_info, hw_config.work);
	ret = bq27541_chip_config(di);
	if (ret) {
		dev_err(di->dev, "Failed to configure device\n");
		return;
	}

	qpnp_battery_gauge_register(&bq27541_batt_gauge);

	bq27541_cntl_cmd(di, BQ27541_SUBCMD_CNTL_STATUS);
	udelay(66);
	bq27541_read(BQ27541_REG_CNTL, &flags, di);
	bq27541_cntl_cmd(di, BQ27541_SUBCMD_DEVICE_TYPE);
	udelay(66);
	bq27541_read(BQ27541_REG_CNTL, &type, di);
	bq27541_cntl_cmd(di, BQ27541_SUBCMD_FW_VER);
	udelay(66);
	bq27541_read(BQ27541_REG_CNTL, &fw_ver, di);

	dev_info(di->dev, "DEVICE_TYPE is 0x%02X, FIRMWARE_VERSION is 0x%02X\n",
			type, fw_ver);
	dev_info(di->dev, "Completed configuration 0x%02X\n", flags);
}

static int bq27541_battery_probe(struct i2c_client *client,
				 const struct i2c_device_id *id)
{
	struct bq27541_device_info *di;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C))
		return -ENODEV;

	di = kzalloc(sizeof(*di), GFP_KERNEL);
	if (!di) {
		dev_err(&client->dev, "failed to allocate device info data\n");
		return -ENOMEM;
	}

	i2c_set_clientdata(client, di);
	di->dev = &client->dev;
	di->client = client;

	di->old_data.is_charging = BQ27541_CHG_CALIB_CNT;

	bq27541_di = di;

	/*
	 * 300ms delay is needed after bq27541 is powered up
	 * and before any successful I2C transaction
	 */
	INIT_DELAYED_WORK(&di->hw_config, bq27541_hw_config);
	schedule_delayed_work(&di->hw_config, BQ27541_INIT_DELAY);

	return 0;
}

static int bq27541_battery_remove(struct i2c_client *client)
{
	struct bq27541_device_info *di = i2c_get_clientdata(client);

	qpnp_battery_gauge_unregister(&bq27541_batt_gauge);
	bq27541_cntl_cmd(di, BQ27541_SUBCMD_DISABLE_DLOG);
	udelay(66);
	bq27541_cntl_cmd(di, BQ27541_SUBCMD_DISABLE_IT);
	cancel_delayed_work_sync(&di->hw_config);

	kfree(di);

	return 0;
}

static int bq27541_battery_resume(struct device *dev)
{
	bq27541_set_suspend_state(bq27541_di, false);

	complete_all(&i2c_resume_done);

	return 0;
}

static int bq27541_battery_suspend(struct device *dev)
{
	bq27541_set_suspend_state(bq27541_di, true);

	return 0;
}

static const struct dev_pm_ops bq27541_battery_pm_ops = {
	.resume  = bq27541_battery_resume,
	.suspend = bq27541_battery_suspend,
};

static const struct of_device_id bq27541_match[] = {
	{ .compatible = "ti,bq27541-battery" },
	{ },
};

static const struct i2c_device_id bq27541_id[] = {
	{ "bq27541", 1 },
	{ },
};
MODULE_DEVICE_TABLE(i2c, BQ27541_id);

static struct i2c_driver bq27541_battery_driver = {
	.driver		= {
			.name = "bq27541-battery",
			.owner = THIS_MODULE,
			.of_match_table = bq27541_match,
			.pm = &bq27541_battery_pm_ops,
	},
	.probe		= bq27541_battery_probe,
	.remove		= bq27541_battery_remove,
	.id_table	= bq27541_id,
};

static int __init bq27541_battery_init(void)
{
	int ret;

	ret = i2c_add_driver(&bq27541_battery_driver);
	if (ret)
		pr_err("Unable to register BQ27541 driver, ret: %d\n", ret);

	return ret;
}
module_init(bq27541_battery_init);

static void __exit bq27541_battery_exit(void)
{
	i2c_del_driver(&bq27541_battery_driver);
}
module_exit(bq27541_battery_exit);

MODULE_LICENSE("GPLv2");
MODULE_AUTHOR("Qualcomm Innovation Center, Inc.");
MODULE_DESCRIPTION("BQ27541 battery monitor driver");
