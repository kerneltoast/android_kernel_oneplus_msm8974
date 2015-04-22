/*
 * Copyright (C) 2014-2015, Sultanxda <sultanxda@gmail.com>
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

#define pr_fmt(fmt) "CPU-boost: " fmt

#include <linux/cpu.h>
#include <linux/cpufreq.h>
#include <linux/hrtimer.h>
#include <linux/input.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/notifier.h>
#include <linux/slab.h>

enum boost_status {
	UNBOOST,
	BOOST,
};

enum boost_pwr {
	LOW,
	MID,
	HIGH,
};

struct boost_policy {
	enum boost_status boost_state;
};

static DEFINE_PER_CPU(struct boost_policy, boost_info);
static struct workqueue_struct *boost_wq;
static struct work_struct boost_work;
static struct delayed_work restore_work;

static bool boost_running;

static u64 last_input_time;
#define MIN_INPUT_INTERVAL (150 * USEC_PER_MSEC)

/**
 * Auto boost freq calculation:
 * Requested boost freqs = maxfreq * boost_factor[i] / BOOST_FACTOR_DIVISOR,
 * so the lowest boost freq in this case would be maxfreq * 3 / 7
 */
static unsigned int boost_freq[3];
static unsigned int boost_factor[3] = {3, 4, 5};
#define BOOST_FACTOR_DIVISOR 7

/* Boost-freq level to use (high, mid, low) */
static enum boost_pwr boost_level;

/* Boost duration in millsecs */
static unsigned int boost_ms;

/* On/off switch */
static unsigned int enabled;
module_param(enabled, uint, 0644);

/**
 * Percentage threshold used to boost CPUs (default 30%). A higher
 * value will cause more CPUs to be boosted -- CPUs are boosted
 * when ((current_freq/max_freq) * 100) < up_threshold
 */
static unsigned int up_threshold = 30;
module_param(up_threshold, uint, 0644);

static void cpu_unboost_all(void)
{
	struct boost_policy *b;
	unsigned int cpu;

	get_online_cpus();
	for_each_possible_cpu(cpu) {
		b = &per_cpu(boost_info, cpu);
		if (b->boost_state == BOOST) {
			b->boost_state = UNBOOST;
			if (cpu_online(cpu))
				cpufreq_update_policy(cpu);
		}
	}
	put_online_cpus();
	boost_running = false;
}

static void __cpuinit cpu_boost_main(struct work_struct *work)
{
	struct boost_policy *b;
	struct cpufreq_policy *policy;
	unsigned int cpu, num_cpus_boosted = 0, num_cpus_to_boost = 0;

	/* Num of CPUs to be boosted based on current freq of each online CPU */
	get_online_cpus();
	for_each_online_cpu(cpu) {
		policy = cpufreq_cpu_get(cpu);
		if (policy != NULL) {
			if ((policy->cur * 100 / policy->max) < up_threshold)
				num_cpus_to_boost++;
			cpufreq_cpu_put(policy);
			/* Only allow 2 CPUs to be staged for boosting from here */
			if (num_cpus_to_boost == 2)
				break;
		}
	}

	/* Num of CPUs to be boosted based on how many of them are online */
	switch (num_online_cpus() * 100 / CONFIG_NR_CPUS) {
	case 25:
		num_cpus_to_boost += 2;
		break;
	case 50 ... 75:
		num_cpus_to_boost++;
		break;
	}

	/* Nothing to boost */
	if (!num_cpus_to_boost) {
		put_online_cpus();
		boost_running = false;
		return;
	}

	/* Boost freq to use based on how many CPUs to boost */
	switch (num_cpus_to_boost * 100 / CONFIG_NR_CPUS) {
	case 25:
		boost_level = HIGH;
		break;
	case 50:
		boost_level = MID;
		break;
	default:
		boost_level = LOW;
	}

	/* Dual-core systems need more power */
	if (CONFIG_NR_CPUS == 2)
		boost_level++;

	/* Calculate boost duration */
	boost_ms = 3000 - ((num_cpus_to_boost * 750) + ((boost_level + 1) * 250));

	/* Prioritize boosting of online CPUs */
	for_each_online_cpu(cpu) {
		b = &per_cpu(boost_info, cpu);
		b->boost_state = BOOST;
		cpufreq_update_policy(cpu);
		num_cpus_boosted++;
		if (num_cpus_boosted == num_cpus_to_boost)
			goto finish_boost;
	}

	/* Boost offline CPUs if we still need to boost more CPUs */
	for_each_possible_cpu(cpu) {
		b = &per_cpu(boost_info, cpu);
		if (b->boost_state == UNBOOST) {
			b->boost_state = BOOST;
			num_cpus_boosted++;
			if (num_cpus_boosted == num_cpus_to_boost)
				goto finish_boost;
		}
	}

finish_boost:
	put_online_cpus();
	queue_delayed_work(boost_wq, &restore_work,
				msecs_to_jiffies(boost_ms));
}

static void __cpuinit cpu_restore_main(struct work_struct *work)
{
	cpu_unboost_all();
}

static int cpu_do_boost(struct notifier_block *nb, unsigned long val, void *data)
{
	struct cpufreq_policy *policy = data;
	struct boost_policy *b = &per_cpu(boost_info, policy->cpu);

	if (val != CPUFREQ_ADJUST)
		return NOTIFY_OK;

	switch (b->boost_state) {
	case UNBOOST:
		policy->min = policy->cpuinfo.min_freq;
		break;
	case BOOST:
		if (boost_freq[boost_level] > policy->max)
			policy->min = policy->max;
		else
			policy->min = boost_freq[boost_level];
		break;
	}

	return NOTIFY_OK;
}

static struct notifier_block cpu_do_boost_nb = {
	.notifier_call = cpu_do_boost,
};

static void cpu_boost_input_event(struct input_handle *handle, unsigned int type,
		unsigned int code, int value)
{
	u64 now;

	if (boost_running)
		return;
	if (!enabled)
		return;

	now = ktime_to_us(ktime_get());
	if (now - last_input_time < MIN_INPUT_INTERVAL)
		return;

	boost_running = true;
	queue_work(boost_wq, &boost_work);
	last_input_time = ktime_to_us(ktime_get());
}

static int cpu_boost_input_connect(struct input_handler *handler,
		struct input_dev *dev, const struct input_device_id *id)
{
	struct input_handle *handle;
	int error;

	handle = kzalloc(sizeof(struct input_handle), GFP_KERNEL);
	if (!handle)
		return -ENOMEM;

	handle->dev = dev;
	handle->handler = handler;
	handle->name = "cpu_input_boost";

	error = input_register_handle(handle);
	if (error)
		goto err2;

	error = input_open_device(handle);
	if (error)
		goto err1;

	return 0;
err1:
	input_unregister_handle(handle);
err2:
	kfree(handle);
	return error;
}

static void cpu_boost_input_disconnect(struct input_handle *handle)
{
	input_close_device(handle);
	input_unregister_handle(handle);
	kfree(handle);
}

static const struct input_device_id cpu_boost_ids[] = {
	/* multi-touch touchscreen */
	{
		.flags = INPUT_DEVICE_ID_MATCH_EVBIT |
			INPUT_DEVICE_ID_MATCH_ABSBIT,
		.evbit = { BIT_MASK(EV_ABS) },
		.absbit = { [BIT_WORD(ABS_MT_POSITION_X)] =
			BIT_MASK(ABS_MT_POSITION_X) |
			BIT_MASK(ABS_MT_POSITION_Y) },
	},
	/* touchpad */
	{
		.flags = INPUT_DEVICE_ID_MATCH_KEYBIT |
			INPUT_DEVICE_ID_MATCH_ABSBIT,
		.keybit = { [BIT_WORD(BTN_TOUCH)] = BIT_MASK(BTN_TOUCH) },
		.absbit = { [BIT_WORD(ABS_X)] =
			BIT_MASK(ABS_X) | BIT_MASK(ABS_Y) },
	},
	{ },
};

static struct input_handler cpu_boost_input_handler = {
	.event		= cpu_boost_input_event,
	.connect	= cpu_boost_input_connect,
	.disconnect	= cpu_boost_input_disconnect,
	.name		= "cpu_input_boost",
	.id_table	= cpu_boost_ids,
};

static int __init cpu_input_boost_init(void)
{
	struct cpufreq_frequency_table *table = cpufreq_frequency_get_table(0);
	int maxfreq = cpufreq_quick_get_max(0);
	int b_level = 0, req_freq[3];
	int i, ret = 1;

	if (!maxfreq) {
		pr_err("Failed to get max freq, input boost disabled\n");
		goto err;
	}

	/* Calculate ideal boost freqs */
	for (i = 0; i < 3; i++)
		req_freq[i] = maxfreq * boost_factor[i] / BOOST_FACTOR_DIVISOR;

	/* Find actual freqs closest to ideal boost freqs */
	for (i = 0;; i++) {
		int curr = table[i].frequency - req_freq[b_level];
		int prev = table[i ? i - 1 : 0].frequency - req_freq[b_level];

		if (!curr || (curr > 0 && prev < 0)) {
			boost_freq[b_level] = table[i].frequency;
			b_level++;
			if (b_level == 3)
				break;
		}
	}

	boost_wq = alloc_workqueue("cpu_input_boost_wq", WQ_HIGHPRI | WQ_NON_REENTRANT, 0);
	if (!boost_wq) {
		pr_err("Failed to allocate workqueue\n");
		ret = -EFAULT;
		goto err;
	}

	cpufreq_register_notifier(&cpu_do_boost_nb, CPUFREQ_POLICY_NOTIFIER);

	INIT_DELAYED_WORK(&restore_work, cpu_restore_main);
	INIT_WORK(&boost_work, cpu_boost_main);

	ret = input_register_handler(&cpu_boost_input_handler);
	if (ret)
		pr_err("Failed to register input handler, err: %d\n", ret);
err:
	return ret;
}
late_initcall(cpu_input_boost_init);

MODULE_AUTHOR("Sultanxda <sultanxda@gmail.com>");
MODULE_DESCRIPTION("CPU Input Boost");
MODULE_LICENSE("GPLv2");
