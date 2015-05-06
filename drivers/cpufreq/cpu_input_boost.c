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

struct boost_policy {
	enum boost_status boost_state;
	struct delayed_work restore_work;
	unsigned int cpu;
};

static DEFINE_PER_CPU(struct boost_policy, boost_info);
static struct workqueue_struct *boost_wq;
static struct work_struct boost_work;

static DEFINE_MUTEX(boost_mutex);

static bool boost_running;
static bool freqs_available;
static unsigned int boost_freq[3];
static unsigned int boost_ms[3];
static unsigned int enabled;

static u64 last_input_time;
#define MIN_INPUT_INTERVAL (150 * USEC_PER_MSEC)

/**
 * Percentage threshold used to boost CPUs (default 30%). A higher
 * value will cause more CPUs to be boosted -- CPUs are boosted
 * when ((current_freq/max_freq) * 100) < up_threshold
 */
static unsigned int up_threshold = 30;

static void cpu_unboost(unsigned int cpu)
{
	struct boost_policy *b;

	b = &per_cpu(boost_info, cpu);
	b->boost_state = UNBOOST;
	get_online_cpus();
	if (cpu_online(cpu))
		cpufreq_update_policy(cpu);
	put_online_cpus();

	if (!cpu)
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

	/* Calculate boost duration for each CPU (CPU0 gets the longest) */
	for (cpu = 0; cpu < num_cpus_to_boost; cpu++)
		boost_ms[cpu] = ((CONFIG_NR_CPUS - cpu) * 600) - (num_cpus_to_boost * 350);

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
	for (cpu = 0; cpu < num_cpus_boosted; cpu++) {
		b = &per_cpu(boost_info, cpu);
		queue_delayed_work(boost_wq, &b->restore_work,
					msecs_to_jiffies(boost_ms[cpu]));
	}
}

static void __cpuinit cpu_restore_main(struct work_struct *work)
{
	struct boost_policy *b = container_of(work, struct boost_policy,
							restore_work.work);
	cpu_unboost(b->cpu);
}

static int cpu_do_boost(struct notifier_block *nb, unsigned long val, void *data)
{
	struct cpufreq_policy *policy = data;
	struct boost_policy *b = &per_cpu(boost_info, policy->cpu);
	unsigned int b_freq;

	if (val != CPUFREQ_ADJUST)
		return NOTIFY_OK;

	if (policy->cpu > 2)
		return NOTIFY_OK;

	mutex_lock(&boost_mutex);
	b_freq = boost_freq[policy->cpu];
	mutex_unlock(&boost_mutex);

	if (!b_freq)
		return NOTIFY_OK;

	switch (b->boost_state) {
	case UNBOOST:
		policy->min = policy->cpuinfo.min_freq;
		break;
	case BOOST:
		if (b_freq > policy->max)
			policy->min = policy->max;
		else
			policy->min = b_freq;
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

	if (boost_running || !enabled || !freqs_available)
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

/**************************** SYSFS START ****************************/
static struct kobject *cpu_input_boost_kobject;

static ssize_t boost_freqs_write(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	unsigned int freq[3], i = 0;
	int ret = sscanf(buf, "%u %u %u", &freq[0], &freq[1], &freq[2]);

	if (ret != 3)
		return -EINVAL;

	if (!freq[0] || !freq[1] || !freq[2])
		return -EINVAL;

	/* Freq order should be [high, mid, low], so always order it like that */
	mutex_lock(&boost_mutex);
	boost_freq[0] = max3(freq[0], freq[1], freq[2]);
	boost_freq[2] = min3(freq[0], freq[1], freq[2]);

	while (++i) {
		if ((freq[i] == boost_freq[0]) ||
			(freq[i] == boost_freq[2])) {
			freq[i] = 0;
			i = 0;
		} else if (freq[i]) {
			boost_freq[1] = freq[i];
			break;
		}
	}
	mutex_unlock(&boost_mutex);

	freqs_available = true;

	return size;
}

static ssize_t enabled_write(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	unsigned int data;
	int ret = sscanf(buf, "%u", &data);

	if (ret != 1)
		return -EINVAL;

	enabled = data;

	return size;
}

static ssize_t up_threshold_write(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	unsigned int data;
	int ret = sscanf(buf, "%u", &data);

	if (ret != 1)
		return -EINVAL;

	up_threshold = data;

	return size;
}

static ssize_t boost_freqs_read(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%u %u %u\n", boost_freq[0], boost_freq[1], boost_freq[2]);
}

static ssize_t enabled_read(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%u\n", enabled);
}

static ssize_t up_threshold_read(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%u\n", up_threshold);
}

static DEVICE_ATTR(boost_freqs, 0644, boost_freqs_read, boost_freqs_write);
static DEVICE_ATTR(enabled, 0644, enabled_read, enabled_write);
static DEVICE_ATTR(up_threshold, 0644, up_threshold_read, up_threshold_write);

static struct attribute *cpu_input_boost_attr[] = {
	&dev_attr_boost_freqs.attr,
	&dev_attr_enabled.attr,
	&dev_attr_up_threshold.attr,
	NULL
};

static struct attribute_group cpu_input_boost_attr_group = {
	.attrs  = cpu_input_boost_attr,
};
/**************************** SYSFS END ****************************/

static int __init cpu_input_boost_init(void)
{
	struct boost_policy *b;
	int i, ret;

	boost_wq = alloc_workqueue("cpu_input_boost_wq", WQ_HIGHPRI, 0);
	if (!boost_wq) {
		pr_err("Failed to allocate workqueue\n");
		ret = -EFAULT;
		goto err;
	}

	cpufreq_register_notifier(&cpu_do_boost_nb, CPUFREQ_POLICY_NOTIFIER);

	for (i = 0; i < min(CONFIG_NR_CPUS, 3); i++) {
		b = &per_cpu(boost_info, i);
		b->cpu = i;
		INIT_DELAYED_WORK(&b->restore_work, cpu_restore_main);
	}

	INIT_WORK(&boost_work, cpu_boost_main);

	ret = input_register_handler(&cpu_boost_input_handler);
	if (ret) {
		pr_err("Failed to register input handler, err: %d\n", ret);
		goto err;
	}

	cpu_input_boost_kobject = kobject_create_and_add("cpu_input_boost", kernel_kobj);
	if (!cpu_input_boost_kobject) {
		pr_err("Failed to create kobject\n");
		goto err;
	}

	ret = sysfs_create_group(cpu_input_boost_kobject, &cpu_input_boost_attr_group);
	if (ret) {
		pr_err("Failed to create sysfs interface\n");
		kobject_put(cpu_input_boost_kobject);
	}
err:
	return ret;
}
late_initcall(cpu_input_boost_init);

MODULE_AUTHOR("Sultanxda <sultanxda@gmail.com>");
MODULE_DESCRIPTION("CPU Input Boost");
MODULE_LICENSE("GPLv2");
