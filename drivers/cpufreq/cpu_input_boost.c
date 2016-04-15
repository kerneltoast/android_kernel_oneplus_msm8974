/*
 * Copyright (C) 2014-2016, Sultanxda <sultanxda@gmail.com>
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

#define pr_fmt(fmt) "CPU-iboost: " fmt

#include <linux/cpu.h>
#include <linux/cpufreq.h>
#include <linux/fb.h>
#include <linux/input.h>
#include <linux/slab.h>

#define FB_BOOST_MS 1100

enum boost_status {
	UNBOOST,
	BOOST,
};

struct boost_policy {
	struct delayed_work ib_unboost_work;
	enum boost_status boost_state;
	unsigned int cpu;
};

static DEFINE_PER_CPU(struct boost_policy, boost_info);
static struct workqueue_struct *boost_wq;
static struct work_struct fb_boost_work;
static struct delayed_work fb_unboost_work;
static struct work_struct ib_boost_work;
static spinlock_t boost_lock;

static bool ib_running;
static enum boost_status fb_boost;
static u64 boost_start_time;
static unsigned int enabled;
static unsigned int ib_adj_duration_ms;
static unsigned int ib_duration_ms;
static unsigned int ib_freq[2];
static unsigned int ib_nr_cpus_boosted;
static unsigned int ib_nr_cpus_to_boost;

static bool fb_boost_active(void)
{
	bool ret;

	spin_lock(&boost_lock);
	ret = fb_boost;
	spin_unlock(&boost_lock);

	return ret;
}

static void set_ib_state(bool state)
{
	spin_lock(&boost_lock);
	ib_running = state;
	spin_unlock(&boost_lock);
}

static void set_fb_boost_state(enum boost_status state)
{
	spin_lock(&boost_lock);
	fb_boost = state;
	spin_unlock(&boost_lock);
}

static void boost_cpu0(unsigned int duration_ms)
{
	struct boost_policy *b = &per_cpu(boost_info, 0);

	b->boost_state = BOOST;
	ib_nr_cpus_boosted++;
	cpufreq_update_policy(0);
	queue_delayed_work(boost_wq, &b->ib_unboost_work,
				msecs_to_jiffies(duration_ms));

	/* Record start time for use if a 2nd CPU to be boosted comes online */
	boost_start_time = ktime_to_ms(ktime_get());
}

/* Unboost function for input boost */
static void unboost_cpu(unsigned int cpu)
{
	struct boost_policy *b = &per_cpu(boost_info, cpu);

	b->boost_state = UNBOOST;
	get_online_cpus();
	if (cpu_online(b->cpu))
		cpufreq_update_policy(b->cpu);
	put_online_cpus();
}

static void unboost_all_cpus(void)
{
	struct boost_policy *b;
	unsigned int cpu;

	get_online_cpus();
	for_each_possible_cpu(cpu) {
		b = &per_cpu(boost_info, cpu);
		b->boost_state = UNBOOST;
		if (cpu_online(cpu))
			cpufreq_update_policy(cpu);
	}
	put_online_cpus();

	set_ib_state(false);
}

/* Stops everything and unboosts all CPUs */
static void stop_all_boosts(void)
{
	/* Make sure input-boost and framebuffer boost are not running */
	cancel_work_sync(&fb_boost_work);
	cancel_work_sync(&ib_boost_work);

	set_fb_boost_state(UNBOOST);
	unboost_all_cpus();
}

/* Main input boost worker */
static void __cpuinit ib_boost_main(struct work_struct *work)
{
	get_online_cpus();

	ib_nr_cpus_boosted = 0;

	/*
	 * Maximum of two CPUs can be boosted at any given time.
	 * Boost two CPUs if only one is online as it's very likely
	 * that another CPU will come online soon (due to user interaction).
	 * The next CPU to come online is the other CPU that will be boosted.
	 */
	ib_nr_cpus_to_boost = num_online_cpus() == 1 ? 2 : 1;

	/*
	 * Reduce the boost duration for all CPUs by a factor of
	 * (1 + num_online_cpus())/(3 + num_online_cpus()).
	 */
	ib_adj_duration_ms = ib_duration_ms * 3 / (3 + num_online_cpus());

	/*
	 * Only boost CPU0 from here. More than one CPU is only boosted when
	 * the 2nd CPU to boost is offline at this point in time, so the boost
	 * notifier will handle boosting the 2nd CPU if/when it comes online.
	 */
	boost_cpu0(ib_adj_duration_ms);

	put_online_cpus();
}

/* Main unboost worker for input boost */
static void __cpuinit ib_unboost_main(struct work_struct *work)
{
	struct boost_policy *b = container_of(work, struct boost_policy,
							ib_unboost_work.work);
	unsigned int cpu;

	unboost_cpu(b->cpu);

	/* Check if all boosts are finished */
	for_each_possible_cpu(cpu) {
		b = &per_cpu(boost_info, cpu);
		if (b->boost_state == BOOST)
			return;
	}

	/* All input boosts are done, ready to accept new boosts now */
	set_ib_state(false);
}

/* Framebuffer-boost worker */
static void __cpuinit fb_boost_main(struct work_struct *work)
{
	unsigned int cpu;

	/* All CPUs will be boosted to policy->max */
	set_fb_boost_state(BOOST);

	/* Immediately boost the online CPUs to policy->max */
	get_online_cpus();
	for_each_online_cpu(cpu)
		cpufreq_update_policy(cpu);
	put_online_cpus();

	queue_delayed_work(boost_wq, &fb_unboost_work,
				msecs_to_jiffies(FB_BOOST_MS));
}

/* Framebuffer-unboost worker */
static void __cpuinit fb_unboost_main(struct work_struct *work)
{
	set_fb_boost_state(UNBOOST);
	unboost_all_cpus();
}

/* Notifier used to actually perform the boosts/unboosts */
static int do_cpu_boost(struct notifier_block *nb, unsigned long val, void *data)
{
	struct cpufreq_policy *policy = data;
	struct boost_policy *b = &per_cpu(boost_info, policy->cpu);

	if (!enabled && policy->min == policy->cpuinfo.min_freq)
		return NOTIFY_OK;

	if (val != CPUFREQ_ADJUST)
		return NOTIFY_OK;

	if (fb_boost_active()) {
		policy->min = policy->max;
		return NOTIFY_OK;
	}

	/* Boost previously-offline CPU */
	if (ib_nr_cpus_boosted < ib_nr_cpus_to_boost &&
		policy->cpu && !b->boost_state) {
		int duration_ms = ib_adj_duration_ms -
			(ktime_to_ms(ktime_get()) - boost_start_time);
		if (duration_ms > 0) {
			b->boost_state = BOOST;
			ib_nr_cpus_boosted++;
			queue_delayed_work(boost_wq, &b->ib_unboost_work,
						msecs_to_jiffies(duration_ms));
		}
	}

	if (b->boost_state)
		policy->min = min(policy->max, ib_freq[policy->cpu ? 1 : 0]);
	else
		policy->min = policy->cpuinfo.min_freq;

	return NOTIFY_OK;
}

static struct notifier_block do_cpu_boost_nb = {
	.notifier_call = do_cpu_boost,
};

/* Framebuffer-boost notifier; CPU is boosted when device wakes up */
static int fb_unblank_boost(struct notifier_block *nb, unsigned long val, void *data)
{
	struct fb_event *evdata = data;
	int *blank = evdata->data;

	if (!enabled)
		return NOTIFY_OK;

	/* Only boost on fb blank events */
	if (val != FB_EVENT_BLANK)
		return NOTIFY_OK;

	/* Only boost for unblank */
	if (*blank != FB_BLANK_UNBLANK)
		return NOTIFY_OK;

	/* Framebuffer boost is already in progress */
	if (fb_boost_active())
		return NOTIFY_OK;

	queue_work(boost_wq, &fb_boost_work);

	return NOTIFY_OK;
}

static struct notifier_block fb_boost_nb = {
	.notifier_call = fb_unblank_boost,
};

/* Input event handler; this is where input boosts start */
static void cpu_ib_input_event(struct input_handle *handle, unsigned int type,
		unsigned int code, int value)
{
	bool do_boost;

	spin_lock(&boost_lock);
	do_boost = !ib_running && enabled && !fb_boost;
	spin_unlock(&boost_lock);

	if (!do_boost)
		return;

	/* Indicate that an input-boost event is in progress */
	set_ib_state(true);

	queue_work(boost_wq, &ib_boost_work);
}

static int cpu_ib_input_connect(struct input_handler *handler,
		struct input_dev *dev, const struct input_device_id *id)
{
	struct input_handle *handle;
	int error;

	handle = kzalloc(sizeof(struct input_handle), GFP_KERNEL);
	if (!handle)
		return -ENOMEM;

	handle->dev = dev;
	handle->handler = handler;
	handle->name = "cpu_iboost";

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

static void cpu_ib_input_disconnect(struct input_handle *handle)
{
	input_close_device(handle);
	input_unregister_handle(handle);
	kfree(handle);
}

static const struct input_device_id cpu_ib_ids[] = {
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

static struct input_handler cpu_ib_input_handler = {
	.event		= cpu_ib_input_event,
	.connect	= cpu_ib_input_connect,
	.disconnect	= cpu_ib_input_disconnect,
	.name		= "cpu_iboost",
	.id_table	= cpu_ib_ids,
};

/**************************** SYSFS START ****************************/
static struct kobject *cpu_ib_kobject;

static ssize_t enabled_write(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	unsigned int data;
	int ret = sscanf(buf, "%u", &data);

	if (ret != 1)
		return -EINVAL;

	spin_lock(&boost_lock);
	enabled = data;
	spin_unlock(&boost_lock);

	/* Ensure that everything is stopped when returning from here */
	if (!enabled)
		stop_all_boosts();

	return size;
}

static ssize_t ib_freqs_write(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	unsigned int freq[2];
	int ret = sscanf(buf, "%u %u", &freq[0], &freq[1]);

	if (ret != 2)
		return -EINVAL;

	if (!freq[0] || !freq[1])
		return -EINVAL;

	/* ib_freq[0] is assigned to CPU0, ib_freq[1] to CPUX (X > 0) */
	ib_freq[0] = freq[0];
	ib_freq[1] = freq[1];

	return size;
}

static ssize_t ib_duration_ms_write(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	unsigned int ms;
	int ret = sscanf(buf, "%u", &ms);

	if (ret != 1)
		return -EINVAL;

	if (!ms)
		return -EINVAL;

	ib_duration_ms = ms;

	return size;
}

static ssize_t enabled_read(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%u\n", enabled);
}

static ssize_t ib_freqs_read(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%u %u\n",
				ib_freq[0], ib_freq[1]);
}

static ssize_t ib_duration_ms_read(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%u\n", ib_duration_ms);
}

static DEVICE_ATTR(enabled, 0644,
			enabled_read, enabled_write);
static DEVICE_ATTR(ib_freqs, 0644,
			ib_freqs_read, ib_freqs_write);
static DEVICE_ATTR(ib_duration_ms, 0644,
			ib_duration_ms_read, ib_duration_ms_write);

static struct attribute *cpu_ib_attr[] = {
	&dev_attr_enabled.attr,
	&dev_attr_ib_freqs.attr,
	&dev_attr_ib_duration_ms.attr,
	NULL
};

static struct attribute_group cpu_ib_attr_group = {
	.attrs  = cpu_ib_attr,
};
/**************************** SYSFS END ****************************/

static int __init cpu_ib_init(void)
{
	struct boost_policy *b;
	int cpu, ret;

	boost_wq = alloc_workqueue("cpu_ib_wq", WQ_HIGHPRI | WQ_NON_REENTRANT, 0);
	if (!boost_wq) {
		pr_err("Failed to allocate workqueue\n");
		ret = -EFAULT;
		goto err;
	}

	cpufreq_register_notifier(&do_cpu_boost_nb, CPUFREQ_POLICY_NOTIFIER);

	INIT_DELAYED_WORK(&fb_unboost_work, fb_unboost_main);

	INIT_WORK(&fb_boost_work, fb_boost_main);

	fb_register_client(&fb_boost_nb);

	for_each_possible_cpu(cpu) {
		b = &per_cpu(boost_info, cpu);
		b->cpu = cpu;
		INIT_DELAYED_WORK(&b->ib_unboost_work, ib_unboost_main);
	}

	INIT_WORK(&ib_boost_work, ib_boost_main);

	spin_lock_init(&boost_lock);

	ret = input_register_handler(&cpu_ib_input_handler);
	if (ret) {
		pr_err("Failed to register input handler, err: %d\n", ret);
		goto err;
	}

	cpu_ib_kobject = kobject_create_and_add("cpu_input_boost", kernel_kobj);
	if (!cpu_ib_kobject) {
		pr_err("Failed to create kobject\n");
		goto err;
	}

	ret = sysfs_create_group(cpu_ib_kobject, &cpu_ib_attr_group);
	if (ret) {
		pr_err("Failed to create sysfs interface\n");
		kobject_put(cpu_ib_kobject);
	}
err:
	return ret;
}
late_initcall(cpu_ib_init);
