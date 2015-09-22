/*
 * Copyright (C) 2014-2015, Sultanxda <sultanxda@gmail.com>
 *
 * Copyright (C) 2015, The Linux Foundation. All rights reserved.
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
#include <linux/kthread.h>
#include <linux/module.h>
#include <linux/sched.h>
#include <linux/slab.h>

#define FB_BOOST_MS 500

enum boost_status {
	UNBOOST,
	WAITING,
	BOOST,
};

struct boost_policy {
	struct delayed_work ib_restore_work;
	struct delayed_work mig_boost_rem;
	struct task_struct *thread;
	bool pending;
	int src_cpu;
	unsigned int boost_state;
	unsigned int cpu;
	unsigned int migration_freq;
	unsigned int task_load;
	atomic_t being_woken;
	spinlock_t lock;
	wait_queue_head_t sync_wq;
};

static DEFINE_PER_CPU(struct boost_policy, boost_info);
static struct workqueue_struct *boost_wq;
static struct delayed_work fb_boost_work;
static struct work_struct boost_work;

static bool ib_running;
static bool load_based_syncs;
static bool suspended;
static unsigned int boost_ms[4];
static unsigned int ib_freq[4];
static unsigned int enabled;
static unsigned int fb_boost;
static unsigned int migration_boost_ms;
static unsigned int migration_load_threshold;
static unsigned int sync_threshold;

/* Boost function for input boost */
static void cpu_boost_cpu(unsigned int cpu)
{
	struct boost_policy *b = &per_cpu(boost_info, cpu);

	b->boost_state = BOOST;
	cpufreq_update_policy(cpu);
	queue_delayed_work(boost_wq, &b->ib_restore_work,
				msecs_to_jiffies(boost_ms[cpu]));
}

/* Unboost function for input boost */
static void cpu_unboost_cpu(unsigned int cpu)
{
	struct boost_policy *b = &per_cpu(boost_info, cpu);

	b->boost_state = UNBOOST;
	get_online_cpus();
	if (cpu_online(b->cpu))
		cpufreq_update_policy(b->cpu);
	put_online_cpus();
}

static void cpu_unboost_all(void)
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

	ib_running = false;
}

/* Stops everything and unboosts all CPUs */
static void stop_remove_all_boosts(void)
{
	struct boost_policy *b;
	unsigned int cpu;

	cancel_work_sync(&boost_work);
	cancel_delayed_work_sync(&fb_boost_work);
	fb_boost = UNBOOST;
	for_each_possible_cpu(cpu) {
		b = &per_cpu(boost_info, cpu);
		cancel_delayed_work_sync(&b->mig_boost_rem);
		cancel_delayed_work_sync(&b->ib_restore_work);
		b->migration_freq = 0;
	}

	cpu_unboost_all();
}

/* Input boost main boost function */
static void __cpuinit ib_boost_main(struct work_struct *work)
{
	unsigned int cpu, nr_cpus_to_boost, nr_boosted = 0;

	get_online_cpus();
	/* Max. of 3 CPUs can be boosted at any given time */
	nr_cpus_to_boost = num_online_cpus() > 2 ? num_online_cpus() - 1 : 1;

	for_each_online_cpu(cpu) {
		/* Calculate boost duration for each CPU (CPU0 is boosted the longest) */
		/* TODO: Make this more standard and configurable from sysfs */
		boost_ms[cpu] = 1500 - (cpu * 200) - (nr_cpus_to_boost * 250);
		cpu_boost_cpu(cpu);
		nr_boosted++;
		if (nr_boosted == nr_cpus_to_boost)
			break;
	}
	put_online_cpus();
}

/* Input boost main restore function */
static void __cpuinit ib_restore_main(struct work_struct *work)
{
	struct boost_policy *b = container_of(work, struct boost_policy,
							ib_restore_work.work);
	cpu_unboost_cpu(b->cpu);

	if (!b->cpu)
		ib_running = false;
}

/* Framebuffer boost function */
static void __cpuinit fb_boost_fn(struct work_struct *work)
{
	unsigned int cpu;

	if (fb_boost == BOOST) {
		get_online_cpus();
		for_each_online_cpu(cpu)
			cpufreq_update_policy(cpu);
		put_online_cpus();
		fb_boost = WAITING;
		queue_delayed_work(boost_wq, &fb_boost_work,
					msecs_to_jiffies(FB_BOOST_MS));
	} else {
		fb_boost = UNBOOST;
		cpu_unboost_all();
	}
}

/*
 * Boost hierarchy: there are three kinds of boosts, and some
 * boosts will take precedence over others. Below is the current
 * hierarchy, from most precedence to least precedence:
 *
 * 1. Framebuffer blank/unblank boost
 * 2. Thread-migration boost (only if the mig boost freq > policy->min)
 * 3. Input boost
 */
static int cpu_do_boost(struct notifier_block *nb, unsigned long val, void *data)
{
	struct cpufreq_policy *policy = data;
	struct boost_policy *b = &per_cpu(boost_info, policy->cpu);

	if (val == CPUFREQ_START) {
		set_cpus_allowed(b->thread, *cpumask_of(b->cpu));
		return NOTIFY_OK;
	}

	if (val != CPUFREQ_ADJUST)
		return NOTIFY_OK;

	switch (b->boost_state) {
	case UNBOOST:
		policy->min = policy->cpuinfo.min_freq;
		break;
	case BOOST:
		policy->min = min(policy->max, ib_freq[policy->cpu]);
		break;
	}

	if (b->migration_freq > policy->min)
		policy->min = min(policy->max, b->migration_freq);

	if (fb_boost)
		policy->min = policy->max;

	return NOTIFY_OK;
}

static struct notifier_block cpu_do_boost_nb = {
	.notifier_call = cpu_do_boost,
};

/* Framebuffer notifier callback function */
static int fb_blank_boost(struct notifier_block *nb, unsigned long val, void *data)
{
	struct fb_event *evdata = data;
	int *blank = evdata->data;

	if (!enabled)
		return NOTIFY_OK;

	/* Only boost on fb blank events */
	if (val != FB_EVENT_BLANK)
		return NOTIFY_OK;

	/* Record suspend state for migration notifier */
	if (*blank != FB_BLANK_UNBLANK) {
		suspended = true;
		/* Only boost for unblank */
		return NOTIFY_OK;
	} else
		suspended = false;

	/* Framebuffer boost is already in progress */
	if (fb_boost)
		return NOTIFY_OK;

	fb_boost = BOOST;
	queue_delayed_work(boost_wq, &fb_boost_work, 0);

	return NOTIFY_OK;
}

static struct notifier_block fb_boost_nb = {
	.notifier_call = fb_blank_boost,
};

/* Worker used to remove thread-migration boost */
static void do_mig_boost_rem(struct work_struct *work)
{
	struct boost_policy *b = container_of(work, struct boost_policy,
							mig_boost_rem.work);
	b->migration_freq = 0;
	cpufreq_update_policy(b->cpu);
}

static int boost_mig_sync_thread(void *data)
{
	int dest_cpu = (int)data;
	int src_cpu, ret;
	struct boost_policy *b = &per_cpu(boost_info, dest_cpu);
	struct cpufreq_policy dest_policy;
	struct cpufreq_policy src_policy;
	unsigned long flags;
	unsigned int req_freq;

	while (1) {
		wait_event_interruptible(b->sync_wq, b->pending ||
					kthread_should_stop());

		if (kthread_should_stop())
			break;

		spin_lock_irqsave(&b->lock, flags);
		b->pending = false;
		src_cpu = b->src_cpu;
		spin_unlock_irqrestore(&b->lock, flags);

		ret = cpufreq_get_policy(&src_policy, src_cpu);
		if (ret)
			continue;

		ret = cpufreq_get_policy(&dest_policy, dest_cpu);
		if (ret)
			continue;

		req_freq = max((dest_policy.max * b->task_load) / 100,
							src_policy.cur);

		if (req_freq <= dest_policy.cpuinfo.min_freq) {
			pr_debug("No sync. Sync Freq:%u\n", req_freq);
			continue;
		}

		if (sync_threshold)
			req_freq = min(sync_threshold, req_freq);

		cancel_delayed_work_sync(&b->mig_boost_rem);

		b->migration_freq = req_freq;

		/* Force policy re-evaluation to trigger adjust notifier. */
		get_online_cpus();
		if (cpu_online(src_cpu))
			/*
			 * Send an unchanged policy update to the source
			 * CPU. Even though the policy isn't changed from
			 * its existing boosted or non-boosted state
			 * notifying the source CPU will let the governor
			 * know a boost happened on another CPU and that it
			 * should re-evaluate the frequency at the next timer
			 * event without interference from a min sample time.
			 */
			cpufreq_update_policy(src_cpu);
		if (cpu_online(dest_cpu)) {
			cpufreq_update_policy(dest_cpu);
			queue_delayed_work_on(dest_cpu, boost_wq,
				&b->mig_boost_rem, msecs_to_jiffies(migration_boost_ms));
		} else
			b->migration_freq = 0;
		put_online_cpus();
	}

	return 0;
}

static int boost_migration_notify(struct notifier_block *nb,
				unsigned long unused, void *arg)
{
	struct migration_notify_data *mnd = arg;
	struct boost_policy *b = &per_cpu(boost_info, mnd->dest_cpu);
	unsigned long flags;

	if (!enabled || !migration_boost_ms)
		return NOTIFY_OK;

	/* Don't boost while suspended or during fb blank/unblank */
	if (suspended || fb_boost)
		return NOTIFY_OK;

	if (load_based_syncs && (mnd->load <= migration_load_threshold))
		return NOTIFY_OK;

	if (load_based_syncs && ((mnd->load < 0) || (mnd->load > 100))) {
		pr_err("Invalid load: %d\n", mnd->load);
		return NOTIFY_OK;
	}

	/* Avoid deadlock in try_to_wake_up() */
	if (b->thread == current)
		return NOTIFY_OK;

	pr_debug("Migration: CPU%d --> CPU%d\n", mnd->src_cpu, mnd->dest_cpu);
	spin_lock_irqsave(&b->lock, flags);
	b->pending = true;
	b->src_cpu = mnd->src_cpu;
	b->task_load = load_based_syncs ? mnd->load : 0;
	spin_unlock_irqrestore(&b->lock, flags);
	/*
	* Avoid issuing recursive wakeup call, as sync thread itself could be
	* seen as migrating triggering this notification. Note that sync thread
	* of a cpu could be running for a short while with its affinity broken
	* because of CPU hotplug.
	*/
	if (!atomic_cmpxchg(&b->being_woken, 0, 1)) {
		wake_up(&b->sync_wq);
		atomic_set(&b->being_woken, 0);
	}

	return NOTIFY_OK;
}

static struct notifier_block boost_migration_nb = {
	.notifier_call = boost_migration_notify,
};

static void cpu_iboost_input_event(struct input_handle *handle, unsigned int type,
		unsigned int code, int value)
{
	if (ib_running || !enabled || fb_boost)
		return;

	ib_running = true;
	queue_work(boost_wq, &boost_work);
}

static int cpu_iboost_input_connect(struct input_handler *handler,
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

static void cpu_iboost_input_disconnect(struct input_handle *handle)
{
	input_close_device(handle);
	input_unregister_handle(handle);
	kfree(handle);
}

static const struct input_device_id cpu_iboost_ids[] = {
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

static struct input_handler cpu_iboost_input_handler = {
	.event		= cpu_iboost_input_event,
	.connect	= cpu_iboost_input_connect,
	.disconnect	= cpu_iboost_input_disconnect,
	.name		= "cpu_iboost",
	.id_table	= cpu_iboost_ids,
};

/**************************** SYSFS START ****************************/
static struct kobject *cpu_iboost_kobject;

static ssize_t ib_freqs_write(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	unsigned int freq[3];
	int ret = sscanf(buf, "%u %u %u", &freq[0], &freq[1], &freq[2]);

	if (ret != 3)
		return -EINVAL;

	if (!freq[0] || !freq[1] || !freq[2])
		return -EINVAL;

	/* ib_freq[0] is assigned to CPU0, ib_freq[1] to CPU1, and so on */
	ib_freq[0] = freq[0];
	ib_freq[1] = freq[1];
	ib_freq[2] = freq[2];
	/* Use same freq for CPU2 & CPU3 (as only 3 cores may be boosted at once) */
	ib_freq[3] = freq[2];

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

	if (!data)
		stop_remove_all_boosts();

	return size;
}

static ssize_t load_based_syncs_write(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	unsigned int data;
	int ret = sscanf(buf, "%u", &data);

	if (ret != 1)
		return -EINVAL;

	load_based_syncs = data;

	return size;
}

static ssize_t migration_boost_ms_write(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	unsigned int data;
	int ret = sscanf(buf, "%u", &data);

	if (ret != 1)
		return -EINVAL;

	migration_boost_ms = data;

	return size;
}

static ssize_t migration_load_threshold_write(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	unsigned int data;
	int ret = sscanf(buf, "%u", &data);

	if (ret != 1)
		return -EINVAL;

	migration_load_threshold = data;

	return size;
}

static ssize_t sync_threshold_write(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	unsigned int data;
	int ret = sscanf(buf, "%u", &data);

	if (ret != 1)
		return -EINVAL;

	sync_threshold = data;

	return size;
}

static ssize_t ib_freqs_read(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%u %u %u\n",
			ib_freq[0], ib_freq[1], ib_freq[2]);
}

static ssize_t enabled_read(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%u\n", enabled);
}

static ssize_t load_based_syncs_read(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%u\n", load_based_syncs);
}

static ssize_t migration_boost_ms_read(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%u\n", migration_boost_ms);
}

static ssize_t migration_load_threshold_read(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%u\n", migration_load_threshold);
}

static ssize_t sync_threshold_read(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%u\n", sync_threshold);
}

static DEVICE_ATTR(ib_freqs, 0644,
			ib_freqs_read, ib_freqs_write);
static DEVICE_ATTR(enabled, 0644,
			enabled_read, enabled_write);
static DEVICE_ATTR(load_based_syncs, 0644,
			load_based_syncs_read, load_based_syncs_write);
static DEVICE_ATTR(migration_boost_ms, 0644,
			migration_boost_ms_read, migration_boost_ms_write);
static DEVICE_ATTR(migration_load_threshold, 0644,
			migration_load_threshold_read, migration_load_threshold_write);
static DEVICE_ATTR(sync_threshold, 0644,
			sync_threshold_read, sync_threshold_write);

static struct attribute *cpu_iboost_attr[] = {
	&dev_attr_ib_freqs.attr,
	&dev_attr_enabled.attr,
	&dev_attr_load_based_syncs.attr,
	&dev_attr_migration_boost_ms.attr,
	&dev_attr_migration_load_threshold.attr,
	&dev_attr_sync_threshold.attr,
	NULL
};

static struct attribute_group cpu_iboost_attr_group = {
	.attrs  = cpu_iboost_attr,
};
/**************************** SYSFS END ****************************/

static int __init cpu_iboost_init(void)
{
	struct boost_policy *b;
	int cpu, ret;

	boost_wq = alloc_workqueue("cpu_iboost_wq", WQ_HIGHPRI, 0);
	if (!boost_wq) {
		pr_err("Failed to allocate workqueue\n");
		ret = -EFAULT;
		goto err;
	}

	cpufreq_register_notifier(&cpu_do_boost_nb, CPUFREQ_POLICY_NOTIFIER);

	INIT_DELAYED_WORK(&fb_boost_work, fb_boost_fn);

	fb_register_client(&fb_boost_nb);

	for_each_possible_cpu(cpu) {
		b = &per_cpu(boost_info, cpu);
		b->cpu = cpu;
		INIT_DELAYED_WORK(&b->ib_restore_work, ib_restore_main);
		init_waitqueue_head(&b->sync_wq);
		atomic_set(&b->being_woken, 0);
		spin_lock_init(&b->lock);
		INIT_DELAYED_WORK(&b->mig_boost_rem, do_mig_boost_rem);
		b->thread = kthread_run(boost_mig_sync_thread, (void *)cpu,
					"boost_sync/%d", cpu);
		set_cpus_allowed(b->thread, *cpumask_of(cpu));
	}

	atomic_notifier_chain_register(&migration_notifier_head, &boost_migration_nb);

	INIT_WORK(&boost_work, ib_boost_main);

	ret = input_register_handler(&cpu_iboost_input_handler);
	if (ret) {
		pr_err("Failed to register input handler, err: %d\n", ret);
		goto err;
	}

	cpu_iboost_kobject = kobject_create_and_add("cpu_input_boost", kernel_kobj);
	if (!cpu_iboost_kobject) {
		pr_err("Failed to create kobject\n");
		goto err;
	}

	ret = sysfs_create_group(cpu_iboost_kobject, &cpu_iboost_attr_group);
	if (ret) {
		pr_err("Failed to create sysfs interface\n");
		kobject_put(cpu_iboost_kobject);
	}
err:
	return ret;
}
late_initcall(cpu_iboost_init);

MODULE_AUTHOR("Sultanxda <sultanxda@gmail.com>");
MODULE_DESCRIPTION("CPU Input Boost");
MODULE_LICENSE("GPLv2");
