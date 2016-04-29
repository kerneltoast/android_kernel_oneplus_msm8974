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

#define pr_fmt(fmt) "cpu_iboost: " fmt

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

struct fb_policy {
	struct work_struct boost_work;
	struct delayed_work unboost_work;
	enum boost_status state;
};

struct ib_pcpu {
	struct delayed_work unboost_work;
	enum boost_status state;
	uint32_t cpu;
};

struct ib_config {
	struct ib_pcpu __percpu *boost_info;
	struct work_struct boost_work;
	bool running;
	uint64_t start_time;
	uint32_t adj_duration_ms;
	uint32_t duration_ms;
	uint32_t freq[2];
	uint32_t nr_cpus_boosted;
	uint32_t nr_cpus_to_boost;
};

struct boost_policy {
	spinlock_t lock;
	struct fb_policy fb;
	struct ib_config ib;
	struct workqueue_struct *wq;
	uint8_t enabled;
};

static struct boost_policy *boost_policy_g;

static void boost_cpu0(struct boost_policy *b);
static bool is_driver_enabled(struct boost_policy *b);
static bool is_fb_boost_active(struct boost_policy *b);
static void set_fb_state(struct boost_policy *b, enum boost_status state);
static void set_ib_status(struct boost_policy *b, bool status);
static void unboost_all_cpus(struct boost_policy *b);
static void unboost_cpu(struct ib_pcpu *pcpu);

static void ib_boost_main(struct work_struct *work)
{
	struct boost_policy *b = boost_policy_g;

	get_online_cpus();

	b->ib.nr_cpus_boosted = 0;

	/*
	 * Maximum of two CPUs can be boosted at any given time.
	 * Boost two CPUs if only one is online as it's very likely
	 * that another CPU will come online soon (due to user interaction).
	 * The next CPU to come online is the other CPU that will be boosted.
	 */
	b->ib.nr_cpus_to_boost = num_online_cpus() == 1 ? 2 : 1;

	/*
	 * Reduce the boost duration for all CPUs by a factor of
	 * (1 + num_online_cpus())/(3 + num_online_cpus()).
	 */
	b->ib.adj_duration_ms = b->ib.duration_ms * 3 /
					(3 + num_online_cpus());

	/*
	 * Only boost CPU0 from here. More than one CPU is only boosted when
	 * the 2nd CPU to boost is offline at this point in time, so the boost
	 * notifier will handle boosting the 2nd CPU if/when it comes online.
	 */
	boost_cpu0(b);

	put_online_cpus();
}

static void ib_unboost_main(struct work_struct *work)
{
	struct boost_policy *b = boost_policy_g;
	struct ib_pcpu *pcpu = container_of(work, typeof(*pcpu),
						unboost_work.work);
	uint32_t cpu;

	unboost_cpu(pcpu);

	/* Check if all boosts are finished */
	for_each_possible_cpu(cpu) {
		pcpu = per_cpu_ptr(b->ib.boost_info, cpu);
		if (pcpu->state == BOOST)
			return;
	}

	/* All input boosts are done, ready to accept new boosts now */
	set_ib_status(b, false);
}

static void fb_boost_main(struct work_struct *work)
{
	struct boost_policy *b = boost_policy_g;
	uint32_t cpu;

	/* All CPUs will be boosted to policy->max */
	set_fb_state(b, BOOST);

	/* Immediately boost the online CPUs to policy->max */
	get_online_cpus();
	for_each_online_cpu(cpu)
		cpufreq_update_policy(cpu);
	put_online_cpus();

	queue_delayed_work(b->wq, &b->fb.unboost_work,
				msecs_to_jiffies(FB_BOOST_MS));
}

static void fb_unboost_main(struct work_struct *work)
{
	struct boost_policy *b = boost_policy_g;

	set_fb_state(b, UNBOOST);
	unboost_all_cpus(b);
}

static int do_cpu_boost(struct notifier_block *nb,
		unsigned long val, void *data)
{
	struct cpufreq_policy *policy = data;
	struct boost_policy *b = boost_policy_g;
	struct ib_pcpu *pcpu = per_cpu_ptr(b->ib.boost_info, policy->cpu);

	if (val != CPUFREQ_ADJUST)
		return NOTIFY_OK;

	if (!is_driver_enabled(b) && policy->min == policy->cpuinfo.min_freq)
		return NOTIFY_OK;

	if (is_fb_boost_active(b)) {
		policy->min = policy->max;
		return NOTIFY_OK;
	}

	/* Boost previously-offline CPU */
	if (b->ib.nr_cpus_boosted < b->ib.nr_cpus_to_boost &&
		policy->cpu && !pcpu->state) {
		int32_t duration_ms = b->ib.adj_duration_ms -
			(ktime_to_ms(ktime_get()) - b->ib.start_time);
		if (duration_ms > 0) {
			pcpu->state = BOOST;
			b->ib.nr_cpus_boosted++;
			queue_delayed_work(b->wq, &pcpu->unboost_work,
						msecs_to_jiffies(duration_ms));
		}
	}

	if (pcpu->state)
		policy->min = min(policy->max,
				b->ib.freq[policy->cpu ? 1 : 0]);
	else
		policy->min = policy->cpuinfo.min_freq;

	return NOTIFY_OK;
}

static struct notifier_block do_cpu_boost_nb = {
	.notifier_call = do_cpu_boost,
};

static int fb_unblank_boost(struct notifier_block *nb,
		unsigned long val, void *data)
{
	struct boost_policy *b = boost_policy_g;
	struct fb_event *evdata = data;
	int *blank = evdata->data;

	/* Only boost for unblank (i.e. when device is woken) */
	if (val != FB_EVENT_BLANK || *blank != FB_BLANK_UNBLANK)
		return NOTIFY_OK;

	if (!is_driver_enabled(b))
		return NOTIFY_OK;

	/* Framebuffer boost is already in progress */
	if (is_fb_boost_active(b))
		return NOTIFY_OK;

	queue_work(b->wq, &b->fb.boost_work);

	return NOTIFY_OK;
}

static struct notifier_block fb_boost_nb = {
	.notifier_call = fb_unblank_boost,
};

static void cpu_ib_input_event(struct input_handle *handle, unsigned int type,
		unsigned int code, int value)
{
	struct boost_policy *b = handle->handler->private;
	bool do_boost;

	spin_lock(&b->lock);
	do_boost = b->enabled && !b->fb.state && !b->ib.running;
	spin_unlock(&b->lock);

	if (!do_boost)
		return;

	set_ib_status(b, true);

	queue_work(b->wq, &b->ib.boost_work);
}

static int cpu_ib_input_connect(struct input_handler *handler,
		struct input_dev *dev, const struct input_device_id *id)
{
	struct input_handle *handle;
	int ret;

	handle = kzalloc(sizeof(*handle), GFP_KERNEL);
	if (!handle)
		return -ENOMEM;

	handle->dev = dev;
	handle->handler = handler;
	handle->name = "cpu_ib_handle";

	ret = input_register_handle(handle);
	if (ret)
		goto err2;

	ret = input_open_device(handle);
	if (ret)
		goto err1;

	return 0;

err1:
	input_unregister_handle(handle);
err2:
	kfree(handle);
	return ret;
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
	.name		= "cpu_ib_handler",
	.id_table	= cpu_ib_ids,
};

static void boost_cpu0(struct boost_policy *b)
{
	struct ib_pcpu *pcpu = per_cpu_ptr(b->ib.boost_info, 0);

	pcpu->state = BOOST;
	b->ib.nr_cpus_boosted++;
	cpufreq_update_policy(0);
	queue_delayed_work(b->wq, &pcpu->unboost_work,
				msecs_to_jiffies(b->ib.adj_duration_ms));

	/* Record start time for use if a 2nd CPU to be boosted comes online */
	b->ib.start_time = ktime_to_ms(ktime_get());
}

static bool is_driver_enabled(struct boost_policy *b)
{
	bool ret;

	spin_lock(&b->lock);
	ret = b->enabled;
	spin_unlock(&b->lock);

	return ret;
}

static bool is_fb_boost_active(struct boost_policy *b)
{
	bool ret;

	spin_lock(&b->lock);
	ret = b->fb.state;
	spin_unlock(&b->lock);

	return ret;
}

static void set_fb_state(struct boost_policy *b, enum boost_status state)
{
	spin_lock(&b->lock);
	b->fb.state = state;
	spin_unlock(&b->lock);
}

static void set_ib_status(struct boost_policy *b, bool status)
{
	spin_lock(&b->lock);
	b->ib.running = status;
	spin_unlock(&b->lock);
}

static void unboost_all_cpus(struct boost_policy *b)
{
	struct ib_pcpu *pcpu;
	uint32_t cpu;

	get_online_cpus();
	for_each_possible_cpu(cpu) {
		pcpu = per_cpu_ptr(b->ib.boost_info, cpu);
		pcpu->state = UNBOOST;
		if (cpu_online(cpu))
			cpufreq_update_policy(cpu);
	}
	put_online_cpus();

	set_ib_status(b, false);
}

static void unboost_cpu(struct ib_pcpu *pcpu)
{
	pcpu->state = UNBOOST;
	get_online_cpus();
	if (cpu_online(pcpu->cpu))
		cpufreq_update_policy(pcpu->cpu);
	put_online_cpus();
}

static ssize_t enabled_write(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct boost_policy *b = boost_policy_g;
	uint32_t data;
	int ret;

	ret = sscanf(buf, "%u", &data);
	if (ret != 1)
		return -EINVAL;

	spin_lock(&b->lock);
	b->enabled = data;
	spin_unlock(&b->lock);

	/* Ensure that everything is stopped when returning from here */
	if (!data) {
		cancel_work_sync(&b->fb.boost_work);
		cancel_work_sync(&b->ib.boost_work);
		set_fb_state(b, UNBOOST);
		unboost_all_cpus(b);
	}

	return size;
}

static ssize_t ib_freqs_write(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct boost_policy *b = boost_policy_g;
	uint32_t freq[2];
	int ret;

	ret = sscanf(buf, "%u %u", &freq[0], &freq[1]);
	if (ret != 2)
		return -EINVAL;

	if (!freq[0] || !freq[1])
		return -EINVAL;

	/* freq[0] is assigned to CPU0, freq[1] to CPUX (X > 0) */
	b->ib.freq[0] = freq[0];
	b->ib.freq[1] = freq[1];

	return size;
}

static ssize_t ib_duration_ms_write(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct boost_policy *b = boost_policy_g;
	uint32_t ms;
	int ret;

	ret = sscanf(buf, "%u", &ms);
	if (ret != 1)
		return -EINVAL;

	if (!ms)
		return -EINVAL;

	b->ib.duration_ms = ms;

	return size;
}

static ssize_t enabled_read(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct boost_policy *b = boost_policy_g;

	return snprintf(buf, PAGE_SIZE, "%u\n", b->enabled);
}

static ssize_t ib_freqs_read(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct boost_policy *b = boost_policy_g;

	return snprintf(buf, PAGE_SIZE, "%u %u\n",
				b->ib.freq[0], b->ib.freq[1]);
}

static ssize_t ib_duration_ms_read(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct boost_policy *b = boost_policy_g;

	return snprintf(buf, PAGE_SIZE, "%u\n", b->ib.duration_ms);
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
	.attrs = cpu_ib_attr,
};

static int sysfs_ib_init(void)
{
	struct kobject *kobj;
	int ret;

	kobj = kobject_create_and_add("cpu_input_boost",
					kernel_kobj);
	if (!kobj) {
		pr_err("Failed to create kobject\n");
		return -ENOMEM;
	}

	ret = sysfs_create_group(kobj, &cpu_ib_attr_group);
	if (ret) {
		pr_err("Failed to create sysfs interface\n");
		kobject_put(kobj);
	}

	return ret;
}

static struct boost_policy *alloc_boost_policy(void)
{
	struct boost_policy *b;

	b = kzalloc(sizeof(*b), GFP_KERNEL);
	if (!b) {
		pr_err("Failed to allocate boost policy\n");
		return NULL;
	}

	b->wq = alloc_workqueue("cpu_ib_wq", WQ_HIGHPRI | WQ_NON_REENTRANT, 0);
	if (!b->wq) {
		pr_err("Failed to allocate workqueue\n");
		goto free_b;
	}

	b->ib.boost_info = alloc_percpu(typeof(*b->ib.boost_info));
	if (!b->ib.boost_info) {
		pr_err("Failed to allocate percpu definition\n");
		goto destroy_wq;
	}

	return b;

destroy_wq:
	destroy_workqueue(b->wq);
free_b:
	kfree(b);
	return NULL;
}

static int __init cpu_ib_init(void)
{
	struct boost_policy *b;
	uint32_t cpu;
	int ret;

	b = alloc_boost_policy();
	if (!b)
		return -ENOMEM;

	cpu_ib_input_handler.private = b;
	ret = input_register_handler(&cpu_ib_input_handler);
	if (ret) {
		pr_err("Failed to register input handler, err: %d\n", ret);
		goto free_mem;
	}

	ret = sysfs_ib_init();
	if (ret)
		goto input_unregister;

	cpufreq_register_notifier(&do_cpu_boost_nb, CPUFREQ_POLICY_NOTIFIER);

	INIT_DELAYED_WORK(&b->fb.unboost_work, fb_unboost_main);

	INIT_WORK(&b->fb.boost_work, fb_boost_main);

	fb_register_client(&fb_boost_nb);

	for_each_possible_cpu(cpu) {
		struct ib_pcpu *pcpu = per_cpu_ptr(b->ib.boost_info, cpu);
		pcpu->cpu = cpu;
		INIT_DELAYED_WORK(&pcpu->unboost_work, ib_unboost_main);
	}

	INIT_WORK(&b->ib.boost_work, ib_boost_main);

	spin_lock_init(&b->lock);

	/* Allow global boost config access */
	boost_policy_g = b;

	return 0;

input_unregister:
	input_unregister_handler(&cpu_ib_input_handler);
free_mem:
	free_percpu(b->ib.boost_info);
	kfree(b);
	return ret;
}
late_initcall(cpu_ib_init);
