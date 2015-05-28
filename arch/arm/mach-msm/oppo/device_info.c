/**
 * Copyright 2008-2013 OPPO Mobile Comm Corp., Ltd, All rights reserved.
 * CONFIG_MACH_MSM8974_14001:
 * FileName:devinfo.c
 * ModuleName:devinfo
 * Author: wangjc
 * Create Date: 2013-10-23
 * Description:add interface to get device information.
 * History:
   <version >  <time>  <author>  <desc>
   1.0		2013-10-23	wangjc	init
*/

#include <linux/module.h>
#include <linux/proc_fs.h>
#include <mach/device_info.h>


enum{
	MSM_BOOT_MODE__NORMAL,
	MSM_BOOT_MODE__RECOVERY = 2, //the number adapt system/core/init/init.c
	MSM_BOOT_MODE__FACTORY,
	MSM_BOOT_MODE__RF,
	MSM_BOOT_MODE__WLAN,
	MSM_BOOT_MODE__MOS,
};

static int ftm_mode = MSM_BOOT_MODE__NORMAL;

int __init board_mfg_mode_init(void)
{	
    char *substr;

    substr = strstr(boot_command_line, "oppo_ftm_mode=");
    if(substr) {
        substr += strlen("oppo_ftm_mode=");
        if(strncmp(substr, "factory2", 5) == 0)
            ftm_mode = MSM_BOOT_MODE__FACTORY;
        else if(strncmp(substr, "ftmwifi", 5) == 0)
            ftm_mode = MSM_BOOT_MODE__WLAN;
		else if(strncmp(substr, "ftmmos", 5) == 0)
            ftm_mode = MSM_BOOT_MODE__MOS;
        else if(strncmp(substr, "ftmrf", 5) == 0)
            ftm_mode = MSM_BOOT_MODE__RF;
        else if(strncmp(substr, "ftmrecovery", 5) == 0)
            ftm_mode = MSM_BOOT_MODE__RECOVERY;
    }

	pr_err("board_mfg_mode_init, " "ftm_mode=%d\n", ftm_mode);

	return 0;
}

char pwron_event[16];

int __init start_reason_init(void)
{
    int i;
    char * substr = strstr(boot_command_line, "androidboot.startupmode="); 
    substr += strlen("androidboot.startupmode=");
    for(i=0; substr[i] != ' '; i++) {
        pwron_event[i] = substr[i];
    }
    pwron_event[i] = '\0';
    
    printk(KERN_INFO "%s: parse poweron reason %s\n", __func__, pwron_event);
	
	return 1;
}

char boot_mode[16];
int __init boot_mode_init(void)
{
	int i;
	char *substr = strstr(boot_command_line, "androidboot.mode=");
	substr += strlen("androidboot.mode=");
	for(i = 0; substr[i] != ' '; i++) {
		boot_mode[i] = substr[i];
	}
	boot_mode[i] = '\0';

	printk(KERN_INFO "%s: parse boot_mode is %s\n", __func__, boot_mode);
	return 1;
}


static struct proc_dir_entry *parent = NULL;

static int device_proc_output (char *buf, struct manufacture_info *priv)
{
	char *p = buf;

	p += sprintf(p, "Device version:\t\t%s\n",
		     priv->version);

	p += sprintf(p, "Device manufacture:\t\t%s\n",
			priv->manufacture);

	return p - buf;
}

static int device_read_proc(char *page, char **start, off_t off,
			   int count, int *eof, void *data)
{
	struct manufacture_info *priv = data;
	
	int len = device_proc_output (page, priv);
	if (len <= off+count)
		*eof = 1;
	*start = page + off;
	len -= off;
	if (len > count)
		len = count;
	if (len < 0)
		len = 0;
	return len;
}

int register_device_proc(char *name, char *version, char *manufacture)
{
	struct proc_dir_entry *d_entry;
	struct manufacture_info *info;

	if(!parent) {
		parent =  proc_mkdir ("devinfo", NULL);
		if(!parent) {
			pr_err("can't create devinfo proc\n");
			return -ENOENT;
		}
	}

	info = kzalloc(sizeof *info, GFP_KERNEL);
	info->version = version;
	info->manufacture = manufacture;

	d_entry = create_proc_read_entry (name, S_IRUGO, parent, device_read_proc, info);
	if(!d_entry) {
		pr_err("create %s proc failed.\n", name);
		kfree(info);
		return -ENOENT;
	}
	return 0;
}

static int __init device_info_init(void)
{
	int ret = 0;
	
	parent =  proc_mkdir ("devinfo", NULL);
	if(!parent) {
		pr_err("can't create devinfo proc\n");
		ret = -ENOENT;
	}
	
	return ret;
}

static void __exit device_info_exit(void)
{

	remove_proc_entry("devinfo", NULL);

}

module_init(device_info_init);
module_exit(device_info_exit);


MODULE_DESCRIPTION("OPPO device info");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Wangjc <wjc@oppo.com>");


