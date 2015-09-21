/*
 * Driver for zte misc functions
 * function1: used for translate hardware GPIO to SYS GPIO number
 */

#include <linux/module.h>

#include <linux/init.h>
#include <linux/fs.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/sched.h>
#include <linux/pm.h>
#include <linux/slab.h>
#include <linux/sysctl.h>
#include <linux/proc_fs.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/workqueue.h>
#include <linux/gpio.h>
#include <linux/of_platform.h>
#include <linux/of_gpio.h>
#include <linux/delay.h>
#include <linux/power_supply.h>//for charging eanble/disable conrotl

int smb1351_is_good = 0;
module_param(smb1351_is_good, int, 0644);

struct zte_gpio_info {
	int sys_num;			//system pin number
	const char *name;
};

#define MAX_SUPPORT_GPIOS 16
struct zte_gpio_info zte_gpios[MAX_SUPPORT_GPIOS];

static struct of_device_id zte_misc_of_match[] = {
	{ .compatible = "zte-misc", },
	{ },
};
MODULE_DEVICE_TABLE(of, zte_misc_of_match);

int get_sysnumber_byname(char* name)
{
	int i;
	for (i = 0; i < MAX_SUPPORT_GPIOS; i++) {
		if (zte_gpios[i].name) {
			if (!strcmp(zte_gpios[i].name,name)) 
				return zte_gpios[i].sys_num;	
		}
	}
	return -1;
}

static int get_devtree_pdata(struct device *dev)
{
	struct device_node *node, *pp;
	int count = -1;
	pr_info("zte_misc: translate hardware pin to system pin\n");
	node = dev->of_node;
	if (node == NULL)
		return -ENODEV;

	pp = NULL;
	while ((pp = of_get_next_child(node, pp))) {
		if (!of_find_property(pp, "label", NULL)) {
			dev_warn(dev, "Found without labels\n");
			continue;
		}
		count++;
		zte_gpios[count].name = kstrdup(of_get_property(pp, "label", NULL),
								GFP_KERNEL);
		zte_gpios[count].sys_num = of_get_gpio(pp, 0);
		
		pr_info("zte_misc: sys_number=%d name=%s\n",zte_gpios[count].sys_num,zte_gpios[count].name);
	}
	return 0;
}
/*battery switch functions*/
#if 0
static int poweroff_bs = 0;
static int factory_mode = 0;
static int is_factory_mode = 0;
#define BATTERY_SWITCH 		"battery_switch"
static int gpio_batt_switch_en = -1;

static int zte_batt_switch_init(void)
{
	int rc;
	
	gpio_batt_switch_en = get_sysnumber_byname(BATTERY_SWITCH);

	rc=gpio_request(gpio_batt_switch_en, BATTERY_SWITCH);
	if (rc) {
    	pr_info("unable to request battery_switch(%d)\n",rc);
    } else {
		pr_info("success\n");
	}
	//gpio_direction_output(gpio_batt_switch_en,0);
	
	return 0;
}

/*
 *hardware will poweroff after 7.3second automatically when pin from
 *low to high (rising edge triggered) and holding it high for at least 1 ms
 */
static int poweroff_bs_set(const char *val, struct kernel_param *kp)
{
	int ret;
    
    if (gpio_batt_switch_en == -1) {
        pr_info("%s:not support battery sitch",__func__); 
        return -1;
    }

	ret = param_set_int(val, kp);

	if (ret)
		return ret;
        
	if (poweroff_bs != 1)
		return -1;
	pr_info("%s:switch down",__func__);
	gpio_direction_output(gpio_batt_switch_en,0);
	msleep(10);
	gpio_direction_output(gpio_batt_switch_en,1);
	return 0;
}
module_param_call(poweroff_bs, poweroff_bs_set, NULL,
			&poweroff_bs, 0644);

static int factory_mode_set(const char *val, struct kernel_param *kp)
{
	int ret;

	ret = param_set_int(val, kp);

	if (ret)
		return ret;
	
	if (factory_mode == 0)
		is_factory_mode = 0;
	else
		is_factory_mode = 1;

	return 0;
}

static int factory_mode_get(char *buffer, struct kernel_param *kp)
{
	return	sprintf(buffer,"%d",is_factory_mode);
}
module_param_call(factory_mode, factory_mode_set, factory_mode_get,
			&factory_mode, 0644);

int battery_switch_enable(void)
{
    if (gpio_batt_switch_en == -1) {
        pr_info("%s:not support battery sitch",__func__); 
        return -1;
    }
    
	if (!is_factory_mode)
		return -1;

	gpio_direction_output(gpio_batt_switch_en,0);
	mdelay(10);
	gpio_direction_output(gpio_batt_switch_en,1);
	mdelay(10);
	return 0;
}
EXPORT_SYMBOL(battery_switch_enable);
#endif
/*
  *Emode function to enable/disable 0% shutdown
  */
int enable_to_shutdown = 1;
static int set_enable_to_shutdown(const char *val, struct kernel_param *kp)
{
	int ret;

	ret = param_set_int(val, kp);
	if (ret) {
		pr_err("error setting value %d\n", ret);
		return ret;
	}

	pr_warn("set_enable_to_shutdown to %d\n", enable_to_shutdown);
	return 0;
}

module_param_call(enable_to_shutdown, set_enable_to_shutdown, param_get_uint,
					&enable_to_shutdown, 0644);

static int zte_misc_charging_enabled;//defined in ****-charger.c
static int zte_misc_control_charging(const char *val, struct kernel_param *kp)
{
	struct power_supply	*batt_psy;
	int rc;
	const union power_supply_propval enable = {1,};
	const union power_supply_propval disable = {0,};

	rc = param_set_int(val, kp);
	if (rc) {
		pr_err("%s: error setting value %d\n", __func__, rc);
		return rc;
	}

	batt_psy = power_supply_get_by_name("battery");
	if (batt_psy) {
		if (zte_misc_charging_enabled != 0) {
			rc = batt_psy->set_property(batt_psy,
					POWER_SUPPLY_PROP_CHARGING_ENABLED, &enable);
			pr_info("%s: enable charging\n",__func__);
		} else {
			rc = batt_psy->set_property(batt_psy,
					POWER_SUPPLY_PROP_CHARGING_ENABLED, &disable);
			pr_info("%s: disable charging\n",__func__);
		}
		if (rc) {
			pr_err("battery does not export CHARGING_ENABLED: %d\n", rc);
		}
	} else
		pr_err("%s: batt_psy is NULL\n",__func__);

	return 0;
}
static int zte_misc_get_charging_enabled_node(char *val, struct kernel_param *kp)
{
	struct power_supply	*batt_psy;
	int rc;
	union power_supply_propval pval = {0,};

	batt_psy = power_supply_get_by_name("battery");
	if (batt_psy) {
		rc = batt_psy->get_property(batt_psy,
					POWER_SUPPLY_PROP_CHARGING_ENABLED, &pval);
			pr_info("%s: enable charging status %d\n",__func__,pval.intval);
		if (rc) {
			pr_err("battery charging_enabled node is not exist: %d\n", rc);
            return	sprintf(val,"%d",-1);
		}
        zte_misc_charging_enabled = pval.intval;
	} else
		pr_err("%s: batt_psy is NULL\n",__func__);

	return	sprintf(val,"%d",pval.intval);
}

module_param_call(charging_enabled, zte_misc_control_charging, zte_misc_get_charging_enabled_node,
					&zte_misc_charging_enabled, 0644);

//static int __devinit zte_misc_probe(struct platform_device *pdev)
static int zte_misc_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	int error;
	
	pr_info("%s +++++\n",__func__);
	
	error = get_devtree_pdata(dev);
	if (error)
		return error;
    
    //zte_batt_switch_init();

	pr_info("%s ----\n",__func__);
	return 0;
}

//static int __devexit zte_misc_remove(struct platform_device *pdev)
static int  zte_misc_remove(struct platform_device *pdev)
{
	return 0;
}

static struct platform_driver zte_misc_device_driver = {
	.probe		= zte_misc_probe,
	//.remove	= __devexit_p(zte_misc_remove),  //zte jiangzhineng changed
	.remove    = zte_misc_remove,
	.driver		= {
		.name	= "zte-misc",
		.owner	= THIS_MODULE,
		.of_match_table = zte_misc_of_match,
	}
};

int __init zte_misc_init(void)
{
	return platform_driver_register(&zte_misc_device_driver);
}

static void __exit zte_misc_exit(void)
{
	platform_driver_unregister(&zte_misc_device_driver);
}
fs_initcall(zte_misc_init);
module_exit(zte_misc_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Misc driver for zte");
MODULE_ALIAS("platform:zte-misc");
