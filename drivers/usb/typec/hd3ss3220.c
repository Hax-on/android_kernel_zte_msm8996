/*
 * hd3ss3220.c -- TI HD3SS3220 USB TYPE-C Controller device driver
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

#include <linux/init.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/kernel.h>
#include <linux/slab.h>

#include <linux/of_device.h>
#include <linux/of_irq.h>
#include <linux/of_gpio.h>
#include <linux/gpio.h>

#include <linux/interrupt.h>
#include <linux/i2c.h>

#include <linux/errno.h>
#include <linux/err.h>

//#include <linux/platform_device.h>
//#include <linux/workqueue.h>
//#include <linux/switch.h>
//#include <linux/timer.h>
//#include <linux/wakelock.h>
//#include <linux/delay.h>
//#include <linux/types.h>


/******************************************************************************
* Register addresses
******************************************************************************/
/* 0x00 - 0x07 reserved */
#define REG_MOD                     0x08
#define REG_INT                     0x09
#define REG_SET                     0x0A
#define REG_CTL                     0X45

/******************************************************************************
* Register bits
******************************************************************************/
/*    REG_MOD (0x08)    */
#define MOD_ACTIVE_CABLE_DETECTION  0X01    /*RU*/
#define MOD_ACCESSORY_CONNECTED_SHIFT   1
#define MOD_ACCESSORY_CONNECTED         (0x07 << MOD_ACCESSORY_CONNECTED_SHIFT)    /*RU*/
#define MOD_CURRENT_MODE_DETECT_SHIFT    4
#define MOD_CURRENT_MODE_DETECT         (0x03 << MOD_CURRENT_MODE_DETECT_SHIFT)    /*RU*/
#define MOD_CURRENT_MODE_ADVERTISE_SHIFT    6
#define MOD_CURRENT_MODE_ADVERTISE          (0x03 << MOD_CURRENT_MODE_ADVERTISE_SHIFT)    /*RW*/

/*    REG_INT (0x09)    */
#define INT_DRP_DUTY_CYCLE_SHIFT    1
#define INT_DRP_DUTY_CYCLE          (0x03 << INT_DRP_DUTY_CYCLE_SHIFT)      /*RW*/
#define INT_VCONN_FAULT_SHIFT       3
#define INT_VCONN_FAULT             (0x01 << INT_VCONN_FAULT_SHIFT)         /*RCU*/
#define INT_INTERRUPT_STATUS_SHIFT  4
#define INT_INTERRUPT_STATUS        (0x01 << INT_INTERRUPT_STATUS_SHIFT)    /*RCU*/
#define INT_CABLE_DIR_SHIFT         5
#define INT_CABLE_DIR               (0x01 << INT_CABLE_DIR_SHIFT)           /*RU*/
#define INT_ATTACHED_STATE_SHIFT    6
#define INT_ATTACHED_STATE          (0x03 << INT_ATTACHED_STATE_SHIFT)      /*RU*/

/*    REG_SET (0x0A)    */
#define SET_I2C_SOFT_RESET_SHIFT    3
#define SET_I2C_SOFT_RESET          (0x01 << SET_I2C_SOFT_RESET_SHIFT)  /*RSU*/
#define SET_MODE_SELECT_SHIFT       4
#define SET_MODE_SELECT             (0x03 << SET_MODE_SELECT_SHIFT)     /*RW*/
#define SET_DEBOUNCE_SHIFT          6
#define SET_DEBOUNCE                (0x03 << SET_DEBOUNCE_SHIFT)        /*RW*/

/*    REG_CTR (0x45)    */
#define CTR_DISABLE_RD_RP_SHIFT     2
#define CTR_DISABLE_RD_RP           (0x01 << CTR_DISABLE_RD_RP_SHIFT)   /*RW*/

/******************************************************************************
* Register values
******************************************************************************/
/* SET_MODE_SELECT */
#define SET_MODE_SELECT_HARDWARE  0x00
#define SET_MODE_SELECT_SNK       0x01
#define SET_MODE_SELECT_SRC       0x02
#define SET_MODE_SELECT_DRP       0x03
/* MOD_CURRENT_MODE_ADVERTISE */
#define MOD_CURRENT_MODE_ADVERTISE_DEFAULT      0x00
#define MOD_CURRENT_MODE_ADVERTISE_MID          0x01
#define MOD_CURRENT_MODE_ADVERTISE_HIGH         0x02
/* MOD_CURRENT_MODE_DETECT */
#define MOD_CURRENT_MODE_DETECT_DEFAULT      0x00
#define MOD_CURRENT_MODE_DETECT_MID          0x01
#define MOD_CURRENT_MODE_DETECT_ACCESSARY    0x02
#define MOD_CURRENT_MODE_DETECT_HIGH         0x03


/******************************************************************************
* Constants
******************************************************************************/

enum drp_toggle_type {
    TOGGLE_DFP_DRP_30 = 0,
    TOGGLE_DFP_DRP_40,
    TOGGLE_DFP_DRP_50,
    TOGGLE_DFP_DRP_60
};

enum current_adv_type {
    HOST_CUR_USB = 0,   /*default 500mA or 900mA*/
    HOST_CUR_1P5,      /*1.5A*/
    HOST_CUR_3A       /*3A*/
};

enum current_det_type {
    DET_CUR_USB = 0,    /*default 500mA or 900mA*/
    DET_CUR_1P5,
    DET_CUR_ACCESSORY,  /*charg through accessory 500mA*/
    DET_CUR_3A
};

enum accessory_attach_type {
    ACCESSORY_NOT_ATTACHED = 0,
    ACCESSORY_AUDIO = 4,
    ACCESSORY_CHG_THRU_AUDIO = 5,
    ACCESSORY_DEBUG = 6
};

enum cable_attach_type {
    CABLE_NOT_ATTACHED = 0,
    CABLE_ATTACHED
};

enum cable_state_type {
    CABLE_STATE_NOT_ATTACHED = 0,
    CABLE_STATE_AS_DFP,
    CABLE_STATE_AS_UFP,
    CABLE_STATE_TO_ACCESSORY
};

enum cable_dir_type {
    ORIENT_CC2 = 0,
    ORIENT_CC1
};

enum vconn_fault_type {
    VCONN_NO_FAULT = 0,
    VCONN_FAULT
};

enum cc_modes_type {
    MODE_UNKNOWN = 0,
    MODE_PORT_PIN,   /*According to hardware port pin*/
    MODE_UFP,
    MODE_DFP,
    MODE_DRP
};

/* Type-C Attrs */
struct type_c_parameters {
    enum current_det_type current_det;         /*charging current on UFP*/
    enum accessory_attach_type accessory_attach;     /*if an accessory is attached*/
    enum cable_attach_type active_cable_attach;         /*if an active_cable is attached*/
    enum cable_state_type attach_state;        /*DFP->UFP or UFP->DFP*/
    enum cable_dir_type cable_dir;           /*cc1 or cc2*/
    enum vconn_fault_type vconn_fault;         /*vconn fault*/
};

struct hd3ss3220_info {
    struct i2c_client  *i2c;
    struct device  *dev_t;
    struct mutex  mutex;
    struct class  *device_class;

    struct pinctrl  *pinctrl;
    struct pinctrl_state  *cc_int_cfg;

    int irq_gpio;

    struct type_c_parameters type_c_param;
};

/* i2c operate interfaces */
static int hd3ss3220_read_reg(struct i2c_client *i2c, u8 reg, u8 *dest)
{
    struct hd3ss3220_info *info = i2c_get_clientdata(i2c);
    int ret;

    mutex_lock(&info->mutex);
    ret = i2c_smbus_read_byte_data(i2c, reg);
    mutex_unlock(&info->mutex);
    if (ret < 0) {
        pr_err("%s: (0x%x) error, ret(%d)\n", __func__, reg, ret);
        return ret;
    }

    ret &= 0xff;
    *dest = ret;
    return 0;
}

/*
static int hd3ss3220_write_reg(struct i2c_client *i2c, u8 reg, u8 value)
{
    struct hd3ss3220_info *info = i2c_get_clientdata(i2c);
    int ret;

    mutex_lock(&info->mutex);
    ret = i2c_smbus_write_byte_data(i2c, reg, value);
    mutex_unlock(&info->mutex);
    if (ret < 0)
        pr_err("%s: (0x%x) error, ret(%d)\n", __func__, reg, ret);

    return ret;
}
*/

static int hd3ss3220_update_reg(struct i2c_client *i2c, u8 reg, u8 val, u8 mask)
{
    struct hd3ss3220_info *info = i2c_get_clientdata(i2c);
    int ret;
    u8 old_val, new_val;

    mutex_lock(&info->mutex);
    ret = i2c_smbus_read_byte_data(i2c, reg);

    if (ret >= 0) {
        old_val = ret & 0xff;
        new_val = (val & mask) | (old_val & (~mask));
        ret = i2c_smbus_write_byte_data(i2c, reg, new_val);
    }
    mutex_unlock(&info->mutex);
    return ret;
}

/* Config DFP/UFP/DRP mode */
/* e.g #echo 1 >/sys/class/type-c/hd3ss3220/mode_select */
static ssize_t mode_select_show(struct device *dev,
                    struct device_attribute *attr, char *buf)
{
    struct hd3ss3220_info *info = dev_get_drvdata(dev);
    u8 value;
    int ret;

    ret = hd3ss3220_read_reg(info->i2c, REG_SET, &value);
    if (ret < 0){
        pr_err("%s: read reg fail!\n", __func__);
        return ret;
    }
    value = (value & SET_MODE_SELECT) >> SET_MODE_SELECT_SHIFT;
    return snprintf(buf, PAGE_SIZE, "%d\n", value);
}

static ssize_t mode_select_store(struct device *dev,
                    struct device_attribute *attr, const char *buf, size_t size)
{
    struct hd3ss3220_info *info = dev_get_drvdata(dev);
    int mode;
    u8 value;
    int ret;

    if (sscanf(buf, "%d", &mode) != 1)
        return -EINVAL;

    if (mode == MODE_DFP)
        value = SET_MODE_SELECT_SRC;
    else if (mode == MODE_UFP)
        value = SET_MODE_SELECT_SNK;
    else if (mode == MODE_DRP)
        value = SET_MODE_SELECT_DRP;
    else if (mode == MODE_PORT_PIN)
        value = SET_MODE_SELECT_HARDWARE;
    else
        return -EINVAL;

    value = value << SET_MODE_SELECT_SHIFT;

    ret = hd3ss3220_update_reg(info->i2c, REG_SET, value, SET_MODE_SELECT);
    if (ret < 0){
        pr_err("%s: update reg fail!\n", __func__);
    }
    return ret;
}


/* Advertise current when act as DFP */
static ssize_t current_advertise_show(struct device *dev,
                    struct device_attribute *attr, char *buf)
{
    struct hd3ss3220_info *info = dev_get_drvdata(dev);
    u8 value;
    int ret;

    ret = hd3ss3220_read_reg(info->i2c, REG_MOD, &value);
    if (ret < 0){
        pr_err("%s: read reg fail!\n", __func__);
        return ret;
    }
    value = (value & MOD_CURRENT_MODE_ADVERTISE) >> MOD_CURRENT_MODE_ADVERTISE_SHIFT;
    return snprintf(buf, PAGE_SIZE, "%d\n", value);
}

static ssize_t current_advertise_store(struct device *dev,
                    struct device_attribute *attr, const char *buf, size_t size)
{
    struct hd3ss3220_info *info = dev_get_drvdata(dev);
    int mode;
    u8 value;
    int ret;

    if (sscanf(buf, "%d", &mode) != 1)
        return -EINVAL;

    if (mode == HOST_CUR_USB)
        value = MOD_CURRENT_MODE_ADVERTISE_DEFAULT;
    else if (mode == HOST_CUR_1P5)
        value = MOD_CURRENT_MODE_ADVERTISE_MID;
    else if (mode == HOST_CUR_3A)
        value = MOD_CURRENT_MODE_ADVERTISE_HIGH;
    else
        return -EINVAL;

    value = value << MOD_CURRENT_MODE_ADVERTISE_SHIFT;

    ret = hd3ss3220_update_reg(info->i2c, REG_MOD, value, MOD_CURRENT_MODE_ADVERTISE);
    if (ret < 0){
        pr_err("%s: update reg fail!\n", __func__);
    }
    return ret;
}

/* Detct current when act as UFP */
static ssize_t current_detect_show(struct device *dev,
                    struct device_attribute *attr, char *buf)
{
    struct hd3ss3220_info *info = dev_get_drvdata(dev);
    u8 value;
    int ret;

    ret = hd3ss3220_read_reg(info->i2c, REG_MOD, &value);
    if (ret < 0){
        pr_err("%s: read reg fail!\n", __func__);
        return ret;
    }
    value = (value & MOD_CURRENT_MODE_DETECT) >> MOD_CURRENT_MODE_DETECT_SHIFT;

    if (value == MOD_CURRENT_MODE_DETECT_DEFAULT)
        return snprintf(buf, PAGE_SIZE, "500mA or 900mA\n");
    else if (value == MOD_CURRENT_MODE_DETECT_MID)
        return snprintf(buf, PAGE_SIZE, "mid 1P5A\n");
    else if (value == MOD_CURRENT_MODE_DETECT_HIGH)
        return snprintf(buf, PAGE_SIZE, "high 3A\n");
    else if (value == MOD_CURRENT_MODE_DETECT_ACCESSARY)
        return snprintf(buf, PAGE_SIZE, "accessary 500mA\n");
    else
        return snprintf(buf, PAGE_SIZE, "unknown\n");
}

/**************************************************************************/
#define TYPE_C_ATTR(field, format_string)    \
static ssize_t      \
field ## _show(struct device *dev, struct device_attribute *attr,   \
        char *buf)               \
{                                   \
    struct hd3ss3220_info *info = dev_get_drvdata(dev); \
    return snprintf(buf, PAGE_SIZE,     \
        format_string, info->type_c_param.field);   \
}                   \
static DEVICE_ATTR(field, S_IRUGO, field ## _show, NULL);

static DEVICE_ATTR(mode_select, S_IRUGO | S_IWUSR,
                    mode_select_show, mode_select_store);
static DEVICE_ATTR(current_advertise, S_IRUGO | S_IWUSR,
                    current_advertise_show, current_advertise_store);
static DEVICE_ATTR(current_det_string, S_IRUGO, current_detect_show, NULL);

TYPE_C_ATTR(current_det, "%d\n")
TYPE_C_ATTR(accessory_attach, "%d\n")
TYPE_C_ATTR(active_cable_attach, "%d\n")
TYPE_C_ATTR(attach_state, "%d\n")
TYPE_C_ATTR(cable_dir, "%d\n")
TYPE_C_ATTR(vconn_fault, "%d\n")

static struct device_attribute *usb_typec_attributes[] = {
    &dev_attr_mode_select,
    &dev_attr_current_advertise,
    &dev_attr_current_det_string,
    &dev_attr_current_det,
    &dev_attr_accessory_attach,
    &dev_attr_active_cable_attach,
    &dev_attr_attach_state,
    &dev_attr_cable_dir,
    &dev_attr_vconn_fault,
    /*end*/
    NULL
};
/******************************************************************************/

static void process_mode_register(struct hd3ss3220_info *info, u8 status)
{
    u8 val;
    u8 tmp = status;

    /* check current_detect */
    val = ((tmp & MOD_CURRENT_MODE_DETECT) >> MOD_CURRENT_MODE_DETECT_SHIFT);
    info->type_c_param.current_det = val;

    /* check accessory attch */
    tmp = status;
    val = ((tmp & MOD_ACCESSORY_CONNECTED) >> MOD_ACCESSORY_CONNECTED_SHIFT);
    info->type_c_param.accessory_attach = val;

    /* check cable attach */
    tmp = status;
    val = (tmp & MOD_ACTIVE_CABLE_DETECTION);
    info->type_c_param.active_cable_attach = val;
}

static void process_interrupt_register(struct hd3ss3220_info *info, u8 status)
{
    u8 val;
    u8 tmp = status;

    /* check attach state */
    val = ((tmp & INT_ATTACHED_STATE) >> INT_ATTACHED_STATE_SHIFT);
    info->type_c_param.attach_state = val;

    /* check cable dir */
    tmp = status;
    val = ((tmp & INT_CABLE_DIR) >> INT_CABLE_DIR_SHIFT);
    info->type_c_param.cable_dir = val;

    /* check vconn fault */
    tmp = status;
    val = ((tmp & INT_VCONN_FAULT) >> INT_VCONN_FAULT_SHIFT);
    info->type_c_param.vconn_fault = val;
}

static irqreturn_t hd3ss3220_irq_thread(int irq, void *handle)
{
    struct hd3ss3220_info *info = (struct hd3ss3220_info *)handle;
    u8 reg_val;
    int ret;

    pr_debug("%s enter\n", __func__);

    ret = hd3ss3220_read_reg(info->i2c, REG_MOD, &reg_val);
    if (ret)
        goto done;

    pr_debug("%s mode_register:0x%x\n", __func__, reg_val);
    process_mode_register(info, reg_val);

    ret = hd3ss3220_read_reg(info->i2c, REG_INT, &reg_val);
    if (ret)
        goto done;

    pr_debug("%s interrupt_register:0x%x\n", __func__, reg_val);
    process_interrupt_register(info, reg_val);

    hd3ss3220_update_reg(info->i2c, REG_INT,(0x1 << INT_INTERRUPT_STATUS_SHIFT),INT_INTERRUPT_STATUS);
done:
    return IRQ_HANDLED;
}

static int  hd3ss3220_suspend(struct i2c_client *client, pm_message_t message)
{
    /* PM interface, fix me if need */
    return 0;
}

static int  hd3ss3220_resume(struct i2c_client *client)
{
    /* PM interface, fix me if need */
    return 0;
}

static int hd3ss3220_pinctrl_init(struct hd3ss3220_info *info)
{
    struct pinctrl_state *set_state;

    info->pinctrl = devm_pinctrl_get(&info->i2c->dev);
    if (IS_ERR_OR_NULL(info->pinctrl)) {
        pr_err("%s: pinctrl not defined\n", __func__);
        return PTR_ERR(info->pinctrl);
    }

    pr_debug("%s: doing pinctrl\n", __func__);
    set_state = pinctrl_lookup_state(info->pinctrl, "hd3ss3220_pin_cfg");
    if (IS_ERR_OR_NULL(set_state)) {
        pr_err("%s: pinctrl lookup failed\n", __func__);
        info->pinctrl = NULL;
        return PTR_ERR(set_state);
    }
    info->cc_int_cfg = set_state;

    /* get more pins conig here, if need */
    return 0;
}

static int hd3ss3220_initialization(struct hd3ss3220_info *info)
{
    int ret = 0;

    /* do initialization here, before enable irq,
     * clear irq,
     * config DRP/UFP/DFP mode,
     * and etc..
     */

    return ret;
}

static int hd3ss3220_create_device(struct hd3ss3220_info *info)
{
    struct device_attribute **attrs = usb_typec_attributes;
    struct device_attribute *attr;
    int err;

    pr_debug("%s:\n", __func__);
    info->device_class = class_create(THIS_MODULE, "type-c");
    if (IS_ERR(info->device_class))
        return PTR_ERR(info->device_class);

    info->dev_t = device_create(info->device_class, NULL, 0, NULL, "hd3ss3220");
    if (IS_ERR(info->dev_t))
        return PTR_ERR(info->dev_t);

    dev_set_drvdata(info->dev_t, info);

    while ((attr = *attrs++)) {
        err = device_create_file(info->dev_t, attr);
        if (err) {
            device_destroy(info->device_class, 0);
            return err;
        }
    }
    return 0;
}

static void hd3ss3220_destroy_device(struct hd3ss3220_info *info)
{
    struct device_attribute **attrs = usb_typec_attributes;
    struct device_attribute *attr;

    while ((attr = *attrs++))
        device_remove_file(info->dev_t, attr);

    device_destroy(info->device_class, 0);
    class_destroy(info->device_class);
    info->device_class = NULL;
}

static int hd3ss3220_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
    struct hd3ss3220_info *info;
    struct device_node *np = client->dev.of_node;
    int ret, irq;

    pr_debug("%s: enter\n", __func__);
    info = kzalloc(sizeof(struct hd3ss3220_info), GFP_KERNEL);
    info->i2c = client;

    /* initialize pinctrl */
    if (!hd3ss3220_pinctrl_init(info)) {
        ret  = pinctrl_select_state(info->pinctrl,info->cc_int_cfg);
        if (ret ) {
            pr_err("%s: error initialize pinctrl\n", __func__);
            goto err_pinctrl;
        }
        else
            pr_debug("%s: initialize pinctrl success\n", __func__);
    }

    /* config gpio(cc irq) */
    ret = of_get_named_gpio(np, "hd3ss,irq_gpio", 0);
    if (ret < 0) {
        pr_err("%s: error invalid irq gpio number: %d\n", __func__, ret);
        goto err_pinctrl;
    }
    else {
        info->irq_gpio = ret;
        pr_debug("%s: valid irq_gpio number: %d\n", __func__, info->irq_gpio);

        irq = gpio_to_irq(info->irq_gpio);
        if (irq < 0) {
            pr_err("%s: error gpio_to_irq returned %d\n", __func__, irq);
            goto err_pinctrl;
        }
        else{
            pr_debug("%s: requesting IRQ %d\n", __func__, irq);
            client->irq = irq;
        }
    }

    /* config more gpio(s) here, if need */

    i2c_set_clientdata(client, info);
    mutex_init(&info->mutex);

    /* create device and sysfs nodes */
    ret = hd3ss3220_create_device(info);
    if (ret) {
        pr_err("%s: create device failed\n", __func__);
        goto err_device_create;
    }

    ret = hd3ss3220_initialization(info);
    if (ret < 0) {
        pr_err("%s: fails to do initialization %d\n", __func__, ret);
        goto err_device_create;
    }

    ret = request_threaded_irq(client->irq, NULL, hd3ss3220_irq_thread,
            IRQF_TRIGGER_LOW | IRQF_ONESHOT, "hd3ss3220_irq", info);
    if (ret){
        dev_err(&client->dev, "error failed to reqeust IRQ\n");
        goto err_request_irq;
    }

    ret = enable_irq_wake(client->irq);
    if (ret < 0){
        dev_err(&client->dev, "failed to enable wakeup src %d\n", ret);
        goto err_enable_irq;
    }

    dev_info(&client->dev, "hd3ss3220 usb type-c ship finish probe\n");
    return 0;

err_enable_irq:
    free_irq(client->irq, NULL);
err_request_irq:
    hd3ss3220_destroy_device(info);
err_device_create:
    mutex_destroy(&info->mutex);
    i2c_set_clientdata(client, NULL);
err_pinctrl:
    kfree(info);
    info = NULL;
    return ret;
}

static int hd3ss3220_remove(struct i2c_client *client)
{
    struct hd3ss3220_info *info = i2c_get_clientdata(client);

    if (client->irq) {
        disable_irq_wake(client->irq);
        free_irq(client->irq, info);
    }

    hd3ss3220_destroy_device(info);
    mutex_destroy(&info->mutex);
    i2c_set_clientdata(client, NULL);

    kfree(info);
    return 0;
}

static const struct of_device_id hd3ss3220_dt_match[] = {
    {
        .compatible = "ti,hd3ss3220",
    },
    {},
};
MODULE_DEVICE_TABLE(of, hd3ss3220_dt_match);

static const struct i2c_device_id hd3ss3220_id_table[] = {
    {
        .name = "hd3ss3220",
    },
};

static struct i2c_driver hd3ss3220_i2c_driver = {
    .driver = {
        .name = "hd3ss3220",
        .of_match_table = of_match_ptr(hd3ss3220_dt_match),
    },
    .probe    = hd3ss3220_probe,
    .remove   = hd3ss3220_remove,
    .suspend  = hd3ss3220_suspend,
    .resume	  = hd3ss3220_resume,
    .id_table = hd3ss3220_id_table,
};

static __init int hd3ss3220_i2c_init(void)
{
    return i2c_add_driver(&hd3ss3220_i2c_driver);
}

static __exit void hd3ss3220_i2c_exit(void)
{
    i2c_del_driver(&hd3ss3220_i2c_driver);
}

module_init(hd3ss3220_i2c_init);
module_exit(hd3ss3220_i2c_exit);

MODULE_DESCRIPTION("I2C bus driver for HD3SS3220 USB Type-C");
MODULE_LICENSE("GPL v2");
