/*
 * ak8157.c  --  audio driver for AK8157
 *
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 *
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/initval.h>
#include <sound/tlv.h>

#include <linux/gpio.h>
#include <linux/of_gpio.h>

struct ak8157_priv {
	struct i2c_client *i2c;
	int rstn;
	int rstn_tmp;
};
#if 1
struct ak8157_priv ak8157_i2c;
unsigned int ak8157_i2c_read(struct i2c_client *i2c, unsigned int reg)
{

	int ret;

	ret = i2c_smbus_read_byte_data(i2c, (u8)(reg & 0xFF));

	if (ret < 0) {
		pr_err("%s reg=%x, ret=%d\n", __func__, reg, ret);
	}

	return ret;
}

static int ak8157_i2c_write(struct i2c_client *i2c, unsigned int reg,
	unsigned int value)
{
    int ret;
	pr_info("%s (addr,data)=(%x, %x)\n", __func__, reg, value);

    ret = i2c_smbus_write_byte_data(i2c, (u8)(reg & 0xFF), (u8)(value & 0xFF));
	if(ret < 0) {
		pr_err("%s ret=%d\n",__func__, ret);
		return EIO;
	}
	
	return 0;
}
#endif

void ak8157_rate_set(int rate)
{
    pr_err("%s rate=%d\n", __func__, rate);

    switch (rate) {
		case 3: // 192K 
			ak8157_i2c_write(ak8157_i2c.i2c, 0x01, 0x22);
			break;
		case 2: // 96K
			ak8157_i2c_write(ak8157_i2c.i2c, 0x01, 0x12);
			break;
		case 1: // 48K
			ak8157_i2c_write(ak8157_i2c.i2c, 0x01, 0x02);
			break;
		case 0: // 44.1K
			ak8157_i2c_write(ak8157_i2c.i2c, 0x01, 0x01);
			break;
		default:
			ak8157_i2c_write(ak8157_i2c.i2c, 0x01, 0x02);
			break;
	}
	msleep(5);
}


static int ak8157_config_init(struct i2c_client *client)
{

	int ret = 0,ret1=0,ret2=0;
	ret = ak8157_i2c_write(client,0x00,0xe7);
	//ret = ak8157_i2c_write(client,0x00,0xe7);
/*	if(ret < 0 )
		{
		pr_err("[ak8157] %s 0x00\n",__FUNCTION__);
		return EIO;
		}*/
	ret1 = ak8157_i2c_read(client,0x00);
	
	ret = ak8157_i2c_write(client,0x01,0x02);
/*	if(ret < 0 ) {
		pr_err("[ak8157] %s 0x01\n",__FUNCTION__);
		return EIO;
	}
	*/
	ret2= ak8157_i2c_read(client,0x01);
	pr_err("[LHS]%s ret1 = %0x,ret2 = %0x\n",__FUNCTION__,ret1,ret2);
	return 0;

}
static int ak8157_i2c_probe(struct i2c_client *i2c,
                            const struct i2c_device_id *id)
{
	struct ak8157_priv *ak8157;
	int ret=0;

	pr_err("%s enter\n",__func__);

	ak8157 = kzalloc(sizeof(struct ak8157_priv), GFP_KERNEL);
	if (ak8157 == NULL) 
		return -ENOMEM;

    ak8157->i2c = i2c;
	i2c_set_clientdata(i2c, ak8157);
	
#if 1
    ak8157->rstn = of_get_named_gpio(i2c->dev.of_node, 
					      "ak8157,rstn-gpio", 0);
	if (!gpio_is_valid(ak8157->rstn)){  
		pr_err("%s error ak8157 rstn ret=%d\n", __func__, ak8157->rstn);
		return -EINVAL;
	}

	ret = gpio_request(ak8157->rstn, "ak8157_rstn_pin");
	if (ret < 0) {
		pr_err("%s(): ak8157_rstn_pin request failed %d\n",
				__func__, ret);
		return ret;
	}

	gpio_direction_output(ak8157->rstn, 0);
	usleep_range(1000, 1005);
	gpio_direction_output(ak8157->rstn, 1);
    	pr_err("[LHS]%s(): reset success! \n",__func__);
#endif

// ZTE_chenjun:TEMP
#if 1
    ak8157->rstn_tmp = of_get_named_gpio(i2c->dev.of_node, 
					      "ak8157,rstn-gpio-tmp", 0);
	if (!gpio_is_valid(ak8157->rstn_tmp)){  
		pr_err("%s error ak8157 rstn_tmp ret=%d\n", __func__, ak8157->rstn_tmp);
		return -EINVAL;
	}

	ret = gpio_request(ak8157->rstn_tmp, "ak8157_rstn_pin_tmp");
	if (ret < 0) {
		pr_err("%s(): ak8157_rstn_pin_tmp request failed %d\n",
				__func__, ret);
		return ret;
	}

	gpio_direction_output(ak8157->rstn_tmp, 0);
	usleep_range(1000, 1005);
	gpio_direction_output(ak8157->rstn_tmp, 1);
    	pr_err("[LHS]%s(): reset tmp success! \n",__func__);
#endif

    	ak8157_config_init(i2c);
    	ak8157_i2c.i2c= i2c;
	ak8157_i2c.rstn=ak8157->rstn;
	ak8157_i2c.rstn_tmp=ak8157->rstn_tmp;
	return ret;
}


void ak8157_power_on(void)
{
	int ret;
	pr_err("[LHS] %s \n",__func__);
	//usleep_range(1000, 1005);
	//gpio_direction_output(ak8157_i2c.rstn,1);
	//ak8157_config_init(ak8157_i2c.i2c);
	ret=ak8157_i2c_write(ak8157_i2c.i2c,0x00,0xc0);
}

void ak8157_power_down(void)
{
	int ret;
	pr_err("[LHS] %s \n",__func__);
	ret = ak8157_i2c_write(ak8157_i2c.i2c,0x00,0xe7);
	//gpio_direction_output(ak8157_i2c.rstn,0);
}


static int ak8157_i2c_remove(struct i2c_client *i2c)
{
	kfree(i2c_get_clientdata(i2c));

	return 0;
}

static const struct i2c_device_id ak8157_i2c_id[] = {
	{ "ak8157", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, ak8157_i2c_id);

static struct i2c_driver ak8157_i2c_driver = {
	.driver = {
		.name = "ak8157",
		.owner = THIS_MODULE,
	},
	.probe = ak8157_i2c_probe,
	.remove = ak8157_i2c_remove,
	.id_table = ak8157_i2c_id,
};

static int __init ak8157_init(void)
{

	pr_info("%s \n",__func__);

	return i2c_add_driver(&ak8157_i2c_driver);
}

module_init(ak8157_init);

static void __exit ak8157_exit(void)
{
	i2c_del_driver(&ak8157_i2c_driver);
}
module_exit(ak8157_exit);

MODULE_DESCRIPTION("ASoC AK8157 codec driver");
MODULE_LICENSE("GPL");

