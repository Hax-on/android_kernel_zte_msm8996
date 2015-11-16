/*
 * isl98608.c  --  audio driver for ISL98608
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

struct isl98608_priv {
	struct i2c_client *i2c;
	//int vdd_ctrl_gpio;
};
struct isl98608_priv g_l98608;


#if 1
unsigned int isl98608_i2c_read(struct i2c_client *i2c, unsigned int reg)
{

	int ret;

	ret = i2c_smbus_read_byte_data(i2c, (u8)(reg & 0xFF));

	if (ret < 0) {
		pr_err("%s reg=%x, ret=%d\n", __func__, reg, ret);
	}

	return ret;
}

static int isl98608_i2c_write(struct i2c_client *i2c, unsigned int reg,
	unsigned int value)
{
    int ret;
	//u8 msg[2];
	pr_err("%s 111 (addr,data)=(%x, %x)\n", __func__, reg, value);
	//msg[0] = reg;
	//msg[1] = value;

   /*  ret= i2c_master_send(i2c,msg,2);
	if(ret < 0) {
		pr_err("%s ret 0000=%d\n",__func__, ret);
		return EIO;
	}
*/
    ret = i2c_smbus_write_byte_data(i2c, (u8)(reg & 0xFF), (u8)(value & 0xFF));
	if(ret < 0) {
		pr_err("%s ret 1 111=%d\n",__func__, ret);
		return EIO;
	}
	
	return 0;
}
#endif

void isl98608_power_down(void)
{
	pr_err("[LHS]%s enter\n",__func__);
	//return ;
	isl98608_i2c_write(g_l98608.i2c,0x05,0x80);
}

void isl98608_power_up(void)
{
	pr_err("[LHS]%s enter\n",__func__);
	isl98608_i2c_write(g_l98608.i2c,0x05,0x87);
	/*return ;
	isl98608_i2c_write(g_l98608.i2c,0x00,0x00);
	isl98608_i2c_write(g_l98608.i2c,0x01,0x00);
	isl98608_i2c_write(g_l98608.i2c,0x02,0x00);
	isl98608_i2c_write(g_l98608.i2c,0x03,0x00);
	isl98608_i2c_write(g_l98608.i2c,0x04,0x00);
	isl98608_i2c_write(g_l98608.i2c,0x05,0x87);	
	isl98608_i2c_write(g_l98608.i2c,0x06,0x28);
	isl98608_i2c_write(g_l98608.i2c,0x07,0x00);
	isl98608_i2c_write(g_l98608.i2c,0x08,0x00);
	isl98608_i2c_write(g_l98608.i2c,0x09,0x28);
	isl98608_i2c_write(g_l98608.i2c,0x0a,0xf0);
	isl98608_i2c_write(g_l98608.i2c,0x0b,0x0a);
	isl98608_i2c_write(g_l98608.i2c,0x0c,0x05);
	isl98608_i2c_write(g_l98608.i2c,0x0d,0x34);*/
}


static int ak98608_config_init(struct i2c_client *client)
{
    pr_err("%s enter\n",__func__);
    
	isl98608_i2c_write(client,0x00,0x00);
	isl98608_i2c_write(client,0x01,0x00);
	isl98608_i2c_write(client,0x02,0x00);
	isl98608_i2c_write(client,0x03,0x00);
	isl98608_i2c_write(client,0x04,0x00);
	isl98608_i2c_write(client,0x05,0x80);	
	isl98608_i2c_write(client,0x06,0x28);
	isl98608_i2c_write(client,0x07,0x00);
	isl98608_i2c_write(client,0x08,0x00);
	isl98608_i2c_write(client,0x09,0x28);
	isl98608_i2c_write(client,0x0a,0xf0);
	isl98608_i2c_write(client,0x0b,0x0a);
	isl98608_i2c_write(client,0x0c,0x05);
	isl98608_i2c_write(client,0x0d,0x34);
	return 0;

}


static int isl98608_i2c_probe(struct i2c_client *i2c,
                            const struct i2c_device_id *id)
{
	struct isl98608_priv *isl98608;
	int ret=0;
	int val_address = 0,val_value = 0;
	pr_err("%s enter\n",__func__);

	isl98608 = kzalloc(sizeof(struct isl98608_priv), GFP_KERNEL);
	if (isl98608 == NULL) {
		pr_err("%s Error: alloc error ! \n",__func__);
		return -ENOMEM;
	}


	if (!i2c_check_functionality(i2c->adapter,
				     I2C_FUNC_SMBUS_BYTE_DATA)) {
		dev_warn(&i2c->dev,
			 "I2C adapter doesn't support I2C_FUNC_SMBUS_BYTE\n");
		pr_err("[LHS] %s I2C adapter doesn't support I2C_FUNC_SMBUS_BYTE",__func__);
		return -EIO;
	}


    	isl98608->i2c = i2c;
	i2c_set_clientdata(i2c, isl98608);
/*
	isl98608->vdd_ctrl_gpio = of_get_named_gpio(i2c->dev.of_node, 
					      "isl98608,vdd-ctrl-gpio", 0);
	if (!gpio_is_valid(isl98608->vdd_ctrl_gpio)){
		pr_err("%s error vdd_ctrl_gpio ret=%d\n", __func__, isl98608->vdd_ctrl_gpio);
		return -EINVAL;
	}
	pr_info("%s enter2\n",__func__);
	ret = gpio_request(isl98608->vdd_ctrl_gpio, "isl98608_vdd_ctrl_gpi");
	if (ret < 0) {
		pr_err("%s(): isl98608_vdd_ctrl_gpi request failed %d\n",
				__func__, ret);
		return ret;
	}
	ret = gpio_direction_output(isl98608->vdd_ctrl_gpio, 1);
	if (ret < 0) {
		pr_err("%s(): isl98608_vdd_ctrl_gpi direction failed %d\n",
				__func__, ret);
		return ret;
	}
*/

/*
    while(0){
	    isl98608_i2c_write(i2c, 0x01, 0xaa);
	    usleep(50);
	    isl98608_i2c_read(i2c, 0x01);
	    usleep(100);
       }
	pr_info("%s successufy\n",__func__);
*/
	//98608 register conifig
	ak98608_config_init(i2c);
	//read reg_data

	g_l98608.i2c=isl98608->i2c;
	for(val_address = 0x00;val_address<0x0e;val_address++)

	{
		 val_value = isl98608_i2c_read(i2c, val_address);
		if(val_value <0 )
		{
			pr_info("%s:i2c read failed\n",__func__);
			return EIO;
		}
		pr_err("[LHS]%s:reg_addr:%x,reg_value:%x\n",__func__,val_address,val_value);
	}
	//msleep(1000);
//	isl98608_i2c_write(i2c,0x05,0x80);
	
	return ret;
}

static int isl98608_i2c_remove(struct i2c_client *i2c)
{
	kfree(i2c_get_clientdata(i2c));

	return 0;
}

static const struct i2c_device_id isl98608_i2c_id[] = {
	{ "isl98608", 0 },
	{ }
};

MODULE_DEVICE_TABLE(i2c, isl98608_i2c_id);

static struct i2c_driver isl98608_i2c_driver = {
	.driver = {
		.name = "isl98608",
		.owner = THIS_MODULE,
	},
	.probe = isl98608_i2c_probe,
	.remove = isl98608_i2c_remove,
	.id_table = isl98608_i2c_id,
};


static int __init isl98608_init(void)
{

	pr_info("%s \n",__func__);

	return i2c_add_driver(&isl98608_i2c_driver);
}

module_init(isl98608_init);

static void __exit isl98608_exit(void)
{
	i2c_del_driver(&isl98608_i2c_driver);
}
module_exit(isl98608_exit);

MODULE_DESCRIPTION("ASoC ISL98608 codec driver");
MODULE_LICENSE("GPL");
