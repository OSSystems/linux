/*
 * Copyright 2015 Parrot SA.
 *
 * Author: Ronan Chauvin <ronan.chauvind@parrot.com>
 *
 * Based on leds-pca955x.c
 *
 * This file is subject to the terms and conditions of version 2 of
 * the GNU General Public License.  See the file COPYING in the main
 * directory of this archive for more details.
 *
 * LED driver for the BD2606 I2C LED driver (7-bit slave address 0x72)
 *
 */

#include <linux/module.h>
#include <linux/delay.h>
#include <linux/string.h>
#include <linux/ctype.h>
#include <linux/leds.h>
#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/workqueue.h>
#include <linux/slab.h>
#include <linux/of.h>

#define BD2606_SHUT_DOWN		0x00	/* LED driver off */
#define BD2606_ILED_CURRENT		0x20	/* ILED current */

#define BD2606_PWM_1			0x0
#define BD2606_PWM_2			0x1
#define BD2606_PWM_3			0x2
#define BD2606_CONTROL			0x3

struct bd26060_platform_data {
	int		num_leds;
	struct led_info	*leds;
	const char *label;
};

static const struct i2c_device_id bd2606_id[] = {
	{ "bd2606", 0 }, 
	{ }
};

static const struct of_device_id bd2606_i2c_dt_ids[] = { 
	{ .compatible = "no,bd2606_i2c" }, 
	{ } 
};

MODULE_DEVICE_TABLE(i2c, bd2606_id);

struct bd2606_led {
	struct i2c_client *client;
	struct work_struct work;
	enum led_brightness brightness;
	struct led_classdev led_cdev;
	int led_num; /* 0 .. 2 potentially */
	char name[32];
};

static struct mutex lock_all;

static int bd2606_reg_read(struct i2c_client *client, u8 reg, u8 *value)
{
	int tmp;

	tmp = i2c_smbus_read_byte_data(client, reg);
	if (tmp < 0)
		return tmp;

	*value = tmp;

	return 0;
}

static int bd2606_reg_write(struct i2c_client *client, u8 reg, u8 value)
{
	return i2c_smbus_write_byte_data(client, reg, value);
}

static void bd2606_led_work(struct work_struct *work)
{
	struct bd2606_led *bd2606 = container_of(work,
		struct bd2606_led, work);
	int offset = bd2606->led_num;
	unsigned char mask = 1 << (2 * offset); 
	unsigned char control_byte=0;

	mutex_lock(&lock_all);
	bd2606_reg_write(bd2606->client, BD2606_PWM_1 + offset, bd2606->brightness >> 2);
	bd2606_reg_read(bd2606->client, BD2606_CONTROL, &control_byte);

	if (bd2606->brightness > 0)
		bd2606_reg_write(bd2606->client, BD2606_CONTROL, control_byte | mask);
	else
		bd2606_reg_write(bd2606->client, BD2606_CONTROL, control_byte & (~mask));

	mutex_unlock(&lock_all);
}

static void bd2606_led_set(struct led_classdev *led_cdev,
	enum led_brightness value)
{
	struct bd2606_led *bd2606;

	bd2606 = container_of(led_cdev, struct bd2606_led, led_cdev);

	bd2606->brightness = value;

	/*
	 * Must use workqueue for the actual I/O since I2C operations
	 * can sleep.
	 */
	schedule_work(&bd2606->work);
}

int bd2606_of_populate_pdata(struct device *dev, struct device_node *np)
{
	struct device_node *child;
	struct bd26060_platform_data *pdata;
	struct led_info *cfg;
	int num_channels;
	int i = 0;

	pdata = devm_kzalloc(dev, sizeof(*pdata), GFP_KERNEL);
	if (!pdata)
		return -ENOMEM;

	num_channels = of_get_child_count(np);
	if (num_channels == 0) {
		dev_err(dev, "no LED channels\n");
		return -EINVAL;
	}

	cfg = devm_kzalloc(dev, sizeof(*cfg) * num_channels, GFP_KERNEL);
	if (!cfg)
		return -ENOMEM;

	pdata->leds = &cfg[0];
	pdata->num_leds = num_channels;

	of_property_read_string(np, "label", &pdata->label);

	for_each_child_of_node(np, child) {

		of_property_read_string(child, "chan-name", &cfg[i].name);
		cfg[i].default_trigger =
			of_get_property(child, "linux,default-trigger", NULL);

		i++;
	}

	dev->platform_data = pdata;

	return 0;
}

static int  bd2606_probe(struct i2c_client *client,
					const struct i2c_device_id *id)
{
	int ret;
	struct bd2606_led *bd2606;
	struct bd26060_platform_data *pdata;
	struct device_node *np = client->dev.of_node;
	unsigned char control_byte=0;
	int i, err;

	if (!dev_get_platdata(&client->dev)) {
		if (np) {
			ret = bd2606_of_populate_pdata(&client->dev, np);
			if (ret < 0)
				return ret;
		} else {
			dev_err(&client->dev, "no platform data\n");
			return -EINVAL;
		}
	}
	pdata = dev_get_platdata(&client->dev);

	if (pdata) {
		if (pdata->num_leds <= 0 || pdata->num_leds > 3) {
			dev_err(&client->dev, "board info must claim at most 3 LEDs");
			return -EINVAL;
		}
	}

	if (bd2606_reg_read(client, BD2606_CONTROL, &control_byte)) {
		dev_err(&client->dev, "I2C_FUNC_I2C not supported\n");
		return -EIO;
	}

	bd2606 = kcalloc(4, sizeof(*bd2606), GFP_KERNEL);
	if (!bd2606)
		return -ENOMEM;

	i2c_set_clientdata(client, bd2606);

	mutex_init(&lock_all);

	for (i = 0; i < 3; i++) {
		bd2606[i].client = client;
		bd2606[i].led_num = i;

		/* Platform data can specify LED names and default triggers */
		if (pdata && i < pdata->num_leds) {
			if (pdata->leds[i].name)
//				snprintf(bd2606[i].name,
//					 sizeof(bd2606[i].name), "bd2606:%s",
//					 pdata->leds[i].name);
				snprintf(bd2606[i].name, sizeof(bd2606[i].name), "%s:%s",
					pdata->label, pdata->leds[i].name);
			if (pdata->leds[i].default_trigger)
				bd2606[i].led_cdev.default_trigger =
					pdata->leds[i].default_trigger;
		} else {
			snprintf(bd2606[i].name, sizeof(bd2606[i].name),
				 "bd2606:%d", i);
		}

		bd2606[i].led_cdev.name = bd2606[i].name;
		bd2606[i].led_cdev.brightness_set = bd2606_led_set;

		INIT_WORK(&bd2606[i].work, bd2606_led_work);

		err = led_classdev_register(&client->dev, &bd2606[i].led_cdev);
		if (err < 0)
			goto exit;
	}

	return 0;

exit:
	while (i--) {
		led_classdev_unregister(&bd2606[i].led_cdev);
		cancel_work_sync(&bd2606[i].work);
	}

	kfree(bd2606);

	return err;
}

static int bd2606_remove(struct i2c_client *client)
{
	struct bd2606_led *bd2606 = i2c_get_clientdata(client);
	int i;

	for (i = 0; i < 3; i++) {
		led_classdev_unregister(&bd2606[i].led_cdev);
		cancel_work_sync(&bd2606[i].work);
	}

	kfree(bd2606);

	return 0;
}

static struct i2c_driver bd2606_driver = {
	.driver = {
		.name	= "leds-bd2606",
		.owner	= THIS_MODULE,
		.of_match_table = bd2606_i2c_dt_ids,
	},
	.probe	= bd2606_probe,
	.remove	= bd2606_remove,
	.id_table = bd2606_id,
};

module_i2c_driver(bd2606_driver);

MODULE_AUTHOR("ShihYuan-Huang <timmy@ubiqconn.com>");
MODULE_DESCRIPTION("BD2606 LED driver");
MODULE_LICENSE("GPL v2");
