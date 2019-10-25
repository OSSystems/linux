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
 * LED driver for the NCP5623C I2C LED driver (7-bit slave address 0x72)
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

#define NCP5623C_SHUT_DOWN		0x00	/* LED driver off */
#define NCP5623C_ILED_CURRENT		0x20	/* ILED current */

#define NCP5623C_PWM_1			0x40
#define NCP5623C_PWM_2			0x60
#define NCP5623C_PWM_3			0x80

static const struct i2c_device_id ncp5623c_id[] = {
	{ "ncp5623c", 0 }, 
	{ }
};

static const struct of_device_id ncp5623c_i2c_dt_ids[] = { 
	{ .compatible = "no,ncp5623c_i2c" }, 
	{ } 
};

MODULE_DEVICE_TABLE(i2c, ncp5623c_id);

struct ncp5623c_led {
	struct i2c_client *client;
	struct work_struct work;
	enum led_brightness brightness;
	struct led_classdev led_cdev;
	int led_num; /* 0 .. 2 potentially */
	char name[32];
};

static void ncp5623c_led_work(struct work_struct *work)
{
	struct ncp5623c_led *ncp5623c = container_of(work,
		struct ncp5623c_led, work);
	int offset = 0x20 * ncp5623c->led_num;
	unsigned char data = NCP5623C_PWM_1 + offset
			     + (ncp5623c->brightness >> 3);

	i2c_master_send(ncp5623c->client, &data, 1);
}

static void ncp5623c_led_set(struct led_classdev *led_cdev,
	enum led_brightness value)
{
	struct ncp5623c_led *ncp5623c;

	ncp5623c = container_of(led_cdev, struct ncp5623c_led, led_cdev);

	ncp5623c->brightness = value;

	/*
	 * Must use workqueue for the actual I/O since I2C operations
	 * can sleep.
	 */
	schedule_work(&ncp5623c->work);
}

int ncp5623c_of_populate_pdata(struct device *dev, struct device_node *np)
{
	struct device_node *child;
	struct led_platform_data *pdata;
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

	for_each_child_of_node(np, child) {

		of_property_read_string(child, "chan-name", &cfg[i].name);
		cfg[i].default_trigger =
			of_get_property(child, "linux,default-trigger", NULL);

		i++;
	}

	dev->platform_data = pdata;

	return 0;
}

static int  ncp5623c_probe(struct i2c_client *client,
					const struct i2c_device_id *id)
{
	int ret;
	struct ncp5623c_led *ncp5623c;
	struct led_platform_data *pdata;
	unsigned char data;
	struct device_node *np = client->dev.of_node;
	int i, err;

	if (!dev_get_platdata(&client->dev)) {
		if (np) {
			ret = ncp5623c_of_populate_pdata(&client->dev, np);
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

	ncp5623c = kcalloc(4, sizeof(*ncp5623c), GFP_KERNEL);
	if (!ncp5623c)
		return -ENOMEM;

	i2c_set_clientdata(client, ncp5623c);

	for (i = 0; i < 3; i++) {
		ncp5623c[i].client = client;
		ncp5623c[i].led_num = i;

		/* Platform data can specify LED names and default triggers */
		if (pdata && i < pdata->num_leds) {
			if (pdata->leds[i].name)
				snprintf(ncp5623c[i].name,
					 sizeof(ncp5623c[i].name), "ncp5623c:%s",
					 pdata->leds[i].name);
			if (pdata->leds[i].default_trigger)
				ncp5623c[i].led_cdev.default_trigger =
					pdata->leds[i].default_trigger;
		} else {
			snprintf(ncp5623c[i].name, sizeof(ncp5623c[i].name),
				 "ncp5623c:%d", i);
		}

		ncp5623c[i].led_cdev.name = ncp5623c[i].name;
		ncp5623c[i].led_cdev.brightness_set = ncp5623c_led_set;

		INIT_WORK(&ncp5623c[i].work, ncp5623c_led_work);

		err = led_classdev_register(&client->dev, &ncp5623c[i].led_cdev);
		if (err < 0)
			goto exit;
	}

	/* Shut dowm chip */
	data = NCP5623C_SHUT_DOWN;
	err = i2c_master_send(client, &data, 1);
	if (err <= 0) {
		dev_err(&client->dev, "i2c communication failed");
		goto exit;
	}

	/* Set up ILED currrent */
	data = NCP5623C_ILED_CURRENT;
	i2c_master_send(client, &data, 1);
	data = NCP5623C_ILED_CURRENT + 0x1F; /* 25 mA */
	i2c_master_send(client, &data, 1);

	return 0;

exit:
	while (i--) {
		led_classdev_unregister(&ncp5623c[i].led_cdev);
		cancel_work_sync(&ncp5623c[i].work);
	}

	kfree(ncp5623c);

	return err;
}

static int ncp5623c_remove(struct i2c_client *client)
{
	struct ncp5623c_led *ncp5623c = i2c_get_clientdata(client);
	int i;

	for (i = 0; i < 3; i++) {
		led_classdev_unregister(&ncp5623c[i].led_cdev);
		cancel_work_sync(&ncp5623c[i].work);
	}

	kfree(ncp5623c);

	return 0;
}

static struct i2c_driver ncp5623c_driver = {
	.driver = {
		.name	= "leds-ncp5623c",
		.owner	= THIS_MODULE,
		.of_match_table = ncp5623c_i2c_dt_ids,
	},
	.probe	= ncp5623c_probe,
	.remove	= ncp5623c_remove,
	.id_table = ncp5623c_id,
};

module_i2c_driver(ncp5623c_driver);

MODULE_AUTHOR("Ronan Chauvin <ronan.chauvin@parrot.com>");
MODULE_DESCRIPTION("NCP5623C LED driver");
MODULE_LICENSE("GPL v2");
