/*
 * Simple driver for Texas Instruments TPS611XX Backlight driver chip
 *        using EasyScale Interface. It supports TPS61158, TPS61161,
 *        TPS61163 and TPS61165.
 *
 * Copyright (C) 2014 Texas Instruments
 * Author: Daniel Jeong  <gshark.jeong@gmail.com>
 *	       Ldd Mlp <ldd-mlp@list.ti.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#include <linux/backlight.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/fb.h>
#include <linux/gpio.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/platform_data/tps611xx_bl.h>
#include <linux/platform_device.h>
#include <linux/slab.h>

#define CMD_FORWARD 0
#define CMD_BACKWARD 1

enum tps611xx_id {
	TPS61158_ID = 0,
	TPS61161_ID,
	TPS61163_ID,
	TPS61165_ID,
};

/*
 * easyscale time spec
 * @es_delay : es delay time(ns)
 * @es_det   : es detection time(ns)
 * @start    : start time of data stream(ns)
 * @eos      : end time of data stream(ns)
 * @reset    : ic shutdown time(ms)
 * @logic_1_low : low time high bit(ns)
 * @logic_0_low : low time low bit(ns)
 * @ackn        : duation of ack condistion(ns)
 * @ack_poll    : ack polling duration(ns)
 */
struct tps611xx_time {
	unsigned int es_delay;
	unsigned int es_det;
	unsigned int start;
	unsigned int eos;
	unsigned int reset;
	unsigned int logic_1_low;
	unsigned int logic_0_low;
	unsigned int ackn;
	unsigned int ack_poll;
};

/*
 * @seq : sequence of data transfer
 * @size: size of data
 * @brt_max : max brightness
 * @brt_bmask : bit mask of dimming bits
 * @rfa_bmask : bit mask of RFA(Request for Acknowledge condition)
 */
struct tps611xx_command {
	int seq;
	int size;
	int brt_max;
	int brt_bmask;
	int rfa_bmask;
};

/*
 * @id : product id
 * @name : product name
 * @addr : device address
 * @cmd  : es command info
 * @time : es time info
 */
struct tps611xx_esdata {
	enum tps611xx_id id;
	char *name;
	int addr;
	struct tps611xx_command cmd;
	struct tps611xx_time time;
};

struct tps611xx_bl_data {
	struct device *dev;
	struct backlight_device *bled;
	struct tps611xx_platform_data *pdata;

	/*
	 * @rfa_en : RFA enable (Request for Acknowledge condition)
	 * @en_gpio: enable pin gpio no.
	 * @esdata : easyscale data
	 */
	int rfa_en;
	unsigned int en_gpio;
	const struct tps611xx_esdata *esdata;
};

static struct tps611xx_esdata tps611xx_info[] = {
	[TPS61158_ID] = {
		.id = TPS61158_ID,
		.name = "tps61158",
		.addr = 0x5800,
		.cmd = {
			.seq = CMD_FORWARD,
			.size = 16,
			.brt_max = 31,
			.brt_bmask = 0x1f,
			.rfa_bmask = 0x80
		},
		.time = {
			.es_delay = 100000,
			.es_det = 450000,
			.start = 3500,
			.eos = 3500,
			.reset = 4,
			.logic_1_low = 5000,
			.logic_0_low = 15000,
			.ackn = 900000,
			.ack_poll = 2000
		},
	},

	[TPS61161_ID] = {
		.id = TPS61161_ID,
		.name = "tps61161",
		.addr = 0x7200,
		.cmd = {
			.seq = CMD_FORWARD,
			.size = 16,
			.brt_max = 31,
			.brt_bmask = 0x1f,
			.rfa_bmask = 0x80
		},
		.time = {
			.es_delay = 120000,
			.es_det = 280000,
			.start = 2000,
			.eos = 2000,
			.reset = 3,
			.logic_1_low = 3000,
			.logic_0_low = 7000,
			.ackn = 512000,
			.ack_poll = 2000
		},
	},

	[TPS61163_ID] = {
		.id = TPS61163_ID,
		.name = "tps61163",
		.addr = 0x8F0000,
		.cmd = {
			.seq = CMD_BACKWARD,
			.size = 24,
			.brt_max = 511,
			.brt_bmask = 0x1ff,
			.rfa_bmask = 0x400
		},
		.time = {
			.es_delay = 100000,
			.es_det = 260000,
			.start = 2000,
			.eos = 2000,
			.reset = 3,
			.logic_1_low = 3000,
			.logic_0_low = 7000,
			.ackn = 512000,
			.ack_poll = 2000
		},
	},

	[TPS61165_ID] = {
		.id = TPS61165_ID,
		.name = "tps61165",
		.addr = 0x7200,
		.cmd = {
			.seq = CMD_FORWARD,
			.size = 16,
			.brt_max = 31,
			.brt_bmask = 0x1f,
			.rfa_bmask = 0x80
		},
		.time = {
			.es_delay = 120000,
			.es_det = 280000,
			.start = 4000,
			.eos = 4000,
			.reset = 3,
			.logic_1_low = 3000,
			.logic_0_low = 7000,
			.ackn = 512000,
			.ack_poll = 2000
		},
	},
};

static int tps611xx_bl_update_status(struct backlight_device *bl)
{
	struct tps611xx_bl_data *pchip = bl_get_data(bl);
	const struct tps611xx_esdata *esdata = pchip->esdata;
	int data_in, t_low, t_logic, max_bmask;
	unsigned long flags;

	data_in = esdata->addr | (bl->props.brightness & esdata->cmd.brt_bmask);
	if (pchip->rfa_en)
		data_in |= esdata->cmd.rfa_bmask;

	max_bmask = 0x1 << esdata->cmd.size;
	t_logic = esdata->time.logic_1_low + esdata->time.logic_0_low;

	local_irq_save(flags);
	/* t_start : 2us high before data byte */
	gpio_direction_output(pchip->en_gpio, 1);
	ndelay(esdata->time.start);

	/* forward command transfer */
	if (esdata->cmd.seq == CMD_FORWARD) {
		int addr_bmask = max_bmask >> 8;

		for (max_bmask >>= 1; max_bmask > 0x0; max_bmask >>= 1) {
			if (data_in & max_bmask)
				t_low = esdata->time.logic_1_low;
			else
				t_low = esdata->time.logic_0_low;

			gpio_direction_output(pchip->en_gpio, 0);
			ndelay(t_low);
			gpio_direction_output(pchip->en_gpio, 1);
			ndelay(t_logic - t_low);

			if (max_bmask == addr_bmask) {
				gpio_direction_output(pchip->en_gpio, 0);
				/* t_eos : low after address byte */
				ndelay(esdata->time.eos);
				gpio_direction_output(pchip->en_gpio, 1);
				/* t_start : high before data byte */
				ndelay(esdata->time.start);
			}
		}
	} else {
		/* backward command tansfer */
		int bmask;

		for (bmask = 0x01; bmask < max_bmask; bmask <<= 1) {
			if (data_in & bmask)
				t_low = esdata->time.logic_1_low;
			else
				t_low = esdata->time.logic_0_low;

			gpio_direction_output(pchip->en_gpio, 0);
			ndelay(t_low);
			gpio_direction_output(pchip->en_gpio, 1);
			ndelay(t_logic - t_low);
		}
	}

	/*
	 * t_eos : low after address byte
	 * t_ackVal is also t_eos
	 */
	gpio_direction_output(pchip->en_gpio, 0);
	ndelay(esdata->time.eos);

	/* RFA management  */
	if (pchip->rfa_en) {
		int max_ack_time = esdata->time.ackn;
		/* set input */
		gpio_direction_input(pchip->en_gpio);
		/* read acknowledge from chip */
		while (max_ack_time > 0) {
			if (gpio_get_value(pchip->en_gpio) == 0)
				break;
			max_ack_time -= esdata->time.ack_poll;
		}
		if (max_ack_time <= 0)
			dev_err(pchip->dev,
				"easyscale : no ack from %s\n", esdata->name);
		else
			ndelay(max_ack_time);
	}
	gpio_direction_output(pchip->en_gpio, 1);
	local_irq_restore(flags);

	return bl->props.brightness;
}

static const struct backlight_ops tps611xx_bl_ops = {
	.update_status = tps611xx_bl_update_status,
};

static ssize_t tps611xx_enable_store(struct device *dev,
				     struct device_attribute *devAttr,
				     const char *buf, size_t size)
{
	struct tps611xx_bl_data *pchip = dev_get_drvdata(dev);
	const struct tps611xx_esdata *esdata = pchip->esdata;
	unsigned long flags;
	unsigned int input;
	int ret;

	ret = kstrtouint(buf, 10, &input);
	if (ret)
		return -EINVAL;

	local_irq_save(flags);
	if (input == 0) {
		/* chip disable */
		gpio_direction_output(pchip->en_gpio, 0);
		/* low more than reset ms to reset */
		mdelay(esdata->time.reset);
	} else {
		/* easyscale detection window */
		gpio_direction_output(pchip->en_gpio, 1);
		ndelay(esdata->time.es_delay);
		gpio_direction_output(pchip->en_gpio, 0);
		ndelay(esdata->time.es_det);
		gpio_direction_output(pchip->en_gpio, 1);
	}
	local_irq_restore(flags);

	return size;
}

static DEVICE_ATTR(enable, S_IWUSR, NULL, tps611xx_enable_store);

#ifdef CONFIG_OF
static const struct of_device_id tps611xx_backlight_of_match[] = {
	{.compatible = "ti,tps61158", .data = &tps611xx_info[TPS61158_ID]},
	{.compatible = "ti,tps61161", .data = &tps611xx_info[TPS61161_ID]},
	{.compatible = "ti,tps61163", .data = &tps611xx_info[TPS61163_ID]},
	{.compatible = "ti,tps61165", .data = &tps611xx_info[TPS61165_ID]},
	{}
};

MODULE_DEVICE_TABLE(of, tps611xx_backlight_of_match);

static int tps611xx_backlight_parse_dt(struct tps611xx_bl_data *pchip)
{
	struct device *dev = pchip->dev;
	struct device_node *node = dev->of_node;
	const struct of_device_id *of_id =
	    of_match_device(tps611xx_backlight_of_match, dev);

	if (!node)
		return -ENODEV;

	if (!of_id || !of_id->data) {
		dev_err(dev, "Failed to find tps611xx chip id\n");
		return -EFAULT;
	}
	pchip->esdata = of_id->data;
	pchip->rfa_en = of_property_read_bool(node, "rfa-enable");
	pchip->en_gpio = of_get_named_gpio(node, "es-gpio", 0);

	return 0;
}
#else
static int tps611xx_backlight_parse_dt(struct tps611xx_bl_data *pchip)
{
	return -ENODEV;
}
#endif

static int tps611xx_backlight_probe(struct platform_device *pdev)
{
	struct tps611xx_bl_data *pchip;
	struct backlight_properties props;
	const struct tps611xx_esdata *esdata;
	struct tps611xx_platform_data *pdata = dev_get_platdata(&pdev->dev);
	unsigned long flags;
	int ret;

	pchip = devm_kzalloc(&pdev->dev,
			     sizeof(struct tps611xx_bl_data), GFP_KERNEL);
	if (pchip == NULL)
		return -ENOMEM;
	pchip->dev = &pdev->dev;

	if (pdata == NULL) {
		ret = tps611xx_backlight_parse_dt(pchip);
		if (ret < 0)
			return ret;
	} else {
		pchip->rfa_en = pdata->rfa_en;
		pchip->en_gpio = pdata->en_gpio_num;
		pchip->esdata = (const struct tps611xx_esdata *)
		    platform_get_device_id(pdev)->driver_data;
	}
	esdata = pchip->esdata;

	memset(&props, 0, sizeof(struct backlight_properties));
	props.brightness = esdata->cmd.brt_max;
	props.max_brightness = esdata->cmd.brt_max;
	props.type = BACKLIGHT_RAW;
	pchip->bled =
	    devm_backlight_device_register(pchip->dev, TPS611XX_NAME,
					   pchip->dev, pchip,
					   &tps611xx_bl_ops, &props);
	if (IS_ERR(pchip->bled))
		return PTR_ERR(pchip->bled);

	/* for enable/disable */
	ret = device_create_file(&(pchip->bled->dev), &dev_attr_enable);
	if (ret < 0) {
		dev_err(pchip->dev, "failed : add sysfs entries\n");
		return ret;
	}
	platform_set_drvdata(pdev, pchip);

	/* EasyScale init */
	ret = gpio_request_one(pchip->en_gpio, GPIOF_OUT_INIT_HIGH, "tps611xx");
	if (ret) {
		device_remove_file(&(pchip->bled->dev), &dev_attr_enable);
		dev_err(pchip->dev, "failed : get gpio %d\n", pchip->en_gpio);
		return ret;
	}

	/*
	 * ES Detection Window
	 *   - ES detect delay
	 *   - ES detect time
	 */
	local_irq_save(flags);
	gpio_direction_output(pchip->en_gpio, 1);
	ndelay(esdata->time.es_delay);
	gpio_direction_output(pchip->en_gpio, 0);
	ndelay(esdata->time.es_det);
	gpio_direction_output(pchip->en_gpio, 1);
	local_irq_restore(flags);
	dev_info(pchip->dev,
		 "%s EasyScale is initialized\n", pchip->esdata->name);
	return 0;
}

static int tps611xx_backlight_remove(struct platform_device *pdev)
{
	struct tps611xx_bl_data *pchip = platform_get_drvdata(pdev);
	const struct tps611xx_esdata *esdata = pchip->esdata;

	device_remove_file(&(pchip->bled->dev), &dev_attr_enable);
	gpio_direction_output(pchip->en_gpio, 0);
	mdelay(esdata->time.reset);
	return 0;
}

static const struct platform_device_id tps611xx_id_table[] = {
	{TPS61158_NAME, (unsigned long)&tps611xx_info[TPS61158_ID]},
	{TPS61161_NAME, (unsigned long)&tps611xx_info[TPS61161_ID]},
	{TPS61163_NAME, (unsigned long)&tps611xx_info[TPS61163_ID]},
	{TPS61165_NAME, (unsigned long)&tps611xx_info[TPS61165_ID]},
	{}
};

static struct platform_driver tps611xx_backlight_driver = {
	.driver = {
		.name = TPS611XX_NAME,
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(tps611xx_backlight_of_match),
	},
	.probe = tps611xx_backlight_probe,
	.remove = tps611xx_backlight_remove,
	.id_table = tps611xx_id_table,
};

module_platform_driver(tps611xx_backlight_driver);

MODULE_DESCRIPTION("EasyScale based tps611xx Backlight Driver");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:tps611xx_bl");
