// SPDX-License-Identifier: GPL-2.0-only
/*
 * Based on the gpio-poweroff driver.
 *
 * Jamie Lentin <jm@lentin.co.uk>
 * Andrew Lunn <andrew@lunn.ch>
 *
 * Copyright (C) 2012 Jamie Lentin
 */
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/gpio/consumer.h>
#include <linux/of_platform.h>
#include <linux/module.h>

#define DEFAULT_TIMEOUT_MS 3000
/*
 * Hold configuration here, cannot be more than one instance of the driver
 * since pm_power_off itself is global.
 */
static u32 timeout = DEFAULT_TIMEOUT_MS;
static u32 active_delay = 100;
static u32 inactive_delay = 100;

struct ltc2955_poweroff {
	struct device *dev;
	struct gpio_desc *kill;
	struct gpio_desc *enable;
};

static struct ltc2955_poweroff *ltc2955_data;

static void ltc2955_poweroff_do_poweroff(void)
{
	/* drive it active, also inactive->active edge */
	gpiod_direction_output(ltc2955_data->kill, 1);
	gpiod_direction_output(ltc2955_data->enable, 1);
	mdelay(active_delay);

	/* drive inactive, also active->inactive edge */
	gpiod_set_value_cansleep(ltc2955_data->kill, 0);
	gpiod_set_value_cansleep(ltc2955_data->enable, 0);
	mdelay(inactive_delay);

	/* drive it active, also inactive->active edge */
	gpiod_set_value_cansleep(ltc2955_data ->kill, 1);
	gpiod_set_value_cansleep(ltc2955_data ->enable, 1);
	
	gpiod_set_value_cansleep(ltc2955_data ->kill, 0);
	/* give it some time */
	mdelay(timeout);

	WARN_ON(1);
}

static int ltc2955_poweroff_probe(struct platform_device *pdev)
{
	struct ltc2955_poweroff *data;
	
	/* If a pm_power_off function has already been added, leave it alone */
	if (pm_power_off != NULL) {
		dev_err(&pdev->dev,
			"%s: pm_power_off function already registered",
		       __func__);
		return -EBUSY;
	}
	
	data = devm_kzalloc(&pdev->dev, sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	data->dev = &pdev->dev;
	platform_set_drvdata(pdev, data);

	data->kill = devm_gpiod_get(&pdev->dev, "kill", GPIOD_OUT_LOW);
	if (IS_ERR(data->kill))
		return PTR_ERR(data->kill);

	data->enable = devm_gpiod_get(&pdev->dev, "enable", GPIOD_OUT_LOW);
	if (IS_ERR(data->enable))
		return PTR_ERR(data->enable);

	ltc2955_data = data;
	pm_power_off = &ltc2955_poweroff_do_poweroff;
	
	return 0;
}

static int ltc2955_poweroff_remove(struct platform_device *pdev)
{
	if (pm_power_off == &ltc2955_poweroff_do_poweroff)
		pm_power_off = NULL;

	return 0;
}

static const struct of_device_id of_ltc2955_poweroff_match[] = {
	{ .compatible = "lltc,ltc2955-poweroff", },
	{},
};

static struct platform_driver ltc2955_poweroff_driver = {
	.probe = ltc2955_poweroff_probe,
	.remove = ltc2955_poweroff_remove,
	.driver = {
		.name = "ltc2955-poweroff",
		.of_match_table = of_ltc2955_poweroff_match,
	},
};

module_platform_driver(ltc2955_poweroff_driver);

MODULE_DESCRIPTION("LTC2955 poweroff driver");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:ltc2955-poweroff");
