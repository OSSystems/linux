/*
 * Simple driver for Texas Instruments TPS611xx Backlight driver chip
 * Copyright (C) 2014 Texas Instruments
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#ifndef __TPS611XX_H
#define __TPS611XX_H

#define TPS611XX_NAME "tps611xx"
#define TPS61158_NAME "tps61158"
#define TPS61161_NAME "tps61161"
#define TPS61163_NAME "tps61163"
#define TPS61165_NAME "tps61165"

/*
 * struct tps611xx platform data
 * @rfa_en : request for acknowledge
 * @en_gpio_num : gpio number for en_pin
 */
struct tps611xx_platform_data {

	int rfa_en;
	unsigned int en_gpio_num;
};

#endif /* __TPS611XX_H */

