/* include/linux/cm36771.h
 *
 * Copyright (C) 2013 Capella Microsystems Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#ifndef __LINUX_CM36771_H
#define __LINUX_CM36771_H

#define CM36771_I2C_NAME "cm36771"

/* Define Slave Address*/
#define	CM36771_slave_add	0xC0>>1

/*Define Command Code*/
#define	ALS_CONF	0x00
#define	ALS_THDH	0x01
#define	ALS_THDL	0x02
#define	PS_CONF1	0x03
#define	PS_CONF3	0x04
#define	PS_THD		0x06

#define	PS_DATA		0x08
#define	ALS_DATA	0x09
#define	INT_FLAG	0x0B

/*cm36771*/
/*for ALS CONF command*/
#define CM36771_CMD00_ALS_IT_80MS 		(0 << 6)
#define CM36771_CMD00_ALS_IT_160MS 		(1 << 6)
#define CM36771_CMD00_ALS_IT_320MS 		(2 << 6)
#define CM36771_CMD00_ALS_IT_640MS 		(3 << 6)
#define CM36771_CMD00_ALS_PERS_1 		(0 << 2)
#define CM36771_CMD00_ALS_PERS_2 		(1 << 2)
#define CM36771_CMD00_ALS_PERS_4 		(2 << 2)
#define CM36771_CMD00_ALS_PERS_8 		(3 << 2)
#define CM36771_CMD00_ALS_INT_EN	 	(1 << 1) /*enable/disable interrupt*/
#define CM36771_CMD00_ALS_SD			(1 << 0) /*enable/disable ALS func, 1:disable , 0: enable*/

/*for PS CONF1 command*/
#define CM36771_CMD03_PS_DR_1_5120		(0 << 6)
#define CM36771_CMD03_PS_DR_1_640		(1 << 6)
#define CM36771_CMD03_PS_DR_1_80		(2 << 6)
#define CM36771_CMD03_PS_DR_1_20		(3 << 6)
#define CM36771_CMD03_PS_IT_1T			(0 << 4)
#define CM36771_CMD03_PS_IT_1_3T		(1 << 4)
#define CM36771_CMD03_PS_IT_1_6T		(2 << 4)
#define CM36771_CMD03_PS_IT_2T			(3 << 4)
#define CM36771_CMD03_PS_PERS_1			(0 << 2)
#define CM36771_CMD03_PS_PERS_2			(1 << 2)
#define CM36771_CMD03_PS_PERS_3			(2 << 2)
#define CM36771_CMD03_PS_PERS_4			(3 << 2)
#define CM36771_CMD03_PS_SD				(1 << 0) /*enable/disable PS func, 1:disable , 0: enable*/

/*for PS CONF2 command*/
#define CM36771_CMD03_PS_BC_CONF1_1		(0 << 15)
#define CM36771_CMD03_PS_BC_CONF1_2_3	(1 << 15)
#define CM36771_CMD03_PS_SP_INT_EN		(1 << 10) /*enable/disable interrupt*/
#define CM36771_CMD03_PS_INT_EN			(1 << 8) /*enable/disable interrupt*/

#define CM36771_CMD03_DEFAULT			0x6000

/*for PS CONF3 command*/
#define CM36771_CMD04_PS_BC_CONF2_1		(2 << 5)
#define CM36771_CMD04_PS_BC_CONF2_2		(3 << 5)
#define CM36771_CMD04_PS_BC_CONF2_3		(5 << 5)
#define CM36771_CMD04_PS_BC_CONF3_1		(1 << 1)
#define CM36771_CMD04_PS_BC_CONF3_2_3	(0 << 1)

#define CM36771_CMD04_DEFAULT			0xA010

/*for INT FLAG*/
#define INT_FLAG_PS_SPF_LEAVE			(1 << 15)
#define INT_FLAG_PS_SPF_ENTER			(1 << 14)
#define INT_FLAG_ALS_IF_L				(1 << 13)
#define INT_FLAG_ALS_IF_H				(1 << 12)
#define INT_FLAG_PS_IF_CLOSE			(1 << 9)
#define INT_FLAG_PS_IF_AWAY				(1 << 8)  

extern unsigned int ps_kparam1;
extern unsigned int ps_kparam2;

struct cm36771_platform_data {
	int intr;
	int (*power)(int, uint8_t); /* power to the chip */
	uint8_t slave_addr;
	uint8_t ps_close_thd_set;
	uint8_t ps_away_thd_set;	
	uint16_t ls_cmd;
	uint16_t ps_conf1_val;
	uint16_t ps_conf3_val;	
};

#endif
