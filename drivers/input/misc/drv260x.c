// SPDX-License-Identifier: GPL-2.0-only
/*
 * DRV260X haptics driver family
 *
 * Author: Dan Murphy <dmurphy@ti.com>
 *
 * Copyright:   (C) 2014 Texas Instruments, Inc.
 */

#include <linux/i2c.h>
#include <linux/regmap.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/regulator/consumer.h>

#include <dt-bindings/input/ti-drv260x.h>

#define DRV260X_STATUS		0x0
#define DRV260X_MODE		0x1
#define DRV260X_RT_PB_IN	0x2
#define DRV260X_LIB_SEL		0x3
#define DRV260X_WV_SEQ_1	0x4
#define DRV260X_WV_SEQ_2	0x5
#define DRV260X_WV_SEQ_3	0x6
#define DRV260X_WV_SEQ_4	0x7
#define DRV260X_WV_SEQ_5	0x8
#define DRV260X_WV_SEQ_6	0x9
#define DRV260X_WV_SEQ_7	0xa
#define DRV260X_WV_SEQ_8	0xb
#define DRV260X_GO				0xc
#define DRV260X_OVERDRIVE_OFF	0xd
#define DRV260X_SUSTAIN_P_OFF	0xe
#define DRV260X_SUSTAIN_N_OFF	0xf
#define DRV260X_BRAKE_OFF		0x10
#define DRV260X_A_TO_V_CTRL		0x11
#define DRV260X_A_TO_V_MIN_INPUT	0x12
#define DRV260X_A_TO_V_MAX_INPUT	0x13
#define DRV260X_A_TO_V_MIN_OUT	0x14
#define DRV260X_A_TO_V_MAX_OUT	0x15
#define DRV260X_RATED_VOLT		0x16
#define DRV260X_OD_CLAMP_VOLT	0x17
#define DRV260X_CAL_COMP		0x18
#define DRV260X_CAL_BACK_EMF	0x19
#define DRV260X_FEEDBACK_CTRL	0x1a
#define DRV260X_CTRL1			0x1b
#define DRV260X_CTRL2			0x1c
#define DRV260X_CTRL3			0x1d
#define DRV260X_CTRL4			0x1e
#define DRV260X_CTRL5			0x1f
#define DRV260X_LRA_LOOP_PERIOD	0x20
#define DRV260X_VBAT_MON		0x21
#define DRV260X_LRA_RES_PERIOD	0x22
#define DRV260X_MAX_REG			0x23

#define DRV260X_GO_BIT				0x01

/* Library Selection */
#define DRV260X_LIB_SEL_MASK		0x07
#define DRV260X_LIB_SEL_RAM			0x0
#define DRV260X_LIB_SEL_OD			0x1
#define DRV260X_LIB_SEL_40_60		0x2
#define DRV260X_LIB_SEL_60_80		0x3
#define DRV260X_LIB_SEL_100_140		0x4
#define DRV260X_LIB_SEL_140_PLUS	0x5

#define DRV260X_LIB_SEL_HIZ_MASK	0x10
#define DRV260X_LIB_SEL_HIZ_EN		0x01
#define DRV260X_LIB_SEL_HIZ_DIS		0

/* Mode register */
#define DRV260X_STANDBY				(1 << 6)
#define DRV260X_STANDBY_MASK		0x40
#define DRV260X_INTERNAL_TRIGGER	0x00
#define DRV260X_EXT_TRIGGER_EDGE	0x01
#define DRV260X_EXT_TRIGGER_LEVEL	0x02
#define DRV260X_PWM_ANALOG_IN		0x03
#define DRV260X_AUDIOHAPTIC			0x04
#define DRV260X_RT_PLAYBACK			0x05
#define DRV260X_DIAGNOSTICS			0x06
#define DRV260X_AUTO_CAL			0x07

/* Audio to Haptics Control */
#define DRV260X_AUDIO_HAPTICS_PEAK_10MS		(0 << 2)
#define DRV260X_AUDIO_HAPTICS_PEAK_20MS		(1 << 2)
#define DRV260X_AUDIO_HAPTICS_PEAK_30MS		(2 << 2)
#define DRV260X_AUDIO_HAPTICS_PEAK_40MS		(3 << 2)

#define DRV260X_AUDIO_HAPTICS_FILTER_100HZ	0x00
#define DRV260X_AUDIO_HAPTICS_FILTER_125HZ	0x01
#define DRV260X_AUDIO_HAPTICS_FILTER_150HZ	0x02
#define DRV260X_AUDIO_HAPTICS_FILTER_200HZ	0x03

/* Min/Max Input/Output Voltages */
#define DRV260X_AUDIO_HAPTICS_MIN_IN_VOLT	0x19
#define DRV260X_AUDIO_HAPTICS_MAX_IN_VOLT	0x64
#define DRV260X_AUDIO_HAPTICS_MIN_OUT_VOLT	0x19
#define DRV260X_AUDIO_HAPTICS_MAX_OUT_VOLT	0xFF

/* Feedback register */
#define DRV260X_FB_REG_ERM_MODE			0x7f
#define DRV260X_FB_REG_LRA_MODE			(1 << 7)

#define DRV260X_BRAKE_FACTOR_MASK	0x1f
#define DRV260X_BRAKE_FACTOR_2X		(1 << 0)
#define DRV260X_BRAKE_FACTOR_3X		(2 << 4)
#define DRV260X_BRAKE_FACTOR_4X		(3 << 4)
#define DRV260X_BRAKE_FACTOR_6X		(4 << 4)
#define DRV260X_BRAKE_FACTOR_8X		(5 << 4)
#define DRV260X_BRAKE_FACTOR_16		(6 << 4)
#define DRV260X_BRAKE_FACTOR_DIS	(7 << 4)

#define DRV260X_LOOP_GAIN_LOW		0xf3
#define DRV260X_LOOP_GAIN_MED		(1 << 2)
#define DRV260X_LOOP_GAIN_HIGH		(2 << 2)
#define DRV260X_LOOP_GAIN_VERY_HIGH	(3 << 2)

#define DRV260X_BEMF_GAIN_0			0xfc
#define DRV260X_BEMF_GAIN_1		(1 << 0)
#define DRV260X_BEMF_GAIN_2		(2 << 0)
#define DRV260X_BEMF_GAIN_3		(3 << 0)

/* Control 1 register */
#define DRV260X_AC_CPLE_EN			(1 << 5)
#define DRV260X_STARTUP_BOOST		(1 << 7)

/* Control 2 register */

#define DRV260X_IDISS_TIME_45		0
#define DRV260X_IDISS_TIME_75		(1 << 0)
#define DRV260X_IDISS_TIME_150		(1 << 1)
#define DRV260X_IDISS_TIME_225		0x03

#define DRV260X_BLANK_TIME_45	(0 << 2)
#define DRV260X_BLANK_TIME_75	(1 << 2)
#define DRV260X_BLANK_TIME_150	(2 << 2)
#define DRV260X_BLANK_TIME_225	(3 << 2)

#define DRV260X_SAMP_TIME_150	(0 << 4)
#define DRV260X_SAMP_TIME_200	(1 << 4)
#define DRV260X_SAMP_TIME_250	(2 << 4)
#define DRV260X_SAMP_TIME_300	(3 << 4)

#define DRV260X_BRAKE_STABILIZER	(1 << 6)
#define DRV260X_UNIDIR_IN			(0 << 7)
#define DRV260X_BIDIR_IN			(1 << 7)

/* Control 3 Register */
#define DRV260X_LRA_OPEN_LOOP		(1 << 0)
#define DRV260X_ANANLOG_IN			(1 << 1)
#define DRV260X_LRA_DRV_MODE		(1 << 2)
#define DRV260X_RTP_UNSIGNED_DATA	(1 << 3)
#define DRV260X_SUPPLY_COMP_DIS		(1 << 4)
#define DRV260X_ERM_OPEN_LOOP		(1 << 5)
#define DRV260X_NG_THRESH_0			(0 << 6)
#define DRV260X_NG_THRESH_2			(1 << 6)
#define DRV260X_NG_THRESH_4			(2 << 6)
#define DRV260X_NG_THRESH_8			(3 << 6)

/* Control 4 Register */
#define DRV260X_AUTOCAL_TIME_150MS		(0 << 4)
#define DRV260X_AUTOCAL_TIME_250MS		(1 << 4)
#define DRV260X_AUTOCAL_TIME_500MS		(2 << 4)
#define DRV260X_AUTOCAL_TIME_1000MS		(3 << 4) 

/**
 * struct drv260x_data -
 * @client: Pointer to the I2C client
 * @regmap: Register map of the device
 * @work: Work item used to off load the enable/disable of the vibration
 * @enable_gpio: Pointer to the gpio used for enable/disabling
 * @regulator: Pointer to the regulator for the IC
 * @mode: The operating mode of the IC (LRA_NO_CAL, ERM or LRA)
 * @library: The vibration library to be used
 * @effect_id: The selected id from an effect
 * @rated_voltage: The rated_voltage of the actuator
 * @overdrive_voltage: The over drive voltage of the actuator
**/
struct drv260x_data {
	struct i2c_client *client;
	struct regmap *regmap;
	struct work_struct work;
	struct gpio_desc *enable_gpio;
	struct regulator *regulator;
	u32 mode;
	u32 library;
	u32 effect_id;
	int rated_voltage;
	int overdrive_voltage;
};

static const struct reg_default drv260x_reg_defs[] = {
	{ DRV260X_STATUS, 0xe0 },
	{ DRV260X_MODE, 0x40 },
	{ DRV260X_RT_PB_IN, 0x00 },
	{ DRV260X_LIB_SEL, 0x00 },
	{ DRV260X_WV_SEQ_1, 0x01 },
	{ DRV260X_WV_SEQ_2, 0x00 },
	{ DRV260X_WV_SEQ_3, 0x00 },
	{ DRV260X_WV_SEQ_4, 0x00 },
	{ DRV260X_WV_SEQ_5, 0x00 },
	{ DRV260X_WV_SEQ_6, 0x00 },
	{ DRV260X_WV_SEQ_7, 0x00 },
	{ DRV260X_WV_SEQ_8, 0x00 },
	{ DRV260X_GO, 0x00 },
	{ DRV260X_OVERDRIVE_OFF, 0x00 },
	{ DRV260X_SUSTAIN_P_OFF, 0x00 },
	{ DRV260X_SUSTAIN_N_OFF, 0x00 },
	{ DRV260X_BRAKE_OFF, 0x00 },
	{ DRV260X_A_TO_V_CTRL, 0x05 },
	{ DRV260X_A_TO_V_MIN_INPUT, 0x19 },
	{ DRV260X_A_TO_V_MAX_INPUT, 0xff },
	{ DRV260X_A_TO_V_MIN_OUT, 0x19 },
	{ DRV260X_A_TO_V_MAX_OUT, 0xff },
	{ DRV260X_RATED_VOLT, 0x3e },
	{ DRV260X_OD_CLAMP_VOLT, 0x8c },
	{ DRV260X_CAL_COMP, 0x0c },
	{ DRV260X_CAL_BACK_EMF, 0x6c },
	{ DRV260X_FEEDBACK_CTRL, 0x36 },
	{ DRV260X_CTRL1, 0x93 },
	{ DRV260X_CTRL2, 0xfa },
	{ DRV260X_CTRL3, 0xa0 },
	{ DRV260X_CTRL4, 0x20 },
	{ DRV260X_CTRL5, 0x80 },
	{ DRV260X_LRA_LOOP_PERIOD, 0x33 },
	{ DRV260X_VBAT_MON, 0x00 },
	{ DRV260X_LRA_RES_PERIOD, 0x00 },
};

#define DRV260X_GO_BIT_ASCII 		0x31

#define DRV260X_DEF_RATED_VOLT		0x90
#define DRV260X_DEF_OD_CLAMP_VOLT	0x90

static const char effects_collection[][50] = {
	"",
	"Strong Click - 100%",
	"Strong Click - 60%",
	"Strong Click - 30%",
	"Sharp Click - 100%",
	"Sharp Click - 60%",
	"Sharp Click - 30%",
	"Soft Bump - 100%",
	"Soft Bump - 60%",
	"Soft Bump - 30%",
	"Double Click - 100%",
	"Double Click - 60%",
	"Triple Click - 100%",
	"Soft Fuzz - 60%",
	"Strong Buzz - 100%",
	"750 ms Alert 100%",
	"1000 ms Alert 100%",
	"Strong Click 1 - 100%",
	"Strong Click 2 - 80%",
	"Strong Click 3 - 60%",
	"Strong Click 4 - 30%",
	"Medium Click 1 - 100%",
	"Medium Click 2 - 80%",
	"Medium Click 3 - 60%",
	"Sharp Tick 1 - 100%",
	"Sharp Tick 2 - 80%",
	"Sharp Tick 3 - 60%",
	"Short Double Click Strong 1 - 100%",
	"Short Double Click Strong 2 - 80%",
	"Short Double Click Strong 3 - 60%",
	"Short Double Click Strong 4 - 30%",
	"Short Double Click Medium 1 - 100%",
	"Short Double Click Medium 2 - 80%",
	"Short Double Click Medium 3 - 60%",
	"Short Double Sharp Tick 1 - 100%",
	"Short Double Sharp Tick 2 - 80%",
	"Short Double Sharp Tick 3 - 60%",
	"Long Double Sharp Click Strong 1 - 100%",
	"Long Double Sharp Click Strong 2 - 80%",
	"Long Double Sharp Click Strong 3 - 60%",
	"Long Double Sharp Click Strong 4 - 30%",
	"Long Double Sharp Click Medium 1 - 100%",
	"Long Double Sharp Click Medium 2 - 80%",
	"Long Double Sharp Click Medium 3 - 60%",
	"Long Double Sharp Tick 1 - 100%",
	"Long Double Sharp Tick 2 - 80%",
	"Long Double Sharp Tick 3 - 60%",
	"Buzz 1 - 100%",
	"Buzz 2 - 80%",
	"Buzz 3 - 60%",
	"Buzz 4 - 40%",
	"Buzz 5 - 20%",
	"Pulsing Strong 1 - 100%",
	"Pulsing Strong 2 - 60%",
	"Pulsing Medium 1 - 100%",
	"Pulsing Medium 2 - 60%",
	"Pulsing Sharp 1 - 100%",
	"Pulsing Sharp 2 - 60%",
	"Transition Click 1 - 100%",
	"Transition Click 2 - 80%",
	"Transition Click 3 - 60%",
	"Transition Click 4 - 40%",
	"Transition Click 5 - 20%",
	"Transition Click 6 - 10%",
	"Transition Hum 1 - 100%",
	"Transition Hum 2 - 80%",
	"Transition Hum 3 - 60%",
	"Transition Hum 4 - 40%",
	"Transition Hum 5 - 20%",
	"Transition Hum 6 - 10%",
	"Transition Ramp Down Long Smooth 1 - 100 to 0%",
	"Transition Ramp Down Long Smooth 2 - 100 to 0%",
	"Transition Ramp Down Medium Smooth 1 - 100 to 0%",
	"Transition Ramp Down Medium Smooth 2 - 100 to 0%",
	"Transition Ramp Down Short Smooth 1 - 100 to 0%",
	"Transition Ramp Down Short Smooth 2 - 100 to 0%",
	"Transition Ramp Down Long Sharp 1 - 100 to 0%",
	"Transition Ramp Down Long Sharp 2 - 100 to 0%",
	"Transition Ramp Down Medium Sharp 1 - 100 to 0%",
	"Transition Ramp Down Medium Sharp 2 - 100 to 0%",
	"Transition Ramp Down Short Sharp 1 - 100 to 0%",
	"Transition Ramp Down Short Sharp 2 - 100 to 0%",
	"Transition Ramp Up Long Smooth 1 - 0 to 100%",
	"Transition Ramp Up Long Smooth 2 - 0 to 100%",
	"Transition Ramp Up Medium Smooth 1 - 0 to 100%",
	"Transition Ramp Up Medium Smooth 2 - 0 to 100%",
	"Transition Ramp Up Short Smooth 1 - 0 to 100%",
	"Transition Ramp Up Short Smooth 2 - 0 to 100%",
	"Transition Ramp Up Long Sharp 1 - 0 to 100%",
	"Transition Ramp Up Long Sharp 2 - 0 to 100%",
	"Transition Ramp Up Medium Sharp 1 - 0 to 100%",
	"Transition Ramp Up Medium Sharp 2 - 0 to 100%",
	"Transition Ramp Up Short Sharp 1 - 0 to 100%",
	"Transition Ramp Up Short Sharp 2 - 0 to 100%",
	"Transition Ramp Down Long Smooth 1 - 50 to 0%",
	"Transition Ramp Down Long Smooth 2 - 50 to 0%",
	"Transition Ramp Down Medium Smooth 1 - 50 to 0%",
	"Transition Ramp Down Medium Smooth 2 - 50 to 0%",
	"Transition Ramp Down Short Smooth 1 - 50 to 0%",
	"Transition Ramp Down Short Smooth 2 - 50 to 0%",
	"Transition Ramp Down Long Sharp 1 - 50 to 0%",
	"Transition Ramp Down Long Sharp 2 - 50 to 0%",
	"Transition Ramp Down Medium Sharp 1 - 50 to 0%",
	"Transition Ramp Down Medium Sharp 2 - 50 to 0%",
	"Transition Ramp Down Short Sharp 1 - 50 to 0%",
	"Transition Ramp Down Short Sharp 2 - 50 to 0%",
	"Transition Ramp Up Long Smooth 1 - 0 to 50%",
	"Transition Ramp Up Long Smooth 2 - 0 to 50%",
	"Transition Ramp Up Medium Smooth 1 - 0 to 50%",
	"Transition Ramp Up Medium Smooth 2 - 0 to 50%",
	"Transition Ramp Up Short Smooth 1 - 0 to 50%",
	"Transition Ramp Up Short Smooth 2 - 0 to 50%",
	"Transition Ramp Up Long Sharp 1 - 0 to 50%",
	"Transition Ramp Up Long Sharp 2 - 0 to 50%",
	"Transition Ramp Up Medium Sharp 1 - 0 to 50%",
	"Transition Ramp Up Medium Sharp 2 - 0 to 50%",
	"Transition Ramp Up Short Sharp 1 - 0 to 50%",
	"Transition Ramp Up Short Sharp 2 - 0 to 50%",
	"Long buzz for programmatic stopping - 100%",
	"Smooth Hum 1 (No kick or brake pulse) - 50%",
	"Smooth Hum 2 (No kick or brake pulse) - 40%",
	"Smooth Hum 3 (No kick or brake pulse) - 30%",
	"Smooth Hum 4 (No kick or brake pulse) - 20%",
	"Smooth Hum 5 (No kick or brake pulse) - 10%"
};

static const char effects_string_page_1[] = {"\
ID      Effect\n\
--------------------------\n\
1       Strong Click - 100%\n\
2       Strong Click - 60%\n\
3       Strong Click - 30%\n\
4       Sharp Click - 100%\n\
5       Sharp Click - 60%\n\
6       Sharp Click - 30%\n\
7       Soft Bump - 100%\n\
8       Soft Bump - 60%\n\
9       Soft Bump - 30%\n\
10      Double Click - 100%\n\
11      Double Click - 60%\n\
12      Triple Click - 100%\n\
13      Soft Fuzz - 60%\n\
14      Strong Buzz - 100%\n\
15      750 ms Alert 100%\n\
16      1000 ms Alert 100%\n\
17      Strong Click 1 - 100%\n\
18      Strong Click 2 - 80%\n\
19      Strong Click 3 - 60%\n\
20      Strong Click 4 - 30%\n\
21      Medium Click 1 - 100%\n\
22      Medium Click 2 - 80%\n\
23      Medium Click 3 - 60%\n\
24      Sharp Tick 1 - 100%\n\
25      Sharp Tick 2 - 80%\n\
26      Sharp Tick 3 - 60%\n\
27      Short Double Click Strong 1 - 100%\n\
28      Short Double Click Strong 2 - 80%\n\
29      Short Double Click Strong 3 - 60%\n\
30      Short Double Click Strong 4 - 30%\n\
31      Short Double Click Medium 1 - 100%\n\
32      Short Double Click Medium 2 - 80%\n\
33      Short Double Click Medium 3 - 60%\n\
34      Short Double Sharp Tick 1 - 100%\n\
35      Short Double Sharp Tick 2 - 80%\n\
36      Short Double Sharp Tick 3 - 60%\n\
37      Long Double Sharp Click Strong 1 - 100%\n\
38      Long Double Sharp Click Strong 2 - 80%\n\
39      Long Double Sharp Click Strong 3 - 60%\n\
40      Long Double Sharp Click Strong 4 - 30%\n\
41      Long Double Sharp Click Medium 1 - 100%\n\
42      Long Double Sharp Click Medium 2 - 80%\n\
43      Long Double Sharp Click Medium 3 - 60%\n\
44      Long Double Sharp Tick 1 - 100%\n\
45      Long Double Sharp Tick 2 - 80%\n\
46      Long Double Sharp Tick 3 - 60%\n\
47      Buzz 1 - 100%\n\
48      Buzz 2 - 80%\n\
49      Buzz 3 - 60%\n\
50      Buzz 4 - 40%\n\
51      Buzz 5 - 20%\n\
52      Pulsing Strong 1 - 100%\n\
53      Pulsing Strong 2 - 60%\n\
54      Pulsing Medium 1 - 100%\n\
55      Pulsing Medium 2 - 60%\n\
56      Pulsing Sharp 1 - 100%\n\
57      Pulsing Sharp 2 - 60%\n\
58      Transition Click 1 - 100%\n\
59      Transition Click 2 - 80%\n\
60      Transition Click 3 - 60%\n\
61      Transition Click 4 - 40%\n\
62      Transition Click 5 - 20%\n\
63      Transition Click 6 - 10%\n\
"
};

static const char effects_string_page_2[] = {"\
ID      Effect\n\
--------------------------\n\
64      Transition Hum 1 - 100%\n\
65      Transition Hum 2 - 80%\n\
66      Transition Hum 3 - 60%\n\
67      Transition Hum 4 - 40%\n\
68      Transition Hum 5 - 20%\n\
69      Transition Hum 6 - 10%\n\
70      Transition Ramp Down Long Smooth 1 - 100 to 0%\n\
71      Transition Ramp Down Long Smooth 2 - 100 to 0%\n\
72      Transition Ramp Down Medium Smooth 1 - 100 to 0%\n\
73      Transition Ramp Down Medium Smooth 2 - 100 to 0%\n\
74      Transition Ramp Down Short Smooth 1 - 100 to 0%\n\
75      Transition Ramp Down Short Smooth 2 - 100 to 0%\n\
76      Transition Ramp Down Long Sharp 1 - 100 to 0%\n\
77      Transition Ramp Down Long Sharp 2 - 100 to 0%\n\
78      Transition Ramp Down Medium Sharp 1 - 100 to 0%\n\
79      Transition Ramp Down Medium Sharp 2 - 100 to 0%\n\
80      Transition Ramp Down Short Sharp 1 - 100 to 0%\n\
81      Transition Ramp Down Short Sharp 2 - 100 to 0%\n\
82      Transition Ramp Up Long Smooth 1 - 0 to 100%\n\
83      Transition Ramp Up Long Smooth 2 - 0 to 100%\n\
84      Transition Ramp Up Medium Smooth 1 - 0 to 100%\n\
85      Transition Ramp Up Medium Smooth 2 - 0 to 100%\n\
86      Transition Ramp Up Short Smooth 1 - 0 to 100%\n\
87      Transition Ramp Up Short Smooth 2 - 0 to 100%\n\
88      Transition Ramp Up Long Sharp 1 - 0 to 100%\n\
89      Transition Ramp Up Long Sharp 2 - 0 to 100%\n\
90      Transition Ramp Up Medium Sharp 1 - 0 to 100%\n\
91      Transition Ramp Up Medium Sharp 2 - 0 to 100%\n\
92      Transition Ramp Up Short Sharp 1 - 0 to 100%\n\
93      Transition Ramp Up Short Sharp 2 - 0 to 100%\n\
94      Transition Ramp Down Long Smooth 1 - 50 to 0%\n\
95      Transition Ramp Down Long Smooth 2 - 50 to 0%\n\
96      Transition Ramp Down Medium Smooth 1 - 50 to 0%\n\
97      Transition Ramp Down Medium Smooth 2 - 50 to 0%\n\
98      Transition Ramp Down Short Smooth 1 - 50 to 0%\n\
99      Transition Ramp Down Short Smooth 2 - 50 to 0%\n\
100     Transition Ramp Down Long Sharp 1 - 50 to 0%\n\
101     Transition Ramp Down Long Sharp 2 - 50 to 0%\n\
102     Transition Ramp Down Medium Sharp 1 - 50 to 0%\n\
103     Transition Ramp Down Medium Sharp 2 - 50 to 0%\n\
104     Transition Ramp Down Short Sharp 1 - 50 to 0%\n\
105     Transition Ramp Down Short Sharp 2 - 50 to 0%\n\
106     Transition Ramp Up Long Smooth 1 - 0 to 50%\n\
107     Transition Ramp Up Long Smooth 2 - 0 to 50%\n\
108     Transition Ramp Up Medium Smooth 1 - 0 to 50%\n\
109     Transition Ramp Up Medium Smooth 2 - 0 to 50%\n\
110     Transition Ramp Up Short Smooth 1 - 0 to 50%\n\
111     Transition Ramp Up Short Smooth 2 - 0 to 50%\n\
112     Transition Ramp Up Long Sharp 1 - 0 to 50%\n\
113     Transition Ramp Up Long Sharp 2 - 0 to 50%\n\
114     Transition Ramp Up Medium Sharp 1 - 0 to 50%\n\
115     Transition Ramp Up Medium Sharp 2 - 0 to 50%\n\
116     Transition Ramp Up Short Sharp 1 - 0 to 50%\n\
117     Transition Ramp Up Short Sharp 2 - 0 to 50%\n\
118     Long buzz for programmatic stopping - 100%\n\
119     Smooth Hum 1 (No kick or brake pulse) - 50%\n\
120     Smooth Hum 2 (No kick or brake pulse) - 40%\n\
121     Smooth Hum 3 (No kick or brake pulse) - 30%\n\
122     Smooth Hum 4 (No kick or brake pulse) - 20%\n\
123     Smooth Hum 5 (No kick or brake pulse) - 10%\n\
"
};

static struct kobject *drv260x_kobj;
static struct drv260x_data *haptics;

/*
 * Rated and Overdriver Voltages:
 * Calculated using the formula r = v * 255 / 5.6
 * where r is what will be written to the register
 * and v is the rated or overdriver voltage of the actuator
 */
static int drv260x_calculate_voltage(unsigned int voltage)
{
	return (voltage * 255 / 5600);
}

static void drv260x_worker(struct work_struct *work)
{
	struct drv260x_data *haptics = container_of(work, struct drv260x_data, work);
	int error;

	gpiod_set_value(haptics->enable_gpio, 1);
	/* Data sheet says to wait 250us before trying to communicate */
	udelay(250);

	error = regmap_write(haptics->regmap, DRV260X_MODE, DRV260X_INTERNAL_TRIGGER);
	if (error) {
		dev_err(&haptics->client->dev, "Failed to write set mode: %d\n", error);
	} else {
		error = regmap_write(haptics->regmap, DRV260X_WV_SEQ_1, haptics->effect_id);
		if (error) {
			dev_err(&haptics->client->dev, "Failed to set effect: %d\n", error);
		} else {
			error = regmap_write(haptics->regmap, DRV260X_GO, DRV260X_GO_BIT);
			if (error)
				dev_err(&haptics->client->dev, "Failed to set go_bit: %d\n", error);
		}	
	}
}

static void drv260x_haptics_play(void)
{
	schedule_work(&haptics->work);
}

static const struct reg_sequence drv260x_lra_cal_regs[] = {
	{ DRV260X_MODE, DRV260X_AUTO_CAL },
	{ DRV260X_CTRL3, DRV260X_NG_THRESH_2 },
	{ DRV260X_FEEDBACK_CTRL, DRV260X_FB_REG_LRA_MODE | DRV260X_BRAKE_FACTOR_4X | DRV260X_LOOP_GAIN_MED},
};

static const struct reg_sequence drv260x_lra_init_regs[] = {
	{ DRV260X_MODE, DRV260X_INTERNAL_TRIGGER },
	{ DRV260X_FEEDBACK_CTRL, DRV260X_FB_REG_LRA_MODE | DRV260X_BRAKE_FACTOR_4X | DRV260X_LOOP_GAIN_MED},
};

static const struct reg_sequence drv260x_erm_cal_regs[] = {
	{ DRV260X_MODE, DRV260X_AUTO_CAL },
	{ DRV260X_A_TO_V_MIN_INPUT, DRV260X_AUDIO_HAPTICS_MIN_IN_VOLT },
	{ DRV260X_A_TO_V_MAX_INPUT, DRV260X_AUDIO_HAPTICS_MAX_IN_VOLT },
	{ DRV260X_A_TO_V_MIN_OUT, DRV260X_AUDIO_HAPTICS_MIN_OUT_VOLT },
	{ DRV260X_A_TO_V_MAX_OUT, DRV260X_AUDIO_HAPTICS_MAX_OUT_VOLT },
	{ DRV260X_FEEDBACK_CTRL, DRV260X_BRAKE_FACTOR_3X | DRV260X_LOOP_GAIN_MED | DRV260X_BEMF_GAIN_2 },
	{ DRV260X_CTRL1, DRV260X_STARTUP_BOOST },
	{ DRV260X_CTRL2, DRV260X_SAMP_TIME_250 | DRV260X_BLANK_TIME_75 | DRV260X_IDISS_TIME_75 },
	{ DRV260X_CTRL3, DRV260X_NG_THRESH_2 | DRV260X_ERM_OPEN_LOOP },
	{ DRV260X_CTRL4, DRV260X_AUTOCAL_TIME_500MS },
};

static int drv260x_init(void)
{
	int error;
	unsigned int cal_buf;

	error = regmap_write(haptics->regmap, DRV260X_RATED_VOLT, haptics->rated_voltage);
	if (error) {
		dev_err(&haptics->client->dev, "Failed to write DRV260X_RATED_VOLT register: %d\n", error);
		return error;
	}

	error = regmap_write(haptics->regmap, DRV260X_OD_CLAMP_VOLT, haptics->overdrive_voltage);
	if (error) {
		dev_err(&haptics->client->dev, "Failed to write DRV260X_OD_CLAMP_VOLT register: %d\n", error);
		return error;
	}

	switch (haptics->mode) {
	case DRV260X_LRA_MODE:
		error = regmap_register_patch(haptics->regmap, drv260x_lra_init_regs, ARRAY_SIZE(drv260x_lra_init_regs));
		if (error) {
			dev_err(&haptics->client->dev, "Failed to write LRA feedback registers: %d\n", error);
			return error;
		}

		break;

	case DRV260X_ERM_MODE:
		error = regmap_register_patch(haptics->regmap, drv260x_erm_cal_regs, ARRAY_SIZE(drv260x_erm_cal_regs));
		if (error) {
			dev_err(&haptics->client->dev, "Failed to write ERM calibration registers: %d\n", error);
			return error;
		}

		error = regmap_update_bits(haptics->regmap, DRV260X_LIB_SEL, DRV260X_LIB_SEL_MASK, haptics->library);
		if (error) {
			dev_err(&haptics->client->dev, "Failed to write DRV260X_LIB_SEL register: %d\n", error);
			return error;
		}

		break;

	default:
		error = regmap_register_patch(haptics->regmap, drv260x_lra_init_regs, ARRAY_SIZE(drv260x_lra_init_regs));
		if (error) {
			dev_err(&haptics->client->dev, "Failed to write LRA init registers: %d\n", error);
			return error;
		}

		error = regmap_update_bits(haptics->regmap, DRV260X_LIB_SEL, DRV260X_LIB_SEL_MASK, haptics->library);
		if (error) {
			dev_err(&haptics->client->dev, "Failed to write DRV260X_LIB_SEL register: %d\n", error);
			return error;
		}

		/* No need to set GO bit here */
		return 0;
	}

	return 0;
}

static const struct regmap_config drv260x_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,

	.max_register = DRV260X_MAX_REG,
	.reg_defaults = drv260x_reg_defs,
	.num_reg_defaults = ARRAY_SIZE(drv260x_reg_defs),
	.cache_type = REGCACHE_NONE,
};

static ssize_t effects_page1_show(struct kobject *kobj, struct kobj_attribute *attr, char *buffer)
{
	return sprintf(buffer, "%s\n", effects_string_page_1);
}

static ssize_t effects_page2_show(struct kobject *kobj, struct kobj_attribute *attr, char *buffer)
{
	return sprintf(buffer, "%s\n", effects_string_page_2);
}

static ssize_t effect_id_show(struct kobject *kobj, struct kobj_attribute *attr, char *buffer)
{
	return sprintf(buffer, "%i\n", haptics->effect_id);
}

static ssize_t effect_id_store(struct kobject *kobj, struct kobj_attribute *attr, const char *buffer, size_t count)
{
	int effect_id;
	int error;

	error = kstrtoint(buffer, 10, &effect_id);
	if (error)
		dev_err(&haptics->client->dev, "Failed to convert string: %d\n", error);

	if(effect_id == 0 || effect_id > 123)
		haptics->effect_id = 1;
	else
		haptics->effect_id = effect_id;

	return count;
}

static ssize_t play_store(struct kobject *kobj, struct kobj_attribute *attr, const char *buffer, size_t count)
{
	if(buffer[0] == DRV260X_GO_BIT_ASCII)
		drv260x_haptics_play();

	return count;
}

static struct kobj_attribute effects_page1_attr = __ATTR(effects1, 0440, effects_page1_show, NULL);
static struct kobj_attribute effects_page2_attr = __ATTR(effects2, 0440, effects_page2_show, NULL);
static struct kobj_attribute effect_id_attr = __ATTR(effect_id, 0660, effect_id_show, effect_id_store);
static struct kobj_attribute play_attr = __ATTR(play, 0220, NULL, play_store);

static int drv260x_create_fs(struct device *dev)
{
    drv260x_kobj = kobject_create_and_add("drv2605l", kernel_kobj);
    if(!drv260x_kobj) {
        dev_err(dev, "Error creating /sys/kernel/drv2605l\n");
        return -ENOMEM;
    }

    if(sysfs_create_file(drv260x_kobj, &effects_page1_attr.attr)) {
		dev_err(dev, "Error creating /sys/kernel/drv2605l/effects1\n");
		kobject_put(drv260x_kobj);
		return -ENOMEM;
	}

	if(sysfs_create_file(drv260x_kobj, &effects_page2_attr.attr)) {
		dev_err(dev, "Error creating /sys/kernel/drv2605l/effects2\n");
		kobject_put(drv260x_kobj);
		return -ENOMEM;
	}

	if(sysfs_create_file(drv260x_kobj, &effect_id_attr.attr)) {
		dev_err(dev, "Error creating /sys/kernel/drv2605l/effect_id\n");
		kobject_put(drv260x_kobj);
		return -ENOMEM;
	}

	if(sysfs_create_file(drv260x_kobj, &play_attr.attr)) {
		dev_err(dev, "Error creating /sys/kernel/drv2605l/play\n");
		kobject_put(drv260x_kobj);
		return -ENOMEM;
	}

	return 0;
}

static int drv260x_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct device *dev = &client->dev;
	u32 voltage;
	int error;

	haptics = devm_kzalloc(dev, sizeof(*haptics), GFP_KERNEL);
	if(!haptics)
		return -ENOMEM;

	error = device_property_read_u32(dev, "mode", &haptics->mode);
	if(error)
	{
		dev_err(dev, "Can't fetch 'mode' property: %d\n", error);
		return error;
	}

	if(haptics->mode < DRV260X_LRA_MODE || haptics->mode > DRV260X_ERM_MODE)
	{
		dev_err(dev, "Vibrator mode is invalid: %i\n", haptics->mode);
		return -EINVAL;
	}

	error = device_property_read_u32(dev, "library-sel", &haptics->library);
	if(error)
	{
		dev_err(dev, "Can't fetch 'library-sel' property: %d\n", error);
		return error;
	}

	if(haptics->library < DRV260X_LIB_EMPTY || haptics->library > DRV260X_ERM_LIB_F)
	{
		dev_err(dev, "Library value is invalid: %i\n", haptics->library);
		return -EINVAL;
	}

	if (haptics->mode == DRV260X_LRA_MODE && haptics->library != DRV260X_LIB_EMPTY && haptics->library != DRV260X_LIB_LRA)
	{
		dev_err(dev, "LRA Mode with ERM Library mismatch\n");
		return -EINVAL;
	}

	if(haptics->mode == DRV260X_ERM_MODE && (haptics->library == DRV260X_LIB_EMPTY || haptics->library == DRV260X_LIB_LRA))
	{
		dev_err(dev, "ERM Mode with LRA Library mismatch\n");
		return -EINVAL;
	}

	error = device_property_read_u32(dev, "vib-rated-mv", &voltage);
	haptics->rated_voltage = error ? DRV260X_DEF_RATED_VOLT : drv260x_calculate_voltage(voltage);

	error = device_property_read_u32(dev, "vib-overdrive-mv", &voltage);
	haptics->overdrive_voltage = error ? DRV260X_DEF_OD_CLAMP_VOLT : drv260x_calculate_voltage(voltage);

	haptics->regulator = devm_regulator_get(dev, "vbat");
	if(IS_ERR(haptics->regulator))
	{
		error = PTR_ERR(haptics->regulator);
		dev_err(dev, "Unable to get regulator, error: %d\n", error);
		return error;
	}

	haptics->enable_gpio = devm_gpiod_get_optional(dev, "enable", GPIOD_OUT_HIGH);
	if(IS_ERR(haptics->enable_gpio))
		return PTR_ERR(haptics->enable_gpio);

	INIT_WORK(&haptics->work, drv260x_worker);

	haptics->client = client;
	i2c_set_clientdata(client, haptics);

	haptics->regmap = devm_regmap_init_i2c(client, &drv260x_regmap_config);
	if(IS_ERR(haptics->regmap))
	{
		error = PTR_ERR(haptics->regmap);
		dev_err(dev, "Failed to allocate register map: %d\n", error);
		return error;
	}

    error = drv260x_create_fs(dev);
    if(error)
	{
		dev_err(dev, "File System create failed: %d\n", error);
		return error;
	}

	error = drv260x_init();
	if(error)
	{
		dev_err(dev, "Device init failed: %d\n", error);
		return error;
	}

	haptics->effect_id = 1;
    
	return 0;
}

static int drv260x_remove(struct i2c_client *client)
{
	sysfs_remove_file(drv260x_kobj, &effects_page1_attr.attr);
	sysfs_remove_file(drv260x_kobj, &effects_page2_attr.attr);
	sysfs_remove_file(drv260x_kobj, &effect_id_attr.attr);
	sysfs_remove_file(drv260x_kobj, &play_attr.attr);
	kobject_put(drv260x_kobj);
	
	return 0;
}

static int drv260x_suspend(struct device *dev)
{
	struct drv260x_data *haptics = dev_get_drvdata(dev);
	int ret = 0;

	ret = regmap_update_bits(haptics->regmap, DRV260X_MODE, DRV260X_STANDBY_MASK, DRV260X_STANDBY);
	
	if(ret)
	{
		dev_err(dev, "Failed to set standby mode\n");
		goto out;
	}

	gpiod_set_value(haptics->enable_gpio, 0);

	ret = regulator_disable(haptics->regulator);
	if(ret)
	{
		dev_err(dev, "Failed to disable regulator\n");
		regmap_update_bits(haptics->regmap, DRV260X_MODE, DRV260X_STANDBY_MASK, 0);
	}
out:
	return ret;
}

static int drv260x_resume(struct device *dev)
{
	struct drv260x_data *haptics = dev_get_drvdata(dev);
	int ret = 0;

	ret = regulator_enable(haptics->regulator);
	if(ret)
	{
		dev_err(dev, "Failed to enable regulator\n");
		goto out;
	}

	ret = regmap_update_bits(haptics->regmap, DRV260X_MODE, DRV260X_STANDBY_MASK, 0);
	if(ret)
	{
		dev_err(dev, "Failed to unset standby mode\n");
		regulator_disable(haptics->regulator);
		goto out;
	}

	gpiod_set_value(haptics->enable_gpio, 1);
out:
	return ret;
}

static const struct dev_pm_ops drv260x_pm_ops = {
	.suspend = drv260x_suspend,
	.resume = drv260x_resume,
};

static const struct i2c_device_id drv260x_id[] = {
	{ "drv2605l", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, drv260x_id);

static const struct of_device_id drv260x_of_match[] = {
	{ .compatible = "ti,drv2604", },
	{ .compatible = "ti,drv2604l", },
	{ .compatible = "ti,drv2605", },
	{ .compatible = "ti,drv2605l", },
	{ }
};
MODULE_DEVICE_TABLE(of, drv260x_of_match);

static struct i2c_driver drv260x_driver = {
	.probe	= drv260x_probe,
	.remove = drv260x_remove,
	.driver		= {
		.name	= "drv260x-haptics",
		.pm	= &drv260x_pm_ops,
		.of_match_table = drv260x_of_match,
	},
	.id_table = drv260x_id,
};
module_i2c_driver(drv260x_driver);

MODULE_DESCRIPTION("TI drv2605l magnosco specific haptics driver");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Marcel Burde <marcel.burde@magnosco.com>");
