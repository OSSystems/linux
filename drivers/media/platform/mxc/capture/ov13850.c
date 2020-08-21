/*
 * ov13850 sensor level driver
 * This driver used to capture raw BGGR data
 * 4 lanes, mipi 8 bit mode
 */

#include <linux/clk.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/pm_runtime.h>
#include <linux/regulator/consumer.h>
#include <linux/sysfs.h>
#include <linux/slab.h>
#include <linux/version.h>
#include <linux/mipi_csi2.h>
#include <media/media-entity.h>
#include <linux/pinctrl/consumer.h>
#include <linux/of_gpio.h>
#include "v4l2-int-device.h"
#include "mxc_v4l2_capture.h"

#define OV13850_XVCLK_FREQ		24000000

#define CHIP_ID				0x00d850
#define OV13850_REG_CHIP_ID		0x300a

#define OV13850_REG_CTRL_MODE		0x0100
#define OV13850_MODE_SW_STANDBY		0x0
#define OV13850_MODE_STREAMING		BIT(0)

#define OV13850_REG_EXPOSURE		0x3500
#define	OV13850_EXPOSURE_MIN		4
#define	OV13850_EXPOSURE_STEP		1
#define OV13850_VTS_MAX			0x7fff

#define OV13850_REG_GAIN_H		0x350a
#define OV13850_REG_GAIN_L		0x350b
#define OV13850_GAIN_H_MASK		0x07
#define OV13850_GAIN_H_SHIFT		8
#define OV13850_GAIN_L_MASK		0xff
#define OV13850_GAIN_MIN		0x10
#define OV13850_GAIN_MAX		0xf8
#define OV13850_GAIN_STEP		1
#define OV13850_GAIN_DEFAULT		0x10

#define OV13850_REG_TEST_PATTERN	0x5e00
#define ENABLE_TEST_PATTERN	0

#define OV13850_REG_VTS			0x380e

#define REG_NULL			0xFFFF

#define OV13850_REG_VALUE_08BIT		1
#define OV13850_REG_VALUE_16BIT		2
#define OV13850_REG_VALUE_24BIT		3

#define OV13850_LANES			4
#define OV13850_BITS_PER_SAMPLE		10

#define OV13850_CHIP_REVISION_REG	0x302A
#define OV13850_R1A			0xb1
#define OV13850_R2A			0xb2

#define OF_CAMERA_PINCTRL_STATE_DEFAULT	"ov13850_camera_default"
#define OF_CAMERA_PINCTRL_STATE_SLEEP	"ov13850_camera_sleep"

#define OV13850_NAME			"ov13850_mipi_raw"
#define OV13850_VCHANNEL	0x4813

#define OV13850_RES_HIGH_WIDTH		4224
#define OV13850_RES_HIGH_HEIGHT		3136
#define OV13850_RES_LOW_WIDTH		2112
#define OV13850_RES_LOW_HEIGHT		1568

#define DEFAULT_FPS	15

static int reset_gpio;//, pwdn_gpio;
static struct sensor_data ov13850_data;
static u32 ov13850_revid;

static const struct regval *ov13850_global_regs;

struct regval {
	u16 addr;
	u8 val;
};

struct ov13850_mode {
	u32 width;
	u32 height;
	struct v4l2_fract max_fps;
	u32 hts_def;
	u32 vts_def;
	u32 exp_def;
	const struct regval *reg_list;
};

struct ov13850 {
	struct i2c_client	*client;
	struct clk		*xvclk;
	int			reset_gpio;
	int			pwdn_gpio;

	struct pinctrl		*pinctrl;
	struct pinctrl_state	*pins_default;
	struct pinctrl_state	*pins_sleep;

	struct media_pad	pad;
	struct v4l2_ctrl	*exposure;
	struct v4l2_ctrl	*anal_gain;
	struct v4l2_ctrl	*digi_gain;
	struct v4l2_ctrl	*hblank;
	struct v4l2_ctrl	*vblank;
	struct v4l2_ctrl	*test_pattern;
	struct mutex		mutex;
	bool			streaming;
	bool			power_on;
	const struct ov13850_mode *cur_mode;
	u32			module_index;
};

struct ov13850 ov13850_info;

/*Init settings for 3.3MP resolution*/
static const struct regval ov13850_init_2112x1568_r2a[] = {
	{0x0103, 0x01},
	{0x0300, 0x01},
	{0x0301, 0x00},
	{0x0302, 0x28},
	{0x0303, 0x00},
	{0x030a, 0x00},
	{0x300f, 0x10},
	{0x3010, 0x01},
	{0x3011, 0x76},
	{0x3012, 0x41},
	{0x3013, 0x12},
	{0x3014, 0x11},
	{0x301f, 0x03},
	{0x3106, 0x00},
	{0x3210, 0x47},
	{0x3500, 0x00},
	{0x3501, 0xc0},
	{0x3502, 0x00},
	{0x3506, 0x00},
	{0x3507, 0x02},
	{0x3508, 0x00},
	{0x350a, 0x00},
	{0x350b, 0x80},
	{0x350e, 0x00},
	{0x350f, 0x10},
	{0x351a, 0x00},
	{0x351b, 0x10},
	{0x351c, 0x00},
	{0x351d, 0x20},
	{0x351e, 0x00},
	{0x351f, 0x40},
	{0x3520, 0x00},
	{0x3521, 0x80},
	{0x3600, 0xc0},
	{0x3601, 0xfc},
	{0x3602, 0x02},
	{0x3603, 0x78},
	{0x3604, 0xb1},
	{0x3605, 0x95},
	{0x3606, 0x73},
	{0x3607, 0x07},
	{0x3609, 0x40},
	{0x360a, 0x30},
	{0x360b, 0x91},
	{0x360C, 0x09},
	{0x360f, 0x02},
	{0x3611, 0x10},
	{0x3612, 0x27},
	{0x3613, 0x33},
	{0x3615, 0x0c},
	{0x3616, 0x0e},
	{0x3641, 0x02},
	{0x3660, 0x82},
	{0x3668, 0x54},
	{0x3669, 0x00},
	{0x366a, 0x3f},
	{0x3667, 0xa0},
	{0x3702, 0x40},
	{0x3703, 0x44},
	{0x3704, 0x2c},
	{0x3705, 0x01},
	{0x3706, 0x15},
	{0x3707, 0x44},
	{0x3708, 0x3c},
	{0x3709, 0x1f},
	{0x370a, 0x27},
	{0x370b, 0x3c},
	{0x3720, 0x55},
	{0x3722, 0x84},
	{0x3728, 0x40},
	{0x372a, 0x00},
	{0x372b, 0x02},
	{0x372e, 0x22},
	{0x372f, 0xa0},
	{0x3730, 0x00},
	{0x3731, 0x00},
	{0x3732, 0x00},
	{0x3733, 0x00},
	{0x3710, 0x28},
	{0x3716, 0x03},
	{0x3718, 0x1c},
	{0x3719, 0x0c},
	{0x371a, 0x08},
	{0x371c, 0xfc},
	{0x3748, 0x00},
	{0x3760, 0x13},
	{0x3761, 0x33},
	{0x3762, 0x86},
	{0x3763, 0x16},
	{0x3767, 0x24},
	{0x3768, 0x06},
	{0x3769, 0x45},
	{0x376c, 0x23},
	{0x376f, 0x80},
	{0x3773, 0x06},
	{0x3d84, 0x00},
	{0x3d85, 0x17},
	{0x3d8c, 0x73},
	{0x3d8d, 0xbf},
	{0x3800, 0x00},
	{0x3801, 0x00},
	{0x3802, 0x00},
	{0x3803, 0x04},
	{0x3804, 0x10},
	{0x3805, 0x9f},
	{0x3806, 0x0c},
	{0x3807, 0x4b},
	{0x3808, 0x08},
	{0x3809, 0x40},
	{0x380a, 0x06},
	{0x380b, 0x20},
	{0x380c, 0x11},
	{0x380d, 0xa0},
	{0x380e, 0x0d},
	{0x380f, 0x00},
	{0x3810, 0x00},
	{0x3811, 0x08},
	{0x3812, 0x00},
	{0x3813, 0x02},
	{0x3814, 0x31},
	{0x3815, 0x31},
	{0x3820, 0x01},
	{0x3821, 0x06},
	{0x3823, 0x00},
	{0x3826, 0x00},
	{0x3827, 0x02},
	{0x3834, 0x00},
	{0x3835, 0x1c},
	{0x3836, 0x08},
	{0x3837, 0x02},
	{0x4000, 0xf1},
	{0x4001, 0x00},
	{0x400b, 0x0c},
	{0x4011, 0x00},
	{0x401a, 0x00},
	{0x401b, 0x00},
	{0x401c, 0x00},
	{0x401d, 0x00},
	{0x4020, 0x00},
	{0x4021, 0xe4},
	{0x4022, 0x04},
	{0x4023, 0xd7},
	{0x4024, 0x05},
	{0x4025, 0xbc},
	{0x4026, 0x05},
	{0x4027, 0xbf},
	{0x4028, 0x00},
	{0x4029, 0x02},
	{0x402a, 0x04},
	{0x402b, 0x08},
	{0x402c, 0x02},
	{0x402d, 0x02},
	{0x402e, 0x0c},
	{0x402f, 0x08},
	{0x403d, 0x2c},
	{0x403f, 0x7F},
	{0x4041, 0x07},
	{0x4500, 0x82},
	{0x4501, 0x3c},
	{0x458b, 0x00},
	{0x459c, 0x00},
	{0x459d, 0x00},
	{0x459e, 0x00},
	{0x4601, 0x83},
	{0x4602, 0x22},
	{0x4603, 0x01},
	{0x4837, 0x19},
	{0x4d00, 0x04},
	{0x4d01, 0x42},
	{0x4d02, 0xd1},
	{0x4d03, 0x90},
	{0x4d04, 0x66},
	{0x4d05, 0x65},
	{0x4d0b, 0x00},
	{0x5000, 0x0e},
	{0x5001, 0x01},
	{0x5002, 0x07},
	{0x5003, 0x4f},
	{0x5013, 0x40},
	{0x501c, 0x00},
	{0x501d, 0x10},
	{0x5100, 0x30},
	{0x5101, 0x02},
	{0x5102, 0x01},
	{0x5103, 0x01},
	{0x5104, 0x02},
	{0x5105, 0x01},
	{0x5106, 0x01},
	{0x5107, 0x00},
	{0x5108, 0x00},
	{0x5109, 0x00},
	{0x510f, 0xfc},
	{0x5110, 0xf0},
	{0x5111, 0x10},
	{0x536d, 0x02},
	{0x536e, 0x67},
	{0x536f, 0x01},
	{0x5370, 0x4c},
	{0x5400, 0x00},
	{0x5400, 0x00},
	{0x5401, 0x61},
	{0x5402, 0x00},
	{0x5403, 0x00},
	{0x5404, 0x00},
	{0x5405, 0x40},
	{0x540c, 0x05},
	{0x5501, 0x00},
	{0x5b00, 0x00},
	{0x5b01, 0x00},
	{0x5b02, 0x01},
	{0x5b03, 0xff},
	{0x5b04, 0x02},
	{0x5b05, 0x6c},
	{0x5b09, 0x02},
	{0x5e10, 0x1c},
	{REG_NULL, 0x00},
};

/*Init settings for 13MP resolution*/
static const struct regval ov13850_init_4224x3136_r2a[] = {
	{0x0103, 0x01},
	{0x0300, 0x00},
	{0x0301, 0x00},
	{0x0302, 0x32},
	{0x0303, 0x01},
	{0x030a, 0x00},
	{0x300f, 0x10},
	{0x3010, 0x01},
	{0x3011, 0x76},
	{0x3012, 0x41},
	{0x3013, 0x12},
	{0x3014, 0x11},
	{0x301f, 0x03},
	{0x3106, 0x00},
	{0x3210, 0x47},
	{0x3500, 0x00},
	{0x3501, 0xc0},
	{0x3502, 0x00},
	{0x3506, 0x00},
	{0x3507, 0x02},
	{0x3508, 0x00},
	{0x350a, 0x00},
	{0x350b, 0x80},
	{0x350e, 0x00},
	{0x350f, 0x10},
	{0x351a, 0x00},
	{0x351b, 0x10},
	{0x351c, 0x00},
	{0x351d, 0x20},
	{0x351e, 0x00},
	{0x351f, 0x40},
	{0x3520, 0x00},
	{0x3521, 0x80},
	{0x3600, 0xc0},
	{0x3601, 0xfc},
	{0x3602, 0x02},
	{0x3603, 0x78},
	{0x3604, 0xb1},
	{0x3605, 0x95},
	{0x3606, 0x73},
	{0x3607, 0x07},
	{0x3609, 0x40},
	{0x360a, 0x30},
	{0x360b, 0x91},
	{0x360C, 0x09},
	{0x360f, 0x02},
	{0x3611, 0x10},
	{0x3612, 0x28},
	{0x3613, 0x33},
	{0x3614, 0x2a},
	{0x3615, 0x0c},
	{0x3616, 0x0e},
	{0x3641, 0x02},
	{0x3660, 0x82},
	{0x3668, 0x54},
	{0x3669, 0x00},
	{0x366a, 0x3f},
	{0x3667, 0xa0},
	{0x3702, 0x40},
	{0x3703, 0x44},
	{0x3704, 0x2c},
	{0x3705, 0x01},
	{0x3706, 0x15},
	{0x3707, 0x44},
	{0x3708, 0x3c},
	{0x3709, 0x1f},
	{0x370a, 0x24},
	{0x370b, 0x3c},
	{0x3710, 0x28},
	{0x3716, 0x03},
	{0x3718, 0x10},
	{0x3719, 0x0c},
	{0x371a, 0x08},
	{0x371b, 0x01},
	{0x371c, 0xfc},
	{0x3720, 0x55},
	{0x3722, 0x84},
	{0x3728, 0x40},
	{0x372a, 0x05},
	{0x372b, 0x02},
	{0x372e, 0x22},
	{0x372f, 0xa0},
	{0x3730, 0x04},
	{0x3731, 0xb8},
	{0x3732, 0x04},
	{0x3733, 0xcc},
	{0x3738, 0x04},
	{0x3739, 0xce},
	{0x373a, 0x04},
	{0x373b, 0xd0},
	{0x3740, 0x01},
	{0x3741, 0xd0},
	{0x3742, 0x00},
	{0x3743, 0x01},
	{0x3748, 0x21},
	{0x3749, 0x22},
	{0x374a, 0x28},
	{0x3760, 0x13},
	{0x3761, 0x33},
	{0x3762, 0x86},
	{0x3763, 0x16},
	{0x3767, 0x24},
	{0x3768, 0x06},
	{0x3769, 0x45},
	{0x376c, 0x23},
	{0x376f, 0x80},
	{0x3773, 0x06},
	{0x3780, 0x90},
	{0x3781, 0x00},
	{0x3782, 0x01},
	{0x3d84, 0x00},
	{0x3d85, 0x17},
	{0x3d8c, 0x73},
	{0x3d8d, 0xbf},
	{0x3800, 0x00},
	{0x3801, 0x0C},
	{0x3802, 0x00},
	{0x3803, 0x04},
	{0x3804, 0x10},
	{0x3805, 0x93},
	{0x3806, 0x0c},
	{0x3807, 0x4B},
	{0x3808, 0x10},
	{0x3809, 0x80},
	{0x380a, 0x0c},
	{0x380b, 0x40},
	{0x380c, 0x11},
	{0x380d, 0xa0},
	{0x380e, 0x0d},
	{0x380f, 0x00},
	{0x3810, 0x00},
	{0x3811, 0x04},
	{0x3812, 0x00},
	{0x3813, 0x04},
	{0x3814, 0x11},
	{0x3815, 0x11},
	{0x3820, 0x00},
	{0x3821, 0x04},
	{0x3823, 0x00},
	{0x3826, 0x00},
	{0x3827, 0x02},
	{0x3834, 0x00},
	{0x3835, 0x1c},
	{0x3836, 0x04},
	{0x3837, 0x01},
	{0x4000, 0xf1},
	{0x4001, 0x00},
	{0x400b, 0x0c},
	{0x4011, 0x00},
	{0x401a, 0x00},
	{0x401b, 0x00},
	{0x401c, 0x00},
	{0x401d, 0x00},
	{0x4020, 0x03},
	{0x4021, 0x6C},
	{0x4022, 0x0D},
	{0x4023, 0x17},
	{0x4024, 0x0D},
	{0x4025, 0xFC},
	{0x4026, 0x0D},
	{0x4027, 0xFF},
	{0x4028, 0x00},
	{0x4029, 0x02},
	{0x402a, 0x04},
	{0x402b, 0x08},
	{0x402c, 0x02},
	{0x402d, 0x02},
	{0x402e, 0x0c},
	{0x402f, 0x08},
	{0x403d, 0x2c},
	{0x403f, 0x7F},
	{0x4041, 0x07},
	{0x4500, 0x82},
	{0x4501, 0x38},
	{0x458b, 0x00},
	{0x459c, 0x00},
	{0x459d, 0x00},
	{0x459e, 0x00},
	{0x4601, 0x04},
	{0x4602, 0x22},
	{0x4603, 0x00},
	{0x4837, 0x1b},
	{0x4d00, 0x04},
	{0x4d01, 0x42},
	{0x4d02, 0xd1},
	{0x4d03, 0x90},
	{0x4d04, 0x66},
	{0x4d05, 0x65},
	{0x4d0b, 0x00},
	{0x5000, 0x0e},
	{0x5001, 0x01},
	{0x5002, 0x07},
	{0x5003, 0x4f},
	{0x5013, 0x40},
	{0x501c, 0x00},
	{0x501d, 0x10},
	{0x5100, 0x30},
	{0x5101, 0x02},
	{0x5102, 0x01},
	{0x5103, 0x01},
	{0x5104, 0x02},
	{0x5105, 0x01},
	{0x5106, 0x01},
	{0x5107, 0x00},
	{0x5108, 0x00},
	{0x5109, 0x00},
	{0x510f, 0xfc},
	{0x5110, 0xf0},
	{0x5111, 0x10},
	{0x536d, 0x02},
	{0x536e, 0x67},
	{0x536f, 0x01},
	{0x5370, 0x4c},
	{0x5400, 0x00},
	{0x5400, 0x00},
	{0x5401, 0x71},
	{0x5402, 0x00},
	{0x5403, 0x00},
	{0x5404, 0x00},
	{0x5405, 0x80},
	{0x540c, 0x05},
	{0x5501, 0x00},
	{0x5b00, 0x00},
	{0x5b01, 0x00},
	{0x5b02, 0x01},
	{0x5b03, 0xff},
	{0x5b04, 0x02},
	{0x5b05, 0x6c},
	{0x5b09, 0x02},
	{0x5e00, 0x00},
	{0x5e10, 0x1c},
	{REG_NULL, 0x00},
};

/*3.3MP Resolution at 15fps*/
static const struct regval ov13850_2112x1568_regs[] = {
	{0x3800, 0x00},
	{0x3801, 0x00},
	{0x3802, 0x00},
	{0x3803, 0x04},
	{0x3804, 0x10},
	{0x3805, 0x9f},
	{0x3806, 0x0c},
	{0x3807, 0x4b},
	{0x3808, 0x08},
	{0x3809, 0x40},
	{0x380a, 0x06},
	{0x380b, 0x20},
	{0x380c, 0x11},
	{0x380d, 0xa0},
	{0x380e, 0x0d},
	{0x380f, 0x00},
	{0x3810, 0x00},
	{0x3811, 0x08},
	{0x3812, 0x00},
	{0x3813, 0x02},
	{0x3814, 0x31},
	{0x3815, 0x31},
	{0x3820, 0x01},
	{0x3821, 0x06},
	{0x3823, 0x00},
	{0x3826, 0x00},
	{0x3827, 0x02},
	{0x3834, 0x00},
	{0x3835, 0x1c},
	{0x3836, 0x08},
	{0x3837, 0x02},
	{REG_NULL, 0x00},
};

/*13MP resolution at 7fps*/
static const struct regval ov13850_4224x3136_regs[] = {
	{0x3800, 0x00},
	{0x3801, 0x0C},
	{0x3802, 0x00},
	{0x3803, 0x04},
	{0x3804, 0x10},
	{0x3805, 0x93},
	{0x3806, 0x0c},
	{0x3807, 0x4B},
	{0x3808, 0x10},
	{0x3809, 0x80},
	{0x380a, 0x0c},
	{0x380b, 0x40},
	{0x380c, 0x11},
	{0x380d, 0xa0},
	{0x380e, 0x0d},
	{0x380f, 0x00},
	{0x3810, 0x00},
	{0x3811, 0x04},
	{0x3812, 0x00},
	{0x3813, 0x04},
	{0x3814, 0x11},
	{0x3815, 0x11},
	{0x3820, 0x00},
	{0x3821, 0x04},
	{0x3823, 0x00},
	{0x3826, 0x00},
	{0x3827, 0x02},
	{0x3834, 0x00},
	{0x3835, 0x1c},
	{0x3836, 0x04},
	{0x3837, 0x01},
	{REG_NULL, 0x00},
};

static const struct ov13850_mode supported_modes[] = {
	{
		.width = 2112,
		.height = 1568,
		.max_fps = {
			.numerator = 20000,
			.denominator = 300000,
		},
		.exp_def = 0x0600,
		.hts_def = 0x12c0,
		.vts_def = 0x0680,
		.reg_list = ov13850_2112x1568_regs,
	},{
		.width = 4224,
		.height = 3136,
		.max_fps = {
			.numerator = 20000,
			.denominator = 150000,
		},
		.exp_def = 0x0600,
		.hts_def = 0x12c0,
		.vts_def = 0x0d00,
		.reg_list = ov13850_4224x3136_regs,
	},
};


/* Write registers up to 4 at a time */
static int ov13850_write_reg(struct i2c_client *client, u16 reg,
			     u32 len, u32 val)
{
	u32 buf_i, val_i;
	u8 buf[6];
	u8 *val_p;
	__be32 val_be;

	if (len > 4)
		return -EINVAL;

	buf[0] = reg >> 8;
	buf[1] = reg & 0xff;

	val_be = cpu_to_be32(val);
	val_p = (u8 *)&val_be;
	buf_i = 2;
	val_i = 4 - len;

	while (val_i < 4)
		buf[buf_i++] = val_p[val_i++];

	if (i2c_master_send(client, buf, len + 2) != len + 2)
		return -EIO;

	return 0;
}

static int ov13850_write_array(struct i2c_client *client,
			       const struct regval *regs)
{
	u32 i;
	int ret = 0;

	for (i = 0; ret == 0 && regs[i].addr != REG_NULL; i++)
		ret = ov13850_write_reg(client, regs[i].addr,
					OV13850_REG_VALUE_08BIT,
					regs[i].val);

	return ret;
}

/* Read registers up to 4 at a time */
static int ov13850_read_reg(struct i2c_client *client, u16 reg,
			    unsigned int len, u32 *val)
{
	struct i2c_msg msgs[2];
	u8 *data_be_p;
	__be32 data_be = 0;
	__be16 reg_addr_be = cpu_to_be16(reg);
	int ret;

	if (len > 4 || !len)
		return -EINVAL;

	data_be_p = (u8 *)&data_be;
	/* Write register address */
	msgs[0].addr = client->addr;
	msgs[0].flags = 0;
	msgs[0].len = 2;
	msgs[0].buf = (u8 *)&reg_addr_be;

	/* Read data from register */
	msgs[1].addr = client->addr;
	msgs[1].flags = I2C_M_RD;
	msgs[1].len = len;
	msgs[1].buf = &data_be_p[4 - len];

	ret = i2c_transfer(client->adapter, msgs, ARRAY_SIZE(msgs));
	if (ret != ARRAY_SIZE(msgs))
		return -EIO;

	*val = be32_to_cpu(data_be);

	return 0;
}

/*@ov13850_enable_test_pattern - This enables the test pattern
   Valid valued for pattern are 1 - 4
 Return value - 0 on success, negative on failure*/
static int ov13850_enable_test_pattern(u32 pattern)
{
	u32 val;
	val = (pattern - 1) | 0x80;

	return ov13850_write_reg(ov13850_info.client,
				 OV13850_REG_TEST_PATTERN,
				 OV13850_REG_VALUE_08BIT,
				 val);
}

/*@__ov13850_start_stream - writing mode register settings
 and streaming register
 Return value - 0 on success, negative on failure*/
static int __ov13850_start_stream(void)
{
	int ret = 0;

	pr_debug("%s: %d\n", __func__, __LINE__);

	if(ov13850_info.streaming)
	{
		pr_debug("OV13850: Already streaming \n");
		return ret;
	}

	ret = ov13850_write_array(ov13850_info.client, ov13850_info.cur_mode->reg_list);
	if (ret)
		return ret;
	ret = ov13850_write_reg(ov13850_info.client,
				 OV13850_REG_CTRL_MODE,
				 OV13850_REG_VALUE_08BIT,
				 OV13850_MODE_STREAMING);
	if(!ret)
	{
		pr_debug("%s: streaming started \n", __func__);
		ov13850_info.streaming = 1;
	}
	pr_debug("%s: %d\n", __func__, __LINE__);
	return ret;
}

static int __ov13850_stop_stream(void)
{
	int ret = 0;

	if(!ov13850_info.streaming)
		return ret;

	ret = ov13850_write_reg(ov13850_info.client,
				 OV13850_REG_CTRL_MODE,
				 OV13850_REG_VALUE_08BIT,
				 OV13850_MODE_SW_STANDBY);
	if(!ret)
	{
		pr_debug("%s: streaming stopped \n", __func__);
		ov13850_info.streaming = 0;
	}
	return ret;

}

/* Calculate the delay in us by clock rate and clock cycles */
static inline u32 ov13850_cal_delay(u32 cycles)
{
	return DIV_ROUND_UP(cycles, OV13850_XVCLK_FREQ / 1000 / 1000);
}


/*@__ov13850_power_on - We only control reset gpio as per board
 requirement, power down line is not software controlled*/
static void __ov13850_power_on(void)
{
	int ret;
	u32 delay_us;

	if (!IS_ERR_OR_NULL(ov13850_info.pins_default)) {
		ret = pinctrl_select_state(ov13850_info.pinctrl,
					   ov13850_info.pins_default);
		if (ret < 0)
			pr_err("OV13850: could not set pins\n");
	}
	
	gpio_set_value(reset_gpio, 0);
	usleep_range(3000, 5000);

	gpio_set_value(reset_gpio, 1);
	usleep_range(3000, 5000);

	/* 8192 cycles prior to first SCCB transaction */
	delay_us = ov13850_cal_delay(8192);
	usleep_range(delay_us, delay_us * 2);

}

static void __ov13850_power_off(void)
{
	int ret;

	__ov13850_stop_stream();
	msleep(10);
	gpio_set_value(reset_gpio, 0);
	if (!IS_ERR_OR_NULL(ov13850_info.pins_sleep)) {
		ret = pinctrl_select_state(ov13850_info.pinctrl,
					   ov13850_info.pins_sleep);
		if (ret < 0)
			pr_err("OV13850: could not set pins\n");
	}
}

static int ov13850_check_sensor_id(struct i2c_client *client)
{
	u32 id = 0;
	int ret;

	ret = ov13850_read_reg(client, OV13850_REG_CHIP_ID,
			       OV13850_REG_VALUE_16BIT, &id);
	printk(KERN_ALERT "OV13850, register 0x300A: %d\n", id);
	if (id != CHIP_ID) {
		pr_err("Unexpected sensor id(%06x), ret(%d)\n", id, ret);
		return -ENODEV;
	}

	ret = ov13850_read_reg(client, OV13850_CHIP_REVISION_REG,
			       OV13850_REG_VALUE_08BIT, &id);
	if (ret) {
		pr_err("Read chip revision register error\n");
		return ret;
	}

	/*We check revision above, but current driver only write settings
	 for revision 0xb2 chips*/
	ov13850_global_regs = ov13850_init_2112x1568_r2a;
	ov13850_revid = id;
	pr_info("Detected OV13850: %06x sensor, REVISION 0x%x\n", CHIP_ID, id);

	return 0;
}

/*@ov13850_set_virtual_channel - virtual channel is 1 for
  ipu 0 and csi 1, write 1 to virtual channel register*/
static int ov13850_set_virtual_channel(int channel)
{
	int retval = 0;
	retval = ov13850_write_reg(ov13850_info.client, OV13850_VCHANNEL,
					OV13850_REG_VALUE_08BIT, channel);
	if(retval)
		pr_err("%s: failed\n", __func__);
	return retval;
}

/*ov13850_start_capture - if valid pattern value then enable test
  pattern, else go for normal streaming. Change ENABLE_TEST_PATTERN
  with value 1 to 4 to enable test pattern */
static int ov13850_start_capture(int frame_rate, int pattern)
{
	int retval = 0;	

	if(pattern > 0 && pattern <= 4)
		ov13850_enable_test_pattern(pattern);
	retval = __ov13850_start_stream();
	if (retval) {
		pr_err("OV13850 start stream failed !\n");
		return retval;
	}

	ov13850_set_virtual_channel(ov13850_data.csi);
	return 0;
}

static int ov13850_write_init_settings(const struct regval *settings)
{
	int retval = 0;
	retval = ov13850_write_array(ov13850_info.client, settings);
	if (retval) {
		pr_err("OV13850 global settings failed !\n");
		return retval;
	}
	msleep(5);
	return retval;
}

/*@ov13850_init_mode - This is called during device init. Enable csi here,
  start streaming, check mipi status and stop the streaming.
  Return - 0 on success, negative on failure*/
static int ov13850_init_mode(int frame_rate)
{
	int retval = 0;
	void *mipi_csi2_info;
	u32 mipi_reg;
	int k = 0;

	mipi_csi2_info = mipi_csi2_get_info();

	/* initial mipi dphy */
	if (!mipi_csi2_info) {
		printk(KERN_ERR "%s() in %s: Fail to get mipi_csi2_info!\n",
		       __func__, __FILE__);
		return -1;
	}

	if (!mipi_csi2_get_status(mipi_csi2_info))
		mipi_csi2_enable(mipi_csi2_info);

	if (!mipi_csi2_get_status(mipi_csi2_info)) {
		pr_err("Can not enable mipi csi2 driver!\n");
		return -1;
	}

	mipi_csi2_set_lanes(mipi_csi2_info);

	mipi_csi2_reset(mipi_csi2_info);

	if (ov13850_data.pix.pixelformat == V4L2_PIX_FMT_SBGGR8)
		mipi_csi2_set_datatype(mipi_csi2_info, MIPI_DT_RAW8);
	else
		pr_err("currently this sensor format can not be supported!\n");

	retval = ov13850_write_init_settings(ov13850_global_regs);
	if(retval)
		return retval;

	while(k++<10)
	{
		mipi_reg = mipi_csi2_dphy_status(mipi_csi2_info);
		pr_debug(KERN_INFO "mipi_csi2_dphy_status: %u\n", mipi_reg);
		msleep(2);
	}

	retval = __ov13850_start_stream();
	if (retval) {
		pr_err("OV13850 start stream failed !\n");
		return retval;
	}

	retval = ov13850_set_virtual_channel(ov13850_data.csi);
	if(retval)
		goto err1;

	if (mipi_csi2_info) {
		unsigned int i;

		i = 0;

		/* wait for mipi sensor ready */
		mipi_reg = mipi_csi2_dphy_status(mipi_csi2_info);
		while ((mipi_reg == 0x200) && i < 10) {
			/*if(mipi_reg == 0x300 || mipi_reg == 0x330)
				break;
			if(mipi_reg == 0x200) break;*/
			mipi_reg = mipi_csi2_dphy_status(mipi_csi2_info);
			i++;
			msleep(10);
		}

		if (i >= 10) {
			pr_err("mipi csi2 can not receive sensor clk!\n");
			retval = -1;
			goto err1;
		}

		i = 0;

		/* wait for mipi stable */
		mipi_reg = mipi_csi2_get_error1(mipi_csi2_info);
		while ((mipi_reg != 0x0) && (i < 10)) {
			mipi_reg = mipi_csi2_get_error1(mipi_csi2_info);
			i++;
			msleep(10);
		}

		if (i >= 10) {
			pr_err("mipi csi2 can not reveive data correctly!\n");
			retval = -1;
			goto err1;
		}
	
		k = 0;
		while(k++<10)
		{	
			mipi_reg = mipi_csi2_get_error2(mipi_csi2_info);
			pr_debug(KERN_INFO "mipi_csi2_get_error2: %u\n", mipi_reg);
			msleep(2);
		}

		k = 0;
		while(k++<10)
		{
			mipi_reg = mipi_csi2_dphy_status(mipi_csi2_info);
			pr_debug("mipi_csi2_dphy_status: %u\n", mipi_reg);
			if(mipi_reg == 0x300)
				break;
			msleep(2);
		}
	}
err1:
	__ov13850_stop_stream();
	return retval;
}

/*@ioctl_enum_framesizes - Return the two supported frame sizes here*/
static int ioctl_enum_framesizes(struct v4l2_int_device *s,
				 struct v4l2_frmsizeenum *fsize)
{
	if (fsize->index > 2)
                return -EINVAL;
	fsize->pixel_format = ov13850_data.pix.pixelformat;
	fsize->discrete.width = supported_modes[fsize->index].width;
	fsize->discrete.height = supported_modes[fsize->index].height;
	return 0;
}

/*@ioctl_g_fmt_cap - Return v4l2_pix_format structure here*/
static int ioctl_g_fmt_cap(struct v4l2_int_device *s, struct v4l2_format *f)
{
	struct sensor_data *sensor = s->priv;
	pr_debug("%s: %d\n", __func__, __LINE__);
	f->fmt.pix = sensor->pix;
	return 0;
}

static int ioctl_enum_fmt_cap(struct v4l2_int_device *s,
			      struct v4l2_fmtdesc *fmt)
{
	if (fmt->index > 0)
                return -EINVAL;
	pr_debug("%s: %d\n", __func__, __LINE__);
	fmt->pixelformat = ov13850_data.pix.pixelformat;

	return 0;
}

static int ioctl_s_power(struct v4l2_int_device *s, int on)
{
	printk(KERN_INFO "%s: %d\n", __func__, __LINE__);
	if (on) {
		__ov13850_power_on();
	} else if (!on) {
		__ov13850_power_off();
	}
	printk(KERN_INFO "%s: %d\n", __func__, __LINE__);

	return 0;
}

static int ioctl_init(struct v4l2_int_device *s)
{
	return 0;
}

static int ioctl_dev_exit(struct v4l2_int_device *s)
{
	void *mipi_csi2_info;

	printk(KERN_INFO "%s: %d\n", __func__, __LINE__);
	mipi_csi2_info = mipi_csi2_get_info();

	/* disable mipi csi2 */
	if (mipi_csi2_info)
		if (mipi_csi2_get_status(mipi_csi2_info))
			mipi_csi2_disable(mipi_csi2_info);

	return 0;
}

static int ioctl_g_ctrl(struct v4l2_int_device *s, struct v4l2_control *vc)
{
	int ret = 0;

	switch (vc->id) {
	/*Need to implement control here further*/
	default:
		ret = -EINVAL;
	}

	return ret;
}

static int ioctl_enum_frameintervals(struct v4l2_int_device *s,
					 struct v4l2_frmivalenum *fival)
{
	if(fival->index > 2)
		return -EINVAL;
	fival->type = V4L2_FRMIVAL_TYPE_DISCRETE;
	fival->discrete.numerator =
			supported_modes[fival->index].max_fps.numerator;
	fival->discrete.denominator =
			supported_modes[fival->index].max_fps.denominator;
	return 0;
}

static int ioctl_s_ctrl(struct v4l2_int_device *s, struct v4l2_control *vc)
{
	int ret = 0;

	pr_debug("In ov13850: ioctl_s_ctrl %d\n",
		 vc->id);

	switch (vc->id) {
	case V4L2_CID_EXPOSURE:
		ret = ov13850_write_reg(ov13850_info.client,
					OV13850_REG_EXPOSURE,
					OV13850_REG_VALUE_24BIT,
					vc->value << 4);
		break;
	case V4L2_CID_ANALOGUE_GAIN:
		ret = ov13850_write_reg(ov13850_info.client,
					OV13850_REG_GAIN_H,
					OV13850_REG_VALUE_08BIT,
					(vc->value >> OV13850_GAIN_H_SHIFT) &
					OV13850_GAIN_H_MASK);
		ret |= ov13850_write_reg(ov13850_info.client,
					 OV13850_REG_GAIN_L,
					 OV13850_REG_VALUE_08BIT,
					 vc->value & OV13850_GAIN_L_MASK);
		break;
	case V4L2_CID_VBLANK:
		ret = ov13850_write_reg(ov13850_info.client,
					OV13850_REG_VTS,
					OV13850_REG_VALUE_16BIT,
					vc->value + ov13850_info.cur_mode->height);
		break;
	default:
		ret = -EPERM;
		break;
	}

	return ret;
}

static int ioctl_g_chip_ident(struct v4l2_int_device *s, int *id)
{
	((struct v4l2_dbg_chip_ident *)id)->match.type =
					V4L2_CHIP_MATCH_I2C_DRIVER;
	strcpy(((struct v4l2_dbg_chip_ident *)id)->match.name,
		OV13850_NAME);
	pr_debug("%s: %d\n", __func__, __LINE__);

	return 0;
}

static int ioctl_g_parm(struct v4l2_int_device *s, struct v4l2_streamparm *a)
{
	struct sensor_data *sensor = s->priv;
	struct v4l2_captureparm *cparm = &a->parm.capture;
	int ret = 0;

	pr_debug("%s: %d\n", __func__, __LINE__);
	switch (a->type) {
	/*Need to verify below settings further*/
	case V4L2_BUF_TYPE_VIDEO_CAPTURE:
		pr_debug("%s: %d\n", __func__, __LINE__);
		memset(a, 0, sizeof(*a));
		a->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		cparm->capability = sensor->streamcap.capability;
		cparm->timeperframe = sensor->streamcap.timeperframe;
		cparm->capturemode = sensor->streamcap.capturemode;
		ret = 0;
		break;
	/*These cases not applicable now*/
	case V4L2_BUF_TYPE_VIDEO_OUTPUT:
	case V4L2_BUF_TYPE_VIDEO_OVERLAY:
	case V4L2_BUF_TYPE_VBI_CAPTURE:
	case V4L2_BUF_TYPE_VBI_OUTPUT:
	case V4L2_BUF_TYPE_SLICED_VBI_CAPTURE:
	case V4L2_BUF_TYPE_SLICED_VBI_OUTPUT:
		ret = -EINVAL;
		break;

	default:
		pr_debug("   type is unknown - %d\n", a->type);
		ret = -EINVAL;
		break;
	}

	return ret;
}

static int ioctl_s_parm(struct v4l2_int_device *s, struct v4l2_streamparm *a)
{
	int ret = 0;
	struct sensor_data *sensor = s->priv;
	struct v4l2_fract *timeperframe = &a->parm.capture.timeperframe;

	switch (a->type) {
	/* This is the only case currently handled. */
	case V4L2_BUF_TYPE_VIDEO_CAPTURE:
	/*Need to set fps as per request */
		pr_debug("%s: %d\n", __func__, __LINE__);
		sensor->streamcap.timeperframe = *timeperframe;
		sensor->streamcap.capturemode =
				(u32)a->parm.capture.capturemode;
		break;
	/* These cases not applicable for now*/
	case V4L2_BUF_TYPE_VIDEO_OUTPUT:
	case V4L2_BUF_TYPE_VIDEO_OVERLAY:
	case V4L2_BUF_TYPE_VBI_CAPTURE:
	case V4L2_BUF_TYPE_VBI_OUTPUT:
	case V4L2_BUF_TYPE_SLICED_VBI_CAPTURE:
	case V4L2_BUF_TYPE_SLICED_VBI_OUTPUT:
		pr_debug("   type is not " \
			"V4L2_BUF_TYPE_VIDEO_CAPTURE but %d\n",
			a->type);
		ret = -EINVAL;
		break;

	default:
		pr_debug("   type is unknown - %d\n", a->type);
		ret = -EINVAL;
		break;
	}

	return ret;
}

static int ioctl_g_ifparm(struct v4l2_int_device *s, struct v4l2_ifparm *p)
{
	if (s == NULL) {
		pr_err("   ERROR!! no slave device set!\n");
		return -1;
	}

	memset(p, 0, sizeof(*p));
	pr_debug("   clock_curr=mclk=%d\n", ov13850_data.mclk);

	return 0;
}

static int ioctl_dev_init(struct v4l2_int_device *s)
{
	int ret;
	void *mipi_csi2_info;
	int frame_rate;

	printk(KERN_INFO "ov13850, %s: %d\n", __func__, __LINE__);

        ov13850_data.on = true;
	ov13850_data.mclk = OV13850_XVCLK_FREQ;
	pr_debug("   Setting mclk to %d MHz\n", OV13850_XVCLK_FREQ);

	frame_rate = DEFAULT_FPS;
	mipi_csi2_info = mipi_csi2_get_info();

        /* enable mipi csi2 */
	if (mipi_csi2_info)
		mipi_csi2_enable(mipi_csi2_info);
	else {
		printk(KERN_ERR "%s() in %s: Fail to get mipi_csi2_info!\n",
		       __func__, __FILE__);
		return -EPERM;
	}
	printk(KERN_INFO "ov13850, %s: %d\n", __func__, __LINE__);

	ret = ov13850_init_mode(frame_rate);
	return ret;
}

/*@ioctl_s_streamon - This is called from mxc capture driver
   Return - 0 on success, negative on failure*/
static void ioctl_s_streamon(struct v4l2_int_device *s, int *err)
{
	*err = -EINVAL;
	if(strcmp(s->name, OV13850_NAME) == 0)
		*err = ov13850_start_capture(DEFAULT_FPS, ENABLE_TEST_PATTERN);
	/*FPS is not handled for now, to handle FPS, need to change
	HTS and VTS register as per request*/
	pr_debug("%s: %s - %d\n", __func__, s->name, *err);
}

/*@ioctl_s_fmt_cap - Switch init register settings and supported mode as
 per received width and height*/
static int ioctl_s_fmt_cap(struct v4l2_int_device *s, struct v4l2_format *f)
{
	int retval = 0;

	if(f->fmt.pix.width == OV13850_RES_HIGH_WIDTH &&
		f->fmt.pix.height == OV13850_RES_HIGH_HEIGHT)
	{
		ov13850_info.cur_mode = &supported_modes[1];
		ov13850_global_regs = ov13850_init_4224x3136_r2a;
	}
	else if(f->fmt.pix.width == OV13850_RES_LOW_WIDTH &&
		f->fmt.pix.height == OV13850_RES_LOW_HEIGHT)
	{
		ov13850_info.cur_mode = &supported_modes[0];
		ov13850_global_regs = ov13850_init_2112x1568_r2a;
	}
	else
	{
		printk(KERN_INFO "OV13850: Unsupported resolution passed\n");
		return -1;
	}

	retval = ov13850_write_init_settings(ov13850_global_regs);
	return retval; 
}

static struct v4l2_int_ioctl_desc ov13850_ioctl_desc[] = {
	{vidioc_int_dev_init_num, (v4l2_int_ioctl_func *) ioctl_dev_init},
	{vidioc_int_dev_exit_num, ioctl_dev_exit},
	{vidioc_int_s_power_num, (v4l2_int_ioctl_func *) ioctl_s_power},
	{vidioc_int_g_ifparm_num, (v4l2_int_ioctl_func *) ioctl_g_ifparm},
	{vidioc_int_init_num, (v4l2_int_ioctl_func *) ioctl_init},
	{vidioc_int_enum_fmt_cap_num,
				(v4l2_int_ioctl_func *) ioctl_enum_fmt_cap},
	{vidioc_int_g_fmt_cap_num, (v4l2_int_ioctl_func *) ioctl_g_fmt_cap},
	{vidioc_int_s_fmt_cap_num, (v4l2_int_ioctl_func *) ioctl_s_fmt_cap},
	{vidioc_int_g_parm_num, (v4l2_int_ioctl_func *) ioctl_g_parm},
	{vidioc_int_s_parm_num, (v4l2_int_ioctl_func *) ioctl_s_parm},
	{vidioc_int_g_ctrl_num, (v4l2_int_ioctl_func *) ioctl_g_ctrl},
	{vidioc_int_s_ctrl_num, (v4l2_int_ioctl_func *) ioctl_s_ctrl},
	{vidioc_int_enum_framesizes_num,
				(v4l2_int_ioctl_func *) ioctl_enum_framesizes},
	{vidioc_int_enum_frameintervals_num,
			(v4l2_int_ioctl_func *) ioctl_enum_frameintervals},
	{vidioc_int_g_chip_ident_num,
				(v4l2_int_ioctl_func *) ioctl_g_chip_ident},
	{vidioc_int_s_streamon_num, (v4l2_int_ioctl_func *) ioctl_s_streamon},
};

static struct v4l2_int_slave ov13850_slave = {
	.ioctls = ov13850_ioctl_desc,
	.num_ioctls = ARRAY_SIZE(ov13850_ioctl_desc),
};

static struct v4l2_int_device ov13850_int_device = {
	.module = THIS_MODULE,
	.name = OV13850_NAME,
	.type = v4l2_int_type_slave,
	.u = {
		.slave = &ov13850_slave,
	},
};

static int ov13850_probe(struct i2c_client *client,
			 const struct i2c_device_id *id)
{
	struct device *dev = &client->dev;
	int ret;

	pr_info("OV13850: ov13850_probe called\n");
	ov13850_info.client = client;
	ov13850_info.cur_mode = &supported_modes[0];
	ov13850_info.streaming = 0;

	reset_gpio = of_get_named_gpio(dev->of_node, "rst-gpios", 0);
	if (!gpio_is_valid(reset_gpio)) {
		dev_err(dev, "no sensor reset pin available");
		return -EINVAL;
	}
	ret = devm_gpio_request_one(dev, reset_gpio, GPIOF_OUT_INIT_LOW,
					"ov13850_reset");
	if (ret < 0) {
		dev_err(dev, "failed to acquire sensor reset pin");
		return ret;
	}

	memset(&ov13850_data, 0, sizeof(ov13850_data));
	ov13850_data.sensor_clk = devm_clk_get(dev, "csi_mclk");
	if (IS_ERR(ov13850_data.sensor_clk)) {
                ov13850_data.sensor_clk = NULL;
                dev_err(dev, "clock-frequency missing or invalid\n");
                return PTR_ERR(ov13850_data.sensor_clk);
        }

	ov13850_info.pinctrl = devm_pinctrl_get(dev);
	if (!IS_ERR(ov13850_info.pinctrl)) {
		ov13850_info.pins_default =
			pinctrl_lookup_state(ov13850_info.pinctrl,
					     OF_CAMERA_PINCTRL_STATE_DEFAULT);
		if (IS_ERR(ov13850_info.pins_default))
			dev_err(dev, "could not get default pinstate\n");

		ov13850_info.pins_sleep =
			pinctrl_lookup_state(ov13850_info.pinctrl,
					     OF_CAMERA_PINCTRL_STATE_SLEEP);
		if (IS_ERR(ov13850_info.pins_sleep))
			dev_err(dev, "could not get sleep pinstate\n");
	}

	mutex_init(&ov13850_info.mutex);
	
	ret = of_property_read_u32(dev->of_node, "mclk",
					&(ov13850_data.mclk));
	if (ret) {
		dev_err(dev, "mclk missing or invalid\n");
		return ret;
	}

	ret = of_property_read_u32(dev->of_node, "mclk_source",
					(u32 *) &(ov13850_data.mclk_source));
	if (ret) {
		dev_err(dev, "mclk_source missing or invalid\n");
		return ret;
	}

	ret = of_property_read_u32(dev->of_node, "csi_id",
					&(ov13850_data.csi));
	if (ret) {
		dev_err(dev, "csi id missing or invalid\n");
		return ret;
	}

	ret = clk_prepare_enable(ov13850_data.sensor_clk);
	if (ret < 0) {
		dev_err(dev, "Failed to enable xvclk\n");
		return ret;
	}

	ov13850_data.io_init = __ov13850_power_on;
	ov13850_data.i2c_client = client;
	/*Only cab handle 8bit mode for now*/
	ov13850_data.pix.pixelformat = V4L2_PIX_FMT_SBGGR8;
	ov13850_data.pix.width = OV13850_RES_LOW_WIDTH;
        ov13850_data.pix.height = OV13850_RES_LOW_HEIGHT;
	ov13850_data.streamcap.capability = V4L2_MODE_HIGHQUALITY |
                                           V4L2_CAP_TIMEPERFRAME;
        ov13850_data.streamcap.capturemode = 0;
        ov13850_data.streamcap.timeperframe.denominator = DEFAULT_FPS;
        ov13850_data.streamcap.timeperframe.numerator = 1;

	__ov13850_power_on();

	ret = ov13850_check_sensor_id(client);
	if (ret)
	{
		dev_err(dev, "ov13850 mipi not found\n");
		clk_disable_unprepare(ov13850_data.sensor_clk);
		goto err_free_handler;
	}

	ov13850_int_device.priv = &ov13850_data;
        ret = v4l2_int_device_register(&ov13850_int_device);
        clk_disable_unprepare(ov13850_data.sensor_clk);
	__ov13850_power_off();

	pr_info("OV13850: ov13850_probe successful\n");

	return 0;

err_free_handler:
	mutex_destroy(&ov13850_info.mutex);

	return ret;
}

static int ov13850_remove(struct i2c_client *client)
{
	v4l2_int_device_unregister(&ov13850_int_device);
	mutex_destroy(&ov13850_info.mutex);
	return 0;
}


#if IS_ENABLED(CONFIG_OF)
static const struct of_device_id ov13850_of_match[] = {
        { .compatible = "ovti,ov13850" },
        {},
};
MODULE_DEVICE_TABLE(of, ov13850_of_match);
#endif

static const struct i2c_device_id ov13850_match_id[] = {
        { "ovti,ov13850", 0 },
        { },
};

static struct i2c_driver ov13850_i2c_driver = {
        .driver = {
                .name = OV13850_NAME,
                .of_match_table = of_match_ptr(ov13850_of_match),
        },
        .probe          = &ov13850_probe,
        .remove         = &ov13850_remove,
        .id_table       = ov13850_match_id,
};

static int __init sensor_mod_init(void)
{
	return i2c_add_driver(&ov13850_i2c_driver);
}

static void __exit sensor_mod_exit(void)
{
	i2c_del_driver(&ov13850_i2c_driver);
}

device_initcall_sync(sensor_mod_init);
module_exit(sensor_mod_exit);

MODULE_AUTHOR("Arun kumar");
MODULE_DESCRIPTION("ov13850 sensor driver");
MODULE_LICENSE("GPL v2");
