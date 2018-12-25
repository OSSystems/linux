/* drivers/input/misc/cm36771.c - cm36771 optical sensors driver
 *
 * Copyright (C) 2014 Capella Microsystems Inc.
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

#include <linux/delay.h>
#include <linux/version.h>

#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif

#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/workqueue.h>
#include <linux/irq.h>
#include <linux/errno.h>
#include <linux/err.h>
#include <linux/gpio.h>
#include <linux/miscdevice.h>
#include <linux/lightsensor.h>
#include <linux/slab.h>
#include <asm/uaccess.h>
#include <asm/mach-types.h>
#include <linux/cm36771.h>
#include <linux/capella_cm3602.h>
#include <asm/setup.h>

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,1,0)
#include <linux/of_gpio.h>
#endif

#ifdef CONFIG_HAS_WAKELOCK
#include <linux/wakelock.h>
#endif

#include <linux/jiffies.h>
#include <linux/math64.h>

#define D(x...) pr_debug(x)

#define I2C_RETRY_COUNT 10

#define NEAR_DELAY_TIME ((100 * HZ) / 1000)

#define CONTROL_INT_ISR_REPORT        0x00
#define CONTROL_ALS                   0x01
#define CONTROL_PS                    0x02

#define CALIBRATION_FILE_PATH	"/efs/cal_data"
#define CHANGE_SENSITIVITY 5 // in percent

static void sensor_irq_do_work(struct work_struct *work);
static DECLARE_WORK(sensor_irq_work, sensor_irq_do_work);

struct cm36771_info {
	struct class *cm36771_class;
	struct device *ls_dev;
	struct device *ps_dev;

	struct input_dev *ls_input_dev;
	struct input_dev *ps_input_dev;

#ifdef CONFIG_HAS_EARLYSUSPEND
	struct early_suspend early_suspend;
#endif
	struct i2c_client *i2c_client;
	struct workqueue_struct *lp_wq;

	int intr_pin;

	int als_enable;
	int ps_enable;
	int ps_irq_flag;

	int irq;

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,1,0)
	int pwr_pin;
#else	
	int (*power)(int, uint8_t); /* power to the chip */
#endif

	uint32_t als_resolution; // represented using a fixed 10(-5) notation
	uint32_t cal_data; // represented using a fixed 10(-5) notation

#ifdef CONFIG_HAS_WAKELOCK
	struct wake_lock ps_wake_lock;
#endif
	int psensor_opened;
	int lightsensor_opened;
	uint8_t slave_addr;

	uint8_t ps_close_thd_set;
	uint8_t ps_away_thd_set;

	uint32_t current_lux;
	uint16_t current_adc;

	uint16_t ps_conf1_val;
	uint16_t ps_conf3_val;

	uint16_t ls_cmd;
};
struct cm36771_info *lp_info;
bool cal_data_retrieved = false;
static struct mutex als_enable_mutex, als_disable_mutex;
static struct mutex ps_enable_mutex, ps_disable_mutex;
static struct mutex CM36771_control_mutex;
static int lightsensor_enable(struct cm36771_info *lpi);
static int lightsensor_disable(struct cm36771_info *lpi);
static void psensor_initial_cmd(struct cm36771_info *lpi);

static int control_and_report(struct cm36771_info *lpi, uint8_t mode, uint16_t param);

static int I2C_RxData(uint16_t slaveAddr, uint8_t cmd, uint8_t *rxData, int length)
{
	uint8_t loop_i;
	int val;
	struct cm36771_info *lpi = lp_info;
	uint8_t subaddr[1];

	struct i2c_msg msgs[] =
	{
		{
			.addr = slaveAddr,
			.flags = 0,
			.len = 1,
			.buf = subaddr,
		},
		{
			.addr = slaveAddr,
			.flags = I2C_M_RD,
			.len = length,
			.buf = rxData,
		},		 
	};
	
	subaddr[0] = cmd;	

	for (loop_i = 0; loop_i < I2C_RETRY_COUNT; loop_i++)
	{
		if (i2c_transfer(lp_info->i2c_client->adapter, msgs, 2) > 0)
			break;

		val = gpio_get_value(lpi->intr_pin);
		/*check intr GPIO when i2c error*/
		if (loop_i == 0 || loop_i == I2C_RETRY_COUNT -1)
			D("[PS][CM36771 error] %s, i2c err, slaveAddr 0x%x ISR gpio %d  = %d\n",
				__func__, slaveAddr, lpi->intr_pin, val);

		msleep(10);
	}
	if (loop_i >= I2C_RETRY_COUNT)
	{
		pr_err("[PS_ERR][CM36771 error] %s retry over %d\n",
			__func__, I2C_RETRY_COUNT);
		return -EIO;
	}

	return 0;
}

static int I2C_TxData(uint16_t slaveAddr, uint8_t *txData, int length)
{
	uint8_t loop_i;
	int val;
	struct cm36771_info *lpi = lp_info;
	struct i2c_msg msg[] =
	{
		{
			.addr = slaveAddr,
			.flags = 0,
			.len = length,
			.buf = txData,
		},
	};

	for (loop_i = 0; loop_i < I2C_RETRY_COUNT; loop_i++)
	{
		if (i2c_transfer(lp_info->i2c_client->adapter, msg, 1) > 0)
			break;

		val = gpio_get_value(lpi->intr_pin);
		/*check intr GPIO when i2c error*/
		if (loop_i == 0 || loop_i == I2C_RETRY_COUNT -1)
			D("[PS][CM36771 error] %s, i2c err, slaveAddr 0x%x, value 0x%x, ISR gpio%d  = %d\n",
				__func__, slaveAddr, txData[0], lpi->intr_pin, val);

		msleep(10);
	}

	if (loop_i >= I2C_RETRY_COUNT)
	{
		pr_err("[PS_ERR][CM36771 error] %s retry over %d\n",
			__func__, I2C_RETRY_COUNT);
		return -EIO;
	}

	return 0;
}

static int cm36771_i2c_read_word(uint16_t slaveAddr, uint8_t cmd, uint16_t *pdata)
{
	uint8_t buffer[2];
	int ret = 0;

	if (pdata == NULL)
		return -EFAULT;

	ret = I2C_RxData(slaveAddr, cmd, buffer, 2);
	if (ret < 0)
	{
		pr_err("[PS_ERR][CM36771 error]%s: I2C_RxData fail [0x%x, 0x%x]\n",
			__func__, slaveAddr, cmd);
		return ret;
	}

	*pdata = (buffer[1] << 8) | buffer[0];
#if 0
	/* Debug use */
	pr_dbg("[CM36771] %s: I2C_RxData[0x%x, 0x%x] = 0x%x\n",
		__func__, slaveAddr, cmd, *pdata);
#endif
	return ret;
}

static int cm36771_i2c_write_word(uint16_t SlaveAddress, uint8_t cmd, uint16_t data)
{
	char buffer[3];
	int ret = 0;
#if 0
	/* Debug use */
	pr_dbg("[CM36771] %s: cm36771_i2c_write_word[0x%x, 0x%x, 0x%x]\n",
		__func__, SlaveAddress, cmd, data);
#endif
	buffer[0] = cmd;
	buffer[1] = (uint8_t)(data & 0xff);
	buffer[2] = (uint8_t)((data & 0xff00) >> 8);	
	
	ret = I2C_TxData(SlaveAddress, buffer, 3);
	if (ret < 0)
	{
		pr_err("[PS_ERR][CM36771 error]%s: I2C_TxData fail\n", __func__);
		return -EIO;
	}

	return ret;
}

static int get_ls_adc_value(uint16_t *als_step, bool resume)
{
	struct cm36771_info *lpi = lp_info;
	int ret = 0;

	if (als_step == NULL)
		return -EFAULT;

	/* Read ALS data: */
	ret = cm36771_i2c_read_word(lpi->slave_addr, ALS_DATA, als_step);
	if (ret < 0)
	{
		pr_err(
			"[LS][CM36771 error]%s: cm36771_i2c_read_word fail\n",
			__func__);
		return -EIO;
	}

	D("[LS][CM36771] %s: raw adc = 0x%X\n",	__func__, *als_step);

	return ret;
}

static int set_lsensor_range(uint16_t low_thd, uint16_t high_thd)
{
	int ret = 0;
	struct cm36771_info *lpi = lp_info;

	cm36771_i2c_write_word(lpi->slave_addr, ALS_THDH, high_thd);
	cm36771_i2c_write_word(lpi->slave_addr, ALS_THDL, low_thd);

	return ret;
}

static int get_ps_adc_value(uint16_t *data)
{
	int ret = 0;
	struct cm36771_info *lpi = lp_info;

	if (data == NULL)
		return -EFAULT;	

	ret = cm36771_i2c_read_word(lpi->slave_addr, PS_DATA, data);
	
	(*data) &= 0xFF;
	
	if (ret < 0) {
		pr_err("[PS][CM36771 error]%s: cm36771_i2c_read_word fail\n",
			__func__);
		(*data) = 0xFFFF;
		return -EIO;
	}
	else {
		D("[PS][CM36771 OK]%s: cm36771_i2c_read_word OK 0x%x\n",
			__func__, *data);
	}

	return ret;
}

static uint16_t mid_value(uint16_t value[], uint8_t size)
{
	int i = 0, j = 0;
	uint16_t temp = 0;

	if (size < 3)
		return 0;

	for (i = 0; i < (size - 1); i++)
	{
		for (j = (i + 1); j < size; j++)
		{
			if (value[i] > value[j])
			{
				temp = value[i];
				value[i] = value[j];
				value[j] = temp;
			}
		}
	}
	
	return value[((size - 1) / 2)];
}

static int get_stable_ps_adc_value(uint16_t *ps_adc)
{
	uint16_t value[3] = {0, 0, 0}, mid_val = 0;
	int ret = 0;
	int i = 0;
	int wait_count = 0;
	struct cm36771_info *lpi = lp_info;

	for (i = 0; i < 3; i++)
	{
		/*wait interrupt GPIO high*/
		while (gpio_get_value(lpi->intr_pin) == 0)
		{
			msleep(10);
			wait_count++;
			if (wait_count > 12)
			{
				pr_err("[PS_ERR][CM36771 error]%s: interrupt GPIO low,"
					" get_ps_adc_value\n", __func__);
				return -EIO;
			}
		}

		ret = get_ps_adc_value(&value[i]);
		if (ret < 0)
		{
			pr_err("[PS_ERR][CM36771 error]%s: get_ps_adc_value\n",
				__func__);
			return -EIO;
		}

		if (wait_count < 60/10) /* wait gpio less than 60ms */
		{
			msleep(60 - (10*wait_count));
		}
		wait_count = 0;
	}

	/*D("Sta_ps: Before sort, value[0, 1, 2] = [0x%x, 0x%x, 0x%x]",
		value[0], value[1], value[2]);*/
	mid_val = mid_value(value, 3);
	D("Sta_ps: After sort, value[0, 1, 2] = [0x%x, 0x%x, 0x%x]",
		value[0], value[1], value[2]);
	*ps_adc = (mid_val & 0xFF);

	return 0;
}

static void sensor_irq_do_work(struct work_struct *work)
{
	struct cm36771_info *lpi = lp_info;
	uint16_t intFlag;
	cm36771_i2c_read_word(lpi->slave_addr, INT_FLAG, &intFlag);
	control_and_report(lpi, CONTROL_INT_ISR_REPORT, intFlag);  
	  
	enable_irq(lpi->irq);
}

static irqreturn_t cm36771_irq_handler(int irq, void *data)
{
	struct cm36771_info *lpi = data;

	disable_irq_nosync(lpi->irq);
	queue_work(lpi->lp_wq, &sensor_irq_work);

	return IRQ_HANDLED;
}

static int als_power(int enable)
{
	struct cm36771_info *lpi = lp_info;

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,1,0)
	if(gpio_is_valid(lpi->pwr_pin))
		gpio_set_value(lpi->pwr_pin, ((enable)? 1: 0));
#else
	if (lpi->power)
		lpi->power(LS_PWR_ON, 1);
#endif

	return 0;
}

static int lightsensor_get_cal_data(struct cm36771_info *lpi)
{
	struct file *cal_filp = NULL;
	int err = 0;
	mm_segment_t old_fs;

	old_fs = get_fs();
	set_fs(KERNEL_DS);

	cal_filp = filp_open(CALIBRATION_FILE_PATH, O_RDONLY, 0666);
	if (IS_ERR(cal_filp))
	{
		err = PTR_ERR(cal_filp);
		if (err != -ENOENT)
			pr_err("[LS][CM36771] %s: Can't open calibration data file\n", __func__);
		set_fs(old_fs);
		return err;
	}

	err = cal_filp->f_op->read(cal_filp,
		(char *)&lpi->cal_data, sizeof(uint32_t), &cal_filp->f_pos);
	if (err != sizeof(uint32_t))
	{
		pr_err("[LS][CM36771] %s: Can't read the calibration data from file\n", __func__);
		err = -EIO;
	}

	pr_debug("[LS][CM36771] %s: cal_data = %d\n",
		__func__, lpi->cal_data);

	filp_close(cal_filp, current->files);
	set_fs(old_fs);

	return err;
}

static void ls_initial_cmd(struct cm36771_info *lpi)
{	
	/* must disable l-sensor interrupt before IST create *//* disable ALS func */
	lpi->ls_cmd &= ~CM36771_CMD00_ALS_INT_EN;
	lpi->ls_cmd |= CM36771_CMD00_ALS_SD;
	cm36771_i2c_write_word(lpi->slave_addr, ALS_CONF, lpi->ls_cmd);  
}

static void psensor_initial_cmd(struct cm36771_info *lpi)
{
	/* must disable p-sensor interrupt before IST create *//* disable ALS func */		
	lpi->ps_conf1_val |= CM36771_CMD03_PS_SD;
	lpi->ps_conf1_val &= ~CM36771_CMD03_PS_INT_EN;  
	cm36771_i2c_write_word(lpi->slave_addr, PS_CONF1, lpi->ps_conf1_val);   
	cm36771_i2c_write_word(lpi->slave_addr, PS_CONF3, lpi->ps_conf3_val);
	cm36771_i2c_write_word(lpi->slave_addr, PS_THD, (lpi->ps_close_thd_set << 8) | lpi->ps_away_thd_set);

	D("[PS][CM36771] %s, finish\n", __func__);	
}

static int psensor_enable(struct cm36771_info *lpi)
{
	int ret = -EIO;
	
	mutex_lock(&ps_enable_mutex);
	D("[PS][CM36771] %s\n", __func__);

	if (lpi->ps_enable)
	{
		D("[PS][CM36771] %s: already enabled\n", __func__);
		ret = 0;
	}
	else
	{
		ret = control_and_report(lpi, CONTROL_PS, 1);
	}
	
	mutex_unlock(&ps_enable_mutex);
	return ret;
}

static int psensor_disable(struct cm36771_info *lpi)
{
	int ret = -EIO;
	
	mutex_lock(&ps_disable_mutex);
	D("[PS][CM36771] %s\n", __func__);

	if (lpi->ps_enable == 0)
	{
		D("[PS][CM36771] %s: already disabled\n", __func__);
		ret = 0;
	}
	else
	{
		ret = control_and_report(lpi, CONTROL_PS, 0);
	}
	
	mutex_unlock(&ps_disable_mutex);
	return ret;
}

static int psensor_open(struct inode *inode, struct file *file)
{
	struct cm36771_info *lpi = lp_info;

	D("[PS][CM36771] %s\n", __func__);

	if (lpi->psensor_opened)
		return -EBUSY;

	lpi->psensor_opened = 1;

	return 0;
}

static int psensor_release(struct inode *inode, struct file *file)
{
	struct cm36771_info *lpi = lp_info;

	D("[PS][CM36771] %s\n", __func__);

	lpi->psensor_opened = 0;

	return psensor_disable(lpi);
	//return 0;
}

static long psensor_ioctl(struct file *file, unsigned int cmd,
			unsigned long arg)
{
	int val;
	struct cm36771_info *lpi = lp_info;

	D("[PS][CM36771] %s cmd %d\n", __func__, _IOC_NR(cmd));

	switch (cmd)
	{
		case CAPELLA_CM3602_IOCTL_ENABLE:
			if (get_user(val, (unsigned long __user *)arg))
				return -EFAULT;
			if (val)
				return psensor_enable(lpi);
			else
				return psensor_disable(lpi);
			break;
		case CAPELLA_CM3602_IOCTL_GET_ENABLED:
			return put_user(lpi->ps_enable, (unsigned long __user *)arg);
			break;
		default:
			pr_err("[PS][CM36771 error]%s: invalid cmd %d\n",
				__func__, _IOC_NR(cmd));
			return -EINVAL;
	}
}

static const struct file_operations psensor_fops = {
	.owner = THIS_MODULE,
	.open = psensor_open,
	.release = psensor_release,
	.unlocked_ioctl = psensor_ioctl
};

struct miscdevice psensor_misc = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "proximity",
	.fops = &psensor_fops
};

static int lightsensor_enable(struct cm36771_info *lpi)
{
	int ret = -EIO;
	
	mutex_lock(&als_enable_mutex);
	D("[LS][CM36771] %s\n", __func__);

	if (lpi->als_enable)
	{
		D("[LS][CM36771] %s: already enabled\n", __func__);
		ret = 0;
	}
	else
	{
		if (!cal_data_retrieved)
		{
			/* get calibration data */		
			ret = lightsensor_get_cal_data(lpi);
			if (ret < 0 && ret != -ENOENT)
			{
				pr_err("[LS][CM36771] %s: lightsensor_get_cal_data() failed\n",
					__func__);
			}
			else
			{
				cal_data_retrieved = true;
			}
		}	
	
		ret = control_and_report(lpi, CONTROL_ALS, 1);
	}
	
	mutex_unlock(&als_enable_mutex);
	return ret;
}

static int lightsensor_disable(struct cm36771_info *lpi)
{
	int ret = -EIO;
	mutex_lock(&als_disable_mutex);
	D("[LS][CM36771] %s\n", __func__);

	if (lpi->als_enable == 0)
	{
		D("[LS][CM36771] %s: already disabled\n", __func__);
		ret = 0;
	}
	else
	{
		ret = control_and_report(lpi, CONTROL_ALS, 0);
	}
	
	mutex_unlock(&als_disable_mutex);
	return ret;
}

static int lightsensor_open(struct inode *inode, struct file *file)
{
	struct cm36771_info *lpi = lp_info;
	int rc = 0;

	D("[LS][CM36771] %s\n", __func__);
	if (lpi->lightsensor_opened)
	{
		pr_err("[LS][CM36771 error]%s: already opened\n", __func__);
		rc = -EBUSY;
	}
	lpi->lightsensor_opened = 1;
	return rc;
}

static int lightsensor_release(struct inode *inode, struct file *file)
{
	struct cm36771_info *lpi = lp_info;

	D("[LS][CM36771] %s\n", __func__);
	lpi->lightsensor_opened = 0;
	return 0;
}

static long lightsensor_ioctl(struct file *file, unsigned int cmd,
		unsigned long arg)
{
	int rc, val;
	struct cm36771_info *lpi = lp_info;

	/*D("[CM36771] %s cmd %d\n", __func__, _IOC_NR(cmd));*/

	switch (cmd)
	{
		case LIGHTSENSOR_IOCTL_ENABLE:
			if (get_user(val, (unsigned long __user *)arg)) {
				rc = -EFAULT;
				break;
			}
			D("[LS][CM36771] %s LIGHTSENSOR_IOCTL_ENABLE, value = %d\n",
				__func__, val);
			rc = val ? lightsensor_enable(lpi) : lightsensor_disable(lpi);
			break;
		case LIGHTSENSOR_IOCTL_GET_ENABLED:
			val = lpi->als_enable;
			D("[LS][CM36771] %s LIGHTSENSOR_IOCTL_GET_ENABLED, enabled %d\n",
				__func__, val);
			rc = put_user(val, (unsigned long __user *)arg);
			break;
		default:
			pr_err("[LS][CM36771 error]%s: invalid cmd %d\n",
				__func__, _IOC_NR(cmd));
			rc = -EINVAL;
	}

	return rc;
}

static const struct file_operations lightsensor_fops = {
	.owner = THIS_MODULE,
	.open = lightsensor_open,
	.release = lightsensor_release,
	.unlocked_ioctl = lightsensor_ioctl
};

static struct miscdevice lightsensor_misc = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "lightsensor",
	.fops = &lightsensor_fops
};

static ssize_t ps_adc_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{

	uint16_t value;
	int ret;
	struct cm36771_info *lpi = lp_info;
	int intr_val;

	intr_val = gpio_get_value(lpi->intr_pin);

	get_ps_adc_value(&value);

	ret = sprintf(buf, "ADC[0x%04X], ENABLE = %d, intr_pin = %d\n", value, lpi->ps_enable, intr_val);

	return ret;
}

static ssize_t ps_enable_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	int ps_en;
	struct cm36771_info *lpi = lp_info;

	ps_en = -1;
	sscanf(buf, "%d", &ps_en);

	if (ps_en != 0 && ps_en != 1
		&& ps_en != 10 && ps_en != 13 && ps_en != 16)
		return -EINVAL;

	if (ps_en)
	{
		D("[PS][CM36771] %s: ps_en=%d\n",
			__func__, ps_en);
		psensor_enable(lpi);
	}
	else
	{
		psensor_disable(lpi);
	}

	D("[PS][CM36771] %s\n", __func__);

	return count;
}

static DEVICE_ATTR(ps_adc, 0664, ps_adc_show, ps_enable_store);

unsigned PS_cmd_test_value;
static ssize_t ps_parameters_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	int ret;
	struct cm36771_info *lpi = lp_info;

	ret = sprintf(buf, "PS_close_thd_set = 0x%x, PS_away_thd_set = 0x%x, PS_cmd_cmd:value = 0x%x\n",
		lpi->ps_close_thd_set, lpi->ps_away_thd_set, PS_cmd_test_value);

	return ret;
}

static ssize_t ps_parameters_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{

	struct cm36771_info *lpi = lp_info;
	char *token[10];
	int i;

	pr_debug("[PS][CM36771] %s\n", buf);
	for (i = 0; i < 3; i++)
		token[i] = strsep((char **)&buf, " ");

	lpi->ps_close_thd_set = simple_strtoul(token[0], NULL, 16);
	lpi->ps_away_thd_set = simple_strtoul(token[1], NULL, 16);	
	PS_cmd_test_value = simple_strtoul(token[2], NULL, 16);
	pr_debug("[PS][CM36771]Set PS_close_thd_set = 0x%x, PS_away_thd_set = 0x%x, PS_cmd_cmd:value = 0x%x\n",
		lpi->ps_close_thd_set, lpi->ps_away_thd_set, PS_cmd_test_value);

	D("[PS][CM36771] %s\n", __func__);

	return count;
}

static DEVICE_ATTR(ps_parameters, 0664,
	ps_parameters_show, ps_parameters_store);


static ssize_t ps_conf_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct cm36771_info *lpi = lp_info;
	return sprintf(buf, "PS_CONF1 = 0x%x, PS_CONF3 = 0x%x\n", lpi->ps_conf1_val, lpi->ps_conf3_val);
}
static ssize_t ps_conf_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	int code1, code2;
	struct cm36771_info *lpi = lp_info;

	sscanf(buf, "0x%x 0x%x", &code1, &code2);

	D("[PS]%s: store value PS conf1 reg = 0x%x PS conf3 reg = 0x%x\n", __func__, code1, code2);

	lpi->ps_conf1_val = code1;
	lpi->ps_conf3_val = code2;

	cm36771_i2c_write_word(lpi->slave_addr, PS_CONF3, lpi->ps_conf3_val);  
	cm36771_i2c_write_word(lpi->slave_addr, PS_CONF1, lpi->ps_conf1_val);

	return count;
}
static DEVICE_ATTR(ps_conf, 0664, ps_conf_show, ps_conf_store);

static ssize_t ps_thd_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	int ret;
	struct cm36771_info *lpi = lp_info;
	ret = sprintf(buf, "%s ps_close_thd_set = 0x%x, ps_away_thd_set = 0x%x\n", __func__, lpi->ps_close_thd_set, lpi->ps_away_thd_set);
	return ret;	
}
static ssize_t ps_thd_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	int code;
	struct cm36771_info *lpi = lp_info;

	sscanf(buf, "0x%x", &code);

	D("[PS]%s: store value = 0x%x\n", __func__, code);

	lpi->ps_away_thd_set = code & 0xFF;
	lpi->ps_close_thd_set = (code & 0xFF00) >> 8;	

	D("[PS]%s: ps_close_thd_set = 0x%x, ps_away_thd_set = 0x%x\n", __func__, lpi->ps_close_thd_set, lpi->ps_away_thd_set);

	return count;
}
static DEVICE_ATTR(ps_thd, 0664, ps_thd_show, ps_thd_store);

static ssize_t ps_hw_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	int ret = 0;
	struct cm36771_info *lpi = lp_info;
#if 1
	// read back from HW
	uint16_t ps_conf1_val = 0x0000;
	uint16_t int_flag = 0x0000;

	ret = cm36771_i2c_read_word(lpi->slave_addr, PS_CONF1, &ps_conf1_val);
	ret = cm36771_i2c_read_word(lpi->slave_addr, INT_FLAG, &int_flag);
	ret = sprintf(buf, "read-PS1: reg = 0x%x, PS3: reg = 0x%x, ps_close_thd_set = 0x%x, ps_away_thd_set = 0x%x, INT_FLAG = 0x%x\n",
		ps_conf1_val, lpi->ps_conf3_val, lpi->ps_close_thd_set, lpi->ps_away_thd_set, int_flag);
#else
	ret = sprintf(buf, "PS1: reg = 0x%x, PS3: reg = 0x%x, ps_close_thd_set = 0x%x, ps_away_thd_set = 0x%x\n",
		lpi->ps_conf1_val, lpi->ps_conf3_val, lpi->ps_close_thd_set, lpi->ps_away_thd_set);
#endif

	return ret;
}
static ssize_t ps_hw_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	int code;
//	struct cm36771_info *lpi = lp_info;

	sscanf(buf, "0x%x", &code);

	D("[PS]%s: store value = 0x%x\n", __func__, code);

	return count;
}
static DEVICE_ATTR(ps_hw, 0664, ps_hw_show, ps_hw_store);

static ssize_t ls_adc_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	int ret;
	struct cm36771_info *lpi = lp_info;
	uint16_t adc_value = 0;
	uint32_t lux_value;
	
	if (lpi->als_enable)
	{
		get_ls_adc_value(&adc_value, 0);
		lux_value = (uint32_t)div64_u64((uint64_t)adc_value * lpi->als_resolution * lpi->cal_data, (uint64_t)100000 * 100000);	
		lpi->current_lux = lux_value;
		lpi->current_adc = adc_value;
	}
	
	D("[LS][CM36771] %s: ADC = 0x%04X, Lux = %d \n",
		__func__, lpi->current_adc, lpi->current_lux);
	ret = sprintf(buf, "ADC[0x%04X] => lux %d\n",
		lpi->current_adc, lpi->current_lux);

	return ret;
}

static ssize_t ls_adc_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	return count;
}

static DEVICE_ATTR(ls_adc, 0664, ls_adc_show, ls_adc_store);

static ssize_t ls_enable_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{

	int ret = 0;
	struct cm36771_info *lpi = lp_info;

	ret = sprintf(buf, "Light sensor Auto Enable = %d\n",
			lpi->als_enable);

	return ret;
}

static ssize_t ls_enable_store(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t count)
{
	int ret = 0;
	int ls_auto;
	struct cm36771_info *lpi = lp_info;

	ls_auto = -1;
	sscanf(buf, "%d", &ls_auto);

	if (ls_auto != 0 && ls_auto != 1 && ls_auto != 147)
		return -EINVAL;

	if (ls_auto)
	{
		ret = lightsensor_enable(lpi);
	}
	else
	{
		ret = lightsensor_disable(lpi);
	}

	D("[LS][CM36771] %s: lpi->als_enable = %d, ls_auto = %d\n",
		__func__, lpi->als_enable, ls_auto);

	if (ret < 0)
		pr_err(
		"[LS][CM36771 error]%s: set auto light sensor fail\n",
		__func__);

	return count;
}

static DEVICE_ATTR(ls_auto, 0664, ls_enable_show, ls_enable_store);

static ssize_t ls_conf_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	struct cm36771_info *lpi = lp_info;
	return sprintf(buf, "ALS_CONF = %x\n", lpi->ls_cmd);
}
static ssize_t ls_conf_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct cm36771_info *lpi = lp_info;
	int value = 0;
	sscanf(buf, "0x%x", &value);

	lpi->ls_cmd = value;
	pr_info("[LS]set ALS_CONF = %x\n", lpi->ls_cmd);
	
	cm36771_i2c_write_word(lpi->slave_addr, ALS_CONF, lpi->ls_cmd);
	return count;
}
static DEVICE_ATTR(ls_conf, 0664, ls_conf_show, ls_conf_store);

static ssize_t ls_cal_data_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	struct cm36771_info *lpi = lp_info;
	int ret;

	ret = sprintf(buf, "%d\n", lpi->cal_data);

	return ret;
}

static ssize_t ls_cal_data_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	uint32_t new_cal_data = 0;
	struct cm36771_info *lpi = lp_info;	
	struct file *cal_filp = NULL;
	mm_segment_t old_fs;
	int err = 0;

	sscanf(buf, "%d", &new_cal_data);
	if (new_cal_data != 0)
	{
		lpi->cal_data = new_cal_data;
	}
	else  // reset calibration data
	{
		lpi->cal_data = (uint32_t)(2000000*0.13);
	}

	old_fs = get_fs();
	set_fs(KERNEL_DS);

	cal_filp = filp_open(CALIBRATION_FILE_PATH,
			O_CREAT | O_TRUNC | O_WRONLY, 0666);
	if (IS_ERR(cal_filp))
	{
		pr_err("[LS][CM36771] %s: Can't open calibration file\n", __func__);
		set_fs(old_fs);
		err = PTR_ERR(cal_filp);
		return err;
	}

	err = cal_filp->f_op->write(cal_filp,
		(char *)&lpi->cal_data, sizeof(uint32_t), &cal_filp->f_pos);
	if (err != sizeof(uint32_t))
	{
		pr_err("[LS][CM36771] %s: Can't write the calibration data to file\n", __func__);
		err = -EIO;
	}

	filp_close(cal_filp, current->files);
	set_fs(old_fs);

	return count;
}
static DEVICE_ATTR(ls_cal_data, 0664, ls_cal_data_show, ls_cal_data_store);

static int lightsensor_setup(struct cm36771_info *lpi)
{
	int ret;

	lpi->ls_input_dev = input_allocate_device();
	if (!lpi->ls_input_dev)
	{
		pr_err("[LS][CM36771 error]%s: could not allocate ls input device\n",
			__func__);
		return -ENOMEM;
	}
	lpi->ls_input_dev->name = "cm36771-ls";
	set_bit(EV_ABS, lpi->ls_input_dev->evbit);
	input_set_abs_params(lpi->ls_input_dev, ABS_MISC, 0, 9, 0, 0);

	ret = input_register_device(lpi->ls_input_dev);
	if (ret < 0)
	{
		pr_err("[LS][CM36771 error]%s: can not register ls input device\n",
				__func__);
		goto err_free_ls_input_device;
	}

	ret = misc_register(&lightsensor_misc);
	if (ret < 0)
	{
		pr_err("[LS][CM36771 error]%s: can not register ls misc device\n",
				__func__);
		goto err_unregister_ls_input_device;
	}

	return ret;

err_unregister_ls_input_device:
	input_unregister_device(lpi->ls_input_dev);
err_free_ls_input_device:
	input_free_device(lpi->ls_input_dev);
	return ret;
}

static int psensor_setup(struct cm36771_info *lpi)
{
	int ret;

	lpi->ps_input_dev = input_allocate_device();
	if (!lpi->ps_input_dev)
	{
		pr_err("[PS][CM36771 error]%s: could not allocate ps input device\n",
			__func__);
		return -ENOMEM;
	}
	lpi->ps_input_dev->name = "cm36771-ps";
	set_bit(EV_ABS, lpi->ps_input_dev->evbit);
	input_set_abs_params(lpi->ps_input_dev, ABS_DISTANCE, 0, 1, 0, 0);

	ret = input_register_device(lpi->ps_input_dev);
	if (ret < 0)
	{
		pr_err("[PS][CM36771 error]%s: could not register ps input device\n",
			__func__);
		goto err_free_ps_input_device;
	}

	ret = misc_register(&psensor_misc);
	if (ret < 0)
	{
		pr_err("[PS][CM36771 error]%s: could not register ps misc device\n",
			__func__);
		goto err_unregister_ps_input_device;
	}

	return ret;

err_unregister_ps_input_device:
	input_unregister_device(lpi->ps_input_dev);
err_free_ps_input_device:
	input_free_device(lpi->ps_input_dev);
	return ret;
}

static int cm36771_setup(struct cm36771_info *lpi)
{
	int ret = 0;

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,1,0)
	if(gpio_is_valid(lpi->pwr_pin)) {
		ret = gpio_request(lpi->pwr_pin, "gpio_cm36771_pwr");
		if(ret < 0) {
			pr_err("[PS][CM36771 error]%s: gpio %d request failed (%d)\n",
				__func__, lpi->pwr_pin, ret);
			return ret;
		}

		ret = gpio_direction_output(lpi->pwr_pin, 0);
		if (ret < 0)
		{
			pr_err("[PS][CM36771 error]%s: fail to set gpio %d as input (%d)\n",
				__func__, lpi->pwr_pin, ret);
			goto fail_free_pwr_pin;
		}
	}
#endif

	als_power(1);
	msleep(5);
	ret = gpio_request(lpi->intr_pin, "gpio_cm36771_intr");
	if (ret < 0)
	{
		pr_err("[PS][CM36771 error]%s: gpio %d request failed (%d)\n",
			__func__, lpi->intr_pin, ret);
		return ret;
	}

	ret = gpio_direction_input(lpi->intr_pin);
	if (ret < 0)
	{
		pr_err("[PS][CM36771 error]%s: fail to set gpio %d as input (%d)\n",
			__func__, lpi->intr_pin, ret);
		goto fail_free_intr_pin;
	}

	/* Default disable P sensor and L sensor */
	ls_initial_cmd(lpi);
	psensor_initial_cmd(lpi);

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,1,0)
	ret = request_irq(lpi->irq, cm36771_irq_handler, IRQF_TRIGGER_LOW, lpi->i2c_client->name, lpi);
	if(ret < 0) 
	{
		pr_err("[PS][CM36771 error]%s: req_irq(%d) fail for gpio %d (%d)\n",
			__func__, lpi->irq,
			lpi->intr_pin, ret);
		goto fail_free_intr_pin;
	}
#else
	ret = request_any_context_irq(lpi->irq,
			cm36771_irq_handler,
			IRQF_TRIGGER_LOW,
			"cm36771",
			lpi);
	if (ret < 0)
	{
		pr_err("[PS][CM36771 error]%s: req_irq(%d) fail for gpio %d (%d)\n",
			__func__, lpi->irq,
			lpi->intr_pin, ret);
		goto fail_free_intr_pin;
	}
#endif

	return ret;

fail_free_intr_pin:
	gpio_free(lpi->intr_pin);

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,1,0)
fail_free_pwr_pin:
	if(gpio_is_valid(lpi->pwr_pin))
		gpio_free(lpi->pwr_pin);
#endif

	return ret;
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static void cm36771_early_suspend(struct early_suspend *h)
{
	struct cm36771_info *lpi = lp_info;

	D("[LS][CM36771] %s\n", __func__);

	if (lpi->als_enable)
		lightsensor_disable(lpi);
}

static void cm36771_late_resume(struct early_suspend *h)
{
	struct cm36771_info *lpi = lp_info;

	D("[LS][CM36771] %s\n", __func__);

	if (!lpi->als_enable)
		lightsensor_enable(lpi);
}
#else
static int cm36771_pm_suspend(struct device *dev)
{
	struct cm36771_info *lpi = lp_info;

	if (lpi->als_enable)
		lightsensor_disable(lpi);
	if (lpi->ps_enable)
		psensor_disable(lpi);

	return 0;
}

static int cm36771_pm_resume(struct device *dev)
{
	struct cm36771_info *lpi = lp_info;

	if (!lpi->als_enable)
		lightsensor_enable(lpi);
	if (!lpi->ps_enable)
		psensor_enable(lpi);

	return 0;
}

static SIMPLE_DEV_PM_OPS(cm36771_pm, cm36771_pm_suspend, cm36771_pm_resume);
#endif

static int cm36771_probe(struct i2c_client *client,
	const struct i2c_device_id *id)
{
	int ret = 0;
	struct cm36771_info *lpi;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,1,0)
#else
	struct cm36771_platform_data *pdata;
#endif

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,1,0)
	struct device_node *devnode = client->dev.of_node;
#endif

	D("[PS][CM36771] %s\n", __func__);

	lpi = kzalloc(sizeof(struct cm36771_info), GFP_KERNEL);
	if (!lpi)
		return -ENOMEM;

	/* D("[CM36771] %s: client->irq = %d\n", __func__, client->irq); */

	lpi->i2c_client = client;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,1,0)
#else
	pdata = client->dev.platform_data;
	if (!pdata)
	{
		pr_err("[PS][CM36771 error]%s: Assign platform_data error!!\n",
			__func__);
		ret = -EBUSY;
		goto err_platform_data_null;
	}
#endif

	lpi->irq = client->irq;

	i2c_set_clientdata(client, lpi);

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,1,0)
	if(devnode) //if use the device tree config
	{
		lpi->intr_pin = of_get_named_gpio(devnode, "intr-pin", 0);
		lpi->pwr_pin = of_get_named_gpio(devnode, "pwr-pin", 0);
	}

	lpi->slave_addr = 0x60;
	
	// low byte, lpi->ps_away_thd_set = 0x3;
	lpi->ps_away_thd_set = 0x30;
	// high byte, lpi->ps_close_thd_set = 0x8;
	lpi->ps_close_thd_set = 0x40;
	// lpi->ps_conf1_val = CM36771_CMD03_DEFAULT | CM36771_CMD03_PS_DR_1_640 | CM36771_CMD03_PS_IT_2T | CM36771_CMD03_PS_PERS_2 | CM36771_CMD03_PS_BC_CONF1_2_3;
	lpi->ps_conf1_val = CM36771_CMD03_DEFAULT | CM36771_CMD03_PS_DR_1_20 | CM36771_CMD03_PS_IT_2T | CM36771_CMD03_PS_PERS_2 | CM36771_CMD03_PS_BC_CONF1_2_3;
	lpi->ps_conf3_val = CM36771_CMD04_DEFAULT | CM36771_CMD04_PS_BC_CONF2_2;
	lpi->ls_cmd = CM36771_CMD00_ALS_IT_160MS | CM36771_CMD00_ALS_PERS_2;
	
	D("[PS][CM36771] %s: ls_cmd 0x%x\n",
		__func__, lpi->ls_cmd);
	
	if (lpi->ls_cmd == 0)
	{
		lpi->ls_cmd = CM36771_CMD00_ALS_IT_160MS | CM36771_CMD00_ALS_PERS_2;
	}
#else	
	lpi->intr_pin = pdata->intr;
	lpi->power = pdata->power;
	
	lpi->slave_addr = pdata->slave_addr;
	
	lpi->ps_away_thd_set = pdata->ps_away_thd_set;
	lpi->ps_close_thd_set = pdata->ps_close_thd_set;	
	lpi->ps_conf1_val = pdata->ps_conf1_val;
	lpi->ps_conf3_val = pdata->ps_conf3_val;
	
	lpi->ls_cmd  = pdata->ls_cmd;
	
	D("[PS][CM36771] %s: ls_cmd 0x%x\n",
		__func__, lpi->ls_cmd);
	
	if (pdata->ls_cmd == 0)
	{
		lpi->ls_cmd = CM36771_CMD00_ALS_IT_160MS | CM36771_CMD00_ALS_PERS_2;
	}
#endif

	lp_info = lpi;

	mutex_init(&CM36771_control_mutex);

	mutex_init(&als_enable_mutex);
	mutex_init(&als_disable_mutex);

	ret = lightsensor_setup(lpi);
	if (ret < 0)
	{
		pr_err("[LS][CM36771 error]%s: lightsensor_setup error!!\n",
			__func__);
		goto err_lightsensor_setup;
	}

	lpi->als_resolution = 5000;
	lpi->cal_data = (uint32_t)(2000000*0.13);
	
	mutex_init(&ps_enable_mutex);
	mutex_init(&ps_disable_mutex);

	ret = psensor_setup(lpi);
	if (ret < 0)
	{
		pr_err("[PS][CM36771 error]%s: psensor_setup error!!\n",
			__func__);
		goto err_psensor_setup;
	}

	lpi->lp_wq = create_singlethread_workqueue("cm36771_wq");
	if (!lpi->lp_wq)
	{
		pr_err("[PS][CM36771 error]%s: can't create workqueue\n", __func__);
		ret = -ENOMEM;
		goto err_create_singlethread_workqueue;
	}
#ifdef CONFIG_HAS_WAKELOCK
	wake_lock_init(&(lpi->ps_wake_lock), WAKE_LOCK_SUSPEND, "proximity");
#endif

	ret = cm36771_setup(lpi);
	if (ret < 0)
	{
		pr_err("[PS_ERR][CM36771 error]%s: cm36771_setup error!\n", __func__);
		goto err_cm36771_setup;
	}
	lpi->cm36771_class = class_create(THIS_MODULE, "optical_sensors");
	if (IS_ERR(lpi->cm36771_class))
	{
		ret = PTR_ERR(lpi->cm36771_class);
		lpi->cm36771_class = NULL;
		goto err_create_class;
	}

	lpi->ls_dev = device_create(lpi->cm36771_class,
				NULL, 0, "%s", "lightsensor");
	if (unlikely(IS_ERR(lpi->ls_dev)))
	{
		ret = PTR_ERR(lpi->ls_dev);
		lpi->ls_dev = NULL;
		goto err_create_ls_device;
	}

	/* register the attributes */
	ret = device_create_file(lpi->ls_dev, &dev_attr_ls_adc);
	if (ret)
		goto err_create_ls_device_file;

	/* register the attributes */
	ret = device_create_file(lpi->ls_dev, &dev_attr_ls_auto);
	if (ret)
		goto err_create_ls_device_file;

	/* register the attributes */
	ret = device_create_file(lpi->ls_dev, &dev_attr_ls_conf);
	if (ret)
		goto err_create_ls_device_file;

	/* register the attributes */
	ret = device_create_file(lpi->ls_dev, &dev_attr_ls_cal_data);
	if (ret)
		goto err_create_ls_device_file;		
		
	lpi->ps_dev = device_create(lpi->cm36771_class,
				NULL, 0, "%s", "proximity");
	if (unlikely(IS_ERR(lpi->ps_dev)))
	{
		ret = PTR_ERR(lpi->ps_dev);
		lpi->ps_dev = NULL;
		goto err_create_ls_device_file;
	}

	/* register the attributes */
	ret = device_create_file(lpi->ps_dev, &dev_attr_ps_adc);
	if (ret)
		goto err_create_ps_device;

	ret = device_create_file(lpi->ps_dev,
		&dev_attr_ps_parameters);
	if (ret)
		goto err_create_ps_device;

	/* register the attributes */
	ret = device_create_file(lpi->ps_dev, &dev_attr_ps_conf);
	if (ret)
		goto err_create_ps_device;

	/* register the attributes */
	ret = device_create_file(lpi->ps_dev, &dev_attr_ps_thd);
	if (ret)
		goto err_create_ps_device;

	ret = device_create_file(lpi->ps_dev, &dev_attr_ps_hw);
	if (ret)
		goto err_create_ps_device;

#ifdef CONFIG_HAS_EARLYSUSPEND
	lpi->early_suspend.level =
			EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	lpi->early_suspend.suspend = cm36771_early_suspend;
	lpi->early_suspend.resume = cm36771_late_resume;
	register_early_suspend(&lpi->early_suspend);
#endif

	D("[PS][CM36771] %s: Probe success!\n", __func__);

	return ret;

err_create_ps_device:
	device_unregister(lpi->ps_dev);
err_create_ls_device_file:
	device_unregister(lpi->ls_dev);
err_create_ls_device:
	class_destroy(lpi->cm36771_class);
err_create_class:
err_cm36771_setup:
	destroy_workqueue(lpi->lp_wq);

#ifdef CONFIG_HAS_WAKELOCK
	wake_lock_destroy(&(lpi->ps_wake_lock));
#endif

	input_unregister_device(lpi->ls_input_dev);
	input_free_device(lpi->ls_input_dev);
	input_unregister_device(lpi->ps_input_dev);
	input_free_device(lpi->ps_input_dev);
err_create_singlethread_workqueue:
	misc_deregister(&psensor_misc);
err_psensor_setup:
	mutex_destroy(&CM36771_control_mutex);
	mutex_destroy(&ps_enable_mutex);
	mutex_destroy(&ps_disable_mutex);
	misc_deregister(&lightsensor_misc);
err_lightsensor_setup:
	mutex_destroy(&als_enable_mutex);
	mutex_destroy(&als_disable_mutex);
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,1,0)
#else
err_platform_data_null:
#endif
	kfree(lpi);
	return ret;
}
   
static int control_and_report(struct cm36771_info *lpi, uint8_t mode, uint16_t param)
{
	int ret = 0;
	uint16_t adc_value = 0;
	uint32_t lux_value = 0;
	uint16_t low_thd;
	uint32_t high_thd;	
	uint16_t ps_data = 0;	
	int val;
	
	mutex_lock(&CM36771_control_mutex);

	if (mode == CONTROL_ALS)
	{
		if (param)
		{
			// enable interrupt
			lpi->ls_cmd |= CM36771_CMD00_ALS_INT_EN;
			lpi->ls_cmd &= ~CM36771_CMD00_ALS_SD;      
		}
		else
		{
			// disable interrupt
			lpi->ls_cmd &= ~CM36771_CMD00_ALS_INT_EN;
			lpi->ls_cmd |= CM36771_CMD00_ALS_SD;
		}
		cm36771_i2c_write_word(lpi->slave_addr, ALS_CONF, lpi->ls_cmd);
		lpi->als_enable=param;
	}
	else if (mode == CONTROL_PS)
	{
		if (param)
		{ 
			lpi->ps_conf1_val &= ~CM36771_CMD03_PS_SD;
			lpi->ps_conf1_val |= CM36771_CMD03_PS_INT_EN;      
		}
		else
		{
			lpi->ps_conf1_val |= CM36771_CMD03_PS_SD;
			lpi->ps_conf1_val &= ~CM36771_CMD03_PS_INT_EN;
		}
		cm36771_i2c_write_word(lpi->slave_addr, PS_CONF1, lpi->ps_conf1_val);    
		lpi->ps_enable=param;  
	}
  
	if ((mode == CONTROL_ALS) || (mode == CONTROL_PS))
	{  
		if (param == 1)
		{
			msleep(100);  
		}
	}
     	
	if (lpi->als_enable)
	{
		if (mode == CONTROL_ALS)
		{
			set_lsensor_range(65535, 0);
			
			input_report_abs(lpi->ls_input_dev, ABS_MISC, -1);
			input_sync(lpi->ls_input_dev);		
		}
		else if	((mode == CONTROL_INT_ISR_REPORT) && ((param & INT_FLAG_ALS_IF_L) || (param & INT_FLAG_ALS_IF_H)))
		{
			// disable interrupt
			lpi->ls_cmd &= ~CM36771_CMD00_ALS_INT_EN;
			ret = cm36771_i2c_write_word(lpi->slave_addr, ALS_CONF, lpi->ls_cmd);  
      
			get_ls_adc_value(&adc_value, 0);
			lux_value = (uint32_t)div64_u64((uint64_t)adc_value * lpi->als_resolution * lpi->cal_data, (uint64_t)100000 * 100000);  
			
			// set interrupt high/low threshold
			low_thd = (uint16_t)((uint32_t)adc_value * (100 - CHANGE_SENSITIVITY) / 100);
			high_thd = (uint32_t)adc_value * (100 + CHANGE_SENSITIVITY) / 100;
			if (high_thd > 65535)
			{
				high_thd = 65535;
			}
			ret = set_lsensor_range(low_thd, (uint16_t)high_thd);
			
			// enable interrupt
			lpi->ls_cmd |= CM36771_CMD00_ALS_INT_EN;	  
			ret = cm36771_i2c_write_word(lpi->slave_addr, ALS_CONF, lpi->ls_cmd);  
    	  
			D("[CM36771] %s: ADC = 0x%04X, Lux = %d, l_thd = 0x%x, h_thd = 0x%x\n",
					__func__, adc_value, lux_value, low_thd, high_thd);
					
    		lpi->current_lux = lux_value;
    		lpi->current_adc = adc_value;    
			input_report_abs(lpi->ls_input_dev, ABS_MISC, lux_value);
			input_sync(lpi->ls_input_dev);
		}
	}

#define PS_CLOSE 1
#define PS_AWAY  (1<<1)
#define PS_CLOSE_AND_AWAY PS_CLOSE+PS_AWAY

	if (lpi->ps_enable)
	{
		int ps_status = 0;
		if (mode == CONTROL_PS)
			ps_status = PS_CLOSE_AND_AWAY;   
		else if (mode == CONTROL_INT_ISR_REPORT)
		{  
			if (param & INT_FLAG_PS_IF_CLOSE)
				ps_status |= PS_CLOSE;      
			if (param & INT_FLAG_PS_IF_AWAY)
				ps_status |= PS_AWAY;
		}
      
		if (ps_status !=0)
		{
			switch (ps_status)
			{
				case PS_CLOSE_AND_AWAY:
					get_stable_ps_adc_value(&ps_data);
					val = (ps_data >= lpi->ps_close_thd_set) ? 0 : 1;
					break;
				case PS_AWAY:
					val = 1;
					break;
				case PS_CLOSE:
					val = 0;
					break;
			};
			input_report_abs(lpi->ps_input_dev, ABS_DISTANCE, val);      
			input_sync(lpi->ps_input_dev);        
		}
	}

	mutex_unlock(&CM36771_control_mutex);
	return ret;
}


static const struct i2c_device_id cm36771_i2c_id[] = {
	{CM36771_I2C_NAME, 0},
	{}
};

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,1,0)
static const struct of_device_id cm36771_i2c_dt_ids[] = {
	{ .compatible = "capella,"CM36771_I2C_NAME },
	{ }
};
#endif

static struct i2c_driver cm36771_driver = {
	.id_table = cm36771_i2c_id,
	.probe = cm36771_probe,
	.driver = {
		.name = CM36771_I2C_NAME,
		.owner = THIS_MODULE,
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,1,0)
		.of_match_table = cm36771_i2c_dt_ids,
#endif
#ifndef CONFIG_HAS_EARLYSUSPEND
		.pm = &cm36771_pm,
#endif
	},
};

static int __init cm36771_init(void)
{
	return i2c_add_driver(&cm36771_driver);
}

static void __exit cm36771_exit(void)
{
	i2c_del_driver(&cm36771_driver);
}

module_init(cm36771_init);
module_exit(cm36771_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("CM36771 Driver");
MODULE_AUTHOR("Capella Microsystems");
