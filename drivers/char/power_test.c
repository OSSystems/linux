#include <linux/module.h>   
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/workqueue.h>
#include <linux/delay.h>

/* This module is for functional test which including all LEDs and buzzer, It's included in
 * kernel image and if you want to start it please un-commend command on file /etc/rc.local
 */

#define BUZ             7
#define WIFI_LED        8 
#define BT_LED          9 
#define RED_LED         84 

#define MY_GPIO_INT_NAME "imx_power_test"
#define MY_DEV_NAME "imx_device"


static unsigned int input_val = 0;
static struct delayed_work mywork,buz_work;

module_param(input_val, uint, 0);
MODULE_PARM_DESC(input_val, "a value");
/*
static short int button_irq = 0;
static unsigned long flags = 0;
static int led_trigger = 0;

static irqreturn_t button_isr(int irq, void *data)
{
	local_irq_save(flags);
	printk("button_isr !!!!\n");
	gpio_set_value(LED, led_trigger);
	led_trigger = led_trigger ? (0):(1);
	local_irq_restore(flags);
	return IRQ_HANDLED;
}
*/

static void turn_off_all(void)
{
            gpio_set_value(BT_LED, 1 );        
            gpio_set_value(WIFI_LED, 1 );        
            gpio_set_value(RED_LED, 1 );        
}

void be_time(int number)
{
    int i;
        for(i=0;i<number ; i++)
        {
            gpio_set_value(BUZ, 1);        
            msleep(300);
            gpio_set_value(BUZ, 0);        
            msleep(300);
        }
}

static void buz_work_run(struct work_struct *work)
{

    if(input_val == 0){
        gpio_set_value(BUZ, 1);        
        ssleep(1);
        gpio_set_value(BUZ, 0);        
        schedule_delayed_work(&buz_work,3*HZ);
    }else {
        be_time(input_val);
        schedule_delayed_work(&buz_work,3*HZ);    
    }
#if 0  

        gpio_set_value(PWM4, 1);        
        udelay(1);
        gpio_set_value(PWM4, 0);        
        udelay(1);
        gpio_set_value(PWM4, 1);        
        udelay(1);
        gpio_set_value(PWM4, 0);        
        udelay(1);
        
        schedule_delayed_work(&buz_work,msecs_to_jiffies(1));
#endif        
}

static void workqueue_run(struct work_struct *work)
{

    static int counter=0;
    
    turn_off_all();
    
    switch(counter){
        case 0:
            gpio_set_value(WIFI_LED, 0 );        
            break;
        case 1:
            gpio_set_value(BT_LED, 0 );        
            break;
        case 2:
            gpio_set_value(RED_LED, 0 );        
            break;         
        default:
            break;
    }
    
    if(++counter > 3) counter = 0;
    
    schedule_delayed_work(&mywork,0.3*HZ);
}

int init_module(void)
{
        
        //printk("enable power test driver\n");
	if(gpio_is_valid(BUZ) < 0) return -1;
	if(gpio_request(BUZ, "BUZ") < 0) return -1;
	gpio_direction_output(BUZ, 0 );
        
	if(gpio_is_valid(BT_LED) < 0) return -1;
	if(gpio_request(BT_LED, "BT_LED") < 0) return -1;
	gpio_direction_output(BT_LED, 0 );
        
	if(gpio_is_valid(WIFI_LED) < 0) return -1;
	if(gpio_request(WIFI_LED, "WIFI_LED") < 0) return -1;
	gpio_direction_output(WIFI_LED, 0 );

	if(gpio_is_valid(RED_LED) < 0) return -1;
	if(gpio_request(RED_LED, "RED_LED") < 0) return -1;
	gpio_direction_output(RED_LED, 0 );
        
              
	INIT_DELAYED_WORK(&mywork, workqueue_run);
	INIT_DELAYED_WORK(&buz_work, buz_work_run);

        if(input_val == 0)
        {
            schedule_delayed_work(&mywork,0.5*HZ);
            schedule_delayed_work(&buz_work,0.5*HZ);
        }
        else
        {
            schedule_delayed_work(&buz_work,0.5*HZ);            
        }
	// -- setup the button gpio as input and request irq
/*        
	if(gpio_request(BUTTON,"BUTTON") < 0) return -1;
	if(gpio_is_valid(BUTTON) < 0) return -1;
	if( (button_irq = gpio_to_irq(BUTTON)) < 0 )  return -1;
	if( request_irq( button_irq, button_isr ,IRQF_TRIGGER_RISING, MY_GPIO_INT_NAME, MY_DEV_NAME)) return -1;
        */
	return 0;

}


void cleanup_module(void)
{
    
//        printk("disable power test driver\n");    
        gpio_set_value(BUZ, 0);   
        turn_off_all();   

	gpio_free(BUZ);
	gpio_free(BT_LED);
	gpio_free(WIFI_LED);
	gpio_free(RED_LED);
        cancel_delayed_work(&buz_work);
        cancel_delayed_work(&mywork);        
}

MODULE_LICENSE("GPL");
