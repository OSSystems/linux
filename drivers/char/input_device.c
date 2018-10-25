#include <linux/input.h>
#include <linux/module.h>
#include <linux/init.h>
#include <asm/irq.h>
#include <asm/io.h>
#include <linux/gpio.h>
#include <linux/workqueue.h>
#include <linux/interrupt.h>

#if 0
    #define dprint(fmt,s...) printk("input:%s,%d:"fmt,__FUNCTION__,__LINE__,##s)
#else
    #define dprint(fmt,s...)
#endif    

#define GPIO_PWR        	107
#define GPIO_PWR_DESC		"power_off_btn"
#define GPIO_PWR_DESC_DEV	"my_power_off_btn"
#define POWER_OFF_TIME          5


#define GPIO_BTN		89  
#define GPIO_BTN_DESC		"function_btn"
#define GPIO_BTN_DESC_DEV	"my_function_btn"

#define GPIO_POWERFAIL		110
#define GPIO_POWERFAIL_DESC	"power_fail_pin"
#define GPIO_POWERFAIL_DESC_DEV "my_power_fail_pin"


#define LCM_PWR                 0

#define NAME			"stuttgart_device"

extern void turn_off_backlight(void);
extern void turn_on_backlight(void);
static struct input_dev *button_dev = NULL;
static short int button_irq = 0;
static void poweroff_handler(struct work_struct *work);
static void button_handler(struct work_struct *work);
static void powerfail_handler(struct work_struct *work);
DECLARE_DELAYED_WORK(my_work,poweroff_handler);
DECLARE_DELAYED_WORK(powerfail_work,powerfail_handler);
DECLARE_DELAYED_WORK(button_work,button_handler);
unsigned int power_button_irq_number,powerfail_irq;

static void button_handler(struct work_struct *work)
{
        enable_irq(power_button_irq_number);    
}

static irqreturn_t button_interrupt(int irq, void *dummy)
{
	int value = gpio_get_value(GPIO_BTN);
        power_button_irq_number = irq;
        
        disable_irq_nosync(power_button_irq_number);
        
	input_report_key(button_dev, BTN_0, value ? 0:1);	//send to input core
	input_sync(button_dev);
     
        schedule_delayed_work(&button_work,msecs_to_jiffies(100));  
        
	return IRQ_HANDLED;
}


static void poweroff_handler(struct work_struct *work)
{
        static int counter = 0;
    	int value = gpio_get_value(GPIO_PWR);
                
        if(value == 0)
        {
            if(counter++ > POWER_OFF_TIME)
            {
                input_report_key(button_dev, BTN_1, value ? 0:1);
                input_sync(button_dev); 
                gpio_direction_output(LCM_PWR,0);
            }
            else
            {
                schedule_delayed_work(&my_work,msecs_to_jiffies(100));                        
            }
        }
        else
            counter = 0;
}

static irqreturn_t power_off_interrupt(int irq, void *dummy)
{
        schedule_delayed_work(&my_work,msecs_to_jiffies(100));

	return IRQ_HANDLED;
}

void powerfail_handler(struct work_struct *work)
{
        
        int value = gpio_get_value(GPIO_POWERFAIL);
        
        if(value == 0)
        {
            turn_off_backlight();
            disable_irq_nosync(powerfail_irq);
            schedule_delayed_work(&powerfail_work,msecs_to_jiffies(20));
            input_report_key( button_dev, BTN_2, 0  );
            input_sync(button_dev);
            enable_irq(powerfail_irq);              
            dprint("\n");           
        }
        else
        {
            turn_on_backlight();
            dprint("\n");           
        }
        

    
}

static irqreturn_t powerfail_interrupt(int irq, void *dummy)
{   

        schedule_delayed_work(&powerfail_work,msecs_to_jiffies(20));
	return IRQ_HANDLED;
}

static int initial_gpio_set(void)
{

	//request GPIO of button
	if(gpio_is_valid(GPIO_BTN) < 0) return -1;    
	if(gpio_request(GPIO_BTN, GPIO_BTN_DESC) < 0) return -1;
        if(gpio_direction_input(GPIO_BTN) < 0 ) return -1;
	
	if (request_irq( gpio_to_irq(GPIO_BTN), 
			(irq_handler_t) button_interrupt,
			 IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING , 
			 GPIO_BTN_DESC,
			 GPIO_BTN_DESC_DEV)) {
                printk("Can't allocate irq %d\n", button_irq);
                return -EBUSY;
        }

	if(gpio_is_valid(GPIO_PWR) < 0) return -1;        
	if(gpio_request(GPIO_PWR, GPIO_PWR_DESC) < 0) return -1;
        if(gpio_direction_input(GPIO_PWR) < 0 ) return -1;        
	
	if (request_irq( gpio_to_irq(GPIO_PWR), 
			(irq_handler_t) power_off_interrupt,
			 IRQF_TRIGGER_FALLING, 
			 GPIO_PWR_DESC,
			 GPIO_PWR_DESC_DEV)) {
               printk("Can't allocate irq %d\n", button_irq);
                return -EBUSY;
        }
                
	if(gpio_is_valid(GPIO_POWERFAIL) < 0) return -1;                
        if(gpio_request(GPIO_POWERFAIL, GPIO_POWERFAIL_DESC) < 0) return -1;
        if(gpio_direction_input(GPIO_POWERFAIL) < 0 ) return -1;   
        
        powerfail_irq = gpio_to_irq(GPIO_POWERFAIL);
	if (request_irq( powerfail_irq, 
			(irq_handler_t) powerfail_interrupt,
                         IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING, 
			 GPIO_POWERFAIL_DESC,
			 GPIO_POWERFAIL_DESC_DEV)) {
               printk("Can't allocate irq for power fail %d\n", button_irq);
               return -EBUSY;
        }

	return 0;
}
static int button_open(struct input_dev *dev)
{
    	int error;
        
    	error = initial_gpio_set();
        
	if(error < 0){
	    printk("initial gpio set fail\n");
	    return -1;
	}	
	
        printk("Open input device init ok\n");                	
        return 0;
}

static int __init button_init(void)
{
    	int error;

	button_dev = input_allocate_device();
	if (!button_dev) {
		printk("Not enough memory\n");
		error = -ENOMEM;
		goto err_free_dev;
	}
	
	button_dev->name = NAME;		
        
        button_dev->open = button_open;
        
//	set_bit(EV_KEY,button_dev->evbit);
        button_dev->evbit[0] = BIT_MASK(EV_KEY);
        button_dev->keybit[BIT_WORD(BTN_0)] |= BIT_MASK(BTN_0) | BIT_MASK(BTN_1) |                      BIT_MASK(BTN_2);
        
	error = input_register_device(button_dev);
	if (error) {
		printk("Failed to register device\n");
		goto err_free_dev;
	}

	
	//input device will avoid report twice, our first report will be low, so report hi first
        input_report_key( button_dev, BTN_2, 1 );
        input_sync(button_dev);

	dprint("\n");
        
	return 0;
        
 err_free_dev:
	input_free_device(button_dev);
	return error;
}

static void __exit button_exit(void)
{

	input_unregister_device(button_dev);
	free_irq(gpio_to_irq(GPIO_BTN),GPIO_BTN_DESC_DEV);
        free_irq(gpio_to_irq(GPIO_PWR),GPIO_PWR_DESC_DEV);
        free_irq(gpio_to_irq(GPIO_POWERFAIL), GPIO_POWERFAIL_DESC_DEV);        
	gpio_free(GPIO_BTN);
	gpio_free(GPIO_PWR); 
        gpio_free(GPIO_POWERFAIL);        
	printk("input_device exit \n");	
}

module_init(button_init);
module_exit(button_exit);
MODULE_LICENSE("GPL");
