#include <linux/module.h>
#include <linux/types.h>
#include <linux/slab.h>
#include <linux/kernel.h>
#include <linux/watchdog.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/delay.h>

#if 0
    #define dprint(fmt,s...) printk("collin:%s,%d:"fmt,__FUNCTION__,__LINE__,##s)
#else
    #define dprint(fmt,s...)
#endif    

#define RESET_PIN           14
#define TPL5010_DONE_PIN    13
#define WAKE_PIN            12
#define TIMEOUT             30

struct delayed_work  mydelay;
static bool nowayout = WATCHDOG_NOWAYOUT;
module_param(nowayout, bool, 0);
MODULE_PARM_DESC(nowayout, "Watchdog cannot be stopped once started "
	"(default=" __MODULE_STRING(WATCHDOG_NOWAYOUT) ")");

static const struct watchdog_info tpl5010_wdt_info = {
	.options = WDIOF_SETTIMEOUT | WDIOF_MAGICCLOSE | WDIOF_KEEPALIVEPING,
//	.options = WDIOF_SETTIMEOUT | WDIOF_MAGICCLOSE,    
	.identity = "TPL5010 Watchdog",
};

static int tpl5010_wdt_ping(struct watchdog_device *wdog)
{
//	struct imx2_wdt_device *wdev = watchdog_get_drvdata(wdog);
        dprint("\n");
        gpio_set_value(TPL5010_DONE_PIN,1);
        udelay(10);                
        //msleep(20);
        gpio_set_value(TPL5010_DONE_PIN,0);
        
	return 0;
}


static int wdt_start(struct watchdog_device *wdog)
{
        dprint("\n");    
        return 0;
}

static int wdt_stop(struct watchdog_device *wdog)
{
        dprint("\n");
        return 0;
}

static const struct watchdog_ops tpl5010_wdt_ops = {
	.owner		= THIS_MODULE,
	.start          = wdt_start,
	.stop           = wdt_stop,        
	.ping           = tpl5010_wdt_ping,        
};

void my_workqueue_handler(struct delayed_work *work)
{
        static int counter = 0;
        dprint("\n");
        gpio_set_value(TPL5010_DONE_PIN,1);
        udelay(10);                
        //msleep(20);        
        gpio_set_value(TPL5010_DONE_PIN,0);
        
        if( counter++ < 10 )
            schedule_delayed_work(&mydelay, HZ*10);
        
}

static int tpl5010_wdt_probe(struct platform_device *pdev)
{
	int ret = 0;
	struct watchdog_device *wdt;
        dprint("\n");
	wdt = devm_kzalloc(&pdev->dev, sizeof(*wdt), GFP_KERNEL);
	if (!wdt)
		return -ENOMEM;

	wdt->info		= &tpl5010_wdt_info;
	wdt->ops		= &tpl5010_wdt_ops;
	wdt->status		= 0;
	wdt->timeout		= TIMEOUT;
	wdt->min_timeout	= 1;
	wdt->max_timeout	= TIMEOUT;
	wdt->parent = &pdev->dev;

	//watchdog_set_nowayout(wdt, nowayout);
	platform_set_drvdata(pdev, wdt);        

        //GPIO init
        ret = gpio_request(TPL5010_DONE_PIN,"tpl5010_down");
        if(ret){
            printk("TPL5010_DONE_PIN request fail\n");
            return 0;
        }
        //Normal low
        ret = gpio_direction_output(TPL5010_DONE_PIN,0);   
        if(ret){            
            printk("TPL5010_DOWN_PIN setting fail\n");
            return 0;
        }
                
        ret = gpio_request(WAKE_PIN,"tpl5010_wake");
        if(ret){            
            printk("TPL5010_WAKE_PIN request fail\n");
            return 0;
        }
        //set as input
        ret = gpio_direction_input(WAKE_PIN);                   
        if(ret){            
            printk("TPL5010_WAKE_PIN setting fail\n");
            return 0;
        }
        
        ret = gpio_request(RESET_PIN,"tpl5010_reset");
        if(ret){            
            printk("TPL5010_RESET_PIN request fail\n");
            return 0;
        }
        //set as input
        ret = gpio_direction_input(RESET_PIN);                   
        if(ret){            
            printk("TPL5010_RESET_PIN setting fail\n");
            return 0;
        }

	ret = watchdog_register_device(wdt);
	if (ret)
		return ret;
       
        INIT_DELAYED_WORK(&mydelay, my_workqueue_handler);

        schedule_delayed_work(&mydelay, HZ*10);
        
        dprint("\n");
	return 0;
}
static int __exit tpl5010_wdt_remove(struct platform_device *pdev)
{
        dprint("\n");
        
	struct watchdog_device *wdog = platform_get_drvdata(pdev);
        
	watchdog_unregister_device(wdog);
	
	return 0;    
}

static const struct of_device_id twl_wdt_of_match[] = {
	{ .compatible = "ti,tpl5010_wdt", },
	{ },
};
MODULE_DEVICE_TABLE(of, twl_wdt_of_match);

static struct platform_driver tpl5010_wdt_driver = {
	.remove		= __exit_p(tpl5010_wdt_remove),    
	.probe		= tpl5010_wdt_probe,
	.driver		= {
		.name		= "tpl5010_wdt",
		.of_match_table	= twl_wdt_of_match,
	},
};

module_platform_driver(tpl5010_wdt_driver);

MODULE_AUTHOR("Bematech Corporation");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:tpl5010_wdt");

