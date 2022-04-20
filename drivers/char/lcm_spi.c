#if 0
  #define dprint(fmt,s...) printk("lcd-%s,%d:"fmt,__FUNCTION__,__LINE__,##s)
#else
  #define dprint(fmt,s...)
#endif  

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/slab.h>
#include <linux/device.h>
#include <asm/uaccess.h>
#include <linux/spi/spi.h>
#include <linux/uaccess.h>

#define LCD_RW	47
#define LCD_EN	48
#define LCD_A0	50
#define LCD_RST	51
#define LCD_D0	52
#define LCD_D1	53
#define LCD_D2	54
#define LCD_D3	55
#define LCD_D4	56
#define LCD_D5	57    
#define LCD_BACKLIGHT	0                          
                      
#define DEVICE_NAME		"lcm"
#define NUMBERS			1

#define DISPLAY_ON()           Write_Instruction(0xaf)   //  Display on
#define DISPLAY_OFF()          Write_Instruction(0xae)   //  Display off
#define SET_ADC()              Write_Instruction(0xa1)   //  Reverse disrect (SEG131-SEG0)
#define CLEAR_ADC()            Write_Instruction(0xa0)   //  Normal disrect (SEG0-SEG131)
#define REVERSE_DISPLAY_ON()   Write_Instruction(0xa7)   //  Reverse display : 0 illuminated
#define REVERSE_DISPLAY_OFF()  Write_Instruction(0xa6)   //  Normal display : 1 illuminated
#define ENTIRE_DISPLAY_ON()    Write_Instruction(0xa5)   //  Entire dislay   Force whole LCD point
#define ENTIRE_DISPLAY_OFF()   Write_Instruction(0xa4)   //  Normal display
#define SET_BIAS()             Write_Instruction(0xa3)   //  bias 1   1/7 bais
#define CLEAR_BIAS()           Write_Instruction(0xa2)   //  bias 0   1/9 bais
#define SET_MODIFY_READ()      Write_Instruction(0xe0)   //  Stop automatic increment of the column address by the read instruction 
#define RESET_MODIFY_READ()    Write_Instruction(0xee)   //  Cancel Modify_read, column address return to its initial value just before the Set Modify Read instruction is started
#define RESET()                Write_Instruction(0xe2)	//   system reset
#define SET_SHL()              Write_Instruction(0xc8)   // SHL 1,COM63-COM0
#define CLEAR_SHL()            Write_Instruction(0xc0)   // SHL 0,COM0-COM63

static struct gpio LCD_gpios[]={
	{LCD_RW, GPIOF_OUT_INIT_HIGH, "lcd_rw"},
	{LCD_EN, GPIOF_OUT_INIT_HIGH, "lcd_en"},
	{LCD_A0, GPIOF_OUT_INIT_LOW, "lcd_a0"},
	{LCD_RST, GPIOF_OUT_INIT_HIGH, "lcd_rst"},
	{LCD_D0, GPIOF_OUT_INIT_HIGH, "lcd_d0"},
	{LCD_D1, GPIOF_OUT_INIT_HIGH, "lcd_d1"},
	{LCD_D2, GPIOF_OUT_INIT_HIGH, "lcd_d2"},
	{LCD_D3, GPIOF_OUT_INIT_HIGH, "lcd_d3"},		
	{LCD_D4, GPIOF_OUT_INIT_HIGH, "lcd_d4"},		
	{LCD_D5, GPIOF_OUT_INIT_HIGH, "lcd_d5"},
	{LCD_BACKLIGHT, GPIOF_OUT_INIT_HIGH, "lcd_backlight"}
};   

#define drawer_sense    92
#define drawer_out1     90
#define drawer_out2     88
         
struct my_dev{
	struct class *dev_class; 
	dev_t  dev_num; 		
	struct cdev cdev;
	struct spi_master * spi_master ;
	struct spi_device * spi_device ;	
	int major_num,minor_num;
} *pmy_dev;

static struct spi_board_info my_spi_board[] = {
	  [0] = 
	{ 
		.modalias = "erc12864", 
		.max_speed_hz = 1000000, 
		.bus_num = 0,
		.chip_select = 0,                  
		.mode = SPI_MODE_3,
	}, 
}; 
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                        
u8 bematech_log[] =
{
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x80,0xC0,0xC0,0xE0,0xE0,0xF0,0xF0,0xF8,0xF8,0xF8,0xF8,0xFC,0xFC,
0xFC,0xFC,0xFC,0xF8,0xF8,0xF8,0xF8,0xF0,0xF0,0xE0,0xC0,0x80,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,

0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x80,0xE0,0xF0,0xF8,0xFC,
0xFE,0xFE,0xFF,0x7F,0x3F,0x1F,0x0F,0x0F,0x0F,0x07,0x07,0x07,0x07,0x07,0x0F,0x1F,
0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xF8,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,

0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x80,0xC0,0xC0,0xE0,0xE0,0xE0,
0xE0,0xC0,0x80,0x00,0x00,0x00,0x00,0x00,0xF0,0xFC,0xFE,0xFF,0xFF,0xFF,0xFF,0xFF,
0x07,0x01,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x80,0xC0,0xF0,
0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0x7F,0x1F,0x07,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,

0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0xE0,0xF8,0xFC,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
0xFF,0xFF,0xFF,0x7F,0x00,0x00,0x00,0x1F,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
0xFE,0xF8,0xF8,0xF8,0xF8,0xF8,0xF8,0xF8,0xF8,0xFC,0xFC,0xFE,0xFF,0xFF,0xFF,0xFF,
0xFF,0xFF,0xFF,0xFF,0xFF,0x7F,0x3F,0x1F,0x0F,0x03,0x01,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,

0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x03,0x0F,0x3F,0x3F,0x3F,0x7F,0x3F,0x3F,0x3F,0x1F,0x1F,0x0F,
0x07,0x03,0x01,0x00,0x00,0x00,0x00,0x00,0x00,0x03,0x07,0x0F,0x1F,0x1F,0x3F,0x3F,
0x3F,0x3F,0x3F,0x7F,0x7F,0x3F,0x3F,0x3F,0x3F,0x3F,0x1F,0x1F,0x1F,0x0F,0x0F,0x07,
0x07,0x03,0x01,0x01,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,

0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xFF,0xFF,0xFF,0x80,0xC0,0xC0,0xC0,
0xC0,0x80,0x80,0x00,0x00,0x00,0x00,0x00,0x00,0x80,0x80,0xC0,0xC0,0xC0,0x80,0x80,
0x00,0x00,0x00,0x00,0x80,0xC0,0xC0,0xC0,0x80,0xC0,0xC0,0xC0,0x80,0x80,0x00,0x00,
0x80,0xC0,0xC0,0xC0,0xC0,0x80,0x00,0x00,0x00,0x00,0x00,0x00,0x80,0x80,0xC0,0xC0,
0xC0,0xC0,0x80,0x80,0x00,0x00,0x00,0xC0,0xC0,0xF8,0xF8,0xF8,0xC0,0xC0,0xC0,0x00,
0x00,0x00,0x00,0x80,0x80,0xC0,0xC0,0xC0,0x80,0x80,0x00,0x00,0x00,0x00,0x00,0x00,
0x80,0x80,0xC0,0xC0,0xC0,0xC0,0x80,0x80,0x00,0x00,0x00,0xFF,0xFF,0xFF,0xFF,0x80,
0xC0,0xC0,0xC0,0xC0,0x80,0x80,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,

0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x40,0xFF,0xFF,0xFF,0x07,0x03,0x03,0x03,
0x07,0xFF,0xFF,0xFF,0xF8,0x00,0xF8,0xFE,0xFF,0xFF,0x77,0x73,0x73,0x73,0x77,0x7F,
0x7F,0x7E,0x00,0x00,0xFF,0xFF,0xFF,0xFF,0x03,0x03,0x03,0x03,0xFF,0xFF,0xFF,0xFF,
0x07,0x03,0x03,0x03,0x07,0xFF,0xFF,0xFE,0xF0,0x00,0xC0,0xE3,0xE7,0xF7,0x73,0x73,
0x73,0x73,0xFF,0xFF,0xFF,0xFC,0x00,0x03,0x03,0xFF,0xFF,0xFF,0x03,0x03,0x03,0x00,
0xF8,0xFE,0xFF,0xFF,0x77,0x73,0x73,0x73,0x7F,0x7F,0x7F,0x7E,0x00,0x00,0xFC,0xFF,
0xFF,0xFF,0x03,0x03,0x03,0x03,0x0F,0x07,0x07,0x06,0x00,0xFF,0xFF,0xFF,0xFF,0x07,
0x03,0x03,0x03,0x07,0xFF,0xFF,0xFE,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,

0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x07,0x0F,0x1F,0x1E,0x1C,0x3C,0x3C,
0x1E,0x1F,0x0F,0x07,0x00,0x00,0x00,0x07,0x0F,0x1F,0x1E,0x3C,0x3C,0x3C,0x1E,0x1F,
0x0E,0x06,0x00,0x00,0x1F,0x1F,0x1F,0x1F,0x00,0x00,0x00,0x00,0x1F,0x1F,0x1F,0x1F,
0x00,0x00,0x00,0x00,0x00,0x1F,0x1F,0x1F,0x1F,0x02,0x0F,0x1F,0x1F,0x3C,0x3C,0x3C,
0x1C,0x1E,0x1F,0x1F,0x1F,0x1F,0x00,0x00,0x00,0x0F,0x1F,0x1F,0x3C,0x3C,0x3C,0x00,
0x01,0x07,0x0F,0x1F,0x1E,0x3C,0x3C,0x3C,0x1E,0x1F,0x0E,0x06,0x00,0x00,0x03,0x07,
0x0F,0x1F,0x1C,0x3C,0x3C,0x1C,0x1F,0x1F,0x0E,0x02,0x00,0x1F,0x1F,0x1F,0x1F,0x00,
0x00,0x00,0x00,0x00,0x1F,0x1F,0x1F,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00
};                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                        

static void write_cmd_bytes(u8* cmd , u8 number)
{
	int ret;
	gpio_set_value(LCD_A0,0);
	udelay(100);	
    ret = spi_write_then_read(pmy_dev->spi_device, cmd, number, NULL, 0);
    if(ret)
		printk("send spi commends fail\n");
	
}                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                           

static void write_data_bytes(u8* data , u8 number)
{
	int ret;
	gpio_set_value(LCD_A0,1);
	udelay(100);
    ret = spi_write_then_read(pmy_dev->spi_device, data, number, NULL, 0);
    if(ret)
		printk("send spi data fail\n");
	
}                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                           

static void Write_Instruction(u8 code)
{
	int ret;
	
	//command set LCD_A0 = LOW
    gpio_set_value(LCD_A0,0);
    msleep(10);
    
    ret = spi_write_then_read(pmy_dev->spi_device, &code, 1, NULL, 0);
    if(ret)
		printk("send spi commend 0x%x fail\n",code);
}

//a(0-63) 32default   Vev=(1-(63-a)/162)Vref   2.1v
/*
static void Set_Contrast_Control_Register(u8 mod)
{
    Write_Instruction(0x81);
	Write_Instruction(mod);
}
*/
static void Set_Column_Address(unsigned char add)
{
    Write_Instruction((0x10|(add>>4)));
	Write_Instruction((0x0f&add));
}

// Set page address 0~8
static void Set_Page_Address(unsigned char add)
{
    add=0xb0|add;
    Write_Instruction(add);    
}

//Specify DDRAM line for COM0 0~63
static void Initial_Dispay_Line(u8 line)
{
    line|=0x40;
    Write_Instruction(line);        
}
/*
static void Regulor_Resistor_Select(u8 code)
{
    code |= 0x20;
    Write_Instruction(code);        
}

static void power_control(u8 code)
{
    code |= 0x28;
    Write_Instruction(code);
}
*/
void Write_Data(unsigned char data)
{

    gpio_set_value(LCD_A0,1);
    //msleep(10);
    Write_Instruction(data);    
}

#define Total_Page_Num 	8
#define Column_Num 		128

 
u8 last_picture[1024];

void Display_Picture(unsigned char pic[])
{
    int i;
    static int Initial_Dispay_Picture = 0;
    //u8 data[Column_Num];
    Initial_Dispay_Line(0);
    if (Initial_Dispay_Picture == 0)
    {        
        //show bematech log once only on bring up machine
        for(i=0;i<Total_Page_Num;i++)
        {
            Set_Page_Address(i);
            Set_Column_Address(0);
            //memcpy(data,&pic[i*Column_Num],Column_Num);
            //write_data_bytes(data,Column_Num);					            
            write_data_bytes(&pic[i*Column_Num],Column_Num);
        }        
    }
    else
    {   
        for(i=0;i<Total_Page_Num;i++)
        {
            /*compare the content of a page from first page to last one picture, if identical, not
             * update data and compare to next page, if any data different in page, update all page.
            */
            if (memcmp(&last_picture[i*Column_Num], &pic[i*Column_Num],Column_Num )!=0)
            {
                Set_Page_Address(i);
                Set_Column_Address(0);
                //memcpy(data,&pic[i*Column_Num],Column_Num);
                //write_data_bytes(data,Column_Num);
                write_data_bytes(&pic[i*Column_Num],Column_Num);
            }            	   
        }
    }
    memcpy(last_picture, pic, 1024);        
    Initial_Dispay_Picture = 1;

}

static ssize_t user_write(struct file *filp, const char *buf, size_t count, loff_t *ppos)
{

    u8 picture[1024];
    int ret;

    dprint("user write\n");    
    ret = copy_from_user(picture,buf,count);
    if(ret)
        printk("received data from user space not correct\n");
    
    Display_Picture(picture);  
    
    return count;
}

static int user_open(struct inode *inode, struct file *filp)
{
	struct my_dev* pdev = container_of(inode->i_cdev,struct my_dev, cdev);

	filp->private_data = pdev;

	return 0;
}

static int user_release(struct inode *inode, struct file *filp)
{
	dprint("device close\n");
	return 0;
}

struct file_operations drv_fops =
{
	.open			=       user_open,	 
	.write			=       user_write,
	.release		=	    user_release,
};


static void clean_screen(void)
{
	int i;
	u8 data[] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
		
	for(i=0;i<Total_Page_Num;i++)
	{
		Set_Page_Address(i);
		Set_Column_Address(0);		
		write_data_bytes(data,sizeof(data));				
		write_data_bytes(data,sizeof(data));				
	}	
}

static void lcd_init(void)
{
    u8 cmd[]={0xA2,0xA0,0xC8,0x23,0x2F,0x81,0x1f,0x40,0x25,0x81,0x19};	

    RESET();
    gpio_set_value(LCD_RST,1);
    msleep(10);
    gpio_set_value(LCD_RST,0);
    msleep(10);
    gpio_set_value(LCD_RST,1);
    msleep(10);
    
	DISPLAY_OFF();
		
	write_cmd_bytes(cmd,sizeof(cmd));
	
	clean_screen();

	DISPLAY_ON();

	Display_Picture(bematech_log);
}
    
static int lcm_init(void) 
{
	int err;
        dprint("\n");
	pmy_dev = kzalloc(sizeof(struct my_dev),GFP_KERNEL);  // Allocate memory for private struct
	if(!pmy_dev)
		dprint("Bad Kmalloc\n");
	
	//Kernel assign major & minor number dynamically
	if(alloc_chrdev_region(&pmy_dev->dev_num,0,NUMBERS,DEVICE_NAME) < 0){
		dprint("can't register device number\n");
		return -1;
	}
	
	cdev_init(&pmy_dev->cdev,&drv_fops);
	pmy_dev->cdev.owner = THIS_MODULE;
	
	if(cdev_add(&pmy_dev->cdev,pmy_dev->dev_num,1))
	{
		dprint("Bad cdev\n");
		return 1;
	}

	pmy_dev->dev_class = class_create(THIS_MODULE,DEVICE_NAME); // create class put in /sysfs
	device_create(pmy_dev->dev_class,NULL,pmy_dev->dev_num,NULL,DEVICE_NAME);

	//create spi master
	pmy_dev->spi_master = NULL;
	pmy_dev->spi_master = spi_busnum_to_master(1);
	if(pmy_dev->spi_master == NULL){
		dprint("Request spi master 0 failed!!\n");
		return -1;
	}
	  
	//create spi device on spi1
	pmy_dev->spi_device = spi_new_device(pmy_dev->spi_master, my_spi_board);	
	put_device(&pmy_dev->spi_master->dev);
	if(!pmy_dev->spi_device){
		dprint("spi_new_device() returned NULL\n");
		return -1;
	}
	dprint("\n");
	//P/S = LOW for spi mode,D0~D5,EN and R/W need pull-low setting them in DTS.
	err = gpio_request_array(LCD_gpios,ARRAY_SIZE(LCD_gpios));
	if(err)
		dprint("allocate LCD gpios error\n");
	else	
		dprint("init success \n");
		
    lcd_init();        
    
    dprint("\n");
        // add gpio setting for cash drawer
/*    
    	err = gpio_request_array(cash_drawer,ARRAY_SIZE(cash_drawer));
	if(err)
		dprint("allocate cash_drawer gpios error\n");
	else	
		dprint("init cash_drawer success \n");

    dprint("\n");
    */
	return 0;
}

void turn_off_backlight(void)
{
        gpio_set_value(LCD_BACKLIGHT,0);
}

EXPORT_SYMBOL_GPL(turn_off_backlight);

void turn_on_backlight(void)
{
        gpio_set_value(LCD_BACKLIGHT,1);
}

EXPORT_SYMBOL_GPL(turn_on_backlight);


static void lcm_exit(void) {
  	
	device_destroy(pmy_dev->dev_class,pmy_dev->dev_num);  
	class_destroy(pmy_dev->dev_class); 
	cdev_del(&pmy_dev->cdev);		
	unregister_chrdev(pmy_dev->major_num, DEVICE_NAME);  	
	gpio_free_array(LCD_gpios,ARRAY_SIZE(LCD_gpios));  
	
	if (pmy_dev->spi_device) {
		device_del(&pmy_dev->spi_device->dev);
		kfree(pmy_dev->spi_device);
	}
  
	kfree(pmy_dev);	      
}

late_initcall(lcm_init);
module_exit(lcm_exit);
MODULE_LICENSE("GPL");
