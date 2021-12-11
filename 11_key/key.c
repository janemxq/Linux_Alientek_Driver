#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/ide.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/errno.h>
#include <linux/gpio.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_gpio.h>
#include <linux/semaphore.h>
#include <asm/mach/map.h>
#include <asm/uaccess.h>
#include <asm/io.h>
#include "ads1118.h"
/***************************************************************
Copyright © ALIENTEK Co., Ltd. 1998-2029. All rights reserved.
文件名		: key.c
作者	  	: 左忠凯
版本	   	: V1.0
描述	   	: Linux按键输入驱动实验
其他	   	: 无
论坛 	   	: www.openedv.com
日志	   	: 初版V1.0 2019/7/18 左忠凯创建
***************************************************************/
#define KEY_CNT			1		/* 设备号个数 	*/
#define KEY_NAME		"key"	/* 名字 		*/

/* 定义按键值 */
#define KEY0VALUE		0XF0	/* 按键值 		*/
#define INVAKEY			0X00	/* 无效的按键值  */

/* key设备结构体 */
struct key_dev{
	dev_t devid;			/* 设备号 	 */
	struct cdev cdev;		/* cdev 	*/
	struct class *class;	/* 类 		*/
	struct device *device;	/* 设备 	 */
	int major;				/* 主设备号	  */
	int minor;				/* 次设备号   */
	struct device_node	*nd; /* 设备节点 */
	int key_gpio;			/* key所使用的GPIO编号		*/
	int cs_gpio;				/* 片选所使用的GPIO编号		*/
	int sclk_gpio;				/* 片选所使用的GPIO编号		*/
	int mosi_gpio;				/* 片选所使用的GPIO编号		*/
	int miso_gpio;				/* 片选所使用的GPIO编号		*/
	atomic_t keyvalue;		/* 按键值 		*/	
};

struct key_dev keydev;		/* key设备 */
/*******************************************************************************
//函数名称：Write_ADS1118（）
//函数功能：设置1118寄存器
//输    入：config:寄存器配置
//          discardData: 是否更新读回数据 0读回 1不读回
//返    回： 16位ad转换数据
//备    注： 
*******************************************************************************/
int16_t Write_ADS1118(struct key_dev *keydev,uint16_t config,uint8_t discardData)
{
	uint8_t i=0;
  static int16_t read=0;
	CS_L(keydev->cs_gpio);
	if(discardData==0) 
	{
		read=0;
	}
	// delay_us(1);
	udelay(1);
	 for(i=0;i<16;i++)     // 循环16次 
	{		
		if(config & 0x8000)MOSI_H(keydev->mosi_gpio);
		else MOSI_L(keydev->mosi_gpio);
		config <<= 1;

		udelay(1);
		SCLK_H(keydev->sclk_gpio);
		udelay(1);
		SCLK_L(keydev->sclk_gpio);
		udelay(1);
		
		if(discardData==0)
		{
			read<<=1;
			if(READ_MISO(keydev->miso_gpio)==1)read ++;
				
		}else{
			udelay(2);
		}
	}
	CS_H(keydev->cs_gpio);
	udelay(2);
	CS_L(keydev->cs_gpio);
//	SCLK_L;
//	MOSI_L;
//	MISO_H;
	return read;
}
/*******************************************************************************
//函数名称：ADS_SEL_Read（）
//函数功能：读取各路电压，通过两个switch选择读取不同的通道
//输    入：road:增益放大器两端的电压选择，并选择测几路电压
//          Ref: 选择参考电压，有6种选择
//          mode: 是否更新读回数据 0读回 1不读回
//输    出：dat：16位ad转换数据
//备    注：这一次读出的转换数据是上一次的转换数据，不要混淆.这里选择的是单次
            转换电压值，当然，也可以选择多次转换,通过寄存器的第8位可以设置
*******************************************************************************/
int16_t ADS_SEL_Read(struct key_dev *keydev,uint8_t road,uint8_t Ref,uint8_t mode)         //测几路电压
{
    int dat = 0;
		uint16_t config = ADS1118_CONFIG_SS_START_OFF |  // 
                    ADS1118_CONFIG_MODE_CONTIN  |  //连续转换模式 
                    ADS1118_CONFIG_DR_128SPS   |  // 转换 速率 SPS
                    ADS1118_CONFIG_TS_MODE_ADC |  // 读取ADC，而不是温度
                    ADS1118_CONFIG_PULL_UP_OFF  |  //上拉电阻启用
                    ADS1118_CONFIG_NOP_VALID   |  // this is valid data (default)
                    ADS1118_CONFIG_RESV;          // reserved bits must be set to 1
	if(road == 8)config|=ADS1118_CONFIG_TS_MODE_TEMP;
	else config|=ADS1118_CONFIG_TS_MODE_ADC;
    switch(road)	   //选择有效转换的路数
    {
			case 0:  config |= ADS1118_CONFIG_MUX_DIFF_0_1;break;    //AINP = AIN0 and AINN = AIN1 (default)
			case 1:  config |= ADS1118_CONFIG_MUX_DIFF_0_3;break;    //AINP = AIN0 and AINN = AIN3
			case 2:  config |= ADS1118_CONFIG_MUX_DIFF_1_3;break;    //AINP = AIN1 and AINN = AIN3
			case 3:  config |= ADS1118_CONFIG_MUX_DIFF_2_3;break;    //AINP = AIN2 and AINN = AIN3
			case 4:  config |= ADS1118_CONFIG_MUX_SINGLE_0;break;    //AINP = AIN0 and AINN = GND
			case 5:  config |= ADS1118_CONFIG_MUX_SINGLE_1;break;    //AINP = AIN1 and AINN = GND
			case 6:  config |= ADS1118_CONFIG_MUX_SINGLE_2;break;    //AINP = AIN2 and AINN = GND
			case 7:  config |= ADS1118_CONFIG_MUX_SINGLE_3;break;    //AINP = AIN3 and AINN = GND
			default : break;
    }
    switch(Ref)
    {
			case 0:  config |= ADS1118_CONFIG_PGA_6_144V;break;    //000 : FS = ±6.144V(1)
			case 1:  config |= ADS1118_CONFIG_PGA_4_096V;break;    //001 : FS = ±4.096V(1)
			case 2:  config |= ADS1118_CONFIG_PGA_2_048V;break;    //002 : FS = ±2.048V(1)
			case 3:  config |= ADS1118_CONFIG_PGA_1_024V;break;    //003 : FS = ±1.024V(1)
			case 4:  config |= ADS1118_CONFIG_PGA_0_512V;break;    //004 : FS = ±0.512V(1)
			case 5: case 6: case 7: config |= ADS1118_CONFIG_PGA_0_256V;break;    //005 : FS = ±0.256V(1)
			default : break;
    }
	// if(mode)
	// {
	//   printk("config=%X\r\n",config);
	// }
    dat = Write_ADS1118(keydev,config,mode);
    return dat;
}
void ADS1118_Init(struct key_dev *keydev)	   //ADS1118初始化
{
	uint16_t config = ADS1118_CONFIG_SS_START_OFF |  // 
                    ADS1118_CONFIG_MODE_CONTIN  |  //连续转换模式 
                    ADS1118_CONFIG_DR_128SPS   |  // 转换 速率 SPS
                    ADS1118_CONFIG_TS_MODE_ADC |  // 读取ADC，而不是温度
                    ADS1118_CONFIG_PULL_UP_ON  |  //上拉电阻启用
                    ADS1118_CONFIG_NOP_VALID   |  // this is valid data (default)
										ADS1118_CONFIG_PGA_6_144V  | // FSR is ±6.144 V
										ADS1118_CONFIG_MUX_SINGLE_0|  //AINP is AIN0 and AINN is GND//单端
                    ADS1118_CONFIG_RESV;          // reserved bits must be set to 1
	
		
	CS_H(keydev->cs_gpio);
	// MISO_H(keydev->miso_gpio);
	SCLK_L(keydev->sclk_gpio);
	MOSI_L(keydev->mosi_gpio);
	
	Write_ADS1118(keydev,config,1);//设置ads1118
}

/*
 * @description	: 初始化按键IO，open函数打开驱动的时候
 * 				  初始化按键所使用的GPIO引脚。
 * @param 		: 无
 * @return 		: 无
 */
static int keyio_init(void)
{
	keydev.nd = of_find_node_by_path("/key");
	if (keydev.nd== NULL) {
		return -EINVAL;
	}

	keydev.key_gpio = of_get_named_gpio(keydev.nd ,"key-gpio", 0);
	if (keydev.key_gpio < 0) {
		printk("can't get key0\r\n");
		return -EINVAL;
	}
	printk("key_gpio=%d\r\n", keydev.key_gpio);
	keydev.cs_gpio = of_get_named_gpio(keydev.nd, "cs-gpio", 0);
	if(keydev.cs_gpio < 0) {
		printk("can't get cs-gpio");
		return -EINVAL;
	}
	keydev.miso_gpio = of_get_named_gpio(keydev.nd, "miso-gpio", 0);
	if(keydev.miso_gpio < 0) {
		printk("can't get miso_gpio");
		return -EINVAL;
	}
	keydev.mosi_gpio = of_get_named_gpio(keydev.nd, "mosi-gpio", 0);
	if(keydev.mosi_gpio < 0) {
		printk("can't get mosi_gpio");
		return -EINVAL;
	}
	keydev.sclk_gpio = of_get_named_gpio(keydev.nd, "sclk-gpio", 0);
	if(keydev.sclk_gpio < 0) {
		printk("can't get sclk_gpio");
		return -EINVAL;
	}
	/* 初始化key所使用的IO */
	gpio_request(keydev.key_gpio, "key0");	/* 请求IO */
	gpio_direction_input(keydev.key_gpio);	/* 设置为输入 */
    gpio_request(keydev.cs_gpio, "cs-gpio");	/* 请求IO */
	gpio_direction_output(keydev.cs_gpio,1);	/* 设置为输出 */
	gpio_direction_output(keydev.mosi_gpio,1);	/* 设置为输出 */
	gpio_direction_output(keydev.sclk_gpio,1);	/* 设置为输出 */
	gpio_direction_input(keydev.miso_gpio);	/* 设置为输入 */
	printk("keydev.cs_gpio =%d\r\n", keydev.cs_gpio);
	printk("keydev.mosi_gpio =%d\r\n", keydev.mosi_gpio);
	printk("keydev.miso_gpio =%d\r\n", keydev.miso_gpio);
	printk("keydev.sclk_gpio =%d\r\n", keydev.sclk_gpio);
	ADS1118_Init(&keydev);//ADS1118初始化
	return 0;
}

/*
 * @description		: 打开设备
 * @param - inode 	: 传递给驱动的inode
 * @param - filp 	: 设备文件，file结构体有个叫做private_data的成员变量
 * 					  一般在open的时候将private_data指向设备结构体。
 * @return 			: 0 成功;其他 失败
 */
static int key_open(struct inode *inode, struct file *filp)
{
	int ret = 0;
	filp->private_data = &keydev; 	/* 设置私有数据 */

	ret = keyio_init();				/* 初始化按键IO */
	if (ret < 0) {
		return ret;
	}

	return 0;
}
static int ads1118_test(struct key_dev *keydev)
{
	int channel=0;
	int16_t value[5];
	for (channel = 0; channel <5;)
	{
		if(READ_MISO(keydev->miso_gpio)==0)
		{
			value[channel]=ADS_SEL_Read(keydev,channel+4,0,0); //读取通道
			if(channel==4) 
			{
				ADS_SEL_Read(keydev,4,0,1);//配置下一通道
				udelay(100);
				ADS_SEL_Read(keydev,4,0,1);//配置下一通道1
			}
			else
			{
				ADS_SEL_Read(keydev,channel+5,0,1);  //配置下一通道
				udelay(100);
				ADS_SEL_Read(keydev,channel+5,0,1);  //配置下一通道
			} 
			channel++;
		}
		mdelay(100);
		// printk(" channel1%02d  value:%X \n",channel,value[channel]);
	}
	if(channel==5)
	{
       printk(" channel1-5 :  %04X %04X %04X %04X %04X\n",value[0],value[1],value[2],value[3],value[4]);
	}
	return 0;
}
static int key_testIO(struct key_dev *keydev)
{
	int i;
	int ret;
	for (i = 0; i <100;i++) 
	{
      gpio_set_value(keydev->cs_gpio, 0);			/* 片选拉低 */
      gpio_set_value(keydev->mosi_gpio, 0);
      gpio_set_value(keydev->sclk_gpio, 0);
	   mdelay(10);
	  gpio_set_value(keydev->cs_gpio, 1);			/* 片选拉低 */
	  gpio_set_value(keydev->mosi_gpio, 1);
	  gpio_set_value(keydev->sclk_gpio, 1);

	}
	printk("key_testIO dev->key_gpio=%d\n", gpio_get_value(keydev->miso_gpio));
	return 1;
}
/*
 * @description		: 从设备读取数据 
 * @param - filp 	: 要打开的设备文件(文件描述符)
 * @param - buf 	: 返回给用户空间的数据缓冲区
 * @param - cnt 	: 要读取的数据长度
 * @param - offt 	: 相对于文件首地址的偏移
 * @return 			: 读取的字节数，如果为负值，表示读取失败
 */
static ssize_t key_read(struct file *filp, char __user *buf, size_t cnt, loff_t *offt)
{
	int ret = 0;
	int value;
	struct key_dev *dev = filp->private_data;

	// if (gpio_get_value(dev->key_gpio) == 0) { 		/* key0按下 */
	// 	while(!gpio_get_value(dev->key_gpio));		/* 等待按键释放 */
	// 	atomic_set(&dev->keyvalue, KEY0VALUE);	
	// } else {	
	// 	atomic_set(&dev->keyvalue, INVAKEY);		/* 无效的按键值 */
	// }

	// value = atomic_read(&dev->keyvalue);
	// ret = copy_to_user(buf, &value, sizeof(value));
	// key_testIO(dev);
    ads1118_test(dev);
	return ret;
}

/*
 * @description		: 向设备写数据 
 * @param - filp 	: 设备文件，表示打开的文件描述符
 * @param - buf 	: 要写给设备写入的数据
 * @param - cnt 	: 要写入的数据长度
 * @param - offt 	: 相对于文件首地址的偏移
 * @return 			: 写入的字节数，如果为负值，表示写入失败
 */
static ssize_t key_write(struct file *filp, const char __user *buf, size_t cnt, loff_t *offt)
{
	return 0;
}

/*
 * @description		: 关闭/释放设备
 * @param - filp 	: 要关闭的设备文件(文件描述符)
 * @return 			: 0 成功;其他 失败
 */
static int key_release(struct inode *inode, struct file *filp)
{
	return 0;
}

/* 设备操作函数 */
static struct file_operations key_fops = {
	.owner = THIS_MODULE,
	.open = key_open,
	.read = key_read,
	.write = key_write,
	.release = 	key_release,
};

/*
 * @description	: 驱动入口函数
 * @param 		: 无
 * @return 		: 无
 */
static int __init mykey_init(void)
{
	/* 初始化原子变量 */
	atomic_set(&keydev.keyvalue, INVAKEY);

	/* 注册字符设备驱动 */
	/* 1、创建设备号 */
	if (keydev.major) {		/*  定义了设备号 */
		keydev.devid = MKDEV(keydev.major, 0);
		register_chrdev_region(keydev.devid, KEY_CNT, KEY_NAME);
	} else {						/* 没有定义设备号 */
		alloc_chrdev_region(&keydev.devid, 0, KEY_CNT, KEY_NAME);	/* 申请设备号 */
		keydev.major = MAJOR(keydev.devid);	/* 获取分配号的主设备号 */
		keydev.minor = MINOR(keydev.devid);	/* 获取分配号的次设备号 */
	}
	
	/* 2、初始化cdev */
	keydev.cdev.owner = THIS_MODULE;
	cdev_init(&keydev.cdev, &key_fops);
	
	/* 3、添加一个cdev */
	cdev_add(&keydev.cdev, keydev.devid, KEY_CNT);

	/* 4、创建类 */
	keydev.class = class_create(THIS_MODULE, KEY_NAME);
	if (IS_ERR(keydev.class)) {
		return PTR_ERR(keydev.class);
	}

	/* 5、创建设备 */
	keydev.device = device_create(keydev.class, NULL, keydev.devid, NULL, KEY_NAME);
	if (IS_ERR(keydev.device)) {
		return PTR_ERR(keydev.device);
	}
	
	return 0;
}

/*
 * @description	: 驱动出口函数
 * @param 		: 无
 * @return 		: 无
 */
static void __exit mykey_exit(void)
{
	/* 注销字符设备驱动 */
	cdev_del(&keydev.cdev);/*  删除cdev */
	unregister_chrdev_region(keydev.devid, KEY_CNT); /* 注销设备号 */

	device_destroy(keydev.class, keydev.devid);
	class_destroy(keydev.class);
}

module_init(mykey_init);
module_exit(mykey_exit);
MODULE_LICENSE("GPL");
MODULE_AUTHOR("zuozhongkai");
