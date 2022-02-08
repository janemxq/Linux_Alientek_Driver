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
#include <asm/mach/map.h>
#include <asm/uaccess.h>
#include <asm/io.h>
/***************************************************************
Copyright © ALIENTEK Co., Ltd. 1998-2029. All rights reserved.
文件名		: gpioled.c
作者	  	: 左忠凯
版本	   	: V1.0
描述	   	: 采用pinctrl和gpio子系统驱动LED灯。
其他	   	: 无
论坛 	   	: www.openedv.com
日志	   	: 初版V1.0 2019/7/13 左忠凯创建
***************************************************************/
#define GPIOLED_CNT			1		  	/* 设备号个数 */
#define GPIOLED_NAME		"gpiopulse"	/* 名字 */
#define LEDOFF 				0			/* 关灯 */
#define LEDON 				1			/* 开灯 */

/* gpioled设备结构体 */
struct gpiopulse_dev{
	dev_t devid;			/* 设备号 	 */
	struct cdev cdev;		/* cdev 	*/
	struct class *class;	/* 类 		*/
	struct device *device;	/* 设备 	 */
	int major;				/* 主设备号	  */
	int minor;				/* 次设备号   */
	struct device_node	*nd; /* 设备节点 */
	int pulse_gpio;			/* pulse所使用的GPIO编号		*/
};

struct gpiopulse_dev gpiopulse;	/* pulse设备 */

/*
 * @description		: 打开设备
 * @param - inode 	: 传递给驱动的inode
 * @param - filp 	: 设备文件，file结构体有个叫做private_data的成员变量
 * 					  一般在open的时候将private_data指向设备结构体。
 * @return 			: 0 成功;其他 失败
 */
static int led_open(struct inode *inode, struct file *filp)
{
	filp->private_data = &gpiopulse; /* 设置私有数据 */
	return 0;
}

/*
 * @description		: 从设备读取数据 
 * @param - filp 	: 要打开的设备文件(文件描述符)
 * @param - buf 	: 返回给用户空间的数据缓冲区
 * @param - cnt 	: 要读取的数据长度
 * @param - offt 	: 相对于文件首地址的偏移
 * @return 			: 读取的字节数，如果为负值，表示读取失败
 */
static ssize_t led_read(struct file *filp, char __user *buf, size_t cnt, loff_t *offt)
{
	return 0;
}

/*
 * @description		: 向设备写数据 
 * @param - filp 	: 设备文件，表示打开的文件描述符
 * @param - buf 	: 要写给设备写入的数据
 * @param - cnt 	: 要写入的数据长度
 * @param - offt 	: 相对于文件首地址的偏移
 * @return 			: 写入的字节数，如果为负值，表示写入失败
 */
static ssize_t led_write(struct file *filp, const char __user *buf, size_t cnt, loff_t *offt)
{
	int retvalue;
	unsigned char databuf[1];
	unsigned char ledstat;
	struct gpiopulse_dev *dev = filp->private_data;

	retvalue = copy_from_user(databuf, buf, cnt);
	if(retvalue < 0) {
		printk("kernel write failed!\r\n");
		return -EFAULT;
	}

	ledstat = databuf[0];		/* 获取状态值 */

	if(ledstat == LEDOFF) {	
		gpio_set_value(dev->pulse_gpio, 0);	/* */
	} else  {
		gpio_set_value(dev->pulse_gpio, 1);	/* */
	}
	return 0;
}

/*
 * @description		: 关闭/释放设备
 * @param - filp 	: 要关闭的设备文件(文件描述符)
 * @return 			: 0 成功;其他 失败
 */
static int led_release(struct inode *inode, struct file *filp)
{
	return 0;
}

/* 设备操作函数 */
static struct file_operations gpioled_fops = {
	.owner = THIS_MODULE,
	.open = led_open,
	.read = led_read,
	.write = led_write,
	.release = 	led_release,
};

/*
 * @description	: 驱动出口函数
 * @param 		: 无
 * @return 		: 无
 */
static int __init led_init(void)
{
	int ret = 0;

	/* 设置LED所使用的GPIO */
	/* 1、获取设备节点：gpioled */
	gpiopulse.nd = of_find_node_by_path("/gpiopulse");
	if(gpiopulse.nd == NULL) {
		printk("gpiopulse node not find!\r\n");
		return -EINVAL;
	} else {
		printk("gpiopulse node find!\r\n");
	}

	/* 2、 获取设备树中的gpio属性，得到LED所使用的LED编号 */
	gpiopulse.pulse_gpio = of_get_named_gpio(gpiopulse.nd, "pulse-gpio", 0);
	if(gpiopulse.pulse_gpio < 0) {
		printk("can't get pulse-gpio");
		return -EINVAL;
	}
	printk("pulse-gpio num = %d\r\n", gpiopulse.pulse_gpio);

	/* 3、设置GPIO1_IO03为输出，并且输出高电平，默认关闭LED灯 */
	ret = gpio_direction_output(gpiopulse.pulse_gpio, 1);
	if(ret < 0) {
		printk("can't set gpio!\r\n");
	}

	/* 注册字符设备驱动 */
	/* 1、创建设备号 */
	if (gpiopulse.major) {		/*  定义了设备号 */
		gpiopulse.devid = MKDEV(gpiopulse.major, 0);
		register_chrdev_region(gpiopulse.devid, GPIOLED_CNT, GPIOLED_NAME);
	} else {						/* 没有定义设备号 */
		alloc_chrdev_region(&gpiopulse.devid, 0, GPIOLED_CNT, GPIOLED_NAME);	/* 申请设备号 */
		gpiopulse.major = MAJOR(gpiopulse.devid);	/* 获取分配号的主设备号 */
		gpiopulse.minor = MINOR(gpiopulse.devid);	/* 获取分配号的次设备号 */
	}
	printk("gpiopulse major=%d,minor=%d\r\n",gpiopulse.major, gpiopulse.minor);	
	
	/* 2、初始化cdev */
	gpiopulse.cdev.owner = THIS_MODULE;
	cdev_init(&gpiopulse.cdev, &gpioled_fops);
	
	/* 3、添加一个cdev */
	cdev_add(&gpiopulse.cdev, gpiopulse.devid, GPIOLED_CNT);

	/* 4、创建类 */
	gpiopulse.class = class_create(THIS_MODULE, GPIOLED_NAME);
	if (IS_ERR(gpiopulse.class)) {
		return PTR_ERR(gpiopulse.class);
	}

	/* 5、创建设备 */
	gpiopulse.device = device_create(gpiopulse.class, NULL, gpiopulse.devid, NULL, GPIOLED_NAME);
	if (IS_ERR(gpiopulse.device)) {
		return PTR_ERR(gpiopulse.device);
	}
	return 0;
}

/*
 * @description	: 驱动出口函数
 * @param 		: 无
 * @return 		: 无
 */
static void __exit led_exit(void)
{
	/* 注销字符设备驱动 */
	cdev_del(&gpiopulse.cdev);/*  删除cdev */
	unregister_chrdev_region(gpiopulse.devid, GPIOLED_CNT); /* 注销设备号 */

	device_destroy(gpiopulse.class, gpiopulse.devid);
	class_destroy(gpiopulse.class);
}

module_init(led_init);
module_exit(led_exit);
MODULE_LICENSE("GPL");
MODULE_AUTHOR("mxq");
