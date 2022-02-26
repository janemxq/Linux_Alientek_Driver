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
#include <linux/input.h>
#include <linux/of_irq.h>
#include <linux/irq.h>
/***************************************************************
Copyright © ALIENTEK Co., Ltd. 1998-2029. All rights reserved.
文件名		: gpioencoder.c
作者	  	: 左忠凯
版本	   	: V1.0
描述	   	: 采用pinctrl和gpio子系统驱动encoder灯。
其他	   	: 无
论坛 	   	: www.openedv.com
日志	   	: 初版V1.0 2019/7/13 左忠凯创建
***************************************************************/
#define GPIOENCODER_CNT			1		  	/* 设备号个数 */
#define GPIOENCODER_NAME		"gpioencoder"	/* 名字 */
static int counter = 0;
static int aState;
static int aLastState; 
/* 中断IO描述结构体 */
struct irq_encoderdesc {
	int gpio;								/* gpio */
	int irqnum;								/* 中断号     */
	unsigned char value;					/* 按键对应的键值 */
	char name[10];							/* 名字 */
	irqreturn_t (*handler)(int, void *);	/* 中断服务函数 */
};
/* gpioencoder设备结构体 */
struct gpioencoder_dev{
	dev_t devid;			/* 设备号 	 */
	struct cdev cdev;		/* cdev 	*/
	struct class *class;	/* 类 		*/
	struct device *device;	/* 设备 	 */
	int major;				/* 主设备号	  */
	int minor;				/* 次设备号   */
	struct device_node	*nd; /* 设备节点 */
	int encoder_gpio[2];			/* 编码器所使用的GPIO编号		*/
	struct irq_encoderdesc irqencoderdesc;	/* 編碼器描述数组 */
	struct timer_list timer;/* 定义一个定时器*/
	unsigned char curkeynum;				/* 当前的按键号 */
	struct input_dev *inputdev;		/* input结构体 */
};

struct gpioencoder_dev gpioencoderdev;	/* pulse设备 */

/*
 * @description		: 打开设备
 * @param - inode 	: 传递给驱动的inode
 * @param - filp 	: 设备文件，file结构体有个叫做private_data的成员变量
 * 					  一般在open的时候将private_data指向设备结构体。
 * @return 			: 0 成功;其他 失败
 */
static int encoder_open(struct inode *inode, struct file *filp)
{
	printk("encoder_open!\r\n");
	filp->private_data = &gpioencoderdev; /* 设置私有数据 */
	return 0;
}
/* @description	: 定时器服务函数，用于按键消抖，定时器到了以后
 *				  再次读取按键值，如果按键还是处于按下状态就表示按键有效。
 * @param - arg	: 设备结构变量
 * @return 		: 无
 */
void timer_function(unsigned long arg)
{
	unsigned char value;
	unsigned char num;
	struct irq_encoderdesc *encoderdesc;
	struct gpioencoder_dev* dev = (struct gpioencoder_dev *)arg;

	aState = gpio_get_value(dev->encoder_gpio[1]); 	/* 读取IO值 */
	if(aState != aLastState)
	{ 						/* 产生脉冲了 */
		// if(aState !=gpio_get_value(dev->encoder_gpio[1]))
		// {
		// 	counter++;//正转
		// }else
		// {
		// 	counter--;//反转
		// }
		if(counter % 2000==0)
		{
		  printk("counter=%d....",counter);
		}
		counter++;
		aLastState=aState;
	} 
	// printk("定时中断进来了....\r\n");
}
/*
 * @description		: 从设备读取数据 
 * @param - filp 	: 要打开的设备文件(文件描述符)
 * @param - buf 	: 返回给用户空间的数据缓冲区
 * @param - cnt 	: 要读取的数据长度
 * @param - offt 	: 相对于文件首地址的偏移
 * @return 			: 读取的字节数，如果为负值，表示读取失败
 */
static ssize_t encoder_read(struct file *filp, char __user *buf, size_t cnt, loff_t *offt)
{
	int retvalue;
	unsigned char databuf[1];
	unsigned char encoderstat;
	struct gpioencoder_dev *dev = filp->private_data;

	// databuf[0]=(gpio_get_value(dev->encoder_gpio[1])<<1|gpio_get_value(dev->encoder_gpio[0]));
	databuf[0]=(gpio_get_value(dev->encoder_gpio[0]));
    // printk("gpio 0=%d 1=%d\r\n",gpio_get_value(dev->encoder_gpio[0]),gpio_get_value(dev->encoder_gpio[1]));
	retvalue = copy_to_user(buf, &databuf, 1);
	if(retvalue < 0) {
		printk("kernel write failed!\r\n");
		return -EFAULT;
	}


	///--------------读口的状态
	// unsigned char value;
	// unsigned char num;
	// struct irq_encoderdesc *encoderdesc;
	// struct gpioencoder_dev* dev = filp->private_data;

	// aState = gpio_get_value(dev->encoder_gpio[0]); 	/* 读取IO值 */

	// if(aState != aLastState)
	// { 						/* 产生脉冲了 */
	// 	// if(aState !=gpio_get_value(dev->encoder_gpio[1]))
	// 	// {
	// 	// 	counter++;//正转
	// 	// }else
	// 	// {
	// 	// 	counter--;//反转
	// 	// }
	// 	printk("counter=%d....\r\n",counter++);
	// 	aLastState=aState;
	// } 
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
static ssize_t encoder_write(struct file *filp, const char __user *buf, size_t cnt, loff_t *offt)
{
	return 0;
}

/*
 * @description		: 关闭/释放设备
 * @param - filp 	: 要关闭的设备文件(文件描述符)
 * @return 			: 0 成功;其他 失败
 */
static int encoder_release(struct inode *inode, struct file *filp)
{
	/* 删除定时器 */
	// del_timer_sync(&gpioencoderdev.timer);	/* 删除定时器 */
		
	/* 释放中断 */
	// free_irq(gpioencoderdev.irqencoderdesc.irqnum, &gpioencoderdev);
	// printk("釋放中断。。。。\r\n");
	return 0;
}

/* 设备操作函数 */
static struct file_operations gpioencoder_fops = {
	.owner = THIS_MODULE,
	.open = encoder_open,
	.read = encoder_read,
	.write = encoder_write,
	.release = 	encoder_release,
};
/* @description		: 中断服务函数，开启定时器，延时10ms，
 *				  	  定时器用于按键消抖。
 * @param - irq 	: 中断号 
 * @param - dev_id	: 设备结构。
 * @return 			: 中断执行结果
 */
static irqreturn_t encoder_handler(int irq, void *dev_id)
{
	struct gpioencoder_dev *dev = (struct gpioencoder_dev *)dev_id;

	dev->curkeynum = 0;
	dev->timer.data = (volatile long)dev_id;
	// mod_timer(&dev->timer, jiffies + usecs_to_jiffies(100));	/* 10ms定时 */
	timer_function(dev);
	// printk("当前计数: count=%d",counter++);
	return IRQ_RETVAL(IRQ_HANDLED);
}
/*
 * @description	: 驱动出口函数
 * @param 		: 无
 * @return 		: 无
 */
static int __init encoder_init(void)
{
	int ret = 0;
    char name[10];
	/* 设置encoder所使用的GPIO */
	/* 1、获取设备节点：gpioencoder */
	gpioencoderdev.nd = of_find_node_by_path("/gpiorotaryencode");
	if(gpioencoderdev.nd == NULL) {
		printk("gpiorotaryencode node not find!\r\n");
		return -EINVAL;
	} else {
		printk("gpiorotaryencode node find!\r\n");
	}

	/* 2、 获取设备树中的gpio属性，得到encoder所使用的gpio编号 */
	gpioencoderdev.encoder_gpio[0] = of_get_named_gpio(gpioencoderdev.nd, "rotary_encode-gpio", 0);//A
	if(gpioencoderdev.encoder_gpio[0] < 0) {
		printk("can't get rotary_encode-gpio 0");
		return -EINVAL;
	}
	printk("rotary_encode-gpio 0 num = %d\r\n", gpioencoderdev.encoder_gpio[0]);

    gpioencoderdev.encoder_gpio[1] = of_get_named_gpio(gpioencoderdev.nd, "rotary_encode-gpio", 1);//B
	if(gpioencoderdev.encoder_gpio[1] < 0) {
		printk("can't get rotary_encode-gpio 1");
		return -EINVAL;
	}
	printk("rotary_encode-gpio 1 num = %d\r\n", gpioencoderdev.encoder_gpio[1]);
	/* 3、设置A B输入 */
	ret = gpio_direction_input(gpioencoderdev.encoder_gpio[0]);
	if(ret < 0) {
		printk("can't set gpio!\r\n");
	}
    ret = gpio_direction_input(gpioencoderdev.encoder_gpio[1]);
	if(ret < 0) {
		printk("can't set gpio!\r\n");
	}
	
	/* 初始化编码器所使用的IO，并且设置成中断模式 */
	memset(gpioencoderdev.irqencoderdesc.name, 0, sizeof(name));	/* 缓冲区清零 */
	sprintf(gpioencoderdev.irqencoderdesc.name, "ENCODER");		/* 组合名字 */
	gpio_request(gpioencoderdev.encoder_gpio[1], name);//1:B  接了电容  0:A 没接电容
	gpio_direction_input(gpioencoderdev.encoder_gpio[1]);	
	gpioencoderdev.irqencoderdesc.irqnum = gpio_to_irq(gpioencoderdev.encoder_gpio[1]);
	/* 申请中断 */
	gpioencoderdev.irqencoderdesc.handler = encoder_handler;
	gpioencoderdev.irqencoderdesc.value = REL_X;
	ret = request_irq(gpioencoderdev.irqencoderdesc.irqnum, gpioencoderdev.irqencoderdesc.handler, 
		                IRQF_TRIGGER_RISING|IRQF_TRIGGER_FALLING, gpioencoderdev.irqencoderdesc.name, &gpioencoderdev);
	if(ret < 0){
		printk("irq %d request failed!\r\n", gpioencoderdev.irqencoderdesc.irqnum);
		return -EFAULT;
	}
    /* 创建定时器 */
	init_timer(&gpioencoderdev.timer);
	gpioencoderdev.timer.function = timer_function;
	/* 注册字符设备驱动 */
	/* 1、创建设备号 */
	if (gpioencoderdev.major) {		/*  定义了设备号 */
		gpioencoderdev.devid = MKDEV(gpioencoderdev.major, 0);
		register_chrdev_region(gpioencoderdev.devid, GPIOENCODER_CNT, GPIOENCODER_NAME);
	} else {						/* 没有定义设备号 */
		alloc_chrdev_region(&gpioencoderdev.devid, 0, GPIOENCODER_CNT, GPIOENCODER_NAME);	/* 申请设备号 */
		gpioencoderdev.major = MAJOR(gpioencoderdev.devid);	/* 获取分配号的主设备号 */
		gpioencoderdev.minor = MINOR(gpioencoderdev.devid);	/* 获取分配号的次设备号 */
	}
	printk("gpioencoderdev major=%d,minor=%d\r\n",gpioencoderdev.major, gpioencoderdev.minor);	
	
	/* 2、初始化cdev */
	gpioencoderdev.cdev.owner = THIS_MODULE;
	cdev_init(&gpioencoderdev.cdev, &gpioencoder_fops);
	
	/* 3、添加一个cdev */
	cdev_add(&gpioencoderdev.cdev, gpioencoderdev.devid, GPIOENCODER_CNT);

	/* 4、创建类 */
	gpioencoderdev.class = class_create(THIS_MODULE, GPIOENCODER_NAME);
	if (IS_ERR(gpioencoderdev.class)) {
		return PTR_ERR(gpioencoderdev.class);
	}

	/* 5、创建设备 */
	gpioencoderdev.device = device_create(gpioencoderdev.class, NULL, gpioencoderdev.devid, NULL, GPIOENCODER_NAME);
	if (IS_ERR(gpioencoderdev.device)) {
		return PTR_ERR(gpioencoderdev.device);
	}
	return 0;
}

/*
 * @description	: 驱动出口函数
 * @param 		: 无
 * @return 		: 无
 */
static void __exit encoder_exit(void)
{
	free_irq(gpioencoderdev.irqencoderdesc.irqnum, &gpioencoderdev);
	/* 删除定时器 */
	del_timer_sync(&gpioencoderdev.timer);	/* 删除定时器 */
	printk("釋放中断。。。。\r\n");
	/* 注销字符设备驱动 */
	cdev_del(&gpioencoderdev.cdev);/*  删除cdev */
	unregister_chrdev_region(gpioencoderdev.devid, GPIOENCODER_CNT); /* 注销设备号 */

	device_destroy(gpioencoderdev.class, gpioencoderdev.devid);
	class_destroy(gpioencoderdev.class);
}

module_init(encoder_init);
module_exit(encoder_exit);
MODULE_LICENSE("GPL");
MODULE_AUTHOR("mxq");
