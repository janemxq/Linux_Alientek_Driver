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
#include <linux/of_gpio.h>
#include <linux/semaphore.h>
#include <linux/timer.h>
#include <linux/i2c.h>
#include <asm/mach/map.h>
#include <asm/uaccess.h>
#include <asm/io.h>
#include "mcp23017reg.h"
/***************************************************************
Copyright © ALIENTEK Co., Ltd. 1998-2029. All rights reserved.
文件名		: mcp23017.c
作者	  	: 左忠凯
版本	   	: V1.0
描述	   	: mcp23017驱动程序
其他	   	: 无
论坛 	   	: www.openedv.com
日志	   	: 初版V1.0 2019/9/2 左忠凯创建
***************************************************************/
#define mcp23017_CNT	1
#define mcp23017_NAME	"mcp23017"

struct mcp23017_dev {
	dev_t devid;			/* 设备号 	 */
	struct cdev cdev;		/* cdev 	*/
	struct class *class;	/* 类 		*/
	struct device *device;	/* 设备 	 */
	struct device_node	*nd; /* 设备节点 */
	int major;			/* 主设备号 */
	void *private_data;	/* 私有数据 */
	unsigned short ir, als, ps;		/* 三个光传感器数据 */
};

static struct mcp23017_dev mcp23017dev;
/*
 * @description	: 从mcp23017读取多个寄存器数据
 * @param - dev:  mcp23017设备
 * @param - reg:  要读取的寄存器首地址
 * @param - val:  读取到的数据
 * @param - len:  要读取的数据长度
 * @return 		: 操作结果
 */
static int mcp23017_read_regs(struct mcp23017_dev *dev, u8 reg, void *val, int len)
{
	int ret;
	struct i2c_msg msg[2];
	struct i2c_client *client = (struct i2c_client *)dev->private_data;
    // printk("mcp23017_read_regs addr =%d \n",client->addr);
	/* msg[0]为发送要读取的首地址 */
	msg[0].addr = client->addr;			/* mcp23017地址 */
	msg[0].flags = 0;					/* 标记为发送数据 */
	msg[0].buf = &reg;					/* 读取的首地址 */
	msg[0].len = 1;						/* reg长度*/

	/* msg[1]读取数据 */
	msg[1].addr = client->addr;			/* mcp23017地址 */
	msg[1].flags = I2C_M_RD;			/* 标记为读取数据*/
	msg[1].buf = val;					/* 读取数据缓冲区 */
	msg[1].len = len;					/* 要读取的数据长度*/

	ret = i2c_transfer(client->adapter, msg, 2);
	if(ret == 2) {
		ret = 0;
	} else {
		printk("i2c rd failed=%d reg=%06x len=%d\n",ret, reg, len);
		ret = -EREMOTEIO;
	}
	return ret;
}

/*
 * @description	: 向mcp23017多个寄存器写入数据
 * @param - dev:  mcp23017设备
 * @param - reg:  要写入的寄存器首地址
 * @param - val:  要写入的数据缓冲区
 * @param - len:  要写入的数据长度
 * @return 	  :   操作结果
 */
static s32 mcp23017_write_regs(struct mcp23017_dev *dev, u8 reg, u8 *buf, u8 len)
{
	u8 b[256];
	struct i2c_msg msg;
	struct i2c_client *client = (struct i2c_client *)dev->private_data;
	// printk("mcp23017_write_regs addr =%d \n",client->addr);
	
	b[0] = reg;					/* 寄存器首地址 */
	memcpy(&b[1],buf,len);		/* 将要写入的数据拷贝到数组b里面 */
		
	msg.addr = client->addr;	/* mcp23017地址 */
	msg.flags = 0;				/* 标记为写数据 */

	msg.buf = b;				/* 要写入的数据缓冲区 */
	msg.len = len + 1;			/* 要写入的数据长度 */

	return i2c_transfer(client->adapter, &msg, 1);
}

/*
 * @description	: 读取mcp23017指定寄存器值，读取一个寄存器
 * @param - dev:  mcp23017设备
 * @param - reg:  要读取的寄存器
 * @return 	  :   读取到的寄存器值
 */
static unsigned char mcp23017_read_reg(struct mcp23017_dev *dev, u8 reg)
{
	u8 data = 0;

	mcp23017_read_regs(dev, reg, &data, 1);
	return data;

#if 0
	struct i2c_client *client = (struct i2c_client *)dev->private_data;
	return i2c_smbus_read_byte_data(client, reg);
#endif
}

/*
 * @description	: 向mcp23017指定寄存器写入指定的值，写一个寄存器
 * @param - dev:  mcp23017设备
 * @param - reg:  要写的寄存器
 * @param - data: 要写入的值
 * @return   :    无
 */
static void mcp23017_write_reg(struct mcp23017_dev *dev, u8 reg, u8 data)
{
	u8 buf = 0;
	buf = data;
	mcp23017_write_regs(dev, reg, &buf, 1);
}
//--------------------------------------------------------------------------------------   
//  函数名称：bit MCP23017_INIT(unsigned char deviceAddr,unsigned char ab,unsigned char hwa,unsigned char o)
//  函数功能：初始化指定地址的MCP23017器件
//	参数：deviceAddr——设备地址，有A0，A1，A2决定
//		  intab——配置INTA、INTB是否关联，取值INTA_INTB_CONJUNCTION、INTA_INTB_INDEPENDENT
//		  hwa——配置A0、A1、A2硬件地址是否使能，取值HWA_EN、HWA_DIS	
//		  o——配置INTA、INTB的输出类型，取值INT_OD、INT_PUSHPULL_HIGH、INT_PUSHPULL_LOW 
//--------------------------------------------------------------------------------------   
static unsigned char MCP23017_INIT(struct mcp23017_dev *dev,unsigned char intab,unsigned char hwa,unsigned char o)
{
	unsigned char state;
	unsigned char res;
	
	//首先设置其他位的默认状态
	state = 0x2E;		//001011 10,BANK = 0,默认不关联AB（bit = 0）,禁用顺序操作,使能变化率控制、使能硬件地址,开漏输出
	if(intab==INTA_INTB_CONJUNCTION)
	{
		state |= 0x40;
	}
	if(hwa==HWA_DIS)
	{
		state &= (~0x08);
	}
	if(o==INT_PUSHPULL_HIGH)
	{
		state &= (~0x04);
		state |= 0x02;
	}
	if(o==INT_PUSHPULL_LOW)
	{
		state &= (~0x04);
		state &= (~0x02);
	}
	
	//写回方向寄存器
	mcp23017_write_reg(dev,MCP23017_IOCON,state);
	
	return 0;
}
//--------------------------------------------------------------------------------------   
//  函数名称：unsigned char MCP23017_IO_PU(unsigned char deviceAddr,unsigned char port,unsigned char pin,unsigned char pu)
//  函数功能：设置指定地址的MCP23017的指定端口的指定引脚是否开启上拉电阻
//	参数：deviceAddr——设备地址，有A0，A1，A2决定
//		  port——端口名称，取值MCP23017_PORTA、MCP23017_PORTB
//		  pin——引脚号，取值PIN0-PIN7对应端口的8个引脚,ALLPIN包括端口所有8个引脚
//		  pull——输入输出方向，取值ENABLE、DISABLE 
//--------------------------------------------------------------------------------------   
static unsigned char MCP23017_IO_PU(struct mcp23017_dev *dev,unsigned char port,unsigned char pin,unsigned char pu)
{
	unsigned char *portState;
	unsigned char res;
	
	//首先读取当前端口方向的配置状态
	//因为B端口的地址比A端口的寄存器的地址都是大1，所以采用+的技巧切换寄存器
	res = mcp23017_read_reg(dev,MCP23017_GPPU+port);

	//如果出错则返回
	if(pu==ENABLE)
	{
		res |= pin;
	}
	else
	{
		res &= (~pin);
	}

	//写回方向寄存器
	mcp23017_write_reg(&mcp23017dev,MCP23017_GPPU+port,res);
	
	return 0;
}

//--------------------------------------------------------------------------------------   
//  函数名称：MCP23017_IO_DIR(unsigned char deviceAddr,unsigned char port,unsigned char pin,unsigned char dir)
//  函数功能：设置制定地址的MCP23017的指定端口的指定引脚为输入或输出状态
//	参数：deviceAddr——设备地址，有A0，A1，A2决定
//		  port——端口名称，取值MCP23017_PORTA、MCP23017_PORTB
//		  pin——引脚号，取值PIN0-PIN7对应端口的8个引脚,ALLPIN包括端口所有8个引脚
//		  dir——输入输出方向，取值INPUT、OUTPUT 
//--------------------------------------------------------------------------------------   
static unsigned char MCP23017_IO_DIR(struct mcp23017_dev *dev,unsigned char port,unsigned char pin,unsigned char dir)
{
	unsigned char *portState;
	static unsigned char res;
	
	//首先读取当前端口方向的配置状态
	//因为B端口的地址比A端口的寄存器的地址都是大1，所以采用+的技巧切换寄存器
	res = mcp23017_read_reg(dev,MCP23017_IODIR+port);

	// //如果出错则返回
	// if(res == 0)
	// {
	// 	return res;
	// }

	if(dir==INPUT)
	{
		res |= pin;
	}
	else
	{
		res &= (~pin);
	}

	//写回方向寄存器
	mcp23017_write_reg(dev,MCP23017_IODIR+port,res);
	
	return res;
}
//--------------------------------------------------------------------------------------   
//  函数名称：unsigned char MCP23017_READ_GPIO(unsigned char deviceAddr,unsigned char port)
//  函数功能：读取指定地址的MCP23017的指定端口值
//	参数：deviceAddr——设备地址，有A0，A1，A2决定
//		  port——端口名称，取值MCP23017_PORTA、MCP23017_PORTB
//	返回值：中断发生时，当时端口的值
//--------------------------------------------------------------------------------------   
static unsigned char MCP23017_READ_GPIO(struct mcp23017_dev *dev,unsigned char port)
{
	//首先读取当前端口状态
	//因为B端口的地址比A端口的寄存器的地址都是大1，所以采用+的技巧切换寄存器
	return mcp23017_read_reg(dev,MCP23017_GPIO+port);
}
//--------------------------------------------------------------------------------------   
//  函数名称：unsigned char MCP23017_WRITE_GPIO(unsigned char deviceAddr,unsigned char port,unsigned char val)
//  函数功能：向指定地址的MCP23017的指定端口写值
//	参数：deviceAddr——设备地址，有A0，A1，A2决定
//		  port——端口名称，取值MCP23017_PORTA、MCP23017_PORTB
//		  val——要写入端口寄存器的值
//--------------------------------------------------------------------------------------   
static unsigned char MCP23017_WRITE_GPIO(struct mcp23017_dev *dev,unsigned char port,unsigned char val)
{	
	unsigned char res;
	
	//因为B端口的地址比A端口的寄存器的地址都是大1，所以采用+的技巧切换寄存器
	mcp23017_write_reg(dev,MCP23017_GPIO+port,val);

	return 0;
}



/*
 * @description	: 读取mcp23017的数据，读取原始数据，包括ALS,PS和IR, 注意！
 *				: 如果同时打开ALS,IR+PS的话两次数据读取的时间间隔要大于112.5ms
 * @param - ir	: ir数据
 * @param - ps 	: ps数据
 * @param - ps 	: als数据 
 * @return 		: 无。
 */
void mcp23017_readdata(struct mcp23017_dev *dev)
{
	// unsigned char i =0;
    // unsigned char buf[6];
	
	// /* 循环读取所有传感器数据 */
    // for(i = 0; i < 6; i++)	
    // {
    //     buf[i] = mcp23017_read_reg(dev, mcp23017_IRDATALOW + i);	
    // }

    // if(buf[0] & 0X80) 	/* IR_OF位为1,则数据无效 */
	// 	dev->ir = 0;					
	// else 				/* 读取IR传感器的数据   		*/
	// 	dev->ir = ((unsigned short)buf[1] << 2) | (buf[0] & 0X03); 			
	
	// dev->als = ((unsigned short)buf[3] << 8) | buf[2];	/* 读取ALS传感器的数据 			 */  
	
    // if(buf[4] & 0x40)	/* IR_OF位为1,则数据无效 			*/
	// 	dev->ps = 0;    													
	// else 				/* 读取PS传感器的数据    */
	// 	dev->ps = ((unsigned short)(buf[5] & 0X3F) << 4) | (buf[4] & 0X0F); 
}

/*
 * @description		: 打开设备
 * @param - inode 	: 传递给驱动的inode
 * @param - filp 	: 设备文件，file结构体有个叫做private_data的成员变量
 * 					  一般在open的时候将private_data指向设备结构体。
 * @return 			: 0 成功;其他 失败
 */
static int mcp23017_open(struct inode *inode, struct file *filp)
{
	filp->private_data = &mcp23017dev;

	/* 初始化mcp23017 */
	//首先设置其他位的默认状态
	unsigned char state = 0x2E;		//001011 10,BANK = 0,默认不关联AB（unsigned char = 0）,禁用顺序操作,使能变化率控制、使能硬件地址,开漏输出
	// if(intab==INTA_INTB_CONJUNCTION)
	// {
	// 	state |= 0x40;
	// }
	// if(hwa==HWA_DIS)
	// {
	// 	state &= (~0x08);
	// }
	// if(o==INT_PUSHPULL_HIGH)
	// {
	// 	state &= (~0x04);
	// 	state |= 0x02;
	// }
	// if(o==INT_PUSHPULL_LOW)
	// {
	// 	state &= (~0x04);
	// 	state &= (~0x02);
	// }
	
	//写回方向寄存器
	// res = I2C_Write_Byte_MCP23017(deviceAddr,MCP23017_IOCON,state);
	MCP23017_INIT(&mcp23017dev,INTA_INTB_INDEPENDENT,HWA_EN,INT_OD);		
    //所有端口使能上拉电阻,低电平输入有效，可以在端口上接按键实验
	MCP23017_IO_PU(&mcp23017dev,MCP23017_PORTA,ALLPIN,ENABLE);
	MCP23017_IO_PU(&mcp23017dev,MCP23017_PORTB,ALLPIN,ENABLE);
	MCP23017_IO_DIR(&mcp23017dev,MCP23017_PORTB,ALLPIN,OUTPUT);
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
static ssize_t mcp23017_read(struct file *filp, char __user *buf, size_t cnt, loff_t *off)
{
	u8 data[2];
	long err = 0;

	struct mcp23017_dev *dev = (struct mcp23017_dev *)filp->private_data;
	
	data[0] = MCP23017_READ_GPIO(dev,MCP23017_PORTA);
	data[1] = MCP23017_READ_GPIO(dev,MCP23017_PORTB);
	// mcp23017_readdata(dev);
	err = copy_to_user(buf, data, sizeof(data));
	printk("mcp23017_read buf[0]=%x  buf[1]=%x \r\n",buf[0],buf[1]);
	return 0;
}
/*
 * @description		: 往设备写入数据 
 * @param - filp 	: 要打开的设备文件(文件描述符)
 * @param - buf 	: 返回给用户空间的数据缓冲区
 * @param - cnt 	: 要写入的数据长度
 * @param - offt 	: 相对于文件首地址的偏移
 * @return 			: 写入的字节数，如果为负值，表示读取失败
 */
static ssize_t mcp23017_write(struct file *filp, char __user *buf, size_t cnt, loff_t *off)
{
	int retvalue;
	u8 data[10];
    retvalue= copy_from_user(data, buf, cnt);
	struct mcp23017_dev *dev = (struct mcp23017_dev *)filp->private_data;

	 	if(retvalue < 0) {
		printk("kernel write failed!\r\n");
		return -EFAULT;
	}
	printk("mcp23017_write data[0]=%x \r\n",data[0]);
	MCP23017_WRITE_GPIO(dev,MCP23017_PORTB,data[0]);
	return 0;
}
/*
 * @description		: 关闭/释放设备
 * @param - filp 	: 要关闭的设备文件(文件描述符)
 * @return 			: 0 成功;其他 失败
 */
static int mcp23017_release(struct inode *inode, struct file *filp)
{
	return 0;
}

/* mcp23017操作函数 */
static const struct file_operations mcp23017_ops = {
	.owner = THIS_MODULE,
	.open = mcp23017_open,
	.read = mcp23017_read,
	.write= mcp23017_write,
	.release = mcp23017_release,
};

 /*
  * @description     : i2c驱动的probe函数，当驱动与
  *                    设备匹配以后此函数就会执行
  * @param - client  : i2c设备
  * @param - id      : i2c设备ID
  * @return          : 0，成功;其他负值,失败
  */
static int mcp23017_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	/* 1、构建设备号 */
	if (mcp23017dev.major) {
		mcp23017dev.devid = MKDEV(mcp23017dev.major, 0);
		register_chrdev_region(mcp23017dev.devid, mcp23017_CNT, mcp23017_NAME);
	} else {
		alloc_chrdev_region(&mcp23017dev.devid, 0, mcp23017_CNT, mcp23017_NAME);
		mcp23017dev.major = MAJOR(mcp23017dev.devid);
	}

	/* 2、注册设备 */
	cdev_init(&mcp23017dev.cdev, &mcp23017_ops);
	cdev_add(&mcp23017dev.cdev, mcp23017dev.devid, mcp23017_CNT);

	/* 3、创建类 */
	mcp23017dev.class = class_create(THIS_MODULE, mcp23017_NAME);
	if (IS_ERR(mcp23017dev.class)) {
		return PTR_ERR(mcp23017dev.class);
	}

	/* 4、创建设备 */
	mcp23017dev.device = device_create(mcp23017dev.class, NULL, mcp23017dev.devid, NULL, mcp23017_NAME);
	if (IS_ERR(mcp23017dev.device)) {
		return PTR_ERR(mcp23017dev.device);
	}

	mcp23017dev.private_data = client;

	return 0;
}

/*
 * @description     : i2c驱动的remove函数，移除i2c驱动的时候此函数会执行
 * @param - client 	: i2c设备
 * @return          : 0，成功;其他负值,失败
 */
static int mcp23017_remove(struct i2c_client *client)
{
	/* 删除设备 */
	cdev_del(&mcp23017dev.cdev);
	unregister_chrdev_region(mcp23017dev.devid, mcp23017_CNT);

	/* 注销掉类和设备 */
	device_destroy(mcp23017dev.class, mcp23017dev.devid);
	class_destroy(mcp23017dev.class);
	return 0;
}

/* 传统匹配方式ID列表 */
static const struct i2c_device_id mcp23017_id[] = {
	{"alientek,mcp23017", 0},  
	{}
};

/* 设备树匹配列表 */
static const struct of_device_id mcp23017_of_match[] = {
	{ .compatible = "alientek,mcp23017" },
	{ /* Sentinel */ }
};

/* i2c驱动结构体 */	
static struct i2c_driver mcp23017_driver = {
	.probe = mcp23017_probe,
	.remove = mcp23017_remove,
	.driver = {
			.owner = THIS_MODULE,
		   	.name = "mcp23017",
		   	.of_match_table = mcp23017_of_match, 
		   },
	.id_table = mcp23017_id,
};
		   
/*
 * @description	: 驱动入口函数
 * @param 		: 无
 * @return 		: 无
 */
static int __init mcp23017_init(void)
{
	int ret = 0;

	ret = i2c_add_driver(&mcp23017_driver);
	return ret;
}

/*
 * @description	: 驱动出口函数
 * @param 		: 无
 * @return 		: 无
 */
static void __exit mcp23017_exit(void)
{
	i2c_del_driver(&mcp23017_driver);
}

/* module_i2c_driver(mcp23017_driver) */

module_init(mcp23017_init);
module_exit(mcp23017_exit);
MODULE_LICENSE("GPL");
MODULE_AUTHOR("zuozhongkai");



