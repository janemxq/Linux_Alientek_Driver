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
#include "mcp4725reg.h"
/***************************************************************
Copyright © ALIENTEK Co., Ltd. 1998-2029. All rights reserved.
文件名		: mcp4725.c
作者	  	: 左忠凯
版本	   	: V1.0
描述	   	: mcp4725驱动程序
其他	   	: 无
论坛 	   	: www.openedv.com
日志	   	: 初版V1.0 2019/9/2 左忠凯创建
***************************************************************/
#define mcp4725_CNT	1
#define mcp4725_NAME	"mcp4725"

struct mcp4725_dev {
	dev_t devid;			/* 设备号 	 */
	struct cdev cdev;		/* cdev 	*/
	struct class *class;	/* 类 		*/
	struct device *device;	/* 设备 	 */
	struct device_node	*nd; /* 设备节点 */
	int major;			/* 主设备号 */
	void *private_data;	/* 私有数据 */
	unsigned short ir, als, ps;		/* 三个光传感器数据 */
};
static u8  chipAddress[2]={0x60,0x61};
static u8  currentAdd=1;//当前地址 
static struct mcp4725_dev mcp4725dev;
/*
 * @description	: 从mcp4725读取多个寄存器数据
 * @param - dev:  mcp4725设备
 * @param - reg:  要读取的寄存器首地址
 * @param - val:  读取到的数据
 * @param - len:  要读取的数据长度
 * @return 		: 操作结果
 */
static int mcp4725_read_regs(struct mcp4725_dev *dev, MCP4725_READ_TYPE dataType)
{
	int ret;
	uint16_t value = dataType;                             //convert enum to integer to avoid compiler warnings                                    
	uint8_t buffer[dataType];
	struct i2c_msg msg[2];
	struct i2c_client *client = (struct i2c_client *)dev->private_data;
    // printk("mcp4725_read_regs addr =%d \n",client->addr);
	/* msg[0]为发送要读取的首地址 */
	msg[0].addr = chipAddress[currentAdd-1];			/* mcp4725地址 */
	msg[0].flags = 0;					/* 标记为发送数据 */
	msg[0].buf =buffer;					/* 读取的首地址 */
	msg[0].len = dataType;						/* reg长度*/

	/* msg[1]读取数据 */
	// msg[1].addr = client->addr;			/* mcp4725地址 */
	// msg[1].flags = I2C_M_RD;			/* 标记为读取数据*/
	// msg[1].buf = val;					/* 读取数据缓冲区 */
	// msg[1].len = 5;					   /* 要读取的数据长度,按最长的读*/

	// ret = i2c_transfer(client->adapter, msg, 1);
	client->addr=chipAddress[currentAdd-1];
	ret=i2c_master_recv(client,buffer,dataType);
	if (ret < 0)
		return ret;
	// if(ret == 2) {
	// 	ret = 0;
	// } else {
	// 	printk("i2c rd failed=%d reg=%06x len=%d\n",ret, reg, len);
	// 	ret = -EREMOTEIO;
	// }
	 /* read data from buffer */
	switch (dataType)
	{
	case MCP4725_READ_SETTINGS:
		ret = buffer[0];
		break;

	case MCP4725_READ_DAC_REG: case MCP4725_READ_EEPROM:
		ret = buffer[value-2];
		ret = (ret << 8) | buffer[value-1];
		break;
	}
	printk("mcp4725_read_regs ret =%x \n",ret);
	return ret;
}
/**************************************************************************/
/*
    getEepromBusyFlag()

    Return EEPROM writing status from DAC register 

    NOTE:
    - any new write command including repeat bytes during EEPROM write mode
      is ignored
    - see MCP4725 datasheet on p.20
*/
/**************************************************************************/ 
uint8_t MCP4725_getEepromBusyFlag(struct mcp4725_dev *dev)
{
  uint16_t value = mcp4725_read_regs(dev, MCP4725_READ_SETTINGS); //BSY,POR,xx,xx,xx,PD1,PD0,xx

  if (value != MCP4725_ERROR) 
  {
	printk("MCP4725_getEepromBusyFlag value=%x \r\n",value);
    return (value & 0x80)==0x80;		//1 - completed, 0 - incompleted
  }
  return 0;										//collision on i2c bus
}
/*
 * @description	: 向mcp4725多个寄存器写入数据
 * @param - dev:  mcp4725设备
 * @param - chipAddress:  mcp4725设备地址 1：0x60  2:0x61
 * @param - reg:  要写入的寄存器首地址
 * @param - val:  要写入的数据缓冲区
 * @param - len:  要写入的数据长度
 * @return 	  :   操作结果
 */
static s32 mcp4725_write_regs(struct mcp4725_dev *dev, uint16_t value, MCP4725_COMMAND_TYPE mode, MCP4725_POWER_DOWN_TYPE powerType)
{
	u8 b[256];
	int ret;
	struct i2c_msg msg;
	struct i2c_client *client = (struct i2c_client *)dev->private_data;
	printk("mcp4725_write_regs addr =%x value=%x\n",chipAddress[currentAdd-1],value);
	
	// b[0] = reg;					/* 寄存器首地址 */
	// memcpy(&b[1],buf,len);		/* 将要写入的数据拷贝到数组b里面 */
		
	// msg.addr = client->addr;	/* mcp4725地址 */
	// msg.flags = 0;				/* 标记为写数据 */

	// msg.buf = b;				/* 要写入的数据缓冲区 */
	// msg.len = len + 1;			/* 要写入的数据长度 */
	switch (mode)
	{
		case MCP4725_FAST_MODE:                                            //see MCP4725 datasheet on p.18
			b[0] = mode | (powerType << 4)  | highByte(value);
			b[1] = lowByte(value);
				
			msg.addr = chipAddress[currentAdd-1];	/* mcp4725地址 */
			msg.flags = 0;				/* 标记为写数据 */

			msg.buf = b;				/* 要写入的数据缓冲区 */
			msg.len = 2;			/* 要写入的数据长度 */
			ret=i2c_transfer(client->adapter, &msg, 1);
			printk("client->addr=%02x，b[0]=%02x b[1]=%02x ret =%04X\r\n",chipAddress[currentAdd-1],b[0],b[1],ret);
			break;

		case MCP4725_REGISTER_MODE: case MCP4725_EEPROM_MODE:              //see MCP4725 datasheet on p.19
			value = value << 4;                                              //D11,D10,D9,D8,D7,D6,D5,D4,  D3,D2,D1,D0,xx,xx,xx,xx
			b[0] = mode  | (powerType << 1);
			b[1] = highByte(value);
			b[2] = lowByte(value);
		
			msg.addr = chipAddress[currentAdd-1];	/* mcp4725地址 */
			msg.flags = 0;				/* 标记为写数据 */

			msg.buf = b;				/* 要写入的数据缓冲区 */
			msg.len = 3;			/* 要写入的数据长度 */
			ret=i2c_transfer(client->adapter, &msg, 1);
			printk("client->addr=%02x，b[0]=%02x b[1]=%02x b[2]=%02x ret =%04X\r\n",chipAddress[currentAdd-1],b[0],b[1],b[2],ret);
			break;
	}
	
	if(ret <=0 )return 0;
	if (mode == MCP4725_EEPROM_MODE)
	{
		if (MCP4725_getEepromBusyFlag(dev) == 1) return 1;                      //write completed, success!!!
			msleep(MCP4725_EEPROM_WRITE_TIME); //typical EEPROM write time 25 msec
		if (MCP4725_getEepromBusyFlag(dev) == 1) return 1;                      //write completed, success!!!
			msleep(MCP4725_EEPROM_WRITE_TIME); //maximum EEPROM write time 25 + 25 = 50 msec
	}
	printk("ret2 =%04X\r\n", ret);
	return ret;
}

/*
 * @description	: 读取mcp4725指定寄存器值，读取一个寄存器
 * @param - dev:  mcp4725设备
 * @param - reg:  要读取的寄存器
 * @return 	  :   读取到的寄存器值
 */
// static unsigned char mcp4725_read_reg(struct mcp4725_dev *dev,  MCP4725_READ_TYPE dataType)
// {
// 	u8 data = 0;
// 	mcp4725_read_regs(dev, dataType);
// 	return data;

// #if 0
// 	struct i2c_client *client = (struct i2c_client *)dev->private_data;
// 	return i2c_smbus_read_byte_data(client, reg);
// #endif
// }

/*
 * @description	: 向mcp4725指定寄存器写入指定的值，写一个寄存器
 * @param - dev:  mcp4725设备
 * @param - reg:  要写的寄存器
 * @param - data: 要写入的值
 * @return   :    无
 */
// static void mcp4725_write_reg(struct mcp4725_dev *dev, uint16_t value, MCP4725_COMMAND_TYPE mode, MCP4725_POWER_DOWN_TYPE powerType)
// {
// 	mcp4725_write_regs(dev, value, mode, powerType);
// }
//--------------------------------------------------------------------------------------   
//  函数名称：bit MCP4725_INIT(unsigned char deviceAddr,unsigned char ab,unsigned char hwa,unsigned char o)
//  函数功能：初始化指定地址的MCP4725器件
//	参数：deviceAddr——设备地址，有A0，A1，A2决定
//		  intab——配置INTA、INTB是否关联，取值INTA_INTB_CONJUNCTION、INTA_INTB_INDEPENDENT
//		  hwa——配置A0、A1、A2硬件地址是否使能，取值HWA_EN、HWA_DIS	
//		  o——配置INTA、INTB的输出类型，取值INT_OD、INT_PUSHPULL_HIGH、INT_PUSHPULL_LOW 
//--------------------------------------------------------------------------------------   
// static unsigned char MCP4725_INIT(struct mcp4725_dev *dev,unsigned char intab,unsigned char hwa,unsigned char o)
// {
// 	unsigned char state;
// 	unsigned char res;
	
// 	// //首先设置其他位的默认状态
// 	// state = 0x2E;		//001011 10,BANK = 0,默认不关联AB（bit = 0）,禁用顺序操作,使能变化率控制、使能硬件地址,开漏输出
// 	// if(intab==INTA_INTB_CONJUNCTION)
// 	// {
// 	// 	state |= 0x40;
// 	// }
// 	// if(hwa==HWA_DIS)
// 	// {
// 	// 	state &= (~0x08);
// 	// }
// 	// if(o==INT_PUSHPULL_HIGH)
// 	// {
// 	// 	state &= (~0x04);
// 	// 	state |= 0x02;
// 	// }
// 	// if(o==INT_PUSHPULL_LOW)
// 	// {
// 	// 	state &= (~0x04);
// 	// 	state &= (~0x02);
// 	// }
	
// 	//写回方向寄存器
// 	// mcp4725_write_reg(dev,MCP4725_IOCON,state);
	
// 	return 0;
// }
/*
 * @description	: 读取mcp4725的数据，读取原始数据，包括ALS,PS和IR, 注意！
 *				: 如果同时打开ALS,IR+PS的话两次数据读取的时间间隔要大于112.5ms
 * @param - ir	: ir数据
 * @param - ps 	: ps数据
 * @param - ps 	: als数据 
 * @return 		: 无。
 */
void mcp4725_readdata(struct mcp4725_dev *dev)
{
	
}

/*
 * @description		: 打开设备
 * @param - inode 	: 传递给驱动的inode
 * @param - filp 	: 设备文件，file结构体有个叫做private_data的成员变量
 * 					  一般在open的时候将private_data指向设备结构体。
 * @return 			: 0 成功;其他 失败
 */
static int mcp4725_open(struct inode *inode, struct file *filp)
{
	filp->private_data = &mcp4725dev;

	/* 初始化mcp4725 */
	//设置参考电压 3.3V

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
static ssize_t mcp4725_read(struct file *filp, char __user *buf, size_t cnt, loff_t *off)
{
	// u8 data[2];
	// long err = 0;
	struct mcp4725_dev *dev = (struct mcp4725_dev *)filp->private_data;
	//  mcp4725_write_regs(dev,0x7ff, MCP4725_FAST_MODE, MCP4725_POWER_DOWN_OFF);
	return mcp4725_read_regs(dev,MCP4725_READ_EEPROM);
}
/*
 * @description		: 往设备写入数据 
 * @param - filp 	: 要打开的设备文件(文件描述符)
 * @param - buf 	: 返回给用户空间的数据缓冲区
 * @param - cnt 	: 要写入的数据长度
 * @param - offt 	: 相对于文件首地址的偏移
 * @return 			: 写入的字节数，如果为负值，表示读取失败
 */
static ssize_t mcp4725_write(struct file *filp, char __user *buf, size_t cnt, loff_t *off)
{
	int retvalue;
	char data[10];
    retvalue= copy_from_user(data, buf, cnt);
	currentAdd=data[0];
	struct mcp4725_dev *dev = (struct mcp4725_dev *)filp->private_data;

	 if(retvalue < 0) 
	{
		printk("kernel write failed!\r\n");
		return -EFAULT;
	}
	printk("mcp4725_write address=%x  data[0]=%x data[1]=%x cnt=%x\r\n",data[0],data[1],data[2],cnt);
    return mcp4725_write_regs(dev, (data[2]<<8)+data[1], MCP4725_EEPROM_MODE, MCP4725_POWER_DOWN_OFF);
}
/*
 * @description		: 关闭/释放设备
 * @param - filp 	: 要关闭的设备文件(文件描述符)
 * @return 			: 0 成功;其他 失败
 */
static int mcp4725_release(struct inode *inode, struct file *filp)
{
	return 0;
}

/* mcp4725操作函数 */
static const struct file_operations mcp4725_ops = {
	.owner = THIS_MODULE,
	.open = mcp4725_open,
	.read = mcp4725_read,
	.write= mcp4725_write,
	.release = mcp4725_release,
};

 /*
  * @description     : i2c驱动的probe函数，当驱动与
  *                    设备匹配以后此函数就会执行
  * @param - client  : i2c设备
  * @param - id      : i2c设备ID
  * @return          : 0，成功;其他负值,失败
  */
static int mcp4725_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	/* 1、构建设备号 */
	if (mcp4725dev.major) {
		mcp4725dev.devid = MKDEV(mcp4725dev.major, 0);
		register_chrdev_region(mcp4725dev.devid, mcp4725_CNT, mcp4725_NAME);
	} else {
		alloc_chrdev_region(&mcp4725dev.devid, 0, mcp4725_CNT, mcp4725_NAME);
		mcp4725dev.major = MAJOR(mcp4725dev.devid);
	}

	/* 2、注册设备 */
	cdev_init(&mcp4725dev.cdev, &mcp4725_ops);
	cdev_add(&mcp4725dev.cdev, mcp4725dev.devid, mcp4725_CNT);

	/* 3、创建类 */
	mcp4725dev.class = class_create(THIS_MODULE, mcp4725_NAME);
	if (IS_ERR(mcp4725dev.class)) {
		return PTR_ERR(mcp4725dev.class);
	}

	/* 4、创建设备 */
	mcp4725dev.device = device_create(mcp4725dev.class, NULL, mcp4725dev.devid, NULL, mcp4725_NAME);
	if (IS_ERR(mcp4725dev.device)) {
		return PTR_ERR(mcp4725dev.device);
	}

	mcp4725dev.private_data = client;

	return 0;
}

/*
 * @description     : i2c驱动的remove函数，移除i2c驱动的时候此函数会执行
 * @param - client 	: i2c设备
 * @return          : 0，成功;其他负值,失败
 */
static int mcp4725_remove(struct i2c_client *client)
{
	/* 删除设备 */
	cdev_del(&mcp4725dev.cdev);
	unregister_chrdev_region(mcp4725dev.devid, mcp4725_CNT);

	/* 注销掉类和设备 */
	device_destroy(mcp4725dev.class, mcp4725dev.devid);
	class_destroy(mcp4725dev.class);
	return 0;
}

/* 传统匹配方式ID列表 */
static const struct i2c_device_id mcp4725_id[] = {
	{"alientek,mcp4725", 0},  
	// {"alientek,mcp4725B", 1},  
	{}
};

/* 设备树匹配列表 */
static const struct of_device_id mcp4725_of_match[] = {
	{ .compatible = "alientek,mcp4725" },
	// { .compatible = "alientek,mcp4725B" }
	{ /* Sentinel */ }
};

/* i2c驱动结构体 */	
static struct i2c_driver mcp4725_driver = {
	.probe = mcp4725_probe,
	.remove = mcp4725_remove,
	.driver = {
			.owner = THIS_MODULE,
		   	.name = "mcp4725",
		   	.of_match_table = mcp4725_of_match, 
		   },
	.id_table = mcp4725_id,
};
		   
/*
 * @description	: 驱动入口函数
 * @param 		: 无
 * @return 		: 无
 */
static int __init mcp4725_init(void)
{
	int ret = 0;

	ret = i2c_add_driver(&mcp4725_driver);
	return ret;
}

/*
 * @description	: 驱动出口函数
 * @param 		: 无
 * @return 		: 无
 */
static void __exit mcp4725_exit(void)
{
	i2c_del_driver(&mcp4725_driver);
}

/* module_i2c_driver(mcp4725_driver) */

module_init(mcp4725_init);
module_exit(mcp4725_exit);
MODULE_LICENSE("GPL");
MODULE_AUTHOR("zuozhongkai");



