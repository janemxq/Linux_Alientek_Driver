#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/ide.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/gpio.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/of_gpio.h>
#include <linux/semaphore.h>
#include <linux/timer.h>
#include <linux/i2c.h>
#include <linux/spi/spi.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_gpio.h>
#include <linux/platform_device.h>
#include <asm/mach/map.h>
#include <asm/uaccess.h>
#include <asm/io.h>
#include "ad7799reg.h"
/***************************************************************
Copyright © ALIENTEK Co., Ltd. 1998-2029. All rights reserved.
文件名		: ad7799.c
作者	  	: 孟晓青
版本	   	: V1.0
描述	   	: ad7799 SPI驱动程序
其他	   	: 无
论坛 	   	: www.openedv.com
日志	   	: 初版V1.0 2021/10/23 孟晓青创建
***************************************************************/
#define AD7799_CNT	1
#define AD7799_NAME	"ad7799"

struct ad7799_dev {
	dev_t devid;				/* 设备号 	 */
	struct cdev cdev;			/* cdev 	*/
	struct class *class;		/* 类 		*/
	struct device *device;		/* 设备 	 */
	struct device_node	*nd; 	/* 设备节点 */
	int major;					/* 主设备号 */
	void *private_data;			/* 私有数据 		*/
	int cs_gpio;				/* 片选所使用的GPIO编号		*/
	AD7799_Rate rate; /**< Selected sample rate */
	AD7799_Mode mode; /**< Selected mode */
	AD7799_Channel channel; /**< Selected channel */
	AD7799_Gain gain; /**< Selected gain */
	AD7799_Polarity polarity; /**< Selected polarity */

	uint32_t rawConversion[3]; /**< Value of raw conversion */
	float voltConversion[3]; /**< Value of voltage */
};

static struct ad7799_dev ad7799dev;

/*
 * @description	: 从ad7799读取多个寄存器数据
 * @param - dev:  ad7799设备
 * @param - reg:  要读取的寄存器首地址
 * @param - val:  读取到的数据
 * @param - len:  要读取的数据长度
 * @return 		: 操作结果
 */
static int ad7799_read_regs(struct ad7799_dev *dev, u8 reg, void *buf, int len)
{
	int ret;
	unsigned char txdata[len];
	struct spi_message m;
	struct spi_transfer *t;
	struct spi_device *spi = (struct spi_device *)dev->private_data;

	gpio_set_value(dev->cs_gpio, 0);				/* 片选拉低，选中ad7799 */
	t = kzalloc(sizeof(struct spi_transfer), GFP_KERNEL);	/* 申请内存 */

	/* 第1次，发送要读取的寄存地址 */
	txdata[0] =AD7799_COMM_READ |  AD7799_COMM_ADDR(reg);/* 写数据的时候寄存器地址bit8要置1 */
	t->tx_buf = txdata;			/* 要发送的数据 */
	t->len = 1;					/* 1个字节 */
	spi_message_init(&m);		/* 初始化spi_message */
	spi_message_add_tail(t, &m);/* 将spi_transfer添加到spi_message队列 */
	ret = spi_sync(spi, &m);	/* 同步发送 */

	/* 第2次，读取数据 */
	txdata[0] = 0x00;			/* 随便一个值，此处无意义 */
	t->rx_buf = buf;			/* 读取到的数据 */
	t->len = len;				/* 要读取的数据长度 */
	spi_message_init(&m);		/* 初始化spi_message */
	spi_message_add_tail(t, &m);/* 将spi_transfer添加到spi_message队列 */
	ret = spi_sync(spi, &m);	/* 同步发送 */

	kfree(t);									/* 释放内存 */
	gpio_set_value(dev->cs_gpio, 1);			/* 片选拉高，释放ad7799 */

	return ret;
}
/***************************************************************************//**
 * @brief Reads the value of the selected register
 *
 * @param regAddress - The address of the register to read.
 * @param size - The size of the register to read.
 *
 * @return data - The value of the selected register register.
*******************************************************************************/
unsigned long AD7799_GetRegisterValue(struct ad7799_dev *dev,unsigned char regAddress, unsigned char size)
{
	unsigned char data[5] = {0x00, 0x00, 0x00, 0x00, 0x00};
	unsigned long receivedData = 0x00;	
	// data[0] = AD7799_COMM_READ |  AD7799_COMM_ADDR(regAddress);
	// AD7799_CS_LOW; 
	ad7799_read_regs(dev,regAddress,data,size); 
	// SPI_Write(data,1);
	// SPI_Read(data,size);
	// AD7799_CS_HIGH;
	if(size == 1)
	{
		receivedData += (data[0] << 0);
	}
	if(size == 2)
	{
		receivedData += (data[0] << 8);
		receivedData += (data[1] << 0);
	}
	if(size == 3)
	{
		receivedData += (data[0] << 16);
		receivedData += (data[1] << 8);
		receivedData += (data[2] << 0);
	}
    return receivedData;
}

/*
 * @description	: 向ad7799多个寄存器写入数据
 * @param - dev:  ad7799设备
 * @param - reg:  要写入的寄存器首地址
 * @param - val:  要写入的数据缓冲区
 * @param - len:  要写入的数据长度
 * @return 	  :   操作结果
 */
static s32 ad7799_write_regs(struct ad7799_dev *dev, u8 reg, u8 *buf, u8 len)
{
	int ret;

	unsigned char txdata[len];
	struct spi_message m;
	struct spi_transfer *t;
	struct spi_device *spi = (struct spi_device *)dev->private_data;

	t = kzalloc(sizeof(struct spi_transfer), GFP_KERNEL);	/* 申请内存 */
	gpio_set_value(dev->cs_gpio, 0);			/* 片选拉低 */

	/* 第1次，发送要读取的寄存地址 */
	txdata[0] = AD7799_COMM_WRITE |  AD7799_COMM_ADDR(reg);	/* 写数据的时候寄存器地址bit8要清零 */
	t->tx_buf = txdata;			/* 要发送的数据 */
	t->len = 1;					/* 1个字节 */
	spi_message_init(&m);		/* 初始化spi_message */
	spi_message_add_tail(t, &m);/* 将spi_transfer添加到spi_message队列 */
	ret = spi_sync(spi, &m);	/* 同步发送 */

	/* 第2次，发送要写入的数据 */
	t->tx_buf = buf;			/* 要写入的数据 */
	t->len = len;				/* 写入的字节数 */
	spi_message_init(&m);		/* 初始化spi_message */
	spi_message_add_tail(t, &m);/* 将spi_transfer添加到spi_message队列 */
	ret = spi_sync(spi, &m);	/* 同步发送 */

	kfree(t);					/* 释放内存 */
	gpio_set_value(dev->cs_gpio, 1);/* 片选拉高，释放ad7799 */
	return ret;
}
/***************************************************************************//**
 * @brief Writes the value to the register
 *
 * @param -  regAddress - The address of the register to write to.
 * @param -  regValue - The value to write to the register.
 * @param -  size - The size of the register to write.
 *
 * @return  None.    
*******************************************************************************/
void AD7799_SetRegisterValue(struct ad7799_dev *dev,
	                         unsigned char regAddress,
                             unsigned long regValue, 
                             unsigned char size)
{
	unsigned char data[4] = { 0x00, 0x00, 0x00, 0x00};	
    if(size == 1)
    {
        data[0] = (unsigned char)regValue;
    }
    if(size == 2)
    {
		data[1] = (unsigned char)((regValue & 0x0000FF) >> 0);
        data[0] = (unsigned char)((regValue & 0x00FF00) >> 8);
    }
    if(size == 3)
    {
		data[2] = (unsigned char)((regValue & 0x0000FF) >> 0);
		data[1] = (unsigned char)((regValue & 0x00FF00) >> 8);
        data[0] = (unsigned char)((regValue & 0xFF0000) >> 16);
    }
	// AD7799_CS_LOW;	
	ad7799_write_regs(dev,regAddress,data,size);   
	// SPI_Write(data,(1 + size));
	// AD7799_CS_HIGH;

}
/*
 * @description	: 读取ad7799指定寄存器值，读取一个寄存器
 * @param - dev:  ad7799设备
 * @param - reg:  要读取的寄存器
 * @return 	  :   读取到的寄存器值
 */
static unsigned char ad7799_read_onereg(struct ad7799_dev *dev, u8 reg)
{
	u8 data = 0;
	ad7799_read_regs(dev, reg, &data, 1);
	return data;
}

/*
 * @description	: 向ad7799指定寄存器写入指定的值，写一个寄存器
 * @param - dev:  ad7799设备
 * @param - reg:  要写的寄存器
 * @param - data: 要写入的值
 * @return   :    无
 */	

static void ad7799_write_onereg(struct ad7799_dev *dev, u8 reg, u8 value)
{
	u8 buf = value;
	ad7799_write_regs(dev, reg, &buf, 1);
}
/**
 * Reset Function. Clear all configuration
 * @param ad7799
 */
void AD7799_Reset(struct ad7799_dev *ad7799) {
	struct spi_device *spi = (struct spi_device *)ad7799->private_data;
	uint8_t dataToSend[4] = { 0xff, 0xff, 0xff, 0xff };
	spi_write(spi, dataToSend, 4);

	ad7799->mode = AD7799_MODE_CONT;
	ad7799->gain = AD7799_GAIN_2;
	ad7799->channel = AD7799_CH_AIN1P_AIN1M;
	ad7799->polarity = AD7799_BIPOLAR;
	ad7799->rate = AD7799_RATE_4_17HZ_74DB;
}
/**
 * Set PGA Gain
 * @param ad7799
 * @param gain
 */
void AD7799_SetGain(struct ad7799_dev *ad7799, AD7799_Gain gain) {
	uint32_t command;
	command=AD7799_GetRegisterValue(ad7799, AD7799_REG_CONF,2);
	command &= ~AD7799_CONF_GAIN(0xFF);
	command |= AD7799_CONF_GAIN((uint32_t ) gain);
	AD7799_SetRegisterValue(ad7799, AD7799_REG_CONF, command, 2);
	ad7799->gain = gain;
}
/**
 * Set unipolar or bipolar conversion
 * @param ad7799
 * @param polarity
 */
void AD7799_SetPolarity(struct ad7799_dev *ad7799, AD7799_Polarity polarity) {
	uint32_t command = 0;
	command=AD7799_GetRegisterValue(ad7799, AD7799_REG_CONF,2);
	command &= ~AD7799_CONF_POLAR(1);
	command |= AD7799_CONF_POLAR((uint32_t ) polarity);
	AD7799_SetRegisterValue(ad7799, AD7799_REG_CONF, command, 2);
	ad7799->polarity = polarity;
}
/**
 * Set the functioning mode
 * @param ad7799
 * @param mode
 */
void AD7799_SetMode(struct ad7799_dev *ad7799, AD7799_Mode mode) {
	uint32_t command;
	command=AD7799_GetRegisterValue(ad7799, AD7799_REG_MODE, 2);
	command &= ~AD7799_MODE_SEL(0xFF);
	command |= AD7799_MODE_SEL((uint32_t ) mode);
	AD7799_SetRegisterValue(ad7799, AD7799_REG_MODE, command, 2);
	ad7799->mode = mode;
}
/**
 * Set reference detection
 * @param ad7799
 * @param state
 */
void AD7799_SetReference(struct ad7799_dev *ad7799, uint8_t state) {
	uint32_t command = 0;
	command=AD7799_GetRegisterValue(ad7799, AD7799_REG_CONF, 2);
	command &= ~AD7799_CONF_REFDET(1);
	command |= AD7799_CONF_REFDET(state);
	AD7799_SetRegisterValue(ad7799, AD7799_REG_CONF, command, 2);
}
/**
 * Set the differential channel
 * @param ad7799
 * @param channel
 */
void AD7799_SetChannel(struct ad7799_dev *ad7799, AD7799_Channel channel) {
	uint32_t command;
	command=AD7799_GetRegisterValue(ad7799, AD7799_REG_CONF, 2);
	command &= ~AD7799_CONF_CHAN(0xFF);
	command |= AD7799_CONF_CHAN((uint32_t ) channel);
	AD7799_SetRegisterValue(ad7799, AD7799_REG_CONF, command, 2);
	ad7799->channel = channel;
}
/**
 * Return 1 when conversion is done
 * @param ad7799
 * @return
 */
uint8_t AD7799_Ready(struct ad7799_dev *ad7799) {
	uint8_t rdy = 0;
	ad7799_read_regs(ad7799, AD7799_REG_STAT, &rdy,1);
	rdy = (rdy & 0x80);
	return (!rdy);
}
/*
 * @description	: 读取ad7799的数据，三个通道一起都上来
 * 				:
 * @param - dev	: ad7799设备
 * @return 		: 无。
 */
void ad7799_readdata(struct ad7799_dev *dev)
{
	unsigned char data[14];
	u8 ChannelBuf[3]={AD7799_CH_AIN1P_AIN1M,AD7799_CH_AIN2P_AIN2M,AD7799_CH_AIN3P_AIN3M};		//通道1  通道2 通道3
	int channel=0;
	for(channel=0;channel<3;channel++)
	{
		AD7799_SetChannel(dev,ChannelBuf[channel]);//通道设置.		0~1
		mdelay(10);
		ad7799_read_regs(dev, AD7799_REG_DATA, data, 3);//清空之前的AD
		
		while( !AD7799_Ready(dev))		//1~2
		{
			mdelay(1);
		}
		ad7799_read_regs(dev, AD7799_REG_DATA, data, 3);//0:通道1 1:通道2
		dev->rawConversion[channel] = (uint32_t)((data[0] << 16) | (data[1]<< 8)|data[0]); 
	}
}

/*
 * @description		: 打开设备
 * @param - inode 	: 传递给驱动的inode
 * @param - filp 	: 设备文件，file结构体有个叫做pr似有ate_data的成员变量
 * 					  一般在open的时候将private_data似有向设备结构体。
 * @return 			: 0 成功;其他 失败
 */
static int ad7799_open(struct inode *inode, struct file *filp)
{
	filp->private_data = &ad7799dev; /* 设置私有数据 */
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
static ssize_t ad7799_read(struct file *filp, char __user *buf, size_t cnt, loff_t *off)
{
	signed int data[9];
	long err = 0;
	struct ad7799_dev *dev = (struct ad7799_dev *)filp->private_data;

	ad7799_readdata(dev);
	data[0] = dev->rawConversion[0]>>16;
	data[1] = dev->rawConversion[0]>>8;
	data[2] = (dev->rawConversion[0]&0xff);
	data[3] = dev->rawConversion[1]>>16;;
	data[4] = dev->rawConversion[1]>>8;
	data[5] = (dev->rawConversion[1]&0xff);
	data[6] = dev->rawConversion[2]>>16;;
	data[7] = dev->rawConversion[2]>>8;
	data[8] = (dev->rawConversion[2]&0xff);
	err = copy_to_user(buf, data, sizeof(data));
	return 0;
}

/*
 * @description		: 关闭/释放设备
 * @param - filp 	: 要关闭的设备文件(文件描述符)
 * @return 			: 0 成功;其他 失败
 */
static int ad7799_release(struct inode *inode, struct file *filp)
{
	return 0;
}

/* ad7799操作函数 */
static const struct file_operations ad7799_ops = {
	.owner = THIS_MODULE,
	.open = ad7799_open,
	.read = ad7799_read,
	.release = ad7799_release,
};

/*
 * ad7799内部寄存器初始化函数 
 * @param  	: 无
 * @return 	: 无
 */
void ad7799_reginit(void)
{
	u8 ID=-1;
	ad7799_read_regs(&ad7799dev,AD7799_REG_ID,&ID, 1);
	if( (ID& 0x0F) != AD7799_ID)
	{
		printk("ad7799 ID no found \r\n");
	}
	printk("ad7799 ID %02d \r\n", ID);
	// u8 value = 0;
	
	// ad7799_write_onereg(&ad7799dev, ICM20_PWR_MGMT_1, 0x80);
	// mdelay(50);
	// ad7799_write_onereg(&ad7799dev, ICM20_PWR_MGMT_1, 0x01);
	// mdelay(50);

	// value = ad7799_read_onereg(&ad7799dev, ICM20_WHO_AM_I);
	// printk("ad7799 ID = %#X\r\n", value);	

	// ad7799_write_onereg(&ad7799dev, ICM20_SMPLRT_DIV, 0x00); 	/* 输出速率是内部采样率					*/
	// ad7799_write_onereg(&ad7799dev, ICM20_GYRO_CONFIG, 0x18); 	/* 陀螺仪±2000dps量程 				*/
	// ad7799_write_onereg(&ad7799dev, ICM20_ACCEL_CONFIG, 0x18); 	/* 加速度计±16G量程 					*/
	// ad7799_write_onereg(&ad7799dev, ICM20_CONFIG, 0x04); 		/* 陀螺仪低通滤波BW=20Hz 				*/
	// ad7799_write_onereg(&ad7799dev, ICM20_ACCEL_CONFIG2, 0x04); /* 加速度计低通滤波BW=21.2Hz 			*/
	// ad7799_write_onereg(&ad7799dev, ICM20_PWR_MGMT_2, 0x00); 	/* 打开加速度计和陀螺仪所有轴 				*/
	// ad7799_write_onereg(&ad7799dev, ICM20_LP_MODE_CFG, 0x00); 	/* 关闭低功耗 						*/
	// ad7799_write_onereg(&ad7799dev, ICM20_FIFO_EN, 0x00);		/* 关闭FIFO						*/
}

 /*
  * @description     : spi驱动的probe函数，当驱动与
  *                    设备匹配以后此函数就会执行
  * @param - client  : spi设备
  * @param - id      : spi设备ID
  * 
  */	
static int ad7799_probe(struct spi_device *spi)
{
	int ret = 0;

	/* 1、构建设备号 */
	if (ad7799dev.major) {
		ad7799dev.devid = MKDEV(ad7799dev.major, 0);
		register_chrdev_region(ad7799dev.devid, AD7799_CNT,AD7799_NAME);
	} else {
		alloc_chrdev_region(&ad7799dev.devid, 0, AD7799_CNT, AD7799_NAME);
		ad7799dev.major = MAJOR(ad7799dev.devid);
	}

	/* 2、注册设备 */
	cdev_init(&ad7799dev.cdev, &ad7799_ops);
	cdev_add(&ad7799dev.cdev, ad7799dev.devid, AD7799_CNT);

	/* 3、创建类 */
	ad7799dev.class = class_create(THIS_MODULE, AD7799_NAME);
	if (IS_ERR(ad7799dev.class)) {
		return PTR_ERR(ad7799dev.class);
	}

	/* 4、创建设备 */
	ad7799dev.device = device_create(ad7799dev.class, NULL, ad7799dev.devid, NULL, AD7799_NAME);
	if (IS_ERR(ad7799dev.device)) {
		return PTR_ERR(ad7799dev.device);
	}

	/*1、  */
	ad7799dev.nd = of_find_node_by_path("/soc/aips-bus@02000000/spba-bus@02000000/ecspi@02010000");
	if(ad7799dev.nd == NULL) {
		printk("ecspi3 node not find!\r\n");
		return -EINVAL;
	} 

	/* 2、  */
	ad7799dev.cs_gpio = of_get_named_gpio(ad7799dev.nd, "cs-gpio", 0);
	if(ad7799dev.cs_gpio < 0) {
		printk("can't get cs-gpio");
		return -EINVAL;
	}

	/* 3、设置GPIO1_IO20为输出，并且输出高电平 */
	ret = gpio_direction_output(ad7799dev.cs_gpio, 1);
	if(ret < 0) {
		printk("can't set gpio!\r\n");
	}

	/*初始化spi_device */
	spi->mode = SPI_MODE_0;	/*MODE0，CPOL=0，CPHA=0*/
	spi_setup(spi);
	ad7799dev.private_data = spi; /* 设置私有数据 */

	/* 初始化ad7799内部寄存器 */
	ad7799_reginit();		
	return 0;
}

/*
 * @description     : spi驱动的remove函数，移除spi驱动的时候此函数会执行
 * @param - client 	: spi设备
 * @return          : 0，成功;其他负值,失败
 */
static int ad7799_remove(struct spi_device *spi)
{
	/* 删除设备 */
	cdev_del(&ad7799dev.cdev);
	unregister_chrdev_region(ad7799dev.devid, AD7799_CNT);

	/* 注销掉类和设备 */
	device_destroy(ad7799dev.class, ad7799dev.devid);
	class_destroy(ad7799dev.class);
	return 0;
}

/* 传统匹配方式ID列表 */
static const struct spi_device_id ad7799_id[] = {
	{"alientek,ad7799", 0},  
	{}
};

/* 设备树匹配列表 */
static const struct of_device_id ad7799_of_match[] = {
	{ .compatible = "alientek,ad7799" },
	{ /* Sentinel */ }
};

/* SPI驱动结构体 */	
static struct spi_driver ad7799_driver = {
	.probe = ad7799_probe,
	.remove = ad7799_remove,
	.driver = {
			.owner = THIS_MODULE,
		   	.name = "ad7799",
		   	.of_match_table = ad7799_of_match, 
		   },
	.id_table = ad7799_id,
};
		   
/*
 * @description	: 驱动入口函数
 * @param 		: 无
 * @return 		: 无
 */
static int __init ad7799_init(void)
{
	return spi_register_driver(&ad7799_driver);
}

/*
 * @description	: 驱动出口函数
 * @param 		: 无
 * @return 		: 无
 */
static void __exit ad7799_exit(void)
{
	spi_unregister_driver(&ad7799_driver);
}

module_init(ad7799_init);
module_exit(ad7799_exit);
MODULE_LICENSE("GPL");
MODULE_AUTHOR("mengxiaoqing");



