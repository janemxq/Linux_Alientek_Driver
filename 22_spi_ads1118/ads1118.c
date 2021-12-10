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
#include "ads1118reg.h"
/***************************************************************
Copyright © ALIENTEK Co., Ltd. 1998-2029. All rights reserved.
文件名		: ads1118.c
作者	  	: 孟晓青
版本	   	: V1.0
描述	   	: ads1118 SPI驱动程序
其他	   	: 无
论坛 	   	: www.openedv.com
日志	   	: 初版V1.0 2021/10/23 孟晓青创建
***************************************************************/
#define ADS1118_CNT	1
#define ADS1118_NAME	"ads1118"
#define ADS1118_NUM_CHANS 5
#define ADS1118_TEMP_CHAN 4
struct ads_table {
	unsigned int rates[8];
	unsigned int divisor;
};

struct ads_channel {
	unsigned int delay_ms;
	int pga;
	u16 cfg;
	bool enabled;
};

/* PGA fullscale voltages in microvolts */
static const unsigned int fullscale_table[8] = {
	6144000, 4096000, 2048000, 1024000, 512000, 256000, 256000, 256000 };

static const struct ads_table ads1018_table = {
	.rates = {128, 250, 490, 920, 1600, 2400, 3300, 3300},
	.divisor = 0x7ff0,
};

static const struct ads_table ads1118_table = {
	.rates = {8, 16, 32, 64, 128, 250, 475, 860},
	.divisor = 0x7fff,
};
// struct ads1118 {
// 	struct device *hwmon_dev;
// 	struct device *dev;
// 	struct mutex update_lock; /* mutex protect updates */
// 	struct ads_channel channel_data[ADS1118_NUM_CHANS];
// 	const struct ads_table *ref;
// };
struct Ads1118_dev {
	dev_t devid;				/* 设备号 	 */
	struct cdev cdev;			/* cdev 	*/
	struct class *class;		/* 类 		*/
	struct device *device;		/* 设备 	 */
	struct device_node	*nd; 	/* 设备节点 */
	int major;					/* 主设备号 */
	void *private_data;			/* 私有数据 		*/
	int cs_gpio;				/* 片选所使用的GPIO编号		*/
	int sclk_gpio;				/* 片选所使用的GPIO编号		*/
	int mosi_gpio;				/* 片选所使用的GPIO编号		*/
	int miso_gpio;				/* 片选所使用的GPIO编号		*/
	struct mutex update_lock; /* mutex protect updates */
	struct ads_channel channel_data[ADS1118_NUM_CHANS];
	const struct ads_table *ref;
};

static struct Ads1118_dev ads1118dev;
static u8 chipNum;
/*
 * NOTE: the bit offsets in the datasheet are 16 bit big
 * endian. I've swapped upper and lower bytes in the defines
 * so no twiddling is needed when sending the cfg to the device.
 */
#define ADS1118_MODE	0	/* single shot mode */
#define ADS1118_PGA	1	/* programmmed gain */
#define ADS1118_MUX	4	/* input channel */
#define ADS1118_SS	7	/* start a conversion */
#define ADS1118_NOP	8	/* validation pattern */
#define ADS1118_PULL_UP	11	/* pullup resistor on MISO */
#define ADS1118_TS_MODE	12	/* temperature sensor mode */
#define ADS1118_DR	13	/* data rate table index */

#define ADS1118_ADC_CFG (BIT(ADS1118_MODE) | BIT(ADS1118_SS) | \
		(0x3 << ADS1118_NOP) | BIT(ADS1118_PULL_UP))
#define ADS1118_TEMP_CFG (ADS1118_ADC_CFG | BIT(ADS1118_TS_MODE))

/* MUX values for AINN (second input or ground) */
#define ADS1118_MUX_AINN1 0
#define ADS1118_MUX_AINN3 1
#define ADS1118_MUX_AINN_GND 4

#define ADS1118_DEFAULT_PGA 1
#define ADS1118_DEFAULT_DR 7

static inline void ads1118_set_cfg(u16 *cfg, u16 value, int offset)
{
	*cfg &= ~(0x07 << offset);
	*cfg |= ((value & 0x07) << offset);
}

static int ads1118_channel_set_pga(struct ads_channel *chan, u32 fullscale)
{
	int i;

	for (i = 7; i >= 0; --i)
		if (fullscale_table[i] == fullscale)
			break;

	if (i < 0)
		return -EINVAL;

	chan->pga = fullscale / 1000;
	ads1118_set_cfg(&chan->cfg, i, ADS1118_PGA);

	return 0;
}

static int ads1118_chan_set_mux(struct ads_channel *chan, u16 in1, u16 in2)
{
	switch (in1) {
	case 0:
		if (in2 == ADS1118_MUX_AINN1)
			break;
	case 1:
	case 2:
		if (in2 == ADS1118_MUX_AINN3)
			break;
	case 3:
		if (in2 == ADS1118_MUX_AINN_GND)
			break;
	default:
		return -EINVAL;
	}

	ads1118_set_cfg(&chan->cfg, in1 + in2, ADS1118_MUX);

	return 0;
}

static int ads1118_chan_set_rate(struct Ads1118_dev *ads,
				 struct ads_channel *chan, u32 rate)
{
	int i;

	for (i = 7; i >= 0; --i)
		if (ads->ref->rates[i] == rate)
			break;

	if (i < 0)
		return -EINVAL;

	chan->delay_ms = DIV_ROUND_UP(1000, rate);
	ads1118_set_cfg(&chan->cfg, i, ADS1118_DR);

	return 0;
}
/*
 * @description	: 从ads1118读取多个寄存器数据
 * @param - dev:  ads1118设备
 * @param - val:  读取到的数据
 * @param - len:  要读取的数据长度
 * @return 		: 操作结果
 */
static int ads1118_read_reg(struct Ads1118_dev *dev, void *buf, int len)
{
	int ret;
	unsigned char txdata[len];
	struct spi_message m;
	struct spi_transfer *t;
	struct spi_device *spi = (struct spi_device *)dev->private_data;

	gpio_set_value(dev->cs_gpio, 0);				/* 片选拉低，选中ads1118 */
	t = kzalloc(sizeof(struct spi_transfer), GFP_KERNEL);	/* 申请内存 */

	/* 读取数据 */
	txdata[0] = 0xff;			/* 随便一个值，此处无意义 */
	t->rx_buf = buf;			/* 读取到的数据 */
	t->len = len;				/* 要读取的数据长度 */
	spi_message_init(&m);		/* 初始化spi_message */
	spi_message_add_tail(t, &m);/* 将spi_transfer添加到spi_message队列 */
	ret = spi_sync(spi, &m);	/* 同步发送 */
    printk("ads1118_read_reg ret=%02X rx_buf=%04X len=%02X !\r\n",ret,t->rx_buf,len);
    
	kfree(t);									/* 释放内存 */
	gpio_set_value(dev->cs_gpio, 1);			/* 片选拉高，释放ads1118 */

	return ret;
}
/*
 * @description	: 向ads1118多个寄存器写入数据
 * @param - dev:  ads1118设备
 * @param - val:  要写入的数据缓冲区
 * @param - len:  要写入的数据长度
 * @return 	  :   操作结果
 */
static s32 ads1118_write_regs(struct Ads1118_dev *dev, u8 *buf, u8 len)
{
	int ret;
    int timeout =0;
	unsigned char rxdata[4];
	struct spi_message m;
	struct spi_transfer *t;
	struct spi_device *spi = (struct spi_device *)dev->private_data;

	t = kzalloc(sizeof(struct spi_transfer), GFP_KERNEL);	/* 申请内存 */
	gpio_set_value(dev->cs_gpio, 0);			/* 片选拉低 */
	udelay(1);
	/* 发送要写入的数据 */
	t->tx_buf = buf;			/* 要写入的数据 */
	t->len = len;				/* 写入的字节数 */
	spi_message_init(&m);		/* 初始化spi_message */
	spi_message_add_tail(t, &m);/* 将spi_transfer添加到spi_message队列 */
	ret = spi_sync(spi, &m);	/* 同步发送 */
    printk("ads1118_write_regs ret=%02X buf[0]=%02X buf[1]=%02X cs_gpio =%d !\r\n",ret,buf[0],buf[1],gpio_get_value(dev->cs_gpio));
	// gpio_set_value(dev->cs_gpio, 1);/* 片选拉高，释放ads1118 */
	
	while(gpio_get_value(dev->miso_gpio)==1)
	{
		timeout++;
        mdelay(1);
		printk("timeoust --");
	  if(timeout >=100)break;
	}
	// gpio_set_value(dev->cs_gpio, 0);			/* 片选拉低 */
	udelay(1);
	/* 读取数据 */
	buf[0] = 0xff;			/* 随便一个值，此处无意义 */
	t->rx_buf = rxdata;			/* 读取到的数据 */
	t->len = 2;				/* 要读取的数据长度 */
	spi_message_init(&m);		/* 初始化spi_message */
	spi_message_add_tail(t, &m);/* 将spi_transfer添加到spi_message队列 */
	ret = spi_sync(spi, &m);	/* 同步发送 */
	printk("read regs ret=%02X rxdata[0]=%02X rxdata[1]=%02X !\r\n",ret,rxdata[0],rxdata[1]);
	kfree(t);					/* 释放内存 */
	gpio_set_value(dev->cs_gpio, 1);/* 片选拉高，释放ads1118 */
	// printk("cs_gpio =%d \n",gpio_get_value(dev->cs_gpio));
	return ret;
}
/*
 * @description		: 打开设备
 * @param - inode 	: 传递给驱动的inode
 * @param - filp 	: 设备文件，file结构体有个叫做pr似有ate_data的成员变量
 * 					  一般在open的时候将private_data似有向设备结构体。
 * @return 			: 0 成功;其他 失败
 */
static int ads1118_open(struct inode *inode, struct file *filp)
{
	filp->private_data = &ads1118dev; /* 设置私有数据 */
	return 0;
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
int16_t ADS_SEL_Read(struct Ads1118_dev * dev,uint8_t road,uint8_t Ref,uint8_t mode)         //测几路电压
{
    int dat = 0;
	uint16_t config = ADS1118_CONFIG_SS_START_OFF |  // 
				ADS1118_CONFIG_MODE_CONTIN  |  //连续转换模式 
				ADS1118_CONFIG_DR_128SPS   |  // 转换 速率 SPS
				ADS1118_CONFIG_TS_MODE_ADC |  // 读取ADC，而不是温度
				ADS1118_CONFIG_PULL_UP_ON  |  //上拉电阻启用
				ADS1118_CONFIG_NOP_VALID   |  // this is valid data (default)
				ADS1118_CONFIG_RESV;          // reserved bits must be set to 1
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
    ads1118_write_regs(dev,&config,mode);
	ads1118_read_reg(dev,dat,2);
    return dat;
}
static int ads1118_testIO(struct Ads1118_dev *ads)
{
	int i;
	for (i = 0; i <100;i++) 
	{
      gpio_set_value(ads->cs_gpio, 0);			/* 片选拉低 */
      gpio_set_value(ads->mosi_gpio, 0);
      gpio_set_value(ads->sclk_gpio, 0);
	   mdelay(10);
	  gpio_set_value(ads->cs_gpio, 1);			/* 片选拉低 */
	  gpio_set_value(ads->mosi_gpio, 1);
	  gpio_set_value(ads->sclk_gpio, 1);
	}
	return 1;
}
static int ads1118_read_adc(struct Ads1118_dev *ads, struct ads_channel *chan,
			    s16 *value)
{
	int ret;
	u8 buf[2];
	struct spi_device *spi = (struct spi_device *)ads->private_data;
    chan->cfg=ADS1118_CONFIG_SS_START_OFF | ADS1118_CONFIG_MUX_SINGLE_0| // 
                    ADS1118_CONFIG_MODE_CONTIN  |  //连续转换模式 
                    ADS1118_CONFIG_DR_128SPS   |  // 转换 速率 SPS
                    ADS1118_CONFIG_TS_MODE_ADC |  // 读取ADC，而不是温度
                    ADS1118_CONFIG_PULL_UP_ON  |  //上拉电阻启用
                    ADS1118_CONFIG_NOP_VALID   |  // this is valid data (default)
                    ADS1118_CONFIG_RESV|ADS1118_CONFIG_PGA_4_096V;          // reserved bits must be set to 1
	// mutex_lock(&ads->update_lock);
	// gpio_set_value(ads->cs_gpio, 0);			/* 片选拉低 */
	// chan->cfg = ADS1118_CONFIG_SS_START_OFF |  // 
    //                 ADS1118_CONFIG_MODE_CONTIN  |  //连续转换模式 
    //                 ADS1118_CONFIG_DR_128SPS   |  // 转换 速率 SPS
    //                 ADS1118_CONFIG_TS_MODE_TEMP |  // 读取温度
    //                 ADS1118_CONFIG_PULL_UP_ON  |  //上拉电阻启用
    //                 ADS1118_CONFIG_NOP_VALID   |  // this is valid data (default)
    //                 ADS1118_CONFIG_RESV|// reserved bits must be set to 1
	// 				ADS1118_CONFIG_PGA_4_096V;
					// chan->cfg =0x058B|ADS1118_CONFIG_TS_MODE_TEMP ;
    printk("spi_write chan->cfg=%x\n", chan->cfg);
	// ret = spi_write(spi, &chan->cfg, sizeof(chan->cfg));
	
	// buf[0]=chan->cfg>>8;
	// buf[1]=chan->cfg&0xFF;
	// ret = spi_write(spi, buf, 2);
	// ret = ads1118_write_regs(ads, buf, 2);
	// ret = ads1118_write_regs(ads, buf, 2);
	// // if (ret < 0)
	// if (ret )
	// {
	// 	printk("spi_write eror=%d\n", ret);
	// 	return ret;
	// }

	ads1118_testIO(ads);
    // gpio_set_value(ads->cs_gpio, 1);/* 片选拉高，释放ads1118 */
	/* wait until conversion finished */
	// msleep(chan->delay_ms);
	// udelay(10);
    // gpio_set_value(ads->cs_gpio, 0);			/* 片选拉低 */
	// msleep(chan->delay_ms);
	// ret = spi_read(spi, &buf, sizeof(buf));
	// ret = ads1118_read_reg(spi, &buf, sizeof(buf));
	// if (ret)
	// {
	// 	printk("spi_read eror=%d\n", ret);
	//     dev_info(&spi->dev, "error reading: %d\n", ret);
	// }
    printk("spi_read buf=%X\n", buf);
	*value = (s16)be16_to_cpu(buf);

// err_unlock:
	// mutex_unlock(&ads->update_lock);
	gpio_set_value(ads->cs_gpio, 1);/* 片选拉高，释放ads1118 */

	return ret;
}
static void ads1118_temp_chan_enable(struct Ads1118_dev *ads)
{
	struct ads_channel *chan = &ads->channel_data[ADS1118_TEMP_CHAN];
	unsigned int rate = ads->ref->rates[ADS1118_DEFAULT_DR];

	chan->cfg = ADS1118_CONFIG_SS_START_OFF |  // 
                    ADS1118_CONFIG_MODE_CONTIN  |  //连续转换模式 
                    ADS1118_CONFIG_DR_128SPS   |  // 转换 速率 SPS
                    ADS1118_CONFIG_TS_MODE_TEMP |  // 读取温度
                    ADS1118_CONFIG_PULL_UP_ON  |  //上拉电阻启用
                    ADS1118_CONFIG_NOP_VALID   |  // this is valid data (default)
                    ADS1118_CONFIG_RESV|// reserved bits must be set to 1
					ADS1118_CONFIG_PGA_4_096V;
	ads1118_chan_set_rate(ads, chan, rate);
	chan->enabled = true;
}

static void ads1118_enable_all(struct Ads1118_dev *ads)
{
	unsigned int i;
	unsigned int fullscale = fullscale_table[ADS1118_DEFAULT_PGA];
	unsigned int rate = ads1118_table.rates[ADS1118_DEFAULT_DR];

	ads1118_temp_chan_enable(ads);

	for (i = 0; i < ADS1118_TEMP_CHAN; ++i) {
		struct ads_channel *chan = &ads->channel_data[i];

		// chan->cfg = ADS1118_ADC_CFG;
		chan->cfg=ADS1118_CONFIG_SS_START_OFF |  // 
                    ADS1118_CONFIG_MODE_CONTIN  |  //连续转换模式 
                    ADS1118_CONFIG_DR_128SPS   |  // 转换 速率 SPS
                    ADS1118_CONFIG_TS_MODE_ADC |  // 读取ADC，而不是温度
                    ADS1118_CONFIG_PULL_UP_ON  |  //上拉电阻启用
                    ADS1118_CONFIG_NOP_VALID   |  // this is valid data (default)
                    ADS1118_CONFIG_RESV|// reserved bits must be set to 1
					ADS1118_CONFIG_PGA_4_096V;          
		// ads1118_channel_set_pga(chan, fullscale);
		chan->delay_ms = DIV_ROUND_UP(1000, rate);
		// ads1118_chan_set_rate(ads, chan, rate);
		// ads1118_chan_set_mux(chan, (u16)i, ADS1118_MUX_AINN_GND);
		switch(i+4)	   //选择有效转换的路数
		{
				case 0:  chan->cfg |= ADS1118_CONFIG_MUX_DIFF_0_1;break;    //AINP = AIN0 and AINN = AIN1 (default)
				case 1:  chan->cfg |= ADS1118_CONFIG_MUX_DIFF_0_3;break;    //AINP = AIN0 and AINN = AIN3
				case 2:  chan->cfg |= ADS1118_CONFIG_MUX_DIFF_1_3;break;    //AINP = AIN1 and AINN = AIN3
				case 3:  chan->cfg |= ADS1118_CONFIG_MUX_DIFF_2_3;break;    //AINP = AIN2 and AINN = AIN3
				case 4:  chan->cfg |= ADS1118_CONFIG_MUX_SINGLE_0;break;    //AINP = AIN0 and AINN = GND
				case 5:  chan->cfg |= ADS1118_CONFIG_MUX_SINGLE_1;break;    //AINP = AIN1 and AINN = GND
				case 6:  chan->cfg |= ADS1118_CONFIG_MUX_SINGLE_2;break;    //AINP = AIN2 and AINN = GND
				case 7:  chan->cfg |= ADS1118_CONFIG_MUX_SINGLE_3;break;    //AINP = AIN3 and AINN = GND
				default : break;
		}
		chan->enabled = true;
	}
}
/*
 * @description		: 从设备读取数据 
 * @param - filp 	: 要打开的设备文件(文件描述符)
 * @param - buf 	: 返回给用户空间的数据缓冲区
 * @param - cnt 	: 要读取的数据长度
 * @param - offt 	: 相对于文件首地址的偏移
 * @return 			: 读取的字节数，如果为负值，表示读取失败
 */
static ssize_t ads1118_read(struct file *filp, int16_t __user *buf, size_t cnt, loff_t *off)
{
	// int16_t data[4];
	s16 read_value;
	int channel=0;
	struct Ads1118_dev *dev = (struct Ads1118_dev *)filp->private_data;
	struct ads_channel *chan ;
	// chan->config=ADS1118_ADC_CFG;
    // chan->enabled=true;
	// chan->pga= fullscale;

	// data[0]=ADS_SEL_Read(dev,4,1,0);
	// copy_to_user(buf, data, sizeof(data));
	// return 0;
	int microvolts;
	int ret;
	int i;
    for(i=0; i<1;i++)
	{
		printk("chanel %d  ----------------\r\n",i+1);
		chan = &dev->channel_data[i];
		ret = ads1118_read_adc(dev, chan, &read_value);
		if (ret < 0)
			return ret;
		microvolts = DIV_ROUND_CLOSEST(read_value * 4096,
						0x7fff);
		printk("温度 =%d\r\n", (((int)read_value) * 1000) >> 7);
	}
	
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
static ssize_t ads1118_write(struct file *filp, char __user *buf, size_t cnt, loff_t *off)
{
	int retvalue;
	unsigned char databuf[1];
	unsigned char ledstat;
	struct gpioled_dev *dev = filp->private_data;

	retvalue = copy_from_user(databuf, buf, cnt);
	if(retvalue < 0) {
		printk("kernel write failed!\r\n");
		return -EFAULT;
	}

	chipNum = databuf[0];		/* 获取芯片号 */
	printk("ads1118_write chipNum=%d\r\n",chipNum);	
	return 0;
}
/*
 * @description		: 关闭/释放设备
 * @param - filp 	: 要关闭的设备文件(文件描述符)
 * @return 			: 0 成功;其他 失败
 */
static int ads1118_release(struct inode *inode, struct file *filp)
{
	return 0;
}

/* ads1118操作函数 */
static const struct file_operations ads1118_ops = {
	.owner = THIS_MODULE,
	.open = ads1118_open,
	.read = ads1118_read,
	.write = ads1118_write,
	.release = ads1118_release,
};

/*
 * ads1118内部寄存器初始化函数 
 * @param  	: 无
 * @return 	: 无
 */
void ads1118_reginit(struct Ads1118_dev *dev)
{
   int16_t data[4];
	uint16_t config = ADS1118_CONFIG_SS_START_OFF |  // 
                    ADS1118_CONFIG_MODE_CONTIN  |  //连续转换模式 
                    ADS1118_CONFIG_DR_128SPS   |  // 转换 速率 SPS
                    ADS1118_CONFIG_TS_MODE_ADC |  // 读取ADC，而不是温度
                    ADS1118_CONFIG_PULL_UP_ON  |  //上拉电阻启用
                    ADS1118_CONFIG_NOP_VALID   |  // this is valid data (default)
										ADS1118_CONFIG_PGA_4_096V  | // FSR is ±4.096 V
										ADS1118_CONFIG_MUX_SINGLE_0|  //AINP is AIN0 and AINN is GND//单端
                    ADS1118_CONFIG_RESV;          // reserved bits must be set to 1
   printk("ads1118_reginit config=%04X !\r\n",config);
//    ads1118_write_regs(dev,&config,2);
// 	data[0]=ADS_SEL_Read(dev,4,1,0); 
// 	printk("ADS_SEL_Read=%04X !\r\n",data[0]);
   dev->ref=&ads1118_table;
   ads1118_enable_all(dev);
}

 /*
  * @description     : spi驱动的probe函数，当驱动与
  *                    设备匹配以后此函数就会执行
  * @param - client  : spi设备
  * @param - id      : spi设备ID
  * 
  */	
static int ads1118_probe(struct spi_device *spi)
{
	int ret = 0;
    printk("ads1118_probe!\r\n");
	/* 1、构建设备号 */
	if (ads1118dev.major) {
		ads1118dev.devid = MKDEV(ads1118dev.major, 0);
		register_chrdev_region(ads1118dev.devid, ADS1118_CNT,ADS1118_NAME);
	} else {
		alloc_chrdev_region(&ads1118dev.devid, 0, ADS1118_CNT, ADS1118_NAME);
		ads1118dev.major = MAJOR(ads1118dev.devid);
	}

	/* 2、注册设备 */
	cdev_init(&ads1118dev.cdev, &ads1118_ops);
	cdev_add(&ads1118dev.cdev, ads1118dev.devid, ADS1118_CNT);

	/* 3、创建类 */
	ads1118dev.class = class_create(THIS_MODULE, ADS1118_NAME);
	if (IS_ERR(ads1118dev.class)) {
		return PTR_ERR(ads1118dev.class);
	}

	/* 4、创建设备 */
	ads1118dev.device = device_create(ads1118dev.class, NULL, ads1118dev.devid, NULL, ADS1118_NAME);
	if (IS_ERR(ads1118dev.device)) {
		return PTR_ERR(ads1118dev.device);
	}

	/*1、  */
	ads1118dev.nd = of_find_node_by_path("/soc/aips-bus@02000000/spba-bus@02000000/ecspi@02010000");
	if(ads1118dev.nd == NULL) {
		printk("ecspi3 node not find!\r\n");
		return -EINVAL;
	} 

	/* 2、  */
	ads1118dev.cs_gpio = of_get_named_gpio(ads1118dev.nd, "cs-gpio", 0);
	if(ads1118dev.cs_gpio < 0) {
		printk("can't get cs-gpio");
		return -EINVAL;
	}
	ads1118dev.miso_gpio = of_get_named_gpio(ads1118dev.nd, "miso-gpio", 0);
	if(ads1118dev.miso_gpio < 0) {
		printk("can't get miso_gpio");
		return -EINVAL;
	}
	ads1118dev.mosi_gpio = of_get_named_gpio(ads1118dev.nd, "mosi-gpio", 0);
	if(ads1118dev.mosi_gpio < 0) {
		printk("can't get mosi_gpio");
		return -EINVAL;
	}
	ads1118dev.sclk_gpio = of_get_named_gpio(ads1118dev.nd, "sclk-gpio", 0);
	if(ads1118dev.sclk_gpio < 0) {
		printk("can't get sclk_gpio");
		return -EINVAL;
	}
	/* 3、设置GPIO1_IO20为输出，并且输出高电平 */
	gpio_direction_output(ads1118dev.cs_gpio, 1);
	// ret = gpio_direction_output(ads1118dev.mosi_gpio,1);
	// gpio_direction_output(ads1118dev.miso_gpio,1);
	// gpio_direction_output(ads1118dev.sclk_gpio, 1);
	
	// ret=gpio_request_one(ads1118dev.device,ads1118dev.sclk_gpio,GPIOF_DIR_OUT);
	// if(ret != 0) {
	// 	printk("can't set gpio! ret=%d\n", ret);
	// }
	printk("ads1118dev.cs_gpio =%d\r\n", ads1118dev.cs_gpio);
	printk("ads1118dev.mosi_gpio =%d\r\n", ads1118dev.mosi_gpio);
	printk("ads1118dev.miso_gpio =%d\r\n", ads1118dev.miso_gpio);
	printk("ads1118dev.sclk_gpio =%d\r\n", ads1118dev.sclk_gpio);
	
	/*初始化spi_device */
	spi->mode = SPI_MODE_1;	
	// spi->max_speed_hz=5000;
	// spi_setup(spi);
	// printk("max_speed_hz=%d mode=%d\r\n",spi->max_speed_hz,spi->mode);
	// ads1118dev.private_data = spi; /* 设置私有数据 */

	/* 初始化ads1118内部寄存器 */
	// ads1118_reginit(& ads1118dev);		
	return 0;
}

/*
 * @description     : spi驱动的remove函数，移除spi驱动的时候此函数会执行
 * @param - client 	: spi设备
 * @return          : 0，成功;其他负值,失败
 */
static int ads1118_remove(struct spi_device *spi)
{
	/* 删除设备 */
	cdev_del(&ads1118dev.cdev);
	unregister_chrdev_region(ads1118dev.devid, ADS1118_CNT);

	/* 注销掉类和设备 */
	device_destroy(ads1118dev.class, ads1118dev.devid);
	class_destroy(ads1118dev.class);
	return 0;
}

/* 传统匹配方式ID列表 */
static const struct spi_device_id ads1118_id[] = {
	{"alientek,ads1118", 1},  
	{}
};

/* 设备树匹配列表 */
static const struct of_device_id ads1118_of_match[] = {
	{ .compatible = "alientek,ads1118" },
	{ /* Sentinel */ }
};

/* SPI驱动结构体 */	
static struct spi_driver ads1118_driver = {
	.probe = ads1118_probe,
	.remove = ads1118_remove,
	.driver = {
			.owner = THIS_MODULE,
		   	.name = "ads1118",
		   	.of_match_table = ads1118_of_match, 
		   },
	.id_table = ads1118_id,
};
		   
/*
 * @description	: 驱动入口函数
 * @param 		: 无
 * @return 		: 无
 */
static int __init ads1118_init(void)
{
	printk("ads1118_init....\r\n");
	return spi_register_driver(&ads1118_driver);
}

/*
 * @description	: 驱动出口函数
 * @param 		: 无
 * @return 		: 无
 */
static void __exit ads1118_exit(void)
{
	printk("ads1118_exit....\r\n");
	spi_unregister_driver(&ads1118_driver);
}

module_init(ads1118_init);
module_exit(ads1118_exit);
MODULE_LICENSE("GPL");
MODULE_AUTHOR("zuozhongkai");



