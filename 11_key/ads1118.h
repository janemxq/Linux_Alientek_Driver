#ifndef __ADS1118_H
#define __ADS1118_H	 

//端口定义
#define CONFIG_VALUE    0X408B       //AIN0-AIN1  4.096  128sps  pull on DOUT

// #define SCLK PCout(12)	//ads1118时钟引脚
// #define MOSI PCout(11)	//ads1118 DIN引脚
// #define CS   PCout(8)	//ads1118 CS引脚

// #define MISO PCout(10)	//ads1118 DOUT/DRDY引脚
#define READ_MISO(x) gpio_get_value(x)

#define SCLK_H(x) gpio_set_value(x,1)
#define SCLK_L(x) gpio_set_value(x,0)

/*单片机为主机*/
#define MOSI_H(x) gpio_set_value(x,1)  //主机出从机入
#define MOSI_L(x) gpio_set_value(x,0)

// #define MISO_H MISO=1  //主机入从机出
// #define MISO_L MISO=0

#define   CS_H(x)    gpio_set_value(x,1)
#define   CS_L(x)    gpio_set_value(x,0)

//单次转换开始
#define ADS1118_CONFIG_SS_START_MASK 0x8000
#define ADS1118_CONFIG_SS_START_ON   0x8000  //开始单次转换
#define ADS1118_CONFIG_SS_START_OFF  0x0000  // 无效
 
//输入多路复用器配置
#define ADS1118_CONFIG_MUX_MASK      0x7000
#define ADS1118_CONFIG_MUX_DIFF_0_1  0x0000  //AINP is AIN0 and AINN is AIN1 //差分
#define ADS1118_CONFIG_MUX_DIFF_0_3  0x1000  //AINP is AIN0 and AINN is AIN3
#define ADS1118_CONFIG_MUX_DIFF_1_3  0x2000  //AINP is AIN1 and AINN is AIN3
#define ADS1118_CONFIG_MUX_DIFF_2_3  0x3000  //AINP is AIN2 and AINN is AIN3
#define ADS1118_CONFIG_MUX_SINGLE_0  0x4000  //AINP is AIN0 and AINN is GND//单端
#define ADS1118_CONFIG_MUX_SINGLE_1  0x5000  //AINP is AIN1 and AINN is GND
#define ADS1118_CONFIG_MUX_SINGLE_2  0x6000  //AINP is AIN2 and AINN is GND
#define ADS1118_CONFIG_MUX_SINGLE_3  0x7000  //AINP is AIN3 and AINN is GND
	
//可编程增益放大器配置
#define ADS1118_CONFIG_PGA_MASK      0x0E00  //FSR is ±0.256 V
#define ADS1118_CONFIG_PGA_6_144V    0x0000  //FSR is ±6.144 V
#define ADS1118_CONFIG_PGA_4_096V    0x0200  //FSR is ±4.096 V
#define ADS1118_CONFIG_PGA_2_048V    0x0400  //FSR is ±2.048 V
#define ADS1118_CONFIG_PGA_1_024V    0x0600  //FSR is ±1.024 V
#define ADS1118_CONFIG_PGA_0_512V    0x0800  //FSR is ±0.512 V
#define ADS1118_CONFIG_PGA_0_256V    0x0A00  //FSR is ±0.256 V

//设备操作模式
#define ADS1118_CONFIG_MODE_MASK     0x0100
#define ADS1118_CONFIG_MODE_CONTIN   0x0000 //连续转换模式
#define ADS1118_CONFIG_MODE_SINGLE   0x0100 //掉电，单次转换模式

//数据速率
#define ADS1118_CONFIG_DR_MASK       0x00E0  //860SPS
#define ADS1118_CONFIG_DR_8SPS       0x0000
#define ADS1118_CONFIG_DR_16SPS      0x0020
#define ADS1118_CONFIG_DR_32SPS      0x0040
#define ADS1118_CONFIG_DR_64SPS      0x0060
#define ADS1118_CONFIG_DR_128SPS     0x0080  
#define ADS1118_CONFIG_DR_250SPS     0x00A0
#define ADS1118_CONFIG_DR_475SPS     0x00C0
#define ADS1118_CONFIG_DR_860SPS     0x00E0

//温度传感器模式
#define ADS1118_CONFIG_TS_MODE_MASK  0x0010
#define ADS1118_CONFIG_TS_MODE_ADC   0x0000  //ADC模式
#define ADS1118_CONFIG_TS_MODE_TEMP  0x0010  //温度传感器模式

//上拉使能
//仅当CS为高电平时，此位使能DOUT / DRDY引脚上的内部弱上拉电阻。 启用后，内部400kΩ电阻器将总线线路连接到电源。 禁用时，DOUT / DRDY引脚悬空。
#define ADS1118_CONFIG_PULL_UP_MASK  0x0008
#define ADS1118_CONFIG_PULL_UP_ON    0x0008  //上拉电阻启用
#define ADS1118_CONFIG_PULL_UP_OFF   0x0000  //上拉电阻禁用

//控制是否将数据写入配置寄存器
#define ADS1118_CONFIG_NOP_MASK      0x0006
#define ADS1118_CONFIG_NOP_VALID     0x0002  // 有效数据，更新Config寄存器
#define ADS1118_CONFIG_NOP_INVALID   0x0006  //无效数据，不更新Config寄存器的内容

//
#define ADS1118_CONFIG_RESV          0x0001  // default, required

//
#define ADS1118_CONST_C_PER_BIT      (0.03125)
#define ADS1118_CONST_BIT_PER_C      32

//
#define ADS1118_CONST_6_144V_LSB_mV  (0.1875)
#define ADS1118_CONST_4_096V_LSB_mV  (0.125)
#define ADS1118_CONST_2_048V_LSB_mV  (0.0625)
#define ADS1118_CONST_1_024V_LSB_mV  (0.03125)
#define ADS1118_CONST_0_512V_LSB_mV  (0.015625)
#define ADS1118_CONST_0_256V_LSB_mV  (0.0078125)

#endif