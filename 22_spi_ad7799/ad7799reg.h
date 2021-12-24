#ifndef AD7799_H
#define AD7799_H
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

#define uint16_t  unsigned short
#define uint32_t  unsigned long
#define uint8_t  unsigned char


#define AD7799_INTERFACE_GPIO		0			//gpio模拟接口
#define AD7799_INTERFACE_SPI1		1			//spi1接口

#define AD7799_INTERFACE_MODE		AD7799_INTERFACE_SPI1	 //当前为spi1接口通信



#define  AD_CS_PIN     GPIO_PIN_3
#define  AD_CS_GPIO    GPIOB

#define AD7799_CS_LOW  AD_CS_0()
#define AD7799_CS_HIGH  AD_CS_1()


/******************通信接口为AD7799_INTERFACE_GPIO模式时,才需要设置下面**********************/
#define  AD_DI_PIN     GPIO_PIN_5
#define  AD_DI_GPIO    GPIOB

#define  AD_SCK_PIN    GPIO_PIN_15
#define  AD_SCK_GPIO   GPIOA

#define  AD_DO_PIN     GPIO_PIN_4
#define  AD_DO_GPIO    GPIOB


#define  AD_CS_0()	 HAL_GPIO_WritePin(AD_CS_GPIO,AD_CS_PIN,GPIO_PIN_RESET)			 
#define  AD_CS_1()	 HAL_GPIO_WritePin(AD_CS_GPIO,AD_CS_PIN,GPIO_PIN_SET)

#define  AD_DI_0()	 HAL_GPIO_WritePin(AD_DI_GPIO,AD_DI_PIN,GPIO_PIN_RESET)
#define  AD_DI_1()	 HAL_GPIO_WritePin(AD_DI_GPIO,AD_DI_PIN,GPIO_PIN_SET) 

#define  AD_SCK_0()	 HAL_GPIO_WritePin(AD_SCK_GPIO,AD_SCK_PIN,GPIO_PIN_RESET)		 
#define  AD_SCK_1()	 HAL_GPIO_WritePin(AD_SCK_GPIO,AD_SCK_PIN,GPIO_PIN_SET) 

#define  AD_DO     (HAL_GPIO_ReadPin(AD_DO_GPIO, AD_DO_PIN))

#define ADC_RDY_DAT (AD_DO)

/******************通信接口为AD7799_INTERFACE_GPIO模式时,才需要设置上面**********************/


/*AD7799 Registers*/
#define AD7799_REG_COMM		0 /* Communications Register(WO, 8-bit) */
#define AD7799_REG_STAT	    0 /* Status Register	    (RO, 8-bit) */
#define AD7799_REG_MODE	    1 /* Mode Register	     	(RW, 16-bit */
#define AD7799_REG_CONF	    2 /* Configuration Register (RW, 16-bit)*/
#define AD7799_REG_DATA	    3 /* Data Register	     	(RO, 16-/24-bit) */
#define AD7799_REG_ID	    4 /* ID Register	     	(RO, 8-bit) */
#define AD7799_REG_IO	    5 /* IO Register	     	(RO, 8-bit) */
#define AD7799_REG_OFFSET   6 /* Offset Register	    (RW, 24-bit */
#define AD7799_REG_FULLSALE	7 /* Full-Scale Register	(RW, 24-bit */

/* Communications Register Bit Designations (AD7799_REG_COMM) */
#define AD7799_COMM_WEN		(1 << 7) 			/* Write Enable */
#define AD7799_COMM_WRITE	(0 << 6) 			/* Write Operation */
#define AD7799_COMM_READ    (1 << 6) 			/* Read Operation */
#define AD7799_COMM_ADDR(x)	(((x) & 0x7) << 3)	/* Register Address */
#define AD7799_COMM_CREAD	(1 << 2) 			/* Continuous Read of Data Register */

/* Status Register Bit Designations (AD7799_REG_STAT) */
#define AD7799_STAT_RDY		(1 << 7) /* Ready */
#define AD7799_STAT_ERR		(1 << 6) /* Error (Overrange, Underrange) */
#define AD7799_STAT_CH3		(1 << 2) /* Channel 3 */
#define AD7799_STAT_CH2		(1 << 1) /* Channel 2 */
#define AD7799_STAT_CH1		(1 << 0) /* Channel 1 */

/* Mode Register Bit Designations (AD7799_REG_MODE) */
#define AD7799_MODE_SEL(x)		(((x) & 0x7) << 13)	/* Operation Mode Select */
#define AD7799_MODE_PSW(x)		(1 << 12)			/* Power Switch Control Bit */	
#define AD7799_MODE_RATE(x)		((x) & 0xF) 		/* Filter Update Rate Select */

/* AD7799_MODE_SEL(x) options */
// #define AD7799_MODE_CONT		 0 /* Continuous Conversion Mode */
// #define AD7799_MODE_SINGLE		 1 /* Single Conversion Mode */
// #define AD7799_MODE_IDLE		 2 /* Idle Mode */
// #define AD7799_MODE_PWRDN		 3 /* Power-Down Mode */
// #define AD7799_MODE_CAL_INT_ZERO 4 /* Internal Zero-Scale Calibration */
// #define AD7799_MODE_CAL_INT_FULL 5 /* Internal Full-Scale Calibration */
// #define AD7799_MODE_CAL_SYS_ZERO 6 /* System Zero-Scale Calibration */
// #define AD7799_MODE_CAL_SYS_FULL 7 /* System Full-Scale Calibration */

/* Configuration Register Bit Designations (AD7799_REG_CONF) */
#define AD7799_CONF_BO_EN	  (1 << 13) 			/* Burnout Current Enable */
#define AD7799_CONF_UNIPOLAR  (1 << 12) 			/* Unipolar/Bipolar Enable */
#define AD7799_CONF_GAIN(x)	  (((x) & 0x7) << 8) 	/* Gain Select */
#define AD7799_CONF_REFDET(x) (((x) & 0x1) << 5) 	/* Reference detect function */
#define AD7799_CONF_BUF		  (1 << 4) 				/* Buffered Mode Enable */
#define AD7799_CONF_CHAN(x)	  ((x) & 0x7) 			/* Channel select */
#define AD7799_CONF_POLAR(x)  (((x) & 0x1) << 12)
/* AD7799_CONF_GAIN(x) options */
// #define AD7799_GAIN_1       0
// #define AD7799_GAIN_2       1
// #define AD7799_GAIN_4       2
// #define AD7799_GAIN_8       3
// #define AD7799_GAIN_16      4
// #define AD7799_GAIN_32      5
// #define AD7799_GAIN_64      6
// #define AD7799_GAIN_128     7

/* AD7799_CONF_REFDET(x) options */
#define AD7799_REFDET_ENA   1	
#define AD7799_REFDET_DIS   0

/* AD7799_CONF_CHAN(x) options */
// #define AD7799_CH_AIN1P_AIN1M	0 /* AIN1(+) - AIN1(-) */
// #define AD7799_CH_AIN2P_AIN2M	1 /* AIN2(+) - AIN2(-) */
// #define AD7799_CH_AIN3P_AIN3M	2 /* AIN3(+) - AIN3(-) */
// #define AD7799_CH_AIN1M_AIN1M	3 /* AIN1(-) - AIN1(-) */
// #define AD7799_CH_AVDD_MONITOR	7 /* AVDD Monitor */

/* ID Register Bit Designations (AD7799_REG_ID) */
#define AD7799_ID			0x9
#define AD7799_ID_MASK		0xF

/* IO (Excitation Current Sources) Register Bit Designations (AD7799_REG_IO) */
#define AD7799_IOEN			(1 << 6)
#define AD7799_IO1(x)		(((x) & 0x1) << 4)
#define AD7799_IO2(x)		(((x) & 0x1) << 5)
/**
 * Enum to set sample rate
 */
typedef enum {
	AD7799_RATE_470HZ = 1, /**< AD7799_RATE_470HZ */
	AD7799_RATE_242HZ, /**< AD7799_RATE_242HZ */
	AD7799_RATE_123HZ, /**< AD7799_RATE_123HZ */
	AD7799_RATE_62HZ, /**< AD7799_RATE_62HZ */
	AD7799_RATE_50HZ, /**< AD7799_RATE_50HZ */
	AD7799_RATE_39HZ, /**< AD7799_RATE_39HZ */
	AD7799_RATE_33_2HZ, /**< AD7799_RATE_33_2HZ */
	AD7799_RATE_19_6HZ_90DB,/**< AD7799_RATE_19_6HZ_90DB */
	AD7799_RATE_16_7HZ_80DB,/**< AD7799_RATE_16_7HZ_80DB */
	AD7799_RATE_16_7HZ_65DB,/**< AD7799_RATE_16_7HZ_65DB */
	AD7799_RATE_12_5HZ_66DB,/**< AD7799_RATE_12_5HZ_66DB */
	AD7799_RATE_10HZ_69DB, /**< AD7799_RATE_10HZ_69DB */
	AD7799_RATE_8_33HZ_70DB,/**< AD7799_RATE_8_33HZ_70DB */
	AD7799_RATE_6_25HZ_72DB,/**< AD7799_RATE_6_25HZ_72DB */
	AD7799_RATE_4_17HZ_74DB /**< AD7799_RATE_4_17HZ_74DB */
} AD7799_Rate;

static const uint16_t settle_time_ms[] = { 0, 4, 8, 16, 32, 40, 48, 60, 101,
		120, 120, 160, 200, 240, 320, 480 };

/**
 * Enum to set functioning mode
 */
typedef enum {
	AD7799_MODE_CONT = 0, /**< AD7799_MODE_CONT */
	AD7799_MODE_SINGLE, /**< AD7799_MODE_SINGLE */
	AD7799_MODE_IDLE, /**< AD7799_MODE_IDLE */
	AD7799_MODE_PWRDN, /**< AD7799_MODE_PWRDN */
	AD7799_MODE_CAL_INT_ZERO,/**< AD7799_MODE_CAL_INT_ZERO */
	AD7799_MODE_CAL_INT_FULL,/**< AD7799_MODE_CAL_INT_FULL */
	AD7799_MODE_CAL_SYS_ZERO,/**< AD7799_MODE_CAL_SYS_ZERO */
	AD7799_MODE_CAL_SYS_FULL /**< AD7799_MODE_CAL_SYS_FULL */
} AD7799_Mode;

/**
 * Enum to set channel
 */
typedef enum {
	AD7799_CH_AIN1P_AIN1M = 0,/**< AD7799_CH_AIN1P_AIN1M */
	AD7799_CH_AIN2P_AIN2M, /**< AD7799_CH_AIN2P_AIN2M */
	AD7799_CH_AIN3P_AIN3M, /**< AD7799_CH_AIN3P_AIN3M */
	AD7799_CH_AIN1M_AIN1M, /**< AD7799_CH_AIN1M_AIN1M */
	AD7799_CH_AVDD_MONITOR = 7/**< AD7799_CH_AVDD_MONITOR */
} AD7799_Channel;

/**
 * Enum to set PGA Gain
 */
typedef enum {
	AD7799_GAIN_1 = 0,/**< AD7799_GAIN_1 */
	AD7799_GAIN_2, /**< AD7799_GAIN_2 */
	AD7799_GAIN_4, /**< AD7799_GAIN_4 */
	AD7799_GAIN_8, /**< AD7799_GAIN_8 */
	AD7799_GAIN_16, /**< AD7799_GAIN_16 */
	AD7799_GAIN_32, /**< AD7799_GAIN_32 */
	AD7799_GAIN_64, /**< AD7799_GAIN_64 */
	AD7799_GAIN_128 /**< AD7799_GAIN_128 */
} AD7799_Gain;

/**
 * enum to set polarity
 */
typedef enum {
	AD7799_BIPOLAR = 0,/**< AD7799_BIPOLAR */
	AD7799_UNIPOLAR /**< AD7799_UNIPOLAR */
} AD7799_Polarity;


// __init  AD7799_Init(struct ad7799_dev *ad7799);

// void AD7799_Reset(struct ad7799_dev *ad7799);

// uint32_t AD7799_GetRegisterValue(struct ad7799_dev *ad7799, uint8_t regAddress,
// 		uint8_t size);

// void AD7799_SetRegisterValue(struct ad7799_dev *ad7799, uint8_t regAddress,
// 		uint32_t regValue, uint8_t size);

// void AD7799_SetRate(struct ad7799_dev *ad7799, AD7799_Rate rate);

// void AD7799_SetMode(struct ad7799_dev *ad7799, AD7799_Mode mode);

// uint8_t AD7799_Ready(struct ad7799_dev *ad7799);

// void AD7799_SetChannel(struct ad7799_dev *ad7799, AD7799_Channel channel);

// void AD7799_SetGain(struct ad7799_dev *ad7799, AD7799_Gain gain);

// void AD7799_SetReference(struct ad7799_dev *ad7799, uint8_t state);

// void AD7799_SetPolarity(struct ad7799_dev *ad7799, AD7799_Polarity polarity);

// void AD7799_SingleConversion(struct ad7799_dev *ad7799);

// void AD7799_RawToVolt(struct ad7799_dev *ad7799);




#endif

