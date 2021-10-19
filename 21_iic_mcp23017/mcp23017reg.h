#ifndef mcp23017_H
#define mcp23017_H
/***************************************************************
Copyright © ALIENTEK Co., Ltd. 1998-2029. All rights reserved.
文件名		: mcp23017reg.h
作者	  	: 左忠凯
版本	   	: V1.0
描述	   	: mcp23017寄存器地址描述头文件
其他	   	: 无
论坛 	   	: www.openedv.com
日志	   	: 初版V1.0 2019/9/2 左忠凯创建
***************************************************************/

#define mcp23017_ADDR    	0X20	/* mcp23017器件地址  */

//-------------------------------定义MCP017内部寄存器地址------------------------------ 

#define MCP23017_IODIR 			0x00
#define MCP23017_IPOL 			0x02
#define MCP23017_GPINTEN 		0x04
#define MCP23017_DEFVAL 		0x06
#define MCP23017_INTCON 		0x08
#define MCP23017_IOCON 			0x0A
#define MCP23017_GPPU 			0x0C
#define MCP23017_INTF 			0x0E
#define MCP23017_INTCAP 		0x10
#define MCP23017_GPIO 			0x12
#define MCP23017_OLAT 			0x14

//-------------------------------定义辅助宏定义--------------------------------------- 
#define MCP23017_PORTA			0x00
#define MCP23017_PORTB			0x01

//定义输入输出
#define INPUT					0x01
#define OUTPUT					0x00
//定义使能或除能
#define ENABLE					0x01
#define DISABLE					0x00

#define SET						0x01
#define RESET					0x00
//定义是否对端口极性取反
//当设置为取反时，读取的端口状态与实际输入状态极性相反
#define POLARITY_REVERSE		0x01
#define POLARITY_NON_REVERSE	0x00

#define PIN0					0x01
#define PIN1					0x02
#define PIN2					0x04
#define PIN3					0x08
#define PIN4					0x10
#define PIN5					0x20
#define PIN6					0x40
#define PIN7					0x80
#define ALLPIN					0xFF

//定义中断的类型
#define DIS_INTERRUPT			0x00	   //关闭中断
#define HIGHLEVEL_INTERRUPT		0x01	   //高电平中断
#define LOWLEVEL_INTERRUPT		0x02	   //低电平中断
#define CHANGE_INTERRUPT		0x03	   //电平变化中断

//定义INTA、INTB是否关联
#define INTA_INTB_CONJUNCTION	0x00	   //AB关联
#define INTA_INTB_INDEPENDENT	0x01	   //AB独立

//定义硬件地址是否使能
#define HWA_EN					0x00	   //A0、A1、A2、硬件地址使能
#define HWA_DIS					0x01	   //A0、A1、A2、硬件地址禁用，

//定义INTA、INTB引脚的输出类型
#define INT_OD					0x00	   //开漏输出
#define INT_PUSHPULL_HIGH		0x01	   //推挽输出，高电平有效
#define INT_PUSHPULL_LOW		0x02	   //推挽输出，低电平有效



#endif

