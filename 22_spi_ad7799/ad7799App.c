#include "stdio.h"
#include "unistd.h"
#include "sys/types.h"
#include "sys/stat.h"
#include "sys/ioctl.h"
#include "fcntl.h"
#include "stdlib.h"
#include "string.h"
#include <poll.h>
#include <sys/select.h>
#include <sys/time.h>
#include <signal.h>
#include <fcntl.h>
#define   AD7799_GAIN  128					//如果增益为64倍,则这里改为64
#define   AD7799_CHIP_GAIN  AD7799_GAIN_2 	//如果增益为64倍,则这里改为AD7799_GAIN_64

#define   AD7799_RefmV    3300				//基准电压 3300mV	

double analyzeAD7799_Data(long data);
/***************************************************************
Copyright © ALIENTEK Co., Ltd. 1998-2029. All rights reserved.
文件名		: icm20608App.c
作者	  	: 左忠凯
版本	   	: V1.0
描述	   	: icm20608设备测试APP。
其他	   	: 无
使用方法	 ：./icm20608App /dev/icm20608
论坛 	   	: www.openedv.com
日志	   	: 初版V1.0 2019/9/20 左忠凯创建
***************************************************************/

/*
 * @description		: main主程序
 * @param - argc 	: argv数组元素个数
 * @param - argv 	: 具体参数
 * @return 			: 0 成功;其他 失败
 */
// int main(int argc, char *argv[])
// {
// 	int fd;
// 	char *filename;
// 	unsigned char data[9];
// 	long lADValue[3];
// 	double ADValues[3];
// 	int ch=0;

// 	int ret = 0;

// 	if (argc != 2) {
// 		printf("Error Usage!\r\n");
// 		return -1;
// 	}

// 	filename = argv[1];
// 	fd = open(filename, O_RDWR);
// 	if(fd < 0) {
// 		printf("can't open file %s\r\n", filename);
// 		return -1;
// 	}

// 	while (1) {
// 		ret = read(fd, data, 1);
// 		printf("ret =%d \r\n",ret);
// 		if(ret == 0) { 	/* 数据读取成功 */
// 			for(ch=0;ch<3;ch++)
// 			{
// 			 lADValue[ch]=0;
//              lADValue[ch] += (data[3*ch] << 16);
// 		     lADValue[ch] += (data[3*ch+1] << 8);
// 		     lADValue[ch] += (data[3*ch+2] << 0);
// 			 ADValues[ch]=analyzeAD7799_Data(lADValue[ch]);
// 			}		
// 			printf("\r\n原始值:\r\n");
// 			printf("通道1 = %06X, 通道2 = %06X, 通道3 = %06X\r\n",  lADValue[0],  lADValue[1],  lADValue[2]);
// 			printf("实际值:");
// 			printf("通道1 = %.2f 毫伏, 通道2= %.2f 毫伏, 通道3 = %.2f 毫伏\r\n", ADValues[0], ADValues[1], ADValues[2]);
// 		}
// 		usleep(1000000); /*100ms */
// 	}
// 	close(fd);	/* 关闭文件 */	
// 	return 0;
// }
double analyzeAD7799_Data(long data)
{
	long value = (data - 0X800000);
	return (float)((float)value*(float)AD7799_RefmV)/(0X800000*AD7799_GAIN);	//0X800000:2.048V    0X000000:0V

}
int main(int argc, char *argv[])
{
	int fd;
	char *filename;
	unsigned int databuf[9];//必须和驱动里面read函数的类型保持一致，不然会卡死!!!!
	signed int gyro_x_adc, gyro_y_adc, gyro_z_adc;
	signed int accel_x_adc, accel_y_adc, accel_z_adc;
	signed int temp_adc;
    long lADValue[3];
	double ADValues[3];
	float gyro_x_act, gyro_y_act, gyro_z_act;
	float accel_x_act, accel_y_act, accel_z_act;
	float temp_act;
    char chip=0;
	int ret = 0;
    int ch=0;
	if (argc != 2) {
		printf("Error Usage!\r\n");
		return -1;
	}

	filename = argv[1];
	fd = open(filename, O_RDWR);
	if(fd < 0) {
		printf("can't open file %s\r\n", filename);
		return -1;
	}
	while (1) {
		
		
		for(chip=0;chip<1;chip++)
		{
			//先把访问的芯片号告诉驱动
			ret = write(fd, &chip, 1);
			ret = read(fd, databuf, sizeof(databuf));
			if(ret == 0) { 			/* 数据读取成功 */
				for(ch=0;ch<3;ch++)
				{
					lADValue[ch]=0;
					lADValue[ch] += (databuf[3*ch] << 16);
					lADValue[ch] += (databuf[3*ch+1] << 8);
					lADValue[ch] += (databuf[3*ch+2] << 0);
					ADValues[ch]=analyzeAD7799_Data(lADValue[ch]);
				}		
				printf("\r\n原始值:\r\n");
				printf("通道1 = %06X, 通道2 = %06X, 通道3 = %06X\r\n",  lADValue[0],  lADValue[1],  lADValue[2]);
				printf("实际值:");
				printf("通道1 = %.2f 毫伏, 通道2= %.2f 毫伏, 通道3 = %.2f 毫伏\r\n", ADValues[0], ADValues[1], ADValues[2]);
				usleep(1000000); /*100ms */
		    }
		  
		}
		
	}
	close(fd);	/* 关闭文件 */	
	return 0;
}