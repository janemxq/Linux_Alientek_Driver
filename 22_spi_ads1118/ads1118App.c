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
#include   <time.h>
#include "ads1118reg.h"

#define   IS_128
#define   AD7799_GAIN  128					//如果增益为64倍,则这里改为64

#define   AD7799_RefmV    3300				//基准电压 3300mV	

float analyzeAD1118_mv(long data)
{
	return data*ADS1118_CONST_6_144V_LSB_mV;//
}
int main(int argc, char *argv[])
{
	int fd,fd_rec;
	char *filename;
	int16_t databuf[5];//必须和驱动里面read函数的类型保持一致，不然会卡死!!!!
	unsigned char recbuf[256];
	struct tm *t;
    time_t tt;
    
    // printf("%4d年%02d月%02d日 %02d:%02d:%02d\n", t->tm_year + 1900, t->tm_mon + 1, t->tm_mday, t->tm_hour, t->tm_min, t->tm_sec);
	signed int gyro_x_adc, gyro_y_adc, gyro_z_adc;
	signed int accel_x_adc, accel_y_adc, accel_z_adc;
	signed int temp_adc;
    long lADValue[4];
	float ADValues[5];
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
	// fd_rec=open("/run/media/mmcblk0p2/scale_rec.txt",O_CREAT|O_RDWR);
	// if(fd_rec<0)
	// {
	// 	printf("can't open file %s\r\n", "/run/media/mmcblk0p2/scale_rec.txt");
	// 	return -1;
	// }
	fd = open(filename, O_RDWR);
	if(fd < 0) {
		printf("can't open file %s\r\n", filename);
		return -1;
	}
	while (1) {
		for(chip=0;chip<1;chip++)
		{
			//先把访问的芯片号告诉驱动
			// ret = write(fd, &chip, 1);
			ret = read(fd, databuf, sizeof(databuf));
			if(ret == 0) { 			/* 数据读取成功 */
				for(ch=0;ch<4;ch++)
				{
					ADValues[ch]=analyzeAD1118_mv(databuf[ch]);
				}
				ADValues[4]=databuf[4]*0.03125;
				time(&tt);
                t = localtime(&tt);		
				sprintf(recbuf,"[%4d年%02d月%02d日 %02d:%02d:%02d] 通道1 = %0.2f mv, 通道2 = %0.2f mv 通道3 = %0.2f mv, 通道4 = %0.2f mv 温度 = %0.2f\r\n",  
				 t->tm_year + 1900, t->tm_mon + 1, t->tm_mday, t->tm_hour, t->tm_min, t->tm_sec,
				 ADValues[0], ADValues[1] , ADValues[2], ADValues[3],ADValues[4]);
				printf(recbuf);
			// 	// write(fd_rec, recbuf, strlen(recbuf));
			// 	// printf("通道1 = %06d 克, 通道2= %06d 克\r\n", ADValues[0], ADValues[1]);
			// 	// usleep(100000); /*100ms */
			// 	sleep(1);/*1s*/
		    }
			sleep(1);/*1s*/
		}
	}
	close(fd);	/* 关闭文件 */	
	return 0;
}