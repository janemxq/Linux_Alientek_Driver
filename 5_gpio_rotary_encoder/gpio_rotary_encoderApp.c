#include "stdio.h"
#include "unistd.h"
#include "sys/types.h"
#include "sys/stat.h"
#include "fcntl.h"
#include "stdlib.h"
#include "string.h"
#include <linux/input.h>
/***************************************************************
Copyright © ALIENTEK Co., Ltd. 1998-2029. All rights reserved.
文件名		: ledApp.c
作者	  	: 左忠凯
版本	   	: V1.0
描述	   	: chrdevbase驱测试APP。
其他	   	: 无
使用方法	 ：./ledtest /dev/led  0 关闭LED
		     ./ledtest /dev/led  1 打开LED		
论坛 	   	: www.openedv.com
日志	   	: 初版V1.0 2019/1/30 左忠凯创建
***************************************************************/

#define LEDOFF 	0
#define LEDON 	1
/* 定义一个input_event变量，存放输入事件信息 */
static struct input_event inputevent;
static int count=0;
static struct timeval oldtime;
/*
 * @description		: main主程序
 * @param - argc 	: argv数组元素个数
 * @param - argv 	: 具体参数
 * @return 			: 0 成功;其他 失败
 */
int main(int argc, char *argv[])
{
	int fd, retvalue;
	char *filename;
	unsigned char databuf[1];
	unsigned char value[1];
	int err = 0;
	if(argc != 2){
		printf("Error Usage!\r\n");
		return -1;
	}

	filename = argv[1];

	/* 打开encoder驱动 */
	fd = open(filename, O_RDWR);
	if(fd < 0){
		printf("file %s open failed!\r\n", argv[1]);
		return -1;
	}

    while(1)
	{
	   /* ----------------------内置驱动的循环读取数据 */
	//    err = read(fd, &inputevent, sizeof(inputevent));
	// 	if (err <=0) {
				
	// 			perror("read error");
	// 			exit(-1);
	// 	}
    //     if(inputevent.value==1)
	// 	{
	// 		count++;
	// 	}else if(inputevent.value==-1)
	// 	{
	// 		count--;
	// 	}
		// if(inputevent.time.tv_usec-oldtime.tv_usec<100 )
		// {
        //      printf("计数值 =%d 时间=%05d %05d %05d微秒 type:%d code:%d value:%d\n",count,
		// 	   inputevent.time.tv_sec,oldtime.tv_sec,inputevent.time.tv_usec-oldtime.tv_usec,inputevent.type, inputevent.code, inputevent.value);
		//      oldtime=inputevent.time;
		// }
		// printf("计数值 =%d 时间=%05d秒%05d微妙 type:%d code:%d value:%d\n",count,
		// 	inputevent.time.tv_sec,inputevent.time.tv_usec,inputevent.type, inputevent.code, inputevent.value);
		// oldtime=inputevent.time;

        // ------------------------- 自己的驱动
		retvalue = read(fd, value, sizeof(value));
		if(retvalue < 0){
			printf("encoder read Failed!\r\n");
			close(fd);
			return -1;
		}
		// sleep(1);
	}

	retvalue = close(fd); /* 关闭文件 */
	if(retvalue < 0){
		printf("file %s close failed!\r\n", argv[1]);
		return -1;
	}
	return 0;
}
