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
/***************************************************************
Copyright © ALIENTEK Co., Ltd. 1998-2029. All rights reserved.
文件名		: ap3216cApp.c
作者	  	: 左忠凯
版本	   	: V1.0
描述	   	: ap3216c设备测试APP。
其他	   	: 无
使用方法	 ：./ap3216cApp /dev/ap3216c
论坛 	   	: www.openedv.com
日志	   	: 初版V1.0 2019/9/20 左忠凯创建
***************************************************************/
void print_bin(unsigned char n)
{
    int i;
    for(i=7; i>=0; i --)
    {
        if(n&(1<<i))printf("1");
        else printf("0");
    }
}
/*
 * @description		: main主程序
 * @param - argc 	: argv数组元素个数
 * @param - argv 	: 具体参数
 * @return 			: 0 成功;其他 失败
 */
int main(int argc, char *argv[])
{
	int fd;
	char *filename;
	unsigned char databuf[2];
	unsigned char ir, als, ps;
	int ret = 0;
	

	if (argc != 2) {
		printf("Error Usage!\r\n");
		return -1;
	}

	filename = argv[1];
	fd = open(filename, O_RDWR);
	if(fd < 0) {
		printf("can't open file %s\r\n", filename);
		return -1;
	}else{
		printf("open file %s successs!!!\r\n", filename);
	}
    // databuf[0] = atoi(argv[2]);	/* 要执行的操作：打开或关闭 */
	ir = 0xAA;
	while (1) {
		ret = read(fd, databuf, sizeof(databuf));
		if(ret == 0) { 			/* 数据读取成功 */
		    // ir = 0xff;
		    ir =~ir;
			write(fd,&ir,1);
			
			printf("IOA(输出) = %x  IOB(输入) =",  databuf[0]);
			print_bin(databuf[1]);
			printf("\r\n");
		}
		usleep(200000); /*延时 1000ms */
	}
	close(fd);	/* 关闭文件 */	
	return 0;
}

