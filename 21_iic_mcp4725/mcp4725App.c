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
	

	if (argc != 3) {
		printf("Error Usage! 需要输入电压值，0-4096 对应 0-3.3v \r\n");
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
	// 3.266v 到达4088已经到达满量程。
    databuf[1] = atoi(argv[2])>>8;	/* 输入电压值，0-4095 对应 0-3.3v*/
    databuf[0] = atoi(argv[2])&0xFF;	/* 输入电压值，0-4095 对应 0-3.3v*/
	while (1) {
		ret = write(fd, databuf, 2);
		// if(ret == 0) { 			/* 数据读取成功 */
			// write(fd,&databuf[0],1);//把A狀態寫到B
			// printf("MCP4725 返回值=%04X\r\n",ret);
		// }
		sleep(2); /*延时 2000ms */
	}
	close(fd);	/* 关闭文件 */	
	return 0;
}

