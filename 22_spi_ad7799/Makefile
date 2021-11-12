# 教程系统的
KERNELDIR := /home/alientek/alpha/alientek-zzk/kernel-zzk/linux-imx-rel_imx_4.1.15_2.1.0_ga_alientek
# 出厂系统的
# KERNELDIR := /home/alientek/alpha/alientek-alpha/kernel-alientek
CURRENT_PATH := $(shell pwd)
# 不编译到内核，动态加载
obj-m := ad7799.o

build: kernel_modules

kernel_modules:
	$(MAKE) -C $(KERNELDIR) M=$(CURRENT_PATH) modules

clean:
	$(MAKE) -C $(KERNELDIR) M=$(CURRENT_PATH) clean