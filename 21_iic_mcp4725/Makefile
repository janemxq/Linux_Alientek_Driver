# 教程系统的
KERNELDIR := /home/alientek/alpha/alientek-zzk/kernel-zzk/linux-imx-rel_imx_4.1.15_2.1.0_ga_alientek
# 出厂系统的
# KERNELDIR := /home/alientek/alpha/alientek-alpha/kernel-alientek/linux-imx-4.1.15-2.1.0-gad512fa-v1.7
CURRENT_PATH := $(shell pwd)

obj-m := mcp4725.o

build: kernel_modules

kernel_modules:
	$(MAKE) -C $(KERNELDIR) M=$(CURRENT_PATH) modules

clean:
	$(MAKE) -C $(KERNELDIR) M=$(CURRENT_PATH) clean