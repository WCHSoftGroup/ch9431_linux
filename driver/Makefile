CONFIG_MODULE_SIG=n

ifeq ($(KERNELRELEASE), )
KERNELDIR := /lib/modules/$(shell uname -r)/build
PWD :=$(shell pwd)
default:
	$(MAKE) -C $(KERNELDIR)  M=$(PWD)  
clean:
	rm -rf *.mk .tmp_versions Module.symvers *.mod.c *.o *.ko .*.cmd Module.markers modules.order *.a *.mod
load:
	insmod ch9431.ko
unload:
	rmmod ch9431
install: default
	rmmod ch9431 || true
	insmod ch9431.ko || true
	mkdir -p /lib/modules/$(shell uname -r)/kernel/drivers/net/can/spi/ || true
	cp -f ./ch9431.ko /lib/modules/$(shell uname -r)/kernel/drivers/net/can/spi/ || true
	depmod -a
uninstall:
	rmmod ch9431 || true
	rm -rf /lib/modules/$(shell uname -r)/kernel/drivers/net/can/spi/ch9431.ko || true
	depmod -a
else
	obj-m := ch9431.o
endif
