obj-m += bmp180_driver.o
KDIR = /lib/modules/$(shell uname -r)/build

all:
	make -C $(KDIR) M=$(shell pwd) modules EXTRA_CFLAGS="-Wno-error=incompatible-pointer-types"
clean: 
	make -C $(KDIR) M=$(shell pwd) clean