obj-m += lt8500.o

export ARCH=arm
export CROSS_COMPILE=/opt/gcc-linaro-4.9-2014.11-x86_64_arm-linux-gnueabihf/bin/arm-linux-gnueabihf-

KDIR ?= /home/andrei/src/work/BeagleBoneBlack/Kernel/ti-linux-kernel

all: clean
	make -C $(KDIR) M=$(PWD) modules

clean:
	make -C $(KDIR) M=$(PWD) clean
