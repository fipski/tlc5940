obj-m += tlc5940.o

KDIR ?= /lib/modules/$(shell uname -r)/build

all:
	make -C $(KDIR) M=$(PWD) modules

clean:
	make -C $(KDIR) M=$(PWD) clean


devicetree:
	dtc -O dtb -o BB-TLC5940-01-00A0.dtbo -b 0 -@ BB-TLC5940-01-00A0.dts

