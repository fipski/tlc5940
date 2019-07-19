obj-m += tlc5940.o

KDIR ?= /lib/modules/$(shell uname -r)/build

all:
	make -C $(KDIR) M=$(PWD) modules

clean:
	make -C $(KDIR) M=$(PWD) clean


devicetree:
	dtc -O dtb -o BB-TLC5940-01-00A0.dtbo -b 0 -@ BB-TLC5940-01-00A0.dts

install:
	systemctl stop tlc5940.service
	echo "> copy module"
	cp tlc5940.ko /lib/modules/$(shell uname -r)/
	echo "> copy device tree blob"
	cp BB-TLC5940-01-00A0.dtbo /lib/firmware/
	echo "> copy setup script"
	cp tlcsetup.sh /usr/bin/
	echo "> running depmod"
	depmod --quick
	echo "> copy unit file"
	cp tlc5940.service /etc/systemd/system/
	echo "> enable service"
	systemctl enable tlc5940.service
