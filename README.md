# TLC5940

Control Texas Instruments TLC5940 LED Drivers with a Beaglebone Black. 
Based on [gerbert / tlc5940](https://github.com/gerbert/tlc5940) and [jyelloz / tlc5940-linux](https://github.com/jyelloz/tlc5940-linux).


## Prerequisites

To install linux-headers uncomment kernel sources in `/etc/apt/sources.list` and install the headers `#apt install linux-headers$(uname -r)`.

## Getting Started

Compile the kernel module with `$make all`.

The Beaglebone's Pinmux must be configured with a device tree overlay. To compile the Device Tree Source run `$make devicetree`.

Copy the kernel module to `/lib/module/$(uname -r)/` and the device tree blob (.dtb) to `/lib/firmware` by running `#make install`.
This also copies a control script (`tlcsetup.sh`) to `/usr/bin/` and a systemd .service file (`tlc5940.service`) to `/etc/systemd/system/`.

Add the device tree file to /boot/uEnv.txt under the custom cape line:

```
dtb_overlay=/lib/firmware/BB-TLC5940-01-00A0.dtbo
```


## Control LEDs

You can modify the led brightness per led using the Linux LED subsystem in `/sys/class/leds` or write a framebuffer to a sysfs file in `/dev/fbuf`.

### LED Subsystem

Write a value to /sys/clas/leds/led0-0/brightness or add a trigger to the trigger file. Each brightness change will send all brightness values over spi - good for testing, but bad writing performance tow write a whole frame.

### Framebuffer file 

Write brighness values as 12bit binary data. Data will be sent on file close. The file is atomic, so it can only be opened by one process.

## TODO

Each dimming cycle is ended by a BLANK signal. At the moment Blank is created by a hrtimer in software, so it has a lot of jitter. The brightness control should be done by the PRU coprocessor.
