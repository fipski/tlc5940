# TLC5940

Control Texas Instruments TLC5940 LED Drivers with a Beaglebone Black.

## Getting Started

The Beaglebone's Pinmux must be configured with a device tree overlay. The makefile is configuerd to do so, run i

```
$make devicetree.
```

The devicetree overlay script gets compiled, copy the result to /lib/firmware

```
$sudo cp BB-TLC5940-01-00A0.dtbo /lib/firmware
```

Add this file to /boot/uEnv.txt under the custom cape line:

```
dtb_overlay=/lib/firmware/BB-TLC5940-01-00A0.dtbo
```

### Prerequisites

to compile install linux-headers$(uname -r)

```
Give examples
```

### Installing

probe the module with

```
modprobe tlc5940.ko
```


## Control LEDs

You can modify the led brightness per led using the Linux LED subsystem in /sys/class/leds or write a framebuffer to a sysfs file in /dev/fbuf.


### LED Subsystem

Write a value to /sys/clas/leds/led0-0/brightness or add a trigger to the trigger file. Each brightness change will send all brightness values over spi.

### Framebuffer file 

Write brighness values as 12bit binary data. Data will be sent on file close. The file is atomic - can only be opened by one process.

