#!/bin/sh

if [ "$1" = "start" ]; then
    echo "starting tlc5940 control"
    echo "...unexporting pwmchip0 line 0"
    echo 0 > /sys/class/pwm/pwmchip0/unexport #unexport pwm line 0
    echo "...loading tlc5940 module"
    modprobe tlc5940
    chown debian:debian /dev/fbuf
fi

if [ "$1" = "stop" ]; then
    echo "stopping tlc5940 control"
    echo "...exporting pwmchip0 line 0"
    echo 0 > /sys/class/pwm/pwmchip0/export
    echo "...unloading tlc5940 module"
    rmmod tlc5940
fi
