#!/usr/bin/fish 
echo $1 > /sys/class/leds/led*-0/brightness
echo $1 > /sys/class/leds/led*-4/brightness
echo $1 > /sys/class/leds/led*-8/brightness
echo $1 > /sys/class/leds/led*-12/brightness
