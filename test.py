#!/usr/bin/python3

import math
import time

ledStart = 47
# ledStart = 0
ledEnd = 96

def singleLed():
    for i in range(ledStart,ledEnd):
        for b in range(0,1024,8):
            with open('/sys/class/leds/led0-'+str(i)+'/brightness', 'a') as the_file:
                the_file.write(str(b))
        with open('/sys/class/leds/led0-'+str(i)+'/brightness', 'a') as the_file:
            the_file.write(str(0))

def setAll(brighntess):
    for i in range(ledStart,ledEnd):
        with open('/sys/class/leds/led0-'+str(i)+'/brightness', 'a') as the_file:
            the_file.write(str(brighntess))




while(True):
    setAll(100)
    time.sleep(0.5)
    setAll(1024)
    time.sleep(1)
    setAll(0)
    singleLed()
