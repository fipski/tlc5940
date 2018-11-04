#!/usr/bin/python3

import math
import time


for i in range(0,2000):
    brightness = 100
    with open('/sys/class/leds/led0-0/brightness', 'a') as the_file:
        the_file.write(str(brightness))
        time.sleep(0.02)
