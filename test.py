#!/usr/bin/python

import time

for i in range(0,4095/2):
    with open ('/sys/class/leds/led31-15/brightness', 'w') as f:
        f.write(str(i*2))
    time.sleep(0.02)

with open ('/sys/class/leds/led31-15/brightness', 'w') as f:
    f.write(str('0'))
