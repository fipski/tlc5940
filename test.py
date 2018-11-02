#!/usr/bin/python3

import math
import time

px = 2*12*4
cl = 4
gs = 1024

for b in range(0,gs+1,1):
    for i in range(3,px,4):
        brightness = int((math.sin(2*math.pi*b/gs+i*2*math.pi/px)/2+0.5)*gs)
        # print('LED: '+str(i)+'Brightness: '+str(brightness))
        with open('/sys/class/leds/led0-'+str(i)+'/brightness', 'a') as the_file:
            the_file.write(str(brightness))
            # time.sleep(0.001)

for i in range(0,px):
    with open('/sys/class/leds/led0-'+str(i)+'/brightness', 'a') as the_file:
        the_file.write('0')
