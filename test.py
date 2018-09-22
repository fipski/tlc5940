#!/usr/bin/python3

import math
import time

px = 24
cl = 4
gs = 1024

for b in range(0,gs+1,64):
    for i in range(0,px):
        for j in range(0,cl):
            brightness = int(math.sin(2*math.pi*b/gs)/2+0.5*gs)
            with open('/sys/class/leds/led'+str(i)+'-'+str(j)+'/brightness', 'a') as the_file:
                the_file.write(str(brightness))
                time.sleep(0.02)


