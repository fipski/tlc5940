#!/usr/bin/python3

import math
import time

ledStart = 47
# ledStart = 0
ledEnd = 96
px = 96

frame = []
framebuffer = []


for i in range(px):
    frame.append(0)
fb_len = int(px*3/2)
for i in range(fb_len):
    framebuffer.append(0)

for i in range(px):
    if i % 4 == 2:
        frame[i] = 100
    else:
        frame[i] = 0




def fbWrite():
    for i in range(px >> 1):
        br_cur = frame[2*i]
        br_next = frame[2*i +1]
        framebuffer[3*i] = ((br_cur >> 4) & 0xff)
        framebuffer[3*i+1] = ((br_cur & 0x0f) << 4) + ((br_next >> 8) & 0x0f)
        framebuffer[3*i+2] = (br_next & 0xff)
    print(', '.join(str(x) for x in framebuffer))
    with open('/dev/fbuf', 'wb') as fb:
        fb.write(bytes(framebuffer))


while(True):
    fbWrite()
    time.sleep(0.5)
