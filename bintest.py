#!/usr/bin/python3

import math
import time

ledStart = 47
# ledStart = 0
ledEnd = 96
px = 10*48

frame = []
framebuffer = []

class FrameBuffer:
    """Access the tlc5940.ko Framebuffer"""
    frame = []
    fb = []
    px
    def __init__(self,px):
        self.px = px
        for i in range(px):
            self.frame.append(0)
        for i in range(int(px*3/2)):
            self.fb.append(0)
    def write(self):
        for i in range(self.px >> 1):
            br_cur = self.frame[2*i]
            br_next = self.frame[2*i +1]
            self.fb[3*i] = ((br_cur >> 4) & 0xff)
            self.fb[3*i+1] = ((br_cur & 0x0f) << 4) + ((br_next >> 8) & 0x0f)
            self.fb[3*i+2] = (br_next & 0xff)
        # print(', '.join(str(x) for x in self.fb))
        with open('/dev/fbuf', 'wb') as fb:
            fb.write(bytes(self.fb))


fb = FrameBuffer(px)
fb.write()

while(True):
    for i in range(0,2001):
        for px in range(0,fb.px,4):
            fb.frame[px+1] = int(2000*(1+math.sin(10.0*math.pi*i/1000+0.1*px+0)))
            fb.frame[px+2] = int(2000*(1+math.sin(10.0*math.pi*i/1000+0.1*px+math.pi/3)))
            fb.frame[px+3] = int(2000*(1+math.sin(10.0*math.pi*i/1000+0.1*px+math.pi*2/3)))
        fb.write()
        time.sleep(0.01)


# for i in range(px):
#     frame.append(0)
# fb_len = int(px*3/2)
# for i in range(fb_len):
#     framebuffer.append(0)

# white = range(0,px,4)
# blue = range(1,px,4)
# green = range(2,px,4)
# red = range(3,px,4)

# for i in range(px):
#     frame[i] = 0

# def fbWrite():
#     for i in range(px >> 1):
#         br_cur = frame[2*i]
#         br_next = frame[2*i +1]
#         framebuffer[3*i] = ((br_cur >> 4) & 0xff)
#         framebuffer[3*i+1] = ((br_cur & 0x0f) << 4) + ((br_next >> 8) & 0x0f)
#         framebuffer[3*i+2] = (br_next & 0xff)
#     print(', '.join(str(x) for x in framebuffer))
#     with open('/dev/fbuf', 'wb') as fb:
#         fb.write(bytes(framebuffer))



# while(True):
#     for i in red:
#         frame[i] = 100
#     fbWrite()
#     time.sleep(0.01)
