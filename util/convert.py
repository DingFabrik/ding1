#!/usr/bin/python
from __future__ import division
import struct
import serdecode
import math

outfile = open("softscope.fifo", "w")

serdecode.resync()

while True:
    s = serdecode.read_frame()
    
    (pos, power, maximum,minimum) = struct.unpack("=hhhh", s)
    outfile.write(struct.pack("=ffff", pos, power, maximum, minimum))
    outfile.flush()

outfile.close()
