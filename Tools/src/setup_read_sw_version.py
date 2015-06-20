#!/usr/bin/python

"""
Utility for reading the software version from a 3DR Gimbal.

"""

import struct
import setup_param
import time
from setup_param import commit_to_flash

def float_to_bytes(f):
    return struct.unpack('4b', struct.pack('<f', f))

def uint32_to_float(i):
    return struct.unpack('f', struct.pack('<I', i))[0]

def float_to_uint32(f):
    return struct.unpack('I',struct.pack('<f',f))[0]

def readSWver(link):
    msg = setup_param.get_SWVER_param(link)
    if not msg:
        return None
    else:
        swver_raw = float_to_bytes(msg.param_value)
        return "%i.%i.%i" % (swver_raw[3], swver_raw[2], swver_raw[1])

def get_assembly_time(link):
    value = setup_param.fetch_param(link, "GMB_ASM_TIME").param_value
    asm_time = float_to_uint32(value)
    if(asm_time == 0):
        asm_time = None    
    return asm_time


def setup_assembly_date(link):
    timestamp =  time.time()
    setup_param.set_param(link, "GMB_ASM_TIME", uint32_to_float(timestamp))
    commit_to_flash(link)
    print "Assembly date set to %s" % (time.ctime(timestamp))



