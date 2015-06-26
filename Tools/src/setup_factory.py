#!/usr/bin/python

"""
Utility for reading the software version from a 3DR Gimbal.

"""

import struct
import setup_param
import time
from setup_param import commit_to_flash
import datetime

def float_to_bytes4(f):
    return struct.unpack('4b', struct.pack('<f', f))

def uint32_to_float(i):
    return struct.unpack('f', struct.pack('<I', i))[0]

def float_to_uint32(f):
    return struct.unpack('I',struct.pack('<f',f))[0]

def string12_to_float3(s):
    return struct.unpack('3f',struct.pack('<12s', s))

def float3_to_string12(f1,f2,f3):
    return struct.unpack('12s',struct.pack('<3f', f1, f2, f3))[0]

def readSWver(link):
    msg = setup_param.get_SWVER_param(link)
    if not msg:
        return None
    else:
        return float_to_bytes4(msg.param_value)[1:]

def get_assembly_time(link):
    value = setup_param.fetch_param(link, "GMB_ASM_TIME")
    if value:
        return float_to_uint32(value.param_value)
    return None

def set_assembly_date(link):
    timestamp = time.time()
    setup_param.set_param(link, "GMB_ASM_TIME", uint32_to_float(timestamp))
    commit_to_flash(link)
    return timestamp

def set_serial_number(link, serial_str):    
    ser_num_1, ser_num_2, ser_num_3 = string12_to_float3(serial_str)
    setup_param.set_param(link, "GMB_SER_NUM_1", ser_num_1)
    setup_param.set_param(link, "GMB_SER_NUM_2", ser_num_2)
    setup_param.set_param(link, "GMB_SER_NUM_3", ser_num_3)
    commit_to_flash(link)
    
def get_serial_number(link):
    ser_num_1 = setup_param.fetch_param(link, "GMB_SER_NUM_1")
    ser_num_2 = setup_param.fetch_param(link, "GMB_SER_NUM_2")
    ser_num_3 = setup_param.fetch_param(link, "GMB_SER_NUM_3")
    if ser_num_1 != None and ser_num_2 != None and ser_num_3 != None:
        serial_str = float3_to_string12(ser_num_1.param_value, ser_num_2.param_value, ser_num_3.param_value)
        if serial_str.startswith('GB'):
            return serial_str
    return None

def set_serial_number_3dr(link,month_serial_number):
    today = datetime.date.today() 
    
    # Build year identifier
    year = hex((today.year - 2010) % 16)[2]
    
    # Build month indentifier
    if today.month < 10:
        month = today.month
    elif today.month == 10:
        month = '0'
    elif today.month == 11:
        month = 'A'
    elif today.month == 12:
        month = 'B'        
    
    # Build per-mount serial number (5 digits)
    number = '%05d' % (month_serial_number % 100000)    
    
    # Build serial number string
    serial_str = 'GB11A' + str(year) + str(month) + str(number)
    
    # Save the serial number on the gimbal
    set_serial_number(link, serial_str)
    
    return serial_str