#!/usr/bin/python

"""
Utility for reading the software version from a 3DR Gimbal.

@author: Angus Peart (angus@3dr.com)
"""

import sys, argparse, struct
from pymavlink import mavutil
from pymavlink.dialects.v10 import common as mavlink

MAVLINK_SYSTEM_ID = 50
MAVLINK_COMPONENT_ID = 230

default_baudrate = 230400

def float_to_bytes(f):
    return struct.unpack('4b', struct.pack('<f', f))

parser = argparse.ArgumentParser()
parser.add_argument("-p", "--port", help="Serial port or device used for MAVLink bootloading")
parser.add_argument("-b", "--baudrate", help="Serial port baudrate")
args = parser.parse_args()

# Accept a command line baudrate, fallback to the default
baudrate = default_baudrate
if args.baudrate:
    baudrate = args.baudrate

# Open the serial port
mavserial = mavutil.mavlink_connection(
    device = args.port,
    baud = baudrate
)
link = mavlink.MAVLink(mavserial, MAVLINK_SYSTEM_ID, MAVLINK_COMPONENT_ID)

# Get a parameter
link.param_request_read_send(MAVLINK_SYSTEM_ID, MAVLINK_COMPONENT_ID, "SYSID_SWVER", -1)

# Wait 10 seconds for a response
msg = mavserial.recv_match(type="PARAM_VALUE", blocking=True, timeout=10)
if not msg:
    print("Requested param not received.")
    sys.exit(1)
else:
    swver_raw = float_to_bytes(msg.param_value)
    print("SW Version %i.%i.%i" % (swver_raw[3], swver_raw[2], swver_raw[1]))
