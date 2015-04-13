#!/usr/bin/python

"""
Utility for reading the software version from a 3DR Gimbal.

@author: Angus Peart (angus@3dr.com)
"""

import sys, argparse, struct
from pymavlink import mavutil
from pymavlink.dialects.v10 import ardupilotmega as mavlink

MAVLINK_SYSTEM_ID = 50
MAVLINK_COMPONENT_ID = 230

default_baudrate = 230400

parser = argparse.ArgumentParser()
parser.add_argument("-p", "--port", help="Serial port or device used for MAVLink bootloading")
parser.add_argument("-b", "--baudrate", help="Serial port baudrate")
args = parser.parse_args()

# Accept a command line baudrate, fallback to the default
baudrate = default_baudrate
if args.baudrate:
    baudrate = args.baudrate

# Open the serial port
mavserial = mavutil.mavserial(
    device = args.port,
    baud = baudrate
)
link = mavlink.MAVLink(mavserial, MAVLINK_SYSTEM_ID, MAVLINK_COMPONENT_ID)

# Request the model
link.gopro_get_request_send(MAVLINK_SYSTEM_ID, MAVLINK_COMPONENT_ID, mavlink.GOPRO_COMMAND_MODEL)
