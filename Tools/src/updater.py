#!/usr/bin/python

'''
     Command-line utility to handle comms to gimbal 
'''

from pymavlink import mavutil
import sys, argparse

from pymavlink.dialects.v10 import common as mavlink
from firmware_loader import update
from read_swver_param import readSWver

MAVLINK_SYSTEM_ID = 255
MAVLINK_COMPONENT_ID = mavlink.MAV_COMP_ID_GIMBAL

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--binary", help="Application binary file load", default=None)
    parser.add_argument("-p", "--port", help="Serial port or device used for MAVLink bootloading")
    parser.add_argument("-b", "--baudrate", help="Serial port baudrate", default=230400)
    args = parser.parse_args()
 
    # Open the serial port
    mavserial = mavutil.mavlink_connection(
        device=args.port,
        baud=args.baudrate
    )
    link = mavlink.MAVLink(mavserial, MAVLINK_SYSTEM_ID, MAVLINK_COMPONENT_ID)
    
    if args.binary:
        update(args.binary, link)
    else:
        readSWver(link)
    

if __name__ == '__main__':
    main()
    sys.exit(0)
