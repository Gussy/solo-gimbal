#!/usr/bin/python

'''
     Command-line utility to handle comms to gimbal 
'''

from pymavlink import mavutil
import sys, argparse

from pymavlink.dialects.v10 import common as mavlink
from firmware_loader import update
from read_swver_param import readSWver
import setup_comutation

MAVLINK_SYSTEM_ID = 255
MAVLINK_COMPONENT_ID = mavlink.MAV_COMP_ID_GIMBAL
TARGET_SYSTEM_ID = 1
TARGET_COMPONENT_ID =  mavlink.MAV_COMP_ID_GIMBAL

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
    link.target_sysid = TARGET_SYSTEM_ID
    link.target_compid = TARGET_COMPONENT_ID
    
    if link.file.recv_match(type='HEARTBEAT', blocking=True, timeout=5) == None:
        print 'failed to comunicate to gimbal'
        return
    
    if args.binary:
        update(args.binary, link)
    else:
        readSWver(link)
        print ''
        setup_comutation.status(link)

if __name__ == '__main__':
    main()
    
    print '\n exit'
    
    sys.exit(0)
