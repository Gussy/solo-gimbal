#!/usr/bin/python

'''
     Command-line utility to handle comms to gimbal 
'''

import sys, argparse

from firmware_loader import update
from setup_mavlink import open_comm, wait_for_hearbeat
import setup_comutation, setup_home
import setup_mavlink
from setup_read_sw_version import readSWver

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--binary", help="Application binary file load", default=None)
    parser.add_argument("--show", help="Show the comutation parameters", action='store_true')
    parser.add_argument("--comutation", help="Run the comutation setup", action='store_true')
    parser.add_argument("--home", help="Home alignment", action='store_true')
    parser.add_argument("-p", "--port", help="Serial port or device used for MAVLink bootloading")
    parser.add_argument("-b", "--baudrate", help="Serial port baudrate", default=230400)
    args = parser.parse_args()
 
    # Open the serial port
    link = open_comm(args.port, args.baudrate)
    
    if wait_for_hearbeat(link) == None:
        print 'failed to comunicate to gimbal'
        return
    
    if args.binary:
        update(args.binary, link)
        return
    elif args.comutation:
        setup_comutation.status(link)
        return
    elif args.show:
        setup_mavlink.printAxisCalibrationParam(link)
        return
    elif args.home:
        setup_home.home(link)
        return
    else:
        readSWver(link)
        return

if __name__ == '__main__':
    main()    
    sys.exit(0)
