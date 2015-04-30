#!/usr/bin/python

'''
     Command-line utility to handle comms to gimbal 
'''

import sys, argparse

from firmware_loader import update
import setup_comutation, setup_home
from setup_mavlink import open_comm, wait_for_hearbeat
import setup_mavlink,setup_param
from setup_read_sw_version import readSWver
import setup_run


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("-p", "--port", help="Serial port or device used for MAVLink bootloading", default='0.0.0.0:14550')
    parser.add_argument("-s","--show", help="Show the comutation parameters", action='store_true')
    parser.add_argument("-r","--reboot", help="Reboot the gimbal", action='store_true')
    parser.add_argument("--load", help="Application binary file load", default=None)
    parser.add_argument("--pid", help="Load PID setting from param file", default=None)
    parser.add_argument("--run", help="run a quick test of the gimbal", action='store_true')
    parser.add_argument("--calibrate", help="Run the comutation setup", action='store_true')
    parser.add_argument("--jointcalibration", help="Calibrate joint angles", action='store_true')
    parser.add_argument("--gyrocalibration", help="Calibrate gyros", action='store_true')
    parser.add_argument("--accelcalibration", help="Calibrate accelerometers", action='store_true')
    parser.add_argument("--erase", help="Erase calibration values", action='store_true')
    args = parser.parse_args()
 
    # Open the serial port
    link = open_comm(args.port, 230400)
    
        
    if wait_for_hearbeat(link) == None:
        print 'failed to comunicate to gimbal'
        return
    
    if args.load:
        update(args.load, link)
        return
    elif args.pid:
        setup_param.load_param_file(args.pid, link)
        return
    elif args.run:
        setup_run.run(link);
        return
    elif args.calibrate:
        setup_comutation.calibrate(link)
        return
    elif args.show:
        setup_comutation.printAxisCalibrationParam(link)
        return
    elif args.jointcalibration:
        setup_home.calibrate_joints(link)
        return
    elif args.gyrocalibration:
        setup_home.calibrate_gyro(link)
        return
    elif args.accelcalibration:
        setup_home.calibrate_accel(link)
        return
    elif args.reboot:
        setup_mavlink.reset_gimbal(link)
        return
    elif args.erase:
        setup_comutation.resetCalibration(link)
        return
    else:
        readSWver(link)
        return

if __name__ == '__main__':
    main()    
    sys.exit(0)
