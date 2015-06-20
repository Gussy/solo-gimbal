#!/usr/bin/python

'''
     Command-line utility to handle comms to gimbal 
'''
import sys, argparse

from firmware_loader import update
import setup_comutation, setup_home
from setup_mavlink import open_comm, wait_for_heartbeat
import setup_mavlink, setup_param
import setup_validate
import setup_run
import time
import numpy
from setup_param import set_offsets
import setup_factory

def handle_file(args, link):
    fileExtension = str(args.file).split(".")[-1].lower()
    if fileExtension == 'param':
        setup_param.load_param_file(args.file, link)
    elif fileExtension == 'ax':
        update(args.file, link)
    else:
        print 'file type not supported'
        sys.exit(1)
    return

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("file",  nargs='?', help="parameter or firmware file to be loaded into the gimbal", default=None)
    parser.add_argument("-p", "--port", help="Serial port or device used for MAVLink bootloading", default=None)
    parser.add_argument("-s", "--show", help="Show all useful gimbal parameters", action='store_true')
    parser.add_argument("-v", "--validate", help="Check gimbal parameters to see if they have valid values", action='store_true')
    parser.add_argument("-d", "--defaults", help="Reset gimbal parameters to default values", action='store_true')
    parser.add_argument("-r","--reboot", help="Reboot the gimbal", action='store_true')
    parser.add_argument("--run", help="run a quick test of the gimbal", action='store_true')
    parser.add_argument("--align", help="move the gimbal to the home position", action='store_true')
    parser.add_argument("--stop", help="Hold the gimbal at the current position", action='store_true')
    parser.add_argument("-c", "--calibrate", help="Run the comutation setup", action='store_true')
    parser.add_argument("-f", "--forcecal", help="Force the comutation setup", action='store_true')
    parser.add_argument("-j", "--jointcalibration", help="Calibrate joint angles", action='store_true')
    parser.add_argument("-g", "--gyrocalibration", help="Calibrate gyros", action='store_true')
    parser.add_argument("-a", "--accelcalibration", help="Calibrate accelerometers", action='store_true')
    parser.add_argument("-x", "--staticcal", help="Calibrate all static home values", action='store_true')
    parser.add_argument("-e", "--erase", help="Erase calibration values", action='store_true')
    parser.add_argument("--date", help="Setup assembly date", action='store_true')
    args = parser.parse_args()
 
    # Open the serial port
    link = open_comm(args.port, 230400)
    
    if wait_for_heartbeat(link) == None:
        print 'failed to comunicate to gimbal'
        sys.exit(1)

    if args.file:
        handle_file(args, link)
        return
    elif args.date:
        setup_factory.set_assembly_date(link)
    elif args.run:
        setup_run.run(link)
        return
    elif args.stop:
        setup_run.stop(link)
        return
    elif args.align:
        setup_run.align(link)
        return
    elif args.calibrate:
        setup_comutation.calibrate(link)
        return
    elif args.forcecal:
        setup_comutation.resetCalibration(link)
        wait_for_heartbeat(link)
        time.sleep(3)
        setup_comutation.calibrate(link)
        return
    elif args.show:
        setup_validate.show(link)
        return
    elif args.validate:
        setup_validate.validate(link)
        return
    elif args.defaults:
        setup_validate.restore_defaults(link)
        return
    elif args.staticcal:
        setup_home.calibrate_joints(link)
        setup_home.calibrate_gyro(link)
        set_offsets(link, 'ACC', numpy.zeros(3)) # Until we have accel cal zero the offsets on calibration
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
        ver = setup_factory.readSWver(link)
        if ver:
            asm_time = setup_factory.get_assembly_time(link)
            print("Software version: v%s" % ver)
            if(asm_time):
                print "Assembled on " +time.ctime(asm_time)
        else:
            print("Unable to read software version")
            sys.exit(1)

if __name__ == '__main__':
    main()    
    sys.exit(0)
