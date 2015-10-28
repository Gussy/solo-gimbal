#!/usr/bin/env python
import time, os, sys, argparse, time, json
import setup_mavlink, setup_factory_pub

import setup_validate, setup_param, setup_comutation
from setup_comutation import Results as calibration_results


def calibrationProgressCallback(axis, progress, status):
    text = "\rCalibrating %s - progress %d%% - %s            " % (axis, progress, status)
    sys.stdout.write(text)
    # The flush is required to refresh the screen on Ubuntu
    sys.stdout.flush()

def runCalibration(link):
    result = setup_comutation.calibrate(link, calibrationProgressCallback)
    sys.stdout.write("\n")
    sys.stdout.flush()

    return result

# Main method when called directly
def command_interface():
    parser = argparse.ArgumentParser()
    parser.add_argument("file",  nargs='?', help="parameter or firmware file to be loaded into the gimbal", default=None)
    parser.add_argument("-p", "--port", help="Serial port or device used for MAVLink bootloading", default=None)
    args = parser.parse_args()

    # Open the serial port
    port, link = setup_mavlink.open_comm(args.port)
    print("Connecting via port %s" % port)

    if link is None:
        print("Failed to open port %s" % port)
        sys.exit(1)

    # Send a heartbeat first to wake up the interface, because mavlink
    link.heartbeat_send(0, 0, 0, 0, 0)

    msg = setup_mavlink.get_any_gimbal_message(link)
    if msg:
        if setup_mavlink.is_bootloader_message(msg) and not args.file:
            print("Gimbal in bootloader, only firmware loading is available")
            sys.exit(0)
    else:
        # If the gimbal is using a different mavlink definitions version to
        #  what this script is using, try detecting the gimbal using a
        #  standard mavlink message that will not have changed.
        # Standard mavlink heartbeats could be used, but are usually not
        #  propogated through the pixhawk in Solo
        ver = setup_factory_pub.read_software_version(link, timeout=2)
        if ver:
            print("Possible mismatch in gimbal mavlink definitions, continuing anyway")
        else:
            print("No gimbal messages received, exiting.")
            sys.exit(1)

    for i in range(20):
        setup_comutation.resetCalibration(link)
        result = runCalibration(link)

        if result == calibration_results.ParamFetchFailed:
            print("Failed to get calibration parameters from the gimbal")
        elif result == calibration_results.CommsFailed:
            print("Gimbal failed to communicate calibration progress")

        # These results require a reset of the Gimbal
        if result == calibration_results.YawFailed:
            print("Yaw calibration failed")

        # Get the calibration data
        if result == calibration_results.Success:
            calibration = setup_comutation.getAxisCalibrationValues(link)
            if calibration:
                for axis in ['yaw']:#['pitch', 'roll', 'yaw']:
                    print("%s: slope=%f intercept=%f" % (axis, calibration[axis]['slope'], calibration[axis]['intercept']))
                print("")
            else:
                print("Error getting calibration values from the gimbal")

if __name__ == '__main__':
    command_interface()
    sys.exit(0)
