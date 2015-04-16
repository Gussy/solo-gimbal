#!/usr/bin/python

"""
Utility for loading firmware into the 3DR Gimbal.

"""
import sys

from parameters_helper import fetch_param, set_param, reset_gimbal

axis_enum = ['PITCH', 'ROLL', 'YAW']
status_enum = ['in progress', 'succeeded', 'failed']

def printAxisCalibrationParam(link):
    print getAxisCalibrationParam(link, axis_enum[0])
    print getAxisCalibrationParam(link, axis_enum[1])
    print getAxisCalibrationParam(link, axis_enum[2])
    

def getAxisCalibrationParam(link, axis_enum):
    home = fetch_param(link, "CC_" + axis_enum + "_HOME")
    icept = fetch_param(link, "CC_" + axis_enum + "_ICEPT")
    slope = fetch_param(link, "CC_" + axis_enum + "_SLOPE")
    return axis_enum, home.param_value, icept.param_value, slope.param_value

def startCalibration(link):
    # Set all commutation calibration parameters to 0
    set_param(link, "CC_YAW_SLOPE", 0.0);
    set_param(link, "CC_YAW_ICEPT", 0.0);
    set_param(link, "CC_ROLL_SLOPE", 0.0);
    set_param(link, "CC_ROLL_ICEPT", 0.0);
    set_param(link, "CC_PITCH_SLOPE", 0.0);
    set_param(link, "CC_PITCH_ICEPT", 0.0);

    # Commit the zeroed out calibration parameters to flash
    set_param(link, "COMMIT_FLASH", 69.0);

    # Reset the gimbal
    reset_gimbal(link);

def status(link):    
    startCalibration(link)
    
    status_per_axis = []
    while(len(status_per_axis) < 3):
        msg_progress = link.file.recv_match(type="GIMBAL_AXIS_CALIBRATION_PROGRESS", blocking=True, timeout=10)
        axis = axis_enum[msg_progress.calibration_axis - 1]
        progress = msg_progress.calibration_progress
        status = status_enum[msg_progress.calibration_status]
        
        text = "\rCalibrating %s - progress %d%% - %s            " % (axis, progress, status)
        sys.stdout.write(text)
        sys.stdout.flush()
        
        if status != 'in progress':
            status_per_axis.append((axis, status))
            if status == 'failed':
                break;
        
    print '\n'
    print status_per_axis
    print ''
    printAxisCalibrationParam(link)    
    
    
    
