#!/usr/bin/python

"""
Utility for loading firmware into the 3DR Gimbal.

"""
import sys

from parameters_helper import set_param, reset_gimbal, printAxisCalibrationParam,\
    getCalibrationProgress

axis_enum = ['PITCH', 'ROLL', 'YAW']
status_enum = ['in progress', 'succeeded', 'failed']

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
        axis, progress, status = getCalibrationProgress(link)
        
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
    
    
    
