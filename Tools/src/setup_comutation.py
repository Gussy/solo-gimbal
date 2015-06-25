#!/usr/bin/python

"""
Utility for loading firmware into the 3DR Gimbal.

"""
import sys
import setup_mavlink, setup_param
from setup_param import getAxisCalibrationParam

axis_enum = ['PITCH', 'ROLL', 'YAW']
status_enum = ['in progress', 'succeeded', 'failed']

outputMethod = sys.stdout.write

def print_and_flush(obj):
    global outputMethod
    
    if outputMethod is sys.stdout.write and 'message' in obj:
        outputMethod(obj['message'])
    else:
        outputMethod(obj)

    # The flush is required to refresh the screen on Ubuntu
    sys.stdout.flush()

def printMessage(msg, level='info'):
    print_and_flush({
        'type': 'status',
        'level': level,
        'message': msg
    })

def printProgressMessage(axis, percentage, status, msg):
    print_and_flush({
        'type': 'calibration_progress',
        'axis': axis,
        'percentage': percentage,
        'status': status,
        'message': msg
    })

def printAxisCalibrationValues(link):
    for i in range(len(axis_enum)):
        result = getAxisCalibrationParam(link, axis_enum[i])
        if result == None:
            printMessage('Failed to get calibration parameters from the gimbal', level='error')
            return
        axis, slope, intercept = result[0].lower(), result[1], result[2]
        msg = ""
        print_and_flush({
            'type': 'calibration_values',
            'axis': axis,
            'slope': slope,
            'intercept': intercept,
            'message': '%s: slope=%f intercept=%f\n' % (axis, slope, intercept)
        })

def getAxisCalibrationParams(link):
    pitch = getAxisCalibrationParam(link, axis_enum[0])
    roll = getAxisCalibrationParam(link, axis_enum[1])
    yaw = getAxisCalibrationParam(link, axis_enum[2])
    return (pitch, roll, yaw)

def resetCalibration(link):
    setup_param.clear_comutation_params(link)    
    setup_mavlink.reset_gimbal(link)

def calibrate(link, outputHandler=None):
    global outputMethod

    # Use a callback for all output if required
    if outputHandler:
        outputMethod = outputHandler

    # Check if a calibration exists, exit if it does
    a, b, c = getAxisCalibrationParams(link)
    existing_error = 'A %s calibration already exists, erase current calibration first'
    if a[1] != 0 and a[2] != 0:
        printMessage(existing_error % a[0].lower(), level='error')
        return
    elif b[1] != 0 and b[2] != 0:
        printMessage(existing_error % b[0].lower(), level='error')
        return
    elif c[1] != 0 and c[2] != 0:
        printMessage(existing_error % c[0].lower(), level='error')
        return
    
    setup_mavlink.requestCalibration(link)

    calibratied_axes = 0
    axis_statuses = {'pitch': 'not started', 'roll': 'not started', 'yaw': 'not started'}
    while(calibratied_axes < 3):

        result = setup_mavlink.getCalibrationProgress(link)
        if result:
            axis, progress, status = result[0], result[1], result[2]
        else:
            printMessage('Gimbal failed to communicate calibration progress', level='error')
            return

        # Adjust number of calibrated axes if a previous calibration is already in progress
        if calibratied_axes == 0 and axis == axis_enum[1]:
            calibratied_axes == 1
        elif calibratied_axes == 0 and axis == axis_enum[2]:
            calibratied_axes == 2
        
        text = "\rCalibrating %s - progress %d%% - %s            " % (axis, progress, status)
        printProgressMessage(axis, progress, status, text)
        sys.stdout.flush()

        if status != 'in progress':
            axis_statuses[axis.lower()] = status

            # Don't continue calibration if an axis fails
            if status == 'failed':
                break

            calibratied_axes += 1

    print_and_flush({
        'type': 'calibration_status',
        'pitch': axis_statuses['pitch'],
        'roll': axis_statuses['roll'],
        'yaw': axis_statuses['yaw'],
        'message': '\n%s: %s, %s: %s, %s: %s\n\n' % ('pitch', axis_statuses['pitch'], 'roll', axis_statuses['roll'], 'yaw', axis_statuses['yaw'])
    })
    printAxisCalibrationValues(link)
