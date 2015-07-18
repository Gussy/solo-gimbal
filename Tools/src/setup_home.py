"""
"""
import math
import numpy as np
import setup_accelcal
import os

from setup_mavlink import get_current_joint_angles, get_current_delta_angles, get_current_delta_velocity
from setup_param import set_offsets, message_brodcasting, set_param, commit_to_flash, set_accel_params
from setup_factory_pub import get_serial_number
from pymavlink.rotmat import Vector3
from math import degrees


NUMBER_OF_SAMPLES = 200
ENCODER_COUNTS_PER_RADIAN = 1000.0 / (2 * math.pi)

def getAverage(link, get_variable, sample_count=NUMBER_OF_SAMPLES, progressCallback=None):
    sum_angles = Vector3()
    for i in range(sample_count):
        if progressCallback:
            progress = float(i) / float(sample_count) * 100
            progressCallback(progress)
        angles = get_variable(link)
        if angles:
            sum_angles += angles
        else:
            return None
    offset = sum_angles / sample_count
    return offset

def calibrate_joints(link, progressCallback=None):
    message_brodcasting(link, True)
    average = getAverage(link, get_current_joint_angles, progressCallback=progressCallback)
    if average:
        set_offsets(link, 'JNT', average)
    else:
        return None
    message_brodcasting(link, False)
    return average

def calibrate_gyro(link, progressCallback=None):
    message_brodcasting(link, True)
    average = getAverage(link, get_current_delta_angles, progressCallback=progressCallback)
    average = average * 100 # sample needs to be in units of rad/s
    if average:
        set_offsets(link, 'GYRO', average)
    else:
        return None
    message_brodcasting(link, False)
    return average

def getAccelSample(link, AVG_COUNT=50, progressCallback=None):
    if progressCallback:
        progressCallback(0)
    # discard first sample
    get_current_delta_velocity(link)
    v = getAverage(link, get_current_delta_velocity, sample_count=AVG_COUNT, progressCallback=progressCallback)
    v = v * 100 # sample needs to be in units of m/s
    return np.asarray([v.x, v.y, v.z])

def calibrate_accel(link, progressCallback=None):
    message_brodcasting(link, True)

    # Orientations
    orientations = [
        'level',
        'on it\'s left side',
        'on it\'s right side',
        'on it\'s back',
        'on it\'s front',
        'upside down',
    ]

    currentMessage = ''
    currentProgress = 0
    progressStep = (1.0 / float(len(orientations))) * 100.0

    def averageProgressCallback(progress):
        averageProgress = currentProgress + (progressStep * float(progress / 100.0))
        if progressCallback:
            progressCallback(averageProgress, currentMessage, False)

    # Get samples
    samples = list()
    for orientation in orientations:
        currentProgress = float(orientations.index(orientation) / float(len(orientations))) * 100.0
        currentMessage = "Place the gimbal %s" % orientation
        if progressCallback is None:
            raw_input(currentMessage)
        else:
            # Spin while waiting for a continue signal
            while not progressCallback(currentProgress, currentMessage, True):
                pass
        sample = getAccelSample(link, progressCallback=averageProgressCallback)
        samples.append(sample)
    
    p = setup_accelcal.calibrate_accel_6dof(samples)
    level = setup_accelcal.calc_level_euler_rpy(p, samples[0])

    if progressCallback is None:
        print('\nCalibration values are ' + str(p))
        print('Offset values are ' + str(level.T * degrees(1)))
    
    # Save the parameters to flash
    set_accel_params(link, p, level)
    
    message_brodcasting(link, False)

    if os.environ.get('GMB_ACC_FILE') is not None:
        f = open(os.environ['GMB_ACC_FILE'], "a")
        f.write("%s,% .4f,% .4f,% .4f,% .4f,% .4f,% .4f\n" % (get_serial_number(link), p[0], p[1], p[2], p[3], p[4], p[5]))
        f.close()

    offsets = Vector3(p[0], p[1], p[2])
    gains = Vector3(p[3], p[4], p[5])
    alignment = Vector3(float(level[0]), float(level[1]), float(level[2]))
    return offsets, gains, alignment
