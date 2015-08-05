import math
import numpy as np
import setup_accelcal
import os

from setup_mavlink import get_current_joint_angles, get_current_delta_angles, get_current_delta_velocity
from setup_param import set_offsets, set_param, commit_to_flash, set_accel_params, enable_torques_message
from setup_factory_pub import get_serial_number
from pymavlink.rotmat import Vector3
from math import degrees

def getAverage(link, get_variable, sample_count=200, progressCallback=None):
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
    average = getAverage(link, get_current_joint_angles, progressCallback=progressCallback)
    if average:
        set_offsets(link, 'JNT', average)
    else:
        return None
    return average

def calibrate_gyro(link, progressCallback=None):
    average = getAverage(link, get_current_delta_angles, progressCallback=progressCallback)
    average = average * 100 # sample needs to be in units of rad/s
    if average:
        set_offsets(link, 'GYRO', average)
    else:
        return None
    return average

def getAccelSample(link, AVG_COUNT=50, progressCallback=None):
    if progressCallback:
        progressCallback(0)
    # discard first sample
    get_current_delta_velocity(link)
    v = getAverage(link, get_current_delta_velocity, sample_count=AVG_COUNT, progressCallback=progressCallback)
    v = v * 100 # sample needs to be in units of m/s
    return np.asarray([v.x, v.y, v.z])

def calibrate_accel(link, progressCallback=None, errorCallback=None):

    enable_torques_message(link, False)

    # Expected values +/- 2.0 [x, y, z]
    # level [ 0    0   -9.8]
    # left  [ 0    9.8  0  ]
    # right [ 0   -9.8  0  ]
    # back  [ 9.8  0    0  ]
    # front [-9.8  0    0  ]
    # down  [ 0    0    9.8]

    GRAVITY_M_S = 9.8
    ORIENTATION_TOL = 2.0

    # Orientations
    orientations = [
        {
            'name': 'level',
            'values': [0, 0, -GRAVITY_M_S]
        },
        {
            'name': 'on it\'s left side',
            'values': [0, GRAVITY_M_S, 0]
        },
        {
            'name': 'on it\'s right side',
            'values': [0, -GRAVITY_M_S, 0]
        },
        {
            'name': 'on it\'s back',
            'values': [GRAVITY_M_S, 0, 0]
        },
        {
            'name': 'on it\'s front',
            'values': [-GRAVITY_M_S, 0, 0]
        },
        {
            'name': 'upside down',
            'values': [0, 0, GRAVITY_M_S]
        }
    ]

    currentMessage = ''
    currentProgress = 0
    progressStep = (1.0 / float(len(orientations))) * 100.0

    def averageProgressCallback(progress):
        averageProgress = currentProgress + (progressStep * float(progress / 100.0))
        if progressCallback:
            progressCallback(averageProgress, currentMessage, False)

    def withinTolerance(number, center, tolerance=ORIENTATION_TOL):
        return (center - tolerance) <= number <= (center + tolerance)

    # Get samples
    samples = list()
    MAX_RETRIES = 5
    failed = False
    for orientation in orientations:
        failCounter = 0
        while True:
            currentProgress = float(orientations.index(orientation) / float(len(orientations))) * 100.0
            currentMessage = "Place the gimbal %s" % orientation['name']
            if progressCallback is None:
                raw_input(currentMessage)
            else:
                # Spin while waiting for a continue signal
                while not progressCallback(currentProgress, currentMessage, True):
                    pass
            sample = getAccelSample(link, progressCallback=averageProgressCallback)

            # Check that the sampled value is close to the values it should be
            if (withinTolerance(sample[0], orientation['values'][0])
                and withinTolerance(sample[1], orientation['values'][1])
                and withinTolerance(sample[2], orientation['values'][2])):
                samples.append(sample)
                break
            elif failCounter >= (MAX_RETRIES - 1):
                failed = True
                break
            else:
                failCounter += 1
                if errorCallback is None:
                    print("Error: Gimbal not %s" % orientation['name'])
        if failed:
            msg = "Stopping calibration, maximum retries reached."
            if errorCallback:
                errorCallback(msg)
            else:
                print(msg)
            break
    if failed:
        return None, None, None

    p = setup_accelcal.calibrate_accel_6dof(samples)
    level = setup_accelcal.calc_level_euler_rpy(p, samples[0])

    if progressCallback is None:
        print('\nCalibration values are ' + str(p))
        print('Offset values are ' + str(level.T * degrees(1)))

    # Save the parameters to flash
    set_accel_params(link, p, level)

    if os.environ.get('GMB_ACC_FILE') is not None:
        f = open(os.environ['GMB_ACC_FILE'], "a")
        f.write("%s,% .4f,% .4f,% .4f,% .4f,% .4f,% .4f\n" % (get_serial_number(link), p[0], p[1], p[2], p[3], p[4], p[5]))
        f.close()

    offsets = Vector3(p[0], p[1], p[2])
    gains = Vector3(p[3], p[4], p[5])
    alignment = Vector3(float(level[0]), float(level[1]), float(level[2]))
    return offsets, gains, alignment
