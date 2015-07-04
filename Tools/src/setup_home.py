"""
"""
import math
import numpy as np
import setup_accelcal

from setup_mavlink import get_current_joint_angles, get_current_delta_angles, get_current_delta_velocity
from setup_param import set_offsets, message_brodcasting, set_param,\
    commit_to_flash
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

def calibrate_joints(link):
    message_brodcasting(link, True)
    average = getAverage(link, get_current_joint_angles)
    if average:
        set_offsets(link, 'JNT', average)
    else:
        return None
    message_brodcasting(link, False)
    return average

def calibrate_gyro(link, progressCallback=None):
    message_brodcasting(link, True)
    average = getAverage(link, get_current_delta_angles, progressCallback=progressCallback)
    if average:
        set_offsets(link, 'GYRO', average)
    else:
        return None
    message_brodcasting(link, False)
    return average

def getAccelSample(link, ui_msg, AVG_COUNT=10):
    raw_input(ui_msg)
    v = getAverage(link, get_current_delta_velocity, sample_count=AVG_COUNT)
    v = v*100 # sample needs to be in units of m/s
    return np.asarray([v.x,v.y,v.z])

def calibrate_accel(link):
    message_brodcasting(link, True)
        
    # Get samples
    samples = [];
    samples.append(getAccelSample(link, "Place the gimbal leveled"))
    samples.append(getAccelSample(link, "Place the gimbal on it's side"))
    samples.append(getAccelSample(link, "Place the gimbal on it's back"))
    samples.append(getAccelSample(link, "Place the gimbal on it's other side"))
    samples.append(getAccelSample(link, "Place the gimbal on it's front side"))
    samples.append(getAccelSample(link, "Place the gimbal upside-down"))
    print(' ')
    
    p = setup_accelcal.calibrate_accel_6dof(samples)
    level = setup_accelcal.calc_level_euler_rpy(p, samples[0])
    
    print('calibration values are '+ str(p))
    print('offset values are '+ str(level.T*degrees(1)))
        
    
    set_param(link, "GMB_OFF_ACC_X", p[0])
    set_param(link, "GMB_OFF_ACC_Y", p[1])
    set_param(link, "GMB_OFF_ACC_Z", p[2])
    set_param(link, "GMB_GN_ACC_X",  p[3])
    set_param(link, "GMB_GN_ACC_Y",  p[4])
    set_param(link, "GMB_GN_ACC_Z",  p[5])
    set_param(link, "GMB_ALN_ACC_X", level[0])
    set_param(link, "GMB_ALN_ACC_Y", level[1])
    set_param(link, "GMB_ALN_ACC_Z", level[2])
    
    message_brodcasting(link, False)
    return


