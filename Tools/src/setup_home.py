"""
"""
import math

from setup_mavlink import get_current_joint_angles, get_current_delta_angles, get_current_delta_velocity
from setup_param import set_offsets, message_brodcasting
from pymavlink.rotmat import Vector3


NUMBER_OF_SAMPLES = 200
ENCODER_COUNTS_PER_RADIAN = 1000.0/(2*math.pi)

def getAverage(link,get_variable):
    sum_angles = Vector3()
    for i in range(NUMBER_OF_SAMPLES):
        angles = get_variable(link)
        sum_angles += angles    
    offset = sum_angles/NUMBER_OF_SAMPLES
    print [offset.x,offset.y,offset.z]
    return offset

def calibrate_joints(link):
    message_brodcasting(link, True)
    average = getAverage(link, get_current_joint_angles)
    set_offsets(link,'JNT',average)
    message_brodcasting(link, False)
    return

def calibrate_gyro(link):
    message_brodcasting(link, True)
    average = getAverage(link, get_current_delta_angles)
    set_offsets(link,'GYRO',average)
    message_brodcasting(link, False)
    return

def calibrate_accel(link):
    message_brodcasting(link, True)
    average = getAverage(link, get_current_delta_velocity)
    set_offsets(link, 'ACC', Vector3()) # TODO: implement accel calibration
    message_brodcasting(link, False)
    return


