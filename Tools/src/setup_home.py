"""
"""
import math
import numpy

from setup_mavlink import receive_home_offset_result, start_home_calibration, \
    get_current_joint_angles,get_current_delta_angles, get_current_delta_velocity
from setup_comutation import printAxisCalibrationParam
from setup_param import set_offsets


NUMBER_OF_SAMPLES = 200
ENCODER_COUNTS_PER_RADIAN = 1000.0/(2*math.pi)

def home(link):
    start_home_calibration(link)
    if receive_home_offset_result(link):
        print 'home positions set'
        printAxisCalibrationParam(link)
    else:
        print 'failed to set home positions'


def getAverage(link,get_variable):
    average = numpy.zeros(3)
    for i in range(NUMBER_OF_SAMPLES):
        angles = numpy.array(get_variable(link), dtype=numpy.float)
        average += angles    
    average /= NUMBER_OF_SAMPLES
    return average

def calibrate_joints(link):
    average = getAverage(link, get_current_joint_angles)
    print average
    set_offsets(link,'JNT',average)
    return

def calibrate_gyro(link):
    average = getAverage(link, get_current_delta_angles)
    print average
    set_offsets(link,'GYRO',average)
    return

def calibrate_accel(link):
    average = getAverage(link, get_current_delta_velocity)
    print average
    set_offsets(link, 'ACC', numpy.zeros(3)) # TODO: implement accel calibration
    return


