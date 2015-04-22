"""
"""
import math
import numpy
from setup_mavlink import receive_home_offset_result, start_home_calibration,\
    printAxisCalibrationParam, get_current_joint_angles, set_joint_offsets

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
    return;
    return;