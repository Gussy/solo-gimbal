"""
"""
import setup_comutation
from parameters_helper import receive_home_offset_result, start_home_calibration


def home(link):
    start_home_calibration(link)
    if receive_home_offset_result(link):
        print 'home positions set'
        setup_comutation.printAxisCalibrationParam(link)
    else:
        print 'failed to set gome positions'
    