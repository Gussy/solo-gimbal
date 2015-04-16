"""
"""
from setup_mavlink import receive_home_offset_result, start_home_calibration,\
    printAxisCalibrationParam


def home(link):
    start_home_calibration(link)
    if receive_home_offset_result(link):
        print 'home positions set'
        printAxisCalibrationParam(link)
    else:
        print 'failed to set gome positions'
    