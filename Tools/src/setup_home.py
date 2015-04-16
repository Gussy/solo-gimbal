"""
"""
import setup_comutation


def home(link):
    link.file.mav.gimbal_set_home_offsets_send(link.target_sysid, link.target_compid);
    if link.file.recv_match(type="GIMBAL_HOME_OFFSET_CALIBRATION_RESULT", blocking=True, timeout=3):
        print 'home positions set'
        setup_comutation.printAxisCalibrationParam(link)
    else:
        print 'failed to set gome positions'
    