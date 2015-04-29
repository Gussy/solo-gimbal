#!/usr/bin/python

"""
Utility for loading firmware into the 3DR Gimbal.

"""
from pymavlink.mavparm import MAVParmDict

def fetch_param(link, param, timeout=10):
    # Get a parameter
    link.param_request_read_send(link.target_sysid, link.target_compid, param, -1)
    # Wait 10 seconds for a response
    msg = link.file.recv_match(type="PARAM_VALUE", blocking=True, timeout=timeout)
    return msg

def set_param(link, param_name, param_value):
    parameters = MAVParmDict()
    parameters.mavset(link.file, param_name, param_value,3);

def commit_to_flash(link):
    parameters = MAVParmDict()
    
    # Commit the zeroed out calibration parameters to flash
    parameters.mavset(link.file, "GMB_FLASH", 69.0, 3)

def load_param_file(pid, link):
    parameters = MAVParmDict()
    parameters.load(pid,'GMB*',link.file, check=False)
    commit_to_flash(link)

def clear_comutation_params(link):
    parameters = MAVParmDict()

    # Set all commutation calibration parameters to 0
    parameters.mavset(link.file, "GMB_YAW_SLOPE", 0.0,3);
    parameters.mavset(link.file, "GMB_YAW_ICEPT", 0.0,3);
    parameters.mavset(link.file, "GMB_ROLL_SLOPE", 0.0,3);
    parameters.mavset(link.file, "GMB_ROLL_ICEPT", 0.0,3);
    parameters.mavset(link.file, "GMB_PITCH_SLOPE", 0.0,3);
    parameters.mavset(link.file, "GMB_PITCH_ICEPT", 0.0,3);
    commit_to_flash(link)

def get_SWVER_param(link):
    return fetch_param(link, "GMB_SWVER")


def set_offsets(link, kind, offsets):    
    set_param(link, "GMB_OFF_"+kind+"_Y", offsets[0]);
    set_param(link, "GMB_OFF_"+kind+"_X", offsets[1]);
    set_param(link, "GMB_OFF_"+kind+"_Z", offsets[2]);
    
    
def getAxisCalibrationParam(link, axis_enum):
    home = fetch_param(link, "GMB_" + axis_enum + "_HOME")
    icept = fetch_param(link, "GMB_" + axis_enum + "_ICEPT")
    slope = fetch_param(link, "GMB_" + axis_enum + "_SLOPE")
    return axis_enum, home.param_value, icept.param_value, slope.param_value



