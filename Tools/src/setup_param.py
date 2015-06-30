#!/usr/bin/python

"""
Utility for loading firmware into the 3DR Gimbal.

"""
import time
from pymavlink.mavparm import MAVParmDict
from pymavlink.dialects.v10.ardupilotmega import MAV_PARAM_TYPE_REAL32
from pymavlink.rotmat import Vector3

def fetch_param(link, param, timeout=2):
    for i in range(timeout):
        link.param_request_read_send(link.target_sysid, link.target_compid, param, -1)
        msg = link.file.recv_match(type="PARAM_VALUE", blocking=True, timeout=timeout)
        if msg:
            return msg
    return None

def set_param(link, param_name, param_value):
    parameters = MAVParmDict()
    parameters.mavset(link.file, param_name, param_value, 3);

def commit_to_flash(link):    
    # Commit the zeroed out calibration parameters to flash
    link.param_set_send(link.target_sysid, 0, "GMB_FLASH", 69.0, MAV_PARAM_TYPE_REAL32)
    # Sleep for two seconds as immedietly following commands will fail (while the C2000 is writing flash)
    time.sleep(2)

def load_param_file(pid, link):
    parameters = MAVParmDict()
    parameters.load(pid, 'GMB*', link.file, check=False)
    commit_to_flash(link)

def clear_comutation_params(link):
    parameters = MAVParmDict()

    # Set all commutation calibration parameters to 0
    parameters.mavset(link.file, "GMB_YAW_SLOPE", 0.0, 3)
    parameters.mavset(link.file, "GMB_YAW_ICEPT", 0.0, 3)
    parameters.mavset(link.file, "GMB_ROLL_SLOPE", 0.0, 3)
    parameters.mavset(link.file, "GMB_ROLL_ICEPT", 0.0, 3)
    parameters.mavset(link.file, "GMB_PITCH_SLOPE", 0.0, 3)
    parameters.mavset(link.file, "GMB_PITCH_ICEPT", 0.0, 3)
    commit_to_flash(link)

def message_brodcasting(link, broadcast=True):
    if broadcast:
        set_param(link, "GMB_BROADCAST", 1)
    else:
        set_param(link, "GMB_BROADCAST", 0)

def set_offsets(link, kind, offsets):
    if offsets and kind:
        set_param(link, "GMB_OFF_" + kind + "_X", offsets.x)
        set_param(link, "GMB_OFF_" + kind + "_Y", offsets.y)
        set_param(link, "GMB_OFF_" + kind + "_Z", offsets.z)
        commit_to_flash(link)

def get_offsets(link, kind, timeout=2):
    x = fetch_param(link, "GMB_OFF_" + kind + "_X", timeout=timeout)
    y = fetch_param(link, "GMB_OFF_" + kind + "_Y", timeout=timeout)
    z = fetch_param(link, "GMB_OFF_" + kind + "_Z", timeout=timeout)
    if x == None or y == None or z == None:
        return None
    else:
        return Vector3(x=x.param_value, y=y.param_value, z=z.param_value)

def get_gains(link, kind):
    P = fetch_param(link, "GMB_" + kind + "_P")
    I = fetch_param(link, "GMB_" + kind + "_I")
    D = fetch_param(link, "GMB_" + kind + "_D")
    if P == None or I == None or D == None:
        return None
    else:
        return P.param_value, I.param_value, D.param_value

def getAxisCalibrationParam(link, axis_enum):
    icept = fetch_param(link, "GMB_" + axis_enum + "_ICEPT")
    slope = fetch_param(link, "GMB_" + axis_enum + "_SLOPE")
    if icept == None or slope == None:
        return None
    else:
        return [icept.param_value, slope.param_value]
