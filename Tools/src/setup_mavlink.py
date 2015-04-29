'''

'''
from pymavlink.dialects.v10.ardupilotmega import MAV_PARAM_TYPE_REAL32
from setup_comutation import axis_enum, status_enum
from pymavlink.mavutil import mavlink
from pymavlink import mavutil
from setuptools.command.build_ext import if_dl

MAVLINK_SYSTEM_ID = 255
MAVLINK_COMPONENT_ID = mavlink.MAV_COMP_ID_GIMBAL
TARGET_SYSTEM_ID = 1
TARGET_COMPONENT_ID = mavlink.MAV_COMP_ID_GIMBAL


def open_comm(port, baudrate):
    mavserial = mavutil.mavlink_connection(device=port,
        baud=baudrate)
    link = mavlink.MAVLink(mavserial, MAVLINK_SYSTEM_ID, MAVLINK_COMPONENT_ID)
    link.target_sysid = TARGET_SYSTEM_ID
    link.target_compid = TARGET_COMPONENT_ID
    return link


def wait_for_hearbeat(link):
    return link.file.recv_match(type='HEARTBEAT', blocking=True, timeout=5)

def fetch_param(link, param, timeout=10):
    # Get a parameter
    link.param_request_read_send(link.target_sysid, link.target_compid, param, -1)
    # Wait 10 seconds for a response
    msg = link.file.recv_match(type="PARAM_VALUE", blocking=True, timeout=timeout)
    return msg

def set_param(link, param_name, param_value):
    link.param_set_send(link.target_sysid, 0, param_name, param_value, MAV_PARAM_TYPE_REAL32)

def getAxisCalibrationParam(link, axis_enum):
    home = fetch_param(link, "GMB_" + axis_enum + "_HOME")
    icept = fetch_param(link, "GMB_" + axis_enum + "_ICEPT")
    slope = fetch_param(link, "GMB_" + axis_enum + "_SLOPE")
    return axis_enum, home.param_value, icept.param_value, slope.param_value

def get_current_joint_angles(link):
    while(True):
        msg_gimbal = link.file.recv_match(type="GIMBAL_REPORT", blocking=True, timeout=2)
        if msg_gimbal is None:
            return None
        else:
            return [msg_gimbal.joint_el, msg_gimbal.joint_roll, msg_gimbal.joint_az]
        
def get_current_delta_angles(link):
    while(True):
        msg_gimbal = link.file.recv_match(type="GIMBAL_REPORT", blocking=True, timeout=2)
        if msg_gimbal is None:
            return None
        else:
            return [msg_gimbal.delta_angle_y, msg_gimbal.delta_angle_x, msg_gimbal.delta_angle_z]
        
def set_offsets(link, kind, offsets):    
    set_param(link, "GMB_OFF_"+kind+"_Y", offsets[0]);
    set_param(link, "GMB_OFF_"+kind+"_X", offsets[1]);
    set_param(link, "GMB_OFF_"+kind+"_Z", offsets[2]);
    
def printAxisCalibrationParam(link):
    print getAxisCalibrationParam(link, axis_enum[0])
    print getAxisCalibrationParam(link, axis_enum[1])
    print getAxisCalibrationParam(link, axis_enum[2])
    
def reset_gimbal(link):
    link.file.mav.command_long_send(link.target_sysid, link.target_compid,42501,0,0,0,0,0,0,0,0)
    result = link.file.recv_match(type="COMMAND_ACK", blocking=True, timeout=3)
    if result:
        return True
    else:
        print 'failed to reboot'
        return False 

def getCalibrationState(link):
    while(True):
        msg_status = link.file.recv_match(type="COMMAND_LONG", blocking=True, timeout=10)
        if msg_status is None:
            return None
        if msg_status.command == 42504:
            return [msg_status.param2, msg_status.param3, msg_status.param1]

def getCalibrationProgress(link):
    while(True):
        msg_progress = link.file.recv_match(type="COMMAND_LONG", blocking=True, timeout=10)
        if msg_progress is None:
            return None
        if msg_progress.command == 42502:
            break

    axis = axis_enum[int(msg_progress.param1) - 1]
    progress = int(msg_progress.param2)
    status = status_enum[int(msg_progress.param3)]
    
    return axis, progress, status

def receive_home_offset_result(link):
    return link.file.recv_match(type="COMMAND_ACK", blocking=True, timeout=3)

def start_home_calibration(link):    
    return link.file.mav.command_long_send(link.target_sysid, link.target_compid,42500,0,0,0,0,0,0,0,0)

def requestCalibration(link):
    return link.file.mav.command_long_send(link.target_sysid, link.target_compid,42503,0,0,0,0,0,0,0,0)
