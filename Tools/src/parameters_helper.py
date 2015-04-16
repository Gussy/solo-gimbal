'''

'''
from pymavlink.dialects.v10.ardupilotmega import MAV_PARAM_TYPE_REAL32
from time import sleep


def fetch_param(link,param,timeout = 10):
    # Get a parameter
    link.param_request_read_send(link.target_sysid, link.target_compid, param, -1)
    # Wait 10 seconds for a response
    msg = link.file.recv_match(type="PARAM_VALUE", blocking=True, timeout=timeout)
    return msg

def set_param(link,param_name,param_value):
    link.param_set_send(link.target_sysid, link.target_compid,param_name,param_value,MAV_PARAM_TYPE_REAL32)


def reset_gimbal(link):
    link.file.mav.gimbal_reset_send(link.target_sysid, link.target_compid)
    
    # clear the receive buffer
    for i in range(10): 
        link.file.recv_msg()
    

