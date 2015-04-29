#!/usr/bin/python

"""
Utility for loading firmware into the 3DR Gimbal.

"""
from pymavlink.mavparm import MAVParmDict

import setup_mavlink


def load_param_file(pid, link):
    parameters = MAVParmDict()
    
    parameters.load(pid,'GMB*',link.file, check=False)
    
    # Commit the zeroed out calibration parameters to flash
    parameters.mavset(link.file, "GMB_FLASH", 69.0, 3)

