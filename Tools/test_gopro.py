#!/usr/bin/python

"""
Utility for reading the software version from a 3DR Gimbal.

@author: Angus Peart (angus@3dr.com)
"""

import sys, argparse, struct, os, serial, fnmatch
import serial.tools.list_ports
from pymavlink import mavutil
from pymavlink.mavutil import mavlink, mavserial, SerialPort
from pymavlink.dialects.v10 import ardupilotmega as mavlink

MAVLINK_SYSTEM_ID = 255
MAVLINK_COMPONENT_ID = mavlink.MAV_COMP_ID_GIMBAL
TARGET_SYSTEM_ID = 1
TARGET_COMPONENT_ID = mavlink.MAV_COMP_ID_GIMBAL

def getSerialPorts(preferred_list=[]):
    if os.name == 'nt':
        ports = list(serial.tools.list_ports.comports())
        ret = []
        for port, desc, hwid in ports:
            for preferred in preferred_list:
                if fnmatch.fnmatch(desc, preferred) or fnmatch.fnmatch(hwid, preferred):
                    ret.append(SerialPort(port, description=desc, hwid=hwid))
        return ret
    return mavutil.auto_detect_serial(preferred_list=preferred_list)

def open_comm(port=None, baudrate=230400):
    if not port:
        serial_list = getSerialPorts(preferred_list=['*USB Serial*','*FTDI*'])
        if len(serial_list) >= 1:
            port = serial_list[0].device
        else:
            port = '0.0.0.0:14550'
    mavserial = mavutil.mavlink_connection(device=port, baud=baudrate)
    link = mavlink.MAVLink(mavserial, MAVLINK_SYSTEM_ID, MAVLINK_COMPONENT_ID)
    link.target_sysid = TARGET_SYSTEM_ID
    link.target_compid = TARGET_COMPONENT_ID
    return (port, link)

def show_gopro(link):
    '''show incoming mavlink gopro messages'''
    while True:
        msg = link.file.recv_match(blocking=True)
        if not msg:
            return
        if msg.get_type() == "GOPRO_HEARTBEAT":
            print("%s: %s" % (msg.get_type(), mavlink.enums['GOPRO_HEARTBEAT_STATUS'][msg.status].name))
        elif msg.get_type() == "GOPRO_GET_RESPONSE":
            print("%s: '%s' = %u" % (msg.get_type(), mavlink.enums['GOPRO_COMMAND'][msg.cmd_id].name, msg.value))
        elif msg.get_type() == "GOPRO_SET_RESPONSE":
            print("%s: '%s' = %u" % (msg.get_type(), mavlink.enums['GOPRO_COMMAND'][msg.cmd_id].name, msg.result))
        elif msg.get_type().startswith("GOPRO"):
            print(msg)

_, link = open_comm()

show_gopro(link)
