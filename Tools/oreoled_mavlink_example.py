#!/usr/bin/env python

# import dronekit
# from pymavlink.dialects.v10 import ardupilotmega as mavlink
# 
# solo = dronekit.connect('udpin:0.0.0.0:14550', wait_ready=True)
# 
# msg = solo.message_factory.led_control_pattern_encode(0, 0,
#                                                         255,
#                                                         mavlink.LED_CONTROL_PATTERN_AVIATION_STROBE,  # pattern
#                                                         255,                                # bias_red
#                                                         255,                              # bias_green
#                                                         255,                               # bias_blue
#                                                         0,                                  # amplitude_red
#                                                         0,                                  # amplitude_green
#                                                         0,                                  # amplitude_blue
#                                                         1000,                               # period (milliseconds)
#                                                         -2,                                 # repeat (-2 is infinite cycles )
#                                                         0)                                  # phase_offset
# solo.send_mavlink(msg)

"""
Example of setting LED patterns on a Solo using Mavlink
"""

import sys, argparse, time

from pymavlink import mavutil
from pymavlink.dialects.v10 import ardupilotmega as mavlink

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("--device", required=True, help="Serial port or device used for MAVLink bootloading")
parser.add_argument("--baudrate", type=int, help="master port baud rate", default=115200)
args = parser.parse_args()

# Open the mavlink copnnection
master = mavutil.mavlink_connection(args.device, baud=args.baudrate)

master.target_system = 1
master.target_component = 0

#define OREOLED_BACKLEFT                0       // back left led instance number
#define OREOLED_BACKRIGHT               1       // back right led instance number
#define OREOLED_FRONTRIGHT              2       // front right led instance number
#define OREOLED_FRONTLEFT               3       // front left led instance number

# Make all LEDs party
master.mav.led_control_macro_send(master.target_system, master.target_component, 255, mavlink.LED_CONTROL_MACRO_FWUPDATE)

# Make all LEDs amber
#master.mav.led_control_macro_send(master.target_system, master.target_component, 255, mavlink.LED_CONTROL_MACRO_AMBER)

# Strobe LEDs like on a real aircraft
def aviation_strobe(instance, red, green, blue):
    master.mav.led_control_pattern_send(master.target_system, master.target_component,
                                        instance,
                                        mavlink.LED_CONTROL_PATTERN_AVIATION_STROBE,  # pattern
                                        red,                                # bias_red
                                        green,                              # bias_green
                                        blue,                               # bias_blue
                                        0,                                  # amplitude_red
                                        0,                                  # amplitude_green
                                        0,                                  # amplitude_blue
                                        1000,                               # period (milliseconds)
                                        -2,                                 # repeat (-2 is infinite cycles )
                                        0)                                  # phase_offset
#aviation_strobe(255, 255, 255, 255)

#master.mav.led_control_pattern_param_send(master.target_system, master.target_component, 255, mavlink.LED_CONTROL_PARAM_PERIOD, 1000)

def set_rgb(instance, red, green, blue):
    if instance > 255 or red > 255 or green > 255 or blue > 255:
        return

    master.mav.led_control_pattern_send(master.target_system, master.target_component,
                                        instance,                           # instance
                                        mavlink.LED_CONTROL_PATTERN_SOLID,  # pattern
                                        red,                                # bias_red
                                        green,                              # bias_green
                                        blue,                               # bias_blue
                                        0,                                  # amplitude_red
                                        0,                                  # amplitude_green
                                        0,                                  # amplitude_blue
                                        0,                                  # period
                                        -2, # -2 is infinite cycles         # repeat
                                        0)                                  # phase_offset

# Set LEDs to solid green
set_rgb(255, 0, 0, 255)


# Stop the mavlink control
#master.mav.led_control_pattern_send(master.target_system, master.target_component, 255, mavlink.LED_CONTROL_PATTERN_STOP, 0, 0, 0, 0, 0, 0, 0, 0, 0)
