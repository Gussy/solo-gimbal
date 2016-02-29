#!/usr/bin/env python

"""
Utility for reading the software version from a 3DR Gimbal.

@author: Angus Peart (angus@3dr.com)
"""

import sys, argparse, struct, time

from pymavlink import mavutil
from pymavlink.dialects.v10 import ardupilotmega as mavlink

MAVLINK_SYSTEM_ID = 1
MAVLINK_COMPONENT_ID = 154

default_baudrate = 230400

def print_heartbeats(m):
    '''show incoming mavlink messages'''
    counter = 0
    while True:
        msg = m.recv_match(type="GOPRO_HEARTBEAT", blocking=True)
        if not msg:
            return
        if msg.get_type() == "BAD_DATA":
            if mavutil.all_printable(msg.data):
                sys.stdout.write(msg.data)
                sys.stdout.flush()
        else:
            print("%i GOPRO_HEARTBEAT=%i=%s" % (counter, msg.status, mavlink.enums['GOPRO_HEARTBEAT_STATUS'][msg.status].name))
            # Counter to stop repeating messages look like a console lock-up
            if counter == 9:
                counter = 0
            else:
                counter+=1

def show_messages(m):
    '''show incoming mavlink messages'''
    while True:
        msg = m.recv_match(blocking=True)
        if not msg:
            return
        if msg.get_type() == "BAD_DATA":
            if mavutil.all_printable(msg.data):
                sys.stdout.write(msg.data)
                sys.stdout.flush()
        else:
            print(msg)

def show_gopro(m):
    '''show incoming mavlink gopro messages'''
    while True:
        msg = m.recv_match(blocking=True)
        if not msg:
            return
        if msg.get_type() == "GOPRO_HEARTBEAT":
            print("%s: %s" % (msg.get_type(), mavlink.enums['GOPRO_HEARTBEAT_STATUS'][msg.status].name))
        elif msg.get_type() == "GOPRO_GET_RESPONSE":
            print("%s: '%s' = %s (%u)" % (msg.get_type(), mavlink.enums['GOPRO_COMMAND'][msg.cmd_id].name, msg.value, msg.status))
        elif msg.get_type() == "GOPRO_SET_RESPONSE":
            print("%s: '%s' = %u" % (msg.get_type(), mavlink.enums['GOPRO_COMMAND'][msg.cmd_id].name, msg.status))
        elif msg.get_type().startswith("GOPRO"):
            print(msg)

def wait_at_least(interval, last):
    # helper to delay a minimum interval since last
    diff = time.time() - last
    if diff < interval:
        time.sleep(interval - diff)

def vid_settings_test(link, m):
    '''
    set and readback all hero 4 black video settings
    '''

    # wait for heartbeat to ensure we're synced
    sys.stdout.write("...syncing")
    sys.stdout.flush()
    hb = m.recv_match(type="GOPRO_HEARTBEAT", blocking=True, timeout=5)
    if not hb:
        print ". error: failed to sync, didn't receive heartbeat"
        return

    print ". ready, starting video settings tests."

    failcount = 0
    last_iter = time.time()
    cmdid = mavlink.GOPRO_COMMAND_VIDEO_SETTINGS

    Hero4BlackSettings = [
        [0, 12, 0, 0],
        [1, 4, 0, 0],
        [1, 4, 1, 0],
        [1, 4, 2, 0],
        [1, 7, 0, 0],
        [1, 7, 1, 0],
        [1, 11, 0, 0],
        [1, 11, 1, 0],
        [1, 11, 2, 0],
        [1, 12, 2, 0],
        [2, 7, 0, 0],
        [2, 11, 0, 0],
        [3, 2, 0, 0],
        [3, 2, 1, 0],
        [3, 2, 2, 0],
        [3, 4, 0, 0],
        [3, 4, 1, 0],
        [3, 4, 2, 0],
        [3, 5, 0, 0],
        [3, 5, 1, 0],
        [3, 5, 2, 0],
        [3, 7, 0, 0],
        [3, 7, 1, 0],
        [3, 7, 2, 0],
        [3, 9, 0, 0],
        [3, 9, 2, 0],
        [3, 11, 0, 0],
        [3, 11, 2, 0],
        [4, 2, 0, 0],
        [4, 4, 0, 0],
        [4, 5, 0, 0],
        [4, 7, 0, 0],
        [4, 8, 0, 0],
        [6, 2, 0, 0],
        [6, 2, 1, 0],
        [6, 5, 0, 0],
        [6, 5, 1, 0],
        [6, 7, 0, 0],
        [6, 7, 1, 0],
        [7, 4, 0, 0],
        [8, 2, 0, 0],
        [8, 4, 0, 0],
        [10, 7, 0, 0],
        [10, 11, 0, 0],
        [11, 2, 0, 0],
        [11, 4, 0, 0],
        [11, 5, 0, 0],
        [11, 7, 0, 0],
        [11, 8, 0, 0],
        [12, 4, 0, 0],
        [13, 2, 0, 0]
    ]

    for i, s in enumerate(Hero4BlackSettings):

        # camera can freeze if we send too frequently,
        # adjust interval between iterations
        wait_at_least(0.2, last_iter)
        last_iter = time.time()

        link.gopro_set_request_send(MAVLINK_SYSTEM_ID, MAVLINK_COMPONENT_ID, cmdid, s)
        setrsp = m.recv_match(blocking=True, type="GOPRO_SET_RESPONSE", timeout=2)
        if not setrsp:
            print "!! case %d, no SET response" % (i)
            failcount += 1
            continue

        if setrsp.status != mavlink.GOPRO_REQUEST_SUCCESS:
            print "!! case %d, SET request failed" % (i)
            failcount += 1
            continue

        link.gopro_get_request_send(MAVLINK_SYSTEM_ID, MAVLINK_COMPONENT_ID, cmdid)
        getrsp = m.recv_match(blocking=True, type="GOPRO_GET_RESPONSE", timeout=2)
        if not getrsp:
            print "!! case %d, no GET response" % (i)
            failcount += 1
            continue

        if getrsp.status != mavlink.GOPRO_REQUEST_SUCCESS:
            print "!! case %d, GET request failed" % (i)
            failcount += 1
            continue

        if args.verbose:
            print "  -- case %d" % (i)
            print "  ", setrsp
            print "  ", getrsp

        if getrsp.value != s:
            print " case %d mismatch. got %s, want %s" % (i, getrsp.value, s)
            failcount += 1

    print "done. %d of %d failed" % (failcount, len(Hero4BlackSettings))

parser = argparse.ArgumentParser()
parser.add_argument("-p", "--port", help="Serial port or device used for MAVLink bootloading")
parser.add_argument("-b", "--baudrate", help="Serial port baudrate")
parser.add_argument("-v", "--verbose", help="more verbose output", action="store_true")
args = parser.parse_args()

# Accept a command line baudrate, fallback to the default
baudrate = default_baudrate
if args.baudrate:
    baudrate = args.baudrate

# Open the serial port
mavserial = mavutil.mavserial(
    device = args.port,
    baud = baudrate
)
link = mavlink.MAVLink(mavserial, 0, 0)

# link.gopro_get_request_send(MAVLINK_SYSTEM_ID, MAVLINK_COMPONENT_ID, mavlink.GOPRO_COMMAND_MODEL)

# link.gopro_get_request_send(MAVLINK_SYSTEM_ID, MAVLINK_COMPONENT_ID, mavlink.GOPRO_COMMAND_BATTERY)

# link.gopro_get_request_send(MAVLINK_SYSTEM_ID, MAVLINK_COMPONENT_ID, mavlink.GOPRO_COMMAND_CAPTURE_MODE)
# link.gopro_set_request_send(MAVLINK_SYSTEM_ID, MAVLINK_COMPONENT_ID, mavlink.GOPRO_COMMAND_CAPTURE_MODE, [0, 0, 0, 0])

# link.gopro_set_request_send(MAVLINK_SYSTEM_ID, MAVLINK_COMPONENT_ID, mavlink.GOPRO_COMMAND_SHUTTER, [1, 0, 0, 0])
# link.gopro_set_request_send(MAVLINK_SYSTEM_ID, MAVLINK_COMPONENT_ID, mavlink.GOPRO_COMMAND_SHUTTER, [0, 0, 0, 0])
# link.gopro_get_request_send(MAVLINK_SYSTEM_ID, MAVLINK_COMPONENT_ID, mavlink.GOPRO_COMMAND_SHUTTER)

# link.gopro_set_request_send(MAVLINK_SYSTEM_ID, MAVLINK_COMPONENT_ID, mavlink.GOPRO_COMMAND_POWER, [1, 0, 0, 0])

# Uncomment the block below to test stopping record when powering off
# link.gopro_set_request_send(MAVLINK_SYSTEM_ID, MAVLINK_COMPONENT_ID, mavlink.GOPRO_COMMAND_SHUTTER, [1, 0, 0, 0])
# time.sleep(5)
# link.gopro_set_request_send(MAVLINK_SYSTEM_ID, MAVLINK_COMPONENT_ID, mavlink.GOPRO_COMMAND_POWER, [0, 0, 0, 0])

#print_heartbeats(mavserial)

show_gopro(mavserial)

#show_messages(mavserial)
