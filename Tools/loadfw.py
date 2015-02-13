#!/usr/bin/python

"""
Utility for loading firmware into the 3DR Gimbal.

@author: Angus Peart (angus@3dr.com)
"""

import sys, signal
from pymavlink import mavutil
from pymavlink.dialects.v10 import common as mavlink
from data import *

MAVLINK_SYSTEM_ID = 50
MAVLINK_COMPONENT_ID = 230

MAVLINK_ENCAPSULATED_DATA_LENGTH = 253

serialport = "COM7"
baudrate = 230400

def wait_handshake(m):
    '''wait for a handshake so we know the target system IDs'''
    msg = m.recv_match(
        type='DATA_TRANSMISSION_HANDSHAKE',
        blocking = True,
        timeout = 1)
    if msg != None:
        if(msg.get_srcSystem() == MAVLINK_SYSTEM_ID and
           msg.get_srcComponent() == MAVLINK_COMPONENT_ID):
            return msg
    return None

# def show_messages(m):
#     '''show incoming mavlink messages'''
#     while True:
#         msg = m.recv_match(blocking=True)
#         if not msg:
#             return
#         if msg.get_type() == "BAD_DATA":
#             if mavutil.all_printable(msg.data):
#                 sys.stdout.write(msg.data)
#                 sys.stdout.flush()
#         else:
#             print(msg)



def load_binary(filename):
    '''load binary image file into a byte array'''
    with open(filename, "rb") as f:
        return bytearray(f.read())

def main():
    # Open the serial port
    mavserial = mavutil.mavserial(
        device = serialport,
        baud = baudrate
    )
    link = mavlink.MAVLink(mavserial, MAVLINK_SYSTEM_ID, MAVLINK_COMPONENT_ID)

    # Load the binary image into a byte array
    #binary = load_binary("Gimbal.bin")
    binary = bytearray(firmware_array)

    # Wait for a handshake from the gimbal which contains the payload length
    sys.stdout.write("Uploading firmware to gimbal ")
    finished = False

    # Loop until we are finished, TODO: timeout
    while(finished == False):
        msg = wait_handshake(mavserial)
        if(msg == None):
            # Handshake timed out
            sys.stdout.write('.')
        else:
            sequence_number = msg.width
            payload_length = msg.payload

            # Calculate the window of data to send
            start_idx = sequence_number*payload_length
            end_idx = (sequence_number+1)*payload_length

            # Slice the binary image
            data = binary[start_idx:end_idx]
            
            # Pad the data to fit the mavlink message
            if len(data) < MAVLINK_ENCAPSULATED_DATA_LENGTH:
                data.extend([0] * (MAVLINK_ENCAPSULATED_DATA_LENGTH - len(data)))
            
            # Send the data with the corrosponding sequence number
            link.encapsulated_data_send(sequence_number, data)
            
            # If the entire image buffer has been sent, exit this loop
            if end_idx >= len(binary):
                finished = True

            sys.stdout.write("!")

    link.data_transmission_handshake_send(mavlink.MAVLINK_TYPE_UINT16_T, 0, 0, 0, 0, 0, 0)
    print(" OK")

# Gracefully handle a interrupt signal
def signal_handler(signal, frame):
    sys.exit(0)
signal.signal(signal.SIGINT, signal_handler)

if __name__ == '__main__':
    main()
