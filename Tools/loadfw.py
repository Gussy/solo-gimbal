#!/usr/bin/python

"""
Utility for loading firmware into the 3DR Gimbal.

@author: Angus Peart (angus@3dr.com)
"""

import sys, signal, argparse
from pymavlink import mavutil
from pymavlink.dialects.v10 import common as mavlink

MAVLINK_SYSTEM_ID = 50
MAVLINK_COMPONENT_ID = 230

MAVLINK_ENCAPSULATED_DATA_LENGTH = 253

# TODO: Add these as arguments instead of being hardcoded
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
    '''Load binary image file into a byte array'''
    with open(filename, "rb") as f:
        return bytearray(f.read())

def bytearray_to_wordarray(data):
    '''Converts an 8-bit byte array into a 16-bit word array'''
    wordarray = list()

    for i in range(len(data)/2):
        # Calculate 16 bit word from two bytes
        msb = data[(i*2)+0]
        lsb = data[(i*2)+1]
        word = (msb << 8) | lsb
        wordarray.append(word)

    return wordarray

def append_checksum(binary):
    '''Calculate and append the XOR checksum to the bytearray'''
    checksum = 0xFFFF
    wordarray = bytearray_to_wordarray(binary)

    # Compute the checksum
    for i in range(len(wordarray)):
        checksum ^= wordarray[i]
    print("Checksum: 0x%04X" % checksum)

    # Add the checksum to the end of the wordarray
    wordarray.extend([checksum&0xFFFF, (checksum&0xFFFF)>>16, 0x0000])

    # Convert the wordarray back into a bytearray
    barray = list()
    for i in range(len(wordarray)):
        lsb = wordarray[i] & 0xFF
        msb = (wordarray[i] >> 8) & 0xFF
        barray.append(lsb)
        barray.append(msb)

    return barray


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("binary", help="application binary file load")
    args = parser.parse_args()

    # Open the serial port
    mavserial = mavutil.mavserial(
        device = serialport,
        baud = baudrate
    )
    link = mavlink.MAVLink(mavserial, MAVLINK_SYSTEM_ID, MAVLINK_COMPONENT_ID)

    # Load the binary image into a byte array
    print("Application binary: %s" % args.binary)
    hexfile = load_binary(args.binary)
    binary = append_checksum(hexfile)

    # Wait for a handshake from the gimbal which contains the payload length
    sys.stdout.write("Uploading firmware to gimbal ")
    finished = False

    # Loop until we are finished, TODO: timeout
    while(finished == False):
        # Wait for target to reset into bootloader mode
        msg = wait_handshake(mavserial)
        if(msg == None):
            # Signal the target to reset into bootloader mode
            link.data_transmission_handshake_send(mavlink.MAVLINK_TYPE_UINT16_T, 0, 0, 0, 0, 0, 0)

            # Handshake timed out
            sys.stdout.write('.')
        else:
            sequence_number = msg.width
            payload_length = msg.payload

            # Calculate the window of data to send
            start_idx = sequence_number*payload_length
            end_idx = (sequence_number+1)*payload_length

            # Clamp the end index from overflowing
            if(end_idx > len(binary)):
                end_idx = len(binary)

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

    # Send an "end of transmission" signal to the target, to cause a target reset
    link.data_transmission_handshake_send(mavlink.MAVLINK_TYPE_UINT16_T, 0, 0, 0, 0, 0, 0)
    print(" OK")

# Gracefully handle a interrupt signal
def signal_handler(signal, frame):
    sys.exit(0)
signal.signal(signal.SIGINT, signal_handler)

if __name__ == '__main__':
    main()
