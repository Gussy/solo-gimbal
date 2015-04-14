#!/usr/bin/python

"""
Utility for loading firmware into the 3DR Gimbal.

"""

import sys

from pymavlink.dialects.v10 import common as mavlink
from firmware_helper import append_checksum, load_firmware


MAVLINK_ENCAPSULATED_DATA_LENGTH = 253

def wait_handshake(m, timeout=1):
    '''wait for a handshake so we know the target system IDs'''
    msg = m.recv_match(
        type='DATA_TRANSMISSION_HANDSHAKE',
        blocking=True,
        timeout=timeout)
    if msg != None:
        if(msg.get_srcComponent() == mavlink.MAV_COMP_ID_GIMBAL):
            return msg
    return None

def update(binary, link):
    print (# Load the binary image into a byte array
        "Application binary: %s" % binary)
    hexfile = load_firmware(binary)
    binary = append_checksum(hexfile)
# Wait for a handshake from the gimbal which contains the payload length
    sys.stdout.write("Uploading firmware to gimbal ")
    sys.stdout.flush()
    finished = False
    timeout_counter = 0
# Loop until we are finished
    while (finished == False):
        # Wait for target to reset into bootloader mode
        msg = wait_handshake(link.file)
        if (msg == None):
    # Signal the target to reset into bootloader mode
            link.data_transmission_handshake_send(mavlink.MAVLINK_TYPE_UINT16_T, 0, 0, 0, 0, 0, 0)
    # Handshake timed out
            sys.stdout.write('.')
            sys.stdout.flush()
    # Timeout after ~10 seconds without messages
            if timeout_counter > 10:
                print ("\nNot response from gimbal, exiting.")
                sys.exit(1)
            timeout_counter += 1
        else:
            timeout_counter = 0
            sequence_number = msg.width
            payload_length = msg.payload
            # Print the bootloader version on the first data handshake
            if sequence_number == 0:
                version_major = (msg.height >> 8) & 0xff
                version_minor = msg.height & 0xff
                sys.stdout.write(' (BL Ver %i.%i) ' % (version_major, version_minor))
                sys.stdout.flush()
            # Calculate the window of data to send
            start_idx = sequence_number * payload_length
            end_idx = (sequence_number + 1) * payload_length
            # Clamp the end index from overflowing
            if (end_idx > len(binary)):
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
            sys.stdout.flush()
    
# Send an "end of transmission" signal to the target, to cause a target reset
    while True:
        link.data_transmission_handshake_send(mavlink.MAVLINK_TYPE_UINT16_T, 0, 0, 0, 0, 0, 0)
        msg = wait_handshake(link.file, timeout=10)
        sys.stdout.flush()    
        if msg == None:
            print(" timeout")
            break
        if msg.width == 0xFFFF:
            print(" OK")
            break
    
