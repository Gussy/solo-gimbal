#!/usr/bin/python

"""
Utility for loading firmware into the 3DR Gimbal.

"""

import sys

from firmware_helper import append_checksum, load_firmware
import setup_mavlink

bootloaderVersionHandler = None
progressHandler = None

MAVLINK_ENCAPSULATED_DATA_LENGTH = 253

class Results:
    Success, NoResponse, Timeout, InBoot, Restarting = 'Success', 'NoResponse', 'Timeout', 'InBoot', 'Restarting'

def decode_bootloader_version(msg):
    """The first message handshake contains the bootloader version int the height field as a 16bit int"""
    version_major = (msg.height >> 8) & 0xff
    version_minor = msg.height & 0xff
    return [version_major, version_minor]

def start_bootloader(link):
    """Check if target is in booloader, if not reset into bootloader mode"""
    msg = setup_mavlink.wait_handshake(link, timeout=4)
    if msg != None:
        return Results.InBoot
    
    timeout_counter = 0;
    while True:
        # Signal the target to reset into bootloader mode
        setup_mavlink.reset_into_bootloader(link)
        msg = setup_mavlink.wait_handshake(link)
        timeout_counter += 1
        
        if msg == None:
            if timeout_counter > 20:
                return Results.NoResponse
        else:
            break
    return Results.Restarting

def send_block(link, binary, msg):
    sequence_number = msg.width
    payload_length = msg.payload

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
    setup_mavlink.send_bootloader_data(link, sequence_number, data)
    return end_idx

def upload_data(link, binary):
    global progressHandler, bootloaderVersionHandler

    msg = setup_mavlink.wait_handshake(link)
    if msg == None:
        return Results.NoResponse

    # Print bootloader version
    if bootloaderVersionHandler:
        blver = decode_bootloader_version(msg)
        bootloaderVersionHandler(blver[0], blver[1])
    
    # Loop until we are finished
    end_idx = 0
    retries = 0
    while end_idx < len(binary):
        msg = setup_mavlink.wait_handshake(link)
        if msg == None:
            retries += 1
            continue
        if retries > 20:
            return Results.NoResponse
        retries = 0

        end_idx = send_block(link, binary, msg)
        
        uploaded_kb = round(end_idx / 1024.0, 2)
        total_kb = round(len(binary) / 1024.0, 2)
        percentage = int((100.0 * end_idx) / len(binary))
        if progressHandler:
            progressHandler(uploaded_kb, total_kb, percentage)

    return Results.Success
            
def finish_upload(link):
    """Send an "end of transmission" signal to the target, to cause a target reset""" 
    while True:
        setup_mavlink.reset_into_bootloader(link)
        msg = setup_mavlink.wait_handshake(link)
        if msg == None:
            return Results.Timeout
        if msg.width == 0xFFFF:
            return Results.Success

def load_binary(binary, link,  bootloaderVersionCallback=None, progressCallback=None):
    global bootloaderVersionHandler, progressHandler

    if progressCallback:
        progressHandler = progressCallback

    if bootloaderVersionCallback:
        bootloaderVersionHandler = bootloaderVersionCallback

    result = upload_data(link, binary)
    if result != Results.Success:
        return result

    return finish_upload(link)
    