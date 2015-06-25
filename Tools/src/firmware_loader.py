#!/usr/bin/python

"""
Utility for loading firmware into the 3DR Gimbal.

"""

import sys

from firmware_helper import append_checksum, load_firmware
import setup_mavlink

outputMethod = sys.stdout.write

MAVLINK_ENCAPSULATED_DATA_LENGTH = 253

def print_and_flush(obj):
    global outputMethod
    
    if outputMethod is sys.stdout.write and 'message' in obj:
        outputMethod(obj['message'])
    else:
        outputMethod(obj)

    # The flush is required to refresh the screen on Ubuntu
    sys.stdout.flush()

def printStatusMessage(msg, level='info'):
    print_and_flush({
        'type': 'status',
        'level': level,
        'message': msg
    })

def printProgressMessage(uploaded_kb, total_kb, percentage, msg):
    print_and_flush({
        'type': 'bootloader_progress',
        'sent': uploaded_kb,
        'total': total_kb,
        'percentage': percentage,
        'message': msg
    })

def decode_bootloader_version(msg):
    """The first message handshake contains the bootloader version int the height field as a 16bit int"""
    version_major = (msg.height >> 8) & 0xff
    version_minor = msg.height & 0xff
    return [version_major, version_minor]

def start_bootloader(link):
    """Check if target is in booloader, if not reset into bootloader mode"""
    
    msg = setup_mavlink.wait_handshake(link.file, timeout=4)
    if (msg is not None):
        printStatusMessage('Target already in bootloader mode\n')
        return
    else:
        printStatusMessage("Restarting target in bootloader mode\n")
    
    timeout_counter = 0;
    while(True):
        # Signal the target to reset into bootloader mode
        setup_mavlink.reset_into_bootloader(link)
        msg = setup_mavlink.wait_handshake(link.file, timeout=4)
        timeout_counter += 1
        
        if (msg is None):
            print_and_flush('.')
            if timeout_counter > 2:
                printStatusMessage("\nNo response from gimbal, exiting.\n", level='error')
                sys.exit(1)
        else:
            break


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


def get_handshake_msg(link, timeout=5):
    msg = setup_mavlink.wait_handshake(link.file, timeout)
    if (msg == None):
        # Handshake timed out
        printStatusMessage("\nNot response from gimbal, exiting.\n", level='error')
        sys.exit(1)
    return msg

def upload_data(link, binary):
    msg = get_handshake_msg(link, timeout=10)    

    # Print bootloader version
    blver = decode_bootloader_version(msg)
    print_and_flush({
        'type': 'bootloader_version',
        'major': blver[0],
        'minor': blver[1],
        'message': 'Bootloader Ver %i.%i\n' % (blver[0], blver[1])
    })
    
    # Loop until we are finished
    end_idx = 0
    while (end_idx < len(binary)):
        msg = get_handshake_msg(link, timeout=1) # Note: If this timeout is 10, windows will choke and send a block every 10s...
        end_idx = send_block(link, binary, msg)
        
        uploaded_kb = round(end_idx / 1024.0, 2)
        total_kb = round(len(binary) / 1024.0, 2)
        percentage = int((100.0 * end_idx) / len(binary))
        text = "\rUpload %2.2fkB of %2.2fkB - %d%%     " % (uploaded_kb, total_kb, percentage)
        printProgressMessage(uploaded_kb, total_kb, percentage, text)
    printProgressMessage(uploaded_kb, total_kb, percentage, '\n')
            
def finish_upload(link):
    """Send an "end of transmission" signal to the target, to cause a target reset""" 
    while True:
        setup_mavlink.reset_into_bootloader(link)
        msg = setup_mavlink.wait_handshake(link.file, timeout=10)
        if msg == None:
            printStatusMessage("Timeout\n", level='error')
            break
        if msg.width == 0xFFFF:
            printStatusMessage("Upload successful\n")
            break    

def update(firmware_file, link, outputHandler=None):
    global outputMethod

    # Use a callback for all output if required
    if outputHandler:
        outputMethod = outputHandler

    # Print the firmware filename
    image = load_firmware(firmware_file)
    printStatusMessage('Application firmware_file: %s\n' % firmware_file)
    
    binary, checksum = append_checksum(image)
    print_and_flush({
        'type': 'bootloader_checksum',
        'checksum': checksum,
        'message': 'Checksum: 0x%04X\n' % checksum
    })
    
    start_bootloader(link)
    upload_data(link, binary)
    finish_upload(link)
    