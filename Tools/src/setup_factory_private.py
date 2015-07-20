import struct, time, datetime
from pymavlink.mavparm import MAVParmDict
import setup_param

def uint32_to_float(i):
    return struct.unpack('f', struct.pack('<I', i))[0]

def string12_to_float3(s):
    return struct.unpack('3f',struct.pack('<12s', s))

def set_assembly_date(link, commit=True):
    timestamp = time.time()
    setup_param.set_param(link, "GMB_ASM_TIME", uint32_to_float(timestamp))
    if commit:
        setup_param.commit_to_flash(link)
    return timestamp

def set_serial_number(link, serial_str, commit=True):
    sanitised_serial = str(serial_str)[:12].upper()
    ser_num_1, ser_num_2, ser_num_3 = string12_to_float3(sanitised_serial)
    setup_param.set_param(link, "GMB_SER_NUM_1", ser_num_1)
    setup_param.set_param(link, "GMB_SER_NUM_2", ser_num_2)
    setup_param.set_param(link, "GMB_SER_NUM_3", ser_num_3)
    if commit:
        setup_param.commit_to_flash(link)
    return sanitised_serial

def set_serial_number_with_time(link, serial_str):
    sanitised_serial = set_serial_number(link, serial_str, commit=False)
    timestamp = set_assembly_date(link, commit=False)
    setup_param.commit_to_flash(link)
    return sanitised_serial, timestamp

def set_serial_number_3dr(link, month_serial_number):
    today = datetime.date.today() 
    
    # Build year identifier
    year = hex((today.year - 2010) % 16)[2]
    
    # Build month indentifier
    if today.month < 10:
        month = today.month
    elif today.month == 10:
        month = '0'
    elif today.month == 11:
        month = 'A'
    elif today.month == 12:
        month = 'B'        
    
    # Build per-mount serial number (5 digits)
    number = '%05d' % (month_serial_number % 100000)    
    
    # Build serial number string
    serial_str = 'GB11A' + str(year) + str(month) + str(number)
    
    # Save the serial number on the gimbal
    set_serial_number(link, serial_str)
    
    return serial_str

def reset(link):
    parameters = MAVParmDict()
    parameters.mavset(link.file, "GMB_SER_NUM_1", 0.0, 3)
    parameters.mavset(link.file, "GMB_SER_NUM_1", 0.0, 3)
    parameters.mavset(link.file, "GMB_SER_NUM_1", 0.0, 3)
    parameters.mavset(link.file, "GMB_ASM_TIME", 0.0, 3)
    setup_param.commit_to_flash(link)

def full_reset(link):
    return link.file.mav.command_long_send(link.target_sysid, link.target_compid, 42505, 0.0, 42.0, 49.0, 12.0, 26.0, 64.0, 85.0, 42.0)
