from setup_mavlink import getSerialPorts
import serial

MAX_RPM = 300

def init_fixture():
    return open_fixture_comm()

def set_rpm(port, rpm):
    if port is None or rpm is None:
        return False
    elif isinstance(rpm, int) and rpm < MAX_RPM:
        port.write(chr(rpm))
        return True

def open_fixture_comm(port=None):
    if not port:
        serial_list = getSerialPorts(preferred_list=['*Arduino*'])
        if len(serial_list) == 1:
            device = serial_list[0].device
        else:
            return None

    # we rather strangely set the baudrate initially to 1200, then change to the desired
    # baudrate. This works around a kernel bug on some Linux kernels where the baudrate
    # is not set correctly
    port = serial.Serial(device, 1200, timeout=0, dsrdtr=False, rtscts=False, xonxoff=False)
    port.setBaudrate(115200)
    return port