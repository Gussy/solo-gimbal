import sys, signal, serial
from setup_mavlink import getSerialPorts

MAX_RPM = 230

_port = None

def init_fixture(wobbleport=None):
    # signal.signal(signal.SIGINT, signal_handler)
    return open_fixture_comm(wobbleport)

def set_rpm(port, rpm):
    if port is None or rpm is None:
        return False
    elif isinstance(rpm, int) and rpm < MAX_RPM:
        port.write(chr(rpm))
        return True

def open_fixture_comm(port=None):
    global _port
    if not port:
        serial_list = getSerialPorts(preferred_list=['*Arduino*'])
        if len(serial_list) >= 1:
            port = serial_list[0].device
        else:
            return None

    try:
        # we rather strangely set the baudrate initially to 1200, then change to the desired
        # baudrate. This works around a kernel bug on some Linux kernels where the baudrate
        # is not set correctly
        _port = serial.Serial(port, 1200, timeout=0, dsrdtr=False, rtscts=False, xonxoff=False)
        _port.setBaudrate(115200)
        return _port
    except serial.serialutil.SerialException:
        return None

def close(port=None):
    global _port
    if port:
        port.close()
    elif _port:
        _port.close()

# def signal_handler(signal, frame):
#     global _port
#     set_rpm(_port, 0)
#     sys.exit(0)
