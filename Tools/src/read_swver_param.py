#!/usr/bin/python

"""
Utility for reading the software version from a 3DR Gimbal.

"""

import sys, struct

def float_to_bytes(f):
    return struct.unpack('4b', struct.pack('<f', f))


def readSWver(link):
    # Get a parameter
    link.param_request_read_send(link.srcSystem, link.srcComponent, "SYSID_SWVER", -1)
# Wait 10 seconds for a response
    msg = link.file.recv_match(type="PARAM_VALUE", blocking=True, timeout=10)
    if not msg:
        print "Requested param not received."
        sys.exit(1)
    else:
        swver_raw = float_to_bytes(msg.param_value)
        print "SW Version %i.%i.%i" % (swver_raw[3], swver_raw[2], swver_raw[1])