#!/usr/bin/env python

"""
Utility for building a release firmware bundle

@author: Angus Peart (angus@3dr.com)
@created: 25th Feb 2015
"""

import argparse
import base64
import json
import os
import subprocess
import sys
import time
import zlib

firmware_prefix = "gimbal_firmware_"
firmware_extension = "ax"

# Parse commandline arguments
parser = argparse.ArgumentParser(description="Firmware generator for the Axon Gimbal.")
parser.add_argument("--board_revision", required=True, action="store", help="set the board revision required")
parser.add_argument("--image", required=True, action="store", help="the firmware image")
parser.add_argument("--outdir", help="image output directory")
args = parser.parse_args()

# Object to hold the packged info
desc = {}

# Get the current git info
cmd = " ".join(["git", "describe", "--tags", "--dirty"])
p = subprocess.Popen(cmd, shell=True, stdout=subprocess.PIPE).stdout
desc['git_identity'] = str(p.read().strip())
p.close()

# Version is extracted from the git identity
desc['version'] = str(desc['git_identity'].split('-')[0][1:])

# Build time is the current time
desc['build_time'] = int(time.time())

# Convert the image to be JSON safe
with open(args.image, "rb") as f:
	bytes = f.read()
	desc['image_size'] = len(bytes)
	desc['image'] = base64.b64encode(zlib.compress(bytes,9)).decode('utf-8')

# Write the output
outputfile = os.path.join(args.outdir, "%s%s.%s" % (firmware_prefix, desc['version'], firmware_extension))
with open(outputfile, 'w') as f:
	f.write(json.dumps(desc, indent=4))
