#!/usr/bin/env python

"""
Utility for generating a version header file

"""

import argparse, struct
from firmware_git_tools import osGitCommand, gitIdentity, gitBranch

def bytes4_to_float(b1, b2, b3, b4):
	return struct.unpack('>f', struct.pack('4b', b1, b2, b3, b4))

# Parse commandline arguments
parser = argparse.ArgumentParser(description="Utility for generating a version header file")
parser.add_argument("output", help="image output file")
args = parser.parse_args()

os_git_command = osGitCommand()
git_identity = gitIdentity(os_git_command)

git_tag = git_identity.split('-')[0]
git_semver = git_tag[1:].split('.')
version_float = bytes4_to_float(int(git_semver[0]), int(git_semver[1]), int(git_semver[2]), 0)[0]

# Write the header file
with open(args.output, 'w') as f:
	f.write("#define GitVersionString \"%s\"\n" % git_identity)
	f.write("#define GitVersionFloat %s\n" % version_float)
