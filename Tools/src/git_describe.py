#!/usr/bin/env python

"""
Utility for generating a version header file

@author: Angus Peart (angus@3dr.com)
@created: 28th Feb 2015
"""

import argparse
import os
import subprocess
import sys

# Parse commandline arguments
parser = argparse.ArgumentParser(description="Utility for generating a version header file")
parser.add_argument("output", help="image output file")
args = parser.parse_args()

# Determine the shell git command based on the current platform
# For windows, we assume that the git binary is in the default install location
os_git_command = ""
if sys.platform.startswith('linux'):
	os_git_command = "git"
elif sys.platform.startswith('win32'):
	if 'PROGRAMFILES(X86)' in os.environ:
		os_git_command = "\"" + os.environ['PROGRAMFILES(X86)'] + "\\Git\\bin\\git.exe" + "\""
	else:
		os_git_command = "\"" + os.environ['PROGRAMFILES'] + "\\Git\\bin\\git.exe" + "\""

# Get the current git info
cmd = " ".join([os_git_command, "describe", "--tags", "--dirty", "--long"])
p = subprocess.Popen(cmd, shell=True, stdout=subprocess.PIPE).stdout
git_identity = str(p.read().strip())
p.close()

cmd = " ".join([os_git_command, "branch", "--list"])
p = subprocess.Popen(cmd, shell=True, stdout=subprocess.PIPE).stdout
git_branch = str(p.read().strip().split('*')[1].split('\n')[0].strip())
p.close()

git_tag = git_identity.split('-')[0]
git_semver = git_tag[1:].split('.')

# Write the header file
with open(args.output, 'w') as f:
	f.write("#define GitVersionString \"%s\"\n" % git_identity)
	f.write("#define GitBranch \"%s\"\n" % git_branch)
	f.write("#define GitTag \"%s\"\n" % git_tag)
	f.write("#define GitCommit \"%s\"\n" % git_identity.split('-')[2])
	f.write("#define GitVersionMajor \"%s\"\n" % git_semver[0])
	f.write("#define GitVersionMinor \"%s\"\n" % git_semver[1])
	f.write("#define GitVersionRevision \"%s\"\n" % git_semver[2])
	f.write("#define GitVersionMajorInt %i\n" % int(git_semver[0]))
	f.write("#define GitVersionMinorInt %i\n" % int(git_semver[1]))
	f.write("#define GitVersionRevisionInt %i\n" % int(git_semver[2]))
