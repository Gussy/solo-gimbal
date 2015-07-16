#!/usr/bin/env python

"""
Utility for generating a version header file

"""

import argparse
from firmware_git_tools import osGitCommand, gitIdentity, gitBranch

# Parse commandline arguments
parser = argparse.ArgumentParser(description="Utility for generating a version header file")
parser.add_argument("output", help="image output file")
args = parser.parse_args()

os_git_command = osGitCommand()
git_identity = gitIdentity(os_git_command)
git_branch = gitBranch(os_git_command)

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
