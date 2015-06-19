#!/usr/bin/env python

"""
Utility for generating a version header file

"""

import argparse, subprocess, os, sys, glob, re

def osGitCommand():
	# Determine the shell git command based on the current platform
	# For windows, we assume that the git binary is in the default install location
	os_git_command = ""
	if sys.platform.startswith('linux') or sys.platform.startswith('darwin'):
		os_git_command = "git"
	elif sys.platform.startswith('win32'):
		github_path = os.environ['LOCALAPPDATA'] + "\\GitHub\\"
		# First, check if git is installed in AppData\Local (this seems to happen with some git installs)
		if os.path.exists(os.environ['LOCALAPPDATA'] + "\\Programs\\Git\\"):
			os_git_command = "\"" + os.environ['LOCALAPPDATA'] + "\\Programs\\Git\\bin\\git.exe" + "\""
		# Check for git installed with GFW (https://windows.github.com/)
		elif os.path.exists(github_path):
			PortableGit = glob.glob(os.path.join(github_path, "PortableGit*"))
			if PortableGit:
				git_path = PortableGit[0]
				os_git_command = "\"" + git_path + "\\bin\\git.exe" + "\""
		else:
			# If not, git is either in Program Files or Program Files (x86)
			if 'PROGRAMFILES(X86)' in os.environ:
				os_git_command = "\"" + os.environ['PROGRAMFILES(X86)'] + "\\Git\\bin\\git.exe" + "\""
			else:
				os_git_command = "\"" + os.environ['PROGRAMFILES'] + "\\Git\\bin\\git.exe" + "\""

	# Check that a git command was found
	if os_git_command == "":
		sys.exit("ERROR: Git command not found for this operating system")

	return os_git_command

def gitIdentity(os_git_command):
	# Get the current git info
	cmd = " ".join([os_git_command, "describe", "--tags", "--dirty", "--long"])
	p = subprocess.Popen(cmd, shell=True, stdout=subprocess.PIPE).stdout
	git_identity = str(p.read().strip())
	p.close()
	return git_identity

def gitBranch(os_git_command):
	cmd = " ".join([os_git_command, "branch", "--list"])
	p = subprocess.Popen(cmd, shell=True, stdout=subprocess.PIPE).stdout
	re_branch = re.search(ur"\*\s(.*)\n", str(p.read()))
	git_branch = re_branch.group(1)
	p.close()
	if re.search(ur"(no branch)", git_branch):
		git_branch = "no branch"
	elif re.search(ur"(detached) from [a-f0-9]{7}", git_branch):
		git_branch = "detached"
	return git_branch

if __name__ == '__main__':
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
		f.write("#define GitVersionMajorInt %i\n" % int(git_semver[0]))
		f.write("#define GitVersionMinorInt %i\n" % int(git_semver[1]))
		f.write("#define GitVersionRevisionInt %i\n" % int(git_semver[2]))
