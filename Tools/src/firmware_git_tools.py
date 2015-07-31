import subprocess, os, sys, glob, re

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
	git_identity = ''
	try:
		# Get the current git info
		cmd = " ".join([os_git_command, "describe", "--tags", "--dirty", "--long"])
		p = subprocess.Popen(cmd, shell=True, stdout=subprocess.PIPE).stdout
		git_identity = str(p.read().strip())
		p.close()
	except Exception:
		pass
	finally:
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
