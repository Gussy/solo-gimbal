#!/bin/bash
set -e
set -v 

pwd
cd GimbalFirmware/build

/tmp/ti/ccsv6/utils/bin/gmake -k all

# clean-up
find . -name '*.pp' -delete
find . -name '*.obj' -delete
find . -name '*.mk' -delete
find . -name '*.pyc' -delete
find . -name '*org.eclipse*' -delete