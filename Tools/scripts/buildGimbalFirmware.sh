#!/bin/bash
set -e
set -v 

pwd
cd GimbalFirmware/build

/tmp/ti/ccsv6/utils/bin/gmake -k -j 8 all

# clean-up
find . -name '*.pp' -delete
find . -name '*.obj' -delete
find . -name '*.mk' -delete
find . -name '*.pyc' -delete
find . -name '*org.eclipse*' -delete