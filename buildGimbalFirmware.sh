#!/bin/bash
set -e
set -v 

cd GimbalFirmware/build
pwd
ls -l /tmp/ti/ccsv6/utils/bin/
/tmp/ti/ccsv6/utils/bin/gmake -k all

# clean-up
find . -name '*.pp' -delete
find . -name '*.obj' -delete
find . -name '*.mk' -delete
find . -name '*.pyc' -delete
find . -name '*org.eclipse*' -delete