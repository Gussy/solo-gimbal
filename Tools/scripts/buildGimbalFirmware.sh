#!/bin/bash
. ~/.profile
set -e
set -v 

pwd
cd GimbalFirmware

make -j8 all

# clean-up
find . -name '*.pp' -delete
find . -name '*.obj' -delete
find . -name '*.mk' -delete
find . -name '*.pyc' -delete
find . -name '*org.eclipse*' -delete