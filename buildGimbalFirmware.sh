#!/bin/bash
set -e
set -v 

cd GimbalFirmware/build
pwd
ls -l /tmp/ti/ccsv6/utils/bin/
/tmp/ti/ccsv6/utils/bin/gmake -k all