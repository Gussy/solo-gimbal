#!/bin/sh
#
# script to re-generate mavlink C code for APM
#
# NOTE: we currently rely on a specific branch of the mavlink generator to accommodate the c2000.
#       before running this script, ensure you're using the generator in http://github.com/3drobotics/mavlink-solo
#       on the 'c2000' branch.
#
#       Longer term, we'll ideally contribute c2000 friendly changes to upstream mavlink so this is not required.
#       See the discussion on Arthur's PR with the current changes for a review of the changes that must be made:
#       https://github.com/mavlink/mavlink/pull/303
#

if ! which mavgen.py > /dev/null; then
    echo "mavgen.py must be in your PATH. Get it from http://github.com/3drobotics/mavlink-solo in the pymavlink/generator directory"
    exit 1
fi

echo "Removing old includes"
rm -rf ../mavlink_library/

echo "Generating C code"
pwd
mavgen.py --c2000 --lang=C --output=../mavlink_library/ ardupilotmega.xml
