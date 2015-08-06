#!/bin/sh
#
# script to re-generate mavlink C code for APM
#
# NOTE: we currently rely on a specific branch of the mavlink generator to accommodate the c2000.
#       before running this script, ensure you're using the generator in http://github.com/3drobotics/mavlink-solo
#       on the 'c2000-generator-only' branch.
#
#       Longer term, we'll ideally contribute c2000 friendly changes to upstream mavlink so this is not required.
#       See the discussion on Arthur's PR with the current changes for a review of the changes that must be made:
#       https://github.com/mavlink/mavlink/pull/303
#
#       this script expects mavlink-solo to be at a sibling level to the solo-gimbal repo,
#       but you can optionally pass in the path to mavlink-solo
#
#       this script expects to be from the top level dir in solo-gimbal
#


#
# change directories into the mavlink-solo repo,
# so we can use `python -m`, which relies on the layout
# of the local relative path.
#
# make note of our current directory so we know where to direct the generator output.
#

GIMBALDIR=`pwd`
MAVSOLO=../mavlink-solo

echo "Removing old includes"
rm -rf ${GIMBALDIR}/shared/mavlink_library/

echo "Generating C code"

# can pass in mavlink-solo location to override default
if [ "$#" -eq  "1" ]
  then
MAVSOLO=$1
fi

pushd ${MAVSOLO}
# ...could verify that we're on the c2000 branch...
python -m pymavlink.tools.mavgen --c2000 --lang C -o ${GIMBALDIR}/shared/mavlink_library message_definitions/v1.0/ardupilotmega.xml
popd
