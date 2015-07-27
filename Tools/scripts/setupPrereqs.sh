#!/bin/bash
set -e
set -v

CWD=$(pwd)
OPT="$HOME/opt"

TI_TARBALL="ti-cgt-c2000_6.4.2.tar.gz"
TI_TARBALL_URL="http://gimbal-ci.s3-website-us-east-1.amazonaws.com/compiler/ti-cgt-c2000_6.4.2.tar.gz"

mkdir -p $OPT
cd $OPT
wget $TI_TARBALL_URL
tar xf ${TI_TARBALL}
rm -f ${TI_TARBALL}

exportline="export TI_ROOT=$OPT/ti/ccsv6/tools/compiler/ti-cgt-c2000_6.4.2/";
echo $exportline >> ~/.profile

. ~/.profile
echo $PATH

cd $CWD