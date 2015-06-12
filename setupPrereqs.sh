#!/bin/bash
set -e
set -v

CWD=$(pwd)

TI_TARBALL="ti-cgt-c2000_6.4.2.tar.gz"
TI_TARBALL_URL="http://gimbal-ci.s3-website-us-east-1.amazonaws.com/compiler/ti-cgt-c2000_6.4.2.tar.gz"

sudo apt-get update -y -qq
sudo apt-get install gcc-multilib -y -qq

mkdir -p /tmp
cd /tmp
wget $TI_TARBALL_URL
tar xf ${TI_TARBALL}
rm -f ${TI_TARBALL}
cd $CWD