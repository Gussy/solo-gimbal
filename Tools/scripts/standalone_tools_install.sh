#!/bin/bash
APT_GET="sudo apt-get -qq --assume-yes"

BASE_PKGS="gawk make git curl"
SITL_PKGS="g++ python-pip python-matplotlib python-serial python-wxgtk2.8 python-scipy python-opencv python-numpy python-pyparsing ccache python-empy"
UBUNTU64_PKGS="libc6:i386 libgcc1:i386 gcc-4.6-base:i386 libstdc++5:i386 libstdc++6:i386 gcc-multilib"
PYTHON_PKGS="pymavlink MAVProxy catkin_pkg"

$APT_GET update
$APT_GET install $BASE_PKGS $SITL_PKGS $UBUNTU64_PKGS

sudo pip install --upgrade pip || {
    echo "pip upgrade failed"
}
sudo pip install --upgrade setuptools || {
    echo "setuptools upgrade failed"
}
for pkg in $PYTHON_PKGS; do
    echo "Installing $pkg"
    sudo pip -q install $pkg || echo "FAILED INSTALL OF $pkg"
done

sudo usermod -a -G dialout $USER

cd
git clone https://github.com/3drobotics/mavlink-solo.git
cd ~/mavlink-solo/pymavlink/
git checkout gimbal_update
sudo python setup.py install

cd
git clone https://github.com/3drobotics/MAVProxy.git
cd ~/MAVProxy/
git checkout watch-gopro
sudo python setup.py install

cd