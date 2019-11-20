#!/bin/bash


if [ "$1" == "clean" ]
then
echo cleaning packages
rm -rd build devel install
fi

. /etc/lsb-release

if [ $DISTRIB_RELEASE == "18.04" ]
then
rosv="melodic"
elif [ $DISTRIB_RELEASE == "16.04" ]
then
rosv="kinetic"
else
    echo "error: ROS kinetic must be installed for this build to work."
    echo "Remedy: sudo apt-get install ros-kinetic-desktop-full"
    echo "It is not recommeded to install two versions of ROS on the same machine - e.g., kinetic and lunar"
    exit 1
fi

export rosv

source /opt/ros/$rosv/setup.bash


# Check if all the installation prerequisites are met.
bash ./isvalid.bash
valid=$?
echo valid is $valid

# fail if installation prerequisites are not met.
if [ "$valid" != "0" ]
then
    echo "abort: prerequisite missing"
    exit 1
fi
source /opt/ros/$rosv/setup.bash

catkin_make install  --only-pkg-with-deps  crcl_rosmsgs
catkin_make install  --only-pkg-with-deps  gz_custom_messages
catkin_make install -DCATKIN_WHITELIST_PACKAGES=""


