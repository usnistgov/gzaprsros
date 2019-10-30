#!/bin/bash


if [ "$1" == "clean" ]
then
echo cleaning packages
rm -rd build devel install
fi

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

catkin_make install  --only-pkg-with-deps  crcl_rosmsgs
catkin_make install -DCATKIN_WHITELIST_PACKAGES=""


