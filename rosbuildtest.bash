#!/bin/bash



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

#catkin_make install  --only-pkg-with-deps  gokin_plugin
#catkin_make install  --only-pkg-with-deps  ikfast_fanuc_plugin
#catkin_make install  --only-pkg-with-deps  gzrcsdemo
#catkin_make install  --only-pkg-with-deps  crcl_rosmsgs


catkin_make install  --only-pkg-with-deps gzmodelplugin
read  -n 1 -p "Wait gzmodelplugin" mainmenuinput

catkin_make install  --only-pkg-with-deps  aprs_headers
read  -n 1 -p "Wait aprs_headers" mainmenuinput

catkin_make install  --only-pkg-with-deps  crcl_rosmsgs
read  -n 1 -p "Wait crcl_rosmsgs" mainmenuinput

catkin_make install  --only-pkg-with-deps gz_custom_messages
read  -n 1 -p "Wait gz_custom_messages" mainmenuinput

catkin_make install  --only-pkg-with-deps gotraj
read  -n 1 -p "Waitgotraj" mainmenuinput

catkin_make install  --only-pkg-with-deps crcllib
read  -n 1 -p "Wait crcllib" mainmenuinput

catkin_make install  --only-pkg-with-deps gzjointcmdplugin
read  -n 1 -p "Wait gzjointcmdplugin" mainmenuinput



catkin_make install  --only-pkg-with-deps gzparallelgripperplugin
read  -n 1 -p "Wait gzparallelgripperplugin" mainmenuinput

catkin_make install  --only-pkg-with-deps ikfast_fanuc_plugin
read  -n 1 -p "Wait ikfast_fanuc_plugin" mainmenuinput

catkin_make install  --only-pkg-with-deps gzrcs
read  -n 1 -p "Wait gzrcs" mainmenuinput

