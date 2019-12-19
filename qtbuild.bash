#!/bin/bash


display_usage() { 
	echo "This script builds the gzaprsros packages using qt." 
	echo "If the argument clean is supplied, then each package makefile is invoked with clean before compilation." 
	echo -e "\nUsage:\nqtbuild.bash [clean] \n" 
	} 
	
p=`pwd`

# Must have qt installed preferrably 5.9
if ! test -x /usr/bin/qmake
then
  # The Qt library is missing...
  echo "error: Qt must be installed for this build to work."
  echo "Installing qt 5.9.1 which works on Ubuntu 16.04"
  wget http://download.qt.io/official_releases/qt/5.9/5.9.1/qt-opensource-linux-x64-5.9.1.run
  chmod +x qt-opensource-linux-x64-5.9.1.run
  ./qt-opensource-linux-x64-5.9.1.run
fi


# Check if all the installation prerequisites are met.
bash ./isvalid.bash
valid=$?


# fail if installation prerequisites are not met.
if [ "$valid" != "0" ]
then
    echo "error: prerequisite missing"
    exit 1
fi

# Copy gzrcs headers to appropriate include subdirectory
cp -r $p/src/aprs_headers/include/aprs_headers $p/include

# FIXME: compile gazebo and ros messages to header files
# I found a convoluted way to dynamically generate header from msg but not worth it.
# So copy for now.
mkdir -p $p/include/crcl_rosmsgs/
cp $p/src/crcl/crcllib/include/crcllib/CrclCommandMsg.h $p/include/crcl_rosmsgs/
cp $p/src/crcl/crcllib/include/crcllib/CrclStatusMsg.h $p/include/crcl_rosmsgs/
cp $p/src/crcl/crcllib/include/crcllib/CrclMaxProfileMsg.h $p/include/crcl_rosmsgs/


# Aprs objects - vision simulator for aprs java framework
cd $p/src/aprs_objects
qmake aprs_objects.pro  -r -Wall  CONFIG+=debug
make
make install


# Gazebo plugin: gzjointcmdplugin
cd $p/src/gzplugins/gzjointcmdplugin
qmake gzjointcmdplugin.pro  -r -Wall  CONFIG+=debug
make
make install

# Gazebo plugin: gzjointcmdplugin
cd $p/src/gzplugins/gzmodelplugin
qmake gzmodelplugin.pro  -r -Wall  CONFIG+=debug
make
make install

# Gazebo plugin: gzjointcmdplugin 
cd $p/src/gzplugins/gzparallelgripperplugin
qmake gzparallelgripperplugin.pro  -r -Wall  CONFIG+=debug
make
make install


# Gomotion trajectory generator: gotraj
cd $p/src/gotraj
qmake gotraj.pro  -r -Wall  CONFIG+=debug
make
make install

# Gazebo Robot Control System: gzrcs
# Prerequisites: gotraj, aprs headers
cd $p/src/gzrcs
qmake gzrcs.pro  -r -Wall  CONFIG+=debug
make
make install


