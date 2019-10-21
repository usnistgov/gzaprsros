#!/bin/bash


p=`pwd`

# Must have qt installed preferrably 5.9

if ! test -x /usr/bin/qmake
then
  # The Qt library is missing...
  echo "error: Qt must be installed for this build to work."
  exit 1
fi

# fail if no eigen3
if ! test -d /usr/include/eigen3
then
    echo "error: Eigen3 must be installed for this build to work"
    exit 0.
fi

# fail if no ros kinetic
if ! test -d /opt/ros/kinetic
then
    echo "error: ROS kinetic must be installed for this build to work."
    exit 0
fi


# Test for libxerces and by assumption code synthesis
if ! test -f /usr/lib/x86_64-linux-gnu/libxerces-c.a
then
    echo "error: libxerces and code synthesis must be installed for this build to work."
    echo "sudo apt-get install libxerces-c3.1"
    exit 0
fi

# Test for boost installation
if ! test -f /usr/lib/x86_64-linux-gnu/libboost_system.so 
then
    echo "error: boost must be installed for this build to work."
    echo "sudo apt-get install libboost-all-dev"
    exit 0
fi



# fail if no assimp for mesh bounding box.
if ! test -d /usr/include/assimp
then
    echo "error: assimp must be installed for this build to work."
    echo "sudo apt-get install assimp-utils"
    exit 0
fi

# Copy gzrcs headers to appropriate include subdirectory

cp -r $p/src/aprs_headers/include/aprs_headers $p/include

# I found a convoluted way to dynamically generate header from msg but not worth it.
mkdir -p $p/include/crcl_rosmsgs/
cp $p/src/crcl/crcllib/include/crcllib/CrclCommandMsg.h $p/include/crcl_rosmsgs/
cp $p/src/crcl/crcllib/include/crcllib/CrclStatusMsg.h $p/include/crcl_rosmsgs/
cp $p/src/crcl/crcllib/include/crcllib/CrclMaxProfileMsg.h $p/include/crcl_rosmsgs/

# FIXME: compile gazebo and ros messages to header files


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


