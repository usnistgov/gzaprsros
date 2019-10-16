#!/bin/bash


# This bash script consolidates commands the help file from:
#  http://gazebosim.org/tutorials?tut=install_from_source&ver=4.0&cat=install


# Prerequisite versioning information
gazebo_major_version=9
export gazebo_major_version
ros_distro=kinetic
export ros_distro



#######################################
# remove the Ubuntu pre-compiled binaries before installing from source
#######################################
sudo apt-get remove --purge '*gazebo*' '*sdformat*' '*ignition-math*' '*ignition-msgs*' '*ignition-transport*'


# gazebo9
sudo /usr/bin/apt-get install --assume-yes gazebo$gazebo_major_version


# Install gazebo development?
sudo /usr/bin/apt-get install --assume-yes libgazebo$gazebo_major_version-dev

# Installing ros gazebo packages
sudo apt-get install ros-kinetic-gazebo9-*



