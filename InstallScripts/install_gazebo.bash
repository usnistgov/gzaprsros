#!/bin/bash


# This bash script consolidates commands the help file from:
#  http://gazebosim.org/tutorials?tut=install_from_source&ver=4.0&cat=install


# Prerequisite versioning information
gazebo_major_version=9
export gazebo_major_version
ros_distro=melodic
export ros_distro



sudo apt install ros-kinetic-desktop-full
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
sudo apt update
sudo apt-get install ros-kinetic-desktop-full



#######################################
# remove the Ubuntu pre-compiled binaries before installing from source
#######################################
sudo apt-get remove --purge '*gazebo*' '*sdformat*' '*ignition-math*' '*ignition-msgs*' '*ignition-transport*'


# gazebo9
sudo /usr/bin/apt-get install --assume-yes gazebo$gazebo_major_version


# Install gazebo development?
sudo /usr/bin/apt-get install --assume-yes libgazebo$gazebo_major_version-dev

# Installing ros gazebo packages
sudo apt-get install ros-kinetic-desktop
sudo apt-get install ros-kinetic-gazebo9-*



