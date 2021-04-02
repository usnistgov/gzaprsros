#!/bin/bash
echo Hello world

#sudo /usr/bin/apt-get remove --assume-yes ros-*
#sudo /usr/bin/apt-get remove  --assume-yes gazebo-*

sudo /usr/bin/apt-get install xclip

# Code Synthesis
# https://charmie11.wordpress.com/2014/08/12/codesynthesis-xsd-installation-2/
sudo /usr/bin/apt-get install --assume-yes libxerces-c-dev
sudo /usr/bin/apt-get install --assume-yes xsdcxx

# gazebo9
sudo /usr/bin/apt-get install --assume-yes gazebo9
sudo /usr/bin/apt-get install --assume-yes libgazebo9-dev

# Python
sudo /usr/bin/apt-get install --assume-yes libboost-python-dev python python-dev python-numpy ipython
sudo /usr/bin/apt-get install --assume-yes python-dev
sudo /usr/bin/apt-get install --assume-yes python-pip

# ROS PACKAGES

declare -a arr=("kinetic" )

## loop through the above list
for i in "${arr[@]}"
do
   echo "Install $i"
   sudo /usr/bin/apt-get install --assume-yes ros-$i-desktop-full
   sudo /usr/bin/apt-get install --assume-yes ros-$i-moveit-full
   sudo /usr/bin/apt-get install --assume-yes ros-$i-python-orocos-kdl  ros-$i-orocos-kinematics-dynamics ros-$i-kdl-conversions ros-$i-orocos-kdl
   sudo /usr/bin/apt-get install --assume-yes ros-$i-rviz-visual-tools
	sudo /usr/bin/apt-get install --assume-yes  ros-$i-gazebo9-*


done
