#!/bin/bash

#    ^^ this has to be bash, not /bin/sh, for arrays to work
# run dos2unix ./runmultiterm.bash

aprs=`pwd`/../Worlds/aprs-lab.world
gzver=9
export gzver
export aprs

p=`pwd`
q=`pwd`/launch
r=`pwd`/gzrcs

p=`pwd`
q=`pwd`/launch
r=`pwd`/gzrcs

# Definition of environment variables for bash
models=`pwd`/../gzdatabase/models
export models

plugins=`pwd`/../plugins$gzver
export plugins


libs=`pwd`/../devel/lib
export libs


gzrcs=`pwd`/../install/lib/gzrcs
export gzrcs

aprs_objects=`pwd`/../devel/lib/aprs_objects
export aprs_objects

# Kill any existing processes or can cause problems
pkill gzserver
pkill gzclient
pkill rosmaster

# Code to remove log files
#pushd .
#cd $gzrcs; find -type d -name "Log*" -prune; find -type d -name "Log*" -prune  -exec rm -rf {} \;
#popd

 
cmd=( gnome-terminal )

cmd+=( --tab  --working-directory="$q" -e 'bash -c "printf \"\e]2;ROS\a\";source /opt/ros/kinetic/setup.bash;export myfolder=`pwd`;roslaunch roscore.launch ;exec bash"')


cmd+=( --tab   --working-directory="$p"  -e 'bash -c "printf \"\e]2;gazebo\a\";source /opt/ros/kinetic/setup.bash; LD_LIBRARY_PATH=/opt/ros/kinetic/lib:/usr/lib/x86_64-linux-gnu/gazebo-$gzver/plugins:$LD_LIBRARY_PATH;export LD_LIBRARY_PATH ; GAZEBO_PLUGIN_PATH=$plugins:/opt/ros/kinetic/lib:$GAZEBO_PLUGIN_PATH; export GAZEBO_PLUGIN_PATH;GAZEBO_MODEL_PATH=$models;export GAZEBO_MODEL_PATH; cd `pwd`; gazebo $aprs --verbose ; exec bash"')

cmd+=( --tab  --working-directory="$q" -e 'bash -c "printf \"\e]2;vision telnet 5002\a\"; sleep 25s; telnet 127.0.0.1 5002 ;exec bash"')

cmd+=( --tab  --working-directory="$q" -e 'bash -c "printf \"\e]2;aprs_objects\a\"; sleep 15s; source /opt/ros/kinetic/setup.bash; cd $aprs_objects; ./aprs_objects  ;exec bash"')

cmd+=( --tab  --working-directory="$r" -e 'bash -c "printf \"\e]2;aprs gz motoman\a\";sleep 20s;export LD_LIBRARY_PATH=/opt/ros/kinetic/lib:$libs:$LD_LIBRARY_PATH; echo $LD_LIBRARY_PATH; source /opt/ros/kinetic/setup.bash; cd $gzrcs; ./gzrcs -r motoman_;exec bash"')


cmd+=( --tab  --working-directory="$r" -e 'bash -c "printf \"\e]2;aprs gz fanuc\a\";sleep 20s;export LD_LIBRARY_PATH=/opt/ros/kinetic/lib:$libs:$LD_LIBRARY_PATH; echo $LD_LIBRARY_PATH; source /opt/ros/kinetic/setup.bash; cd $gzrcs; ./gzrcs -r fanuc_;exec bash"')

"${cmd[@]}"

#trap 'zenity --error --title="Notice" --text="Close all";pkill gnome-terminal-server; pkill gzserver; pkill gzclient' 1 XIT



