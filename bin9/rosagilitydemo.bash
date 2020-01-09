#!/bin/bash

#    ^^ this has to be bash, not /bin/sh, for arrays to work
# run dos2unix ./runmultiterm.bash

pkill gzserver
pkill gzserver
pkill gzclient
pkill gzclient
pkill rosmaster


libs=`pwd`/../devel/lib
export libs

qtlibs=`pwd`/../lib
export qtlibs

aprs=`pwd`/..
export aprs

aprs_objects=`pwd`/../devel/lib/aprs_objects
export aprs_objects

gzrcs=`pwd`/../install/lib/gzrcs
export gzrcs




#pushd .
#cd $gzrcs; find -type d -name "Log*" -prune; find -type d -name "Log*" -prune  -exec rm -rf {} \;
#popd

 
cmd=( gnome-terminal )

cmd+=( --tab  --working-directory="$q" -e 'bash -c "printf \"\e]2;ROS\a\";cd $aprs; source /opt/ros/kinetic/setup.bash;source devel/setup.bash;pwd; roslaunch gzrcs agilitylab.launch ;exec bash"')

cmd+=( --tab  --working-directory="$q" -e 'bash -c "printf \"\e]2;vision telnet 5002\a\"; sleep 25s; telnet 127.0.0.1 5002 ;exec bash"')

cmd+=( --tab  --working-directory="$q" -e 'bash -c "printf \"\e]2;aprs_objects\a\"; sleep 15s; source /opt/ros/kinetic/setup.bash; cd $aprs_objects; ./aprs_objects  ;exec bash"')

cmd+=( --tab  --working-directory="$r" -e 'bash -c "printf \"\e]2;aprs gz motoman\a\";sleep 20s;export LD_LIBRARY_PATH=/opt/ros/kinetic/lib:$libs:$qtlibs:$LD_LIBRARY_PATH; echo $LD_LIBRARY_PATH; source /opt/ros/kinetic/setup.bash; cd $gzrcs; ./gzrcs -r motoman_;exec bash"')


cmd+=( --tab  --working-directory="$r" -e 'bash -c "printf \"\e]2;aprs gz fanuc\a\";sleep 20s;export LD_LIBRARY_PATH=/opt/ros/kinetic/lib:$libs:$qtlibs:$LD_LIBRARY_PATH; echo $LD_LIBRARY_PATH; source /opt/ros/kinetic/setup.bash; cd $gzrcs; ./gzrcs -r fanuc_works_;exec bash"')



"${cmd[@]}"




