#!/bin/bash

#    ^^ this has to be bash, not /bin/sh, for arrays to work
# run dos2unix ./runmultiterm.bash

pkill gzserver
pkill gzserver
pkill gzclient
pkill gzclient
pkill rosmaster

. /etc/lsb-release
if [ $DISTRIB_RELEASE == "18.04" ]
then
rosv="melodic"
elif [ $DISTRIB_RELEASE == "16.04" ]
then
rosv="kinetic"
fi
export rosv
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

gzrcsdemo=`pwd`/../install/lib/gzrcsdemo
export gzrcsdemo



#pushd .
#cd $gzrcs; find -type d -name "Log*" -prune; find -type d -name "Log*" -prune  -exec rm -rf {} \;
#popd

 
cmd=( gnome-terminal )

cmd+=( --tab  --working-directory="$q" -e 'bash -c "printf \"\e]2;ROS\a\";cd $aprs; source /opt/ros/$rosv/setup.bash;source devel/setup.bash;pwd; roslaunch gzrcs agilitylab.launch ;exec bash"')

#cmd+=( --tab  --working-directory="$q" -e 'bash -c "printf \"\e]2;vision telnet 5002\a\"; sleep 25s; telnet 127.0.0.1 5002 ;exec bash"')

cmd+=( --tab  --working-directory="$q" -e 'bash -c "printf \"\e]2;aprs_objects\a\"; sleep 15s; source /opt/ros/$rosv/setup.bash; cd $aprs_objects; ./aprs_objects  ;exec bash"')

cmd+=( --tab  --working-directory="$r" -e 'bash -c "printf \"\e]2;aprs gz motoman\a\";sleep 20s;export LD_LIBRARY_PATH=/opt/ros/$rosv/lib:$libs:$qtlibs:$LD_LIBRARY_PATH; echo $LD_LIBRARY_PATH; source /opt/ros/$rosv/setup.bash; cd $gzrcs; ./gzrcs -r motoman_;exec bash"')


cmd+=( --tab  --working-directory="$r" -e 'bash -c "printf \"\e]2;aprs gz fanuc\a\";sleep 20s;export LD_LIBRARY_PATH=/opt/ros/$rosv/lib:$libs:$qtlibs:$LD_LIBRARY_PATH; echo $LD_LIBRARY_PATH; source /opt/ros/$rosv/setup.bash; cd $gzrcs; ./gzrcs -r fanuc_;exec bash"')

cmd+=( --tab  --working-directory="$r" -e 'bash -c "printf \"\e]2;fanuc crcl demo\a\";sleep 20s;export LD_LIBRARY_PATH=/opt/ros/$rosv/lib:$libs:$qtlibs:$LD_LIBRARY_PATH; echo $LD_LIBRARY_PATH; source /opt/ros/$rosv/setup.bash; cd $gzrcsdemo; ./gzrcsdemo -r fanuc_;exec bash"')

cmd+=( --tab  --working-directory="$r" -e 'bash -c "printf \"\e]2;motoman crcl demo\a\";sleep 20s;export LD_LIBRARY_PATH=/opt/ros/$rosv/lib:$libs:$qtlibs:$LD_LIBRARY_PATH; echo $LD_LIBRARY_PATH; source /opt/ros/$rosv/setup.bash; cd $gzrcsdemo; ./gzrcsdemo -r motoman_;exec bash"')



"${cmd[@]}"




