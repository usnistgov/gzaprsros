#!/bin/bash
# prerequisite:
# sudo apt-get install screen
 
p=`pwd`
 
cmd=( gnome-terminal )

cmd+=( --tab  --working-directory="$p" -e 'bash -c "printf \"\e]2;GZ Joint Status\a\";gz topic -e /gazebo/default/fanuc/jtscomm_status -d 1000 ;exec bash"')


"${cmd[@]}"
