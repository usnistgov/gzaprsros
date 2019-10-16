#!/bin/bash

p=`pwd`


ans=$(yad --geometry=400x100 --title "World Configuration" --form --text="Choose options" \
 --field="Physics Engine:CB" "ODE"\!"BULLET"\!"DART"\!"SimBody"\
 --field="Version:CB" "2"\!"3"\!"4"\!"5"\!"6"\!"7"\
 --field="DEBUG:CB" "NO"\!"YES" \
 --field="Robots:CB" "Yes"\!"No" \
 --field="Motoman:CB" "Yes"\!"No" \
 --field="Fanuc:CB" "Yes"\!"No" \
 --field="Safety:CB" "Yes"\!"No" \
 --field="Camera:CB" "Yes"\!"No" \
 --field="Conveyor:CB" "Yes"\!"No"\
 --field="Furniture:CB" "Yes"\!"No"\
 --field="ROS Plugins:CB" "No"\!"Yes" \
 --field="GZ Vision Sim:CB" "Yes"\!"No" \
 )
 
echo ans = $ans


if [ -z "$ans" ]
then
    notify-send "Configuration setup cancelled"
#    zenity --info --text="Configuration setup cancelled" 
    exit
fi

OIFS=$IFS
IFS='|'  
read -r PHYSICS VERSION DEBUG ROBOTS MOTOMAN FANUC  SAFETY CAMERA CONVEYOR FURNITURE ROS GZVISIONSIM  <<< "$ans" # read field values and assign to variables 
#MULTILINE=$(echo -e $MULTILINE)                   # Convert '\n' in TXT field back into linefeeds 
echo -e "$PHYSICS=$VERSION\nDEBUG=$DEBUG\n$GRASP=$(echo $GRASP)\n$SAFETY=$(echo $SAFETY)\n$CAMERA=$(echo $CAMERA)\n$CONVEYOR\n$ROS\n$FURNITURE\n$PAVEL"
IFS=$OIFS

debug=$(echo $DEBUG)
physics=$(echo $PHYSICS)
version=$(echo $VERSION)
robots=$(echo $ROBOTS | grep "Yes" | wc -l)
motoman=$(echo $MOTOMAN | grep "Yes" | wc -l)
fanuc=$(echo $FANUC | grep "Yes" | wc -l)
#kinematic=$(echo $KINEMATIC | grep "Yes" | wc -l)
#grasp=$(echo $GRASP | grep "Yes" | wc -l)
safety=$(echo $SAFETY | grep "Yes" | wc -l)
camera=$(echo $CAMERA | grep "Yes" | wc -l)
conveyor=$(echo $CONVEYOR | grep "Yes" | wc -l)
furniture=$(echo $FURNITURE | grep "Yes" | wc -l)
#handles=$(echo $HANDLES | grep "Yes" | wc -l)
ros=$(echo $ROS | grep "Yes" | wc -l)
gzvisionsim=$(echo $GZVISIONSIM | grep "Yes" | wc -l)
#pavel=$(echo $PAVEL | grep "Yes" | wc -l)
#basic=$(echo $BASIC| grep "Yes" | wc -l)

# Determine preprocessor defines
D="-D$PHYSICS -DCONTACT "

if [ "$gzvisionsim" = 1 ] ; then
D="$D -DGZVISIONSIM"
fi
if [ "$debug" = 1 ] ; then
D="$D -DDEBUG"
fi
if [ "$robots" = 1 ] ; then
D="$D -DROBOTS"
fi
if [ "$motoman" = 1 ] ; then
D="$D -DMOTOMAN"
fi

if [ "$fanuc" = 1 ] ; then
D="$D -DFANUC"
fi
if [ "$grasp" = 1 ] ; then
D="$D -DGRASP_HACK"
fi

if [ "$safety" = 1 ] ; then
D="$D -DSAFETY"
fi

if [ "$camera" = 1 ] ; then
D="$D -DCAMERAS"
fi

if [ "$conveyor" = 1 ] ; then
D="$D -DCONVEYOR"
fi

if [ "$ros" = 1 ] ; then
D="$D -DROS_PLUGIN"
fi

if [ "$furniture" = 1 ] ; then
D="$D -DFURNITURE"
fi

#if [ "$pavel" = 1 ] ; then
D="$D  -DPARALLEL_GRIPPER_PLUGIN "
#fi

#if [ "$basic" = 1 ] ; then
D="$D -DPAVEL -DSIMPLE"
#fi

# Assume all gears for now
# -E option, nothing is done except preprocessing.
# -P  Inhibit generation of linemarkers in the output from the preprocessor. This might be useful when running the preprocessor on something that is not C code, and will be sent to a program which might be confused by the linemarkers. 
# -C Do not discard comments.

world=`pwd`
model=`pwd`/../../models
export world
export model


# This will create a template based on above parameterization. 
#gcc -E $D -P -C  - < $model/motoman_aprs/Templates/motoman_aprs_template.sdf |  $p/removecomments.perl | sed '/^$/d'   >  $model/motoman_aprs/motoman_aprs.sdf


echo "vision sim"
gcc -E $D -P -C  - < $world/Templates/gzvisionsim_template.world |  $p/removecomments.perl | sed '/^$/d'   >  $world/gzvisionsim.world
echo "Fanuc gears"
gcc -E $D  -P -C -I$world/Templates - < $world/Templates/fanuc_gears.world |  $p/removecomments.perl | sed '/^$/d'   >  $world/fanuc_gears.world
echo "Motoman gears"
gcc -E $D  -P -C -I$world/Templates - < $world/Templates/motoman_gears.world |  $p/removecomments.perl | sed '/^$/d'   >  $world/motoman_gears.world
echo "GZ camera"
gcc -E $D -P -C  - < $world/Templates/camera_template.world |  $p/removecomments.perl | sed '/^$/d'   >  $world/camera.world
echo "GZ Safety"
gcc -E $D -P -C  - < $world/Templates/safety_system_template.world |  $p/removecomments.perl | sed '/^$/d'   >  $world/safety_system.world
echo "Robots setup"
gcc -E $D -P -C  - < $world/Templates/robots_template.world |  $p/removecomments.perl | sed '/^$/d'   >  $world/robots.world
echo "Version setup"
gcc -E $D -P -C  - < $world/aprs-lab-V${version}.world |  $p/removecomments.perl | sed '/^$/d'   >  $world/aprs-lab.world


######################################################
# Handle the gear templates based on the user configuration

gcc -E $D -P -C  - < $model/gear_support/sku_part_large_gear/sku_part_large_gear_template.sdf |  $p/removecomments.perl | sed '/^$/d'   >  $model/gear_support/sku_part_large_gear/sku_part_large_gear.sdf 

gcc -E $D -P -C  - < $model/gear_support/sku_part_medium_gear/sku_part_medium_gear_template.sdf |  $p/removecomments.perl | sed '/^$/d'   >  $model/gear_support/sku_part_medium_gear/sku_part_medium_gear.sdf

gcc -E $D -P -C  - < $model/gear_support/sku_part_small_gear/sku_part_small_gear_template.sdf |  $p/removecomments.perl | sed '/^$/d'   >  $model/gear_support/sku_part_small_gear/sku_part_small_gear.sdf






