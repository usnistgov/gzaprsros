# Gazebo Simulated Robot Joint Control

This package contains a Gazebo plugin for controlling robot joints in Gazebo simulation.

## Requirements

Gazebo 9 + SDK
For QT: Gazebo ignition math 4, ignition msgs1, ignition transport4 and sdformat-6.2 Or modify the qmake pro file.
Only tested with ROS kinetic and Ubuntu 16.04 LTS


## Build

To build, either 
a) compile top level with ROS "catkin_make install" and dlls will be copied to plugins9
b) invoke Qt 5.9.1 IDE and compile. 


## Contents

### Gazebo Plugins

The gzjointcommandpluging plugin is used to command and report the joints of a robot. It uses a Gazebo custom message modeled after the ROS sensor_msg joint state. So each command must supply a list of names, and either an one or more accompanying position, velocity or effort values. Of note, if you combine a name and effort, the the joint will attempt to do force control. 


### Gazebo World
The gazebo sdf world includes the gzparallelgripper as plugins for both the motoman sia20d and the fanuc 200id. The control and status topic names are different for each robot.

## Description of the arguments:

    <finger> multiple elements that contain the names of all the finger joints. These will be excluded from control. IF YOU DONT YOU WILL SEE SEVERE JITTERING WHEN COMBINED WITH THE gzparallelplugin gripper control.
    <cmdtopic> contains the gazebo command topic name
    <statustopic> contains the gazebo s topic name 
    <debug> 0 turns off debugging print statements, 1 turns on
   
### Example
Note the finger links are deduced from the joint names. Further you must fully instantiate the joint names. 
The joint names are fanuc_prism1 and fanuc_prism2, but are part of the fanuc_lrmated200id robot and part of the lrmate model..
So the fully qualified name for joint1 would be lrmate::fanuc_lrmate200id::fanuc_prism1 shown below:

            <plugin filename="libgzjointcmdplugin.so" name="fanuc_jnts">
                <topic>~/fanuc/joints</topic>
                    <finger>fanuc_lrmate200id::fanuc_prism1</finger>
                    <finger>fanuc_lrmate200id::fanuc_prism2</finger>
                <cmdtopic> ~/fanuc/robotcmd </cmdtopic>
                <statustopic> ~/fanuc/robotstatus </statustopic>
                <debug> 0 </debug>
            </plugin>

