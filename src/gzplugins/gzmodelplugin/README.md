# Model Pose Exporting

This package contains a gazebo plugin for publishing all the part models (have sku in their name) in the gazebo simulation.

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

The gzmodelplugin  plugin is a linked library used to publish the poses of all the models in the Gazebo simulation that contain sku as part of their name. Part of the published datais the bounding box of the model so that in testing gzrcs can determine the top offset in which to grab the object (not implemented but communicated). The object model data is published on the topic name ~/ariac/model as the message type: gazebo::msgs::Model.

This pose model information can also be used as a vision simulator to feed to an APRS framework controller.


### Gazebo World
The gazebo sdf world includes the gzmodelplugin as a model plugin for the world. 

## Description of the arguments:

    <update_rate> rate of updating the Gazebo model poses in milliseconds.
    <debug> 0 turns off debugging print statements, 1 turns on
   
### Example
	<world>
	. . .
	   <plugin filename="libgzmodelplugin.so" name="model_info">
		<debug> 0 </debug>
	   </plugin>
	</world>
