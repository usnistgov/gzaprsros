
# Gazebo World Setup
----



This readme gives a brief overview of how to configure the gzaprsros World SDF file for Gazebo. Various models of the APRS laboratory are available, with configurable selections for versions, robots, kitting models, flags and the like. Configuration of the world was developed to provide a variety of platforms with differing simulation performance.


Modelling in Gazebo is done with SDF. SDF is formalized XML to describe visualization, physics and control for objects in a simulated world. Objects in the simulated world include robots, sensors and static or dynamic objects. The ability to communicate with these objects is enabled with the Gazebo message transport. The ability to extend SDF models is allowed through plugins
# <a name="Configuration"></a>Configuration






To configure the Gazebo world, navigate to the ./World folder. You should see the following APRS lab world SDF definitions, as well as the configuration bash script world_setup.bash.

![Figure1](./images/gzworldsetup_image1.gif)



Using the script engine bash, yad/zenity, g++ preprocessor, a Perl script, and templates, the SDF world can be customized to meet various simulation needs.  This assumes yad is installed, which in xenial requires:
	sudo apt-get install yad





We will assume g++ is already installed. The g++ preprocessor is used for #define value definitions that are embedded in the template SDF files to delineate conditional inclusions.  The template folder contains parts of the world with embedded preprocessor conditionals to factor in or out certain configurations. Then, the Perl script removecomments.perl, which is part of the folder, is used to clear out the comments non-SDF/XML header left behind by the g++ preprocessor. All these components are combined into the selected aprs version  to produce the aprs-lab.world  file used by the gzaprsros application.


Double click on the world_setup.bash file or invoke the world_setup.bash  script from the command line.  Once invoked you will get a simple yad GUI as shown in the figure below to configure the aprs-lab.world  file used by the gzaprsros application.




![Figure2](./images/gzworldsetup_image2.gif)



The breakdown of the various configuration variable in shown in the table below.



<TABLE>
<TR>
<TD>Variable<BR></TD>
<TD>Description<BR></TD>
</TR>
<TR>
<TD>Physics Engine<BR></TD>
<TD>This selects the physics engine Gazebo will use. The default is ODE, but other options are available but through experimentation have been removed as they did not improve performance.<BR></TD>
</TR>
<TR>
<TD>Version<BR></TD>
<TD>There are 6 versions of the APRS lab to select from. Version 2 is a simple lightweight robot and kitting object model to lighten the simulation load on Gazebo. Version 6 can contain furniture, safety system, cameras, as part of the Gazebo world file.<BR></TD>
</TR>
<TR>
<TD>Debug<BR></TD>
<TD>The debug flag set to True will cause the Gazebo plugins to emit debugging messages.<BR></TD>
</TR>
<TR>
<TD>Motoman<BR></TD>
<TD>If True, the Motoman robot and assisted kitting models will be part of the world.<BR></TD>
</TR>
<TR>
<TD>Fanuc<BR></TD>
<TD>If True, the Fanuc robot and assisted kitting models will be part of the world.<BR></TD>
</TR>
<TR>
<TD>Safety<BR></TD>
<TD>If true, the safety curtain will be displayed.<BR></TD>
</TR>
<TR>
<TD>Camera<BR></TD>
<TD>If True, the Gazebo cameras are present in the model. You will need ROS plugins to communicate with cameras.<BR></TD>
</TR>
<TR>
<TD>Conveyor<BR></TD>
<TD>If True, the conveyor model will be included in the model. Note untested.<BR></TD>
</TR>
<TR>
<TD>Furniture<BR></TD>
<TD>If True, all the APRS lab furniture, PCs, monitors, etc. models will be included in the SDF world.<BR></TD>
</TR>
<TR>
<TD>ROS Plugins<BR></TD>
<TD>If true the gazebo-ros plugins for the conveyor, camera and safety system will be incorporated as part of the world SDF file.<BR></TD>
</TR>
<TR>
<TD>Gz Vision Sim<BR></TD>
<TD>If True, the vision simulator plugin will run which emits the Gazebo model for all kitting models on a given socket port (typically port 5001 or 5002 but configurable).<BR></TD>
</TR>
</TABLE>





One of the major considerations of the simulation fidelity is the processing load of the configured world. Many of the objects in the world are designated to be static, which reduces the physics calculations. Such designated static objects include the kits and trays, chairs, tables, and PCs. Static objects do not incur any physics so that a static tray elevated to a fixed position in space will stay at this position (making the requirement for an underlying table moot). However, the placement of a "live" gear in a static tray will produce physics to detect and report collisions between the static tray and the dynamic gear, which is not computationally free. Below is a screenshot of Gazebo simulation with Version 6 Agility lab and the safety system (which includes 86 laser ROS plugins) resulting in a real-time factor shown at the bottom to be 0.34 which is quite computationally intensive especially when you consider that "nothing is being done". So, an attempt was made to reduce the computational overhead by making objects such as trays and tables to be static to avoid the physics collision overhead.

![Figure3](./images/gzworldsetup_image3.gif)






The configuration to make this simulation visualization is: 
	Version=6, Debug=0, Motoman=1, Fanuc=1, Safety=1, Camera=1, Conveyor=1, Furniture=1, ROS Plugins=1


Note, in order to communicate with the cameras and the safety system, the ROS Plugins must be true to allow communication to these sensors.


A quick summary of the APRS label model versions is given. Version 2 only has the robots, gears and trays, and does not include furniture nor camera sensors nor safety system as part of the World.  Thus, no matter if the furniture flag is true, the furniture will not be present in the simulation world. Version 6 has the robots, gears and trays, as well as the furniture, cameras, and safety system are part of the World. However, with Version 6 you can remove individual aspects (for example the furniture) so that the configuration flags are interpreted. Of note, version 2 was used extensively in grasping experimentation as the Gazebo simulation real-time factor is close to 1.0 so that simulation physics evaluation should be more accurate.
# <a name="Plugins"></a>Plugins



The Gazebo SDF world allows "plugin(s)" to be embedded in a model to provide access, control, and status to a model(s). In fact, the same plugin can be used twice for different models in the world, but with different SDF/XML options so that the plugin acts differently. A plugin is code that is compiled as a shared library and inserted into the simulation as part of the SDF XML. The plugin has direct access to all the functionality of Gazebo through the standard C++ classes.
## <a name="gzjointcmdplugin"></a>gzjointcmdplugin



The gzjointcmdplugin plugin is used to control the joints of the robot model. It also reports the joint status values.  gzjointcmdplugin  is a custom  C++  gazebo plugin that uses a customized Gazebo/protobuf message.  The code for this plugin can be found at: src/gzplugins/gzjointcmdplugin. The files JointsComm.pb.{h,cc} were generated elsewhere using the Google protobuf compiler and included in this plugin (and the client) for intercommunication with this plugin.


Below is the use of the plugin libgzjointcmdplugin to control a Fanuc robot "model" joints.
	<plugin filename="libgzjointcmdplugin.so" name="fanuc_jnts">
	  <finger>fanuc_lrmate200id::fanuc_prism1</finger>
	  <finger>fanuc_lrmate200id::fanuc_prism2</finger>
	  <cmdtopic> ~/fanuc/robotcmd </cmdtopic>
	  <statustopic> ~/fanuc/robotstatus </statustopic>
	  <debug> 0 </debug>
	</plugin>


By changing the <cmdtopic> SDF/XML element, we can in fact use the same code to control the Motoman:
	<plugin filename="libgzjointcmdplugin.so" name="moto_jnts">
	  <cmdtopic> ~/motoman/robotcmd </cmdtopic>
	  <statustopic> ~/motoman/robotstatus </statustopic>
	  <finger>motoman_sia20d::motoman_right_finger_joint</finger>
	  <finger>motoman_sia20d::motoman_left_finger_joint</finger>
	  <debug> 0 </debug>
	</plugin>


There are some peculiarities to defining the a fully qualified joint name (e.g., a partial joint name is motoman_sia20d::motoman_right_finger_joint)  that will be omitted. So, the libgzjointcmdplugin has 4 options: cmdtopic, statustopic, finger (multiple elements allowed) and debug. The cmdtopic is the Gazebo topic that a client advertised on that this plugin listens to receive joint updates. The plugin itself reads the robot model it is embedded in at startup to gather all the joints it will be updating. The <finger> option tells the plugin to ignore the finger joint for updating as this is handled elsewhere (in our case the libgzparallelgripperplugin).   YOU MUST INCLUDE THESE FINGERS, as the  libgzjointcmdplugin will attempt to update the fingers at the same time as the libgzparallelgripperplugin which will lead to oscillation of the finger joints (between the commanded position of libgzparallelgripperplugin and zero as updated by libgzjointcmdplugin).
## <a name="gzparallelgripper"></a>gzparallelgripper 



The gzparallelgripper plugin is used to control the gripper joints of the robot model assuming the gripper is a disembodied parallel gripper (two fingers open/close).  The gzparallelgripper plugin monitors the pose error of a grasped object while the gripper is subject to various user-defined forces. The parallel gripper fingertips can be actuated using the GripCommand.pb.{h,cc} to accept (enabled/disabled) command which sends a close(true) or open(false) to actuate the gripper.  The plugin uses a simplified control scheme: a constant user-defined force is applied through both finger joints with a proportional corrective force used to maintain the symmetrical position of the gripper's fingertips. This scheme was used to better reflect the functionality of actual pneumatic grippers.


A gripper plugin SDF/XML example is shown below:
	<plugin name="ParallelGripperPlugin" filename="libgzparallelgripperplugin.so">
	  <grip_force_close>5</grip_force_close>
	  <joint1>motoman_sia20d::motoman_left_finger_joint</joint1>
	  <joint2>motoman_sia20d::motoman_right_finger_joint</joint2>
	  <grip_kp>10000</grip_kp>
	  <control_topic> ~/gripper/motoman_sia20d/control  </control_topic>
	  <state_topic> ~/gripper/motoman_sia20d/state  </state_topic>
	  <debug> 0 </debug>
	  <collisions> 1 </collisions>
	</plugin>


Where the <grip_force_close> defines the force to apply to fingers when closing. <joint1> and <joint2> specify the parallel finger joints Gazebo names. The <grip_kp> defines the proportional gain in the closing algorithm. The <control_topic> is a Gazebo topic name used to send customized Gazebo messages defining an enable/disable gripper command. The <state_topic> is a Gazebo topic name used to report a customized Gazebo messages defining the state of the parallel gripper. The <collisions> flag enables the plugin to detect collisions between the finger links and an object, and to create/destroy a virtual joint between  the object and the gripper joint when closing/opening the gripper.  



## <a name="gzmodelplugin"></a>gzmodelplugin 



The gzmodelplugin is a custom plugin that communicates the active poses of all the kitting  related objects in the simulation world. The algorithm to "detect" a kitting object is to see if the model name contains "sku".  The advertised communication message type is gazebo::msgs::Model  which is a predefined Gazebo message type. The communication topic is hard-coded as ~/ariac/model. Below is the <plugin> XML to include the publishing of the kitting model object pose.  



	<plugin filename="libgzmodelplugin.so" name="model_info">
	  <debug> 0 </debug>
	</plugin>





There are two primary purposes for the kitting model information: vision simulation and for testing.  In testing, the grasping of the top of a gear is an important element in transferring a gear from a tray to a kit. However, one system configuration variable that must be hard coded and adjusted according to the object is the grasping offset from the given object pose. Contained within the model plugin are a couple ways to return the bounding box of the object in the commmunication message from which it is assumed, the grasping offset can be determined. Since gears are the only objects being grasped in the kitting simulation, the only difference in grasping location is based on the size of the gear. However, the bounding box is provided for each kitting object, but is currently not used so the hard-coded z offset is still used.





The kitting model object pose information primary purpose is to be used by a vision simulator which reformats the information into the same format as the APRS laboratory vision system.  The folder src/aprs_objects contains the code to read the Gazebo communication and then reformat into the vision simulation format that is streamed as text over port 5001 or 5002 (configurable by ROS params at startup). If you use the agilitydemo bash scripts they launch a script to read the Gazebo model update and translate into the APRS agility lab vision reporting format. Below is a screen shot of a telnet session that connects to the vision simulator stream (port 5002):


    

![Figure4](./images/gzworldsetup_image4.gif)






There is a vision simulation of the APRS agility lab vision system in the folder src/aprs_objects that uses the Gazebo World model. The vision simulation code uses ULAPI, Gazebo and ROS to communicate, command, control a TCP stream of simulated vision data. The vision simulated format is continually streamed  to different ports depending on the configuration, but in general on port #5002 (Motoman robot camera) and port #5001 for (Fanuc robot camera). The vision simulation code generates a line per object pose detailing all the detected objects in its field of view. The vision report contains the object, type, confidence and xy position. There is no z position as the camera sensing is not adroit enough, so the vision simulation code discards the z value from the Gazebo model information.  Therein, the agility lab vision format is: 
	parttype,rotation, x,y, confidence%, metatype,





where:
	parttype= { sku_part_large_gear |sku_part_small_gear| sku_small_gear_vessel | sku_large_gear_vessel | sku_kit_s2l2_vessel}
	metatype = { P|PT| KT}
	rot = (0,360)
	confidence= (0,100)


for example:
	sku_part_large_gear,-0.15,716.51, 296.00,0.99,P,





Upon operation, the visual simulation code creates a ROS aprs_objects_node that is a Gazebo client while performing minimal ROS interaction – console logging output and parameter reading.  Below are the two simulated cameras circled in read (which can actually return images) with one camera analyzing the Fanuc LRMate kitting working volume and the other camera analyzing the Motoman sia200D kitting working volume.

![Figure5](./images/gzworldsetup_image5.gif)



Thus, aprs_objects_node is configurable using ROS params at ROS startup. You can see these parameters upon startup of the ROS master. Parameters "cam1"  and "cam2" define the TCP socket ports on which to stream the simulated vision data. Then, each camera needs an XY bounding box to  define its focus area for which it will "find" kitting objects. For camera1 these include: "cam1_x_min", "cam1_y_min",  "cam1_x_max" , and "cam1_y_max" . Likewise, camera2 has bounding box ROS params: "cam2_x_min", "cam2_y_min",  "cam2_x_max" , and "cam2_y_max". The values for these ROS params are displayed at startup of the ROS master:




![Figure6](./images/gzworldsetup_image6.gif)



So these camera ROS parameters bounding boxes are defined in a Gazebo world space which uses meters as units. After reading these values, the vision simulator will then filter objects based on pose location to produce simulated values for either camera1 or camera2 to stream to the appropriate TCP port.


These camera parameters are set in the launch file:
	<launch>
	<env name="GAZEBO_MEDIA_PATH"  value="$(find gzrcs)/../../gzdatabase/media" />   
	<env name="GAZEBO_PLUGIN_PATH"  value="$(find gzrcs)/../../plugins9" />   
	<env name="GAZEBO_MODEL_PATH"  value="$(find gzrcs)/../../gzdatabase/models" />   
	<env name="GAZEBO_RESOURCE_PATH"  value="$(find gzrcs)/../../Worlds" />   
		<!-- These params are for aprs_object vision simulator   -->
		<param name="debug" value="31" type="int" />
		<param name="cam1" value="5001" type="int" />
		<param name="cam2" value="5002" type="int" />
		<param name="cam1_x_min" value="0" type="double" />
		<param name="cam1_y_min" value="-1.5" type="double" />
		<param name="cam1_x_max" value="1" type="double" />
		<param name="cam1_y_max" value="-0.8" type="double" />
		<param name="cam2_x_min" value="-1" type="double" />
		<param name="cam2_y_min" value="-0.75" type="double" />
		<param name="cam2_x_max" value="1" type="double" />
		<param name="cam2_y_max" value="1" type="double" />
		
	  <include file="$(find gazebo_ros)/launch/empty_world.launch">
	    <arg name="world_name" value="aprs-lab.world"/> <!-- Note: the world_name is with respect to GAZEBO_RESOURCE_PATH environmental variable -->
	    <arg name="paused" value="false"/>
	    <arg name="use_sim_time" value="true"/>
	    <arg name="gui" value="true"/>
	    <arg name="recording" value="false"/>
	    <arg name="debug" default="0" />
	  </include>
	</launch>





