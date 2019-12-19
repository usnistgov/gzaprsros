/**

@mainpage  Agility Performance Robot System Simulation

Agility Performance Robot System simulation based on C++, Gazebo, and ROS.

\copyright  This work was by employees of the U.S. Government as part of their official duties and are not protected by copyright in the U.S. (17 U.S.C. §105) and may be used without obtaining permission from NIST.

DISCLAIMER:

 This software was produced by the National Institute of Standards
 and Technology (NIST), an agency of the U.S. government, and by statute is
not subject to copyright in the United States.  Recipients of this software
 assume all responsibility associated with its operation, modification,
 maintenance, and subsequent redistribution.

 See NIST Administration Manual 4.09.07 b and Appendix I.
 
 
The GitHUB repository contains code that uses Gazebo to do kitting simulation. The simulation environment is based on the Agility Lab setup at NIST. There are 2 robots - a Fanuc LRMate and a Motoman sia200d being simulated that each provide a trajectory and kinematic module. In addition, the simulation provides acts as a server accepting  CRCL commands https://github.com/ros-industrial/crcl. 

The code uses gnu C++ 2011 standard in a Linux implementation, tested on Ubuntu 16.04 and 18.04. The 
simulation uses  Gazebo version 9   and  Ros I version Kinetic. The use of ROS is limited, but catkin_make is used to 
compile all the packages. There are several bash scripts under the bin9 folder that can be used to launch the  with rosagilitydemo.bash 
being the ROS catkin_make launch script. 


\section Requirements

Ubuntu 16.04 or 18.04. The following packages are installed by apt-get (assuming you have sudo privilidges).

ros kinetic
gazebo9
eigen3
xerces-c
code synthesis xsd
boost
assimp
readline
yad



\section Requirements

Navigate to the NIST github site, and clone the gzaprsros repository:


\section Build
\code
> git clone https://github.com/usnistgov/gzaprsros.git
\end code
To build gzaprsros run the bash script:
\code
> ./rosbuild.bash
\end code


*/



