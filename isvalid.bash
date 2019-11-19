#!/bin/bash
p=`pwd`

source /opt/ros/melodic/setup.bash


# fail if no ros melodic
if ! test -d /opt/ros/melodic
then
    echo "error: ROS kinetic must be installed for this build to work."
    echo "Remedy: sudo apt-get install ros-melodic-desktop-full"
    echo "It is not recommeded to install two versions of ROS on the same machine - e.g., melodic and lunar"
    exit 1
fi

# Check gazebo existence and version number 
if which gazebo >/dev/null
then
	v=`gazebo -v`
	num=`echo $v | sed "s/^.*version \([0-9.]*\).*/\1/"`
	major=${num:0:1}
	minor=${num:1:1}
	if [ $major != "9" ]
	then
	    echo "error: Gazebo 9 must be installed for this build to work."
	    echo "Use: sudo apt-get install *gazebo*"
	    echo "It is not recommended to have two versions of gazebo installed"
	    echo "Most likely: check where build directory is, and run 'make uninstall'"
	    exit 1
	fi
else
	echo "Gazebo needs to be installed - continue (Y/N)?"
	read confirm
	declare -u  confirm
	if [ "$confirm" == "Y" ]
	then	
		# gazebo9
		sudo /usr/bin/apt-get install --assume-yes gazebo9
		# Install gazebo development
		sudo /usr/bin/apt-get install --assume-yes libgazebo9-dev
	fi

fi

# check that another version of gazebo is not installed WARN IF SO
dup=`ldconfig -p | grep -E libgazebo.*\.so\.[0-9]+ | grep -v -E libgazebo.*\.so\.9`
if [ "$dup" != "" ]
then
	echo "warning: APPEARS AS IF MULTIPLE Gazebo ARE INSTALLED."
	echo $dup
fi

# Test for eigen3
PKG_OK=$(dpkg-query -W --showformat='${Status}\n' gazebo9|grep "install ok installed")
if [ "" == "$PKG_OK" ]; then
  echo "error: gazebo9 must be installed for this build to work."
  echo "Installing gazebo9"
  sudo apt-get --force-yes --yes install gazebo9*
fi



# Test for eigen3
PKG_OK=$(dpkg-query -W --showformat='${Status}\n' libeigen3-dev|grep "install ok installed")
if [ "" == "$PKG_OK" ]; then
  echo "error: eigen3 must be installed for this build to work."
  echo "Installing libeigen3-dev"
  sudo apt-get --force-yes --yes install libeigen3-dev
fi

# Test for code synthesis
PKG_OK=$(dpkg-query -W --showformat='${Status}\n' xsdcxx|grep "install ok installed")
if [ "" == "$PKG_OK" ]; then
  echo "warning: codesynthesis must be installed for XML development to work."
  echo "Installing xsdcxx"
  sudo apt-get --force-yes --yes install xsdcxx
fi

# Test for libxerces  
PKG_OK=$(dpkg-query -W --showformat='${Status}\n' libxerces-c-dev|grep "install ok installed")
if [ "" == "$PKG_OK" ]; then
  echo "error: libxerces-c3.1 must be installed for this build to work."
  echo "Installing libxerces-c3.1"
  sudo apt-get --force-yes --yes install libxerces-c-dev
fi

# Test for boost if not installed install
PKG_OK=$(dpkg-query -W --showformat='${Status}\n' libboost-all-dev|grep "install ok installed")
if [ "" == "$PKG_OK" ]; then
  echo "error: boost must be installed for this build to work."
  echo "Installing libboost-all-dev."
  sudo apt-get --force-yes --yes install libboost-all-dev
fi

PKG_OK=$(dpkg-query -W --showformat='${Status}\n' assimp-utils|grep "install ok installed")
if [ "" == "$PKG_OK" ]; then
  echo "error: assimp must be installed for this build to work."
  echo "Installing assimp-utils."
  sudo apt-get --force-yes --yes install assimp-utils
fi

PKG_OK=$(dpkg-query -W --showformat='${Status}\n' yad|grep "install ok installed")
if [ "" == "$PKG_OK" ]; then
  echo "No yad. Installing yad."
  sudo apt-get --force-yes --yes install yad
fi


# Copy gzrcs headers to appropriate include subdirectory
mkdir -p $p/include/aprs_headers/
cp -r $p/src/aprs_headers/include/aprs_headers $p/include

# FIXME: compile gazebo and ros messages to header files
# I found a convoluted way to dynamically generate header from msg but not worth it.
# So copy for now.
mkdir -p $p/include/crcl_rosmsgs/
cp $p/src/crcl/crcllib/include/crcllib/CrclCommandMsg.h $p/include/crcl_rosmsgs/
cp $p/src/crcl/crcllib/include/crcllib/CrclStatusMsg.h $p/include/crcl_rosmsgs/
cp $p/src/crcl/crcllib/include/crcllib/CrclMaxProfileMsg.h $p/include/crcl_rosmsgs/


exit 0


