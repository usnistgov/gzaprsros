cmake_minimum_required(VERSION 2.8.3)
project(gotraj)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  cmake_modules
  trajectory_msgs
  geometry_msgs
  sensor_msgs
  tf
  tf_conversions
  eigen_conversions
  )


find_package(Boost REQUIRED system thread chrono regex log_setup log)
find_package(Eigen REQUIRED)

catkin_package(
  INCLUDE_DIRS
  include
  LIBRARIES
  gotraj
  CATKIN_DEPENDS
  roscpp
  moveit_core
  sensor_msgs
  trajectory_msgs
  geometry_msgs
  tf
  tf_conversions
  eigen_conversions
  DEPENDS
  Boost
  Eigen

  )
include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
  # include breakdown for this project
  #    include/${PROJECT_NAME}
  #   include/${PROJECT_NAME}/CrclXsd
  include/${PROJECT_NAME}/NIST
  /usr/include/eigen3
  $ENV{HOME}/src/robot-agility/gz/installs/rcs_pkgs/include
  /home/isd/michalos/src/robot-agility/gz/installs/rcs_pkgs/include
  )

set(CMAKE_CXX_FLAGS "-w ${CMAKE_CXX_FLAGS}")
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")
set(CMAKE_CXX_FLAGS "-DDEBUG ${CMAKE_CXX_FLAGS}")
set(CMAKE_CXX_FLAGS "-DBOOST_LOG_DYN_LINK ${CMAKE_CXX_FLAGS}")
set(CMAKE_CXX_FLAGS "-fpermissive ${CMAKE_CXX_FLAGS}")
set(CMAKE_CXX_FLAGS "-g ${CMAKE_CXX_FLAGS}")

set(CMAKE_CXX_FLAGS "-DGO_REAL_DOUBLE ${CMAKE_CXX_FLAGS}")


set(CMAKE_EXE_LINKER_FLAGS "-rdynamic ${CMAKE_EXE_LINKER_FLAGS}")

set(GOTRAJ_LIB_SOURCES
  src/gointerp.cpp
  src/gomath.cpp
  src/gomotion.cpp
  src/gotraj.cpp
  src/gotraj_.cpp
  )    
add_library(gotraj ${GOTRAJ_LIB_SOURCES})

link_directories(/usr/lib/x86_64-linux-gnu/)


target_link_libraries(gotraj
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  )

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
  )

install(TARGETS gotraj
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})   

#install(FILES readme.md DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
