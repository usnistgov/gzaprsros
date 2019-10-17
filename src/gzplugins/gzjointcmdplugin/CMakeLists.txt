cmake_minimum_required(VERSION 2.8.3)

SET(CMAKE_INSTALL_PREFIX "../plugins9") 

project(gzjointcmdplugin)

## Compile as C++11, supported in ROS Kinetic and newer
# Neither works
#add_compile_options(-std=c++11) # Doesnt work
#set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
    roscpp)

find_package(gazebo REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES fix_tray_objects
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

#set(CMAKE_CXX_FLAGS "-DGZJNTCMDPLUGIN_LIBRARY=1 ${CMAKE_CXX_FLAGS}")

include_directories(
  include ${catkin_INCLUDE_DIRS}${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

add_library(gzjointcmdplugin
    gzjntcommandplugin.cpp
    JointsComm.pb.cc
)
add_dependencies(gzjointcmdplugin ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(gzjointcmdplugin
  ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES}
)


install(TARGETS gzjointcmdplugin
 # ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CMAKE_INSTALL_PREFIX}
 # RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)                 

