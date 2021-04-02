cmake_minimum_required(VERSION 2.8.3)

# copy library to gazebo plugin area - assuming gazebo version 9
SET(CMAKE_INSTALL_PREFIX "../plugins9") 

project(gzparallelgripperplugin)


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

include_directories(
  include ${catkin_INCLUDE_DIRS}${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

add_library(gzparallelgripperplugin
    gzparallelgripperplugin.cpp
    JointsComm.pb.cc
    GripCommand.pb.cc
)
add_dependencies(gzparallelgripperplugin ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(gzparallelgripperplugin
  ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES}
)


install(TARGETS gzparallelgripperplugin
 # ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CMAKE_INSTALL_PREFIX}
 # RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)                 
