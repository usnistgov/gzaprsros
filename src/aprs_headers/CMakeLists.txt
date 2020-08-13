cmake_minimum_required(VERSION 3.5.1)
project(aprs_headers)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  std_msgs
  tf
  tf_conversions
  std_msgs 
  sensor_msgs
  message_generation
  std_msgs 
  sensor_msgs
)

find_package(Boost REQUIRED COMPONENTS system thread)



include_directories(include ${catkin_INCLUDE_DIRS})

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system thread)

# This package needs an exe or so for dependencies work



add_message_files(
  FILES
  CrclCommandMsg.msg
  CrclMaxProfileMsg.msg
  CrclStatusMsg.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs  
  sensor_msgs 
)


  
###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
 include
 ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")
set(CMAKE_CXX_FLAGS "-Wno-deprecated-declarations ${CMAKE_CXX_FLAGS}")
set(CMAKE_CXX_FLAGS "-Wno-format-security ${CMAKE_CXX_FLAGS}")

# Debug flags to enable code
set(CMAKE_CXX_FLAGS "-DCNC ${CMAKE_CXX_FLAGS}")

file(MAKE_DIRECTORY ${aprs_headers_INCLUDE_DIRS}/${PROJECT_NAME})
file(GLOB DATATYPE_RAW ${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}/*)
message("DEBUG DATATYPE_RAW         :   ${DATATYPE_RAW}")

add_custom_target(${PROJECT_NAME} 
  ALL 
  COMMAND ${CMAKE_COMMAND} -E copy ${DATATYPE_RAW} ${aprs_headers_INCLUDE_DIRS}/${PROJECT_NAME}/
)
  
  
  
  
catkin_package(
  INCLUDE_DIRS
  include
#  LIBRARIES
#  gotraj
  CATKIN_DEPENDS
  message_runtime
  std_msgs
  sensor_msgs
  tf
  tf_conversions
  DEPENDS
  Boost
  )
  
#add_custom_command(OUTPUT Done.txt
#                   COMMAND ${CMAKE_COMMAND} -E make_directory  ${aprs_headers_INCLUDE_DIRS}/${PROJECT_NAME}/
#                   COMMAND ${CMAKE_COMMAND} -E copy
#                       ${CONFIG_SOURCES}
#                        ${aprs_headers_INCLUDE_DIRS}/${PROJECT_NAME}
#                   )             
  
#############
## Install ##
#############


install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_INCLUDE_DESTINATION})

## Mark cpp header files for installation
 install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.h"
   PATTERN ".svn" EXCLUDE
 )

  
                   
#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_gz_crcl_server.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
