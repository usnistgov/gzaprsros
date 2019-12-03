cmake_minimum_required(VERSION 2.8.3)

SET(CMAKE_INSTALL_PREFIX "../lib") 

project(ikfast_fanuc_plugin)


## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
    roscpp)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES fix_tray_objects
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

#set(CMAKE_CXX_FLAGS "-DGZJNTCMDPLUGIN_LIBRARY=1 ${CMAKE_CXX_FLAGS}")

include_directories(
  include
 ./include/ikfast_fanuc_plugin
 ${catkin_INCLUDE_DIRS}
  ../../../include/dll/include
  ../../gz_custom_messages/include/gz_custom_messages
  ../../aprs_headers/include
 )


add_library(ikfast_fanuc_plugin
    src/ikfast_fanuc_plugin.cpp 
    src/fanuc_lrmate200id_manipulator_ikfast_solver.cpp
)

link_directories(${CATKIN_DEVEL_PREFIX}/install)
link_directories(${CATKIN_DEVEL_PREFIX}/devel/lib)



add_dependencies(ikfast_fanuc_plugin ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")
set(CMAKE_CXX_FLAGS "-Wno-deprecated-declarations ${CMAKE_CXX_FLAGS}")

target_link_libraries(ikfast_fanuc_plugin
  ${catkin_LIBRARIES} 
)
install(TARGETS ikfast_fanuc_plugin
 # ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CMAKE_INSTALL_PREFIX}
 # RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)                 

