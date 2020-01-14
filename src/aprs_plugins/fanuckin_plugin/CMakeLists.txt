cmake_minimum_required(VERSION 2.8.3)

SET(CMAKE_INSTALL_PREFIX "../lib") 

project(fanuckin_plugin)


## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
    roscpp
    )

#LIST(APPEND CMAKE_MODULE_PATH "/usr/share/cmake-2.8/Modules/")
#find_package(Eigen REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES fix_tray_objects
#  CATKIN_DEPENDS other_catkin_pkg
 # DEPENDS Eigen
)

#set(CMAKE_CXX_FLAGS "-DGZJNTCMDPLUGIN_LIBRARY=1 ${CMAKE_CXX_FLAGS}")
set(CMAKE_CXX_FLAGS "-DIKFAST_NO_MAIN ${CMAKE_CXX_FLAGS}")
set(CMAKE_CXX_FLAGS "-DIKFAST_CLIBRARY ${CMAKE_CXX_FLAGS}")
set(CMAKE_CXX_FLAGS "-DIKFAST_NAMESPACE=FanucLRMate200d ${CMAKE_CXX_FLAGS}")


include_directories(
  include
 ./include/fanuckin_plugin
 ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
  ../../gz_custom_messages/include/gz_custom_messages
  ../../aprs_headers/include
  ../../../include/dll/include
  /usr/include/eigen3
 )


add_library(fanuckin_plugin
    src/fanuckin_plugin.cpp 
    src/fanuc_lrmate200id.cpp
)

link_directories(${CATKIN_DEVEL_PREFIX}/install)
link_directories(${CATKIN_DEVEL_PREFIX}/devel/lib)



add_dependencies(fanuckin_plugin ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")
set(CMAKE_CXX_FLAGS "-Wno-deprecated-declarations ${CMAKE_CXX_FLAGS}")

target_link_libraries(fanuckin_plugin
  ${catkin_LIBRARIES} 
)
install(TARGETS fanuckin_plugin
 # ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CMAKE_INSTALL_PREFIX}
 # RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)                 

