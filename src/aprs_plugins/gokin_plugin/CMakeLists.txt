cmake_minimum_required(VERSION 2.8.3)

SET(CMAKE_INSTALL_PREFIX "../lib") 

project(gokin_plugin)


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
set(CMAKE_CXX_FLAGS "-Wno-format-security ${CMAKE_CXX_FLAGS}")

include_directories(
 include
 ${catkin_INCLUDE_DIRS}
  ../../gz_custom_messages/include/gz_custom_messages
  ../../aprs_headers/include
  ../../../include/dll/include

  )


add_library(gokin_plugin
   src/genserkins.c 
    src/gokin.cpp
    src/gomath.c
    src/gotypes.c
    src/inifile.c
)

link_directories(${CATKIN_DEVEL_PREFIX}/install)
link_directories(${CATKIN_DEVEL_PREFIX}/devel/lib)



add_dependencies(gokin_plugin ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

target_link_libraries(gokin_plugin
  ${catkin_LIBRARIES} 
)

## Mark cpp header files for installation
 install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "gokin.h"
   PATTERN ".svn" EXCLUDE
 )

install(TARGETS gokin_plugin
 # ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CMAKE_INSTALL_PREFIX}
 # RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)                 

