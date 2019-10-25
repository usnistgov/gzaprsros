QT += core
QT -= gui

TARGET = gzrcs
CONFIG += console
CONFIG -= app_bundle
TEMPLATE = app
message("Compiling gzrcs")


# You have to add gzversion to Projects "Build Environment"
# for this to work in qmake
gzversion=9

message(GZ Version  $$gzversion)

# Version handling
versionpath="$$_PRO_FILE_PWD_/version.bash"
dummy=$$system($$versionpath )

versionnum=$$cat(version)
major=$$member(versionnum,0)
minor=$$member(versionnum,1)
build=$$member(versionnum,2)
message(major number $$major)
message(minor number $$minor)
message(build number $$build)

#CONFIG +=  c++11 gokin
CONFIG +=  c++11
release: DESTDIR = release
debug:   DESTDIR = debug

OBJECTS_DIR = $$DESTDIR/.obj
MOC_DIR = $$DESTDIR/.moc
RCC_DIR = $$DESTDIR/.qrc
#UI_DIR = $$DESTDIR/.ui

DEFINES+=QT_NO_VERSION_TAGGING
DEFINES+=CNC
DEFINES+=GZCONTACT
DEFINES+=GRIPPER
DEFINES+=ROS
DEFINES+=GAZEBO
DEFINES+=DEBUG
DEFINES+=KDL_CONVERSION
#DEFINES+=CRCL_ROS_TOPIC
DEFINES+=CRCL_DLL
DEFINES+=MOTOGOKIN
DEFINES+=MAJOR=$$major
DEFINES+=MINOR=$$minor
DEFINES+=BUILD=$$build

QMAKE_CXXFLAGS +=-std=c++11
QMAKE_CXXFLAGS +=-Wno-unused-variable
QMAKE_CXXFLAGS +=-Wno-sign-compare
QMAKE_CXXFLAGS +=-Wno-unused-parameter
QMAKE_CXXFLAGS +=-Wno-reorder
QMAKE_CXXFLAGS +=-Wno-format-extra-args
QMAKE_CXXFLAGS +=-Wno-unused-local-typedefs
QMAKE_CXXFLAGS +=-Wno-ignored-qualifiers
QMAKE_CXXFLAGS +=-Wno-deprecated-declarations
QMAKE_CXXFLAGS +=-Wno-unused-function
QMAKE_CXXFLAGS +=-Wno-unused-but-set-variable
QMAKE_CXXFLAGS +=-Wno-missing-field-initializers
QMAKE_CXXFLAGS +=-Wno-unused-parameter


#############################################################
# Gazebo
#############################################################
#LIBS += -L/usr/lib/x86_64-linux-gnu
LIBS += -lgazebo -lgazebo_client
LIBS += -lgazebo_common -lgazebo_transport -lgazebo_msgs
LIBS += -lprotobuf

contains(gzversion, 11){
message("Compiling for gazebo 9")
INCLUDEPATH += "/usr/local/include/ign-math/include"
INCLUDEPATH += "/usr/local/include/gazebo-11/gazebo/msgs"
INCLUDEPATH += "/usr/local/include/gazebo-11/gazebo/transport"
INCLUDEPATH += "/usr/include/sdformat-6.2"
INCLUDEPATH += "/usr/local/include/gazebo-11"
}


contains(gzversion, 9){
message("Compiling for gazebo 9")
INCLUDEPATH += "/usr/include/ignition/math4"
INCLUDEPATH += "/usr/include/ignition/msgs1"
INCLUDEPATH += "/usr/include/ignition/transport4"
INCLUDEPATH += "/usr/include/sdformat-6.2"
INCLUDEPATH += "/usr/include/gazebo-9"
}

INCLUDEPATH += "/usr/include/assimp"

contains(gzversion, 7){
message("Compiling for gazebo 7")
INCLUDEPATH += "/usr/include/ignition/math2"
INCLUDEPATH += "/usr/include/sdformat-4.0"
INCLUDEPATH += "/usr/include/gazebo-7"
INCLUDEPATH += "/usr/local/include/ignition"
LIBS += -lgazebo_math
}

QMAKE_CXXFLAGS +=-std=c++11
# Local includes
message(Current folder $$PWD)
INCLUDEPATH += $$PWD/../../include
INCLUDEPATH += $$PWD/include
INCLUDEPATH += $$PWD/src

# Ros installation headers
INCLUDEPATH += $$PWD/../../install/include

# Eigen - header only
INCLUDEPATH += "/usr/include/eigen3"


#ROS
INCLUDEPATH += "/opt/ros/kinetic/include"

# Be careful the order of the libraries is important
# Boost - much could be and is replaced by C11 std
LIBS += -lboost_system
LIBS += -lboost_chrono
LIBS += -lboost_thread
LIBS += -lboost_filesystem
LIBS += -lboost_date_time


LIBS += -lassimp


# Local ROS libs
LIBS += -L$$PWD/../../lib
LIBS +=  -lgotraj
LIBS +=  -lcrcllib

LIBS += -L$$PWD/../../install/lib

# Installed Ros libs
LIBS += -L/opt/ros/kinetic/lib
LIBS += -lroscpp -lcpp_common -lroslib -lrosconsole -lrostime -lrospack
LIBS += -lurdf -lurdfdom_sensor -lurdfdom_model_state -lurdfdom_model -lurdfdom_world
LIBS +=  -ltf -ltf2 -lclass_loader
LIBS +=   -lxmlrpcpp -lroscpp_serialization
LIBS +=  -lrosconsole_bridge  -lrosconsole_log4cxx  -lrosconsole_backend_interface
LIBS += -lactionlib  -ltf2_ros

# Not used.
gokin {
DEFINES+=GOKIN

# gomotion - installed from github into usr/local
INCLUDEPATH += "/usr/local/include"
# To use static lib best to use full path
LIBS += $$(HOME)/src/gomotion/lib/libgokin.a
LIBS += $$(HOME)/src/gomotion/lib/libgo.a
#LIBS += -lgokin -lgo
# OR
#LIBS += -lgofanuckin
}

SOURCES += \
    src/Controller.cpp \
    src/CrclApi.cpp \
    src/fanuclrmate200idkinematics.cpp \
    src/Globals.cpp \
    src/Kinematics.cpp \
    src/RobotControlException.cpp \
    src/motomansia20dkinematics.cpp \
    src/nist_robotsnc.cpp \
    src/RCSInterpreter.cpp \
    src/Shape.cpp \
    src/Demo.cpp \
    src/gazebo.cpp \
    src/commandlineinterface.cpp \
    src/cros.cpp \
    src/kinematicring.cpp \
    src/gripper.cpp \
    src/gokin/gokin.cpp \
    src/gokin/genserkins.c \
    src/gokin/gomath.c \
    src/gokin/gotypes.c \
    src/gokin/inifile.c \
    src/assimp.cpp \
    src/GripCommand.pb.cc \
    src/JointsComm.pb.cc

HEADERS += \
    include/gzrcs/Controller.h \
    include/gzrcs/CrclApi.h \
    include/gzrcs/Demo.h \
    include/gzrcs/Globals.h \
    include/gzrcs/Gripper.h \
    include/gzrcs/Kinematics.h \
    include/gzrcs/RobotControlException.h \
    include/gzrcs/RCSInterpreter.h \
    include/gzrcs/Shape.h \
    include/gzrcs/gazebo.h \
    include/gzrcs/cros.h \
    include/gzrcs/kinematicring.h \
     include/gzrcs/commandlineinterface.h \
    include/gzrcs/Communication.h \
    include/gzrcs/Controller.h \
    include/gzrcs/CrclApi.h \
    include/gzrcs/cros.h \
    include/gzrcs/Demo.h \
    include/gzrcs/gazebo.h \
    include/gzrcs/Globals.h \
    include/gzrcs/Gripper.h \
    include/gzrcs/kinematicring.h \
    include/gzrcs/Kinematics.h \
    include/gzrcs/MotionControl.h \
    include/gzrcs/MotionException.h \
    include/gzrcs/nist_robotsnc.h \
    include/gzrcs/RCSInterpreter.h \
    include/gzrcs/RobotControlException.h \
    include/gzrcs/Shape.h \
    ../../include/aprs_headers/Config.h \
    ../../include/aprs_headers/Conversions.h \
    ../../include/aprs_headers/Core.h \
    ../../include/aprs_headers/Debug.h \
    ../../include/aprs_headers/IRcs.h \
    ../../include/aprs_headers/Logger.h \
    ../../include/aprs_headers/LoggerMacros.h \
    ../../include/aprs_headers/RCSMsgQueue.h \
    ../../include/aprs_headers/RCSTimer.h \
    ../../include/aprs_headers/Config.h \
    ../../include/aprs_headers/Conversions.h \
    ../../include/aprs_headers/Core.h \
    ../../include/aprs_headers/Debug.h \
    ../../include/aprs_headers/File.h \
    ../../include/aprs_headers/IRcs.h \
    ../../include/aprs_headers/Logger.h \
    ../../include/aprs_headers/LoggerMacros.h \
    ../../include/aprs_headers/RCSMsgQueue.h \
    ../../include/aprs_headers/RCSPriorityQueue.h \
    ../../include/aprs_headers/RCSTimer.h \
    include/gzrcs/assimp.h \
    ../../include/aprs_headers/env.h \
    include/gzrcs/GripCommand.pb.h \
    include/gzrcs/JointsComm.pb.h
DISTFILES += \
    Notes.txt \
    Todo \
    SampleCommands \
    Readme.txt \
    Tests.txt

# Below goes to the build directory
#config_features.path     = "$$OUT_PWD/$$DESTDIR/config"

config_features.path     = ../../install/lib/$$TARGET/config
config_features.files     =    $$PWD/config/Config.ini \
   $$PWD/config/MotomanSia20d.urdf\
   $$PWD/config/FanucLRMate200iD.urdf\
   $$PWD/config/motoman_sia20d.ini

message("gzrcs install $$config_features.path")
message("gzrcs instal files $$config_features.files")
INSTALLS  += config_features

target.path = ../../install/lib/$$TARGET/
INSTALLS += target
