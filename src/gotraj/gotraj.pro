QT += core
QT -= gui

TARGET = gotraj
CONFIG += console
CONFIG -= app_bundle
TEMPLATE = lib

machine = woodsy

CONFIG +=  c++11
release: DESTDIR = release
debug:   DESTDIR = debug

OBJECTS_DIR = $$DESTDIR/.obj
MOC_DIR = $$DESTDIR/.moc
RCC_DIR = $$DESTDIR/.qrc
#UI_DIR = $$DESTDIR/.ui

#QMAKE_CXXFLAGS +=-DQt

#QMAKE_CXXFLAGS +=-DCHECKERS

QMAKE_CXXFLAGS +=-DXSD_CXX11
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
QMAKE_CXXFLAGS +=-Wno-write-strings
QMAKE_CXXFLAGS +=-DDEBUG
#QMAKE_CXXFLAGS +=-DROBOTIQ



contains(machine, woodsy){
message("Compiling for woodsy")

QMAKE_CXXFLAGS +=-std=c++11
#INCLUDEPATH += "$$(HOME)/src/robot-agility/gz/installs/ros_pkgs/include"
INCLUDEPATH += "/opt/ros/kinetic/include"

INCLUDEPATH += "$$(HOME)/src/robot-agility/gz/Client/gotraj/include"
INCLUDEPATH += "$$(HOME)/src/robot-agility/gz/Client/gotraj/src"
INCLUDEPATH += "/usr/include/eigen3"

# Quaternion rpy constructor :(

#ROS
#INCLUDEPATH += "/opt/ros/kinetic/include"
#INCLUDEPATH += "/usr/include"
#INCLUDEPATH += "/usr/include/eigen3"

# xerces code synthesis dependency
#LIBS += "/usr/lib/x86_64-linux-gnu/libxerces-c.a"


# Boost - many could be replace by C11 std
LIBS += -L/usr/lib/x86_64-linux-gnu
LIBS += -lboost_system
LIBS += -lboost_chrono
LIBS += -lboost_thread
LIBS += -lboost_filesystem
LIBS += -lboost_date_time
LIBS += -lboost_regex
LIBS += -lboost_log_setup
LIBS += -lboost_log
LIBS += -lboost_locale

# Local ROS libs
LIBS += -L$$(HOME)/src/robot-agility/gz/installs/ros_pkgs/lib
LIBS +=  -lgotraj

# Ros libs
LIBS += -L/opt/ros/kinetic/lib -lroscpp -lcpp_common -lroslib -lrosconsole -lrostime -lrospack
LIBS += -lurdf -lurdfdom_sensor -lurdfdom_model_state -lurdfdom_model -lurdfdom_world
LIBS +=  -ltf -ltf2 -lclass_loader
LIBS +=   -lxmlrpcpp -lroscpp_serialization
LIBS +=  -lrosconsole_bridge  -lrosconsole_log4cxx  -lrosconsole_backend_interface
LIBS += -lactionlib -lpthread -ltf2_ros -ltf_conversions

# Gazebo
LIBS += -L/usr/lib/x86_64-linux-gnu -lgazebo -lgazebo_client
LIBS += -lgazebo_common -lgazebo_transport -lgazebo_msgs -lgazebo_math
LIBS += -lprotobuf -lignition-math2


}


contains(machine, linuxros){
message("Compiling for linuxros")
INCLUDEPATH += "/usr/local/michalos/nistfanuc_ws/src/nist_robotsnc/include"
INCLUDEPATH += "/usr/local/michalos/nistfanuc_ws/src/nist_robotsnc/include/nist_robotsnc"
#INCLUDEPATH += "/usr/local/michalos/nistcrcl_ws/devel/include"
INCLUDEPATH += "/usr/local/michalos/nistfanuc_ws/devel/include"
INCLUDEPATH += "/usr/local/michalos/nistfanuc_ws/src/nistcrcl/include"
INCLUDEPATH += "/usr/local/michalos/nistfanuc_ws/src/gotraj/include"
#INCLUDEPATH += "/usr/local/michalos/nistfanuc_ws/src/gokin/include"
#INCLUDEPATH += "/usr/include/freetype2"

INCLUDEPATH += "/usr/include"

#ROS
INCLUDEPATH += "/opt/ros/kinetic/include"
INCLUDEPATH += "/usr/include"
INCLUDEPATH += "/usr/include/eigen3"

# Libxml2
LIBS += -L/usr/lib -lxml2
# xerces code synthesis dependency
LIBS += "/usr/lib/x86_64-linux-gnu/libxerces-c.a"

# Boost - many could be replace by C11 std
LIBS += -L/usr/lib/x86_64-linux-gnu
LIBS += -lboost_system
LIBS += -lboost_chrono
LIBS += -lboost_thread
LIBS += -lboost_filesystem
LIBS += -lboost_date_time
LIBS += -lboost_regex
LIBS += -lboost_log_setup
LIBS += -lboost_log
LIBS += -lboost_locale

# Local ROS libs
LIBS += -L/usr/local/michalos/nistfanuc_ws/devel/lib
LIBS += -lgokin -lgotraj

# Ros libs
LIBS += -L/opt/ros/kinetic/lib -lroscpp -lcpp_common -lroslib -lrosconsole -lrostime -lrospack
LIBS +=  -ltf -ltf2 -lclass_loader
LIBS +=   -lxmlrpcpp -lroscpp_serialization
LIBS +=  -lrosconsole_bridge  -lrosconsole_log4cxx  -lrosconsole_backend_interface
LIBS += -lurdf -lurdfdom_sensor -lurdfdom_model_state -lurdfdom_model -lurdfdom_world
LIBS += -lactionlib -lpthread -ltf2_ros -ltf_conversions

QMAKE_CXXFLAGS +=-DROS
QMAKE_CXXFLAGS +=-DROS=1

}

SOURCES += \
    src/gointerp.cpp \
    src/gomath.cpp \
    src/gomotion.cpp \
    src/gotraj.cpp \
    src/gotraj_.cpp

HEADERS += \
    include/gotraj/gointerp.h \
    include/gotraj/gomath.h \
    include/gotraj/gomotion.h \
    include/gotraj/gotraj.h \
    include/gotraj/gotraj_.h \
    include/gotraj/gotypes.h

DISTFILES += \
    Notes


config_features.path     = "~/src/robot-agility/src/installs/ros_pkgs/include/$$TARGET"
config_features.files     = $$HEADERS

message("mkspecs $$config_features.path")
message("mkspecs files $$config_features.files")
INSTALLS                  += config_features




unix {
    target.path = ~/src/robot-agility/gz/installs/ros_pkgs/lib
    INSTALLS += target
}
