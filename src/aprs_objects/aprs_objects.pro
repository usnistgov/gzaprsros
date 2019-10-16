QT += core
QT -= gui

gzversion=9
# THIS SHOULD WORK
gz = $$(gzversion)
message(Compiling for $$gz)

CONFIG +=  c++11
release: DESTDIR = release
debug:   DESTDIR = debug

OBJECTS_DIR = $$DESTDIR/.obj
MOC_DIR = $$DESTDIR/.moc
RCC_DIR = $$DESTDIR/.qrc
#UI_DIR = $$DESTDIR/.ui

TARGET = aprs_objects
CONFIG += console
CONFIG -= app_bundle

TEMPLATE = app
QMAKE_CXXFLAGS +=-std=c++11
QMAKE_CXXFLAGS +=-Wno-unused-parameter
QMAKE_CXXFLAGS +=-Wno-unused-variable
QMAKE_CXXFLAGS +=-Wno-unused-function
QMAKE_CXXFLAGS +=-Wno-unused-but-set-variable
QMAKE_CXXFLAGS +=-Wno-multichar


# Gazebo

contains(gzversion, 9){
message("Compiling for gazebo 9")
INCLUDEPATH += "/usr/include/ignition/math4"
INCLUDEPATH += "/usr/include/ignition/msgs1"
INCLUDEPATH += "/usr/include/ignition/transport4"
INCLUDEPATH += "/usr/include/sdformat-6.1"
INCLUDEPATH += "/usr/include/gazebo-9"
}

contains(gzversion, 7){
message("Compiling for gazebo 7")
INCLUDEPATH += "/usr/include/ignition/math2"
INCLUDEPATH += "/usr/include/sdformat-4.4"
INCLUDEPATH += "/usr/include/gazebo-7"
INCLUDEPATH += "/usr/local/include/ignition"
LIBS += -lgazebo_math
}

LIBS += -lgazebo -lgazebo_client
LIBS += -lgazebo_common -lgazebo_transport -lgazebo_msgs
LIBS += -lprotobuf

#Ulapi
LIBS += -ldl

#ROS
INCLUDEPATH += "/opt/ros/kinetic/include"

LIBS += -L/opt/ros/kinetic/lib
LIBS += -lroscpp -lcpp_common -lroslib -lrosconsole -lrostime -lrospack
LIBS += -lurdf -lurdfdom_sensor -lurdfdom_model_state -lurdfdom_model -lurdfdom_world
LIBS +=  -ltf -ltf2 -lclass_loader
LIBS +=   -lxmlrpcpp -lroscpp_serialization
LIBS +=  -lrosconsole_bridge  -lrosconsole_log4cxx  -lrosconsole_backend_interface
LIBS += -lactionlib  -ltf2_ros -ltf_conversions


# Boost
LIBS += -lboost_system



SOURCES += main.cpp \
    aprs_objects_node.cpp \
    unix_ulapi.c \
    gazebo.cpp \
    xybidist.cpp

# The following define makes your compiler emit warnings if you use
# any feature of Qt which as been marked deprecated (the exact warnings
# depend on your compiler). Please consult the documentation of the
# deprecated API in order to know how to port your code away from it.
DEFINES += QT_DEPRECATED_WARNINGS

# You can also make your code fail to compile if you use deprecated APIs.
# In order to do so, uncomment the following line.
# You can also select to disable deprecated APIs only up to a certain version of Qt.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0

HEADERS += \
    aprs_objects.h \
    ulapi.h \
    gazebo.h

DISTFILES += \
    Notes
