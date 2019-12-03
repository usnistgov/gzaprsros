#-------------------------------------------------
#
# Project created by QtCreator 2017-12-19T10:21:42
#
#-------------------------------------------------
# /home/isd/michalos/src/APRS/kdl/kdl_plugin.pro

QT       -= core  gui

message("Compiling orocos kdl with qt ")
TARGET = kdl_plugin
TEMPLATE = lib

CONFIG += c++11

gzversion=9


release: DESTDIR = release
debug:   DESTDIR = debug

OBJECTS_DIR = $$DESTDIR/.obj
MOC_DIR = $$DESTDIR/.moc
RCC_DIR = $$DESTDIR/.qrc
UI_DIR = $$DESTDIR/.ui

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


INCLUDEPATH += $$PWD/include
INCLUDEPATH += $$PWD/../../../include/dll/include
# Boost DLL installation headers
INCLUDEPATH += $$PWD/../../install/include
INCLUDEPATH += $$PWD/../../gzrcs/include
INCLUDEPATH += $$PWD/../../aprs_headers/include
INCLUDEPATH += "/usr/include/eigen3"

#ROS
INCLUDEPATH += "/opt/ros/kinetic/include"
LIBS += -L/opt/ros/kinetic/lib
LIBS += -lroscpp -lcpp_common -lroslib -lrosconsole -lrostime -lrospack
LIBS += -lurdf -lurdfdom_sensor -lurdfdom_model_state -lurdfdom_model -lurdfdom_world
LIBS +=  -ltf -ltf2 -lclass_loader
LIBS +=   -lxmlrpcpp -lroscpp_serialization
LIBS +=  -lrosconsole_bridge  -lrosconsole_log4cxx  -lrosconsole_backend_interface
LIBS += -lactionlib  -ltf2_ros
LIBS += -lorocos-kdl -lkdl_parser 

LIBS += -lboost_system
LIBS += -lboost_regex

SOURCES +=   src/kdl_plugin.cpp

HEADERS += Debug.h\
    include/kdl_plugin/kdl_plugin.h


config_features.path     = "../../include/$$TARGET"
config_features.files     = $$HEADERS

message("gokin_plugin installation to folder $$config_features.path")
message("mkspecs files $$config_features.files")
INSTALLS                  += config_features

target.path = ../../../lib
INSTALLS += target

