#-------------------------------------------------
#
# Project created by QtCreator 2017-12-19T10:21:42
#
#-------------------------------------------------


QT       -= core  gui

message("Compiling gzjointcmdplugin ")
TARGET = gokin_plugin
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
# Boost DLL installation headers
INCLUDEPATH += $$PWD/../../install/include
INCLUDEPATH += $$PWD/../../gzrcs/include
INCLUDEPATH += $$PWD/../../aprs_headers/include
INCLUDEPATH += $$PWD/../../../include/dll/include

#ROS
INCLUDEPATH += "/opt/ros/kinetic/include"
LIBS += -L/opt/ros/kinetic/lib
LIBS += -lroscpp -lcpp_common -lroslib -lrosconsole -lrostime -lrospack
LIBS += -lurdf -lurdfdom_sensor -lurdfdom_model_state -lurdfdom_model -lurdfdom_world
LIBS +=  -ltf -ltf2 -lclass_loader
LIBS +=   -lxmlrpcpp -lroscpp_serialization
LIBS +=  -lrosconsole_bridge  -lrosconsole_log4cxx  -lrosconsole_backend_interface
LIBS += -lactionlib  -ltf2_ros


LIBS += -lboost_system
LIBS += -lboost_regex

SOURCES +=   src/genserkins.c \
    src/gokin.cpp\
    src/gomath.c\
    src/gotypes.c\
    src/inifile.c

HEADERS += Debug.h\
    include/gokin/genserkins.h\
    include/gokin/gokin.h\
    include/gokin/gomath.h\
    include/gokin/gotypes.h\
    include/gokin/inifile.h \
    include/gokin_plugin/Debug.h \
    include/gokin_plugin/genserkins.h \
    include/gokin_plugin/gokin.h \
    include/gokin_plugin/_gokin.h \
    include/gokin_plugin/gomath.h \
    include/gokin_plugin/goserkins.h \
    include/gokin_plugin/gotypes.h \
    include/gokin_plugin/inifile.h


config_features.path     = "../../include/$$TARGET"
config_features.files     = $$HEADERS

message("gokin_plugin installation to folder $$config_features.path")
message("mkspecs files $$config_features.files")
INSTALLS                  += config_features

target.path = ../../../lib
INSTALLS += target

