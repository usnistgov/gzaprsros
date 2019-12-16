
QT -= gui core

CONFIG += c++11

TARGET = testgokin_fanuc
CONFIG += console
CONFIG -= app_bundle

TEMPLATE = app

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


SOURCES += main.cpp

# The following define makes your compiler emit warnings if you use
# any feature of Qt which as been marked deprecated (the exact warnings
# depend on your compiler). Please consult the documentation of the
# deprecated API in order to know how to port your code away from it.
DEFINES += QT_DEPRECATED_WARNINGS

# You can also make your code fail to compile if you use deprecated APIs.
# In order to do so, uncomment the following line.
# You can also select to disable deprecated APIs only up to a certain version of Qt.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0



INCLUDEPATH += $$PWD/../include
INCLUDEPATH += $$PWD/../include/gokin_fanuc_plugin
# Boost DLL installation headers
INCLUDEPATH += $$PWD/../../../gzrcs/include
INCLUDEPATH += $$PWD/../../../aprs_headers/include
INCLUDEPATH += $$PWD/../../../../include/dll/include

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


LIBS += -L/home/isd/michalos/build/gokin_fanuc_plugin/debug
LIBS += -lgokin_fanuc_plugin


config_features.path     = "../../include/$$TARGET"
config_features.files     = $$HEADERS

message("ikfast_fanuc_plugin installation to folder $$config_features.path")
message("mkspecs files $$config_features.files")
INSTALLS                  += config_features

target.path = ../../../lib
INSTALLS += target

