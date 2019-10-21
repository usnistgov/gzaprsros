#-------------------------------------------------
#
# Project created by QtCreator 2017-12-19T10:21:42
#
#-------------------------------------------------


QT       -= core  gui

messaget("Compiling gzjointcmdplugin ")
TARGET = gzjointcmdplugin
TEMPLATE = lib

DEFINES += GZJNTCMDPLUGIN_LIBRARY
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

contains(gzversion, 11){
message("Compiling for gazebo 11")
INCLUDEPATH += "/usr/include/ignition/math4"
INCLUDEPATH += "/usr/include/ignition/msgs1"
INCLUDEPATH += "/usr/include/ignition/transport4"
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

contains(gzversion, 7){
message("Compiling for gazebo 7")
INCLUDEPATH += "/usr/include/ignition/math2"
INCLUDEPATH += "/usr/include/sdformat-4.0"
INCLUDEPATH += "/usr/include/gazebo-7"
LIBS += -lgazebo_math
}

INCLUDEPATH += "/usr/local/include/ignition"

LIBS += -lgazebo
LIBS += -lgazebo_common -lgazebo_transport -lgazebo_msgs
LIBS += -lboost_system
LIBS += -lboost_regex

SOURCES +=   gzjntcommandplugin.cpp \
    JointsComm.pb.cc

HEADERS += gzjntcommandplugin_global.h \
    gzjntcommandplugin.h \
    gzjntcommandplugin_global.h \
    JointsComm.pb.h


target.path = $$PWD/../../../plugins$$gzversion
target.path += ~/.gazebo/plugins
INSTALLS += target


