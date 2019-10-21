#-------------------------------------------------
#
# Project created by QtCreator 2017-12-19T10:21:42
#
#-------------------------------------------------


QT       -= core  gui


TARGET = gzparallelgripperplugin
TEMPLATE = lib
messaget("Compiling gzparallelgripperplugin")

DEFINES += GZPARALLELGRIPPERPLUGIN_LIBRARY
CONFIG += c++11

gzversion=9


release: DESTDIR = release
debug:   DESTDIR = debug

CONFIG += skip_target_version_ext

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
INCLUDEPATH += "/usr/include/sdformat-4.2"
INCLUDEPATH += "/usr/include/gazebo-7"
LIBS += -lgazebo_math
}

INCLUDEPATH += "/usr/local/include/ignition"

LIBS += -lgazebo
LIBS += -lgazebo_common -lgazebo_transport -lgazebo_msgs
LIBS += -lboost_system
LIBS += -lboost_regex


SOURCES +=   gzparallelgripperplugin.cpp \
    GripCommand.pb.cc \
    JointsComm.pb.cc

HEADERS += gzParallelGripperPlugin_global.h \
    gzparallelgripperplugin.h \
    GripCommand.pb.h \
    JointsComm.pb.h


target.path = $$PWD/../../../plugins$$gzversion
target.path += ~/.gazebo/plugins
INSTALLS += target


