
QT += core
QT -= gui

gzversion=9

TARGET = crclclient
CONFIG += console
CONFIG -= app_bundle
TEMPLATE = lib
message("Compiling crclclient")


CONFIG +=  c++11
release: DESTDIR = release
debug:   DESTDIR = debug

OBJECTS_DIR = $$DESTDIR/.obj
MOC_DIR = $$DESTDIR/.moc
RCC_DIR = $$DESTDIR/.qrc
#UI_DIR = $$DESTDIR/.ui

DEFINES+=QT_NO_VERSION_TAGGING
DEFINES+=DEBUG

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
QMAKE_CXXFLAGS +=-Wno-missing-field-initializers
QMAKE_CXXFLAGS +=-std=c++11
#QMAKE_CXXFLAGS +=-fpermissive

#INCLUDEPATH += "../../../include"
INCLUDEPATH += "../../aprs_headers/include"
INCLUDEPATH += "../crcl_rosmsgs/include"

INCLUDEPATH += "./include/crclclient/CrclXsd"
INCLUDEPATH += "./include"
INCLUDEPATH += "./src"

#ROS
INCLUDEPATH += "/opt/ros/kinetic/include"

# Local  libs
LIBS += -L../../../lib
#LIBS +=  -lgotraj


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


# Ros libs
LIBS += -L/opt/ros/kinetic/lib -lroscpp -lcpp_common -lroslib -lrosconsole -lrostime -lrospack
LIBS += -lurdf -lurdfdom_sensor -lurdfdom_model_state -lurdfdom_model -lurdfdom_world
LIBS +=  -ltf -ltf2 -lclass_loader
LIBS +=   -lxmlrpcpp -lroscpp_serialization
LIBS +=  -lrosconsole_bridge  -lrosconsole_log4cxx  -lrosconsole_backend_interface
LIBS += -lactionlib -lpthread -ltf2_ros -ltf_conversions

# Code Synthesis
# xerces code synthesis dependency - no XML parsing
# Static lib
#LIBS += "/usr/lib/x86_64-linux-gnu/libxerces-c.a"

# xerces code synthesis dependency
#LIBS += "/usr/lib/x86_64-linux-gnu/libxerces-c.a"
LIBS +=  -lxerces-c


SOURCES += \
    src/crcl.cpp \
    src/nistcrcl.cpp \
    src/CRCL/CRCLCommandInstance.cxx \
    src/CRCL/CRCLCommands.cxx \
    src/CRCL/CRCLProgramInstance.cxx \
    src/CRCL/CRCLStatus.cxx \
    src/CRCL/DataPrimitives.cxx \
    src/Crcl2Rcs.cpp \
    src/CrclServerInterface.cpp \
    src/CrclClientInterface.cpp \
    src/Rcs2Crcl.cpp \
    src/CrclBufferHandler.cpp \
    src/CrclSocketClient.cpp \
    src/CrclSocketServer.cpp

HEADERS += \
    include/crclclient/crcl.h \
    include/crclclient/CrclXsd/CRCLCommandInstance.hxx \
    include/crclclient/CrclXsd/CRCLCommands.hxx \
    include/crclclient/CrclXsd/CRCLProgramInstance.hxx \
    include/crclclient/CrclXsd/CRCLStatus.hxx \
    include/crclclient/CrclXsd/DataPrimitives.hxx \
     include/crclclient/nistcrcl.h \
    include/crclclient/Crcl2Rcs.h \
    include/crclclient/CrclServerInterface.h \
    include/crclclient/CrclClientInterface.h \
    include/crclclient/Rcs2Crcl.h \
    ../../aprs_headers/include/aprs_headers/IRcs.h \
    ../../aprs_headers/include/aprs_headers/seriallinkrobot.h \
    include/crclclient/CrclBufferHandler.h \
    include/crclclient/CrclSocketClient.h \
    include/crclclient/CrclSocketServer.h \
    include/crclclient/CrclBufferHandler.h

DISTFILES += \
    include/nistcrcl/CrclXsd/CRCLCommandInstance.xsd \
    include/nistcrcl/CrclXsd/CRCLCommands.xsd \
    include/nistcrcl/CrclXsd/CRCLProgramInstance.xsd \
    include/nistcrcl/CrclXsd/CRCLStatus.xsd \
    include/nistcrcl/CrclXsd/DataPrimitives.xsd \
    include/nistcrcl/CrclXsd/CreateTree.bash \
    include/nistcrcl/NIST/RCSTimer.txt \
    Notes

headers_features.path     = "../../../include/crclclient"
headers_features.files     = include/crclclient/nistcrcl.h
INSTALLS                  += headers_features

target.path = ../../../lib
INSTALLS += target


