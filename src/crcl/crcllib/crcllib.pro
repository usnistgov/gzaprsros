
QT += core
QT -= gui

machine = woodsy
gzversion=9

TARGET = crcllib
CONFIG += console
CONFIG -= app_bundle
TEMPLATE = lib


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
#QMAKE_CXXFLAGS +=-DXSD_CXX11


contains(machine, woodsy){
message("Compiling for woodsy")

QMAKE_CXXFLAGS +=-std=c++11
#INCLUDEPATH += "$$(HOME)/src/robot-agility/gz/installs/rcs_pkgs/include"
INCLUDEPATH += "$$(HOME)/src/robot-agility/gz/installs/rcs_pkgs/include"

INCLUDEPATH += "$$(HOME)/src/robot-agility/gz/Crcl/crcllib/include/crcllib/CrclXsd"
INCLUDEPATH += "$$(HOME)/src/robot-agility/gz/Crcl/crcllib/include"
INCLUDEPATH += "$$(HOME)/src/robot-agility/gz/Crcl/crcllib/src"

#ROS
INCLUDEPATH += "/opt/ros/kinetic/include"
#INCLUDEPATH += "/usr/include"
#INCLUDEPATH += "/usr/include/eigen3"



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
LIBS += -L$$(HOME)/src/robot-agility/gz/installs/rcs_pkgs/lib
LIBS +=  -lgotraj

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

}


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
    src/crclserver.cpp

HEADERS += \
    include/crcllib/crcl.h \
    include/crcllib/CrclConfig.h \
    include/crcllib/CrclXsd/CRCLCommandInstance.hxx \
    include/crcllib/CrclXsd/CRCLCommands.hxx \
    include/crcllib/CrclXsd/CRCLProgramInstance.hxx \
    include/crcllib/CrclXsd/CRCLStatus.hxx \
    include/crcllib/CrclXsd/DataPrimitives.hxx \
    ../../installs/rcs_pkgs/include/aprs_headers/Core.h \
    ../../installs/rcs_pkgs/include/aprs_headers/Debug.h \
    ../../installs/rcs_pkgs/include/aprs_headers/RCSMsgQueue.h \
    ../../installs/rcs_pkgs/include/aprs_headers/Logger.h \
    include/crcllib/nistcrcl.h \
    ../../installs/rcs_pkgs/include/aprs_headers/Conversions.h \
    include/crcllib/Crcl2Rcs.h \
    ../../installs/rcs_pkgs/include/aprs_headers/RCSThreadTemplate.h \
    ../../installs/rcs_pkgs/include/aprs_headers/IRcs.h \
    include/crcllib/CrclServerInterface.h \
    include/crcllib/CrclClientInterface.h \
    include/crcllib/crclserver.h \
    include/crcllib/crclserver.h \
    ../../installs/rcs_pkgs/include/aprs_headers/RCSPriorityQueue.h \
    ../../installs/rcs_pkgs/include/aprs_headers/RCSMsgQueueThread.h \
    ../../installs/rcs_pkgs/include/aprs_headers/LoggerMacros.h

DISTFILES += \
    include/nistcrcl/CrclXsd/CRCLCommandInstance.xsd \
    include/nistcrcl/CrclXsd/CRCLCommands.xsd \
    include/nistcrcl/CrclXsd/CRCLProgramInstance.xsd \
    include/nistcrcl/CrclXsd/CRCLStatus.xsd \
    include/nistcrcl/CrclXsd/DataPrimitives.xsd \
    include/nistcrcl/CrclXsd/CreateTree.bash \
    include/nistcrcl/NIST/RCSTimer.txt \
    Notes

headers_features.path     = "~/src/robot-agility/gz/installs/rcs_pkgs/include/crcllib"
headers_features.files     = include/crcllib/nistcrcl.h
INSTALLS                  += headers_features

unix {
    target.path = ~/src/robot-agility/gz/installs/rcs_pkgs/lib
    INSTALLS += target
}


