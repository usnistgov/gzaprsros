QT += core
QT -= gui

TARGET = gotraj
CONFIG += console
CONFIG -= app_bundle
TEMPLATE = lib
message("Compiling gotraj")


CONFIG +=  c++11
release: DESTDIR = release
debug:   DESTDIR = debug

OBJECTS_DIR = $$DESTDIR/.obj
MOC_DIR = $$DESTDIR/.moc
RCC_DIR = $$DESTDIR/.qrc
#UI_DIR = $$DESTDIR/.ui


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
QMAKE_CXXFLAGS +=-std=c++11

INCLUDEPATH += "./include"
INCLUDEPATH += "./src"
INCLUDEPATH += "/usr/include/eigen3"
INCLUDEPATH += "/opt/ros/kinetic/include"

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


config_features.path     = "../../include/$$TARGET"
config_features.files     = $$HEADERS

message("gotraj installation to folder $$config_features.path")
message("mkspecs files $$config_features.files")
INSTALLS                  += config_features

target.path = ../../lib
INSTALLS += target
