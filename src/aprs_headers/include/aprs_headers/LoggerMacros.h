// **************************************************************************

// LoggerMacros.h
//
// Description: macros to use and reuse logger class
//
// DISCLAIMER:
// This software was developed by U.S. Government employees as part of
// their official duties and is not subject to copyright. No warranty implied
// or intended.
// **************************************************************************


#include <aprs_headers/Logger.h>

// If already loaded macros undef and redefine with new Logging class
#ifdef logAbort
#undef logFatal
#undef logStatus
#undef logError
#undef logWarn
#undef logInform
#undef logDebug
#undef logTrace
#undef LOG_ONCE
#undef LOG_THROTTLE
#endif

#include "boost/iostreams/stream.hpp"
#include "boost/iostreams/device/null.hpp"
#include <boost/iostreams/stream.hpp>

#ifndef ONCE
#define ONCE(X)  \
{ boost::iostreams::stream< boost::iostreams::null_sink > nullOstream( ( boost::iostreams::null_sink() ) );\
    static std::ostream* os;\
    static int Y##__LINE__=-1;    if(++Y##__LINE__==0) os=&X;  else os=&nullOstream;}\
    *os
#endif

#ifndef GLOGGER
#define GLOGGER GLogger
extern Logging::CLogger GLogger;
#else
extern Logging::CLogger GLOGGER;
#endif

#warning "You will need to declare an instance of "##GLOGGER

#define logAbort(fmt, ...)                                            \
    GLOGGER.logMessage(__FILE__, __LINE__, Logging::LogLevel::_LOGFATAL_, fmt, \
    ## __VA_ARGS__);                             \
    ExitProcess(-1)

#define logFatal(fmt, ...)                                            \
    GLOGGER.logMessage(__FILE__, __LINE__, Logging::LogLevel::_LOGFATAL_, fmt, \
    ## __VA_ARGS__)


#define logStatus(fmt, ...)                                           \
    GLOGGER.logMessage(__FILE__, __LINE__, Logging::LogLevel::_LOGFATAL_, fmt, \
    ## __VA_ARGS__)

#define logError(fmt, ...)                                            \
    GLOGGER.logMessage(__FILE__, __LINE__, Logging::LogLevel::_LOGERROR_, fmt, \
    ## __VA_ARGS__)

#define logWarn(fmt, ...)                                            \
    GLOGGER.logMessage(__FILE__, __LINE__, Logging::LogLevel::_LOGWARN_, fmt, \
    ## __VA_ARGS__)

#define logInform(fmt, ...)                                          \
    GLOGGER.logMessage(__FILE__, __LINE__, Logging::LogLevel::_LOGINFO_, fmt, \
    ## __VA_ARGS__)


// logTrace & logDebug only used in debugging mode
#ifdef DEBUG
#define logTrace(fmt, ...)                                            \
    GLOGGER.logMessage(__FILE__, __LINE__, Logging::LogLevel::_LOGFATAL_, fmt, \
    ## __VA_ARGS__)
#define logDebug(fmt, ...)                                            \
    GLOGGER.logMessage(__FILE__, __LINE__, Logging::LogLevel::_LOGDEBUG_, fmt, \
    ## __VA_ARGS__)

#else
#define logTrace(fmt, ...)
#define logDebug(fmt, ...)
#endif

#define LOG_ONCE(X)                                   \
{                                                 \
    static bool __log_stream_once__hit__ = false; \
    if ( !__log_stream_once__hit__ ) {            \
    __log_stream_once__hit__ = true;          \
    X;                                        \
    }                                             \
    }
// Throttle logging has not been tested. 
// Especially now as seconds from epoch.
// seconds since 0,0,2000 


#define LOG_THROTTLE(secs, X)                                   \
{                                                 \
    static double last_hit = 0.0;                 \
    static double now = CLogger::getNow(); \
    if (last_hit + secs <= now)) {            \
    last_hit = now;                    \
    X;                                        \
    }                                             \
    }
