// **************************************************************************

// Logger.h
//
// Description:
//
// DISCLAIMER:
// This software was developed by U.S. Government employees as part of
// their official duties and is not subject to copyright. No warranty implied
// or intended.
// **************************************************************************
#pragma once
#include <fstream>
#include <iostream>
#include <stdarg.h>
#include <string>
#include <time.h>       /* time_t, struct tm, difftime, time, mktime */
#include <sys/time.h>
// Based on ROS Console
// https://github.com/ros/console_bridge/blob/master/include/console_bridge/console.h
// Interesting C++ Logging filtering macros here:
// https://github.com/ros/ros_comm/blob/4383f8fad9550836137077ed1a7120e5d3e745de/tools/rosconsole/include/ros/console.h

namespace Logging
{
/** \brief The set of priorities for message logging */
struct LogLevel
{
    const static int _LOGDEBUG_ = 4;
    const static int _LOGINFO_  = 3;
    const static int _LOGWARN_  = 2;
    const static int _LOGERROR_ = 1;
    const static int _LOGNONE_  = 0;
    const static int _LOGFATAL_ = -1;
};
struct CLogger
{
    //static std::string        ExeDirectory ;

    enum TimeFormat {
        HUM_READ,
        GMT,
        GMT_UV_SEC,
        LOCAL
    };
    /*!
          * \brief GetTimeStamp returns a timestamp string depending on the input format.
          * \param  format is one of an enumeration describing how to format timestamp.
          * \return a formated timestamp string.
          */
    static std::string strTimestamp(TimeFormat format = GMT_UV_SEC)
    {
        char            timeBuffer[50];
        struct tm *     timeinfo;
        struct timeval  tv;
        struct timezone tz;

        gettimeofday(&tv, &tz);
        timeinfo = ( format == LOCAL ) ? localtime(&tv.tv_sec) : gmtime(&tv.tv_sec);

        switch ( format )
        {
        case HUM_READ:
        {
            strftime(timeBuffer, 50, "%a, %d %b %Y %H:%M:%S %Z", timeinfo);
        }
            break;

        case GMT:
        {
            strftime(timeBuffer, 50, "%Y-%m-%dT%H:%M:%SZ", timeinfo);
        }
            break;

        case GMT_UV_SEC:
        {
            strftime(timeBuffer, 50, "%Y-%m-%dT%H:%M:%S", timeinfo);
        }
            break;

        case LOCAL:
        {
            strftime(timeBuffer, 50, "%Y-%m-%dT%H:%M:%S%z", timeinfo);
        }
            break;
        }

        //    if ( format == GMT_UV_SEC )
        //    {
        //        sprintf(timeBuffer + strlen(timeBuffer), ".%06dZ", tv.tv_usec);
        //    }

        return std::string(timeBuffer)+" ";

    }

    CLogger( )
    {
        debugLevel( ) = 7;
        isOutputConsole( ) = 0;
        isTimestamping( )  = 0;
        _sDebugString     = "";
        _nCounter        = 1;
    }

    ~CLogger( )
    {
        if ( ofsDebugFile.is_open( ) )
        {
            ofsDebugFile.close( );
        }
    }

    static std::string & loggerFolder()
    {
        static std::string loggerDirectory;
        return loggerDirectory;
    }

    static double getNow()
    {

        time_t timer;
        struct tm y2k = {0};

        y2k.tm_hour = 0;   y2k.tm_min = 0; y2k.tm_sec = 0;
        y2k.tm_year = 100; y2k.tm_mon = 0; y2k.tm_mday = 1;

        time(&timer);  /* get current time; same as: timer = time(NULL)  */

        return difftime(timer,mktime(&y2k)); // in seconds

    }
    int &              debugLevel ( )
    {
        return _debuglevel;
    }
    bool &             isTimestamping ( ) { return _bTimestamp; }
    std::string &      DebugString ( ) { return _sDebugString; }
    int &              isOutputConsole ( ) { return _nOutputConsole; }
    std::ofstream & operator () (void) { return this->ofsDebugFile; }
    void               close ( ) { ofsDebugFile.close( ); }
    void               open (std::string filename, int bAppend = false)
    {
        this->_filename = filename;
        open(bAppend);
    }

    void               open (int bAppend = 0)
    {
        std::ios_base::openmode opMode = std::ofstream::out;

        if ( bAppend )
        {
            opMode |= std::ofstream::app;
        }

        ofsDebugFile.open(_filename.c_str( ), opMode); // , OF_SHARE_DENY_NONE);
    }

    void               message (std::string msg)
    {
        if ( isOutputConsole( ) )
        {
#ifdef _WINDOWS
            OutputDebugString(msg.c_str( ));
#else
            std::cout << msg.c_str();
#endif
        }

        if ( !ofsDebugFile.is_open( ) )
        {
            return;
        }

        if ( isTimestamping( ) )
        {
            ofsDebugFile << strTimestamp( );
        }
        ofsDebugFile << msg;
        ofsDebugFile.flush( );
    }

    void               logMessage (const char *file,
                                   int line,
                                   int level,
                                   const char *fmt,
                                   ...)
    {
        if ( level > debugLevel( ) )
        {
            return;
        }

        va_list ap;
        va_start(ap, fmt);

        int         m;
        int         n = strlen(fmt) + 1028;
        std::string tmp(n, '0');

        // Kind of a bogus way to insure that we don't
        // exceed the limit of our buffer
        while ( ( m = vsnprintf(&tmp[0], n - 1, fmt, ap) ) < 0 )
        {
            n = n + 1028;
            tmp.resize(n, '0');
        }

        va_end(ap);
        tmp = tmp.substr(0, m);

        if ( isOutputConsole( ) )
        {
#ifdef _WINDOWS
            OutputDebugString(tmp.c_str( ));
#else
            std::cout << tmp.c_str();
#endif
        }

        if ( !ofsDebugFile.is_open( ) )
        {
            return;
        }

        if ( isTimestamping( ) )
        {
            ofsDebugFile << strTimestamp( );
        }
        ofsDebugFile << tmp;
        ofsDebugFile.flush( );
    }

    std::ofstream      ofsDebugFile;
protected:
    int                _debuglevel;
    bool               _bTimestamp;
    std::string        _sDebugString;
    int                _nOutputConsole;
    int                _nDebugReset;
    std::string        _filename;
    int                _nCounter;
};
}

