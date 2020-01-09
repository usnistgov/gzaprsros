// Globals.cpp

/*
 * DISCLAIMER:
 * This software was produced by the National Institute of Standards
 * and Technology (NIST), an agency of the U.S. government, and by statute is
 * not subject to copyright in the United States.  Recipients of this software
 * assume all responsibility associated with its operation, modification,
 * maintenance, and subsequent redistribution.
 *
 * See NIST Administration Manual 4.09.07 b and Appendix I.
 */
//#pragma message "Compiling " __FILE__ 

#include "gzrcs/Globals.h"
#include <map>
#include <iostream>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <dirent.h>
#include <sys/types.h>
#include <sys/stat.h>

#include "aprs_headers/File.h"


CGlobals Globals;
Logger   ofsRobotURDF;
Logger   ofsMotionTrace;
Logger   STATUS_LOG;
Logger   ofsRobotCrcl;
Logger   ofsGnuPlotCart;
Logger   ofsGnuPlotJnt;



#include <signal.h>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>

// Static declarations
bool CGlobals::bRunning=true;
bool CGlobals::bPaused=false;

////////////////////////////////////////////////////////////////////////////////
static std::string  LeftTrim (std::string  str, std::string trim = " \t\r\n")
{
    size_t startpos = str.find_first_not_of(trim);

    if ( std::string::npos != startpos )
    {
        str = str.substr(startpos);
    }
    return str;
}
////////////////////////////////////////////////////////////////////////////////
static std::string  RightTrim (std::string  str, std::string trim = " \t\r\n")
{
    size_t endpos = str.find_last_not_of(trim);

    if ( std::string::npos != endpos )
    {
        str = str.substr(0, endpos + 1);
    }
    return str;
}


////////////////////////////////////////////////////////////////////////////////
void my_handler(int s)
{
    std::cout << "Caught ^C \n"<< std::flush;
    CGlobals::bRunning=false;
}

////////////////////////////////////////////////////////////////////////////////
struct sigaction sigIntHandler;

void CGlobals::catchControlC()
{
    sigIntHandler.sa_handler = my_handler;
    sigemptyset(&sigIntHandler.sa_mask);
    sigIntHandler.sa_flags = 0;

    sigaction(SIGINT, &sigIntHandler, NULL);
}


////////////////////////////////////////////////////////////////////////////////
CGlobals::CGlobals()
{
    srand((unsigned int) time(NULL)); //activates the simple randome number generator

    bHandleExceptions=false;
    bReadAllInstances=false;
    bCannedDemo=false;
    bWorldCRCLCoordinates=false;
    bGearLocations=0;
    bGripperSpeed=0;
    bRepeatCannedDemo=0;

    // Global debugging files
    DEBUG_World_Command()=0; // Log controller action loop for robot servo of world cartesian move
    DEBUG_Log_Gripper_Status()=0;
    DEBUG_GnuPlot()=0;
    DEBUG_Log_Cyclic_Robot_Position()=0;
}

////////////////////////////////////////////////////////////////////////////////
CGlobals::~CGlobals() {
    // Doesn't matter if never opened

    ofsMotionTrace.close();
    STATUS_LOG.close();
    ofsRobotCrcl.close();
    ofsGnuPlotCart.close();
    ofsGnuPlotJnt.close();

}
////////////////////////////////////////////////////////////////////////////////
bool CGlobals::ok()
{
    return bRunning;
}

////////////////////////////////////////////////////////////////////////////////
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <dirent.h>
#include <sys/types.h>
pid_t CGlobals::procFind(const char* name)
{
    DIR* dir;
    struct dirent* ent;
    char* endptr;
    char buf[512];

    if (!(dir = opendir("/proc"))) {
        perror("can't open /proc");
        return -1;
    }

    while((ent = readdir(dir)) != NULL) {
        /* if endptr is not a null character, the directory is not
         * entirely numeric, so ignore it */
        long lpid = strtol(ent->d_name, &endptr, 10);
        if (*endptr != '\0') {
            continue;
        }

        /* try to open the cmdline file */
        snprintf(buf, sizeof(buf), "/proc/%ld/cmdline", lpid);
        FILE* fp = fopen(buf, "r");

        if (fp) {
            if (fgets(buf, sizeof(buf), fp) != NULL) {
                /* check the first token in the file, the program name */
                std::string first = strtok(buf, " ");
                if(first.find("/")!=std::string::npos)
                    first=first.substr(first.find_last_of('/') + 1);

                if (!strcmp(first.c_str(), name)) {
                    fclose(fp);
                    closedir(dir);
                    return (pid_t)lpid;
                }
            }
            fclose(fp);
        }

    }

    closedir(dir);
    return -1;
}

////////////////////////////////////////////////////////////////////////////////
std::string CGlobals::strFormat(const char *fmt, ...)
{
    va_list argptr;
    va_start(argptr, fmt);
    int m;
    int n = (int) strlen(fmt) + 1028;
    std::string tmp(n, '0');
    while ((m = vsnprintf(&tmp[0], n - 1, fmt, argptr)) < 0) {
        n = n + 1028;
        tmp.resize(n, '0');
    }
    va_end(argptr);
    return tmp.substr(0, m);
}

////////////////////////////////////////////////////////////////////////////////
void CGlobals::sleep(unsigned int ms) {
    std::this_thread::sleep_for(std::chrono::milliseconds(ms)) ;
}

////////////////////////////////////////////////////////////////////////////////
std::vector<std::string> CGlobals::split(const std::string &text, char sep, bool bKeepEmpty) {
    std::vector<std::string> tokens;
    std::size_t start = 0, end = 0;
    bKeepEmpty; // unused for now;
    std::string token;
    while ((end = text.find(sep, start)) != std::string::npos) {
        token = text.substr(start, end - start);
//        if (!token.empty())
            tokens.push_back(token);
        start = end + 1;
    }
    token = trim(text.substr(start));
//    if (!token.empty())
        tokens.push_back(token);
    return tokens;
}

////////////////////////////////////////////////////////////////////////////////
void CGlobals::logfilesSetup()
{
    std::string robotname = Globals.appProperties["robot"];
    robotname = RightTrim(robotname,"_");
    logfolder() = getexefolder()+"Log_"+ robotname+"-"+getTimeStamp(LOGFILE)+"/";
    File lf(logfolder());
    lf.mkpath(logfolder().c_str(), 0777);
//    GLogger.loggerFolder()=logfolder();
}

////////////////////////////////////////////////////////////////////////////////
void CGlobals::debugSetup()
{
    // Setup folder for logging files
    logfilesSetup();

    STATUS_LOG.enable()=1;
    STATUS_LOG.open(Globals.logfolder()+"/" + Globals.appProperties["robot"] + ".log");

//    LOG_FATAL.copyfmt(GLogger.ofsDebugFile);
//    LOG_FATAL.clear(GLogger.ofsDebugFile.rdstate()); //2
//    LOG_FATAL.basic_ios<char>::rdbuf(GLogger.ofsDebugFile.rdbuf());           //3

    ofsRobotCrcl.open(logfolder()+Globals.appProperties["robot"]+"_nc_crcl.log");

    if(DEBUG_GnuPlot())
    {
        ofsGnuPlotCart.open(logfolder()+"CartXYZPlot.log");
        ofsGnuPlotJnt.open(logfolder()+"JntPlot.log");
    }

    ofsMotionTrace.open(logfolder() + "MotionTrace.log");

}
////////////////////////////////////////////////////////////////////////////////
void CGlobals::assignOfs(std::ostream *inOfs, std::ostream *replacementOfs)
{

    (*inOfs).copyfmt((*replacementOfs));
    (*inOfs).clear((*replacementOfs).rdstate()); //2
    (*inOfs).basic_ios<char>::rdbuf((*replacementOfs).rdbuf());           //3

}

////////////////////////////////////////////////////////////////////////////////
std::string  CGlobals::trim (std::string  s)
{
    return LeftTrim(RightTrim(s) );
}
////////////////////////////////////////////////////////////////////////////////
bool CGlobals::readFile (std::string filename, std::string & contents)
{
    std::ifstream     in(filename.c_str( ), std::ifstream::in );
    std::stringstream buffer;

    if(!in.is_open())
    {
        STATUS_LOG << "CGlobals::ReadFile failed file does not exist" << filename << "\n" << std::flush;

    }
    buffer << in.rdbuf( );
    contents = buffer.str( );
    return true;
}
////////////////////////////////////////////////////////////////////////////////
void CGlobals::writeFile (std::string filename, std::string & contents)
{
    std::ofstream outFile(filename.c_str( ) );

    outFile << contents.c_str( );
}
////////////////////////////////////////////////////////////////////////////////
void CGlobals::appendFile (std::string filename, std::string  contents)
{
    std::ofstream outFile;

    outFile.open(filename.c_str( ), std::ofstream::out | std::ofstream::app);
    outFile << contents.c_str( );
}

/**
* @brief Tokenize takes a string and delimiters and parses into vector
* @param str string to tokenize
* @param delimiters string containing delimiters
* @return  std vector of tokens from parsed string
*/
std::vector<std::string> CGlobals::tokenize (const std::string & str,
    const std::string & delimiters)
{
    std::vector<std::string> tokens;
    std::string::size_type   delimPos = 0, tokenPos = 0, pos = 0;

    if ( str.length( ) < 1 )
    {
        return tokens;
    }

    while ( 1 )
    {
        delimPos = str.find_first_of(delimiters, pos);
        tokenPos = str.find_first_not_of(delimiters, pos);

        if ( std::string::npos != delimPos )
        {
            if ( std::string::npos != tokenPos )
            {
                if ( tokenPos < delimPos )
                {
                    tokens.push_back(str.substr(pos, delimPos - pos));
                }
                else
                {
                    tokens.push_back("");
                }
            }
            else
            {
                tokens.push_back("");
            }
            pos = delimPos + 1;
        }
        else
        {
            if ( std::string::npos != tokenPos )
            {
                tokens.push_back(str.substr(pos));
            }
            else
            {
                tokens.push_back("");
            }
            break;
        }
    }

    return tokens;
}

static std::string trim (std::string source, std::string delims = " \t\r\n")
{
    std::string result = source.erase(source.find_last_not_of(delims) + 1);
    return result.erase(0, result.find_first_not_of(delims));
}
/**
* @brief TrimmedTokenize takes a string and delimiters and parses into
* vector,
* but trims tokens of leading and trailing spaces before saving
* @param str string to tokenize
* @param delimiters string containing delimiters
* @return  std vector of tokens from parsed string trimmed
*  tokens of leading and trailing spaces
*/
std::vector<std::string> CGlobals::trimmedTokenize (std::string value,
    std::string delimiter)
{
    std::vector<std::string> tokens = tokenize(value, delimiter);

    for ( size_t i = 0; i < tokens.size( ); i++ )
    {
        if ( tokens[i].empty( ) )
        {
            tokens.erase(tokens.begin( ) + i);
            i--;
            continue;
        }
        tokens[i] = trim(tokens[i]);
    }
    return tokens;
}

#ifdef _WINDOWS
#include "targetver.h"
#include "Windows.h"

// #include "StdStringFcn.h"
#include <Lmcons.h>

#define SECURITY_WIN32
#include "security.h"
#pragma comment(lib, "Secur32.lib")

#if 0
unsigned int CGlobals::ErrorMessage (std::string errmsg)
{
    OutputDebugString(errmsg.c_str( ) );
    std::cout << errmsg;
    return E_FAIL;
}
unsigned int CGlobals::DebugMessage (std::string errmsg)
{
    OutputDebugString(errmsg.c_str( ) );
    return E_FAIL;
}
#endif
// std::string CGlobals::ExeDirectory()
// { unsigned int CGlobals::DebugStrFormat(const char *fmt, ...) {
    va_list argptr;

    va_start(argptr, fmt);
    std::string str = FormatString(fmt, argptr);
    va_end(argptr);
    return -1; // FIXME: return DebugMessage( str);
}
//	TCHAR buf[1000];
//	GetModuleFileName(NULL, buf, 1000);
//	std::string path(buf);
//	path=path.substr( 0, path.find_last_of( '\\' ) +1 );
//	return path;
// }

#if 0
std::string CGlobals::GetUserName ( )
{
    TCHAR username[UNLEN + 1];
    DWORD size = UNLEN + 1;

    ::GetUserName( (TCHAR *) username, &size);
    return username;
}
std::string CGlobals::GetUserDomain ( )
{
    TCHAR username[UNLEN + 1];
    DWORD size = UNLEN + 1;

    // NameDnsDomain campus.nist.gov
    if ( GetUserNameEx(NameSamCompatible, (TCHAR *) username, &size) )
    {
        std::string domain = username;
        domain = domain.substr(0, domain.find_first_of('\\') );
        return domain;
    }
    return "";
}
#endif
std::string CGlobals::GetTimeStamp (TimeFormat format)
{
    SYSTEMTIME st;
    char       timestamp[64];

    GetSystemTime(&st);
    sprintf(timestamp, "%4d-%02d-%02dT%02d:%02d:%02d", st.wYear, st.wMonth,
        st.wDay, st.wHour, st.wMinute, st.wSecond);

    if ( format == GMT_UV_SEC )
    {
        sprintf(timestamp + strlen(timestamp), ".%04dZ", st.wMilliseconds);
    }
    else
    {
        strcat(timestamp, "Z");
    }

    return timestamp;
}
#else
#if 0
unsigned int CGlobals::ErrorMessage (std::string errmsg)
{
    std::cout << errmsg;
    return -1;
}
unsigned int CGlobals::DebugMessage (std::string errmsg)
{
    std::cout << errmsg;
    return -1;
}
#endif
static inline std::string FormatString(const char *fmt, va_list ap) {
    int m, n = (int) strlen(fmt) + 1028;
    std::string tmp(n, '0');

    while ((m = vsnprintf(&tmp[0], n - 1, fmt, ap)) < 0) {
        n = n + 1028;
        tmp.resize(n, '0');
    }

    return tmp.substr(0, m);
}

#include <sys/time.h>
std::string CGlobals::getTimeStamp (TimeFormat format)
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
    case LOGFILE:
        {
            strftime(timeBuffer, 50, "%Y-%m-%d-%H-%M-%S", timeinfo);
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

    return std::string(timeBuffer);

}
#endif
