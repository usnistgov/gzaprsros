// Globals.h

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

#ifndef _GLOBALS_H_
#define _GLOBALS_H_
#include <stdio.h>
#include <stdarg.h>

#include <vector>
#include <map>
#include <string>
#include <fstream>
#include <sstream>
#include <sstream>
#include <cstring>
#include <ctime>
#include <chrono>
#include <thread>

#ifndef DEBUG
#include "boost/iostreams/stream.hpp"
#include "boost/iostreams/device/null.hpp"
#endif

#include <boost/thread.hpp>
#include <boost/preprocessor.hpp>

#include <boost/foreach.hpp>
#include <boost/tokenizer.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/any.hpp>

#include <tf/tf.h>

#include <aprs_headers/Core.h>
#include "aprs_headers/Config.h"
#include "aprs_headers/seriallinkrobot.h"

namespace RCS {

extern Nist::Config robotconfig;
extern std::mutex cncmutex;
static const int NORMAL = 0;
static const int EXITING = 1;
static const int PAUSED = 2;
static const int ERROR = 3;
static const int ONCE = 4;
static const int NOOP = 5;
static const int AUTO = 6;
extern int state;
}

#ifndef ROSPACKAGENAME
#define ROSPACKAGENAME "gzrcsdemo"
#endif

#ifndef ROS
//#define ROS_INFO(X) LOG_FATAL << X << "\n"
//#define ROS_DEBUG(X) LOG_FATAL << X << "\n"
//#define ROS_DEBUG_NAMED(...)
//#define ROS_FATAL_NAMED(...)
//#define ROS_ERROR_STREAM(X) LOG_FATAL << X << "\n"
//#define ROS_WARN_NAMED(...)
#endif

#ifndef TODO
#define DO_PRAGMA(x) _Pragma (#x)
          #define TODO(x) DO_PRAGMA(message ("TODO - " #x))
#endif      

/** \def TOOLS_DEPRECATED
    Macro that marks functions as deprecated -from ROS */

#ifdef __GNUC__
#define TOOLS_DEPRECATED __attribute__((deprecated))
#elif defined(_MSC_VER)
#define TOOLS_DEPRECATED __declspec(deprecated)
#elif defined(__clang__)
#define TOOLS_DEPRECATED __attribute__((deprecated("Use of this method is deprecated")))
#else
#define TOOLS_DEPRECATED /* Nothing */
#endif


#if defined DEBUG
#define IfDebug(arg)    arg
#else
#define IfDebug(arg)
#endif

//#if  ! defined S_OK
//#define strncasecmp(x, y, z)    _strnicmp(x, y, z)
//#else,Globals.slr
//#define _strnicmp strncasecmp
//#define S_OK 0
//#define E_FAIL -1
//#endif


// Adapter from  IKFAST
#ifndef NC_ASSERT
#include <stdexcept>
#include <sstream>
#include <iostream>
#ifndef __PRETTY_FUNCTION__
#define __PRETTY_FUNCTION__ __func__
#endif
#define NC_ASSERT(b) { if( !(b) ) { std::stringstream ss; ss << "nc exception: " << __FILE__ << ":" << __LINE__ << ": " <<__PRETTY_FUNCTION__ << ": Assertion '" << #b << "' failed"; throw std::runtime_error(ss.str()); } }
#endif



#ifndef CLEANSTORE
#define CLEANSTORE(Y, X, Z) \
	try{ Y = X; }           \
	catch ( ... ) { Y = Z; }
#define VALIDSTORE(Y, X) \
	try{ Y = X; }        \
	catch ( ... ) { }
#endif



/**
 * \brief gives a random from 0 to 1
 * \return double
 */
inline double Random() {
    return ((double) rand() / (RAND_MAX));
}

inline int OneZero() {
    return ((double) rand() / (RAND_MAX));
}

/**
 * \brief gives a random between min and max
 * \param min minimum number as double
 * \param max maximum number as double
 * \return number as double between min and max
 */
inline double Random(double min, double max) {
    return min + Random() * (max - min); //gives a random from 0 to max ) +min
}

/**
 * \brief gives a random between min and max
 * \param min minimum number as integer
 * \param max maximum number as integer
 * \return number as integer between min and max
 */
inline int Random(int min, int max) {
    return min + (rand() % (max - min)); //gives a random from 0 to max ) +min
}

/**
 * \brief CGlobals is a catch-all data structure for collecting global functions, extensions, parameters, etc.
 *  Functions here usually vary between windows and linux, or
 * there is no easy mechanism in C++ to extend classes (e.g., string) like in C#
 */
class CGlobals {
public:

    enum TimeFormat {
        HUM_READ,
        GMT,
        GMT_UV_SEC,
        LOCAL,
        LOGFILE
    };
    /*!
     * \brief Constructor for globals function. Functions here usually vary between windows and linux, or
     * there is no easy mechanism in C++ to extend classes (e.g., string) like in C#.
     */
    CGlobals();

    /**
      * \brief destructor. Closes all output debug streams.
      * */
    ~CGlobals();

    /**
     * @brief catch_control_c provides a Linux mechanism to catch a console control c.
     * In this method the flag CGlobals::bRunning is set to false.
     */
    void catchControlC();

    /**
     * @brief Ok  checks if application has been terminated by control C
     * @return false if control c hit
     */
    bool ok();

    /**
     * @brief proc_find programmatically determine if a process is already running on Linux
     * @param name  process or program name
     * @return pid if true, -1 if not found.
     */
    pid_t procFind(const char* name) ;


    /*!
     * \brief StrFormat  accepts a traditional C format string and expects parameter to follow on calling stack and will
     * produce a string from it.
     * \param fmt is the C format string.
     */
    std::string strFormat(const char *fmt, ...);


    template<typename ... Args>
    std::string format( const std::string& format, Args ... args )
    {
        size_t size = snprintf( nullptr, 0, format.c_str(), args ... ) + 1; // Extra space for '\0'
        std::unique_ptr<char[]> buf( new char[ size ] );
        snprintf( buf.get(), size, format.c_str(), args ... );
        return std::string( buf.get(), buf.get() + size - 1 ); // We don't want the '\0' inside
    }

    /*!
     * \brief sleep milliseconds. Equivalent to Sleep in windows.
     * \param ms number of milliseconds to sleep
     */
    void sleep(unsigned int ms);
    /*!
     * \brief Reads a file all at once into a string. Include file open, read, close. If fails, empty string is only diagnostic.
     * \param filename is the name of the file to read from
     * \param contents is a reference to a string in which to store file contents.
     */
    bool readFile(std::string filename, std::string & contents);

    /*!
     * \brief Writes entire string contents to a file all at once. Include file open, write, close.
     * No error messages.
     * \param filename is the name of the file to write to
     * \param contents is a reference to a string in which to write string.
     */
    void writeFile(std::string filename, std::string & contents);

    /*!
     * \brief Appends entire string contents to a file all at once. Include file open, write, close.
     * No error messages.
     * \param fi    static const int NORMAL = 0;
    static const int EXITING = 1;
    static const int PAUSED = 2;
    static const int ERROR = 3;
    static const int ONCE = 4;
    static const int NOOP = 5;
    static const int AUTO = 6;
    int state;lename is the name of the file to write to
     * \param contents is a reference to a string in which to write string.
     */
    void appendFile(std::string filename, std::string contents);

    /*!
     * \brief Trim cleans blank characters from the front and back of a string. 
     * Blank chars are white space, tab, carriage return.
     * \param str is the string to trim. Will trim a copy.
     * \return a new trimmed string 
     */
    std::string trim(std::string s);

    /*!
     * \brief GetTimeStamp returns a timestamp string depending on the input format.
     * \param  format is one of an enumeration describing how to format timestamp.
     * \return a formated timestamp string. 
     */
    std::string getTimeStamp(TimeFormat format = GMT_UV_SEC);


    /*!    static const int NORMAL = 0;
    static const int EXITING = 1;
    static const int PAUSED = 2;
    static const int ERROR = 3;
    static const int ONCE = 4;
    static const int NOOP = 5;
    static const int AUTO = 6;
    int state;
     * \brief Open logging streams for debugging. Close on CGlobals destroctor.
     */
    void debugSetup();

    /*!
     * \brief Assign a new output file stream to a file stream.
     */    
    void assignOfs(std::ostream *inOfs, std::ostream *replacementOfs);

    /**
    * @brief Tokenize takes a string and delimiters and parses into vector
    * @param str string to tokenize
    * @param delimiters string containing delimiters
    * @return  std vector of tokens from parsed string
    */
    std::vector<std::string> tokenize (const std::string & str,
        const std::string & delimiters);
    /**
    * @brief TrimmedTokenize takes a string and delimiters and parses into
    * vector,
    * but trims tokens of leading and trailing spaces before saving
    * @param str stslrring to tokenize
    * @param delimiters string containing delimiters
    * @return  std vector of tokens from parsed string trimmed
    *  tokens of leading and trailing spaces
    */
    std::vector<std::string> trimmedTokenize (std::string value,
        std::string delimiter);

    /**
     * @brief strconvert convert data of type Template T into string
     * @param data of type T
     * @return  string
     */
    template<typename T>
    std::string strConvert (T data )
    {
        std::ostringstream ss;
        try {
            ss <<  std::fixed << std::setprecision(3)<< data;
        }
        catch(...)
        {
        }
        return ss.str();
    }

    /**
     * @brief convert  changes a string into a type
     * @param data string containg type T
     * @return type T instance that was contained in string.
     */
    template<typename T>
    T convert (std::string data )
    {
        T result;
        try {
            std::istringstream stream(data);

            if ( stream >> result )
            {
                return result;
            }
            //else if ( ( data == "yes" ) || ( data == "true" ) )
            //{
            //	return 1;
            //}
        }
        catch(...)
        {
            // FIXME: should throw on error.
            throw std::runtime_error("Bad conversion");
        }
        return T();
    }
    /**
     * @brief split split up text into a vector of strings based on a separator within string
     * @param text string to split into vector
     * @param sep string separator to use to split string
     * @param bKeepEmpty boolean flag upon whether to keep empty strings. Default is false.
     * @return vector of strings split up by separator
     */
    std::vector<std::string> split(const std::string &text, char sep, bool bKeepEmpty = false);

    /**
     * @brief logfilesSetup sets up logging folder. makes folder if none exists.
     */
    void logfilesSetup();

    // -----------------------------------------
    // Application specifie parameters
    std::map< std::string, std::string> appProperties; /**<map of application properties, e.g., ["prop"]="value" */
    std::string inifile; /**< inifile path name  */
     static bool bRunning;
     static bool bPaused;
    int bPublishPoint;
    int bRos;
    int bGazebo;
    int bHandleExceptions;
    int bReadAllInstances;
    int bCannedDemo;
    int bWorldCRCLCoordinates;
    int bGearLocations;
    int bGripperSpeed;
    int bClosestFree;
    int bClosestOpenSlot;
    int bFlywheel;
    int bGzGripperPlugin;

    std::string sRosMasterUrl;
    std::string sRosPackageName;

    VAR(std::string, logfolder);

    VAR(tf::Quaternion, QBend); /**< rotation to achieve pose rotation for grasping */
    VAR(tf::Pose, Retract);         /**< pose offset for retract */
    VAR(tf::Pose, RetractInv); /**< inverse pose offset for retract */
    VAR(tf::Pose, Correction);         /**< correction offset for pose */
    VAR(tf::Pose, CorrectionInv);         /**< inverse correction offset for pose */
    // Demo specific  - Parts and offsets variables
    VAR(std::vector<std::string>, part_list); /// robot parts from set of all parts
    VAR(SINGLE_ARG(std::map<std::string, tf::Pose>), gripperoffset);       /// gripper offset for each part
    VAR(SINGLE_ARG(std::map<std::string, tf::Pose>), slotoffset);          /// gripper offset for container slot
    VAR(SINGLE_ARG(std::map<std::string, double>), graspforce);            /// grasping force for robot gripper for a part

    //tf::Pose & _basePose;  /**< world to robot base transform  */
    VAR(tf::Pose, basePose); /**< inverse pose offset for retract */


    // Debug Flags for more debugging information:
    VAR(bool, DEBUG_World_Command); // Log controller action loop for robot servo of world cartesian move
    VAR(bool, DEBUG_Log_Robot_Position);
    VAR(bool, DEBUG_Log_Robot_Config);
    VAR(bool, DEBUG_Log_Cyclic_Robot_Position);

    // Motion rate variables
    VAR(std::vector<double>, linearmax);
    VAR(std::vector<double>, rotationmax);
    VAR(std::vector<double>, base_linearmax);
    VAR(std::vector<double>, base_rotationmax);

    VAR(double, currentLinearSpeed);
    VAR(double, currentAngularSpeed);
    VAR(double, currentRobotJointSpeed );
    VAR(double, currentGripperJointSpeed );

    VAR(double, accelerationMultipler);

    // Saved named robot goal states
    VAR(SINGLE_ARG(std::map<std::string, std::vector<std::string>>), namedCommand);
    VAR(SINGLE_ARG(std::map<std::string, std::vector<double>>), namedJointMove);
    VAR(SINGLE_ARG(std::map<std::string, tf::Pose>), namedPoseMove);


    VAR(std::vector<long unsigned int>, allJointNumbers);
    VAR(std::vector<std::string>, jointNames); // this needs to read URDF or hard code in ini file

    VAR(std::string,crclIp );
    VAR(size_t,crclPort );

};
extern CGlobals Globals; /**< global definition of globals  */
#ifndef WIN32
extern void DebugBreak(); /**< global definition of windows DebugBreak equivalent.  */
#endif

extern std::ofstream ofsRobotCrcl;
extern std::ofstream ofsRobotURDF;
extern std::ofstream ofsMotionTrace;
extern std::ofstream LOG_FATAL;
#ifdef DEBUG
#define LOG_DEBUG LOG_FATAL
#else
extern boost::iostreams::stream< boost::iostreams::null_sink > LOG_DEBUG;
#endif
#endif
