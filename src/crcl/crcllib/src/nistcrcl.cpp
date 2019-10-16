

// roslaunch nistcrcl crclserver.launch
// 129.6.32.176 64444
// You can test with nist java crcl tool.
// Assuming you have it installed: cd crcl4java; ./run.sh; ->  connect 

/***
Sample ROS output:
[ INFO] [1468518803.433983207]: Crcl listen: [127.0.0.1:64444]
Accept 127.0.0.1:47319

[ INFO] [1468518814.538768991]: Crcl command: [<?xml version="1.0" encoding="UTF-8" standalone="yes"?><CRCLCommandInstance><CRCLCommand xsi:type="GetStatusType" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance"><CommandID>1</CommandID></CRCLCommand></CRCLCommandInstance>] from 127.0.0.1:47319


Problem is you need to send status to the client. And there can be multiple clients.


!!! Make sure you clean and source after you make messages, or rostopic will fail with message:
ERROR: Cannot load message class for [nistcrcl/crcl_command]. Are your messages built?

After connecting with javacrcl:
michalos@rufous:nistcrcl_ws> rostopic echo /crcl_command
xml: <?xml version="1.0" encoding="UTF-8" standalone="yes"?><CRCLCommandInstance><CRCLCommand xsi:type="GetStatusType" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance"><CommandID>1</CommandID></CRCLCommand></CRCLCommandInstance>
ip: 127.0.0.1
port: 47548
---



*/


// C++ header includes
#include <string>
#include <sys/stat.h>
#include <fcntl.h>
#include <iostream>
#include <sstream>
#include <limits.h>
#include <unistd.h>

// boost header includes
#include <boost/format.hpp>

// xerces header includes
#include <xercesc/dom/DOM.hpp>
#include <xercesc/util/PlatformUtils.hpp>
#include <xercesc/framework/XMLGrammarPoolImpl.hpp>

// ros header includes
//#include <ros/ros.h>
//#include <ros/package.h>
//#include <ros/master.h>
#include "std_msgs/String.h"

#include "crcl_rosmsgs/CrclCommandMsg.h"

#include "crcllib/nistcrcl.h"
#include "crcllib/crclserver.h"
#include "crcllib/Crcl2Rcs.h"
#include "crcllib/crclserver.h"

#include <aprs_headers/File.h>

#include "aprs_headers/Config.h"
#define GLOGGER CrclLogger
#include <aprs_headers/LoggerMacros.h>

Logging::CLogger CrclLogger;

#include <sys/time.h>
enum TimeFormat {
    HUM_READ,
    GMT,
    GMT_UV_SEC,
    LOCAL,
    LOGFILE
};
std::string getTimeStamp (TimeFormat format)
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

namespace crcl
{


bool crclServer::bDebugStatusMsg=false;
bool crclServer::bDebugCommandMsg=false;
bool crclServer::bCrclStopIgnore=false;
bool crclServer::bFlywheel=false;
bool crclServer::bProcessAllCrclMessages=false;
std::string crclServer::sRobot;
static boost::iostreams::stream<boost::iostreams::null_sink> nullout { boost::iostreams::null_sink{} };
std::ofstream *crclServer::debugstream=(std::ofstream *) &nullout;



////////////////////////////////////////////////////////////////////////////////
crclServer::crclServer(std::string crclIp,
                         int crclport,
                         double d_cycle_time,
                         std::string robot_urdf,
                         std::string base_link,
                         std::string tip_link
                         )
{

    // Crcl socke t
    this->crclIp=crclIp;
    this->crclport=crclport;

    this->urdf_xml=robot_urdf;
    this->base_link=base_link;
    this->tip_link=tip_link;

    // CRCl Communication handler - bundles xml into messages
    CBufferHandler::_bTrace = false;

    crcl2ros=std::shared_ptr<CCrcl2RosMsg> (new CCrcl2RosMsg(urdf_xml, base_link, tip_link));
    crcl2ros->setCmdQueue(NULL);

    crclCommServer= std::shared_ptr<CCrclSession> ( new CCrclSession(0.01, "Command GUI", crclport, crcl2ros.get()));
#ifdef QTHREAD
    CCrclSession::AssignMessageQueueThread(crcl2ros.get());
#else
    CCrclSession::AssignMessageQueue(&crcl2ros->msgq);
#endif

}

////////////////////////////////////////////////////////////////////////////////
void crclServer::statusUpdate(const crcl_rosmsgs::CrclStatusMsg::ConstPtr & statusmsg)
{
    crcl2ros->statusUpdate(statusmsg);
}

////////////////////////////////////////////////////////////////////////////////
void crclServer::setCmdQueue(RCS::CrclMessageQueue *crclcmdsq)
{
    crcl2ros->setCmdQueue(crclcmdsq);
}
////////////////////////////////////////////////////////////////////////////////
void crclServer::start ( )
{
    std::string inifile =getexefolder()+   + "config/Config.ini";


    CBufferHandler::_bTrace = false;

    // Initialize xercesc used by code synthesis to parse CRCL XML
    xercesc::XMLPlatformUtils::Initialize();

    // Initialize subelements of crcl2ros
    crcl2ros->init();

    // Logging setup for now

    std::string logFolder=CrclLogger.loggerFolder();
    CrclLogger.open(logFolder+"/crclxml_" + crcl2ros->robotName + ".log");
    CrclLogger.debugLevel()=3;
    CrclLogger.isTimestamping( )=true;
    CrclLogger.isOutputConsole()=true;

    // Load NIST style - less draconian errors. Maybe worse.
    Nist::Config cfg;
    if(!cfg.loadFile(inifile))
    {
        logFatal( "crcl_server can't load ini file\n");
    }

    std::string length_units = cfg.getSymbolValue<std::string>("CRCL.length_units", "METER");
    std::string angle_units = cfg.getSymbolValue<std::string>("CRCL.angle_units", "RADIAN");
    if(crcl2ros->crclinterface->SetLengthUnits(length_units) == Crcl::CANON_FAILURE)
    {
        logFatal( "SetLengthUnits from ini file failed\n");
    }

    if(crcl2ros->crclinterface->SetAngleUnits(angle_units)== Crcl::CANON_FAILURE)
    {
        logFatal( "SetAngleUnits from ini file failed\n");
    }
    crclServer::bDebugStatusMsg = (bool) cfg.getSymbolValue<int>("CRCL.DebugStatusMsg", "0");
    crclServer::bDebugCommandMsg = (bool) cfg.getSymbolValue<int>("CRCL.DebugCommandMsg", "0");
    crclServer::bCrclStopIgnore = (bool) cfg.getSymbolValue<int>("CRCL.StopIgnore", "0");
    CCrclSession::bDebugXML = (bool) cfg.getSymbolValue<int>("CRCL.DebugXML", "0");
    crclServer::bFlywheel=(bool) cfg.getSymbolValue<double>("CRCL.flywheel", "0");
    crclServer::bProcessAllCrclMessages=(bool) cfg.getSymbolValue<double>("CRCL.processAllCrclMessages", "0");
    crclServer::sRobot= cfg.getSymbolValue<std::string>("system.robots","robie");

    logStatus( " crclServer started %s\n", getTimeStamp(GMT_UV_SEC).c_str());
    if(crcl2ros->crclcmdsq != NULL)
        logStatus( " crcl2ros crclcmdsq is set\n");

    // Start asio crcl reader
    logStatus( "Start crcl socket reader\n");
    logStatus( "   crcl socket reader Ip=%s Port=%d\n",   crclIp.c_str(), crclport);

    crclCommServer->start();


    // Start crcl translater to ros msg
    logStatus( "Start crcl translater to ros msg\n");

    crcl2ros->start();

}

////////////////////////////////////////////////////////////////////////////////
void crclServer::stop ( )
{
    logStatus( "Stop crclServer %s\n", getTimeStamp(GMT_UV_SEC).c_str());

    //stop communication thread that accepts queued XML messages and translates into CRCL
    crclCommServer->stop();

    // stop processing crcl message from boost asio into RCS ros message format
    crcl2ros->stop();

    // stop code synthesis xercesc use
    xercesc::XMLPlatformUtils::Terminate();

    // close logging file
    CrclLogger.close();
}


}
