#include "aprs_headers/Core.h"
#include "aprs_headers/IRcs.h"
#include "aprs_headers/Debug.h"

#include "gzrcs/cros.h"
#include "gzrcs/Globals.h"

ros::NodeHandlePtr CRos::nh;
CRos Ros;
ros::AsyncSpinner * CRos::_spinner;
////////////////////////////////////////////////////////////////////

CRosCrclRobotHandler::CRosCrclRobotHandler()
{

}

////////////////////////////////////////////////////////////////////////////////
void CRosCrclRobotHandler::init(std::shared_ptr<RCS::CController> cnc, std::string prefix)
{
    _cnc=cnc;
    _prefix=prefix;
}

////////////////////////////////////////////////////////////////////////////////
void CRosCrclRobotHandler::cmdCallback(const crcl_rosmsgs::CrclCommandMsg::ConstPtr& cmdmsg)
{
    crcl_rosmsgs::CrclCommandMsg cmd(*cmdmsg);

    //   ROS_INFO("CController::CmdCallback");
    // 4/11/2018 appears as if jointnum is not filled - do hack.

    // Not sure how the joint names lined up?
    cmd.jointnum.clear();

    size_t j=0;
    for(size_t i=0; i< _cnc->robotKinematics()->jointNames.size(); i++)
    {
        if(cmd.joints.name[j] == _cnc->robotKinematics()->jointNames[i])
        {
            cmd.jointnum.push_back(i);
            j++;
        }
    }

    _cnc->crclcmds.addMsgQueue(cmd);
}
////////////////////////////////////////////////////////////////////////////////
void CRosCrclRobotHandler::start()
{
    _crclCmd = Ros.nh->subscribe(_prefix + "crcl_command", 10, &CRosCrclRobotHandler::cmdCallback, this);
    _crclStatus = Ros.nh->advertise<crcl_rosmsgs::CrclStatusMsg>(_prefix + "crcl_status", 1);
}

////////////////////////////////////////////////////////////////////////////////
void CRosCrclRobotHandler::stop()
{
    _crclCmd.shutdown();
    _crclStatus.shutdown();
}

////////////////////////////////////////////////////////////////////////////////
void CRosCrclRobotHandler::publishCrclStatus(crcl_rosmsgs::CrclStatusMsg &statusmsg)
{
    // If no one listening don't publish
    if( _crclStatus.getNumSubscribers()==0)
        return;

    statusmsg.header.stamp = ros::Time::now();
    _crclStatus.publish(statusmsg);
}

///////////////////////////////////////////////////////////////
///  CRos
//////////////////////////////////////////////////////////////////


///////////////////////////////////////////////////////////////
CRos::CRos()
{
}


////////////////////////////////////////////////////////////////////////////////
void CRos::init()
{
    // Wait for ROS master to be running -  use rosout, rosmaster is python program
    std::cout << "Connecting to ROS master ";
    while( Globals.procFind("rosout") < 0)
    {
        Globals.sleep(1000);
        std::cout << "." << std::flush;
    }
    std::cout << "\n" << std::flush;
    Globals.sleep(1000);


    ros::M_string remappings;
    remappings["__master"]=  Globals.sRosMasterUrl.c_str(); //
    remappings["__name"]= Globals.sRosPackageName; //Globals.AppProperties["Package"];

    // Initialize ROS environment ROS_MASTER_URI MUST BE SET or seg fault
    ros::init(remappings,Globals.sRosPackageName) ; // Globals.AppProperties["Package"]);

    nh = ros::NodeHandlePtr(new ros::NodeHandle);
    //  Required for multithreaded ROS communication  NOT TRUE: if not ros::spinOnce
    _spinner = new ros::AsyncSpinner(2);
    _spinner->start();

}

////////////////////////////////////////////////////////////////////////////////
CRosCrclRobotHandler CRos::addRobotController(std::shared_ptr<RCS::CController> cnc, std::string prefix)
{
    CRosCrclRobotHandler crcl;
    _crclRosCncs.push_back(crcl);
    crcl.init( cnc, prefix);
    return crcl;
}

////////////////////////////////////////////////////////////////////////////////
void CRos::close()
{
#ifdef ROS
    if(Globals.bRos)
        ros::shutdown();
#endif
}

