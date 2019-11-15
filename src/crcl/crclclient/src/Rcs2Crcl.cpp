

#include "crclclient/Rcs2Crcl.h"
#include "crclclient/crcl.h"
#include <aprs_headers/LoggerMacros.h>

static tf::Pose Convert(geometry_msgs::Pose m)
{
    return tf::Pose(tf::Quaternion(m.orientation.x, m.orientation.y, m.orientation.z, m.orientation.w),
                    tf::Vector3(m.position.x, m.position.y, m.position.z));
}
static Crcl::PoseToleranceType Convert(RCS::PoseTolerance t)
{
    Crcl::PoseToleranceType  tol ;
    tol.XPointTolerance (t[0]);
    tol.YPointTolerance (t[1]);
    tol.ZPointTolerance (t[2]);
    // optional orientation tolerance in XZ axes ignored
    return tol;
}


////////////////////////////////////////////////////////////////////////////////
CRcs2Crcl::CRcs2Crcl( ) :
    RCS::Thread(.01, "CRcs2Crcl")
{
    _commandnum=0;
    // Controller instantiatio of shared objects - NOT dependent on ROS
    crclinterface = boost::shared_ptr<Crcl::CrclClientCmdInterface>(
                new Crcl::CrclClientCmdInterface());
}

////////////////////////////////////////////////////////////////////////////////
void CRcs2Crcl::init(std::string crclIp,
                     int crclport,
                     double d_cycle_time,
                     RCS::ISerialLinkRobot robotkin)
{

    session= std::shared_ptr<CrclSocketClient> ( new CrclSocketClient());
    session->Init(crclIp, crclport);
    _robotkin=robotkin;
    this->crclIp=crclIp;
    this->crclport=crclport;


}

////////////////////////////////////////////////////////////////////////////////
void CRcs2Crcl::start ( )
{
    // Start  crcl socket reader/.writer
    logStatus( "Start crcl socket reader\n");
    logStatus( "   crcl socket reader Ip=%s Port=%d\n",   crclIp.c_str(), crclport);
    session->start();

    // Initialize canon and units
    std::string xmlcmd;
    xmlcmd = crclinterface->InitCanon();
    session->SyncWrite(xmlcmd);
    xmlcmd = crclinterface->SetAngleUnits("RADIAN");
    session->SyncWrite(xmlcmd);
    _angleUnit = RCS::CanonAngleUnit::RADIAN;
    _angleConversion = 1.0;

    xmlcmd = crclinterface->SetLengthUnits("METER");
    session->SyncWrite(xmlcmd);
    _lengthUnit = RCS::CanonLengthUnit::METER;
    _lengthConversion = 1.0;

    CRcs2Crcl::Thread::start();
}

////////////////////////////////////////////////////////////////////////////////
void CRcs2Crcl::stop ( )
{
    //stop communication thread that accepts queued XML messages and translates into CRCL
    session->stop();

    CRcs2Crcl::Thread::stop();
}

////////////////////////////////////////////////////////////////////////////////
int CRcs2Crcl::cmd_status(int cmdnum)
{
    std::lock_guard<std::mutex> guard(_crclmutex);
    return rcsstatus[cmdnum];
}
////////////////////////////////////////////////////////////////////////////////
RCS::CCanonWorldModel CRcs2Crcl::status()
{
    std::lock_guard<std::mutex> guard(_crclmutex);
    return _wm;
}
////////////////////////////////////////////////////////////////////////////////
int CRcs2Crcl::getStatus(RCS::CCanonWorldModel & wm)
{
    std::lock_guard<std::mutex> guard(_crclmutex);
    //https://github.com/ros-industrial/crcl/blob/master/instances/statusExample.xml
    wm.crclCommandStatus= RCS::CanonStatusType::CANON_ERROR;

    // check status to see if done
    _commandnum++;
    crclinterface->SetCommandNum(_commandnum);
    std::string xmlcmd= crclinterface->GetStatus();
    session->SyncWrite(xmlcmd);

    // now read response until echo command is getstatus
    size_t i;
    for(i=0; i< 25; i++)
    {
        if(session->MessageQueue().sizeMsgQueue() == 0)
        {
            ::sleep(0.1);
            continue;
        }
        // get the latest hopefully status.
        std::string crclstatus = session->MessageQueue().popFrontMsgQueue();

        std::cout << crclstatus << std::endl;

        if(crclstatus.find("CRCLStatus")==std::string::npos)
            continue;
        if(_status_handler.ParseCRCLStatusString(crclstatus) == RCS::CanonReturn::CANON_SUCCESS)
        {
            Crcl::CrclStatus _status = _status_handler.status();
            wm.echoCmdId=lastcmd.CommandID();
            wm.crclCommandStatus=_status.CommandStatus();
            wm.currentpose= Crcl::Convert(_status._CurrentPose);
            wm.robotjoints= Crcl::Convert(_status._CurrentJoints);
            wm._eepercent = _status.Gripper().Position();
            break;
        }

    }
    if(i==25)
        return -1;
    return 0;
}

////////////////////////////////////////////////////////////////////////////////
int CRcs2Crcl::action()
{

    std::lock_guard<std::mutex> guard(_crclmutex);
    try {
        while(cmdq.sizeMsgQueue() > 0)
        {
            // fixme: problem out of order command numbers
            // or knowing command numbers if queue size > 1?

            std::tuple<RCS::CCanonCmd,std::string> cmdseq= cmdq.popFrontMsgQueue();
            std::string xmlcmd = std::get<1>(cmdseq);
            session->SyncWrite(xmlcmd);

            lastcmd= std::get<0>(cmdseq);
            int cmdnum = lastcmd.commandNum();
            // check status to see if done
            crclinterface->SetCommandNum(cmdnum+1);
            xmlcmd= crclinterface->GetStatus();
            session->SyncWrite(xmlcmd);

            // now read status response
            if(session->MessageQueue().sizeMsgQueue() > 0)
            {
                // should get the last...
                while(session->MessageQueue().sizeMsgQueue() > 1)
                     session->MessageQueue().popFrontMsgQueue();

                // get the latest hopefully status.
                std::string crclstatus = session->MessageQueue().popFrontMsgQueue();

                if(_status_handler.ParseCRCLStatusString(crclstatus) == RCS::CanonReturn::CANON_SUCCESS)
                {
                    _status = _status_handler.status();
                    rcsstatus[cmdnum]=_status.CommandStatus();
                    _wm.echoCmdId=lastcmd.CommandID();;
                    _wm.crclCommandStatus=_status.CommandStatus();
                    _wm.currentpose= Crcl::Convert(_status._CurrentPose);
                    _wm.robotjoints= Crcl::Convert(_status._CurrentJoints);
                    _wm._eepercent = _status.Gripper().Position();
                 }
            }

        }


    }
    catch(...)
    {

    }
    return 1;
}
////////////////////////////////////////////////////////////////////////////////
int CRcs2Crcl::addcommand(crcl_rosmsgs::CrclCommandMsg msg)
{
    RCS::CCanonCmd cmd;
    cmd.Set(msg);

    std::string xmlcmd;\

    cmd.commandNum() = _commandnum;  // ignore command num from caller.
    crclinterface->SetCommandNum(_commandnum);

    switch(cmd.crclcommand)
    {
    case RCS::CanonCmdType::CANON_NOOP :
    {
        _commandnum=_commandnum-2;
        return  -1;
    }
    case RCS::CanonCmdType::CANON_INIT_CANON :
    {
        xmlcmd= crclinterface->InitCanon();
        return  cmd.commandNum();
    }
    case RCS::CanonCmdType::CANON_END_CANON :
    {
        xmlcmd= crclinterface->EndCanon(0);
        break;
    }
    case RCS::CanonCmdType::CANON_MOVE_JOINT :
    {
        ActuateJointsType::ActuateJoint_sequence crcljts = Crcl::ConvertCmdJnts(cmd.joints,_angleConversion);
        xmlcmd = crclinterface->ActuateJoints(crcljts);
        break;
    }
    case RCS::CanonCmdType::CANON_MOVE_TO :
    {
        Crcl::PoseType crclpose =  Crcl::Convert(Convert(cmd.finalpose)  ,_lengthConversion);
        xmlcmd = crclinterface->MoveTo(crclpose);
        break;
    }
    case RCS::CanonCmdType::CANON_DWELL:
    {
        xmlcmd = crclinterface->Dwell(cmd.dwell_seconds);
        break;
    }
    case RCS::CanonCmdType::CANON_MESSAGE :
    {
        xmlcmd = crclinterface->Message(cmd.opmessage);
        break;
    }
    case RCS::CanonCmdType::CANON_MOVE_THRU :
    {
#if 0
        std::vector<Crcl::PoseType> waypoints;
        std::vector<Crcl::PoseToleranceType> tolerances;
        std::vector<double> accelerations;
        for(size_t i= cmd.waypoints.size(); i< cmd.waypoints.size(); i++)
        {
            waypoints.push_back( Crcl::Convert(cmd.waypoints[i]));
            tolerances.push_back(0.0); // must stop at each waypoint for now....

        }
        cmdq.addMsgQueue( std::make_tuple( cmd.commandNum(), crclinterface->MoveThroughTo(waypoints, waypoints.size(), ));
        #endif
                return  -1;
    }
    case RCS::CanonCmdType::CANON_SET_COORDINATED_MOTION :
    {
        xmlcmd = crclinterface->SetMotionCoordination(cmd.bCoordinated);
        break;
    }
    case RCS::CanonCmdType::CANON_STOP_MOTION :
    {
        xmlcmd = crclinterface->StopMotion(cmd.stoptype);
        break;
    }
    case RCS::CanonCmdType::CANON_SET_GRIPPER :
    {
        xmlcmd = crclinterface->SetEndEffector(cmd.eepercent);
        break;
    }
    case RCS::CanonCmdType::CANON_PAVEL_GRIPPER :
    {
        xmlcmd = crclinterface->SetEndEffector(cmd.eepercent);
        break;
    }
    case RCS::CanonCmdType::CANON_OPEN_GRIPPER :
    {
        xmlcmd = crclinterface->SetEndEffector(1.0);
        break;
    }
    case RCS::CanonCmdType::CANON_CLOSE_GRIPPER :
    {
        xmlcmd = crclinterface->SetEndEffector(0.0);
        break;
    }
    case RCS::CanonCmdType::CANON_SET_TOLERANCE :
    {
        Crcl::PoseToleranceType  tol = Convert(cmd.gripperTol);
        xmlcmd = crclinterface->SetEndPoseTolerance(tol);
        //   case CanonCmdType::CANON_CONTACT_GRIPPER :    return  "CANON_CONTACT_GRIPPER";
        //   case CanonCmdType:: CANON_SET_EE_PARAMETERS:return  "CANON_SET_EE_PARAMETERS";
        break;
    }
    default:
        return  -1;
    }
    {
        std::lock_guard<std::mutex> guard(_crclmutex);
        rcsstatus[_commandnum]=RCS::CanonStatusType::CANON_WORKING;
        _commandnum=_commandnum+2;
    }
//    cmdq.addMsgQueue( std::make_tuple( cmd.commandNum(), xmlcmd));
    cmdq.addMsgQueue( std::make_tuple( cmd, xmlcmd));

    return cmd.commandNum();

}
