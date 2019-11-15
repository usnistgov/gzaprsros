
#include <urdf/model.h>
#include <CRCLStatus.hxx>
#include "crclclient/Crcl2Rcs.h"
#include <aprs_headers/Debug.h>
#include <crclclient/nistcrcl.h>

#define GLOGGER CrclLogger
#include <aprs_headers/LoggerMacros.h>


using namespace urdf;
using namespace RCS;
namespace RCS {
    CMessageQueue<RCS::CCanonCmd> cmds;
    CCanonWorldModel wm; // for motion control planning wm
}
boost::mutex CCrcl2RosMsg::cncmutex;

size_t num_joints;
size_t num_links;



////////////////////////////////////////////////////////////////////////////////
CCrcl2RosMsg::CCrcl2RosMsg(std::string xml_string, std::string base_link, std::string tip_link) :
    RCS::Thread(.01, "CCrcl2RosMsg")
{
    this->xmlString=xml_string;
    this->baseLink=base_link;
    this->tipLink=tip_link;
    this->crclcmdsq=NULL;
}

////////////////////////////////////////////////////////////////////////////////
bool CCrcl2RosMsg::parseURDF(std::string xml_string, std::string base_link, std::string tip_link)
{
    urdf::Model robot_model;
    if(xml_string.empty())
    {
        return false;
    }

    robot_model.initString(xml_string);

    robotName=robot_model.getName();

    urdf::LinkConstSharedPtr link =robot_model.getLink(tip_link);
    while (link->name != base_link) { // && joint_names.size() <= num_joints_) {
        linkNames.push_back(link->name);
        urdf::JointSharedPtr joint   = link->parent_joint;
        if (joint) {
            if (joint->type != urdf::Joint::UNKNOWN && joint->type != urdf::Joint::FIXED) {

                jointNames.push_back(joint->name);
                axis.push_back(Convert<urdf::Vector3, tf::Vector3>(joint->axis));
                xyzorigin.push_back(Convert<urdf::Vector3, tf::Vector3>(joint->parent_to_joint_origin_transform.position));
                double roll, pitch, yaw;
                joint->parent_to_joint_origin_transform.rotation.getRPY(roll, pitch, yaw);
                rpyorigin.push_back(tf::Vector3(roll, pitch, yaw));

                float lower, upper, maxvel = 0.0, maxeffort = 0.0;
                int hasLimits;
                if (joint->type != urdf::Joint::CONTINUOUS) {
                    maxvel = joint->limits->velocity;
                    maxeffort = joint->limits->effort;
                    if (joint->safety) {
                        lower = joint->safety->soft_lower_limit;
                        upper = joint->safety->soft_upper_limit;
                    } else {
                        lower = joint->limits->lower;
                        upper = joint->limits->upper;
                    }
                    hasLimits = 1;
                } else {
                    lower = -M_PI;
                    upper = M_PI;
                    hasLimits = 0;
                }
                if (hasLimits) {
                    jointHasLimits.push_back(true);
                    jointMin.push_back(lower);
                    jointMax.push_back(upper);
                } else {
                    jointHasLimits.push_back(false);
                    jointMin.push_back(-M_PI);
                    jointMax.push_back(M_PI);
                }
                jointEffort.push_back(maxeffort);
                jointVelmax.push_back(maxvel);
            }
        } else {
            ROS_WARN_NAMED("nc", "no joint corresponding to %s", link->name.c_str());
        }
        link = link->getParent();
    }

    std::reverse(linkNames.begin(), linkNames.end());
    std::reverse(jointNames.begin(), jointNames.end());
    std::reverse(jointMin.begin(), jointMin.end());
    std::reverse(jointMax.begin(), jointMax.end());
    std::reverse(jointHasLimits.begin(), jointHasLimits.end());
    std::reverse(axis.begin(), axis.end());
    std::reverse(xyzorigin.begin(), xyzorigin.end());
    std::reverse(rpyorigin.begin(), rpyorigin.end());
    std::reverse(jointEffort.begin(), jointEffort.end());
    std::reverse(jointVelmax.begin(), jointVelmax.end());

    return true;
}
////////////////////////////////////////////////////////////////////////////////
void CCrcl2RosMsg::statusUpdate(const crcl_rosmsgs::CrclStatusMsg::ConstPtr& statusmsg)
{
    std::lock_guard<std::mutex> guard(_crclmutex);
     try {
        crclinterface->crclwm.StatusID() = (unsigned long long) statusmsg->crclcommandnum;
        static char * value[] =
        {
            "CRCL_Done",
            "CRCL_Error",
            "CRCL_Working",
            "CRCL_Ready"
        };
        //        crclinterface->crclwm.CommandStatus() = ::CommandStateEnumType(CanonStatusType::value(statusmsg->crclcommandstatus));
        if(statusmsg->crclcommandstatus < 4)
            crclinterface->crclwm.Update( ::CommandStateEnumType( value[statusmsg->crclcommandstatus]));

        // FIXME: this has to be in the agreed upon CRCL units
        tf::Pose pose = Convert< geometry_msgs::Pose, tf::Pose>(statusmsg->statuspose);

        // default tf length units are Meters - convert to crcl
        pose.setOrigin(pose.getOrigin() *  1.0/crclinterface->crclwm._lengthConversion);
        crclinterface->crclwm.Update(pose );

        sensor_msgs::JointState js = statusmsg->statusjoints;

        for(size_t i=0; i< js.position.size(); i++)
            js.position[i]=js.position[i]* 1.0/crclinterface->crclwm._angleConversion;

        crclinterface->crclwm.Update((sensor_msgs::JointState&) js);
        crclinterface->crclwm.Gripper().Position() = statusmsg->eepercent;
    }
    catch(...)
    {
        logFatal("crclwm command status failed\n");
    }
}

////////////////////////////////////////////////////////////////////////////////
void CCrcl2RosMsg::init()
{

    // Controller instantiatio of shared objects - NOT dependent on ROS
    crclinterface = boost::shared_ptr<Crcl::CrclServerDelegateInterface>(
            new Crcl::CrclServerDelegateInterface());
    crclinterface->SetAngleUnits("DEGREE");

    // Couple code attempts at reading from robot_description parameter - see above
    crclinterface->crclwm.jointnames.clear();

    parseURDF(xmlString, baseLink,  tipLink);

    crclinterface->crclwm.jointnames=jointNames;
}


////////////////////////////////////////////////////////////////////////////////
int CCrcl2RosMsg::action()
{
    try {

        int n=0;
        /////////////////////////////////////////////////////////////////////////////////////////////
        // See if new CRCL commanded motion - if so, interpret as RCS command in session
        if(CCrclSession::InMessages().sizeMsgQueue() > n)
        {
            CrclMessage msg = CCrclSession::InMessages().popFrontMsgQueue();
            std::string crclcmd = boost::get<0>(msg);
            Crcl::CrclReturn ret = crclinterface->DelegateCRCLCmd(crclcmd);

            if (ret == Crcl::CANON_STATUSREPLY)
            {
                Crcl::CrclStatus crclstatus;
                {
                    // can't have reporting and robot controller so make copy
                    std::lock_guard<std::mutex> guard(_crclmutex);
                    crclstatus = crclinterface->crclwm;
                }
                std::string sStatus = Crcl::CrclClientCmdInterface().GetStatusReply(&crclstatus);

                // no mutex as there is only one session running handling all client CRCL messages
                CCrclSession *_pSession;
                _pSession = boost::get<1>(msg);
                _pSession->SyncWrite(sStatus);

                if(crcl::crclServer::bDebugStatusMsg)
                {
                    logDebug("===========================================================\n"
                              "[%s]\n",
                              crclcmd.c_str());
                }

            }
            if(crcl::crclServer::bFlywheel)
            {
                n=CCrclSession::InMessages().sizeMsgQueue();
            }
        }
        /////////////////////////////////////////////////////////////////////////////////////////////
        // interpret translated CRCL command. Commands in canonical form: standard units (mm, radians)
        // Note many CRCL commands are NOT translated into corresponding canoncial ROS commands
        // FIXME: this is an extra step that is probably unncessary
        n = 0; // flywheel implementation - whether to process all messages at once or per cycle.
        for (;RCS::cmds.sizeMsgQueue() > n;)
        {
            RCS::CCanonCmd cc = RCS::cmds.popFrontMsgQueue();
            crcl_rosmsgs::CrclCommandMsg rosmsg;

            enum crclcommand {
                noop = 0,
                initCanon = 1,
                endCanon = 2,
                actuatejoints = 3,
                moveto = 4,
                dwell = 5,
                message = 6,
                moveThroughTo = 7,
                setCoordinatedMotion = 8,
                stopMotion = 9,
                setEndEffector = 10,
                openToolChange = 11,
                closeToolChanger = 12,
                setEEParameter = 25,
                nocommand=26

            };
            rosmsg.crclcommand = nocommand;
            //rosmsg.crclcommandnum = cc.CommandID();
            rosmsg.crclcommandnum = (long unsigned int) cc.CrclCommandID();
            CCanonCmd::setRosMsgTimestamp(rosmsg.header);
            // ROS equivalent
            //rosmsg.header.stamp=ros::Time::now();

            if(cc.crclcommand == CanonCmdType::CANON_INIT_CANON)
            {
                rosmsg.crclcommand=initCanon;
            }
            else if(cc.crclcommand == CanonCmdType::CANON_END_CANON)
            {
                rosmsg.crclcommand=endCanon;
            }
            else if (cc.crclcommand == CanonCmdType::CANON_MOVE_JOINT)
            {
                rosmsg.crclcommand = actuatejoints;
                rosmsg.jointnum=cc.jointnum;
                rosmsg.joints = cc.joints; // this passes pos, vel, accel/force/torque
                rosmsg.bCoordinated = cc.bCoordinated;
            }
            else if (cc.crclcommand == CanonCmdType::CANON_MOVE_TO)
            {
                rosmsg.crclcommand = moveto;
                rosmsg.finalpose.position.x = cc.finalpose.position.x;
                rosmsg.finalpose.position.y = cc.finalpose.position.y;
                rosmsg.finalpose.position.z = cc.finalpose.position.z;
                rosmsg.finalpose.orientation.x = cc.finalpose.orientation.x;
                rosmsg.finalpose.orientation.y = cc.finalpose.orientation.y;
                rosmsg.finalpose.orientation.z = cc.finalpose.orientation.z;
                rosmsg.finalpose.orientation.w = cc.finalpose.orientation.w;

                // Fixme: rates are ignored by RCS crcl interpreter
                if(cc.Rates().CurrentTransSpeed() >0.0)
                {
                    // clear rate vector
                    rosmsg.profile.clear();

                    // save current translation rate definition in move
                    // ignores specified acceleration for now
                    ::crcl_rosmsgs::CrclMaxProfileMsg trans_profile;
                    trans_profile.maxvel=cc.Rates().CurrentTransSpeed();
                    trans_profile.maxacc=10.*cc.Rates().CurrentTransSpeed();
                    trans_profile.maxjerk= 100.*cc.Rates().CurrentTransSpeed();
                    rosmsg.profile.push_back(trans_profile);

                    // save current rotational rate definition in move
                    // ignores specified acceleration for now
                    ::crcl_rosmsgs::CrclMaxProfileMsg rot_profile;
                    rot_profile.maxvel=cc.Rates().CurrentRotSpeed();
                    rot_profile.maxacc=10.*cc.Rates().CurrentRotSpeed();
                    rot_profile.maxjerk= 100.*cc.Rates().CurrentRotSpeed();
                    rosmsg.profile.push_back(rot_profile);
                }
            }
            else if (cc.crclcommand == CanonCmdType::CANON_STOP_MOTION)
            {
                // Fixme: there are other parameters specifying stop
                if(crcl::crclServer::bCrclStopIgnore)
                {
                    rosmsg.crclcommand = stopMotion;
                    //cc.stoptype; // fixme: add stoptype to crcl messages
                }            // publish ros message if found corresponding crcl command
                if (rosmsg.crclcommand != noop) {
                    logInform("ROS command: [%d] ", rosmsg.crclcommand);
                    crclcmdsq->addMsgQueue(rosmsg);
                }
             }
            else if (cc.crclcommand == CanonCmdType::CANON_MOVE_THRU)
            {
                rosmsg.crclcommand = moveThroughTo;
                rosmsg.finalpose = cc.finalpose;

                // now save waypoints
                rosmsg.waypoints=cc.waypoints;
            }
            else if (cc.crclcommand == CanonCmdType::CANON_DWELL)
            {
                rosmsg.crclcommand = dwell;
                rosmsg.dwell_seconds = cc.dwell;
            }
            else if (cc.crclcommand == CanonCmdType::CANON_SET_GRIPPER)
            {
                rosmsg.crclcommand = setEndEffector;
                rosmsg.eepercent = cc.eepercent;
            }
            else if (cc.crclcommand == CanonCmdType::CANON_SET_EE_PARAMETERS)
            {
                rosmsg.crclcommand = setEEParameter;
                rosmsg.parameter_names = cc.parameterNames;
                rosmsg.parameter_values = cc.parameterValues;
            }
            // added oct 30 2018 to see if "processing all messages important"
            // actually no flywheel but processAllCrclMessages
            else if(!crcl::crclServer::bProcessAllCrclMessages)
            {
                rosmsg.crclcommand=noop;
            }

            // publish ros message if found corresponding crcl command
            if (rosmsg.crclcommand != nocommand) {
                logInform("ROS command: [%d] ", rosmsg.crclcommand);
                crclcmdsq->addMsgQueue(rosmsg);
            }

            // publish ros message if found corresponding crcl command
            if (crcl::crclServer::bDebugCommandMsg) {
                printf("ROS command: %s\n", CCanonCmd().Set(rosmsg).toString().c_str());
            }

            if(crcl::crclServer::bFlywheel)
            {
                n=RCS::cmds.sizeMsgQueue();
            }

        }
    }
    catch (std::exception & e)
    {
        std::cerr << "Exception in  CController::Action() thread: " << e.what() << "\n";
    }
    catch (...)
    {
        std::cerr << "Exception in CController::Action() thread\n";
    }
    return 1;
}
