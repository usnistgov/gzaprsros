// RCSInterpreter.cpp

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

#include <algorithm>  // max
#include <boost/ref.hpp>

#include "gzrcs/RCSInterpreter.h"
#include "gzrcs/Controller.h"
#include "gzrcs/Globals.h"
#include "gzrcs/RobotControlException.h"

#include "aprs_headers/Conversions.h"
#include "aprs_headers/Debug.h"
#include "aprs_headers/Logger.h"
#include "aprs_headers/LoggerMacros.h"

// Debug flags are set in Debug.h
// Depending on what is defined, you will log certain information


#define TRAJ_LOG  std::cout
//#define TRAJ_LOG  ofsRobotMoveTo

using namespace RCS;
using namespace sensor_msgs;

////////////////////////////////////////////////////////////////////////////////
static void LogRobotPosition(std::string message,
                             tf::Pose r_goalpose,
                             tf::Pose r_lastpose,
                             tf::Pose r_nextpose,
                             std::vector<double> goaljoints,
                             std::vector<double> nextjoints,
                             std::vector<double> curjoints)

{

    if(!Globals.DEBUG_Log_Robot_Position())
        return;

    std::stringstream ss;
#if 1
    ss << message;
    ss << "ROBOT COORDINATES\n";
    ss << "    GoalRobot Pose= " << RCS::dumpPoseSimple(r_goalpose).c_str() << "\n";
    ss << "    Current   Pose= " << RCS::dumpPoseSimple(r_lastpose).c_str() << "\n";
    ss << "    NextRobot Pose= " << RCS::dumpPoseSimple(r_nextpose).c_str() << "\n";
    ss << "ROBOT JOINTS\n";
    ss << "    Goal joints     "  << RCS::vectorDump<double>(goaljoints) << "\n" << std::flush;
    ss << "    Next joints     "  << RCS::vectorDump<double>(nextjoints) << "\n" << std::flush;
    ss << "    Current joints  "  << RCS::vectorDump<double>(curjoints) << "\n" << std::flush;
    //TRAJ_LOG<< ss.str();
#else
    ss << "    GoalRobot Pose= " << RCS::dumpPoseSimple(r_goalpose).c_str() << "\n";
    ss << "    Goal joints     "  << RCS::vectorDump<double>(goaljoints) << "\n" << std::flush;
    ss << RCS::vectorDump(nextjoints) << ","
       << boost::format("%5.2f") % r_nextpose.getOrigin().getX() << ","
       << boost::format("%5.2f") % r_nextpose.getOrigin().getY() << ","
       << boost::format("%5.2f") % r_nextpose.getOrigin().getZ() << "\n" << std::flush;
#endif
    if(Globals.DEBUG_Log_Robot_Position())
        ofsMotionTrace << ss.str();
    if(Globals.DEBUG_Log_Robot_Position()>1)
        std::cout << ss.str();

}
static void LogGripperStatus(std::string message,
                             std::shared_ptr<RCS::CController>_nc,
                             RCS::CCanonCmd &incmd,
                             bool & bFullContact
                             )
{
    if(Globals.DEBUG_Log_Gripper_Status())
    {
        std::stringstream ss;
        ss << message;
        ss << "Gripper JOINTS\n";
        ss << "    Gripper Fingers "  << RCS::vectorDump<std::string>(_nc->fingerNames()) << "\n" << std::flush;
        ss << "    CMD  joints     "  << RCS::vectorDump<double>(incmd.nextGripperGoalJoints.position) << "\n" << std::flush;
        ss << "    Current joints  "  << RCS::vectorDump<double>(_nc->cncGripperJoints().position) << "\n" << std::flush;

        //TRAJ_LOG<< ss.str();
        ofsMotionTrace << ss.str();
    }
}
////////////////////////////////////////////////////////////////////////////////
// @todo problem if is going from 0..-n so that max is negative number
static bool joint_compare(std::vector<double> j1, std::vector<double> max_j2)
{
    if(j1.size() != max_j2.size())
        return false;
    for(size_t i=0; i< j1.size(); i++)
        if(fabs(j1[i])< fabs(max_j2[i]))
            return false;
    return true;
}


/**
  * @brief UpdateJointState convert from crcl number based joint spec to ros  name based,
  * Crcl joint indexes should already be adjusted to zero-based, NOT one-based counting
  * @param jointnums vector of joint numbers
  * @param oldjoints vector containing old joint position
  * @param njoints new joints
  * @return sensor_msgs::JointState new joint state with update
  */
sensor_msgs::JointState CGoInterpreter::updateJointState(std::vector<uint64_t> jointnums,
                                                         sensor_msgs::JointState oldjoints,
                                                         sensor_msgs::JointState newjoints) {
    sensor_msgs::JointState joints = oldjoints;
    if (joints.velocity.size() != joints.position.size())
        joints.velocity.resize(joints.position.size(), 0.0);
    if (joints.effort.size() != joints.position.size())
        joints.effort.resize(joints.position.size(), 0.0);

    // Check each joint, to see if joint is being actuated, if so, change goal position
    for (size_t i = 0; i < jointnums.size(); i++) {
        size_t n = jointnums[i]; // should already have indexes -1;
        joints.position[n] = newjoints.position[i]; // joint numbers already adjusted from CRCL to rcs model
        if(newjoints.velocity.size() > i)
            joints.velocity[n] = newjoints.velocity[i];
        if(newjoints.effort.size() > i)
            joints.effort[n] = newjoints.effort[i];
    }
    return joints;
}

////////////////////////////////////////////////////////////////////////////////
///  GoInterpreter
/////////////////////////////////////////////////////////////////////////////
CGoInterpreter::CGoInterpreter(std::shared_ptr<RCS::CController> nc,
                               boost::shared_ptr<RCS::IKinematic> k)
    :  _nc(nc)
    , _kinematics(k)
    , _lastcmdnum(-1)
{

    _goRobot = std::shared_ptr<GoTraj> (new GoTraj());
    _goGripper = std::shared_ptr<GoTraj> (new GoTraj());
    _name="GoTraj"; /// thread name
}

////////////////////////////////////////////////////////////////////////////////
void CGoInterpreter::init(std::vector<double> initjts)
{
    sensor_msgs::JointState jts = RCS::emptyJointState(initjts.size());
    jts.position = initjts;
    jts.name = this->_nc->robotKinematics()->jointNames;
    if(0!=_goRobot->Init(jts, this->_nc->cycleTime()))
    {
        logFatal( "Go robot init failed\n") ;
    }

    jts=_nc->cncGripperJoints();
    if(0!=_goGripper->Init(jts, this->_nc->cycleTime()))
    {
        logFatal( "Go gripper init failed\n") ;
    }
}
///////////////////////////////////////////////////////////////////////////////
int CGoInterpreter::parseJointCommand(crcl_rosmsgs::CrclCommandMsg &incmd,
                                      RCS::CCanonCmd &outcmd,
                                      RCS::CCanonWorldModel instatus)
{

    outcmd.Set( incmd );

    // Copy current joint rates suitable for gotraj
    std::vector<gomotion::GoTrajParams> jparams = _nc->robotJointRateParams();

    // establish the goal joint positions for all joints
    sensor_msgs::JointState finaljoints;
    finaljoints = updateJointState(incmd.jointnum, instatus.robotjoints, incmd.joints);
    if (incmd.crclcommandnum != _lastcmdnum)
    {
        _lastcmdnum = incmd.crclcommandnum;

        // setup speed joint speed parameters if any
        _nc->updateJointRates(incmd.jointnum,incmd.joints);

        // prime gotraj trajectory generator
        _goRobot->InitJoints(instatus.robotjoints,  // now
                             finaljoints,  // goal
                             jparams);
    }

    outcmd.joints = _goRobot->NextJoints();

    // check if go is done
    if (_goRobot->IsDone())
    {
        _lastcmdnum=-1;
        return CanonStatusType::CANON_DONE;
    }
    else
        return CanonStatusType::CANON_WORKING;
}

////////////////////////////////////////////////////////////////////////////////
int CGoInterpreter::parseMoveThruCommand(crcl_rosmsgs::CrclCommandMsg &incmd,
                                         RCS::CCanonCmd &outcmd,
                                         RCS::CCanonWorldModel instatus)
{

    bool bNewCmd=false;
    tf::Pose r_goalpose;

    // outcmd contains output, initialize with current command
    outcmd.Set( incmd );

    r_goalpose = Convert<geometry_msgs::Pose, tf::Pose>(incmd.finalpose);
    r_goalpose = _nc->robotRemoveGripperCoord(r_goalpose );

    // There is no base, assumes robot centered at (0,0,0)
    // FK does not include gripper.
    tf::Pose r_lastpose;
    _kinematics->FK(_nc->_status.robotjoints.position, r_lastpose);

    // Kinematic ring calcuation:
    // prerobotxform[i] * finalpose *  postrobotxform[i]
    // base * finalpose * gripper
    tf::Pose w_curpose=_nc->worldCoord(r_lastpose);

    if (incmd.crclcommandnum != _lastcmdnum)
    {
        _lastcmdnum = incmd.crclcommandnum;

        std::vector<tf::Pose> waypoints ;

        // Convert waypoints from message pose to tf pose
        for(size_t i=0; i< incmd.waypoints.size(); i++)
        {
            tf::Pose next_waypoint;
            next_waypoint=Convert<geometry_msgs::Pose, tf::Pose>(incmd.waypoints[i]);
            if(Globals.bWorldCRCLCoordinates)
                next_waypoint = _nc->robotRemoveGripperCoord(next_waypoint );
            else
                next_waypoint = _nc->robotRemoveGripperCoord(next_waypoint );
            waypoints.push_back(next_waypoint);
        }

        tf::Pose r_next_waypoint;
        if(waypoints.size() > 0)
        {
            r_next_waypoint=waypoints[0];
            r_next_waypoint = _nc->robotRemoveGripperCoord(r_next_waypoint );
            // goal pose should be in robot base frame, but need to remove gripper from final pose
            waypoints.push_back(r_goalpose);
        }
        else
        {
            r_goalpose = _nc->robotRemoveGripperCoord(r_goalpose );
            r_next_waypoint=r_goalpose;
        }

        if(0!=_goRobot->InitPose(r_lastpose,
                                 r_next_waypoint,
                                 _nc->linearParams(),
                                 _nc->rotationalParams()))
        {
            logFatal("Go init pose failed\n");
        }

        for(size_t i=1; i<  waypoints.size(); i++)
        {
            _goRobot->AppendPose(waypoints[i]);
        }


        bNewCmd=true;
    }
    tf::Pose r_nextpose ;
    r_nextpose = _goRobot->NextPose();

    // ikfast solution based on 0,0,0 origin not base offset origin
    //incmd.joints.position = Cnc.robotKinematics()->IK(goalpose, incmd.ConfigMin(), incmd.ConfigMax());
    sensor_msgs::JointState r_goaljoints;

    // Set joints to zero for error or unreachable or singularity???
    // FYI so it moves "home" if failed
    r_goaljoints.position.resize(_nc->robotKinematics()->numJoints(), 0.0);
    outcmd.joints.position.resize(_nc->robotKinematics()->numJoints(), 0.0);

    std::vector<double> hint = subset(_nc->_status.robotjoints.position, _nc->robotKinematics()->numJoints());

    outcmd.joints.position=hint; // use hint as seed
    _kinematics->IK(r_nextpose, outcmd.joints.position);
    outcmd.joints.name = _kinematics->jointNames;
    outcmd.crclcommand = CanonCmdType::CANON_MOVE_JOINT;
#if 1
    // Log trajectory details.
    std::string message;

    if (bNewCmd)
        message=Globals.strFormat("%s: GO CANON_MOVE_TO Started\n",_nc->name().c_str());

    if(_goRobot->IsDone())
        message=Globals.strFormat("%s: GO CANON_MOVE_TO Ended\n",_nc->name().c_str());

#ifdef DEBUG
    if (bNewCmd || _goRobot->IsDone())
        LogRobotPosition(
                    message,
                    r_goalpose,
                    r_lastpose,
                    r_nextpose,
                    r_goaljoints.position,
                    outcmd.joints.position,
                    _nc->_status.robotjoints.position);
#endif
#endif

    if (_goRobot->IsDone())
    {
        _lastcmdnum=-1;
        return CanonStatusType::CANON_DONE;
    }
    else
    {
        return CanonStatusType::CANON_WORKING;
    }

}
////////////////////////////////////////////////////////////////////////////////
int CGoInterpreter::parseMovetoCommand(crcl_rosmsgs::CrclCommandMsg &incmd,
                                       RCS::CCanonCmd &outcmd,
                                       RCS::CCanonWorldModel instatus)
{
    bool bNewCmd=false;
    tf::Pose r_goalpose;

    outcmd.Set( incmd );

    // convert from ROS geometry message pose into tf pose
    r_goalpose = Convert<geometry_msgs::Pose, tf::Pose>(incmd.finalpose);

    // goal pose should be in robot base frame, but need to remove gripper from final pose
    r_goalpose = _nc->robotRemoveGripperCoord(r_goalpose );

    // correct pose for minor error
    r_goalpose=r_goalpose * _nc->Correction().inverse();



    // There is no base, assumes robot centered at (0,0,0)
    // FK does not include gripper.
    tf::Pose r_lastpose;
    _kinematics->FK(_nc->_status.robotjoints.position, r_lastpose);


    if (incmd.crclcommandnum != _lastcmdnum)
    {
        _lastcmdnum = incmd.crclcommandnum;



        // Check for speeds - if any set gotraj profile
        std::vector< ::crcl_rosmsgs::CrclMaxProfileMsg> profiles = incmd.profile;
        if(profiles.size() > 0)
        {
            ::crcl_rosmsgs::CrclMaxProfileMsg profile = profiles[0];
            _nc->setLinearSpeeds( profile.maxvel);
        }

        // prim gotraj trajectory generator
        if(0!=_goRobot->InitPose(r_lastpose,
                                 r_goalpose,
                                 _nc->linearParams(),
                                 _nc->rotationalParams()))
        {
            logFatal("go init pose failed goal pose= %s\n", dumpPoseSimple(r_goalpose).c_str());
        }

        bNewCmd=true;
    }
    tf::Pose r_nextpose ;

    r_nextpose = _goRobot->NextPose();

    // ikfast solution based on 0,0,0 origin not base offset origin
    sensor_msgs::JointState r_goaljoints;

    // Set joints to zero (home) for error or unreachable or singularity if not solved
    r_goaljoints.position.resize(_nc->robotKinematics()->numJoints(), 0.0);
    outcmd.joints.position.resize(_nc->robotKinematics()->numJoints(), 0.0);
    outcmd.joints.name = _kinematics->jointNames;
    outcmd.crclcommand = CanonCmdType::CANON_MOVE_JOINT;

    std::vector<double> hint = subset(_nc->_status.robotjoints.position, _nc->robotKinematics()->numJoints());

    // Compute next joints to move to using next pose.
    outcmd.joints.position=hint; // use hint as seed
    if(_kinematics->IK(r_nextpose, outcmd.joints.position))
    {
        logStatus("IK failed next pose= %s\n", dumpPoseSimple(r_nextpose).c_str());
        return CanonStatusType::CANON_ERROR;
    }

#if 1
    // Log trajectory details.
    std::string message;

    if (bNewCmd)
        message=Globals.strFormat("%s: GO CANON_MOVE_TO Started\n",_nc->name().c_str());

    if(_goRobot->IsDone())
        message=Globals.strFormat("%s: GO CANON_MOVE_TO Ended\n",_nc->name().c_str());

#ifdef DEBUG
        LogRobotPosition(
                    message,
                    r_goalpose,
                    r_lastpose,
                    r_nextpose,
                    r_goaljoints.position,
                    outcmd.joints.position,
                    _nc->_status.robotjoints.position);
#endif
#endif

    if (_goRobot->IsDone())
    {
        _lastcmdnum=-1;
        return CanonStatusType::CANON_DONE;
    }
    else
    {
        return CanonStatusType::CANON_WORKING;
    }

}

int  CGoInterpreter::setGripperVelFmax(RCS::CCanonCmd &outcmd, double vel, double fmax )
{
    // set gazebo gripper vel/fmax
    outcmd.gripperControlAlgorithm=CanonControlType::VELOCITY_CONTROL;
    outcmd.nextGripperGoalJoints=_nc->cncGripperJoints();
    outcmd.nextGripperGoalJoints.position.clear();

    size_t n = outcmd.nextGripperGoalJoints.name.size();
    outcmd.nextGripperGoalJoints.effort.resize(n, fmax);
    outcmd.nextGripperGoalJoints.velocity.clear();

    //   outcmd.nextGripperGoalJoints.velocity.resize(n, vel);

    for(size_t j=0; j< n; j++)
    {
        outcmd.nextGripperGoalJoints.velocity.push_back( -1.0 * _nc->cncGripper().multipler(outcmd.nextGripperGoalJoints.name[j])*vel);
    }
    return 0;
}

int   CGoInterpreter::setGripperFoce(RCS::CCanonCmd &outcmd, double force )
{
    // set gazebo gripper vel/fmax
    outcmd.gripperControlAlgorithm=CanonControlType::VELOCITY_CONTROL;
    outcmd.nextGripperGoalJoints=_nc->cncGripperJoints();
    outcmd.nextGripperGoalJoints.position.clear();
    outcmd.nextGripperGoalJoints.velocity.clear();
    outcmd.nextGripperGoalJoints.effort.clear();

    size_t n = outcmd.nextGripperGoalJoints.name.size();
    //outcmd.nextGripperGoalJoints.effort.resize(n, force); /// this did not work right finger went to max position
    for(size_t j=0; j< n; j++)
    {
        outcmd.nextGripperGoalJoints.effort.push_back( -1.0 * _nc->cncGripper().multipler(outcmd.nextGripperGoalJoints.name[j])*force);
    }

    return 0;
}

////////////////////////////////////////////////////////////////////////////////
int CGoInterpreter::parseEEParamGripperCommand(crcl_rosmsgs::CrclCommandMsg &incmd,
                                               RCS::CCanonCmd &outcmd,
                                               RCS::CCanonWorldModel instatus)
{
    // return updated command and status
    outcmd.Set( incmd );

    if (incmd.crclcommandnum != _lastcmdnum)
    {
        // save command id for identification of new command
        _lastcmdnum = incmd.crclcommandnum;

        // Determine action - position and percentage change gripper control
        // action - vel/fmax set gripper parameters
        std::vector<std::string>::iterator iter = std::find(incmd.parameter_names.begin(), incmd.parameter_names.end(), "action");
        size_t index = std::distance(incmd.parameter_names.begin(), iter);
        if(index == incmd.parameter_names.size())
        {
            logFatal("Error: bad parameter names - no leading action\n");
            return CanonStatusType::CANON_ERROR;
        }

        // What is the gripper action?
        std::string action = incmd.parameter_values[index];

        if(boost::iequals(action,"percentage"))
        {
            outcmd.gripperControlAlgorithm=CanonControlType::POSITION_CONTROL;
            _nc->crclGripperAlgorithm()="percentage";
        }
        else if(boost::iequals(action,"position"))
        {
            outcmd.gripperControlAlgorithm=CanonControlType::POSITION_CONTROL;
            _nc->crclGripperAlgorithm()="position";
        }
        else if(boost::iequals(action,"Vel/Fmax"))
        {
            if(incmd.parameter_values.size() < 3 ||
                    incmd.parameter_names.size() < 3 ||
                    !boost::iequals(incmd.parameter_names[1],"vel") ||
                    !boost::iequals(incmd.parameter_names[2],"fmax"))
            {
                fprintf(stderr,"Bad vel/fmax ee parameters");
                return CanonStatusType::CANON_ERROR;
            }


            double vel = Globals.convert<double>(incmd.parameter_values[1]);
            double fmax = Globals.convert<double>(incmd.parameter_values[2]);
            // set gazebo gripper vel/fmax
            outcmd.gripperControlAlgorithm=CanonControlType::VELOCITY_CONTROL;
            _nc->crclGripperAlgorithm()="Vel/Fmax";
            outcmd.nextGripperGoalJoints=_nc->cncGripperJoints();
            outcmd.nextGripperGoalJoints.position.clear();

            size_t n = outcmd.nextGripperGoalJoints.name.size();
            outcmd.nextGripperGoalJoints.effort.resize(n, fmax);
            outcmd.nextGripperGoalJoints.velocity.clear();
            for(size_t j=0; j< n; j++)
            {
                outcmd.nextGripperGoalJoints.velocity.push_back( _nc->cncGripper().multipler(outcmd.nextGripperGoalJoints.name[j])*vel);
            }
            _lastcmdnum=-1;
            return CanonStatusType::CANON_DONE;

        }
        else
        {
            logFatal("Unknown gripper action paramter setting\n");
        }
    }
    _lastcmdnum=-1;
    return CanonStatusType::CANON_DONE;
}
////////////////////////////////////////////////////////////////////////////////
int CGoInterpreter::gripperSpeed(crcl_rosmsgs::CrclCommandMsg &incmd)
{
    if(!Globals.bGripperSpeed)
        return -1;

    // CLose grasping slow down speeds
    if(incmd.eepercent == 0.0)
    {
        _nc->setRobotJointSpeeds(0.05);
        _nc->setLinearSpeeds(0.0125);
        _nc->setRotationalSpeeds(0.05);
        _nc->setGripperJointSpeeds(0.0125);
        return 0;
    }
    // Open grasping regular speeds
    else if(incmd.eepercent == 1.0)
    {
        _nc->setRobotJointSpeeds(1.);
        _nc->setLinearSpeeds(1.);
        _nc->setRotationalSpeeds(.1);
        _nc->setGripperJointSpeeds(.01);
        return 1;
    }
    return -2;
}

////////////////////////////////////////////////////////////////////////////////
int CGoInterpreter::parseGripperCommand(crcl_rosmsgs::CrclCommandMsg &incmd,
                                        RCS::CCanonCmd &outcmd,
                                        RCS::CCanonWorldModel instatus)
{
    // return updated command and status
    outcmd.Set( incmd );

    // Copy current joint rates suitable for gotraj
    std::vector<gomotion::GoTrajParams> jparams = _nc->gripperJointRateParams();

    if (incmd.crclcommandnum != _lastcmdnum)
    {
        // save command id for identification of new command
        _lastcmdnum = incmd.crclcommandnum;

        if (incmd.eepercent == 0.0)
        {

            // set speeds based on closed grasping
            gripperSpeed(incmd);

            // fetch latest joint rate parameters.
            jparams = _nc->gripperJointRateParams();

            // FIXME: use convoluted close
            _nc->gripper_goal_joints()=_nc->cncGripper().close();

            // prime gotraj trajectory generator
            _goGripper->InitJoints(_nc->cncGripperJoints(),  // now
                                   _nc->gripper_goal_joints(),
                                   jparams);
        }
        else if (incmd.eepercent == 1.0)
        {
            _nc->bGrasping()=false;

            // set speeds based on open grasping
            gripperSpeed(incmd);

            // fetch latest joint rate parameters.
            jparams = _nc->gripperJointRateParams();

            _nc->gripper_goal_joints()=_nc->cncGripper().open();
            _goGripper->InitJoints(_nc->cncGripperJoints(),  // now
                                   _nc->gripper_goal_joints(),
                                   jparams);
        }
        else
        {
            // fixme: should interpolate between gripper positions
            if(boost::iequals(_nc->crclGripperAlgorithm(),"percentage"))
            {
                _nc->gripper_goal_joints()=_nc->cncGripper().set_position(incmd.eepercent);
                _goGripper->InitJoints(_nc->cncGripperJoints(),  // now
                                       _nc->gripper_goal_joints(),
                                       jparams);
            }
            else
            {
                _nc->gripper_goal_joints()=_nc->cncGripper().set_abs_position(incmd.eepercent);
                _goGripper->InitJoints(_nc->cncGripperJoints(),  // now
                                       _nc->gripper_goal_joints(),
                                       jparams);
            }
        }
    }


    // Does prismatic matter in gotraj
    int ret= CanonStatusType::CANON_DONE;
    // check if go is done
    if (_goGripper->IsDone())
    {
        _lastcmdnum=-1;
        ret= CanonStatusType::CANON_DONE;
        outcmd.nextGripperGoalJoints=_nc->gripper_goal_joints();
        outcmd.nextGripperGoalJoints.name= _nc->cncGripperJoints().name;
    }
    else
    {
        ret= CanonStatusType::CANON_WORKING;
        outcmd.nextGripperGoalJoints=  _goGripper->NextJoints();
        outcmd.nextGripperGoalJoints.name= _nc->cncGripperJoints().name;
    }

    return ret;

}

////////////////////////////////////////////////////////////////////////////////
int CGoInterpreter::parseStopCommand(crcl_rosmsgs::CrclCommandMsg &incmd,
                                     RCS::CCanonCmd &outcmd,
                                     RCS::CCanonWorldModel instatus) {

    // FIXME: save last command type, then do until done
    if (incmd.crclcommandnum != _lastcmdnum) {
        _lastcmdnum = incmd.crclcommandnum;
        _goRobot->InitStop();
    }
    // FIXME: should update robot motion until go motion queue is empty
    return CanonStatusType::CANON_STOP;

}

////////////////////////////////////////////////////////////////////////////////
int CGoInterpreter::parseCommand(crcl_rosmsgs::CrclCommandMsg &incmd,
                                 RCS::CCanonCmd &outcmd,
                                 RCS::CCanonWorldModel instatus)
{
    try
    {
        outcmd.Set(incmd);
        if (incmd.crclcommand == CanonCmdType::CANON_INIT_CANON)
        {
            // Fixme: reset any other items?
            // Should we move home too?
            _lastcmdnum=-1;
            return CanonStatusType::CANON_DONE;
        }
        else if (incmd.crclcommand == CanonCmdType::CANON_END_CANON)
        {
            _lastcmdnum=-1;
            return CanonStatusType::CANON_DONE;
        }

        else if (incmd.crclcommand == CanonCmdType::CANON_MOVE_JOINT)
        {
            return parseJointCommand(incmd, outcmd, instatus);
        }
        else if (incmd.crclcommand == CanonCmdType::CANON_MOVE_TO)
        {
            return parseMovetoCommand(incmd, outcmd, instatus);
        }
        else if (incmd.crclcommand == CanonCmdType::CANON_STOP_MOTION)
        {
            _lastcmdnum=-1;
            return CanonStatusType::CANON_DONE;
            // FIXME: test whether this actually works. Most "stops"
            // are just commands to 0 vel in aprs
            //return parseStopCommand(incmd, outcmd, instatus);
        }
        else if (incmd.crclcommand == CanonCmdType::CANON_MOVE_THRU)
        {
            return parseMoveThruCommand(incmd, outcmd, instatus);
        }

        else if (incmd.crclcommand == CanonCmdType::CANON_PAVEL_GRIPPER
                 || incmd.crclcommand == CanonCmdType::CANON_SET_GRIPPER)
        {
            return parsePavelGripperCommand(incmd, outcmd, instatus);
        }
        else if (incmd.crclcommand == CanonCmdType::CANON_DWELL )
        {
            incmd.dwell_seconds -= ((double) _nc->cycleTime());
            if(incmd.dwell_seconds<=0.0)
            {
                _lastcmdnum=-1;
                return CanonStatusType::CANON_DONE;
            }
            return CanonStatusType::CANON_WORKING;
        }
        else if (incmd.crclcommand == CanonCmdType::CANON_SET_GRIPPER)
        {
            return parsePavelGripperCommand(incmd, outcmd, instatus);
            // return parseGripperCommand(incmd, outcmd, instatus);
        }
        else if (incmd.crclcommand == CanonCmdType::CANON_SET_EE_PARAMETERS)
        {
            parseEEParamGripperCommand(incmd, outcmd, instatus);
            return CanonStatusType::CANON_DONE;
        }

        else // if(incmd.crclcommand == CanonCmdType::CANON_NOOP)
        {
            // This causes a cycle between every CRCL command even if not motion.
            // not seeming to catch all - such as STOP  so catch all for now
            _lastcmdnum=-1;
            return CanonStatusType::CANON_DONE;
        }

    }
    catch (RobotControlException & e)
    {
        LOG_DEBUG << "Exception in  GoInterpreter::ParseCommand() thread: " << e.what() << "\n";
        outcmd.crclcommand = CanonCmdType::CANON_STOP_MOTION;
        outcmd.opmessage = e.what();
        outcmd.stoptype = CanonStopMotionType::NORMAL;
        return CanonStatusType::CANON_ERROR;
    }

    // Eventually should never get here
    return CanonStatusType::CANON_ERROR;
}


////////////////////////////////////////////////////////////////////////////////
int CGoInterpreter::parsePavelGripperCommand(crcl_rosmsgs::CrclCommandMsg &incmd,
                                             RCS::CCanonCmd &outcmd,
                                             RCS::CCanonWorldModel instatus)
{
    // return updated command and status
    outcmd.Set( incmd );

    if (incmd.crclcommandnum != _lastcmdnum)
    {
        // save command id for identification of new command
        _lastcmdnum = incmd.crclcommandnum;

        if (incmd.eepercent == 0.0)
        {
            outcmd.eepercent=incmd.eepercent;
            _nc->bGrasping()=true;
            _nc->gripper_goal_joints()=_nc->cncGripper().close();

        }
        else if (incmd.eepercent == 1.0)
        {
            _nc->bGrasping()=false;
            outcmd.eepercent=incmd.eepercent;
            _nc->gripper_goal_joints()=_nc->cncGripper().open();
        }
    }

    // Does prismatic matter in gotraj
    int ret=  CanonStatusType::CANON_DONE;
    outcmd.nextGripperGoalJoints=_nc->gripper_goal_joints();
    outcmd.nextGripperGoalJoints.name= _nc->cncGripperJoints().name;

    if(_nc->cncGripper().isGrasping()==_nc->bGrasping())
    {
        _lastcmdnum=-1;
        ret = CanonStatusType::CANON_DONE;
    }
    else
        ret = CanonStatusType::CANON_WORKING;

    // FIXME: hack for now - as closing on nothing will be in continuous loop....
    _lastcmdnum=-1;
    return CanonStatusType::CANON_DONE;
    // return ret;

}
