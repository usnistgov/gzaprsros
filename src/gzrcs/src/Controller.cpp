// Controller.cpp

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


#include <sstream>
#include <iostream>
#include <csignal>
#include <mutex>
#include <chrono>

#define BOOST_ALL_NO_LIB

#include <boost/exception/all.hpp>
#include <boost/thread.hpp>

#include "gzrcs/Controller.h"
#include "gzrcs/RobotControlException.h"
#include "gzrcs/cros.h"
#include "aprs_headers/Debug.h"

#define GLOGGER GLogger
#include "aprs_headers/LoggerMacros.h"

#ifdef CRCL_DLL
#include <crcllib/nistcrcl.h>
#endif



// RCS namespace declarations
namespace RCS {

std::vector<std::shared_ptr<CController> > ncs;

Nist::Config robotconfig;

std::mutex cncmutex;


// ---------------------------------------------------
// CController

CController::CController(std::string name, double cycletime) :
    _robotName(name)
  , RCS::Thread(cycletime, name)
{
    //IfDebug(LOG_DEBUG << "CController::CController"); // not initialized until after main :()
    bGrasping() = false;
    currentRobotJointSpeed()=1.0;
    currentRobotJointSpeed()=1.0;
    last_crcl_command_num=0;  // can't be negative
    accelerationMultipler()=10.0;
    state=NORMAL;
    bInited=false;

}

////////////////////////////////////////////////////////////////////////////////
CController::~CController(void)
{

}

////////////////////////////////////////////////////////////////////////////////
bool CController::verify()
{
    if(robotKinematics() == NULL)
    {
        logFatal("Controller has no kinematic element\n");
        return false;
    }
    if(robotInterpreter() == NULL)
    {
        logFatal("Controller has no robotInterpreter element\n");
        return false;
    }
    return true;
}

////////////////////////////////////////////////////////////////////////////////
void CController::setup()
{
    std::string prefix=robotPrefix();

    // Save thread name
    name() = prefix + "controller";
    crclLastUpdateTime() = RCS::Timer::etime();

    ////////////////////////////////////////////////////////////////////////////////
    /// Gripper
    ////////////////////////////////////////////////////////////////////////////////
    // Initialize gripper and gripper joint values
    // FIXME: read state, open loop currently

    std::string errmsg = cncGripper().init(robotPrefix());
    if(!errmsg.empty())
        std::cout << "Error configuring cncGripper and Gazebo plugin" << errmsg << "\n";

//////////////////////////////////////////////////////////////////////////////
    /// Gazebo connections to model reader and joint updates
    ////////////////////////////////////////////////////////////////////////////////
#ifdef GAZEBO
    // Set up and start parts reading from gazebo world model
    // setup and start joint updater publishing to gazebo
    // @fixme only one instance of gz instance model is required.
    if(Globals.bGazebo)
    {
        std::string errmsg;

//        if(Globals.bGearLocations)
//        {
//            errmsg=gzInstances().init();
//            if(!errmsg.empty())
//                std::cout << "Error configuring Gazebo Model plugin" << errmsg << "\n";

//            gzInstances().start();
//        }

        errmsg = _writer.init(this->robotPrefix());
        if(!errmsg.empty())
        {
            std::cout << "Error configuring Gazebo Robot plugin" << errmsg << "\n";
        }

        // supply gazebo topic writer with names of joints
        std::vector<std::string> robotJointNames;
        std::vector<std::string> gripperJointNames;
        robotJointNames.insert(robotJointNames.end(), this->_status.robotjoints.name.begin(),
                     this->_status.robotjoints.name.end());
#ifdef GRIPPER
        gripperJointNames.insert(gripperJointNames.end(), this->cncGripperJoints().name.begin(),
                     this->cncGripperJoints().name.end());
#endif

        // Advertise gz topic  for connections
        _writer.start(robotJointNames,gripperJointNames);
        _cncGripper.start();
    }


#endif

    ////////////////////////////////////////////////////////////////////////////////
    /// ROS
    ////////////////////////////////////////////////////////////////////////////////
#if defined(ROS) && defined(CRCL_ROS_TOPIC)
    // Create ros topic communication handler - read commands, report status
    // Method Start() initiates communication. If any.
    // Possibly make topic names part of ini file.
    {
        pRosCrcl()= std::shared_ptr<CRosCrclRobotHandler>(new CRosCrclRobotHandler() );
        pRosCrcl()->Init(std::shared_ptr<RCS::CController>(this), prefix);
        pRosCrcl()->Start();
    }
#endif

    ////////////////////////////////////////////////////////////////////////////////
    /// CRCL DLL
    ////////////////////////////////////////////////////////////////////////////////
#ifdef CRCL_DLL
    {
        pCrclServer()=boost::shared_ptr<crcl::crclServer>( new crcl::crclServer(
                                                            crclIp(),
                                                            crclPort(),
                                                            0.1, // dummy - no longer thread
                                                            robotKinematics()->urdfXml(),
                                                            robotKinematics()->baseLink(),
                                                            robotKinematics()->endLink() ));

        pCrclServer()->setCmdQueue(&crclcmds);
        if(Globals.DEBUG_LogRobotCrcl())
            pCrclServer()->setDebugStream(&ofsRobotCrcl);
        pCrclServer()->start();
    }
#endif

    // RCS Postprocess Publish current status to crcl, gzebo, robot
    // REMOVED Thu 05 Sep 2019 03:06:48 PM EDT
 //   this->addPostprocessFcn(std::bind(&CController::publishStatus,this));

    ////////////////////////////////////////////////////////////////////////////////
    /// motion rates - vel, acc, jerk
    ////////////////////////////////////////////////////////////////////////////////
    setRobotJointSpeeds(1.0);
    setGripperJointSpeeds(1.0);
    setLinearSpeeds(1.0);
    setRotationalSpeeds(.1);

    ////////////////////////////////////////////////////////////////////////////////
    /// Status
    ////////////////////////////////////////////////////////////////////////////////
    // Initialze status - open loop for now
    _status.init();

    std::cout << "Waiting feedback from robot ";
    while(!_writer.readCycles())
    {
        Globals.sleep(250);
        std::cout << "." << std::flush;
    }
    std::cout << "\n" << std::flush;
    // Initialize robot and robot joint values - simulation
    // assume zero for now. Fixme: read joint state and set this value
    _status.robotjoints = _writer.robotStatus(); // RCS::zeroJointState(robotKinematics()->jointNames().size());
//    _status.robotjoints.name = robotKinematics()->jointNames();
//    _status.robotjoints.position.resize(_status.robotjoints.name.size(), 0.0);

    // this prevents the robot from outputting garbage.
#ifdef FIXME
    _nextcc.joints=_writer.robotStatus();
#endif

#if 0
    _status._eepercent=0.0;
    gripperjoints = gripper.setPosition(status.eepercent);
#else
    _status._eepercent=1.0;
    cncGripperJoints()=cncGripper().open();
#endif

    bInited=true;
}
////////////////////////////////////////////////////////////////////////////////
void CController::cleanup ( )
{
    pCrclServer()->stop();

//    // These are NOT threads, but ROS or Gazebo subscriptions
//    if(Globals.bGearLocations)
//        gzInstances().stop();

}

////////////////////////////////////////////////////////////////////////////////
void CController::publishStatus()
{

    // dont command robot or report status initialized.
    if(!bInited)
        return;

    if(Globals.bGazebo)
    {
        // FIXME: this should be mutexed except alreaady in mutex?
        // this will contain status joint values
        sensor_msgs::JointState robotJoints, gripperJoints;

        // get current status of joint positions
//        robotJointNames.position.insert(std::end(joints.position), std::begin(this->status.robotjoints.position),
//                               std::end(this->status.robotjoints.position));
        robotJoints=this->_status.robotjoints;

#ifdef GRIPPER
        // if gripper, get current status of gripper joint positions
        // should have names, but will assume gazebo writer has them.
        gripperJoints=this->cncGripperJoints();
#endif

        // update gazebo writer
        using namespace std::chrono;
        milliseconds ms = duration_cast< milliseconds >(
                    system_clock::now().time_since_epoch()
                    );
        _writer.updateRobot(ms.count(), robotJoints);
        if(Globals.bPavelGripperPlugin )
        {
            if (_status._eepercent>=0.0)
                _cncGripper.updateGripper(ms.count(),  _status._eepercent);
        }
        else
        {
            _writer.updateGripper(ms.count(), gripperJoints);
        }


    }
#if defined(CRCL_ROS_TOPIC)
    RCS_Time current_time = RCS::Timer::etime();

    double seconds_since_last_update = RCS::ToSeconds( current_time - crclLastUpdateTime() );

    if ( seconds_since_last_update < crclPublishStatusRate() )
    {
        return;
    }
    crclLastUpdateTime() = current_time;
#endif
    boost::shared_ptr<crcl_rosmsgs::CrclStatusMsg> statusmsg;
    statusmsg=boost::shared_ptr<crcl_rosmsgs::CrclStatusMsg>(new crcl_rosmsgs::CrclStatusMsg());
    statusmsg->crclcommandstatus = _status.crclCommandStatus;
    statusmsg->crclcommandnum = _status.echocmd.crclcommandnum;
    statusmsg->crclstatusnum = _status.echocmd.crclcommandnum;
    _status.currentpose = robotKinematics()->FK(_status.robotjoints.position); /**<  current robot pose */
    // add in gripper to get real location in robot space
    _status.currentpose = robotAddGripperCoord(_status.currentpose);
    statusmsg->statuspose = Convert<tf::Pose, geometry_msgs::Pose>(_status.currentpose);
    statusmsg->statusjoints = _status.robotjoints;
    statusmsg->eepercent = _status._eepercent;

    // Status is updated to CRCL handler by either ROS topic or DLL call
#if defined(CRCL_ROS_TOPIC)
    if(Globals.bRos && pRosCrcl().get() != NULL)
        pRosCrcl()->PublishCrclStatus(statusmsg);
#endif
#ifdef CRCL_DLL
    pCrclServer()->statusUpdate(statusmsg);
#endif
}

////////////////////////////////////////////////////////////////////////////////
bool CController::isBusy()
{
    // FIXME this should be based on mutex? or mutex flag
    return (crclcmds.size() > 0 || _status.crclCommandStatus != CanonStatusType::CANON_DONE );
}

tf::Pose CController::currentPose()
{
    return robotKinematics()->FK(_status.robotjoints.position); /**<  current robot pose */
}

////////////////////////////////////////////////////////////////////////////////
sensor_msgs::JointState  CController::where()
{
    sensor_msgs::JointState joints;
    joints.name.insert(joints.name.end(), this->_status.robotjoints.name.begin(),
                       this->_status.robotjoints.name.end());

    joints.name.insert(joints.name.end(), this->cncGripperJoints().name.begin(),
                       this->cncGripperJoints().name.end());

    joints.position.insert(std::end(joints.position), std::begin(this->_status.robotjoints.position),
                           std::end(this->_status.robotjoints.position));
    joints.position.insert(std::end(joints.position),std::begin( this->cncGripperJoints().position),
                           std::end(this->cncGripperJoints().position));
    return joints;
}


////////////////////////////////////////////////////////////////////////////////
bool CController::updateRobot()
{

    // dont command robot or report status initialized.
    if(!bInited)
        return false;

#if 0
    // Save last joints to differentiate values of joint velocities status
    sensor_msgs::JointState & lastjoints(laststatus.robotjoints);
    lastjoints = status.robotjoints;
    if (lastjoints.velocity.size() == 0)
        lastjoints.velocity.resize(lastjoints.position.size(), 0.0);
    if (lastjoints.effort.size() == 0)
        lastjoints.effort.resize(lastjoints.position.size(), 0.0);
#endif

    // Update robot position based on output from RCSInterpreter
    // Note crcl command may have not update of position (e.g., dwell).
    // FIXME: /?? only update if position changed?
    if (RCS::hasMotion(_nextcc.joints))
    {
        _status.robotControlAlgorithm = _nextcc.robotControlAlgorithm;
        _status.robotjoints = _nextcc.joints;   // <<< THIS IS WHERE POSITION UPDATE HAPPENS - should do position or velocity
    }
    // make sure joints have names
    _status.robotjoints.name = robotKinematics()->jointNames();



    // Update status based on current joint positions and last joint positions
    // NOT SURE WANT TO BLANK OUT VEL EFFORT IF NOT REQUIRED
#if 0
    for (size_t k = 0; k < lastjoints.position.size(); k++) {
        if (status.robotjoints.velocity.size() <= k)
            status.robotjoints.velocity.push_back(0.0);
        status.robotjoints.velocity[k] = (fabs(status.robotjoints.position[k]) + fabs(lastjoints.position[k])) / 2.0;
        if (status.robotjoints.effort.size() <= k)
            status.robotjoints.effort.push_back(0.0);
        status.robotjoints.effort[k] = (fabs(status.robotjoints.velocity[k]) + fabs(lastjoints.velocity[k])) / 2.0;
    }
#endif

    // Give pose of robot no gripper or base offsets included
    tf::Pose & lastpose(_laststatus.currentpose);
    lastpose = _status.currentpose;
    _status.currentpose = robotKinematics()->FK(_status.robotjoints.position); /**<  current robot pose */

#if 0
    // Fixme: this does not work
    std::vector<int> outofbounds;
    std::string msg;
    if (robotKinematics()->CheckJointPositionLimits(status.robotjoints.position, outofbounds, msg)) {
        throw RobotControlException(last_crcl_command_num1000, msg.c_str());
    }
#endif

    // Update gripper position based on output from RCSInterpreter
    // Note crcl command may have not update of gripper.
    //if(_nextcc.next_gripper_goal_joints.position.size() > 0 )
 //   if(RCS::hasMotion(_nextcc.nextGripperGoalJoints))
    if(_nextcc.crclcommand == CanonCmdType::CANON_PAVEL_GRIPPER ||
       _nextcc.crclcommand == CanonCmdType::CANON_SET_GRIPPER     )
    {
        cncGripperJoints()=_nextcc.nextGripperGoalJoints;
        _status._eepercent = _nextcc.eepercent;

        // assign latest control algorithm for gripper fingers
        _status.gripperControlAlgorithm = _nextcc.robotControlAlgorithm;
    }

    {
        static tf::Pose lastpose=tf::Identity();
        if(!(lastpose == _status.currentpose) )
        {
            if(Globals.DEBUG_Log_Cyclic_Robot_Position())
            {
                ofsMotionTrace << name().c_str() << " UPDATED ROBOT\n";
                ofsMotionTrace << "  Robot Pose    =" << RCS::dumpPoseSimple(_status.currentpose).c_str() << "\n";
                ofsMotionTrace << "  Goal Joints   =" << vectorDump<double>(_nextcc.joints.position).c_str() << "\n" << std::flush;
            }

            if(Globals.DEBUG_GnuPlot())
            {
                static int cycle_num=1;
                ofsGnuPlotJnt << cycle_num++ << " "  << vectorDump<double>(_status.robotjoints.position, " ") << " " <<
                              vectorDump<double>(cncGripperJoints().position, " ") << "\n" << std::flush;
                ofsGnuPlotCart << _status.currentpose.getOrigin().x() << " " << _status.currentpose.getOrigin().y() << " " <<
                                  _status.currentpose.getOrigin().z() << "\n" << std::flush;
            }
        }
        lastpose = _status.currentpose;
    }

    this->publishStatus();
    return true;
}

////////////////////////////////////////////////////////////////////////////////
int CController::action() {
    try {

        // FIXME Muddling of crcl and joint cmd, crcl and joint status
        bool bNoCommands=true;
        int n=0;  // flywheel do loop until no more commands.
        for (size_t j=0; crclcmds.size()>n; j++)
        {
            bNoCommands=false;
            // Translate into Cnc.cmds
            // FIXME: this is an upcast
            //RCS::CanonCmd nextcc;
            _laststatus = _status;
            crcl_rosmsgs::CrclCommandMsg msg = crclcmds.peek();
nextposition:

            // If new command, log
            if(msg.crclcommandnum != last_crcl_command_num)
            {
                std::unique_lock<std::mutex> lock(cncmutex);
                RCS::CCanonCmd cc;
                cc.Set(msg);

                // write to "robot"_nc_crcl.log
                if(Globals.DEBUG_LogRobotCrcl())
                    ofsRobotCrcl << Logging::CLogger::strTimestamp() << " " << cc.toString() << "\n" << std::flush;
            }
            // Save command number for comparison next time
            last_crcl_command_num=msg.crclcommandnum;


            // Change: Preprocess gripper to pavel gripping plugin
            if(Globals.bPavelGripperPlugin && msg.crclcommand ==CanonCmdType::CANON_SET_GRIPPER)
            {
                std::unique_lock<std::mutex> lock(cncmutex);
                msg.crclcommand = CanonCmdType::CANON_PAVEL_GRIPPER;
            }

            int cmdstatus = robotInterpreter()->parseCommand(msg, _nextcc, _laststatus);

            // replace existing front of crclcmd message queue with updated message
            crclcmds.poke(msg);

            if (cmdstatus == CanonStatusType::CANON_WORKING)
            {
                _nextcc.status = CanonStatusType::CANON_WORKING;

                // update status
                _status.echocmd = _nextcc;
                _status.crclCommandStatus = CanonStatusType::CANON_WORKING;

            }
            // Signals done with canon command
            else if (cmdstatus == CanonStatusType::CANON_DONE)
            {
                _nextcc.status = CanonStatusType::CANON_DONE;
                _lastcc.status = CanonStatusType::CANON_DONE;
                _status.crclCommandStatus = CanonStatusType::CANON_DONE;
                if(crclcmds.size()> 0)
                    crclcmds.pop();
            }
            else if (cmdstatus == CanonStatusType::CANON_STOP)
            {
                // assume last command was world, joint or ujoint motion
                // stop inserted in front of queue. We are done. Messy? Yes.
                crclcmds.pop(); // pop stop command
                if (crclcmds.size() > 0) {
                    msg = crclcmds.peek();
                    crclcmds.clear();

                    // if motion command need to stop motion gracefully
                    // need to enable stop!
                    if (msg.crclcommand == CanonCmdType::CANON_MOVE_JOINT ||
                            msg.crclcommand == CanonCmdType::CANON_MOVE_TO) {
                        crclcmds.addMsgQueue(msg);
                        goto nextposition;
                    }
                }
            }
            else if (cmdstatus == CanonStatusType::CANON_ERROR)
            {
                crclcmds.clear();
                _nextcc.status = CanonStatusType::CANON_ERROR;

                // update status
                _status.echocmd = _nextcc;
                _status.crclCommandStatus = CanonStatusType::CANON_ERROR;
            }
            else
            {
                LOG_DEBUG << "Command not handled";
            }

            updateRobot();
            _lastcc = _nextcc;

            // Now, we only do one command at a time
            if(!Globals.bFlywheel)
                n=crclcmds.size();


        }

        if(bNoCommands)
        {
            updateRobot();
            _lastcc = _nextcc;
        }

    }
    catch (RobotControlException & e)
    {
        std::cerr << "Exception in  CController::Action() thread: " << e.what() << "\n";
        std::raise(SIGINT);
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

////////////////////////////////////////////////////////////////////////////////
void CController::dumpRobotNC(std::shared_ptr<CController> nc)
{

    ofsRobotURDF << "============================================================\n";
    ofsRobotURDF << "NC " << nc->name().c_str() << "\n";
    ofsRobotURDF << "base link " << nc->robotKinematics()->baseLink().c_str() << "\n";
    ofsRobotURDF << "ee link " << nc->robotKinematics()->endLink().c_str() << "\n";
    ofsRobotURDF << "num joints " << nc->robotKinematics()->numJoints() << "\n";
    ofsRobotURDF << "baseoffset " << RCS::dumpPoseSimple(nc->basePose()).c_str() << "\n";
    ofsRobotURDF << "tooloffset " << RCS::dumpPoseSimple(nc->gripperPose()).c_str() << "\n";
    ofsRobotURDF << "Joint names " << vectorDump<std::string>(nc->robotKinematics()->jointNames()).c_str() << "\n" << std::flush;

    for (std::map<std::string, std::vector<double>>::iterator it = nc->namedJointMove().begin(); it != nc->namedJointMove().end(); it++)
        ofsRobotURDF << (*it).first << "=" << vectorDump<double>(nc->namedJointMove()[(*it).first]).c_str() << "\n";
    ofsRobotURDF << "Cycletime   " << nc->cycleTime() << "\n";

    ofsRobotURDF << std::flush;
}


////////////////////////////////////////////////////////////////////////////////
void CController::setSpeeds(double speed)
{
    setRobotJointSpeeds(speed);
    setLinearSpeeds(speed);
    setRotationalSpeeds(speed);
    setGripperJointSpeeds(speed);

}
////////////////////////////////////////////////////////////////////////////////
void CController::slowSpeeds()
{
    setRobotJointSpeeds(0.1);
    setLinearSpeeds(0.1);
    setRotationalSpeeds(0.1);
    setGripperJointSpeeds(0.1);

}

////////////////////////////////////////////////////////////////////////////////
void CController::mediumSpeeds()
{
    setRobotJointSpeeds(1.0);
    setLinearSpeeds(1.0);
    setRotationalSpeeds(1.0);
    setGripperJointSpeeds(1.0);
}

////////////////////////////////////////////////////////////////////////////////
void CController::fastSpeeds()
{
    setRobotJointSpeeds(10.0);
    setLinearSpeeds(10.0);
    setRotationalSpeeds(10.0);
    setGripperJointSpeeds(10.0);
}

////////////////////////////////////////////////////////////////////////////////
double CController::setRobotJointSpeeds(double speed)
{
    double now_speed = currentRobotJointSpeed();
    currentRobotJointSpeed()=speed;

    // set speeds as multiple of baseline
    robotJointRateParams()= std::vector<gomotion::GoTrajParams>   (robotKinematics()->numJoints(),
                                                              gomotion::GoTrajParams(1.0 * speed,
                                                                                     10.0 * speed,
                                                                                     100.0 * speed
                                                                                     )
                                                              );
    currentGripperJointSpeed()=speed;
    gripperJointRateParams()=std::vector<gomotion::GoTrajParams>   (cncGripperJoints().name.size(),
                                                                     gomotion::GoTrajParams(1.0 * speed,
                                                                                            10.0 * speed,
                                                                                            100.0 * speed
                                                                                            )
                                                                     );
    return now_speed;
}
////////////////////////////////////////////////////////////////////////////////
double CController::setGripperJointSpeeds(double speed)
{
    double now_speed = currentGripperJointSpeed();
    currentGripperJointSpeed()=speed;

    gripperJointRateParams()=std::vector<gomotion::GoTrajParams>   (cncGripperJoints().name.size(),
                                                                     gomotion::GoTrajParams(1.0 * speed,
                                                                                            10.0 * speed,
                                                                                            100.0 * speed
                                                                                            )
                                                                     );
    return now_speed;
}
////////////////////////////////////////////////////////////////////////////////
double CController::setLinearSpeeds(double speed)
{
    double now_speed = currentLinearSpeed();
    currentLinearSpeed()=speed;
    // FIXME: check not exceeding ini file max linear speed

    // set speeds as multiple of baseline
    linearParams()=gomotion::GoTrajParams(speed, accelerationMultipler()*speed, 10.0*accelerationMultipler()*speed);
    return now_speed;
}
////////////////////////////////////////////////////////////////////////////////
double CController::setRotationalSpeeds(double speed)
{
    double now_speed = currentAngularSpeed();
    currentAngularSpeed()=speed;

    // set speeds as multiple of baseline
    rotationalParams()=gomotion::GoTrajParams(speed, accelerationMultipler()*speed, accelerationMultipler()*10.0*speed);
    return now_speed;
}
////////////////////////////////////////////////////////////////////////////////
void CController::updateJointRates(std::vector<uint64_t> jointnums,
                                     sensor_msgs::JointState newjoints)
{
    // Check each joint, to see if joint is being actuated, if so, change velocity
    // if defined
    for (size_t i = 0; i < jointnums.size(); i++) {
        size_t n = jointnums[i]; // should already have indexes -1;
        if(newjoints.velocity.size()>i &&  newjoints.velocity[i]>0.0)
        {
            robotJointRateParams()[n].vel = newjoints.velocity[i];
            robotJointRateParams()[n].acc = newjoints.velocity[i]*10.;
            robotJointRateParams()[n].jerk = newjoints.velocity[i]*100.;
        }
        if(newjoints.effort.size()>i &&  newjoints.effort[i]>0.0)
            robotJointRateParams()[n].acc = newjoints.effort[i];
    }
}
}


