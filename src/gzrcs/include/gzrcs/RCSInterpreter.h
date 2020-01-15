#ifndef _RCSInterpreter_h
#define _RCSInterpreter_h

//
// RCSInterface.h
//

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

#include <vector>
#include <memory>

// This is a ROS message definition
#include <geometry_msgs/Point.h>

#include "aprs_headers/Core.h"
#include "aprs_headers/IRcs.h"
#include "aprs_headers/Debug.h"
#include "aprs_headers/IKinematic.h"

#include "gotraj/gotraj.h"


#include "gzrcs/Controller.h"


using namespace gomotion;

namespace RCS {



/**
 * @brief The CGoInterpreter class uses gotraj from the gomotion https://github.com/frederickproctor/gomotion
 * github repository. THe control system presents a ROS tf data representation to the gotraj (which converts
 * to gomotion representation behind the scenes. gotraj uses posiiton, vel, acceleration and jerk to determine
 * trajectory motion waypoints. gotraj stops at every goal point and waypoint. gotraj requires a cycle time to
 * determine how to plan the path. gotraj requires rate parameters (vel, acceleration, jerk) to calculate the motion.
 */
class CGoInterpreter : public IRCSInterpreter {

public:
    /**
     * @brief CGoInterpreter initialize variables, and creates gotraj motion planner
     * for robot and gripper.
     * @param nc pointer to robot controller
     * @param k pointer to robot kinematics handler
     */
    CGoInterpreter(std::shared_ptr<CController> nc,
                  boost::shared_ptr<IKinematic> k);
    /**
     * @brief init intialize gotraj for robot and gripper with cycle time and initial joint values.
     * @param jnts supplied robot joints values. FIXME: Uses nc gripper joint values to initialize gotraj.
     */
    void init(std::vector<double> jnts);

    /**
     * @brief parseCommand top level parsing command that distributes CRCL motion commands.
     * @param incmd crcl command
     * @param outcmd ROS
     * @param instatus
     * @return
     */
    virtual int parseCommand(crcl_rosmsgs::CrclCommandMsg &incmd,
                             CCanonCmd& outcmd,
                             CCanonWorldModel instatus);

    /**
     * @brief parseJointCommand accepts a CRCL joint command and translates individual joint updates into
     * a robot all joints with names command.
     * Uses gotraj to generate a smooth joint trajectory  from current joint position (given by instatus) to goal
     * joint position (as given by incmd).
     * @param incmd - crcl style robot joint command containing joint numbers assocatied new positions
     * @param outcmd - ros style all robot  joints command containing names, positions, velocities and effort.
     * @param instatus
     * @return CRCL done or working
     */
    virtual int parseJointCommand(crcl_rosmsgs::CrclCommandMsg &incmd,
                                  CCanonCmd &outcmd,
                                  CCanonWorldModel instatus);
    /**
     * @brief parseMovetoCommand given a  CRCL move Cartesian command, use gotraj to generate a smooth trajectory
     * from the current position as given by instatus to the current position and crcl to determine destination position.
     * This command is called repeatedly until the gotraj queue for this command is empty, upon which a
     * @param incmd - Cartesian pose goal
     * @param outcmd - next joints values
     * @param instatus - status of the robot
     * @return CRCL done or working
     */
    virtual int parseMovetoCommand(crcl_rosmsgs::CrclCommandMsg &incmd,
                                  CCanonCmd &outcmd,
                                  CCanonWorldModel instatus);
    /**
     * @brief parseStopCommand accepts a CRCL stop command and coordinates stopping.
     * @param incmd crcl command
     * @param outcmd rcs handling of stop.
     * @param instatus current robot and gripper wm and status.
     * @return CRCL stop
     */
    virtual int parseStopCommand(crcl_rosmsgs::CrclCommandMsg &incmd,
                                 CCanonCmd &outcmd,
                                 CCanonWorldModel instatus);
    /**
     * @brief parseGripperCommand parse CRCL open/close/percentage command.
     * @param incmd crcl
     * @param outcmd rcs handling of gripper motion using gotraj to plan trajectory.
     * @param instatus current robot and gripper wm and status.
     * @return CRCL done or working
     */
    virtual int parseGripperCommand(crcl_rosmsgs::CrclCommandMsg &incmd,
                                    CCanonCmd &outcmd,
                                    CCanonWorldModel instatus) ;

    /**
      * @brief parsePavelGripperCommand pavel's gazebo plugin force-based open/close gripping
     * @param incmd crcl  gripper command
     * @param outcmd ros sensor message style gripper joint updates
     * @param instatus current robot and gripper wm and status.
     * @return CRCL done or working
     */
     virtual int parsePavelGripperCommand(crcl_rosmsgs::CrclCommandMsg &incmd,
                                                       CCanonCmd &outcmd,
                                                       CCanonWorldModel instatus);
    /**
     * @brief parseEEParamGripperCommand sets up how to interpret the CRCL gripper command. Either a percentage, position or
     * a vel/fmax algorithm is set to establish how to handle gripper commmands. This is not explicitly spelled out in CRCl but
     * CRCl does allow the setting of end effector parameters.
     * @param incmd  CRCL command
     * @param outcmd ROS sensor message joint  values for gripper. Others joints assume current status position.
     * @param instatus current robot and gripper wm and status.
     * @return CRCL done or working
     */
    virtual int parseEEParamGripperCommand(crcl_rosmsgs::CrclCommandMsg &incmd,
                                                CCanonCmd &outcmd,
                                                CCanonWorldModel instatus);

    /**
     * @brief parseMoveThruCommand  a Cartesian motion that uses waypoints to plan a trajectory.
     * The gotraj planner accepts the beginning, each waypoint and the goal point to plan a motion.
     * Motion rates are determined by CRCL. gotraj planning does motion stops at each waypoint as well
     * as the goal point. Tolerances are not incorporated in the current motion strategy.
     * @param incmd CRCL waypoint motion command with tolerances.
     * @param outcmd a ROS sensor message joint  values for robot. Gripper joints assume current status position.
     * @param instatus current robot and gripper wm and status.
     * @return CRCL done or working
     */
    virtual int parseMoveThruCommand(crcl_rosmsgs::CrclCommandMsg &incmd,
                                    CCanonCmd &outcmd,
                                    CCanonWorldModel instatus) ;

    /**
     * @brief setGripperFoce set the ROS out command to use force
     * @param outcmd updated command incorporating gripper force directive
     * @param vel velocity of gripper motion
     * @return 0 sucessful, negative fail.
     */
    int   setGripperFoce(CCanonCmd &outcmd,
                                         double vel);
    /**
     * @brief setGripperFoce set the ROS out command to use velocity and force max
     * @param outcmd updated command incorporating gripper force directive
     * @param vel velocity of gripper motion
     * @param fmax maximum force to apply to gripper finger joints.
     * @return 0 sucessful, negative fail.
     */
    int  setGripperVelFmax(CCanonCmd &outcmd, double vel, double fmax );

    /**
     * @brief gripperSpeed increase or decrease robot motion speeds depending on whether grasp is open or close.
     * @param incmd contains type of gripper motion - open/close/neither
     * @return -1 not smart speed, -2 no changes, 0 sucessfully changed close speeds, 1 changed open speeds
     */
    int gripperSpeed(crcl_rosmsgs::CrclCommandMsg &incmd);

    virtual sensor_msgs::JointState updateJointState(std::vector<uint64_t> jointnums,
                                        sensor_msgs::JointState oldjoints,
                                        sensor_msgs::JointState newjoints) ;

protected:
    boost::shared_ptr<IKinematic> _kinematics; /**<  kinematics pointer */
    std::shared_ptr<CController>_nc; /**<  robot controller pointer */
    uint64_t _lastcmdnum; /**<  last command num to check for new command */
    std::shared_ptr<GoTraj> _goRobot; /**<  gotraj dll  motion planning for robot Cartesian and joint motion */
    std::shared_ptr<GoTraj> _goGripper; /**<  gotraj dll for gripper joint motion */


};



}

#endif
