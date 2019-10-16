

//
// Gripper.h
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
#ifndef _GRIPPER_H
#define _GRIPPER_H

#include <string>
#include <vector>
#include <math.h>
#include <memory>
#include <string>
#include <vector>


#include <sensor_msgs/JointState.h>
#include <ros/time.h>


#include "aprs_headers/Core.h"
#include "aprs_headers/IRcs.h"

#include "gzrcs/Globals.h"

#include "aprs_headers/Debug.h"

#include <gzrcs/gazebo.h>

/**
 * @brief The GripperInterface class merely sets up a JointState with names and positions
 * for opening and closing a robotiq gripper. Upon trial and error the 0..1 ee percentage
 * prescribed by CRCl so FIXME needs to be normalized by actual gripper open and close radian
 * readings.
 */

class GripperInterface
{
public:

    GripperInterface();


    /**
     * @brief init
     * @param jointnames fully qualified names of the gripper joints
     * @param joints_open vector containing open position of all the joints
     * @param joints_closed vector containing closed position of all the joints
     * @param joints_multipler multiple when using position setting (not percentage for now).
     * so a +1 stores position, while a -1 stores inverse.
     */



    /**
     * @brief init read robot's gripper name, then fetch gripper parameterization
     * @param robotName prefix name of robot
     * @return
     */
    virtual std::string init( std::string robotName);


    void start();
    void stop();
    void updateGripper(double time, double eepercent);
    CGzParallelGripper _gripperwriter;  /// gazebo pavel gripper joint update publisher (writer).

    sensor_msgs::JointState increment(sensor_msgs::JointState gripperjoints,
                                      std::vector<std::string> joints,
                                      double amount);

    /**
     * @brief close set all the gripper joints to the closed position.
     * @return a jointstate command object with names and joints position closed.
     */
    sensor_msgs::JointState close();
    /**
     * @brief open set all the gripper joints to the open position.
     * @return a jointstate command object with names and joints position open
     */
    sensor_msgs::JointState open();

    bool is_open();
    bool is_closed();
    int isGrasping();
    /**
     * @brief setPosition set joints to the following position. Is a hard number
     * not a percentage for now.
     * @param position degree or prismatic value for the joints.
     * @return  revised jointstate command.
     */
    sensor_msgs::JointState set_position(double position);

    /**
     * @brief setAbsPosition sets an absolute gripper position, not a percentage of open/close.
     * @param position to use for joints.
     * @return
     */
    sensor_msgs::JointState set_abs_position(double position);

    std::vector<std::string> jointnames()
    {
        return joint_state.name;
    }
    /**
     * @brief publish_jointstate not used. Gripper joints published elsewhere.
     * Could be using gazebo, ROS or HW interface.
     */
    void publish_jointstate()
    {
        // this has been factored OUT of this class.
        // could be ros, gazebo, crcl, or other communication
        // gripper joints commands (at least for robotiq) should be good
        assert(0);
    }
    double multipler(std::string name)
    {
        std::vector<std::string>::iterator iter = std::find(joints_names.begin(), joints_names.end(), name);
        size_t index = std::distance(joints_names.begin(), iter);
        if(index == joints_names.size())
            return 1; // error
        return joints_multipler[index];
    }

    void update(double eepercent);

protected:

    std::string gripper_name;
    std::vector<std::string> joints_names;
    sensor_msgs::JointState joint_state;
    std::vector<double> joints_open;
    std::vector<double> joints_closed;
    std::vector<double> joints_multipler;
    std::vector<std::string> fingerNames;
    std::vector<bool> finger_contact;

};



#endif
