#ifndef SERIALLINKROBOT_H
#define SERIALLINKROBOT_H

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

// /home/isd/michalos/src/github/nist/gzaprsros/include/aprs_headers/seriallinkrobot.h
// seriallinkrobot.h
//

#include <iostream>
#include <vector>
#include <stdio.h>
#include <string>
#include <map>
#include <algorithm>

// URDF
#include <tf/tf.h>
#include <urdf/model.h>

// You must have ROS installed to use this URDF parsing.
namespace RCS {


/**
 * @brief The ISerialLinkRobot struct defines an interface containing all the
 * necessary variable information to contain robot kinematics. It handles URDF
 * format.
 */

struct ISerialLinkRobot
{
    std::vector<tf::Vector3> axis;
    std::vector<tf::Vector3> xyzorigin;
    std::vector<tf::Vector3> rpyorigin;

    std::vector<std::string> jointNames;
    std::vector<std::string> linkNames;
    std::vector<double> jointMin;
    std::vector<double> jointMax;
    std::vector<bool> jointHasLimit;
    std::vector<double> jointVelMax;
    std::vector<double> jointEffort;
    std::string robot_name;
    std::string urdf_Xml;
    std::string base_link ;
    std::string end_link;

};

/**
 * @brief The CSerialLinkRobot struct an implementation to fill an ISerialLinkRobot
 * from the specification of a URDF XML string. Given the URDF, and
 * a base and tip link, the robot kinematic information will be
 * extracted from the URDF.
 */
struct CSerialLinkRobot
{

    /**
     * @brief CSerialLinkRobot requires a pointer to a  ISerialLinkRobot
     * instance that will be filled with URDF robot information.
     * @param robot pointer to a ISerialLinkRobot instance
     */
    CSerialLinkRobot(ISerialLinkRobot * robot ): _robot(robot)
    {

    }

    /**
     * @brief parseURDF parses a URDF XML string given a base and tip link name, using
     * ROS URDFdom to parse the URDF. After the parse, the ISerialLinkRobot variable
     * information will be extracted from the URDFdom.
     * @param xml_string URDF in string format.
     * @param base_link  name of the base link in the URDF.
     * @param end_link name of the tip link in the URDF
     * @return  true parse correctly. False, failed.
     */
    bool parseURDF(std::string xml_string, std::string base_link, std::string end_link)
    {
        urdf::Model robot_model;
        if(xml_string.empty())
        {
            std::cerr<< "Empty URDF string\n";
            return false;
        }


        robot_model.initString(xml_string);

        urdf::LinkConstSharedPtr link = robot_model.getLink(end_link);
        if(link.get() == NULL)
        {
            std::cerr<< "NULL pointer end_link\n";
            return false;
        }

        _robot->end_link = end_link;
        _robot->base_link = base_link;
        _robot->urdf_Xml = xml_string;
        _robot->robot_name = robot_model.getName() ;

        while (link->name != base_link)
        {
            _robot->linkNames.push_back(link->name);
            urdf::JointSharedPtr joint   = link->parent_joint;
            if (joint) {
                if (joint->type != urdf::Joint::UNKNOWN && joint->type != urdf::Joint::FIXED)
                {

                    _robot->jointNames.push_back(joint->name);
                    _robot->axis.push_back(Convert(joint->axis));
                    _robot->xyzorigin.push_back(Convert(joint->parent_to_joint_origin_transform.position));
                    double roll, pitch, yaw;
                    joint->parent_to_joint_origin_transform.rotation.getRPY(roll, pitch, yaw);
                    _robot->rpyorigin.push_back(tf::Vector3(roll, pitch, yaw));

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
                        _robot->jointHasLimit.push_back(true);
                        _robot->jointMin.push_back(lower);
                        _robot->jointMax.push_back(upper);
                    } else {
                        _robot->jointHasLimit.push_back(false);
                        _robot->jointMin.push_back(-M_PI);
                        _robot->jointMax.push_back(M_PI);
                        ////////////////////////////////////////////////////////////////////////////////
                    }
                    _robot->jointEffort.push_back(maxeffort);
                    _robot->jointVelMax.push_back(maxvel);
                }
            } else
            {
                std::cerr << "IKinematics::ParseURDF no joint corresponding to " << link->name << "\n";
            }
            link = link->getParent();
        }

        std::reverse(_robot->linkNames.begin(), _robot->linkNames.end());
        std::reverse(_robot->jointNames.begin(), _robot->jointNames.end());
        std::reverse(_robot->jointMin.begin(), _robot->jointMin.end());
        std::reverse(_robot->jointMax.begin(), _robot->jointMax.end());
        std::reverse(_robot->jointHasLimit.begin(), _robot->jointHasLimit.end());
        std::reverse(_robot->axis.begin(), _robot->axis.end());
        std::reverse(_robot->xyzorigin.begin(), _robot->xyzorigin.end());
        std::reverse(_robot->rpyorigin.begin(), _robot->rpyorigin.end());
        std::reverse(_robot->jointEffort.begin(), _robot->jointEffort.end());
        std::reverse(_robot->jointVelMax.begin(), _robot->jointVelMax.end());

        return true;
    }
private:
    ISerialLinkRobot * _robot;
    inline tf::Vector3 Convert (urdf::Vector3 v) {
        return tf::Vector3(v.x, v.y, v.z);
    }
};

}
#endif // SERIALLINKROBOT_H
