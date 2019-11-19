#ifndef SERIALLINKROBOT_H
#define SERIALLINKROBOT_H

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

};

struct CSerialLinkRobot : public ISerialLinkRobot
{

    std::string urdf_Xml;
    std::string base_link ;
    std::string end_link;
//    VAR(std::string, prefix);

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

        this->end_link = end_link;
        this->base_link = base_link;
        this->urdf_Xml = xml_string;
        this->robot_name = robot_model.getName() ;

        while (link->name != base_link)
        {
            linkNames.push_back(link->name);
            urdf::JointSharedPtr joint   = link->parent_joint;
            if (joint) {
                if (joint->type != urdf::Joint::UNKNOWN && joint->type != urdf::Joint::FIXED)
                {

                    jointNames.push_back(joint->name);
                    axis.push_back(Convert(joint->axis));
                    xyzorigin.push_back(Convert(joint->parent_to_joint_origin_transform.position));
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
                        jointHasLimit.push_back(true);
                        jointMin.push_back(lower);
                        jointMax.push_back(upper);
                    } else {
                        jointHasLimit.push_back(false);
                        jointMin.push_back(-M_PI);
                        jointMax.push_back(M_PI);
                        ////////////////////////////////////////////////////////////////////////////////
                    }
                    jointEffort.push_back(maxeffort);
                    jointVelMax.push_back(maxvel);
                }
            } else
            {
                std::cerr << "IKinematics::ParseURDF no joint corresponding to " << link->name << "\n";
            }
            link = link->getParent();
        }

        std::reverse(linkNames.begin(), linkNames.end());
        std::reverse(jointNames.begin(), jointNames.end());
        std::reverse(jointMin.begin(), jointMin.end());
        std::reverse(jointMax.begin(), jointMax.end());
        std::reverse(jointHasLimit.begin(), jointHasLimit.end());
        std::reverse(axis.begin(), axis.end());
        std::reverse(xyzorigin.begin(), xyzorigin.end());
        std::reverse(rpyorigin.begin(), rpyorigin.end());
        std::reverse(jointEffort.begin(), jointEffort.end());
        std::reverse(jointVelMax.begin(), jointVelMax.end());

        return true;
    }
private:
    inline tf::Vector3 Convert (urdf::Vector3 v) {
        return tf::Vector3(v.x, v.y, v.z);
    }
};

}
#endif // SERIALLINKROBOT_H
