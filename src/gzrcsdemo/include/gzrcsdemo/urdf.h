#ifndef URDF_H
#define URDF_H

#define GLOGGER GLogger
#include "aprs_headers/LoggerMacros.h"
#include "gzrcsdemo/Globals.h"
#include <urdf/model.h>


class urdf_reader
{
public:
    VAR(std::string, urdfXml);
    VAR(std::string, baseLink );
    VAR(std::string, endLink);

    VAR(std::vector<tf::Vector3>, axis);
    VAR(std::vector<tf::Vector3>, xyzorigin);
    VAR(std::vector<tf::Vector3>, rpyorigin);

    VAR(std::vector<std::string>, jointNames);
    VAR(std::vector<std::string>, linkNames);
    VAR(std::string, prefix);
    VAR(std::vector<double>, jointMin);
    VAR(std::vector<double>,jointMax);
    VAR(std::vector<bool>,jointHasLimit);
    VAR(std::vector<double>,jointVelMax);
    VAR(std::vector<double>,jointEffort);

    bool parseURDF(std::string xml_string, std::string base_link, std::string end_link)
    {
        urdf::Model robot_model;
        if(xml_string.empty())
        {
            if(Globals.bHandleExceptions)
                throw RobotControlException(No_URDF_String);
            logFatal("No URDF string");
            return false;
        }

        robot_model.initString(xml_string);

        urdf::LinkConstSharedPtr link = robot_model.getLink(end_link);
        if(link.get() == NULL)
        {
            if(Globals.bHandleExceptions)
                throw RobotControlException(Null_Pointer);
            logFatal("NULL pointer end_link");
            return false;
        }
        while (link->name != base_link)
        {
            linkNames().push_back(link->name);
            urdf::JointSharedPtr joint   = link->parent_joint;
            if (joint) {
                if (joint->type != urdf::Joint::UNKNOWN && joint->type != urdf::Joint::FIXED)
                {

                    jointNames().push_back(joint->name);
                    axis().push_back(Convert<urdf::Vector3, tf::Vector3>(joint->axis));
                    xyzorigin().push_back(Convert<urdf::Vector3, tf::Vector3>(joint->parent_to_joint_origin_transform.position));
                    double roll, pitch, yaw;
                    joint->parent_to_joint_origin_transform.rotation.getRPY(roll, pitch, yaw);
                    rpyorigin().push_back(tf::Vector3(roll, pitch, yaw));

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
                        jointHasLimit().push_back(true);
                        jointMin().push_back(lower);
                        jointMax().push_back(upper);
                    } else {
                        jointHasLimit().push_back(false);
                        jointMin().push_back(-M_PI);
                        jointMax().push_back(M_PI);
                        ////////////////////////////////////////////////////////////////////////////////
                    }
                    jointEffort().push_back(maxeffort);
                    jointVelMax().push_back(maxvel);
                }
            } else
            {
                logError("IKinematics::ParseURDF no joint corresponding to %s", link->name.c_str());
            }
            link = link->getParent();
        }

        std::reverse(linkNames().begin(), linkNames().end());
        std::reverse(jointNames().begin(), jointNames().end());
        std::reverse(jointMin().begin(), jointMin().end());
        std::reverse(jointMax().begin(), jointMax().end());
        std::reverse(jointHasLimit().begin(), jointHasLimit().end());
        std::reverse(axis().begin(), axis().end());
        std::reverse(xyzorigin().begin(), xyzorigin().end());
        std::reverse(rpyorigin().begin(), rpyorigin().end());
        std::reverse(jointEffort().begin(), jointEffort().end());
        std::reverse(jointVelMax().begin(), jointVelMax().end());

        return true;
    }
};
#endif // URDF_H
