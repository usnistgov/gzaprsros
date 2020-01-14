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

#include <aprs_headers/Testing.h>

template <typename Derived>
class TestingKinematics
{
public:
    int  runtests(std::string filepath)
    {
        int runstatus=0;
        std::string line;

        //    if(filepath.empty() || !File(filepath).exists() )
        //    {
        //        std::cout << filename << "does not exist\n";
        //        return 0;
        //    }
        std::ifstream myfile( filepath );
        std::string summary;
        if (myfile)  // same as: if (myfile.good())
        {
            while (getline( myfile, line ))  // same as: while (getline( myfile, line ).good())
            {
                size_t numJoints =static_cast<Derived*>(this)->numJoints();
                if(line.empty())
                    continue;

                size_t n;
                if((n=line.find("="))==std::string::npos)
                    continue;

                // split between =
                std::string test = line.substr(0, n); // cmd
                std::string eq = line.substr(n+1); // answer
                boost::trim(test);
                boost::trim(eq);

                std::vector<std::string> result;
                boost::split( result, test, boost::is_any_of(" "), boost::token_compress_on );
                std::string cmd(*(result.begin()));
                boost::trim(cmd);
                test=test.erase(0,std::string(cmd + " ").length());

                std::vector<double> ds;
                boost::split( result, test, boost::is_any_of(","), boost::token_compress_on );
                ds.resize(result.size());
                std::transform(result.begin(), result.end(), ds.begin(), [](const std::string& val)
                {
                    std::string tval = boost::trim_copy(val);
                    return std::stod(tval);
                });

                std::vector<double> answer;
                result.clear();
                boost::split( result, eq, boost::is_any_of(","), boost::token_compress_on );
                answer.resize(result.size());
                std::transform(result.begin(), result.end(), answer.begin(), [](const std::string& val)
                {
                    std::string tval = boost::trim_copy(val);
                    return std::stod(tval);
                });

                // now run command.
                if(cmd == "fk")
                {
                    std::cout << "Run fk command\n";
                    tf::Pose pose;
                    if(!static_cast<Derived*>(this)->FK(ds, pose ))
                    {
                        summary+="FK Internal Failed:"+line;
                        runstatus=-1;
                        continue;
                    }
                    tf::Pose answerpose = tf::Pose(tf::Quaternion(answer[3], answer[4], answer[5], answer[6]), tf::Vector3(answer[0], answer[1], answer[2]));
                    if(!RCS::EQ(pose, answerpose))
                    {
                        summary+="FK Failed:"+line;
                        runstatus=-1;
                    }
                }
                else if(cmd == "ik")
                {
                    tf::Pose pose = tf::Pose(tf::Quaternion(ds[3], ds[4], ds[5], ds[6]), tf::Vector3(ds[0], ds[1], ds[2]));
                    std::vector<double> joints(numJoints);
                    std::cout << "Run ik command\n";
                    if(!static_cast<Derived*>(this)->IK(pose, joints ))
                    {
                        summary+="IK Internal Failed:"+line;
                        runstatus=-1;
                        continue;

                    }
                    if(!RCS::EQ<double>(joints, answer))
                    {
                        summary+="IK Failed:"+line;
                        runstatus=-1;
                    }
                }
            }
            myfile.close();
        }
        else
        {
            summary+="File open failed";
            runstatus=-1;
        }
        return runstatus;
    }
};


}
#endif // SERIALLINKROBOT_H
