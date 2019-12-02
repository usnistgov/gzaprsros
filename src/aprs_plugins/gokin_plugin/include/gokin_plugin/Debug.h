#include <iostream>
#include <string>
#include <sstream>
#include <tf/tf.h>
#include <sensor_msgs/JointState.h>
#include <gokin_plugin/gokin.h>

/*
 * 1:
 * Joints  = -0.063,-0.153,-0.592,-0.000,-1.000,-0.000
 * Joint Deg= -3.61, -8.766, -33.919, 0, -57.296, 0
 * FK      = 0.327679,-0.0206711,0.466133
 * RPY     = 0,0,0
 * Will RPY: 180, -7.551, -3.61
 * Q       = 0.997334,-0.0314264,0.0658178,0.00207395


 * 1:
 * Joints  = 0,0,0,0,0,0
 * FK      = 0.465, 0, 0.695
 * RPY     = -180, -90, 0

 * 2:
 * Joints  = -0.063: 0.333:-0.847:-0.000:-0.259:-0.000
 * FK      = 0.327679,-0.0206711,0.266133
 * RPY:    = 0,0,0
 * Will:    180, -7.551, -3.61
 * Status Pose     T   = 0.327679,-0.0206711,0.266133


*/

namespace RCS
{
inline std::string DumpJoints(std::vector<double> joints)
{
    std::stringstream ss;
    ss << "Joints=";
    for(size_t i=0; i< joints.size(); i++)
    {
        if(i>0)
            ss << ",";
        ss << joints[i] ;
    }
    ss  << "\n";
    return ss.str();
}

inline std::string DumpGoPose(go_pose pose)
{
    go_rpy rpy;
    std::stringstream ss;
    ss << "\txyz=" << pose.tran.x << "," <<  pose.tran.y << "," <<  pose.tran.z << "\n";
    ss << "\tq=" << pose.rot.x << "," <<  pose.rot.y << "," <<  pose.rot.z << "," <<  pose.rot.s << "\n";
    go_quat_rpy_convert(&pose.rot, &rpy);
    ss << "\trpy=" << rpy.r << "," <<  rpy.p << "," <<  rpy.y << "\n";
    return ss.str();
}

inline std::string DumpTfPose(tf::Pose pose)
{
    std::stringstream ss;
    ss << "\txyz=" << pose.getOrigin().x() << "," <<  pose.getOrigin().y() << "," <<  pose.getOrigin().z() << "\n";
    ss << "\tq=" << pose.getRotation().x() << "," <<  pose.getRotation().y() << "," <<  pose.getRotation().z() << "," <<  pose.getRotation().w() << "\n";

    go_pose gopose;
    gopose.rot.x=pose.getRotation().x();
    gopose.rot.y=pose.getRotation().y();
    gopose.rot.z=pose.getRotation().z();
    gopose.rot.s=pose.getRotation().w();

    go_rpy rpy;
    go_quat_rpy_convert(&gopose.rot, &rpy);
    ss << "\trpy=" << rpy.r << "," <<  rpy.p << "," <<  rpy.y << "\n";
    return ss.str();
}

}
