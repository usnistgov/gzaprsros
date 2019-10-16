#ifndef GOKIN_H
#define GOKIN_H

#include <tf/tf.h>
#include <sensor_msgs/JointState.h>
#include <fstream>

#include <gokin/goserkins.h>
#include <gokin/_gokin.h>
#include <gokin/genserkins.h>

namespace gomotion
{
class GoMotoKin
{
public:
    GoMotoKin();
    int FK(std::vector<double> jv, tf::Pose &pose);
    int IK(tf::Pose pose, std::vector<double>&) ;
    int Debug(bool flag);
    int DebugStream(std::ostream&);
    int SetWristOffset(double x);
    int Init(std::string filename);
private:
    tf::Pose basepose;
    bool bDebug;
    std::ofstream out;

    std::string inifile_name;

    double m_per_length_units;
    double rad_per_angle_units;
    int link_number;
    go_link link_params[GENSER_MAX_JOINTS];
    genser_struct kins;
    go_real home_joint[GENSER_MAX_JOINTS];
    char kin_name[GO_KIN_NAME_LEN] ;
    go_pose home_position;
};
}
#endif // GOKIN_H
