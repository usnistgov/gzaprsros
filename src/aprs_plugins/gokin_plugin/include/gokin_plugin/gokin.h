#ifndef GOKIN_H
#define GOKIN_H

#include <tf/tf.h>
#include <sensor_msgs/JointState.h>
#include <fstream>
#include <string>

#include <gokin_plugin/goserkins.h>
#include <gokin_plugin/_gokin.h>
#include <gokin_plugin/genserkins.h>

#include <aprs_headers/IKinematic.h>
#include <aprs_headers/seriallinkrobot.h>

#include <boost/config.hpp>
namespace RCS
{

class GoMotoKin : public IKinematic, public  CSerialLinkRobot
{
public:


public:
    GoMotoKin();
   ~GoMotoKin();
    int init(std::string urdf, std::string baselink, std::string tiplink);
    const  std::string & getName(void){ return robot_name; }

    int FK(std::vector<double> jv, tf::Pose &pose);
    int IK(tf::Pose pose, std::vector<double>&) ;
    int debug(bool flag);
    int debugStream(std::ostream&);
    int SetWristOffset(double x);
    size_t numJoints() { return jointNames.size(); }
    // this is here for posterity - to be replace by URDF xtring read init
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
//    boost::filesystem::path temp ;
//    std::string tempstr;
};

extern "C" BOOST_SYMBOL_EXPORT  GoMotoKin goserkin;

}
#endif // GOKIN_H
