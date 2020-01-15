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
#include <boost/dll/alias.hpp>

namespace RCS
{

class GoKin :
        public IKinematic,
        public CSerialLinkRobot,
        public TestingKinematics<GoKin>
{
public:


public:
    GoKin();
    ~GoKin();
    int init();
    int init(std::string urdf, std::string baselink, std::string tiplink);
    const  std::string & getName(void){ return robot_name; }

    int FK(std::vector<double> jv, tf::Pose &pose);
    int IK(tf::Pose pose, std::vector<double>&) ;
    int debug(bool flag);
    int debugStream(std::ostream&);
    int SetWristOffset(double x);
    size_t numJoints() ;

    std::string set(std::string param,  std::string value);
    std::string set(std::string param,  void * value);
    std::string get(std::string param);
    bool isError(){ return errmsg.empty(); }
    int calibrate(const std::vector<double>& joints, const tf::Pose pose);

    std::string  runtests(std::string filepath)
    {
        return TestingKinematics<GoKin>::runtests(filepath);
    }

    /**
      * @brief create boost dll package factory creator
      * @return shared pointer to new gokin instance
      */
    static boost::shared_ptr<GoKin> create()
    {
        return boost::shared_ptr<GoKin>( new GoKin());
    }

    // this is here for posterity - to be replace by URDF xtring read init
    int Init(std::string filename);
private:

    tf::Pose basepose;
    tf::Pose poseLocal2Wrld;

    bool bDebug;
    std::ofstream out;
    std::string errmsg;

    std::string inifile_name;

    double m_per_length_units;
    double rad_per_angle_units;
    int link_number;
    go_link link_params[GENSER_MAX_JOINTS];
    genser_struct kins;
    go_real home_joint[GENSER_MAX_JOINTS];
    char kin_name[GO_KIN_NAME_LEN] ;
    go_pose home_position;
    std::string _urdf;
    std::string _urdffile;
    std::string _baselink;
    std::string  _tiplink;
    std::string  _inifilename;
    std::string ini;
    size_t _nJoints;

    //    boost::filesystem::path temp ;
    //    std::string tempstr;
};

//extern "C" BOOST_SYMBOL_EXPORT  GoKin goserkin;
BOOST_DLL_ALIAS(
            GoKin::create, create_plugin
        )


}
#endif // GOKIN_H
