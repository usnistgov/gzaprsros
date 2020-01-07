#ifndef GO_KINFANUC_H
#define GO_KINFANUC_H



#include <tf/tf.h>
#include <sensor_msgs/JointState.h>
#include <fstream>
#include <string>


#include <aprs_headers/IKinematic.h>
#include <aprs_headers/seriallinkrobot.h>

#include <boost/config.hpp>
#include <boost/dll/alias.hpp>
#define USE_DEFINES
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ulapi.h>
#include <inifile.h>
#include <go.h>
#include <genserkins.h>

namespace RCS
{
class gomotion_plugin :  public IKinematic, public  CSerialLinkRobot
{
public:

    gomotion_plugin();

    int init();

    int FK(std::vector<double> jv, tf::Pose &pose);
    int IK(tf::Pose pose, std::vector<double>&) ;

    const  std::string & getName(void){ return robot_name; }
    int debug(bool flag);
    int debugStream(std::ostream&);
    size_t numJoints() ;
    std::string set(std::string param,  std::string value);
    std::string set(std::string param,  void * value);
    std::string get(std::string param);
    bool isError(){ return errmsg.empty(); }

    /**
     * @brief create boost dll factory method. Creates gokin_fanuc instances.
     * @return  wrapped shared pointer to new gokin_fanuc instance.
     */
    static boost::shared_ptr<gomotion_plugin> create()
    {
        return boost::shared_ptr<gomotion_plugin>( new gomotion_plugin());
    }

private:

    bool bDebug;
    bool bHandleExceptions;
    std::ofstream out;
    std::string errmsg;
    std::string _urdf;
    std::string _urdffile;
    std::string _baselink;
    std::string  _tiplink;
    std::string  _inifilename;
    std::string ini;
    size_t _nJoints;

    int
    ini_load(char * inifile_name,
         double * m_per_length_units,
         double * rad_per_angle_units,
         go_pose * home,
         int * link_number,
         go_link * link_params,
         go_real * jhome,
         char * kin_name);
    double m_per_length_units;
    double rad_per_angle_units;
    int link_number;
    go_link link_params[GENSER_MAX_JOINTS];
    genser_struct kins;
    go_real joints[GENSER_MAX_JOINTS];
    go_real home_joint[GENSER_MAX_JOINTS];
    go_pose pose;
    go_pose home_position;
    char kin_name[GO_KIN_NAME_LEN] = "";
    go_rpy rpy;
    int t;
    int res;

};

BOOST_DLL_ALIAS(
            RCS::gomotion_plugin::create, create_plugin
        )

}

#endif
