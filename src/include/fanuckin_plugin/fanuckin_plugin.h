#ifndef FANUCKIN_PLUGIN
#define FANUCKIN_PLUGIN



#include <tf/tf.h>
#include <sensor_msgs/JointState.h>
#include <fstream>
#include <string>

#include <aprs_headers/IKinematic.h>
#include <aprs_headers/seriallinkrobot.h>
#include <aprs_headers/Debug.h>

#include <boost/dll/alias.hpp>
#include <boost/config.hpp>

class fanuc_lrmate200id;
namespace RCS
{

class Cfanuckin_plugin : public IKinematic, public  CSerialLinkRobot
{
public:


public:
    Cfanuckin_plugin();
   ~Cfanuckin_plugin();
    int init();
    const  std::string & getName(void){ return robot_name; }

    int FK(std::vector<double> jv, tf::Pose &pose);
    int IK(tf::Pose pose, std::vector<double>&) ;
    int debug(bool flag);
    int debugStream(std::ostream&);
    size_t numJoints() ;
    std::string set(std::string param,  std::string value);
    std::string set(std::string param,  void * value);
    std::string get(std::string param);
    bool isError(){ return errmsg.empty(); }

    static boost::shared_ptr<Cfanuckin_plugin> create()
    {
        return boost::shared_ptr<Cfanuckin_plugin>( new Cfanuckin_plugin());
    }

    int calibrate(const std::vector<double>& joints, const tf::Pose pose);

private:

    tf::Pose basepose;
    tf::Pose poseLocal2Wrld;

   std::string DumpTransformMatrices();
    std::string DumpUrdfJoint();

    bool bDebug;
    bool bHandleExceptions;
    std::ofstream out;
    std::string errmsg;
    std::string _urdf;
    std::string _urdffile;
    std::string _baselink;
    std::string  _tiplink;

    std::string inifile_name;
    double maxIterations, epsilon;
    std::vector<tf::Transform> A0;
    std::shared_ptr<fanuc_lrmate200id> kin;

};

//extern "C" BOOST_SYMBOL_EXPORT  Cfanuckin_plugin kdl_plugin;
BOOST_DLL_ALIAS(
            RCS::Cfanuckin_plugin::create, create_plugin
        )
}

#endif
