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

class fanuc_lrmate200id;
namespace RCS
{
class gokin_fanuc :  public IKinematic, public  CSerialLinkRobot
{
public:

    gokin_fanuc();

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

    int calibrate(const std::vector<double>& joints, const tf::Pose pose)
    {
        return 0;
    }    /**
     * @brief create boost dll factory method. Creates gokin_fanuc instances.
     * @return  wrapped shared pointer to new gokin_fanuc instance.
     */
    static boost::shared_ptr<gokin_fanuc> create()
    {
        return boost::shared_ptr<gokin_fanuc>( new gokin_fanuc());
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
    std::shared_ptr<fanuc_lrmate200id> fanuckin;



};

BOOST_DLL_ALIAS(
            RCS::gokin_fanuc::create, create_plugin
        )

}

#endif
