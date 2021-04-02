#ifndef KDL_PLUGIN
#define KDL_PLUGIN

/*
 * DISCLAIMER:
 * This software was produced by the National Institute of Standards
 * and Technology (NIST), an agency of the U.S. government, and by statute is
 * not subject to copyright in the United States.  Recipients of this software
 * assume all responsibility associated with its operation, modification,
 * maintenance, and subsequent redistribution.
 *
 * See NIST Administration Manual 4.09.07 b and Appendix I.
 */


#include <tf/tf.h>
#include <sensor_msgs/JointState.h>
#include <fstream>
#include <string>

#include <kdl/jntarray.hpp>
#include <kdl/chaindynparam.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/jntspaceinertiamatrix.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/frames.hpp>
#include <kdl/jacobian.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>

#include <aprs_headers/IKinematic.h>
#include <aprs_headers/seriallinkrobot.h>

#include <boost/dll/alias.hpp>
#include <boost/config.hpp>
namespace RCS
{

class Ckdl_plugin : public IKinematic, public  CSerialLinkRobot
{
public:


public:
    Ckdl_plugin();
   ~Ckdl_plugin();
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

    static boost::shared_ptr<Ckdl_plugin> create()
    {
        return boost::shared_ptr<Ckdl_plugin>( new Ckdl_plugin());
    }

private:
    KDL::Tree tree;
    KDL::Chain chain;
    KDL::ChainFkSolverPos_recursive* fk_solver;
    KDL::ChainIkSolverPos_NR_JL *ik_solver_pos;
    KDL::ChainIkSolverVel_pinv * ik_solver_vel;

    tf::Pose basepose;

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

};

//extern "C" BOOST_SYMBOL_EXPORT  Ckdl_plugin kdl_plugin;
BOOST_DLL_ALIAS(
            RCS::Ckdl_plugin::create, create_plugin
        )
}

#endif
