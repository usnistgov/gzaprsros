//

// fanuc_lrmate200id.h
//

/*
  DISCLAIMER:
  This software was produced by the National Institute of Standards
  and Technology (NIST), an agency of the U.S. government, and by statute is
  not subject to copyright in the United States.  Recipients of this software
  assume all responsibility associated with its operation, modification,
  maintenance, and subsequent redistribution.

  See NIST Administration Manual 4.09.07 b and Appendix I.
*/

/***
        fanuc_lrmate200id fanuc;

        // Have to be converted to radians
        double goal[] = { -0.0005, 29.930, -29.947, 0.12, -59.935, 0.062};
        std::vector<double> goaljts(goal, goal + sizeof(goal) / sizeof(double) ) ;
        // transform angles from degree to radians
        std::transform(goaljts.begin(), goaljts.end(), goaljts.begin(),
            std::bind1st(std::multiplies<double>(),  M_PI / 180.0 ));

        urdf::Pose pose = fanuc.fanuc_lrmate200id_kin_fwd(&goaljts[0]);
        std::cout << pose;

        double goal1[] = { -0.0005, 29.916, -29.949, 179.997, 60.509, 178.810};
        std::vector<double> goal1jts(goal1, goal1 + sizeof(goal1) / sizeof(double) ) ;
        // transform angles from degree to radians
        std::transform(goal1jts.begin(), goal1jts.end(), goal1jts.begin(),
            std::bind1st(std::multiplies<double>(),  M_PI / 180.0 ));

        urdf::Pose pose1 = fanuc.fanuc_lrmate200id_kin_fwd(&goal1jts[0]);
        std::cout << pose1;

Translation =    522.6895:    -0.1496:    69.2683
Rotation    =    179.8958:    -0.1518:     0.1216
Translation =    521.9315:    -0.0009:    69.4712
Rotation    =    179.9945:     0.3920:    -1.1920

#include "fanuc_lrmate200id.h"

class FanucLrMate200idKinematics : public IKinematics {
    fanuc_lrmate200id fanuckin;
    tf::Pose base;
public:

    FanucLrMate200idKinematics() {
        base = tf::Pose(tf::Quaternion(0, 0, 0, 1),
                tf::Vector3(0.0, 0.0, 0.330));
    }

    virtual RCS::Pose FK(std::vector<double> jv) {

        tf::Pose pose = fanuckin.fanuc_lrmate200id_kin_fwd(&jv[0]);
        return base*pose;
        //return pose;
    }

    virtual std::vector<double> IK(RCS::Pose & pose,
            std::vector<double> oldjoints) {
        tf::Pose base(tf::Quaternion(0, 0, 0, 1),
                tf::Vector3(0.0, 0.0, 0.330));
        pose = base.inverse() * pose;
        std::vector<double> joints = fanuckin.fanuc_lrmate200id_kin_inv(pose);
        return joints;
    }

    virtual size_t AllPoseToJoints(RCS::Pose & pose,
            std::vector<std::vector<double> > & newjoints) {
        return 0;
    }

    virtual std::vector<double> NearestJoints(
            std::vector<double> oldjoints,
            std::vector<std::vector<double> > & newjoints) {
        ROS_ERROR("FanucLrMate200idKinematics::NearestJoints() not implemented");
        return std::vector<double>();
    }

    virtual void Init(ros::NodeHandle &nh) {
        armkin = boost::shared_ptr<::Kinematics>(new ::Kinematics());
        armkin->init(nh, _tiplinkname, _rootlinkname);
        moveit_msgs::GetKinematicSolverInfo::Request request;
        moveit_msgs::GetKinematicSolverInfo::Response response;
        armkin->getFKSolverInfo(request, response);
        joint_names.clear();
        link_names.clear();
        num_joints = response.kinematic_solver_info.joint_names.size();
        for (unsigned int i = 0; i < response.kinematic_solver_info.joint_names.size(); i++) {
            joint_names.push_back(response.kinematic_solver_info.joint_names[i]);
        }
        for (unsigned int i = 0; i < response.kinematic_solver_info.link_names.size(); i++) {
            link_names.push_back(response.kinematic_solver_info.link_names[i]);
        }
    }



};


//#define lrmate200KIN    
#ifdef lrmate200KIN
        boost::shared_ptr<IKinematics> lrmate200idkin;
        lrmate200idkin = boost::shared_ptr<IKinematics>(new FanucLrMate200idKinematics());
        lrmate200idkin->Init(std::string("manipulator"), std::string("tool0"));
        lrmate200idkin->Init(nh);
        TestFk(lrmate200idkin, "lrmate200idkin");
        TestIk(lrmate200idkin, "lrmate200idkin");
#endif

        */
#ifndef fanuc_lrmate200id_h
#define fanuc_lrmate200id_h
#include <fstream>

#include <tf/tf.h>
#include <aprs_headers/IKinematic.h>
#include <aprs_headers/seriallinkrobot.h>

#include <boost/config.hpp>
#include <boost/dll/alias.hpp>



#define WRIST_OFFSET              0.080 /*  nominal offset in -Z for wrist */

#define THREE21_KIN_NUM_JOINTS    6

#define THREE21_SHOULDER_RIGHT    0x01
#define THREE21_ELBOW_DOWN        0x02
#define THREE21_WRIST_FLIP        0x04
#define THREE21_SINGULAR          0x08

namespace RCS
{
struct three21_kin
{
    double a1;
    double a2;
    double a3;
    double d2;
    double d3;
    double d4;
    long   iflags;

    // genser_struct gk;
};

struct  fanuc_lrmate200id_kin_struct
{
    three21_kin tk;
    tf::Pose  t7;           /* final tool transform */
    tf::Pose  t7_inv;       /* its inverse */
};

class fanuc_lrmate200id : public IKinematic, public  CSerialLinkRobot
{
public:
    fanuc_lrmate200id(void);
    ~fanuc_lrmate200id(void);

    int init();
    int init(std::string urdf, std::string baselink, std::string tiplink);
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


    static boost::shared_ptr<fanuc_lrmate200id> create()
    {
        return boost::shared_ptr<fanuc_lrmate200id>( new fanuc_lrmate200id());
    }
private:
    // -------------------------------------------
    fanuc_lrmate200id_kin_struct kins;
    tf::Pose fanuc_lrmate200id_kin_fwd(const double *motors);
    std::vector<double> fanuc_lrmate200id_kin_inv(tf::Pose & pos);

    std::vector<double> add_fkgearing(std::vector<double> motors) {
        std::vector<double> joints(motors.size(), 0.0);
        /* gearing equations */
        joints[0] = motors[0];
        joints[1] = motors[1] - M_PI_2;
        joints[2] = -(motors[1] + motors[2]);
        joints[3] = -motors[3];
        joints[4] = -motors[4];
        joints[5] = -motors[5];
        return joints;
    }
    bool bDebug;
    std::ofstream out;
    std::string errmsg;
    std::string inifile_name;
    std::string _urdf;
    std::string _urdffile;
    std::string _baselink;
    std::string  _tiplink;
    std::string  _inifilename;
    std::string ini;
    size_t _nJoints;

};
//extern "C" BOOST_SYMBOL_EXPORT  GoKin goserkin;
BOOST_DLL_ALIAS(
        RCS::fanuc_lrmate200id::create, create_plugin
        )

}
#endif
