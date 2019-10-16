
// DISCLAIMER:
// This software was developed by U.S. Government employees as part of
// their official duties and is not subject to copyright. No warranty implied 
// or intended.

#include <iostream>
#include <memory>
#include <boost/format.hpp>

#include "gzrcs/Controller.h"
#include "gzrcs/Kinematics.h"
#include "gzrcs/Globals.h"
#include "gzrcs/RobotControlException.h"

#define URDF_CONVERSIONS
#include <aprs_headers/Conversions.h>
#include <aprs_headers/Debug.h>
#define GLOGGER GLogger
#include "aprs_headers/LoggerMacros.h"

using namespace RCS;



////////////////////////////////////////////////////////////////////////////////
bool IKinematics::parseURDF(std::string xml_string, std::string base_link, std::string end_link)
{
    urdf::Model robot_model;
    if(xml_string.empty())
    {
        if(Globals.bHandleExceptions)
            throw RobotControlException(No_URDF_String);
        logFatal("No URDF string");
        return false;
    }

    robot_model.initString(xml_string);

    urdf::LinkConstSharedPtr link = robot_model.getLink(end_link);
    if(link.get() == NULL)
    {
        if(Globals.bHandleExceptions)
            throw RobotControlException(Null_Pointer);
        logFatal("NULL pointer end_link");
        return false;
    }
    while (link->name != base_link)
    {
        linkNames().push_back(link->name);
        urdf::JointSharedPtr joint   = link->parent_joint;
        if (joint) {
            if (joint->type != urdf::Joint::UNKNOWN && joint->type != urdf::Joint::FIXED)
            {

                jointNames().push_back(joint->name);
                axis().push_back(Convert<urdf::Vector3, tf::Vector3>(joint->axis));
                xyzorigin().push_back(Convert<urdf::Vector3, tf::Vector3>(joint->parent_to_joint_origin_transform.position));
                double roll, pitch, yaw;
                joint->parent_to_joint_origin_transform.rotation.getRPY(roll, pitch, yaw);
                rpyorigin().push_back(tf::Vector3(roll, pitch, yaw));

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
                    jointHasLimit().push_back(true);
                    jointMin().push_back(lower);
                    jointMax().push_back(upper);
                } else {
                    jointHasLimit().push_back(false);
                    jointMin().push_back(-M_PI);
                    jointMax().push_back(M_PI);
                    ////////////////////////////////////////////////////////////////////////////////
                }
                jointEffort().push_back(maxeffort);
                jointVelMax().push_back(maxvel);
            }
        } else
        {
            logError("IKinematics::ParseURDF no joint corresponding to %s", link->name.c_str());
        }
        link = link->getParent();
    }

    std::reverse(linkNames().begin(), linkNames().end());
    std::reverse(jointNames().begin(), jointNames().end());
    std::reverse(jointMin().begin(), jointMin().end());
    std::reverse(jointMax().begin(), jointMax().end());
    std::reverse(jointHasLimit().begin(), jointHasLimit().end());
    std::reverse(axis().begin(), axis().end());
    std::reverse(xyzorigin().begin(), xyzorigin().end());
    std::reverse(rpyorigin().begin(), rpyorigin().end());
    std::reverse(jointEffort().begin(), jointEffort().end());
    std::reverse(jointVelMax().begin(), jointVelMax().end());

    return true;
}



#ifdef MOTOGOKIN
#include <gokin/gokin.h>
#include "gzrcs/Controller.h"
MotomanSia20dGoKin::MotomanSia20dGoKin(std::shared_ptr<RCS::CController> nc) {
    _nc = nc;
    _pGoKin = std::shared_ptr<gomotion::GoMotoKin>(new gomotion::GoMotoKin());
    _pGoKin->Init(getexefolder() + "/config/motoman_sia20d.ini" );
}

tf::Pose MotomanSia20dGoKin::FK(std::vector<double> joints)
{
    tf::Pose pose;
    int res = _pGoKin-> FK(joints, pose);
    return pose;
}

int MotomanSia20dGoKin::allIK(tf::Pose & pose,
                              std::vector<std::vector<double>> &joints) {
    assert(0);
    return 0;
}

int MotomanSia20dGoKin::IK(tf::Pose pose,
                           std::vector<double> oldjoints,
                           std::vector<double>& newjoints)
{
    int res =_pGoKin->IK(pose,oldjoints);
    newjoints=oldjoints;
    return res;
}

bool MotomanSia20dGoKin::isSingular(tf::Pose pose, double threshold) {
    return false;
}
#endif

#ifdef GOKIN
static std::string DumpGoPose(go_pose pose)
{
    go_rpy rpy;
    std::stringstream ss;
    ss << "\txyz=" << pose.tran.x << "," <<  pose.tran.y << "," <<  pose.tran.z << "\n";
    ss << "\tq=" << pose.rot.x << "," <<  pose.rot.y << "," <<  pose.rot.z << "," <<  pose.rot.s << "\n";
    go_quat_rpy_convert(&pose.rot, &rpy);
    ss << "\trpy=" << rpy.r << "," <<  rpy.p << "," <<  rpy.y << "\n";
    return ss.str();
}

std::map<int, std::string> goerr = {{GO_RESULT_OK , "none"},
                                    { GO_RESULT_IGNORED,		"action can't be done, ignored"},
                                    { GO_RESULT_BAD_ARGS,		"arguments bad, e.g., null pointer"},
                                    { GO_RESULT_RANGE_ERROR,	"supplied range value out of bounds"},
                                    { GO_RESULT_DOMAIN_ERROR,	"resulting domain out of bounds"},
                                    { GO_RESULT_ERROR,		    "action can't be done, a problem"},
                                    { GO_RESULT_IMPL_ERROR,		"function not implemented"},
                                    { GO_RESULT_NORM_ERROR,		"a value is expected to be normalized "},
                                    { GO_RESULT_DIV_ERROR,		"divide by zero "},
                                    { GO_RESULT_SINGULAR,       "a matrix is singular "},
                                    { GO_RESULT_NO_SPACE,	    "no space for append operation "},
                                    { GO_RESULT_EMPTY,          "data structure is empty "},
                                    { GO_RESULT_BUG	,           " a bug in Go, e.g., unknown case "}
                                   };


////////////////////////////////////////////////////////////////////////////////
#include "aprs_headers/Config.h"

static int
ini_load(std::string inifile_name,
         double *m_per_length_units,
         double *rad_per_angle_units,
         int *link_number,
         std::vector<go_link> & link_params)
{
    int link;
    double d1, d2, d3, d4, d5, d6,d7,d8,d9;
    std::string servo_string, ini_string;
    Nist::Config cfg;
    if(!boost::filesystem::exists(inifile_name))
    {
        fprintf(stderr, "inifile does not exist\n", ini_string.c_str());
        return -1;

    }
    if(!cfg.load(inifile_name))
    {
        fprintf(stderr, "inifile failed to load\n", ini_string.c_str());
        return -1;
    }

    d1 = cfg.GetSymbolValue<double>("GOMOTION.LENGTH_UNITS_PER_M","1000");
    if (d1 <= 0.0) {
        fprintf(stderr, "invalid entry: [GOMOTION] LENGTH_UNITS_PER_M = %s must be positive\n", ini_string.c_str());
        return -1;
    } else {
        *m_per_length_units = 1.0 / d1;
    }

    d2 = cfg.GetSymbolValue<double>("GOMOTION.ANGLE_UNITS_PER_RAD","57.2957795130823");
    if (d2 <= 0.0) {
        fprintf(stderr, "invalid entry: [GOMOTION] ANGLE_UNITS_PER_RAD = %s must be positive\n", ini_string.c_str());
        return -1;
    } else {
        *rad_per_angle_units = 1.0 / d2;
    }

    int n = cfg.GetSymbolValue<int>("SERVO.HOWMANY","0");

    link_params.resize(n);

    for (link = 0; link < n ; link++)
    {
        //servo_string = StrFormat("SERVO_%d", link + 1);
        static char buffer[10];
        sprintf(buffer,"SERVO_%d", link + 1);
        servo_string=buffer;

        ini_string = cfg.GetSymbolValue<std::string>(servo_string + ".QUANTITY", "ERROR");
        if(ini_string=="Angle")
            link_params[link].quantity = GO_QUANTITY_ANGLE;
        else if(ini_string=="Length")
            link_params[link].quantity = GO_QUANTITY_LENGTH;
        else
        {
            fprintf(stderr, "bad entry: [%s] QUANTITY = %s\n", servo_string.c_str(), ini_string.c_str());
            return -1;
        }
        if(cfg.Exists(servo_string + ".DH_PARAMETERS"))
        {
            ini_string = cfg.GetSymbolValue<std::string>(servo_string + ".DH_PARAMETERS", "ERROR");
            if (4 == sscanf(ini_string.c_str(), "%lf %lf %lf %lf", &d1, &d2, &d3, &d4))
            {
                go_dh dh;
                dh.a = (go_real) (*m_per_length_units * d1);
                dh.alpha = (go_real) (*rad_per_angle_units * d2);
                dh.d = (go_real) (*m_per_length_units * d3);
                dh.theta = (go_real) (*rad_per_angle_units * d4);
                link_params[link].u.dh = dh;
                link_params[link].type = GO_LINK_DH;

            }
        }
        else if (cfg.Exists(servo_string + ".PP_PARAMETERS"))
        {
            ini_string = cfg.GetSymbolValue<std::string>(servo_string + ".DH_PARAMETERS", "ERROR");
            if (6 == sscanf(ini_string.c_str(), "%lf %lf %lf %lf", &d1, &d2, &d3, &d4, &d5, &d6))
            {

                go_rpy rpy;
                link_params[link].u.pp.pose.tran.x = (go_real) (*m_per_length_units * d1);
                link_params[link].u.pp.pose.tran.y = (go_real) (*m_per_length_units * d2);
                link_params[link].u.pp.pose.tran.z = (go_real) (*m_per_length_units * d3);
                rpy.r = (go_real) (*rad_per_angle_units * d4);
                rpy.p = (go_real) (*rad_per_angle_units * d5);
                rpy.y = (go_real) (*rad_per_angle_units * d6);
                go_rpy_quat_convert(&rpy, &link_params[link].u.pp.pose.rot);
                link_params[link].type = GO_LINK_PP;
            }
            else {
                fprintf(stderr, "bad entry: [%s] PP = %s\n", servo_string.c_str(), ini_string.c_str());
                return -1;
            }
        }
        else if(cfg.Exists(servo_string + ".URDF_PARAMETERS"))
        {
            ini_string = cfg.GetSymbolValue<std::string>(servo_string + ".URDF_PARAMETERS", "ERROR");
            if (9 == sscanf(ini_string.c_str(), "%lf %lf %lf %lf %lf %lf %lf %lf %lf", &d1, &d2, &d3, &d4, &d5, &d6, &d7, &d8, &d9))
            {
                go_rpy rpy;
                go_cart cart;
                link_params[link].u.urdf.pose.tran.x = (go_real) (*m_per_length_units * d1);
                link_params[link].u.urdf.pose.tran.y = (go_real) (*m_per_length_units * d2);
                link_params[link].u.urdf.pose.tran.z = (go_real) (*m_per_length_units * d3);
                rpy.r = (go_real) (*rad_per_angle_units * d4);
                rpy.p = (go_real) (*rad_per_angle_units * d5);
                rpy.y = (go_real) (*rad_per_angle_units * d6);
                go_rpy_quat_convert(&rpy, &link_params[link].u.urdf.pose.rot);
                cart.x = (go_real) (*m_per_length_units * d7);
                cart.y = (go_real) (*m_per_length_units * d8);
                cart.z = (go_real) (*m_per_length_units * d9);
                if (GO_RESULT_OK != go_cart_unit(&cart, &cart))
                {
                    fprintf(stderr, "bad entry: [%s] URDF = %s\n", servo_string.c_str(), ini_string.c_str());
                    return -1;
                }
                link_params[link].u.urdf.axis = cart;
                link_params[link].type = GO_LINK_URDF;
            }
            else
            {
                fprintf(stderr, "bad entry: [%s] PP = %s\n", servo_string.c_str(), ini_string.c_str());
                return -1;
            }
        }
        else if (cfg.Exists(servo_string + ".PK_PARAMETERS"))
        {
            ini_string = cfg.GetSymbolValue<std::string>(servo_string + ".PK_PARAMETERS", "ERROR");
            if (6 == sscanf(ini_string.c_str(), "%lf %lf %lf %lf %lf %lf", &d1, &d2, &d3, &d4, &d5, &d6))
            {
                link_params[link].u.pk.base.x = (go_real) (*m_per_length_units * d1);
                link_params[link].u.pk.base.y = (go_real) (*m_per_length_units * d2);
                link_params[link].u.pk.base.z = (go_real) (*m_per_length_units * d3);
                link_params[link].u.pk.platform.x = (go_real) (*m_per_length_units * d4);
                link_params[link].u.pk.platform.y = (go_real) (*m_per_length_units * d5);
                link_params[link].u.pk.platform.z = (go_real) (*m_per_length_units * d6);
                link_params[link].type = GO_LINK_PK;
            }
            else
            {
                fprintf(stderr, "bad entry: [%s] PK = %s\n", servo_string.c_str(), ini_string.c_str());
                return -1;
            }
        }
        else
        {
            /* no "DH,PP,PK_PARAMETERS" in this section, so we're done */
            break;
        }
    } /* for (link) */
    *link_number = link;

    return 0;
}

static void print_params(std::vector<go_link> & link_params, int link_number)
{
    go_rpy rpy;
    int t;

    for (t = 0; t < link_number; t++) {
        if (GO_LINK_DH == link_params[t].type) {
            printf("%d: %.3f %.3f %.3f %.3f\n", t+1,
                   link_params[t].u.dh.a,
                   link_params[t].u.dh.alpha,
                   link_params[t].u.dh.d,
                   link_params[t].u.dh.theta);
        } else if (GO_LINK_PP == link_params[t].type) {
            go_quat_rpy_convert(&link_params[t].u.pp.pose.rot, &rpy);
            printf("%d: %.3f %.3f %.3f / %.3f %.3f %.3f\n", t+1,
                   link_params[t].u.urdf.pose.tran.x,
                   link_params[t].u.urdf.pose.tran.y,
                   link_params[t].u.urdf.pose.tran.z,
                   rpy.r, rpy.p, rpy.y);
        } else if (GO_LINK_URDF == link_params[t].type) {
            go_quat_rpy_convert(&link_params[t].u.urdf.pose.rot, &rpy);
            printf("%d: %.3f %.3f %.3f / %.3f %.3f %.3f / %.3f %.3f %.3f\n", t+1,
                   link_params[t].u.urdf.pose.tran.x,
                   link_params[t].u.urdf.pose.tran.y,
                   link_params[t].u.urdf.pose.tran.z,
                   rpy.r, rpy.p, rpy.y,
                   link_params[t].u.urdf.axis.x,
                   link_params[t].u.urdf.axis.y,
                   link_params[t].u.urdf.axis.z);
        } else {
            printf("unknown\n");
        }
    }
}
////////////////////////////////////////////////////////////////////////////////
FanucLRMate200idGoSerkin::FanucLRMate200idGoSerkin(std::shared_ptr<RCS::CController> nc)
{
    _nc = nc;
    kin_name="goserkin";

    _pGoKin=std::shared_ptr<genser_struct>(new genser_struct());
    genser_kin_init(_pGoKin.get());

    std::string inipath=getexefolder() + "config/"+_nc->Name() + ".ini";
    if (0 !=ini_load(inipath,
                     &this->m_per_length_units,
                     &this->rad_per_angle_units,
                     &this->link_number,
                     this->link_params))
    {
        fprintf(stderr, "can't load ini file %s\n", inipath.c_str());
        throw RobotControlException(File_not_found,"FanucLRMate200idGoSerkin Constructor" );
    }
    this->m_per_length_units=0.001;
    this->rad_per_angle_units=0.017453292519943302;
    print_params(this->link_params, this->link_number);
    genser_kin_set_parameters(_pGoKin.get(), &link_params[0], link_number);

}

////////////////////////////////////////////////////////////////////////////////
tf::Pose FanucLRMate200idGoSerkin::FK(std::vector<double> joints)
{
    go_pose gopose;
    tf::Pose pose;
    genser_kin_fwd(_pGoKin.get(), &joints[0], &gopose);
    pose=tf::Pose(tf::Quaternion(gopose.rot.x,gopose.rot.y,gopose.rot.z,gopose.rot.s ),
                  tf::Vector3(gopose.tran.x, gopose.tran.y, gopose.tran.z) );
    return pose;

}

////////////////////////////////////////////////////////////////////////////////
int FanucLRMate200idGoSerkin::IK(tf::Pose pose,
                                 std::vector<double> oldjoints, std::vector<double>& joints)
{
    joints.resize(6,0.0);
    int ret;
    go_pose gopose;
    gopose.tran.x=pose.getOrigin().x();
    gopose.tran.y=pose.getOrigin().y();
    gopose.tran.z=pose.getOrigin().z();
    gopose.rot.x=pose.getRotation().x();
    gopose.rot.y=pose.getRotation().y();
    gopose.rot.z=pose.getRotation().z();
    gopose.rot.s=pose.getRotation().w();

    if ((ret=genser_kin_inv(_pGoKin.get(), &gopose, &joints[0]))) {
        std::cerr<< "GoFanucKin::IK Can't run Fanuc LR Mate 200iD inverse kinematics. Error=" << goerr[ret]  <<".\n";
        std::cerr << "Input tf pose" << DumpPoseSimple(pose);
        std::cerr<< "Go pose" << DumpGoPose(gopose);
        return 1;
    }

    return 0;
}

#endif


tf::Pose KinUtils::FK_test(IKinematics *kin, std::vector<double> joints )
{
    return kin->FK(joints);
}

int KinUtils::IK_test(IKinematics *kin,
                      tf::Pose pose,
                      std::vector<double> oldjoints,
                      std::vector<double>& joints)
{
    return kin->IK(pose, oldjoints, joints);

}
#ifdef GOKIN

int KinUtils::SetQuaternionFromRpy(const double roll, const double pitch, const double yaw, tf::Quaternion& q)
{
    go_quat goq;

    go_rpy rpy;
    rpy.r=roll;
    rpy.p=pitch;
    rpy.y=yaw;

    // translate go quaternion into go rpy
    int hr=go_quat_rpy_convert(&goq, &rpy);
    if (GO_RESULT_OK != hr )
        return hr;

    // translate go quaternion into tf quaternion
    q.setX(goq.x);
    q.setY(goq.y);
    q.setZ(goq.z);
    q.setW(goq.s);

    return GO_RESULT_OK;
}

int KinUtils::GetRpyFromQuaternion(const tf::Quaternion& q, double &roll, double & pitch, double & yaw )
{
    // translate tf quaternion into go quaternion
    go_quat goq;
    goq.x=q.x();
    goq.y=q.y();
    goq.z=q.z();
    goq.s=q.w();

    // translate go quaternion into go rpy
    go_rpy rpy;
    int hr=go_rpy_quat_convert(&rpy, &goq);
    if (GO_RESULT_OK != hr )
        return hr;

    // translate go rpy into doubles
    roll=rpy.r;
    pitch=rpy.p;
    yaw=rpy.y;

    return GO_RESULT_OK;
}
#endif


#ifdef Trac_IK
#include <trac_ik/trac_ik.hpp>
#include <kdl_parser/kdl_parser.hpp>

// http://www.orocos.org/kdl/examples
// http://docs.ros.org/jade/api/tf_conversions/html/c++/tf__kdl_8cpp_source.html
static void poseTFToKDL(const tf::Pose& t, KDL::Frame& k) {
    for (unsigned int i = 0; i < 3; ++i)
        k.p[i] = t.getOrigin()[i];
    for (unsigned int i = 0; i < 9; ++i)
        k.M.data[i] = t.getBasis()[i / 3][i % 3];
}

static void poseKDLToTF(const KDL::Frame& k, tf::Pose& t) {
    t.setOrigin(tf::Vector3(k.p[0], k.p[1], k.p[2]));
    t.setBasis(tf::Matrix3x3(k.M.data[0], k.M.data[1], k.M.data[2],
            k.M.data[3], k.M.data[4], k.M.data[5],
            k.M.data[6], k.M.data[7], k.M.data[8]));
}

using namespace KDL;
Fanuc200idTrac_IK::Fanuc200idTrac_IK(std::shared_ptr<RCS::CController> nc) {
    _nc = nc;
    kin_name="tracik";

}
void Fanuc200idTrac_IK::InitUrdf(std::string urdf)
{
    double timeout = 0.005;
    double eps = 1e-5;
    urdf_xml=urdf;

#if 0
    KDL::Tree tree;
    if (!kdl_parser::treeFromString(urdf_xml, tree)) {
        std::cerr << "Could not initialize tree object\n";
        return;
        throw std::runtime_error("Exception:  robot_model.treeFromString(xml,tree)");
    }
    if (!tree.getChain(_rootlinkname,
                       _tiplinkname, chain)) {
        ROS_ERROR("Could not initialize chain object");
        return;
        throw std::runtime_error("Exception: tree.getChain)");
    }
#endif

    if (!ParseURDF(urdf_xml, BaseLink(), EndLink() ))
    {
        LOG_FATAL <<  "Could not parse the xml for kinematic solver" << _groupname << "\n" << std::flush;
    }
    KDL::Chain chain;

    /**
    1: 0.000 0.000 0.330 / 0.000 -0.000 0.000 / 0.000 0.000 1.000
    2: 0.050 0.000 0.000 / 0.000 -0.000 0.000 / 0.000 1.000 0.000
    3: 0.000 0.000 0.330 / 0.000 -0.000 0.000 / 0.000 -1.000 0.000

    4: 0.000 0.000 0.035 / 0.000 -0.000 0.000 / -1.000 0.000 0.000
    5: 0.335 0.000 0.000 / 0.000 -0.000 0.000 / 0.000 -1.000 0.000
    6: 0.080 0.000 0.000 / 0.000 -0.000 0.000 / 1.000 0.000 0.000
    */
    chain.addSegment(Segment(Joint(Vector(0.0,0.0,0.33 ), Vector(0,0,1) , Joint::RotAxis ),
                             KDL::Frame(KDL::Rotation(), KDL::Vector(0.0,0.0,0.33 ))));

    chain.addSegment(Segment(Joint(Vector(0.05,0.0,0.0 ), Vector(0,1,0) , Joint::RotAxis ),
                             KDL::Frame(KDL::Rotation(), KDL::Vector(0.05,0.0,0.0 ))));

    chain.addSegment(Segment(Joint(Vector(0.0,0.0,.33 ), Vector(0,-1,0) , Joint::RotAxis ),
                             KDL::Frame(KDL::Rotation(), KDL::Vector(0.0,0.0,.33 ))));

    chain.addSegment(Segment(Joint(Vector(0.0,0.0,0.035 ), Vector(-1,0,0) , Joint::RotAxis ),
                             KDL::Frame(KDL::Rotation(), KDL::Vector(0.0,0.0,.035 ))));

    chain.addSegment(Segment(Joint(Vector(0.335,0.0,0.0 ), Vector(0,-1,0) , Joint::RotAxis ),
                             KDL::Frame(KDL::Rotation(), KDL::Vector(0.335,0.0,0.0 ))));

    chain.addSegment(Segment(Joint(Vector(0.080,0.0,0.0 ), Vector(1,0,0) , Joint::RotAxis ),
                             KDL::Frame(KDL::Rotation(), KDL::Vector(0.080,0.0,0.0 ))));

    std::vector<KDL::Segment> chain_segs = chain.segments;

    KDL::JntArray j_min;
    KDL::JntArray j_max;
    j_min.resize(chain.getNrOfJoints());
    j_max.resize(chain.getNrOfJoints());

    for(unsigned int i = 0; i < chain_segs.size(); ++i)
    {
        j_min(i)=JointMin()[i];
        j_max(i)=JointMax()[i];
    }


    _pTRAC_IK = std::shared_ptr<TRAC_IK::TRAC_IK> (new TRAC_IK::TRAC_IK(chain,  j_min, j_max));
    //_pTRAC_IK = std::shared_ptr<TRAC_IK::TRAC_IK> (new TRAC_IK::TRAC_IK(_rootlinkname, _tiplinkname, urdf_xml, timeout, eps));

    num_joints = joint_names.size();

    bool valid = _pTRAC_IK->getKDLChain(chain);

    if (!valid) {
        std::cerr <<"There was no valid KDL chain found\n";
        return;
    }
    valid = _pTRAC_IK->getKDLLimits(ll,ul);

    if (!valid) {
        std::cerr << "There were no valid KDL joint limits found";
        return;
    }
    // Set up KDL IK
    //fk_solver= std::shared_ptr<KDL::ChainFkSolverPos_recursive>(new KDL::ChainFkSolverPos_recursive(chain)); // Forward kin. solver
    //   vik_solver=std::shared_ptr<KDL::ChainIkSolverVel_pinv>(new KDL::ChainIkSolverVel_pinv  (chain)); // PseudoInverse vel solver
    //    kdl_solver=std::shared_ptr<KDL::ChainIkSolverPos_NR_JL>(new KDL::ChainIkSolverPos_NR_JL(chain,ll,ul,*(fk_solver.get()), vik_solver, 1, eps)); // Joint Limit Solver

}
using namespace KDL;
//http://docs.ros.org/kinetic/api/kdl_parser/html/kdl__parser_8cpp_source.html
//motoman_joint_s = 0.0, 0.0, 0.41,  0.0,-0.0, 0.0,  0.0, 0.0, 1.0
//motoman_joint_l = 0.0, 0.0, 0.00,  0.0, 0.0, 0.0,  0.0, 1.0, 0.0
//motoman_joint_e = 0.0, 0.0, 0.49,  0.0,-0.0, 0.0,  0.0, 0.0, 1.0
//motoman_joint_u = 0.0, 0.0, 0.00,  0.0, 0.0, 0.0,  0.0,-1.0, 0.0
//motoman_joint_r = 0.0, 0.0, 0.42,  0.0,-0.0, 0.0,  0.0, 0.0,-1.0
//motoman_joint_b = 0.0, 0.0, 0.00,  0.0, 0.0, 0.0,  0.0,-1.0, 0.0
//motoman_joint_t = 0.0, 0.0, 0.18,  0.0,-0.0, 0.0,  0.0, 0.0,-1.0
tf::Pose Fanuc200idTrac_IK::FK(std::vector<double> joints)
{
    KDL::Chain chain;
    _pTRAC_IK->getKDLChain(chain);

    KDL::ChainFkSolverPos_recursive fksolver = KDL::ChainFkSolverPos_recursive(chain);
    unsigned int nj = joints.size();
    KDL::JntArray injoints = KDL::JntArray(nj);

    // Assign  values to the joint positions
    for (unsigned int i = 0; i < nj; i++) {
        injoints(i) = (double) joints[i];
    }

    // Create the frame that will contain the results
    KDL::Frame cartpos;

    // Calculate forward position kinematics
    bool kinematics_status;
    kinematics_status = fksolver.JntToCart(injoints, cartpos);

    if (kinematics_status < 0) {
        std::cerr << "Error: could not calculate forward kinematics :(";
    }

    tf::Pose pose;
    poseKDLToTF(cartpos, pose);
    return pose;
}


int Fanuc200idTrac_IK::IK(tf::Pose pose,
                          std::vector<double> oldjoints,
                          std::vector<double> & joints)
{
    // Create joint array
    KDL::Frame kdl_frame;
    poseTFToKDL(pose,kdl_frame);
    unsigned int nj = oldjoints.size();
    KDL::JntArray injoints = KDL::JntArray(nj);

    // Assign values to the joint positions
    for (unsigned int i = 0; i < nj; i++) {
        injoints(i) = (double) oldjoints[i];
    }

    // Computer IK
    KDL::JntArray outjoints = KDL::JntArray(nj);
    _pTRAC_IK->CartToJnt(injoints, kdl_frame, outjoints);

    // Now assign KDL joints to stl vector joints
    joints.resize(nj, 0.0);
    for (unsigned int i = 0; i < nj; i++) {
        joints[i] = (double) outjoints(i);
    }
    return 0;
}

#endif
