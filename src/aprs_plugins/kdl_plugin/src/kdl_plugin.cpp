
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


#include <iostream>
#include <stdlib.h>
#include <kdl_plugin/kdl_plugin.h>

using namespace RCS;
//Ckdl_plugin kdl_plugin;
static bool readFile (std::string filename, std::string & contents)
{
    std::ifstream     in(filename.c_str( ), std::ifstream::in );
    std::stringstream buffer;

    if(!in.is_open())
    {
        return false;
    }
    buffer << in.rdbuf( );
    contents = buffer.str( );
    return true;
}
////////////////////////////////////////////////////////////////////////////////
static KDL::Frame tfPoseToKDLFrame(tf::Pose m)
{
    KDL::Frame k;
    k.p[0] = m.getOrigin().x();
    k.p[1] = m.getOrigin().y();
    k.p[2] = m.getOrigin().z();

    //Quaternion (double x, double y, double z, double w)
    k.M = KDL::Rotation::Quaternion( m.getRotation().x(), m.getRotation().y(), m.getRotation().z(),m.getRotation().w());
    return k;
}

////////////////////////////////////////////////////////////////////////////////
static tf::Pose  KDLFrameToTfPose(KDL::Frame k)
{
//    tf::Pose m;

//    m.getOrigin().setX(k.p[0]);
//    m.getOrigin().setY(k.p[1]);
//    m.getOrigin().setZ(k.p[2]);

    double x,y,z,w;
    k.M.GetQuaternion(x,y,z,w);
//    m.setRotation(tf::Quaternion(x,y,z,w));
    tf::Pose p(tf::Quaternion(x,y,z,w), tf::Vector3(k.p[0],k.p[1],k.p[2]));
    return p;
}

////////////////////////////////////////////////////////////////////////////////
static std::vector<double> KdlJointArray2Vector(KDL::JntArray joint_list)
{
    std::vector<double>  joints;

    // Fill in KDL joint list
    for(size_t i=0; i< joint_list.data.size(); i++)
        joints.push_back(joint_list(i));

    return joints;
}
////////////////////////////////////////////////////////////////////////////////
static KDL::JntArray  vectorToKdlJointArray(std::vector<double> joints)
{
    KDL::JntArray  joint_list(joints.size());

    // Fill in KDL joint list
    for(size_t i=0; i< joints.size(); i++)
        joint_list(i)=joints[i];
    return joint_list;
}

////////////////////////////////////////////////////////////////////////////////
Ckdl_plugin::Ckdl_plugin() : CSerialLinkRobot(this)
{
    bDebug=false;
}

////////////////////////////////////////////////////////////////////////////////
Ckdl_plugin::~Ckdl_plugin()
{
    // delete temp file...

}
size_t Ckdl_plugin::numJoints()
{
    return chain.getNrOfJoints();
}

////////////////////////////////////////////////////////////////////////////////
int Ckdl_plugin::init()
{

    int hr;
    errmsg.clear();
    maxIterations=1000;
    epsilon=0.001;


    debugStream(std::cout);
    //kin_name[0]=0;
    std::string kin_name;

    if(_urdf.empty() && _urdffile.empty())
    {
        errmsg= "GoKin missing urdf or urdffile parameters\n";
        hr= Bad_Parameter;

    }
    else if(_urdffile.empty() )
    {
        errmsg=std::string("Urdffile parameter not found ")+_urdffile;
        hr= Bad_Parameter;
    }
    else if( !readFile(_urdffile,_urdf))
    {
        errmsg=std::string("Urdf file not found ")+_urdffile;
        hr= Bad_Parameter;
    }

    if(hr<0)
        return hr;

    if(! parseURDF(_urdf, _baselink, _tiplink))
    {
        std::cerr<< "Ckdl_plugin failed to parse URDF string\n";
        errmsg="Ckdl_plugin failed to parse URDF string\n";
        return -1;
    }

    // reference https://wiki.ros.org/kdl_parser/Tutorials/Start%20using%20the%20KDL%20parser
    if (!kdl_parser::treeFromString(_urdf, tree))
    {
        std::cerr<<"Failed to construct kdl tree\n";
        errmsg="Failed to construct kdl tree\n";
        return -1;
    }

    if (!tree.getChain(_baselink, _tiplink, chain))
    {
        std::stringstream ss;
        ss << "Couldn't find KDL chain from "<< _baselink<<  " to " << _tiplink << "\n";
        std::cerr<< ss.str();
        errmsg= ss.str();
        return -1;
    }

    // Build Solversjnt_pos_out
     fk_solver = new KDL::ChainFkSolverPos_recursive(chain);
     ik_solver_vel = new KDL::ChainIkSolverVel_pinv(chain);

     KDL::JntArray joint_min= vectorToKdlJointArray(this->jointMin);
     KDL::JntArray joint_max= vectorToKdlJointArray(this->jointMax);

     ik_solver_pos = new KDL::ChainIkSolverPos_NR_JL(chain, joint_min, joint_max,
             *fk_solver, *ik_solver_vel, maxIterations, epsilon);

     return 0;
}

////////////////////////////////////////////////////////////////////////////////
int Ckdl_plugin::debugStream(std::ostream& o)
{
    out.copyfmt(o); //1
    out.clear(o.rdstate()); //2
    out.basic_ios<char>::rdbuf(o.rdbuf());
    return 0;
}

////////////////////////////////////////////////////////////////////////////////
int Ckdl_plugin::debug(bool flag)
{
    bDebug=flag;
    return 0;
}


////////////////////////////////////////////////////////////////////////////////
int Ckdl_plugin::IK(tf::Pose pose, std::vector<double>&  joints)
{
    errmsg.clear();
    KDL::JntArray jnt_pos_in=vectorToKdlJointArray(joints);
    KDL::Frame F_dest=tfPoseToKDLFrame(pose);

    // output
    KDL::JntArray jnt_pos_out;

    int ik_valid = ik_solver_pos->CartToJnt(jnt_pos_in, F_dest, jnt_pos_out);
    if(ik_valid<0)
    {
        errmsg="KDL CartToJnt failed";
        return -1;
    }
    joints=KdlJointArray2Vector(jnt_pos_out);
    return 0;

}

////////////////////////////////////////////////////////////////////////////////
int Ckdl_plugin::FK(std::vector<double> joints, tf::Pose &pose)
{
    errmsg.clear();
    KDL::JntArray kdL_current_joints=vectorToKdlJointArray(joints);
    KDL::Frame kdl_pose_frame;
    fk_solver->JntToCart(kdL_current_joints,kdl_pose_frame);

    pose = KDLFrameToTfPose(kdl_pose_frame);
    return 0;
}

////////////////////////////////////////////////////////////////////////////////
std::string Ckdl_plugin::get(std::string param)
{
    const char* ws = " \t\n\r";

    param.erase(param.find_last_not_of(ws) + 1);
    param.erase(0, param.find_first_not_of(ws));
    std::transform(param.begin(), param.end(),param.begin(), ::toupper);
    if(param == "ERROR")
    {
        return errmsg;
    }
    else if(param == "HELP")
    {
        std::stringstream ss;
        ss << "Ckdl_plugin kinematics using orocos kdl solver\n";
        ss << "Parameters Get:\n";
        ss << "\turdf\n";
        ss << "\turdf\n";
        ss << "\tbase\n";
        ss << "\ttip\n";
        ss << "\tmaxiterations\n";
        ss << "\tepsilon\n";
        ss << "Parameters Set:\n";
        ss << "\tdebug\n";
        ss << "\tHandleExceptions\n";
        ss << "\tmaxiterations\n";
        ss << "\tepsilon\n";
        return ss.str();
    }
    else if(param == "URDFFILE")
    {
        return _urdffile;
    }
    else if(param == "URDF")
    {
        return _urdf;
    }
    else if(param == "BASE")
    {
        return _baselink;
    }
    else if(param == "TIP")
    {
        return _tiplink;
    }
    else if(param == "MAXITERATIONS")
    {
        return std::to_string( maxIterations);
    }
    else if(param == "EPSILON")
    {
        return std::to_string(epsilon);
    }
    return std::string("No get for parameter ") + param;
}


////////////////////////////////////////////////////////////////////////////////
std::string Ckdl_plugin::set(std::string param,  std::string value)
{
    const char* ws = " \t\n\r";

    param.erase(param.find_last_not_of(ws) + 1);
    param.erase(0, param.find_first_not_of(ws));
    std::transform(param.begin(), param.end(),param.begin(), ::toupper);
    if(param == "DEBUG")
    {
        bDebug = std::stoi(value);
        return "";
    }
    else if(param == "HANDLEEXCEPTIONS")
    {
        bHandleExceptions = std::stoi(value);
        return "";
    }
    else if(param == "URDF")
    {
        _urdf=value;
    }
    else if(param == "URDFFILE")
    {
        _urdffile=value;

    }
    else if(param == "BASELINK")
    {
        _baselink=value;
    }
    else if(param == "TIPLINK")
    {
        _tiplink=value;
    }
    else if(param == "MAXITERATIONS")
    {
        // this is actually an error?
        maxIterations= std::stod(value);
    }
    else if(param == "EPSILON")
    {
        // this is actually an error?
        epsilon= std::stod(value);
    }
    return std::string("No match for ") + param;

}
////////////////////////////////////////////////////////////////////////////////
std::string Ckdl_plugin::set(std::string param,  void * value)
{
    return std::string("No match for ") + param;
}
