
#include <fanuckin_plugin/fanuckin_plugin.h>

#include <iostream>
#include <stdlib.h>

using namespace RCS;

#include <fanuckin_plugin/fanuc_lrmate200id.h>

////////////////////////////////////////////////////////////////////////////////
template<typename ... Args>
//static std::string strformat( const std::string& format, Args ... args )
static std::string strformat( const char * format, Args ... args )
{
    size_t size = snprintf( nullptr, 0, format, args ... ) + 1; // Extra space for '\0'
    std::unique_ptr<char[]> buf( new char[ size ] );
    snprintf( buf.get(), size, format, args ... );
    return std::string( buf.get(), buf.get() + size - 1 ); // We don't want the '\0' inside
}

////////////////////////////////////////////////////////////////////////////////
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
//MatrixEXd ComputeUrdfTransform(double angle, Eigen::Vector3d axis, Eigen::Vector3d origin, Eigen::Vector3d rotation)
//{
//    Eigen::Matrix3d t33;
//    MatrixEXd m1 = Create4x4IdentityMatrix();
//    MatrixEXd tmp = Create4x4IdentityMatrix();

//    Eigen::Vector3d unit = axis.array().abs();
//    t33 = Eigen::AngleAxisd(axis.sum() * angle, unit);
//    m1.block<3, 3>(0, 0) = t33;
//    tmp.block<3, 1>(0, 3) = origin;
//    return  tmp * m1;  // i dont understand why this matrix multiply order works!
//}

static  tf::Transform CreatetfIdentityMatrix() {
    tf::Transform t;
    t.setIdentity();
    return t;
}
////////////////////////////////////////////////////////////////////////////////
tf::Transform ComputeUrdfTransform(double angle, tf::Vector3 axis, tf::Vector3 origin, tf::Vector3 rotation)
{
    tf::Matrix3x3 t33;
    tf::Transform m1 = CreatetfIdentityMatrix();
    tf::Transform tmp = CreatetfIdentityMatrix();

    tf::Vector3 unit = axis.absolute();
    double sum = axis.getX() + axis.getY() + axis.getZ();
    tf::Quaternion  q (unit, angle*sum);
    //t33 = Eigen::AngleAxisd(axis.sum() * angle, unit);
    //m1.block<3, 3>(0, 0) = t33;
    m1.setRotation(q);
    //tmp.block<3, 1>(0, 3) = origin;
    m1.setOrigin(origin);
    return  tmp * m1;  // i dont understand why this matrix multiply order works!
}

////////////////////////////////////////////////////////////////////////////////
std::string Cfanuckin_plugin::DumpUrdfJoint()
{
    std::stringstream s;
    for (int i = 0; i < this->jointNames.size(); i++) {
        s << "Joint       = " << jointNames[i].c_str() << std::endl;
        s << " Axis       = " << axis[i].getX() << " " << axis[i].getY()<< " " << axis[i].getZ() << "\n";
        s << " XYZ Origin = " << xyzorigin[i].getX() << " " << xyzorigin[i].getY()<< " " << xyzorigin[i].getZ() << "\n";
        s << " RPY Origin = " << rpyorigin[i].getX() << " " << rpyorigin[i].getY()<< " " << rpyorigin[i].getZ() << "\n";

        s << " Has Limits = " << this->jointHasLimit[i] << std::endl;
        s << " Min Pos    = " << jointMin[i] << std::endl;
        ;
        s << " Max Pos    = " << jointMax[i] << std::endl;
        ;
        s << " Max Vel    = " << this->jointVelMax[i] << std::endl;
        ;
        s << " Max Effort = " << jointEffort[i] << std::endl;
        ;
    }
    return s.str();
}

static std::string Dump4x4Matrix( const tf::Transform m)
{
    std::stringstream s;

    for ( size_t i = 0; i < 3; i++ )
    {
        s << boost::format("%8.5f") % m.getBasis().getRow(i).getX() << ":";
        s << boost::format("%8.5f") % m.getBasis().getRow(i).getY() << ":";
        s << boost::format("%8.5f") % m.getBasis().getRow(i).getZ() << ":";
        s << boost::format("%8.5f") % m.getOrigin()[i] << std::endl;
    }
    s << boost::format("%8.5f") % 0.0 << ":"
      << boost::format("%8.5f") % 0.0 << ":"
      << boost::format("%8.5f") % 0.0 << ":"
      << boost::format("%8.5f") % 1.0 << std::endl;
    return s.str( );
}

////////////////////////////////////////////////////////////////////////////////
std::string Cfanuckin_plugin::DumpTransformMatrices()
{
    // Fixme: row versus column major :( May not matter since eigen rectifies when accessing
    std::string dump;
    for (int i = 0; i < jointNames.size(); i++) {
        tf::Transform m = ComputeUrdfTransform(M_PI_2, axis[i], xyzorigin[i], rpyorigin[i]);
        dump += strformat("%dT%d=\n", i, i + 1);

        dump += Dump4x4Matrix(m);
    }
    return dump;
}


////////////////////////////////////////////////////////////////////////////////
Cfanuckin_plugin::Cfanuckin_plugin() : CSerialLinkRobot(this)
{
    bDebug=false;

    // assign calibrated local to world transform to identity
    poseLocal2Wrld=tf::Pose::getIdentity();

    kin =  std::shared_ptr<fanuc_lrmate200id> ( new fanuc_lrmate200id());

}

////////////////////////////////////////////////////////////////////////////////
Cfanuckin_plugin::~Cfanuckin_plugin()
{

}

////////////////////////////////////////////////////////////////////////////////
int Cfanuckin_plugin::calibrate(const std::vector<double>& joints, const tf::Pose pose)
{
    tf::Pose myPose;

    this->FK(joints, myPose);

    poseLocal2Wrld=myPose.inverse()*pose;


    return 0;
}


////////////////////////////////////////////////////////////////////////////////
size_t Cfanuckin_plugin::numJoints()
{
    return 6;
}

////////////////////////////////////////////////////////////////////////////////
int Cfanuckin_plugin::init()
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
        std::cerr<< "Cfanuckin_plugin failed to parse URDF string\n";
        errmsg="Cfanuckin_plugin failed to parse URDF string\n";
        return -1;
    }


    std::cout << DumpTransformMatrices();

    return 0;
}

////////////////////////////////////////////////////////////////////////////////
int Cfanuckin_plugin::debugStream(std::ostream& o)
{
    out.copyfmt(o); //1
    out.clear(o.rdstate()); //2
    out.basic_ios<char>::rdbuf(o.rdbuf());
    return 0;
}

////////////////////////////////////////////////////////////////////////////////
int Cfanuckin_plugin::debug(bool flag)
{
    bDebug=flag;
    return 0;
}


////////////////////////////////////////////////////////////////////////////////
int Cfanuckin_plugin::IK(tf::Pose pose, std::vector<double>&  joints)
{
    errmsg.clear();


    pose = pose * poseLocal2Wrld.inverse();

    // subtract out base pedastral of fanuc...
    double z = pose.getOrigin().getZ()-0.33;
    pose.getOrigin().setZ(z);


    kin->fanuc_lrmate200id_kin_inv(pose, joints);
    return 0;

}

////////////////////////////////////////////////////////////////////////////////
int Cfanuckin_plugin::FK(std::vector<double> thetas, tf::Pose &pose)
{
    errmsg.clear();
    if(thetas.size() != this->numJoints() )
    {
        errmsg ="FK: Incorrect number of joint values\n";
        return Bad_Joints_Size;
    }

#if 0
    std::vector<tf::Transform> AllM;
    tf::Transform t = CreatetfIdentityMatrix();
    A0.clear();
    std::vector<double> joints=thetas;

    // Is this necessary
    //joints[2] = thetas[1] + thetas[2];

    for (int i = 0; i < thetas.size(); i++)
    {
        AllM.push_back(ComputeUrdfTransform(joints[i], axis[i], xyzorigin[i], rpyorigin[i]));
    }

    for (int i = 0; i < thetas.size(); i++)
    {
        t = t * AllM[i];
        A0.push_back(t);
    }

    tf::Pose t0_6pose(t.getRotation(), t.getOrigin()) ;
    //return t0_6pose;
    pose =  poseLocal2Wrld*t0_6pose;
#else
    kin->fanuc_lrmate200id_kin_fwd (&thetas[0], pose);

    // add in base of fanuc...
    double z = pose.getOrigin().getZ()+0.33;
    pose.getOrigin().setZ(z);

    pose =  poseLocal2Wrld*pose;

#endif


    return 0;
}

////////////////////////////////////////////////////////////////////////////////
std::string Cfanuckin_plugin::get(std::string param)
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
        ss << "Cfanuckin_plugin kinematics using orocos kdl solver\n";
        ss << "Parameters Get:\n";
        ss << "\thelp\n";
        ss << "\terror\n";
        ss << "\turdf\n";
        ss << "\turdfile\n";
        ss << "\tbaselink\n";
        ss << "\ttiplink\n";
        ss << "\tjoints\n";
        ss << "Parameters Set:\n";
        ss << "\tdebug\n";
        ss << "\turdf\n";
        ss << "\turdfile\n";
        ss << "\tHandleExceptions\n";
        ss << "\tbaselink\n";
        ss << "\ttiplink\n";
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
    else if(param == "BASELINK")
    {
        return _baselink;
    }
    else if(param == "TIPLINK")
    {
        return _tiplink;
    }
    else if(param == "JOINTS")
    {
        return DumpUrdfJoint();
    }
    return std::string("No get for parameter ") + param;
}


////////////////////////////////////////////////////////////////////////////////
std::string Cfanuckin_plugin::set(std::string param,  std::string value)
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
    return std::string("No match for ") + param;

}
////////////////////////////////////////////////////////////////////////////////
std::string Cfanuckin_plugin::set(std::string param,  void * value)
{
    return std::string("No match for ") + param;
}
