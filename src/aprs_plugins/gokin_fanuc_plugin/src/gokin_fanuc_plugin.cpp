

#include <gokin_fanuc_plugin/gokin_fanuc_plugin.h>
#include <gokin_fanuc_plugin/fanuc_lrmate200id.h>
#include <aprs_headers/Debug.h>

using namespace RCS;

/*!
 * \brief Convert urdf::Vector into an tf vector.
 * \param v is a urdf::Vector3t.
 * \return  tf::Vector3 vector.
 */

static tf::Vector3 Convert (urdf::Vector3 v) {
    return tf::Vector3(v.x, v.y, v.z);
}

static tf::Pose Convert(urdf::Pose pose) {
    // http://answers.ros.org/question/193286/some-precise-definition-or-urdfs-originrpy-attribute/
    tf::Quaternion q(pose.rotation.w, pose.rotation.x, pose.rotation.y, pose.rotation.z);
    return tf::Pose (q, tf::Vector3(pose.position.x, pose.position.y, pose.position.z));
}

static urdf::Vector3 Convert (tf::Vector3 v) {
    return urdf::Vector3(v.getX(), v.getY(), v.getZ());
}

static urdf::Pose Convert(tf::Pose pose) {
    // http://answers.ros.org/question/193286/some-precise-definition-or-urdfs-originrpy-attribute/
    urdf::Pose p;
    p.rotation=urdf::Rotation(pose.getRotation().getX(), pose.getRotation().getY(),pose.getRotation().getZ(),pose.getRotation().getW());
    p.position=urdf::Vector3(pose.getOrigin().getX(),pose.getOrigin().getY(),pose.getOrigin().getZ());
    return p;
}

////////////////////////////////////////////////////////////////////////////////
gokin_fanuc::gokin_fanuc() : CSerialLinkRobot((ISerialLinkRobot*) this)
{
    bDebug=false;
    bHandleExceptions=false;
    debugStream(std::cout);
    robot_name="fanuc lrmate200id";

    fanuckin = std::shared_ptr<fanuc_lrmate200id> ( new fanuc_lrmate200id());


}
////////////////////////////////////////////////////////////////////////////////
int gokin_fanuc::debugStream(std::ostream& o)
{
    out.copyfmt(o); //1
    out.clear(o.rdstate()); //2
    out.basic_ios<char>::rdbuf(o.rdbuf());
    return 0;
}
////////////////////////////////////////////////////////////////////////////////
size_t gokin_fanuc::numJoints()
{
    return 6;
}
////////////////////////////////////////////////////////////////////////////////
int gokin_fanuc::debug(bool flag)
{    std::shared_ptr<fanuc_lrmate200id> fanuckin;

    bDebug=flag;
    return 0;
}
////////////////////////////////////////////////////////////////////////////////
int gokin_fanuc::init()
{
    errmsg.clear();
    int hr=0;

    return hr;
}
////////////////////////////////////////////////////////////////////////////////
int gokin_fanuc::FK(std::vector<double> joints, tf::Pose &pose)
{
    errmsg.clear();
    urdf::Pose p;
    fanuckin->fwd_kin(joints,p);
    p.position.x/=1000.0;
    p.position.y/=1000.0;
    p.position.z/=1000.0;
    pose=::Convert(p);

    return 0;
}


//////////////////////////////////////////////////////////////////////////////
int gokin_fanuc::IK(tf::Pose pose,
         std::vector<double>& newjoints)
{
    // Clear error message
    errmsg.clear();

    urdf::Pose p = ::Convert(pose);
    p.position.x*=1000.0;
    p.position.y*=1000.0;
    p.position.z*=1000.0;

    fanuckin->inv_kin (p,newjoints);

    return 0;
}
////////////////////////////////////////////////////////////////////////////////
std::string gokin_fanuc::get(std::string param)
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
        ss << "gokin_fanuc kinematics solver using ikfast kinematic solver for fanuc 200id\n";
        ss << "Parameters Get:\n";
        ss << "\thelp\n";
        ss << "\terror\n";
        ss << "\turdf\n";
        ss << "\turdffile\n";
        ss << "\tbaselink\n";
        ss << "\ttiplink\n";
        ss << "Parameters Set:\n";
        ss << "\tdebug\n";
        ss << "\tHandleExceptions\n";
        ss << "\turdf\n";
        ss << "\turdffile\n";
        ss << "\tbaselink\n";
        ss << "\ttiplink\n";
        return ss.str();
    }
    else if(param == "URDF")
    {
        return _urdf;
    }
    else if(param == "URDFFILE")
    {
        return _urdffile;
    }
    else if(param == "BASELINK")
    {
        return _baselink;
    }
    else if(param == "TIPLINK")
    {
        return _tiplink;
    }
    return std::string("No get for parameter ") + param;
}


////////////////////////////////////////////////////////////////////////////////
std::string gokin_fanuc::set(std::string param,  std::string value)
{
    const char* ws = " \t\n\r";
    errmsg.clear();

    param.erase(param.find_last_not_of(ws) + 1);
    param.erase(0, param.find_first_not_of(ws));
    std::transform(param.begin(), param.end(),param.begin(), ::toupper);
    if(param == "DEBUG")
    {
        bDebug = std::stoi(value);
    }
    else if(param == "HANDLEEXCEPTIONS")
    {
        bHandleExceptions = std::stoi(value);
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
    else
    {
        errmsg=std::string("No match for ") + param;
    }
    return errmsg;
}


////////////////////////////////////////////////////////////////////////////////
std::string gokin_fanuc::set(std::string param,  void * value)
{
    return std::string("No match for ") + param;
}
