

#include "gokin_plugin/gokin.h"
#include "gokin_plugin/Debug.h"
#include "aprs_headers/Core.h"
#include <gokin_plugin/goserkins.h>
#include <iostream>
#include <stdlib.h>

using namespace RCS;
GoKin goserkin;


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
GoKin::GoKin() : CSerialLinkRobot(this)
{
    bDebug=false;
}

////////////////////////////////////////////////////////////////////////////////
GoKin::~GoKin()
{
    // delete temp file...

}

////////////////////////////////////////////////////////////////////////////////
int GoKin::init(std::string urdf, std::string baselink, std::string tiplink)
{
    errmsg.clear();
    debugStream(std::cout);
    kin_name[0]=0;
    _urdf=urdf;
    _baselink=baselink;
    _tiplink=tiplink;


    if(! parseURDF(urdf, baselink, tiplink))
    {
        std::cerr<< "GoMotoKinBad URDF string\n";
        errmsg="GoMotoKinBad URDF string\n";
        return -1;
    }

    // Now generate the ini file
    ini.clear();
    ini +="\n[GOMOTION]\n";

    ini +="NAME="+robot_name+"\n";
    ini +="VERSION=1.0\n";
    ini +="EXT_INIT_STRING=\n";
    ini +="LENGTH_UNITS_PER_M=1000\n";
    ini +="ANGLE_UNITS_PER_RAD=57.2957795130823\n";

    ini +="\n[TRAJ]\n";

    ini +="SHM_KEY=2201\n";
    ini +="KINEMATICS=genserkins\n";
    ini +="DEBUG=0x0\n";
    ini +="CYCLE_TIME=0.100\n";

    ini +="HOME=0 0 1500 0 0 0\n";  // fixme: dunny

    ini +="\n[SERVO]\n";
    ini +="HOWMANY=";
    ini +=std::to_string(jointNames.size()) + "\n";


    for(size_t i=0; i< jointNames.size(); i++)
    {
        ini +="\n[SERVO_"+std::to_string(i+1)+"]\n";

        ini +="; QUANTITY refers to the quantity of motion effected by the joint,\n";
        ini +="; in SI terms, Length or Angle.\n";
        ini +="NAME=";
        ini +=jointNames[i]+"\n";
        ini +="QUANTITY=Angle\n";
        ini +="URDF_PARAMETERS=";
        for(size_t j=0; j< 3; j++)
            ini +=std::to_string(xyzorigin[i][j]*1000)+" ";
        for(size_t j=0; j< 3; j++)
            ini +=std::to_string(rpyorigin[i][j]*1000)+" ";
        for(size_t j=0; j< 3; j++)
            ini +=std::to_string((double) axis[i][j])+" ";
        ini+="\n";
        ini +="DEBUG=0x0\n";
        ini +="CYCLE_TIME=0.010\n";
        ini +="HOME=0\n";

        ini +="; INPUT_SCALE in user degrees per raw radians\n";
        ini +="INPUT_SCALE=57.295779513082323\n";
        ini +="; OUTPUT_ADJUST in raw radians per user degrees (per sec), space, rad/s offset\n";
        ini +="OUTPUT_SCALE=0.017453292519943\n";


        ini +="MIN_LIMIT="  + std::to_string(jointMin[i]) + "\n";
        ini +="MAX_LIMIT=" + std::to_string(jointMax[i])+ "\n";
    }
    //const char* tf = std::tmpnam(nullptr);
    //const char* tf ="/home/isd/michalos/build/gzrcs/debug/config/motoman_sia20d_new.ini";
//    return Init("/home/isd/michalos/build/gzrcs/debug/config/motoman_sia20d.ini");
//    std::ofstream ofs (tf, std::ofstream::out);
//    ofs << ini;
//    ofs.close();
    char t[] = "/tmp/fileXXXXXX";
    int fd;
    fd = mkstemp(t);
     _inifilename=t;

    write(fd,ini.c_str(),ini.size());
    close(fd);
    return Init(t);
}


////////////////////////////////////////////////////////////////////////////////
int GoKin::Init(std::string filename)
{
    errmsg.clear();
    debugStream(std::cout);
    kin_name[0]=0;
    inifile_name = filename;

    if (GO_RESULT_OK != genser_kin_init(&kins))
    {
      errmsg="Can't init GoKin general serial kinematics\n";
      return Initialization_Failed;
    }

    if (0 != ini_load((char *) inifile_name.c_str(),
                      &m_per_length_units,
                      &rad_per_angle_units,
                      &home_position,
                      (int*) &link_number,
                      (go_link*)  link_params,
                      home_joint,
                       kin_name)) {
      errmsg=strformat( "Can't load GoKin ini file %s\n", inifile_name.c_str());
      return Ini_File_Error;
    }

    print_params(link_params, link_number);

    if (GO_RESULT_OK != genser_kin_set_parameters(&kins, link_params, link_number)) {
        errmsg=strformat( "can't set GoKin kinematics parameters\n");
        return Bad_Parameter;
    }

    return GO_RESULT_OK;

}
////////////////////////////////////////////////////////////////////////////////
int GoKin::debugStream(std::ostream& o)
{
    out.copyfmt(o); //1
    out.clear(o.rdstate()); //2
    out.basic_ios<char>::rdbuf(o.rdbuf());
    return GO_RESULT_OK;
}

////////////////////////////////////////////////////////////////////////////////
int GoKin::debug(bool flag)
{
    bDebug=flag;
    return GO_RESULT_OK;
}

////////////////////////////////////////////////////////////////////////////////
int GoKin::SetWristOffset(double offset)
{
    return GO_RESULT_OK;
}

////////////////////////////////////////////////////////////////////////////////
int GoKin::FK(std::vector<double> joints, tf::Pose &pose)
{

    errmsg.clear();
    go_pose gopose;

    //go_result genser_kin_fwd(void * kins, const go_real *joints,  go_pose * pos)
    if (GO_RESULT_OK != genser_kin_fwd(&kins, &joints[0], &gopose)) {
        errmsg= "Can't run general serial forward kinematics\n";
        return -1;
      }

    pose=tf::Pose(tf::Quaternion(gopose.rot.x,gopose.rot.y,gopose.rot.z,gopose.rot.s ),
                  tf::Vector3(gopose.tran.x, gopose.tran.y, gopose.tran.z) );


    return GO_RESULT_OK;
}


////////////////////////////////////////////////////////////////////////////////
int GoKin::IK(tf::Pose pose, std::vector<double>&  joints)
{

    errmsg.clear();
    size_t n=joints.size();
    // Last joints are an estimate for next joints
//    joints.resize(7,0.0);
    std::vector<double> cpyjoints(joints);
    for(size_t i=n; i< GENSER_MAX_JOINTS; i++)
        cpyjoints.push_back(0.0);


    if(bDebug)
    {
        out<< "GoKin::IK\n" << DumpTfPose(pose);
    }

    go_pose gopose;
    gopose.tran.x=pose.getOrigin().x();
    gopose.tran.y=pose.getOrigin().y();
    gopose.tran.z=pose.getOrigin().z();
    gopose.rot.x=pose.getRotation().x();
    gopose.rot.y=pose.getRotation().y();
    gopose.rot.z=pose.getRotation().z();
    gopose.rot.s=pose.getRotation().w();

    int res =  genser_kin_inv(&kins, &gopose, &cpyjoints[0]);
    if (GO_RESULT_OK != res) {
        std::string ss;
        ss+= strformat( "Can't run general serial inverse kinematics %s\n", go_result_to_string(res));
        ss+= strformat( "tf:\n%s", DumpTfPose(pose).c_str());
        ss+= strformat( "go:\n%s", DumpGoPose(gopose).c_str());
        errmsg=ss;
        return res;
    }
    if(bDebug)
    {
        out<< "IK Joints:\n";
        out<<  strformat("%s", DumpJoints(joints).c_str());
    }
    joints.clear();
    joints.insert(joints.begin(), cpyjoints.begin(), cpyjoints.begin()+n);
    return GO_RESULT_OK;
}


////////////////////////////////////////////////////////////////////////////////
std::string GoKin::get(std::string param)
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
        ss << "GoKin kinematics solver using gomotion genserkins\n";
        ss << "Parameters:\n";
        ss << "\tini\n";
        ss << "\tinifile\n";
        ss << "\turdf\n";
        ss << "\tbase\n";
        ss << "\ttip\n";
        return ss.str();
    }
    else if(param == "INI")
    {
        return ini;
    }
    else if(param == "INIFILE")
    {
        return _inifilename;
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
    return std::string("No get for parameter ") + param;
}


////////////////////////////////////////////////////////////////////////////////
std::string GoKin::set(std::string param,  std::string value)
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
    return std::string("No match for ") + param;

}
