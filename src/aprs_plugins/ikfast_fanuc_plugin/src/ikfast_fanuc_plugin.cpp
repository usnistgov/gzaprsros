

#include <ikfast_fanuc_plugin/ikfast_fanuc_plugin.h>
#include <aprs_headers/Debug.h>

//using namespace RCS;

//IKFAST_FanucKin ikfast_fanuc_kin;

#define IKFAST_HAS_LIBRARY
#define IKFAST_NO_MAIN
//#define IKFAST_NAMESPACE FanucLRMate200d

#include <ikfast_fanuc_plugin/ikfast.h>
using namespace  ikfast;
//using namespace FanucLRMate200d;


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
RCS::IKFAST_FanucKin::IKFAST_FanucKin() : CSerialLinkRobot((ISerialLinkRobot*) this)
{
    bDebug=false;
    bHandleExceptions=false;
    debugStream(std::cout);
}
////////////////////////////////////////////////////////////////////////////////
int RCS::IKFAST_FanucKin::debugStream(std::ostream& o)
{
    out.copyfmt(o); //1
    out.clear(o.rdstate()); //2
    out.basic_ios<char>::rdbuf(o.rdbuf());
    return 0;
}
////////////////////////////////////////////////////////////////////////////////
size_t RCS::IKFAST_FanucKin::numJoints()
{
    return GetNumJoints ( );
}
////////////////////////////////////////////////////////////////////////////////
int RCS::IKFAST_FanucKin::debug(bool flag)
{
    bDebug=flag;
    return 0;
}
////////////////////////////////////////////////////////////////////////////////
int RCS::IKFAST_FanucKin::init()
{
    errmsg.clear();
    int hr=0;
    if(_urdf.empty() && _urdffile.empty())
    {
        errmsg= "IKFAST_FanucKin missing urdf or urdffile parameters\n";
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


    else if(! parseURDF(_urdf, _baselink, _tiplink))
    {
        errmsg="GoMotoKinBad URDF string\n";
        hr= -1;
    }

    return hr;
}
////////////////////////////////////////////////////////////////////////////////
int RCS::IKFAST_FanucKin::FK(std::vector<double> joints, tf::Pose &pose)
{
    errmsg.clear();
    // DO NOT Handle gearing of joints
    double eetrans[4];
    double eerot[9];

    // FIXME: put in check for zero length joints

    // / Computes the end effector coordinates given the joint values using ikfast. This function is used to double check ik
    // Units? joint angles in radians or degree
    // Elsewhere: Returns the forward kinematic solution given the joint angles (in radians)
    ComputeFk(&joints[0], eetrans, eerot);

    pose.getOrigin().setX(eetrans[0]);
    pose.getOrigin().setY(eetrans[1]);
    pose.getOrigin().setZ(eetrans[2]);
    //pose.setRotation(Convert2Rotation(eerot));
    tf::Matrix3x3 m(eerot[3 * 0 + 0], eerot[3 * 0 + 1], eerot[3 * 0 + 2],
            eerot[3 * 1 + 0], eerot[3 * 1 + 1], eerot[3 * 1 + 2],
    eerot[3 * 2 + 0], eerot[3 * 2 + 1], eerot[3 * 2 + 2]);
    pose.setBasis(m);
    return 0;
}
// http://docs.ros.org/jade/api/moveit_msgs/html/msg/PositionIKRequest.html
//http://docs.ros.org/hydro/api/ric_mc/html/GetPositionIK_8h_source.html

//////////////////////////////////////////////////////////////////////////////
int RCS::IKFAST_FanucKin::allIK(tf::Pose & pose, std::vector<std::vector<double>> &joints)
{

    // Inverse kinematics
    ikfast::IkSolutionList<double> solutions;

    // std::vector<double> vfree(GetNumFreeParameters());
    std::vector<double> vfree;

    double eetrans[3] = {pose.getOrigin().x(), pose.getOrigin().y(), pose.getOrigin().z()};
    double eerot[9];

 //    Convert2RotationMatrix(pose.getRotation(), eerot);
    tf::Matrix3x3 m3x3 = pose.getBasis ();
    eerot[3 * 0 + 0] = m3x3[0][0];
    eerot[3 * 0 + 1] = m3x3[0][1];
    eerot[3 * 0 + 2] = m3x3[0][2];
    eerot[3 * 1 + 0] = m3x3[1][0];
    eerot[3 * 1 + 1] = m3x3[1][1];
    eerot[3 * 1 + 2] = m3x3[1][2];
    eerot[3 * 2 + 0] = m3x3[2][0];
    eerot[3 * 2 + 1] = m3x3[2][1];
    eerot[3 * 2 + 2] = m3x3[2][2];
    bool bSuccess = ComputeIk(eetrans, eerot, vfree.size() > 0 ? &vfree[0] : NULL, solutions);

    //TODO(Fix Singularity issue in FanucLRMate200idFastKinematics::AllPoseToJoints)
    if (!bSuccess) {
        std::stringstream ss;
        ss <<"Failed to get ik solution:"<<dumpPoseSimple(pose)<<"\n"<< std::flush;
        IKFAST_FanucKin::out << ss.str();
        errmsg=ss.str();

#if 0
        if(this->bDebug)
             IKFAST_FanucKin::out <<"Failed to get ik solution:"<<RCS::dumpPoseSimple(pose)<<"\n"<< std::flush;

        if(bHandleExceptions)
            throw RobotControlException(10, _nc->name().c_str());
#endif
        return -1;
    }

    // There are no redundant joints, so no free dof
    std::vector<double> solvalues(GetNumJoints());
    if(bDebug)
        IKFAST_FanucKin::out << "IKFAST IK Solve: " << dumpPoseSimple(pose).c_str()<<"\n";

    for (std::size_t i = 0; i < solutions.GetNumSolutions(); ++i) {
        const ikfast::IkSolutionBase<double> & sol = solutions.GetSolution(i);

        std::vector<double> vsolfree(sol.GetFree().size());
        sol.GetSolution(&solvalues[0], vsolfree.size() > 0 ? &vsolfree[0] : NULL);

        // No gearing done
        std::vector<double> jnts;
        for (std::size_t j = 0; j < solvalues.size(); ++j) {
            jnts.push_back(solvalues[j]);
        }
        joints.push_back(jnts);

        if(bDebug)
            IKFAST_FanucKin::out << "IK Solution[" << i <<"]="<<vectorDump( solvalues)<<"\n";
    }
    return solutions.GetNumSolutions();
}


//////////////////////////////////////////////////////////////////////////////
std::vector<double> RCS::IKFAST_FanucKin::nearestJoints(
        std::vector<double> oldjoints,
        std::vector<std::vector<double>> &newjoints)
{
    std::vector<double> finaljoints;
    GetClosestSolution(newjoints,oldjoints, finaljoints);
    return finaljoints;
#if 0
     Eigen::VectorXd oldjointvec = ConvertJoints(oldjoints);
    double min = std::numeric_limits<double>::infinity();
    size_t index = 0;
    for (size_t i = 0; i < newjoints.size(); i++) {
        Eigen::VectorXd newjointvec = ConvertJoints(newjoints[i]);
        double diff = (oldjointvec - newjointvec).norm();
        if (diff < min) {
            min = diff;
            index = i;
        }
    }
    // save "best" solution - closset ignoring importance of wrist
    finaljoints.insert(finaljoints.begin(), newjoints[index].begin(), newjoints[index].end());
    return finaljoints;
#endif
}

//////////////////////////////////////////////////////////////////////////////
int RCS::IKFAST_FanucKin::IK(tf::Pose pose,
         std::vector<double>& newjoints)
{
    // Clear error message
    errmsg.clear();

    // Copy seed
    std::vector<double> oldjoints;
    oldjoints=newjoints;

    // find all ikfast solution
    std::vector<std::vector<double>> allsolutions;
    int bFlag = allIK(pose, allsolutions);

    if(bFlag<0)
    {
        errmsg+="IKFAST_FanucKin::IK failed\n";
        return -1;
    }

    // pick closet solution to seed ....
    newjoints= nearestJoints(oldjoints, allsolutions);
    return 0;
}
////////////////////////////////////////////////////////////////////////////////
std::string RCS::IKFAST_FanucKin::get(std::string param)
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
        ss << "IKFAST_FanucKin kinematics solver using ikfast kinematic solver for fanuc 200id\n";
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
    else if(param == "ALL")
    {
        std::stringstream ss;
        ss << "ROBOTNAME="<< robot_name << std::endl;
        ss << "NUMJOINTS="<< std::to_string(numJoints())<< std::endl;
        ss << "JOINTS=";
        for(size_t i=0; i< jointNames.size(); i++)
        {
            if(i>0)
                ss<< ",";
            ss << jointNames[i];
        }
        ss<< std::endl;
        ss << "INIFILE="<< _inifilename << std::endl;
        ss << "TIPLINK="<< _tiplink << std::endl;
        ss << "BASELINK="<< _baselink << std::endl;
        ss << "URDFFILE="<< _urdffile << std::endl;
        ss << "ERROR="<< errmsg << std::endl;

        return ss.str();
    }
    return std::string("No get for parameter ") + param;
}


////////////////////////////////////////////////////////////////////////////////
std::string RCS::IKFAST_FanucKin::set(std::string param,  std::string value)
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
std::string RCS::IKFAST_FanucKin::set(std::string param,  void * value)
{
    return std::string("No match for ") + param;
}
