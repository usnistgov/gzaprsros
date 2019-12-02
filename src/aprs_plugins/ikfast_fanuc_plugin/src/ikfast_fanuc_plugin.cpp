

#include <ikfast_fanuc_plugin/ikfast_fanuc_plugin.h>
#include <aprs_headers/Debug.h>

using namespace RCS;

IKFAST_FanucKin ikfast_fanuc_kin;

#define IKFAST_HAS_LIBRARY
#define IKFAST_NO_MAIN
//#define IKFAST_NAMESPACE FanucLRMate200d

#include <ikfast_fanuc_plugin/ikfast.h>
using namespace  ikfast;
//using namespace FanucLRMate200d;

////////////////////////////////////////////////////////////////////////////////
IKFAST_FanucKin::IKFAST_FanucKin() : CSerialLinkRobot((ISerialLinkRobot*) this)
{
    bDebug=false;
    debugStream(std::cout);
}
////////////////////////////////////////////////////////////////////////////////
int IKFAST_FanucKin::debugStream(std::ostream& o)
{
    out.copyfmt(o); //1
    out.clear(o.rdstate()); //2
    out.basic_ios<char>::rdbuf(o.rdbuf());
    return 0;
}
////////////////////////////////////////////////////////////////////////////////
size_t IKFAST_FanucKin::numJoints()
{
    return GetNumJoints ( );
}
////////////////////////////////////////////////////////////////////////////////
int IKFAST_FanucKin::debug(bool flag)
{
    bDebug=flag;
    return 0;
}
////////////////////////////////////////////////////////////////////////////////
int IKFAST_FanucKin::init(std::string urdf, std::string baselink, std::string tiplink)
{

    if(! parseURDF(urdf, baselink, tiplink))
    {
        return -1;
    }
    return 0;
}
////////////////////////////////////////////////////////////////////////////////
int IKFAST_FanucKin::FK(std::vector<double> joints, tf::Pose &pose)
{
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
int IKFAST_FanucKin::allIK(tf::Pose & pose, std::vector<std::vector<double>> &joints)
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
        IKFAST_FanucKin::out <<"Failed to get ik solution:"<<RCS::dumpPoseSimple(pose)<<"\n"<< std::flush;

#if 0
        if(this->bDebug)
             IKFAST_FanucKin::out <<"Failed to get ik solution:"<<RCS::dumpPoseSimple(pose)<<"\n"<< std::flush;

        if(Globals.bHandleExceptions)
            throw RobotControlException(10, _nc->name().c_str());
#endif
        return -1;
    }

    // There are no redundant joints, so no free dof
    std::vector<double> solvalues(GetNumJoints());
    if(bDebug)
        IKFAST_FanucKin::out << "IKFAST IK Solve: " << RCS::dumpPoseSimple(pose).c_str()<<"\n";

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
std::vector<double> IKFAST_FanucKin::nearestJoints(
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
int IKFAST_FanucKin::IK(tf::Pose pose,
         std::vector<double>& newjoints)
{
    // Copy seed
    std::vector<double> oldjoints;
    oldjoints=newjoints;

    // find all ikfast solution
    std::vector<std::vector<double>> allsolutions;
    int bFlag = allIK(pose, allsolutions);

    if(bFlag<0)
        return -1;
    // pick closet solution to seed ....
    newjoints= nearestJoints(oldjoints, allsolutions);
    return 0;
}
