#include <nist_robotsnc/Fanuc/fanuc_lrmate200id_tracik.h>
#include <nist_robotsnc/Fanuc/trac_ik.hpp>
#include <nist_robotsnc/Globals.h>
#include <nist_robotsnc/RobotControlException.h>

#include <nist/Conversions.h>
#include <nist/Debug.h>

//////////////////////////////////////////////////////////////////////////////
fanuc_lrmate200id_tracik::fanuc_lrmate200id_tracik(std::shared_ptr<RCS::CController> nc)
{
    this->nc=nc;
    kin_name="tracik";

    basepedestal_tfpose= tf::Pose(tf::Quaternion(0,0,0,1), tf::Vector3(0,0,-0.30));
//    basepedestal_tfpose= tf::Pose(tf::Quaternion(0,0,0,1), tf::Vector3(0,0,0));
    bDebug=true;
    std::string destfolder = getexefolder();
    out.open(destfolder+"trac_ik.log", std::ofstream::out);
}
//////////////////////////////////////////////////////////////////////////////
void fanuc_lrmate200id_tracik::InitUrdf(std::string urdf)
{
    double timeout = 0.005;
    double eps = 1e-5;
    urdf_xml=urdf;

    if (!ParseURDF(urdf_xml, BaseLink(), EndLink() ))
    {
        LOG_FATAL <<  "Could not parse the xml for kinematic solver" << _groupname << "\n" << std::flush;
    }
    tracik_solver = std::shared_ptr<TRAC_IK::TRAC_IK> (new
                                                       TRAC_IK::TRAC_IK(_rootlinkname, _tiplinkname, urdf_xml, timeout, eps));

    num_joints = joint_names.size();

    bool valid = tracik_solver->getKDLChain(chain);

    if (!valid) {
        std::cerr <<"There was no valid KDL chain found\n";
        return;
    }
    valid = tracik_solver->getKDLLimits(ll,ul);

    if (!valid) {
        std::cerr << "There were no valid KDL joint limits found";
        return;
    }
    // Set up KDL IK
    fk_solver= std::shared_ptr<KDL::ChainFkSolverPos_recursive>(new KDL::ChainFkSolverPos_recursive(chain)); // Forward kin. solver
    //   vik_solver=std::shared_ptr<KDL::ChainIkSolverVel_pinv>(new KDL::ChainIkSolverVel_pinv  (chain)); // PseudoInverse vel solver
    //    kdl_solver=std::shared_ptr<KDL::ChainIkSolverPos_NR_JL>(new KDL::ChainIkSolverPos_NR_JL(chain,ll,ul,*(fk_solver.get()), vik_solver, 1, eps)); // Joint Limit Solver

}


//////////////////////////////////////////////////////////////////////////////
fanuc_lrmate200id_tracik::~fanuc_lrmate200id_tracik(void)
{

}

int fanuc_lrmate200id_tracik::DebugStream(std::ostream& o)
{
    out.copyfmt(o); //1
    out.clear(o.rdstate()); //2
    out.basic_ios<char>::rdbuf(o.rdbuf());
    return 0;
}

int fanuc_lrmate200id_tracik::Debug(bool flag)
{
    bDebug=flag;
    return 0;
}

//////////////////////////////////////////////////////////////////////////////
tf::Pose fanuc_lrmate200id_tracik::FK(std::vector<double> joints)
{
    KDL::JntArray joint_list(chain.getNrOfJoints());

    // Make sure KDL and FK joint list of same size
    if(joints.size() != chain.getNrOfJoints() )
    {
        if(Globals.bHandleExceptions)
            throw RobotControlException(Mismatched_KDL_joint_size_FK_Joint_size,
                                        _nc->Name().c_str());
        return tf::Identity();
    }

    // Fill in KDL joint list
    for(size_t i=0; i< joints.size(); i++)
        joint_list(i)=joints[i];

    KDL::Frame end_effector_pose;
    fk_solver->JntToCart(joint_list, end_effector_pose);

    // Convert and return pose
    tf::Pose pose = RCS::Convert<KDL::Frame, tf::Pose >(end_effector_pose);
    //return pose * basepedestal_tfpose;
    if(this->bDebug)
    {
        out << "FK tracpose               " << RCS::DumpPoseSimple(pose )<<"\n";
        out << "FK tracbasepedestal_tfpose" << RCS::DumpPoseSimple(pose * basepedestal_tfpose)<<"\n";
    }
    return pose* basepedestal_tfpose;
}

//////////////////////////////////////////////////////////////////////////////
int fanuc_lrmate200id_tracik::IK(tf::Pose inpose,
                                 std::vector<double> oldjoints, std::vector<double>&newjoints)
{
    int rc;
    tf::Pose pose = inpose * basepedestal_tfpose.inverse();
    if(this->bDebug)
    {
        out << "IK in tracpose               " << RCS::DumpPoseSimple(inpose )<<"\n";
        out << "IK base tracpose          " << RCS::DumpPoseSimple(pose )<<"\n";
    }
    // Convert tp pose to KDL pose
    KDL::Frame end_effector_pose = RCS::Convert<tf::Pose, KDL::Frame >(pose);
    // KDL::JntArray nominal= Convert<std::vector<double>, KDL::JntArray >(oldjoints);
    KDL::JntArray result;

    KDL::JntArray nominal(chain.getNrOfJoints());

    for (uint j=0; j<nominal.data.size(); j++) {
        //nominal(j) = (ll(j)+ul(j))/2.0;
        nominal(j) = oldjoints[j];
    }

    double eps = 1e-5;

#if 0
    KDL::ChainIkSolverVel_pinv vik_solver(chain); // PseudoInverse vel solver
    KDL::ChainIkSolverPos_NR_JL kdl_solver(chain,ll,ul,*fk_solver, vik_solver, 1, eps); //
    KDL::JntArray q;
    boost::posix_time::ptime start_time;
    boost::posix_time::time_duration diff;

    result=nominal; // start with nominal
    start_time = boost::posix_time::microsec_clock::local_time();
    double timeout= 0.05;
    double elapsed = 0;
    do {
        q=result; // when iterating start with last solution
        rc=kdl_solver.CartToJnt(q,end_effector_pose,result);
        diff = boost::posix_time::microsec_clock::local_time() - start_time;
        elapsed = diff.total_nanoseconds() / 1e9;
    } while (rc < 0 && elapsed < timeout);

#else
    rc=tracik_solver->CartToJnt(nominal,end_effector_pose,result);

#endif
    newjoints= RCS::Convert<KDL::JntArray,std::vector<double> >(result);
    if(this->bDebug)
        out << "IKTracik Joints=          " << RCS::VectorDump(newjoints,",") << "\n";
    return rc;  // -1 for error  >=0 for sucess

}

//////////////////////////////////////////////////////////////////////////////
bool fanuc_lrmate200id_tracik::IsSingular(tf::Pose pose, double threshold)
{
    return false;
}
