

#include "gokin/gokin.h"
#include "gokin/Debug.h"
#include "aprs_headers/Core.h"
#include <gokin/goserkins.h>
#include <gzrcs/RobotControlException.h>
using namespace gomotion;

GoMotoKin::GoMotoKin()
{
    bDebug=false;
}
int GoMotoKin::Init(std::string filename)
{
    DebugStream(std::cout);
    kin_name[0]=0;
    inifile_name = filename;

    if (GO_RESULT_OK != genser_kin_init(&kins))
    {
      fprintf(stdout, "can't init GoMotoKin general serial kinematics\n");
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
      fprintf(stdout, "can't load GoMotoKin ini file %s\n", inifile_name.c_str());
      return Ini_File_Error;
    }

    print_params(link_params, link_number);

    if (GO_RESULT_OK != genser_kin_set_parameters(&kins, link_params, link_number)) {
      fprintf(stdout, "can't set GoMotoKin kinematics parameters\n");
      return Bad_Parameter;
    }

    return GO_RESULT_OK;

}
int GoMotoKin::DebugStream(std::ostream& o)
{
    out.copyfmt(o); //1
    out.clear(o.rdstate()); //2
    out.basic_ios<char>::rdbuf(o.rdbuf());
    return GO_RESULT_OK;
}

int GoMotoKin::Debug(bool flag)
{
    bDebug=flag;
    return GO_RESULT_OK;
}

int GoMotoKin::SetWristOffset(double offset)
{
    return GO_RESULT_OK;
}

int GoMotoKin::FK(std::vector<double> joints, tf::Pose &pose)
{

    go_pose gopose;

    //go_result genser_kin_fwd(void * kins, const go_real *joints,  go_pose * pos)
    if (GO_RESULT_OK != genser_kin_fwd(&kins, &joints[0], &gopose)) {
        fprintf(stdout, "Can't run general serial forward kinematics\n");
        return 1;
      }

    pose=tf::Pose(tf::Quaternion(gopose.rot.x,gopose.rot.y,gopose.rot.z,gopose.rot.s ),
                  tf::Vector3(gopose.tran.x, gopose.tran.y, gopose.tran.z) );


    if(bDebug)
    {
        out<< "GoMotoKin::FK\n";;
        out<< DumpJoints(joints);
    }

#if 0
    pose=basepose*pose;

    if(bDebug)
        out << "After base add in z:" << gomotion::DumpTfPose(pose);
#endif
    return GO_RESULT_OK;

}


int GoMotoKin::IK(tf::Pose pose, std::vector<double>&  joints)
{

    size_t n=joints.size();
    // Last joints are an estimate for next joints
//    joints.resize(7,0.0);
    std::vector<double> cpyjoints(joints);
    for(size_t i=n; i< GENSER_MAX_JOINTS; i++)
        cpyjoints.push_back(0.0);


    if(bDebug)
    {
        out<< "GoMotoKin::IK\n" << DumpTfPose(pose);
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
        fprintf(stdout, "Can't run general serial inverse kinematics %s\n", go_result_to_string(res));
        fprintf(stdout, "tf:\n%s", DumpTfPose(pose).c_str());
        fprintf(stdout, "go:\n%s", DumpGoPose(gopose).c_str());
        return res;
    }
    if(bDebug)
    {
        fprintf(stdout, "IK Joints:\n");
        fprintf(stdout, "%s", DumpJoints(joints).c_str());
    }
    joints.clear();
    joints.insert(joints.begin(), cpyjoints.begin(), cpyjoints.begin()+n);
    return GO_RESULT_OK;
}
