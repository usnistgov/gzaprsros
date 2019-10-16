
#ifndef _KINEMATICS_H
#define _KINEMATICS_H

/*
DISCLAIMER:
This software was produced by the National Institute of Standards
and Technology (NIST), an agency of the U.S. government, and by statute is
not subject to copyright in the United States.  Recipients of this software
assume all responsibility associated with its operation, modification,
maintenance, and subsequent redistribution.

See NIST Administration Manual 4.09.07 b and Appendix I.
 */

//#include <boost/shared_ptr.hpp>
//#include <boost/thread/mutex.hpp>

#include <memory>
#include <numeric>

#include <urdf/model.h>
#include <sstream>

#include "aprs_headers/Core.h"
#include "aprs_headers/IRcs.h"
#include "aprs_headers/Conversions.h"
#include "aprs_headers/Debug.h"

#include "gzrcs/Globals.h"


#define _USE_MATH_DEFINES
#include <math.h>

namespace RCS {
struct CController;
};


/**
 * \brief The IKinematics provides is an abstract class with pure virtual functions that are
 * overriden by actual kinematic implementations.
 *
 * */
class IKinematics {
protected:

    std::shared_ptr<RCS::CController> _nc; /// set by derived class
    size_t _numJoints;
    std::string _groupname;
    bool parseURDF(std::string xml_string, std::string base_link, std::string end_link);

public:
    std::string kinName;

public:
    VAR(std::string, urdfXml);
    VAR(std::string, baseLink );
    VAR(std::string, endLink);

    VAR(std::vector<tf::Vector3>, axis);
    VAR(std::vector<tf::Vector3>, xyzorigin);
    VAR(std::vector<tf::Vector3>, rpyorigin);

    VAR(std::vector<std::string>, jointNames);
    VAR(std::vector<std::string>, linkNames);
    VAR(std::string, prefix);
    VAR(std::vector<double>, jointMin);
    VAR(std::vector<double>,jointMax);
    VAR(std::vector<bool>,jointHasLimit);
    VAR(std::vector<double>,jointVelMax);
    VAR(std::vector<double>,jointEffort);

    IKinematics()
    {

    }

    size_t numJoints() {
#if 0
        assert(joint_names.size() != 0);
#endif
        return jointNames().size();
    }
    


    /**
     * @brief VerifyLimits check set of joints against position max/mins
     * @param joints set of joint position values
     * @return true if within limits
     */
    virtual bool verifyLimits(std::vector<double> joints)
    {
        return true;
    }


    /*!
     * \brief FK performs the forward kinematics using the joint values of the robot provided.
     * \param vector of all robot joint values in doubles.
     * \return corresponding Cartesian robot pose of end  effector.
     */
    virtual tf::Pose FK(std::vector<double> jv) = 0;
    /*!
     * \brief IK performs the inverse kinematics using the Cartesian pose provided.
     * \param  Cartesian robot pose of end  effector.
     * \param  oldjoints seed joint values to use as best guess for IK joint values.
     * \return vector of all robot joint values in doubles.
     */
    virtual int IK(tf::Pose pose,
                   std::vector<double> oldjoints, std::vector<double>&) = 0;


    /*!
     * \brief AllIK solves  the inverse kinematics to find all solutions using the Cartesian pose provided.
     * \param  Cartesian robot pose of end  effector.
     * \param  vector of double vectos to hold all the IK joint solutions.
     * \return number of solutions found.
     */
    virtual int allIK(tf::Pose & pose,
                      std::vector<std::vector<double> > & newjoints)
    {
        assert(0);
        return 0;
    }

    /*!
     * \brief nearestJoints finds the joint set that is closest to the old joints.
     * \param  old seed  joint values to use as best guess for IK joint values.
     * \param  vector of double vectos that holds all the IK joint solutions.
     * \return vector of doubles with closest set to seed joints.
     */
    virtual std::vector<double> nearestJoints(
            std::vector<double> oldjoints,
            std::vector<std::vector<double> > & newjoints)
    {
        assert(0);
        return std::vector<double> ();
    }

    /*!
     * \brief Init is necessary for ROS to initialize it kinematics using robot model .
     * \param  groupname name of  chained joints in robot model. e.g., 0T6
     * \param  tiplinkname name of end effector link in robot model. e.g., 6 in 0T6
     * \param  rootlinkname name of base link in robot model. e.g., 0 in 0T6
     */

    virtual void init(
            std::string groupname,
            std::string tiplinkname,
            std::string rootlinkname) {
        _groupname = groupname;
        endLink() = tiplinkname;
        baseLink() = rootlinkname;
    }

    std::vector<unsigned long> allJointNumbers() {

        std::vector<unsigned long> jointnum(numJoints());
        std::iota(jointnum.begin(), jointnum.end(), 0); // adjusted already to 0..n-1
        return jointnum;
    }

    /*!
     * \brief Returns true if the determinant of the jacobian is near zero. .
     * \param  groupname name of  chained joints in robot model.
     * \param  eelinkname name of end effector joint in robot model.
     */
    virtual bool isSingular(tf::Pose pose, double threshold) {
        return false;
    }


    /*!
     * \brief Initialize kinematics using robot_description to fill parameters .
     * \param  nh ros node handle of node.
     */
    virtual void initUrdf(std::string urdf)
    {
        urdfXml()=urdf;
        if (!parseURDF(urdfXml(), baseLink(),  endLink()))
        {
            LOG_FATAL <<  "Could not parse the xml for kinematic solver" << _groupname << "\n" << std::flush;
        }
        _numJoints = jointNames().size();
    }

    /**
     * @brief UpdateJointState convert from crcl number based joint spec to ros  name based,
     * Crcl joint indexes should already be adjusted to zero-based, NOT one-based counting
     * @param jointnums vector of joint numbers
     * @param oldjoints vector containing old joint position
     * @param njoints new joints
     * @return sensor_msgs::JointState new joint state with update
     */
    virtual sensor_msgs::JointState updateJointState(std::vector<uint64_t> jointnums,
                                        sensor_msgs::JointState oldjoints,
                                        sensor_msgs::JointState newjoints) {
        sensor_msgs::JointState joints = oldjoints;
        if (joints.velocity.size() != joints.position.size())
            joints.velocity.resize(joints.position.size(), 0.0);
        if (joints.effort.size() != joints.position.size())
            joints.effort.resize(joints.position.size(), 0.0);

        // Check each joint, to see if joint is being actuated, if so, change goal position
        for (size_t i = 0; i < jointnums.size(); i++) {
            size_t n = jointnums[i]; // should already have indexes -1;
            joints.position[n] = newjoints.position[i]; // joint numbers already adjusted from CRCL to rcs model
            if(newjoints.velocity.size() > i)
                joints.velocity[n] = newjoints.velocity[i];
            if(newjoints.effort.size() > i)
                joints.effort[n] = newjoints.effort[i];
        }
        return joints;
    }

    /*!
     * \brief Compares array of joint positions  against joint minimums and maximums.
     * \param  joints array that contains the value of each joints.
     *  \ param outofbounds array that will contain the indexes of the out of bound joints.
     * Negative indexes indicate joint value less that minimum joint value.
     * \param  msg is a message describing which joints are out of range.
     * \return bool whether joints in bounds, 0=within bounds, >1 joint(s) out of bounds
     */
    virtual bool checkJointPositionLimits(std::vector<double> joints, std::vector<int> &outofbounds, std::string &msg) {
        std::stringstream errmsg;
        outofbounds.clear();
        for (size_t i = 0; i < joints.size(); i++) {
            if (joints[i] < jointMin()[i] || joints[i] > jointMax()[i]) {
                if (joints[i] < jointMin()[i]) {
                    outofbounds.push_back(-i);
                    errmsg << jointNames()[i] << "exceed minimum\n";
                }
                if (joints[i] > jointMax()[i]) {

                    outofbounds.push_back(i);
                    errmsg << jointNames()[i] << "exceed maximum\n";
                }

            }
        }
        msg = errmsg.str();
        return outofbounds.size() > 0;
    }

    /*!
     * \brief Compute distance between seed state and solution joint state.
     * ROS routine.
     * First, normalize all solution joint values to (-2pi,+2pi).
     * \param  ik_seed_state contains original joint value
     * \param  solution is candidate joint values.
     * \return distance between joint vectors
     */
    double Harmonize(const std::vector<double> &ik_seed_state, std::vector<double> &solution) const {
        double dist_sqr = 0;
        std::vector<double> ss = ik_seed_state;
        for (size_t i = 0; i < ik_seed_state.size(); ++i) {
#if 1
            while (ss[i] > 2 * M_PI) {
                ss[i] -= 2 * M_PI;
            }
            while (ss[i] < -2 * M_PI) {
                ss[i] += 2 * M_PI;
            }
            while (solution[i] > 2 * M_PI) {
                solution[i] -= 2 * M_PI;
            }
            while (solution[i] < -2 * M_PI) {
                solution[i] += 2 * M_PI;
            }
#endif
            dist_sqr += fabs(ik_seed_state[i] - solution[i]);
        }
        return dist_sqr;
    }

    void GetClosestSolution(const std::vector<std::vector<double>> &solutions,
                            const std::vector<double> &ik_seed_state, std::vector<double> &solution) {
        double mindist = DBL_MAX;
        int minindex = -1;
        std::vector<double> sol;

        for (size_t i = 0; i < solutions.size(); ++i) {
            sol = solutions[i];
            double dist = Harmonize(ik_seed_state, sol);
            if (minindex == -1 || dist < mindist) {
                minindex = i;
                mindist = dist;
            }
        }
        if (minindex >= 0) {
            solution = solutions[minindex];
            Harmonize(ik_seed_state, solution);
        }
    }
    std::string DumpTransformMatrices();
};
typedef std::shared_ptr<IKinematics> IKinematicsSharedPtr;

class MotomanSia20dFastKinematics : public IKinematics {
public:

    MotomanSia20dFastKinematics(std::shared_ptr<RCS::CController> nc) {
        _nc = nc;
        kinName="ikfast";
    }

    virtual tf::Pose FK(std::vector<double> joints);

    virtual int allIK(tf::Pose & pose,
                      std::vector<std::vector<double>> &joints);
//    Eigen::VectorXd ConvertJoints(std::vector<double> v);
//    std::vector<double> ConvertJoints(Eigen::VectorXd ev);
    virtual std::vector<double> nearestJoints(
            std::vector<double> oldjoints,
            std::vector<std::vector<double>> &newjoints);
    virtual int IK(tf::Pose pose,
                   std::vector<double> oldjoints, std::vector<double>&);
    virtual bool isSingular(tf::Pose pose, double threshold);
    //virtual void Init(ros::NodeHandle &nh) ;
    virtual bool verifyLimits(std::vector<double> joints);


};

class FanucLRMate200idFastKinematics : public IKinematics {
public:

    FanucLRMate200idFastKinematics(std::shared_ptr<RCS::CController> nc) {
        _nc = nc;
        kinName="ikfast";
    }

    virtual tf::Pose FK(std::vector<double> joints);
    virtual int allIK(tf::Pose & pose,
                      std::vector<std::vector<double>> &joints);
//    Eigen::VectorXd ConvertJoints(std::vector<double> v);
//    std::vector<double> ConvertJoints(Eigen::VectorXd ev);
    virtual std::vector<double> nearestJoints(
            std::vector<double> oldjoints,
            std::vector<std::vector<double>> &newjoints);
    virtual int IK(tf::Pose pose,
                   std::vector<double> oldjoints, std::vector<double>&);

    virtual bool isSingular(tf::Pose pose, double threshold);
    virtual bool verifyLimits(std::vector<double> joints);
};

class KinUtils
{
public:
    static tf::Pose FK_test(IKinematics *kin, std::vector<double> joints );
    static int IK_test(IKinematics *kin,
                       tf::Pose pose,
                       std::vector<double> oldjoints,
                       std::vector<double>&);
#ifdef GOKIN
    static int SetQuaternionFromRpy(const double roll, const double pitch, const double yaw, tf::Quaternion&);
    static int GetRpyFromQuaternion(const tf::Quaternion&, double &roll, double & pitch, double & yaw);
#endif
};

#ifdef GOKIN
//#include <gzrcs/genserkins.h>
#include <genserkins.h>
class FanucLRMate200idGoSerkin : public IKinematics {
public:

    FanucLRMate200idGoSerkin(std::shared_ptr<RCS::CController> nc) ;

    virtual tf::Pose FK(std::vector<double> joints);
    virtual int IK(tf::Pose pose,
                   std::vector<double> oldjoints, std::vector<double>&);

private:
    double m_per_length_units;
    double rad_per_angle_units;
    int link_number;
    std::vector<go_link> link_params;
    std::shared_ptr<genser_struct>  _pGoKin;
};
#endif

#ifdef MOTOGOKIN
namespace gomotion{
struct GoMotoKin;
};
class MotomanSia20dGoKin  : public IKinematics {
public:

    MotomanSia20dGoKin(std::shared_ptr<RCS::CController> nc);
    virtual tf::Pose FK(std::vector<double> joints);
    virtual int allIK(tf::Pose & pose,
                      std::vector<std::vector<double>> &joints);
    virtual int IK(tf::Pose pose,
                                   std::vector<double> oldjoints, std::vector<double>&);
    virtual bool isSingular(tf::Pose pose, double threshold);
private:
    std::shared_ptr<gomotion::GoMotoKin>  _pGoKin;
};
#endif

#ifdef Trac_IK
#include <trac_ik/trac_ik.hpp>
#include <kdl_parser/kdl_parser.hpp>
class Fanuc200idTrac_IK : public IKinematics
{
   public:
    Fanuc200idTrac_IK(std::shared_ptr<RCS::CController> nc) ;
    virtual tf::Pose FK(std::vector<double> joints);
    virtual int IK(tf::Pose pose,
           std::vector<double> oldjoints,
           std::vector<double> & joints);
    void InitUrdf(std::string urdf);

    std::shared_ptr<TRAC_IK::TRAC_IK>_pTRAC_IK ;
    KDL::JntArray ll;
    KDL::JntArray ul;
};
#endif

#endif
