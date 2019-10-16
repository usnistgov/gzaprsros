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
//#pragma message "Compiling " __FILE__ 
#include "trajectoryMaker.h"
#include <numeric>    // std::accumulate
#include <functional> // std::minus
#include "Globals.h"
#include <algorithm>
#include "NIST/Boost.h"
#include "Conversions.h"
#include "Debug.h"
using namespace RCS;
using namespace Conversion;

// FIxme move to posemath.h
static bool IsEqual(tf::Pose t1, tf::Pose t2, double epsilon = 0.001) {
    double t = (t1.getOrigin().distance(t2.getOrigin()));
    double r = (t1.getRotation().dot(t2.getRotation()));
    return (t + r)<epsilon;
}

bool WorldTrajectoryMaker::evalWorldTrajectory(const double &max_vel,
        const tf::Pose & curpose,
        const tf::Pose & lastpose,
        const tf::Pose & goalpose,
        tf::Pose & nextpose) {

    //double tvel = curvel.norm();
    double tvel = curpose.getOrigin().distance(lastpose.getOrigin());
    double rvel = curpose.getRotation().dot(lastpose.getRotation());

    double dIncrement = tvel / (goalpose.getOrigin().distance(curpose.getOrigin()));
    tf::Vector3 trans = curpose.getOrigin().lerp(goalpose.getOrigin(), dIncrement);
    tf::Quaternion rot = curpose.getRotation().slerp(goalpose.getRotation(), dIncrement);
    if (goalpose.getOrigin().distance(trans) < 0.001)
        trans = goalpose.getOrigin();
    if (goalpose.getRotation().dot(rot) < 0.001)
        rot = goalpose.getRotation();

    nextpose.setOrigin(trans);
    nextpose.setRotation(rot);

    return IsEqual(nextpose, goalpose);

}

bool JointTrajectoryMaker::evalJointPositionTrajectory(const std::vector<double> &max_vel,
        const JointState & curjoints,
        const JointState & lastjoints,
        const JointState & goaljoints,
        JointState & nextjoints) {
    nextjoints.position = curjoints.position;
    size_t distindex, velmaxindex;

    Eigen::VectorXd here = Convert<std::vector<double>, Eigen::VectorXd>(curjoints.position);
    Eigen::VectorXd there = Convert<std::vector<double>, Eigen::VectorXd>(goaljoints.position);
    Eigen::VectorXd last = Convert<std::vector<double>, Eigen::VectorXd>(lastjoints.position);
    Eigen::VectorXd diff = there - here;
    Eigen::VectorXd absdiff = diff.array().abs();
    Eigen::VectorXd curvel = (here - last).array().abs();
    Eigen::VectorXd maxjointvel = Convert<std::vector<double>, Eigen::VectorXd>(max_vel) * _cycletime / 10.0;
    Eigen::VectorXd maxvel = maxjointvel;
#if 0
    for (int i = 0; i < maxvel.size(); i++) {
        double sign = .1;
        if (curvel(i) > absdiff(i))
            sign = -.1;
        maxvel(i) = std::min(maxjointvel(i), curvel(i) + sign * maxjointvel(i));
        // what if destination stopping exceeded by accl/decl
    }
#endif
    Eigen::VectorXd numcycles = absdiff;
    // assume moves are bigger than ramp up and down.
    for (int i = 0; i < numcycles.size(); i++)
        numcycles(i) = std::ceil(absdiff(i) / maxvel(i));

    size_t maxcycles = numcycles.maxCoeff();

    for (size_t i = 0; i < nextjoints.position.size(); i++) {
        nextjoints.position[i] = nextjoints.position[i] + diff(i)* 1.0 / maxcycles;
    }
    double dmax = 0.0;
    for (size_t i = 0; i < nextjoints.position.size(); i++) {

        if (fabs(nextjoints.position[i] - goaljoints.position[i]) < 0.001)
            nextjoints.position[i] = goaljoints.position[i];
        dmax = std::max(dmax, fabs(nextjoints.position[i] - goaljoints.position[i]));
    }

#if defined(DEBUG)
    std::cout << "Here     =" << DumpEVector(here) << "\n";
    std::cout << "There    =" << DumpEVector(there) << "\n";
    std::cout << "Maxvel   =" << DumpEVector(maxvel) << "\n";
    std::cout << "Diff     =" << DumpEVector(diff) << "\n";
    std::cout << "Absdiff  =" << DumpEVector(absdiff) << "\n";
    std::cout << "Numcycles=" << DumpEVector(numcycles) << "\n";
    std::cout << "maxcycles=" << maxcycles << "\n";
    std::cout << "numcycles.maxCoeff  =" << distindex << "\n";

    std::cout << "Next     =" << VectorDump<double>(nextjoints.position) << "\n" << std::flush;
#endif
    return dmax < 0.001;
}

bool JointTrajectoryMaker::stopJointPositionTrajectory(CanonStopMotionType stopType, const std::vector<double> &max_vel,
        const JointState & curjoints,
        const JointState & lastjoints,
        const JointState & goaljoints,
        JointState & nextjoints) {
    nextjoints.position = curjoints.position;
    size_t distindex, velmaxindex;

    Eigen::VectorXd here = Convert<std::vector<double>, Eigen::VectorXd>(curjoints.position);
    Eigen::VectorXd there = Convert<std::vector<double>, Eigen::VectorXd>(goaljoints.position);
    Eigen::VectorXd last = Convert<std::vector<double>, Eigen::VectorXd>(lastjoints.position);
    Eigen::VectorXd diff = there - here;
    Eigen::VectorXd absdiff = diff.array().abs();
    Eigen::VectorXd curvel = (here - last).array().abs();
    Eigen::VectorXd maxjointvel = Convert<std::vector<double>, Eigen::VectorXd>(max_vel) * _cycletime / 10.0;
    Eigen::VectorXd maxvel = maxjointvel;
    double stopMultiplier;
    stopMultiplier = .5  ;
   //stopMultiplier = (stopType == CanonStopMotionType::IMMEDIATE) ? 10.0 * maxjointvel(i) : maxjointvel(i);
#if 0
    for (int i = 0; i < maxvel.size(); i++) {
        double sign = .1;
        if (curvel(i) > absdiff(i))
            sign = -.1;
        maxvel(i) = std::min(maxjointvel(i), curvel(i) - stopMultiplier * maxjointvel(i));
        // what if destination stopping exceeded by accl/decl
    }
#endif
    Eigen::VectorXd numcycles = absdiff;
    // assume moves are bigger than ramp up and down.
    for (int i = 0; i < numcycles.size(); i++)
        numcycles(i) = std::ceil(absdiff(i) / maxvel(i));

    size_t maxcycles = numcycles.maxCoeff();

    for (size_t i = 0; i < nextjoints.position.size(); i++) {
        nextjoints.position[i] = nextjoints.position[i] + diff(i)* 1.0 / maxcycles;
    }
    double dmax = 0.0;
    for (size_t i = 0; i < nextjoints.position.size(); i++) {

        if (fabs(nextjoints.position[i] - goaljoints.position[i]) < 0.001)
            nextjoints.position[i] = goaljoints.position[i];
        dmax = std::max(dmax, fabs(nextjoints.position[i] - goaljoints.position[i]));
    }

#if defined(DEBUG)
    std::cout << "Here     =" << DumpEVector(here) << "\n";
    std::cout << "There    =" << DumpEVector(there) << "\n";
    std::cout << "Maxvel   =" << DumpEVector(maxvel) << "\n";
    std::cout << "Diff     =" << DumpEVector(diff) << "\n";
    std::cout << "Absdiff  =" << DumpEVector(absdiff) << "\n";
    std::cout << "Numcycles=" << DumpEVector(numcycles) << "\n";
    std::cout << "maxcycles=" << maxcycles << "\n";
    std::cout << "numcycles.maxCoeff  =" << distindex << "\n";

    std::cout << "Next     =" << VectorDump<double>(nextjoints.position) << "\n" << std::flush;
#endif
    return dmax < 0.001;
}

std::vector<JointState> TrajectoryMaker::GetJtsPlan() {
    return plannedjts;
}

bool TrajectoryMaker::Plan(JointState curjoints, JointState goaljoints) {
    return makeJointPositionTrajectory(currates, curjoints, goaljoints);
}

bool TrajectoryMaker::makeJointPositionTrajectory(IRate rates, JointState & curjoints, JointState & goaljoints) {
    assert(curjoints.position.size() == goaljoints.position.size());
    std::vector<std::vector<double> > displacements;
    for (size_t i = 0; i < curjoints.position.size(); i++) {
//        std::vector<double> d = makeJointTrajectory(curjoints.position[i], goaljoints.position[i]);
//        displacements.push_back(d);
    }
    updateJointCommands(curjoints.position, displacements);
    return true;

}

#if 1

bool TrajectoryMaker::makeJointPositionTrajectory(IRate rates, std::vector<double> & curjoints, std::vector<double> & goaljoints) {
    assert(curjoints.size() == goaljoints.size());
    std::vector<std::vector<double> > displacements;
    for (size_t i = 0; i < curjoints.size(); i++) {
//        std::vector<double> d = makeJointTrajectory(curjoints[i], goaljoints[i]);
//        displacements.push_back(d);
    }
    updateJointCommands(curjoints, displacements);
    return true;
}
#endif

void TrajectoryMaker::updateJointCommands(std::vector<double> & curjoints, std::vector<std::vector<double> > & displacements) {
    // imax ends up being the number of new motion commands
    size_t imax = 0;
    plannedjts.clear();
    for (size_t i = 0; i < displacements.size(); i++) {
        imax = std::max(imax, displacements[i].size());
    }

    std::vector<double> lastjoints = curjoints;
    JointState joint;
    joint.position = curjoints;
    plannedjts.push_back(joint);

    for (size_t i = 0; i < imax; i++) {
        JointState joint;
        // initialize last joint position
        joint.position.insert(joint.position.begin(), lastjoints.begin(), lastjoints.end());

        // update new joint position
        for (size_t j = 0; j < displacements.size(); j++) {
            if ((displacements[j].size() > 0) && (displacements[j].size() <= imax)) {
                joint.position[j] += displacements[j][i];
            }
        }
        lastjoints = joint.position; // save last joint positions
        plannedjts.push_back(joint);
        //std::cout << "Joint Displacements" << RCS::VectorDump<double> (joint.position);
    }

}

std::vector<double> TrajectoryMaker::makeJointValues(double current, std::vector<double> displacements) {
    for (size_t i = 0; i < displacements.size(); i++) {
        displacements[i] += current;
    }
    return displacements;
}