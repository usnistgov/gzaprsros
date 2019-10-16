/*
DISCLAIMER:
This software was produced by the National Institute of Standards
and Technology (NIST), an agency of the U.S. government, and by statute is
not subject to copyright in the United States.  Recipients of this software
assume all responsibility associated with its operation, modification,
maintenance, and subsequent redistribution.

See NIST Administration Manual 4.09.07 b and Appendix I.
 */

#include "nist_robotsnc/kinematicring.h"
#include <nist/Conversions.h>  // for tf::Identity()

namespace RCS
{



// ----------------------------------------------------
// KinematicRing

////////////////////////////////////////////////////////////////////////////////
KinematicRing::KinematicRing() :
    _prerobotxform(1, tf::Identity()),
    _postrobotxform(2, tf::Identity()),
    _gripperPose(_postrobotxform[0]),
    _basePose(_prerobotxform[0]),
    _toolPose(_postrobotxform[1])
{
    _postrobotxform.clear();
    _postrobotxform.resize(2, tf::Identity());
}

////////////////////////////////////////////////////////////////////////////////
void KinematicRing::setGripperOffset(tf::Pose offset)
{
    assert(_postrobotxform.size()>0);
    _gripperPose = offset;
}


////////////////////////////////////////////////////////////////////////////////
void KinematicRing::setToolOffset(tf::Pose offset) {
    assert(_postrobotxform.size()>1);
    _toolPose = offset;
}

////////////////////////////////////////////////////////////////////////////////
void KinematicRing::setBaseOffset(tf::Pose offset) {
    assert(_prerobotxform.size()>0);
    _basePose = offset;
}

////////////////////////////////////////////////////////////////////////////////
tf::Pose KinematicRing::worldCoord(tf::Pose robotpose) {
#if 0
    // Fixme: this should be variable depending
    // on current kinematic ring
    for (size_t i = 0; i < prerobotxform.size(); i++)
        robotpose = prerobotxform[i] * robotpose;
    //for (size_t i = 0; i < postrobotxform.size(); i++)
    //    robotpose = robotpose * postrobotxform[i];
    robotpose = robotpose * _gripperPose;
    return robotpose;
#else
    return basePose() * robotpose * gripperPose();
#endif
}

////////////////////////////////////////////////////////////////////////////////
tf::Pose KinematicRing::robotAndGripperCoord(tf::Pose worldpose)
{
    worldpose = basePose().inverse() *  worldpose ;

    return worldpose;
}

////////////////////////////////////////////////////////////////////////////////
tf::Pose KinematicRing::robotRemoveGripperCoord(tf::Pose pose)
{
    pose =  pose * gripperPose().inverse() ;

    return pose;
}

////////////////////////////////////////////////////////////////////////////////
tf::Pose KinematicRing::robotAddGripperCoord(tf::Pose pose)
{
    pose =  pose * gripperPose() ;

    return pose;
}

////////////////////////////////////////////////////////////////////////////////
tf::Pose KinematicRing::robotOnlyCoord(tf::Pose worldpose) {
#if 0
    // Fixme: this should be variable depending
    // on current kinematic ring
    //for(size_t i=0; i< prerobotxform.size(); i++)
    for (size_t i = prerobotxform.size(); i-- > 0;)
        worldpose = prerobotxform[i].inverse() * worldpose;
    //for(size_t i=0; i< postrobotxform.size(); i++)
    for (size_t i = postrobotxform.size(); i-- > 0; )
        worldpose =  worldpose * postrobotxform[i].inverse();
#else
    worldpose = basePose().inverse() *  worldpose * gripperPose().inverse();
#endif

    // return basePose().inverse() *  worldpose * gripperPose().inverse();
    return worldpose;
}

////////////////////////////////////////////////////////////////////////////////
tf::Pose KinematicRing::addBaseTransform(tf::Pose pose) {
    return _basePose * pose;
}

////////////////////////////////////////////////////////////////////////////////
tf::Pose  KinematicRing::gripperPose() {
    return _gripperPose;
}

////////////////////////////////////////////////////////////////////////////////
tf::Pose  KinematicRing::basePose() {
    return _basePose;
}

////////////////////////////////////////////////////////////////////////////////
tf::Pose  KinematicRing::toolPose() {
    return _toolPose;
}
}
