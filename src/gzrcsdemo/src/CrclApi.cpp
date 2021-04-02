/*
DISCLAIMER:
This software was produced by the National Institute of Standards
and Technology (NIST), an agency of the U.S. government, and by statute is
not subject to copyright in the United States.  Recipients of this software
assume all responsibility associated with its operation, modification,
maintenance, and subsequent redistribution.

See NIST Administration Manual 4.09.07 b and Appendix I.
 */


#include "gzrcsdemo/CrclApi.h"
#include "gzrcsdemo/Globals.h"
#include "gzrcsdemo/Shape.h"
#include "gzrcsdemo/RobotControlException.h"

#include "aprs_headers/Conversions.h"


using namespace RCS;
std::shared_ptr<CCrclApi>  crclApi;

// Simplistic Testing code
int CCrclApi::_crclcommandnum = 1;
RCS::IRate CCrclApi::rates;

////////////////////////////////////////////////////////////////////////////////
CCrclApi::CCrclApi()
{
    _mydwell = .5;
    setVelocity(1.0);
    _crclClient=  std::shared_ptr<crcl::crclClient>(new crcl::crclClient());
    _crclClient->init(Globals.crclIp(),Globals.crclPort(), 0.01);
}
////////////////////////////////////////////////////////////////////////////////
void CCrclApi::start()
{
    _crclClient->start();
}

////////////////////////////////////////////////////////////////////////////////
void CCrclApi::stop()
{
    _crclClient->stop();

}


////////////////////////////////////////////////////////////////////////////////
void CCrclApi::setDwell(double d)
{
    _mydwell=d;
}
////////////////////////////////////////////////////////////////////////////////
void CCrclApi::slow()
{
    rates.CurrentTransSpeed() = rates.CurrentTransSpeed()/2.;
    rates.MaxJointVelocity() = rates.MaxJointVelocity() / 2.0;
    //_nc->slowSpeeds();
}
////////////////////////////////////////////////////////////////////////////////
void CCrclApi::medium()
{
    setVelocity(1.0);
    //_nc->mediumSpeeds();
}
////////////////////////////////////////////////////////////////////////////////
void CCrclApi::fast()
{
    rates.CurrentTransSpeed()= rates.CurrentTransSpeed()*2.;
    rates.MaxJointVelocity() = rates.MaxJointVelocity() * 2.0;
   //_nc->fastSpeeds();
}

////////////////////////////////////////////////////////////////////////////////
void CCrclApi::setGraspingDwell(double d)
{
    _mygraspdwell=d;
}
////////////////////////////////////////////////////////////////////////////////
void CCrclApi::setVelocity(double speed)
{
    rates.CurrentTransSpeed()=speed;
    rates.MaxJointVelocity() = speed;

}

////////////////////////////////////////////////////////////////////////////////
::crcl_rosmsgs::CrclMaxProfileMsg CCrclApi::getSpeeds()
{
    ::crcl_rosmsgs::CrclMaxProfileMsg profile;
    profile.maxvel=rates.CurrentTransSpeed();
    profile.maxacc=10.*rates.CurrentTransSpeed();
    profile.maxjerk= 100.*rates.CurrentTransSpeed();
    return profile;
}


////////////////////////////////////////////////////////////////////////////////
void CCrclApi::setGripper(double ee)
{
    std::cout << "setGripper " << ee << std::endl;
    // FIXME: set gripper to 0..1
    crcl_rosmsgs::CrclCommandMsg cmd;
    CCanonCmd::setRosMsgTimestamp(cmd.header);
    cmd.crclcommand = CanonCmdType::CANON_PAVEL_GRIPPER; // CanonCmdType::CANON_SET_GRIPPER;
    cmd.crclcommandnum = _crclcommandnum++;
    cmd.eepercent = ee;
    _crclClient->addcommand(cmd);
}


////////////////////////////////////////////////////////////////////////////////
void CCrclApi::setContactGripper(double ee)
{
    std::cerr << "CCrclApi::setContactGripper not implemented \n" ;
    return;

    crcl_rosmsgs::CrclCommandMsg cmd;
    CCanonCmd::setRosMsgTimestamp(cmd.header);
    cmd.crclcommand = CanonCmdType::CANON_PAVEL_GRIPPER; // CanonCmdType::CANON_CONTACT_GRIPPER;
    cmd.crclcommandnum = _crclcommandnum++;
    cmd.eepercent = ee;
    _crclClient->addcommand(cmd);
}

////////////////////////////////////////////////////////////////////////////////
void CCrclApi::closeGripper()
{
     setGripper(0.0);
}
////////////////////////////////////////////////////////////////////////////////
void CCrclApi::smartCloseGripper()
{
     setContactGripper(0.0);
}

////////////////////////////////////////////////////////////////////////////////
void CCrclApi::openGripper()
{
   setGripper(1.0);
}

////////////////////////////////////////////////////////////////////////////////
void CCrclApi::doDwell(double dwelltime) {

    std::cout << "doDwell " << dwelltime << std::endl;
    crcl_rosmsgs::CrclCommandMsg cmd;
    CCanonCmd::setRosMsgTimestamp(cmd.header);
    cmd.crclcommand = CanonCmdType::CANON_DWELL;
    cmd.crclcommandnum = _crclcommandnum++;
    cmd.dwell_seconds = dwelltime;
    cmd.eepercent=-1.0; // keep as is
    _crclClient->addcommand(cmd);
}

////////////////////////////////////////////////////////////////////////////////
void CCrclApi::moveTo(tf::Pose pose)
{

    std::cout << "moveTo " << dumpPoseSimple(pose) << std::endl;
    crcl_rosmsgs::CrclCommandMsg cmd;
    CCanonCmd::setRosMsgTimestamp(cmd.header);
    cmd.crclcommandnum = _crclcommandnum++;
    cmd.crclcommand = CanonCmdType::CANON_MOVE_TO;
    cmd.profile.push_back(getSpeeds());
    cmd.finalpose = Convert<tf::Pose, geometry_msgs::Pose> (pose);
    cmd.eepercent=-1.0; // keep as is
    _crclClient->addcommand(cmd);
 }


////////////////////////////////////////////////////////////////////////////////
void  CCrclApi::pick(tf::Pose pose )
{
    std::cout << "pick " << dumpPoseSimple(pose) << std::endl;

    tf::Vector3 offset = pose.getOrigin();

    openGripper(); // make sure griper is open

    // Approach object
    moveTo(Globals.Retract() * tf::Pose(Globals.QBend(), offset));
    doDwell(_mydwell);
    moveTo(tf::Pose(Globals.QBend(), offset));
    doDwell(_mydwell);
    closeGripper();
    doDwell(_mygraspdwell);
    moveTo(Globals.Retract() * tf::Pose(Globals.QBend(), offset));

}

////////////////////////////////////////////////////////////////////////////////
void CCrclApi::moveJoints(std::vector<long unsigned int> jointnum,
                         std::vector<double> positions,
                         double vel)
{

    //std::cout << "moveJoints " << vectorDump<double>(positions) << std::endl;

    crcl_rosmsgs::CrclCommandMsg cmd;
    CCanonCmd::setRosMsgTimestamp(cmd.header);
    cmd.crclcommandnum = _crclcommandnum++;
    cmd.crclcommand = CanonCmdType::CANON_MOVE_JOINT;
    cmd.joints = RCS::zeroJointState(jointnum.size());
    cmd.joints.position = positions;
    cmd.jointnum=jointnum;
    cmd.eepercent=-1.0; // keep as is
    cmd.bCoordinated = true;

    ::crcl_rosmsgs::CrclMaxProfileMsg profile;
    vel = rates.MaxJointVelocity();
    profile.maxvel=vel;
    profile.maxacc=vel*10.;
    profile.maxjerk=vel*100.;
    cmd.profile.push_back(profile)  ;

    _crclClient->addcommand(cmd);

}


////////////////////////////////////////////////////////////////////////////////
int CCrclApi::getStatus(RCS::CCanonWorldModel &status)
{
    crcl_rosmsgs::CrclCommandMsg cmd;
    CCanonCmd::setRosMsgTimestamp(cmd.header);
    cmd.crclcommandnum = _crclcommandnum++;
    cmd.crclcommand = CanonCmdType::CANON_GET_STATUS;
    return _crclClient->syncGetStatus(status);
}

////////////////////////////////////////////////////////////////////////////////
void CCrclApi::place(tf::Pose pose)
{

    std::cout << "place " << dumpPoseSimple(pose) << std::endl;
    tf::Vector3 offset = pose.getOrigin();

    // Retract
    moveTo(Globals.Retract() * tf::Pose(Globals.QBend(), offset));
    doDwell(_mydwell);

    // Place the grasped object
    moveTo(tf::Pose(Globals.QBend(), offset));
    doDwell(_mydwell);

    // open gripper and wait
    openGripper();
    doDwell(_mygraspdwell);

    // Retract from placed object
    moveTo(Globals.Retract() * tf::Pose(Globals.QBend(), offset));
}

////////////////////////////////////////////////////////////////////////////////
int CCrclApi::pickup(std::string objname)
{
    std::cout << "pickup " << objname << std::endl;
    if(grasp(objname) < 0)
        return -1;

    return retract();
}
////////////////////////////////////////////////////////////////////////////////
int CCrclApi::retract(std::string objname)
{
    std::cout << "retract " << objname << std::endl;
    ShapeModel::CShape * shape = ShapeModel::instances.findInstance(objname);
    if(shape==NULL)
    {
        shape = ShapeModel::instances.findSubstrInstance(objname);
        if(shape==NULL)
            return -1;
    }

    tf::Pose pose = Globals.basePose().inverse() * shape->_location;
    tf::Vector3 offset = pose.getOrigin();

    // Retract
    moveTo(Globals.Retract() * tf::Pose(Globals.QBend(), offset));

    return 0;
}

int CCrclApi::retract(double amt)
{
#ifdef FIXME
    sensor_msgs::JointState curjoints, nextjoints;
    curjoints.position = _nc->_status.robotjoints.position;

    tf::Pose r_curpose = _nc->robotKinematics()->FK(curjoints.position);
#if 0
    //FIXME!!!!!!!!
    // instead just move up in z direction
    // This joint motion code was here as ikfast unreliable in straight line up
    r_curpose.getOrigin().setZ(r_curpose.getOrigin().z()+ amt); // move up

    _nc->robotKinematics()->IK(r_curpose,
                           Subset(curjoints.position, _nc->robotKinematics()->NumJoints()),
                           nextjoints.position);
    move_joints(_nc->robotKinematics()->AllJointNumbers(), nextjoints.position);
#endif

    // Retract
    moveTo(_nc->Retract() * r_curpose);
#endif
    return 0;

}
////////////////////////////////////////////////////////////////////////////////
int CCrclApi::approach(std::string objname)
{
    std::cout << "approach " << objname << std::endl;
    ShapeModel::CShape * shape = ShapeModel::instances.findInstance(objname);
    if(shape==NULL)
    {
        shape = ShapeModel::instances.findSubstrInstance(objname);
        if(shape==NULL)
            return -1;
    }

    tf::Pose pose = Globals.basePose().inverse() * shape->_location;
    tf::Vector3 offset = pose.getOrigin();
    this->openGripper();

    // Approach
    moveTo(Globals.Retract() * tf::Pose(Globals.QBend(), offset));

    return 0;
}

////////////////////////////////////////////////////////////////////////////////
int CCrclApi::grasp(std::string objname)
{
    std::cout << "grasp " << objname << std::endl;

    ShapeModel::CShape * shape = ShapeModel::instances.findInstance(objname);
    if(shape==NULL)
    {
         return -1;
    }

    tf::Pose pose = Globals.basePose().inverse() * shape->_location;
    tf::Vector3 offset = pose.getOrigin();

    tf::Pose gripperoffset  = Globals.gripperoffset()[shape->_type];
   // double graspforce = _nc->graspforce()[shape->_type];

    // Grasp
    moveTo(tf::Pose(Globals.QBend(), offset) * gripperoffset);
    doDwell(_mydwell);
    closeGripper();
    doDwell(_mygraspdwell);

    return 0;

}
////////////////////////////////////////////////////////////////////////////////
int CCrclApi::moveTo(std::string objname) {
    std::cout << "moveTo " << objname << std::endl;
    ShapeModel::CShape * shape = ShapeModel::instances.findInstance(objname);
    if(shape==NULL)
    {
        shape = ShapeModel::instances.findSubstrInstance(objname);
        if(shape==NULL)
            return -1;
    }
    tf::Pose pose = Globals.basePose().inverse() * shape->_location;
    tf::Vector3 offset = pose.getOrigin();

    std::cout << "Moveto gear type is " << shape->_type << "\n";

    tf::Pose gripperoffset  = Globals.gripperoffset()[shape->_type];
    std::cout << "Moveto gripperoffset     " << dumpPoseSimple(gripperoffset) << "\n";

    tf::Pose endPose =  tf::Pose(Globals.QBend(), offset) ;
    std::cout << "Moveto endpose           " << dumpPoseSimple(endPose) << "\n";

    endPose=endPose* gripperoffset;
    std::cout << "Moveto endpose & Offset  " << dumpPoseSimple(endPose) << "\n";

    moveTo(endPose);
    return 0;
}

////////////////////////////////////////////////////////////////////////////////
bool CCrclApi::isDone()
{
    // queue empty, and status not working.
    return _crclClient->isDone();
}
////////////////////////////////////////////////////////////////////////////////
bool CCrclApi::isBusy()
{
    // queue empty, and status not working.
    return _crclClient->isBusy();
}
////////////////////////////////////////////////////////////////////////////////
bool CCrclApi::isConnected()
{
    // is connected to crcl socket.
    return _crclClient->isConnected();
}
