

//
// Gripper.cpp
//

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

#include "gzrcs/Gripper.h"
#include "gzrcs/Controller.h"

////////////////////////////////////////////////////////////////////////////////
GripperInterface::GripperInterface()
{
}

////////////////////////////////////////////////////////////////////////////////
std::string GripperInterface::init(std::string robotName)
{
     gripper_name = RCS::robotconfig.getSymbolValue<std::string>(robotName + ".robot.gripper", ",");

     joint_state.name = RCS::robotconfig.getTokens<std::string>( gripper_name+  + ".gripperJoints", ",");
     // Prepend the current robot prefix to these names. CONVENTION/STANDARD/HACK
     for(size_t k=0; k<joint_state.name.size(); k++)
         joint_state.name[k]=robotName + joint_state.name[k];
    this->joints_names=joint_state.name;

     joints_multipler = RCS::robotconfig.getTokens<double>( gripper_name  + ".gripperMultiplier", ",");

    // ncs[i]->gripperJointsMin() = RCS::robotconfig.getTokens<double>( ncs[i]->gripperName()  + ".gripperMin", ",");
    // ncs[i]->gripperJointsMax() = RCS::robotconfig.getTokens<double>( ncs[i]->gripperName()  + ".gripperMax", ",");

     this->joints_open = RCS::robotconfig.getTokens<double>( gripper_name  + ".gripperOpen", ",");
     this->joints_closed = RCS::robotconfig.getTokens<double>(gripper_name  + ".gripperClose", ",");


     if(joint_state.name.size() ==0)
         return "Config error: Gripper " +  gripper_name + " joint names missing names\n";   // ||joints_multipler.size() ==0)
     if(joints_multipler.size() ==0)
         return "Config error: Gripper " +  gripper_name + " joints_multipler missing names\n";   // ||joints_multipler.size() ==0)

    return _gripperwriter.init(robotName);

}

////////////////////////////////////////////////////////////////////////////////
void GripperInterface::start()
{
    _gripperwriter.start();
}

////////////////////////////////////////////////////////////////////////////////
void GripperInterface::stop()
{
    _gripperwriter.stop();
}

////////////////////////////////////////////////////////////////////////////////
void GripperInterface::updateGripper(double time, double eepercent)
{

    _gripperwriter.updateGripper(time, eepercent);
}

////////////////////////////////////////////////////////////////////////////////
sensor_msgs::JointState GripperInterface::close()
{
    joint_state.position=this->joints_closed;
    return joint_state;
}

////////////////////////////////////////////////////////////////////////////////
sensor_msgs::JointState GripperInterface::open()
{
    joint_state.position=this->joints_open;
    return joint_state;
}

////////////////////////////////////////////////////////////////////////////////
bool GripperInterface::is_open()
{
    return ! _gripperwriter.isGrasping();

//    return  std::equal(joint_state.position.begin(), joint_state.position.end(),this->joints_open.begin() );
}

////////////////////////////////////////////////////////////////////////////////
bool GripperInterface::is_closed()
{
    return  _gripperwriter.isGrasping();
 //   return  std::equal(joint_state.position.begin(), joint_state.position.end(),this->joints_closed.begin() );

}

////////////////////////////////////////////////////////////////////////////////
int GripperInterface::isGrasping()
{
    return  _gripperwriter.isGrasping();
}


////////////////////////////////////////////////////////////////////////////////
sensor_msgs::JointState GripperInterface::increment(sensor_msgs::JointState gripperjoints,
                                  std::vector<std::string> joints,
                                  double amount)
{
    joint_state=gripperjoints;

    // assume joint mimic each other
    for(size_t i=0; i< joints.size(); i++)
    {
        std::vector<std::string>::iterator it;
        if((it=std::find(joints_names.begin(), joints_names.end(), joints[i])) == joints_names.end())
            continue;
        size_t index = std::distance(joints_names.begin(), it);
        joint_state.position[index]=joints_multipler[index] * amount + joint_state.position[index];
    }
    return joint_state;
}

////////////////////////////////////////////////////////////////////////////////
sensor_msgs::JointState GripperInterface::set_position(double percent)
{
    // assume joint mimic each other
    for(size_t i=0; i< joint_state.name.size(); i++)
    {
        joint_state.position[i]= (joints_open[i] - joints_closed[i]) * percent + joint_state.position[i];
     }
    return joint_state;
}

////////////////////////////////////////////////////////////////////////////////
sensor_msgs::JointState GripperInterface::set_abs_position(double position)
{
    for(size_t i=0; i< joint_state.name.size(); i++)
        joint_state.position[i]=joints_multipler[i]*position;

    return joint_state;
}


void GripperInterface::update(double eepercent)
{
    if(Globals.bGzGripperPlugin )
    {
        //if (eepercent>=0.0)
            //_writer.updateGripper(ms.count(), eepercent);
    }
    else
    {
       // _writer.updateGripper(ms.count(), gripperJoints);
    }
}
