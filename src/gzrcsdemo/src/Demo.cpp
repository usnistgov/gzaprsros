

/*
DISCLAIMER:
This software was produced by the National Institute of Standards
and Technology (NIST), an agency of the U.S. government, and by statute is
not subject to copyright in the United States.  Recipients of this software
assume all responsibility associated with its operation, modification,
maintenance, and subsequent redistribution.

See NIST Administration Manual 4.09.07 b and Appendix I.
 */
//#pragma message "Compiling " __FILE__ 
#include <memory>


#include "gzrcsdemo/Demo.h"
#include "gzrcsdemo/Globals.h"
#include "gzrcsdemo/Shape.h"
#include "gzrcsdemo/RobotControlException.h"
#include "aprs_headers/Conversions.h"
#include "gzrcsdemo/CrclApi.h"

using namespace RCS;

#include "boost/iostreams/stream.hpp"
#include "boost/iostreams/device/null.hpp"
#include <boost/iostreams/stream.hpp>

boost::iostreams::stream< boost::iostreams::null_sink > nullOstream( ( boost::iostreams::null_sink() ) );


static std::ostream* os;
#define ONCE(X)  \
{ static int Y##__LINE__=-1;\
    if(++Y##__LINE__==0) os=&X; \
    else os=&nullOstream;}\
    *os

////////////////////////////////////////////////////////////////////////////////
CGearDemo::CGearDemo()
{
}
////////////////////////////////////////////////////////////////////////////////
int CGearDemo::init(std::string robotName)
{
    ShapeModel::CShapes::initDefinitions();
    // Crcl api command interface to robot
    crclApi= std::shared_ptr<CCrclApi>(new CCrclApi());


    // concifg
    Globals.bGripperSpeed=RCS::robotconfig.getSymbolValue<int>("demo.gripper_speed","0");
    Globals.bClosestFree=RCS::robotconfig.getSymbolValue<int>("demo.closest_free","0");
    Globals.bClosestOpenSlot=RCS::robotconfig.getSymbolValue<int>("demo.closet_open_slot","0");
    double dDwellTime = RCS::robotconfig.getSymbolValue<double>("demo." + robotName + ".dwell.time", "1000.0");
    double dGraspingDwellTime = RCS::robotconfig.getSymbolValue<double>("demo." + robotName + ".dwell.grasping", "1000.0");
    if(crclApi.get() == NULL)
    {
        std::cout << "Demo assigning parameter to NULL crclApi\n";
        return -1;
    }
    crclApi->setDwell(dDwellTime);
    crclApi->setGraspingDwell(dGraspingDwellTime);
    crclApi->medium();
    return 0;
}

////////////////////////////////////////////////////////////////////////////////
void CGearDemo::start()
{

    if(Globals.bGearLocations)
    {
        // these are the gear/shape static definitions
        // in theory only required for testing
        ShapeModel::CShapes::initDefinitions();

        std::string  errmsg=gzInstances.init();
        if(!errmsg.empty())
            std::cout << "Error configuring Gazebo Model plugin" << errmsg << "\n";

        gzInstances.start();
    }

    crclApi->start();

}

////////////////////////////////////////////////////////////////////////////////
void CGearDemo::stop()
{
    // These are NOT threads, but ROS or Gazebo subscriptions
    if(Globals.bGearLocations)
        gzInstances.stop();

    crclApi->stop();
}



////////////////////////////////////////////////////////////////////////////////
int CGearDemo::issueRobotCommands(int & state)
{

    // Finish queuing commands before handling them....
    std::unique_lock<std::mutex> lock(cncmutex);


    if( ShapeModel::instances.size()==0)
    {
        ONCE(  std::cout) << "Error: No gear instances read - restart Gazebo!\n";
        return state;
    }

    switch(state)
    {
    case 0:
    {
        std::cout << "State 0 \n";
        // Find a free gear
        if ((_instance = ShapeModel::instances.findFreeGear(Globals.part_list())) ==NULL) //, r.cnc()->currentPose())) == NULL)
        {
            ONCE( std::cout) << "Error: No Free Gear found in tray to move\n";
            return -1;
        }
        return state++;
    }
    case 1:
    {

        std::cout << "State 1 \n";
        gearname = _instance->_name;
        // Ok we will move this gear - mark as no longer free standing
        _instance->_properties["state"] = "stored";

        affpose =  _instance->_location ;
        affpose = Globals.basePose().inverse() * affpose ;

        // The object gripper offset is where on the object it is to be gripped
        // e.g., offset.gripper.largegear = 0.0,0.0, -0.030, 0.0, 0.0.,0.0
        gripperoffset = Globals.gripperoffset()[_instance->_type];
        // gripperoffset = tf::Pose(tf::QIdentity(),tf::Vector3(0.0,0.0, - _instance->_height * .8));

        bend=Globals.QBend();

        // The gripperoffset is the robot gripper offset back to the 0T6 equivalent
        pickpose =  tf::Pose(bend, affpose.getOrigin()) * gripperoffset ;

        offset = pickpose.getOrigin();

        // Retract
        crclApi->moveTo(Globals.Retract() * tf::Pose(bend, offset));
        crclApi->doDwell(crclApi->_mydwell);
        return state++;
    }
    case 2:
    {
        std::cout << "State 2 \n";
        crclApi->moveTo(tf::Pose(bend, offset) );
        crclApi->doDwell(crclApi->_mydwell);
        return state++;
    }
    case 3:
    {
        std::cout << "State 3 \n";
        crclApi->closeGripper();
        crclApi->doDwell(crclApi->_mygraspdwell);
        return state++;
    }
    case 4:
    {
        std::cout << "State 4 \n";
        crclApi->moveTo(Globals.Retract() * tf::Pose(bend, offset));
        crclApi->doDwell(crclApi->_mydwell);
        return state++;
    }

    case 5:
    {
        std::cout << "State 5 \n";
        // Find a kit slot and then its offset from the centroid of the kit
        // Find a gear slot in a kit
        ShapeModel::CShape * kit=NULL;
        ShapeModel::CShape * slot=NULL;
        if(!ShapeModel::instances.findFreeGearKitSlot(_instance,
                                                      slotpose,
                                                      Globals.part_list()))
        {
            static int bThrottle=1;
            if(bThrottle>0)
                std::cout << "Error: No Free Kit Slot\n";
            bThrottle--;
            return -1;
        }

        slotpose = Globals.basePose().inverse() * slotpose;
        slotoffset = Globals.slotoffset()["vessel"];
        placepose = tf::Pose(bend, slotpose.getOrigin())* slotoffset; // fixme: what if gear rotated
        offset = placepose.getOrigin();

        // Approach
        crclApi->moveTo(Globals.Retract()* tf::Pose(bend, offset));
        crclApi->doDwell(crclApi->_mydwell);
        return state++;
    }
    case 6:
    {
        std::cout << "State 6 \n";
        // Place the grasped object
        crclApi->moveTo(tf::Pose(bend, offset));
        crclApi->doDwell(crclApi->_mydwell);
        return state++;
    }
    case 7:
    {
        std::cout << "State 7 \n";
        // open gripper and wait
        crclApi->openGripper();
        crclApi->doDwell(crclApi->_mygraspdwell);
        return state++;
    }
    case 8:
    {
        std::cout << "State 8 \n";
        // Retract from placed object
        //r.Retract(0.04);
        crclApi->moveTo(Globals.Retract() * tf::Pose(bend, offset));
        crclApi->doDwell(crclApi->_mydwell);
        return state++;
    }
    }
    // all done reset
    state=0;
    return 0;

}

int CGearDemo::isDone(int & state)
{
    // FIXME: must read CRCL to determine if done/busy.
    // Or queued commands?
    return (state==9 && !crclApi->isBusy());
}

int CGearDemo::isBusy(int & state)
{
    // FIXME: must read CRCL to determine if done/busy.
    // Or queued commands?
    return (crclApi->isBusy());
}







