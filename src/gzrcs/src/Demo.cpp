

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


#include "gzrcs/Demo.h"
#include "gzrcs/Controller.h"
#include "gzrcs/Globals.h"
#include "gzrcs/Shape.h"
#include "gzrcs/RobotControlException.h"
#include "gzrcs/CrclApi.h"
//#include "aprs_headers/Conversions.h"

using namespace RCS;

// polled wait on function with timeout

#include <chrono>
#include <functional>
typedef std::function<bool ()> TPollFcn;

/**
 * @brief The timer struct
 *  while( gripper.state() != 0 && t.seconds_elapsed() > 1 )
        { ::sleep(50) ; }

    if(gripper.state() == 0)
        return error;
 */

//http://www.cplusplus.com/forum/beginner/91449/
struct timer
{
    typedef std::chrono::steady_clock clock ;
    typedef std::chrono::seconds seconds ;

    timer(TPollFcn fcn) : savefcn(fcn)
    {
    }

    int poll(int equality)
    {
        while( savefcn() != equality && seconds_elapsed() > 1 )
        {
            ::sleep(50) ;
        }

        if(savefcn() == equality)
            return 0;

        return 1;

    }

    void reset()
    {
        start = clock::now() ;
    }

    unsigned long long seconds_elapsed() const
    { return std::chrono::duration_cast<seconds>( clock::now() - start ).count() ; }

    private: clock::time_point start = clock::now() ;
    TPollFcn savefcn;
};



// Log once

int CGearDemo::init(std::string robotName)
{

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
}

void CGearDemo::stop()
{
    // These are NOT threads, but ROS or Gazebo subscriptions
    if(Globals.bGearLocations)
        gzInstances.stop();

}

////////////////////////////////////////////////////////////////////////////////
CGearDemo::CGearDemo(std::shared_ptr<CCrclApi>  crclApi) : r(crclApi)
{
    ShapeModel::CShapes::initDefinitions();
}

////////////////////////////////////////////////////////////////////////////////
int CGearDemo::issueRobotCommands(int & state)
{

    // Finish queuing commands before handling them....
    std::unique_lock<std::mutex> lock(cncmutex);

    // Must declare all variables beforehand
    RCS::CCanonCmd cmd;
    tf::Pose pickpose;
    std::string gearname;
    tf::Pose affpose;
    tf::Pose gripperoffset;
    tf::Quaternion bend;
    tf::Vector3 offset;
    tf::Pose slotpose;
    tf::Pose slotoffset;
    tf::Pose placepose;

    if( ShapeModel::instances.size()==0)
    {
        STATUS_LOG.ONCE() << "Error: No gear instances can be read from Gazebo model- restart Gazebo!\n";
        return -1;
    }

    switch(state)
    {
    case 0:
    {
        // Find a free gear
        if ((_instance = ShapeModel::instances.findFreeGear(r->cnc()->part_list(), r->cnc()->currentPose())) == NULL)
        {
            STATUS_LOG.ONCE() << "Error: No Free Gear in tray to move\n";
            return -1;
        }
        return state++;
    }
    case 1:
    {

        gearname = _instance->_name;
        // Ok we will move this gear - mark as no longer free standing
        _instance->_properties["state"] = "stored";

        affpose =  _instance->_location ;
        affpose = r->cnc()->basePose().inverse() * affpose ;

        // The object gripper offset is where on the object it is to be gripped
        // e.g., offset.gripper->largegear = 0.0,0.0, -0.030, 0.0, 0.0.,0.0
        gripperoffset = r->cnc()->gripperoffset()[_instance->_type];
       // gripperoffset = tf::Pose(tf::QIdentity(),tf::Vector3(0.0,0.0, - _instance->_height * .8));

        bend=r->cnc()->QBend();

        // The gripperoffset is the robot gripper offset back to the 0T6 equivalent
        pickpose =  tf::Pose(bend, affpose.getOrigin()) * gripperoffset ;

        offset = pickpose.getOrigin();

        // Retract
        r->moveTo(r->cnc()->Retract() * tf::Pose(bend, offset));
        r->doDwell(r->_mydwell);
        return state++;
    }
    case 2:
    {
        r->moveTo(tf::Pose(bend, offset) );
        r->doDwell(r->_mydwell);
        return state++;
    }
    case 3:
    {
        r->closeGripper();
        r->doDwell(r->_mygraspdwell);
        return state++;
    }
    case 4:
    {
        r->moveTo(r->cnc()->Retract() * tf::Pose(bend, offset));
        r->doDwell(r->_mydwell);
        return state++;
    }

    case 5:
    {
        // Find a kit slot and then its offset from the centroid of the kit
        // Find a gear slot in a kit
        ShapeModel::CShape * kit=NULL;
        ShapeModel::CShape * slot=NULL;
        if(!ShapeModel::instances.findFreeGearKitSlot(_instance,
                                                      slotpose,
                                                      r->cnc()->part_list()))
        {
            static int bThrottle=1;
            if(bThrottle>0)
                std::cout << "Error: No Free Kit Slot\n";
            bThrottle--;
            return -1;
        }

        slotpose = r->cnc()->basePose().inverse() * slotpose;
        slotoffset = r->cnc()->slotoffset()["vessel"];
        placepose = tf::Pose(bend, slotpose.getOrigin())* slotoffset; // fixme: what if gear rotated
        offset = placepose.getOrigin();

        // Approach
        r->moveTo(r->cnc()->Retract()* tf::Pose(bend, offset));
        r->doDwell(r->_mydwell);
        return state++;
    }
    case 6:
    {
        // Place the grasped object
        r->moveTo(tf::Pose(bend, offset));
        r->doDwell(r->_mydwell);
        return state++;
    }
    case 7:
    {
        // open gripper and wait
        r->openGripper();
        r->doDwell(r->_mygraspdwell);
        return state++;
    }
    case 8:
    {
        // Retract from placed object
        //r->Retract(0.04);
        r->moveTo(r->cnc()->Retract() * tf::Pose(bend, offset));
        r->doDwell(r->_mydwell);
        return state++;
    }
    }
    state=0;
    return state;

}

int CGearDemo::isDone(int & state)
{
    return (state==9 && !r->cnc()->isBusy());
}








