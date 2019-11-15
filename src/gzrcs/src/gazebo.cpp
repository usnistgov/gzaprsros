
#include "aprs_headers/Debug.h"

#include "gzrcs/gazebo.h"
#include "gzrcs/Globals.h"
#include "gzrcs/Shape.h"
#include "aprs_headers/Config.h"

// static definitions
gazebo::transport::NodePtr CGazebo::_node;
bool CGazebo::_bInited=false;
std::string CGazebo::_inifile;
std::string CGazebo::_robotname;

// extern global definitions
CGazebo gz;
static Nist::Config gzconfig;


////////////////////////////////////////////////////////////////////////////////
CGazebo::CGazebo()
{
}

////////////////////////////////////////////////////////////////////////////////
std::string CGazebo::init(std::string robotname, std::string inifile)
{

    _inifile=inifile;
    _robotname=robotname;

    if(!gzconfig.loadFile(CGazebo::_inifile))
        return "ini file  " + inifile + "did not open";


    int argc=1;
    char* argv[1];
    argv[0]=(char *) getexepath().c_str();

    // Wait for gazebo server to be running -  gzserver program
    std::cout << "Connecting to gazebo server";
    while( Globals.procFind("gzserver") < 0)
    {
        Globals.sleep(1000);
        std::cout << "." << std::flush;
    }
    std::cout << "\n" << std::flush;
    Globals.sleep(1000);


    // Load gazebo
#if GAZEBO_MAJOR_VERSION < 6
    gazebo::setupClient(argc, argv);

#else
    gazebo::client::setup(argc, argv);
#endif
    // Create our node for communication
    _node= gazebo::transport::NodePtr(new gazebo::transport::Node());
    _node->Init();

    _bInited=true;
    return "";
}
////////////////////////////////////////////////////////////////////////////////
void CGazebo::stop()
{
    _node=nullptr;

    // Make sure to shut everything down.
#if GAZEBO_MAJOR_VERSION < 6
    gazebo::shutdown();
#else
    gazebo::client::shutdown();
#endif
}



/////////////////////////////////////////////////////////////////////////////
/// CGzModelReader
/////////////////////////////////////////////////////////////////////////////
std::map<std::string, long> CGzModelReader::gzModelName2Id;
std::map<std::string, long> CGzModelReader::gzLinkName2Id;
std::map<std::string, ignition::math::Vector3d> CGzModelReader::gzModelBoundingBox;

CGzModelReader::CGzModelReader()
{
    _modeltopicname="/gazebo/default/ariac/model";
}


std::string CGzModelReader::init()
{
   _modeltopicname = gzconfig.getSymbolValue<std::string>("gazebo.modeltopicname","ERROR");
    if(_modeltopicname=="ERROR")
        return "Bad model topic name";
    return "";

}


/////////////////////////////////////////////////////////////////////////////
void CGzModelReader::start()
{
    _sub = CGazebo::_node->Subscribe<gazebo::msgs::Model>(_modeltopicname, &CGzModelReader::onUpdate,this);
    Globals.sleep(1000);
    std::cout << "Gazebo model reader subscriber"  << _sub << "started\n";
}
/////////////////////////////////////////////////////////////////////////////
void CGzModelReader::stop()
{
    _sub->Unsubscribe();
    // want to kill gazebo update of model callback
    _sub.reset();
    std::cout << "Gazebo model reader subscriber stopped\n";
}

#include <sys/stat.h>
#include "aprs_headers/Config.h"
#include <algorithm>



/////////////////////////////////////////////////////////////////////////////
void CGzModelReader::onUpdate(ConstModelPtr &_msg)
{
    std::string name = _msg->name();

    // only save links for gear related models.
    if(name.find("sku")==std::string::npos)
        return;

    ignition::math::Pose3d  pose = gazebo::msgs::ConvertIgn(_msg->pose());
    tf::Pose tfpose(tf::Quaternion(pose.Rot().X(),
                                   pose.Rot().Y(),
                                   pose.Rot().Z(),
                                   pose.Rot().W()),
                    tf::Vector3(pose.Pos().X(),
                                pose.Pos().Y(),
                                pose.Pos().Z()));





#if 0
    //"model://gear_support/meshes/new_big_gear_Rotatex_Centered_ZeroZmin.stl"
    std::string meshfile;
//    std::cout << _msg->DebugString();
    if(_msg->link_size()>0)
    {
        CLEANSTORE(meshfile, _msg->link(0).visual(0).geometry().mesh().filename(), "") ;
        double x,y,z;
        CLEANSTORE(x, _msg->link(0).visual(0).geometry().mesh().scale().x(), -1) ;
        CLEANSTORE(y, _msg->link(0).visual(0).geometry().mesh().scale().y(), -1) ;
        CLEANSTORE(z, _msg->link(0).visual(0).geometry().mesh().scale().z(), -1) ;
        tf::Vector3 scale(x,y,z);
    }
#else
    std::string meshfile;
    tf::Vector3 scale(1.,1.,1.);

#endif

    // Lock updates of instance and model link to id
    std::lock_guard<std::mutex> guard(_mymutex);
    ShapeModel::instances.storeInstance(name, tfpose, meshfile, scale);


//    const google::protobuf::Message *prototype =
//      google::protobuf::MessageFactory::generated_factory()->GetPrototype(
//          _msg->GetDescriptor());
//    std::unique_ptr<google::protobuf::Message> payload(prototype->New());
// //   payload->ParseFromString(response->serialized_data());
//    payload->ParseFromString(_msg->SerializeAsString());
//    std::cout << payload->DebugString() << std::endl;
 //   std::cout << _msg->DebugString() << std::endl;


    // Dont really for link id after all models are read once.
    if(Globals.bReadAllInstances)
        return;

    ignition::math::Vector3d bbox(0,0,0);
    if(_msg->visual_size()>0)
    {
        const gazebo::msgs::Visual visual ( _msg->visual(0));
        gazebo::msgs::Geometry g;
        gazebo::msgs::BoxGeom *bg;
        gazebo::msgs::Vector3d v;
        // these macros catch null pointer access
        VALIDSTORE(g, visual.geometry()) ;
        VALIDSTORE(bg, g.mutable_box()) ;
        VALIDSTORE(v, bg->size()) ;
        bbox.Set( v.x(), v.y(), v.z());
    }
    gzModelBoundingBox[name]=bbox;

    gzModelName2Id[name]=_msg->id();
    for(size_t j=0; j<_msg->link_size(); j++)
    {
        const ::gazebo::msgs::Link&  link (_msg->link(0));
        std::string fullyInstantiatedName = name + "::" + link.name();
        gzLinkName2Id[fullyInstantiatedName] =link.id();
    }

}


//////////////////////////////////////////////////////////////////////////////
/// CGzRobotHandler
///////////////////////configure///////////////////////////////////////////////////////
std::string CGzRobotHandler::init(std::string robotName
                           //std::string gzRobotJointPrefix,
                           //std::string gzRobotCmdTopicName,
                           //std::string gzRobotStatusTopicName
                           )
{

    // Robot joint name prefix, which is a mess
    _mGzRobotJointPrefix = gzconfig.getSymbolValue<std::string>("gazebo."+robotName+".gzJointPrefix","ERROR");

    if(_mGzRobotJointPrefix=="ERROR")
        return "Bad CGzRobotHandler Joint Prefix name. i.e., gazebo."+robotName+".gzJointPrefix";

    _mGzRobotCmdTopicName = gzconfig.getSymbolValue<std::string>("gazebo."+robotName+".gzRobotCmdTopicName","ERROR");
    _mGzRobotStatusTopicName = gzconfig.getSymbolValue<std::string>("gazebo."+robotName+".gzRobotStatusTopicName","ERROR");
    if(_mGzRobotCmdTopicName=="ERROR" || _mGzRobotStatusTopicName == "ERROR")
        return "Bad CGzRobotHandler topic(s) name. Either gazebo."+robotName+".gzRobotCmdTopicName or gazebo."+robotName+".gzRobotStatusTopicName";

    return "";
}
/////////////////////////////////////////////////////////////////////////////
void CGzRobotHandler::start(std::vector<std::string> robotnames,
                              std::vector<std::string> fingernames)
{
    _bRunning=true;
    _robotCmdJoints.name=robotnames;
    _gripperJoints.name = fingernames;
    this->_robotStatusJoints.name=robotnames;
    this->_robotStatusJoints.position.resize(robotnames.size(), 0.0);
    this->_robotStatusJoints.velocity.resize(robotnames.size(), 0.0);
    this->_robotStatusJoints.effort.resize(robotnames.size(), 0.0);


    // create publish gazebo connection
    if(!_mGzRobotCmdTopicName.empty())
    {
        _robotCmdPub= CGazebo::_node->Advertise<message::JointsComm>(_mGzRobotCmdTopicName);
    }

    if(!_mGzRobotStatusTopicName.empty())
    {
        _robotStatusSub= CGazebo::_node->Subscribe<message::JointsComm>(_mGzRobotStatusTopicName,
                                                                        &CGzRobotHandler::OnStatusUpdate,this);
    }
}
/////////////////////////////////////////////////////////////////////////////
void CGzRobotHandler::stop() {
    _bRunning=false;
}
/////////////////////////////////////////////////////////////////////////////
bool CGzRobotHandler::updateRobot(double dTime, sensor_msgs::JointState joints)
{
    _writeCycles++;

    // if same joints do not update
    if(_robotCmdJoints.position==joints.position &&
            _robotCmdJoints.name==joints.name)
        return false;

    // Copy and save robot joint position values.
    {
        std::lock_guard<std::mutex> guard(_mymutex);
        _robotCmdJoints.name=joints.name;
        _robotCmdJoints.position=joints.position;
    }

    message::JointsComm jnts;
    for(size_t i=0; i< _robotCmdJoints.name.size(); i++)
    {
        jnts.add_name(_mGzRobotJointPrefix+ _robotCmdJoints.name[i].c_str());
        if(i >= _robotCmdJoints.position.size())
        {
            std::cerr << "Wrong number of joints in CGzJointCmdWriter\n";
            continue;
        }
        jnts.add_position(joints.position[i]);
    }
    _robotCmdPub->Publish(jnts);

    return true;
}

//////////////////////////////////////////////////////////////////////////////
void CGzRobotHandler::OnStatusUpdate(ConstRobotCmdPtr &_msg)
{
    std::lock_guard<std::mutex> guard(_mymutex);
    _readCycles++;
    if(_msg->name_size() == 0)
    {
        std::cout << "Problem with on robot status update\n";
        return;
    }
    this->_robotStatusJoints.name.clear();
    this->_robotStatusJoints.position.clear();
    this->_robotStatusJoints.velocity.clear();
    this->_robotStatusJoints.effort.clear();

    for (size_t i=0; i< _msg->name_size(); i++) {
        _robotStatusJoints.name.push_back(_msg->name(i));
        if(i< _msg->position_size())
            _robotStatusJoints.position.push_back(_msg->position(i));
        if(i< _msg->velocity_size())
            _robotStatusJoints.velocity.push_back(_msg->velocity(i));
        if(i< _msg->effort_size())
            _robotStatusJoints.effort.push_back(_msg->effort(i));
    }
}


//////////////////////////////////////////////////////////////////////////////
bool CGzRobotHandler::updateGripper(double dTime, sensor_msgs::JointState joints)
{
    if(joints.position.size()>0)
    {
        _gripperJoints.velocity.clear(); // allow change of velocity
        if(_gripperJoints.position==joints.position)
            return false;
        _gripperJoints.position=joints.position;
        _gripperJoints.name=joints.name;

        for(size_t i=0; i< _gripperJoints.name.size(); i++)
        {
            // Convert to a Gazebo JointCmd message
            gazebo::msgs::JointCmd  msg;

            msg.set_name(_mGzRobotJointPrefix+ _gripperJoints.name[i].c_str());
            double val=joints.position[i];
            ::gazebo::msgs::PID * position = new ::gazebo::msgs::PID();

            position->set_target(val);
            // Be careful:   delete position_;
            // minor leak.
            msg.set_allocated_position(position);

            _robotCmdPub->Publish(msg);
        }
    }
    else if(joints.velocity.size()>0)
    {
        _gripperJoints.position.clear(); // allow change of position
        if(_gripperJoints.velocity==joints.velocity)
            return false;
        _gripperJoints.velocity=joints.velocity;
        _gripperJoints.effort=joints.effort;
        _gripperJoints.name=joints.name;

        for(size_t i=0; i< _gripperJoints.name.size(); i++)
        {
            // Convert to a Gazebo JointCmd message
            gazebo::msgs::JointCmd  msg;

            msg.set_name(_mGzRobotJointPrefix+ _gripperJoints.name[i].c_str());
            double val=joints.velocity[i];
            ::gazebo::msgs::PID * velocity = new ::gazebo::msgs::PID();

            velocity->set_target(val);
            // Be careful:   delete velocity;
            // minor leak.
            msg.set_allocated_velocity(velocity);
            msg.set_force(_gripperJoints.effort[i]);  // max force

            _robotCmdPub->Publish(msg);
        }
    }
    else if(joints.effort.size()>0)
    {
        _gripperJoints.position.clear(); // allow change of position
        if(_gripperJoints.effort==joints.effort)
            return false;
        _gripperJoints.effort=joints.effort;
        _gripperJoints.name=joints.name;

        for(size_t i=0; i< _gripperJoints.name.size(); i++)
        {
            // Convert to a Gazebo JointCmd message
            gazebo::msgs::JointCmd  msg;

            msg.set_name(_mGzRobotJointPrefix+ _gripperJoints.name[i].c_str());
            double val=10.; // dummy for now
            ::gazebo::msgs::PID * velocity = new ::gazebo::msgs::PID();
            velocity->set_target(val);
            msg.set_allocated_velocity(velocity);
            msg.set_force(_gripperJoints.effort[i]);  // max force
            _robotCmdPub->Publish(msg);
        }
    }
    return true;
}

//////////////////////////////////////////////////////////////////////////////
/// CGzParallelGripper
//////////////////////////////////////////////////////////////////////////////
std::string CGzParallelGripper::init(std::string robotName)
{
    // gazebo communication topic names - from config.ini gazebo section
    _mGzGripperCmdTopicName = gzconfig.getSymbolValue<std::string>("gazebo."+robotName+".gzGripperCmdTopicName","ERROR");
    _mGzGripperStatusTopicName = gzconfig.getSymbolValue<std::string>("gazebo."+robotName+".gzGripperStatusTopicName","ERROR");
     if(_mGzGripperCmdTopicName=="ERROR"|| _mGzGripperStatusTopicName =="ERROR")
         return "Bad model topic(s) name. Either gazebo."+robotName+".gzGripperCmdTopicName or gazebo."+robotName+".gzGripperStatusTopicName";

    _mEE=-1;
    return "";
}

/////////////////////////////////////////////////////////////////////////////
void CGzParallelGripper::start()
{
    _bRunning=true;
    if(Globals.bGzGripperPlugin && !_mGzGripperCmdTopicName.empty())
    {
        _gripperpub = CGazebo::_node->Advertise<message::GripCommand>(_mGzGripperCmdTopicName);
    }

    if(Globals.bGzGripperPlugin && !_mGzGripperStatusTopicName.empty())
    {
        _grippersub=CGazebo::_node->Subscribe<message::GripCommand>(_mGzGripperStatusTopicName.c_str(), &CGzParallelGripper::OnStatusUpdate,this);;
    }

}
/////////////////////////////////////////////////////////////////////////////
void CGzParallelGripper::stop() {
    _bRunning=false;
}

/////////////////////////////////////////////////////////////////////////////
void CGzParallelGripper::OnStatusUpdate(ConstGripperCmdPtr &_msg)
{
    std::lock_guard<std::mutex> guard(_mymutex);
    _readCycles++;
    this->_nGripperState=_msg->state();
}
/////////////////////////////////////////////////////////////////////////////
int CGzParallelGripper::isGrasping()
{
    std::lock_guard<std::mutex> guard(_mymutex);
    return this->_nGripperState;
}

/////////////////////////////////////////////////////////////////////////////
bool CGzParallelGripper::updateGripper(double time, double eepercent)
{
    if(!Globals.bGzGripperPlugin)
    {
        std::cerr << "updating Pavel gripper plugin but not enabled\n";
        return false;
    }

    // check to see if sent last time. If so, skip.
    if(_mEE == eepercent)
        return false;

    // Convert to a Gazebo Custom GripCommand message
    message::GripCommand  msg;
    if(eepercent>0.0)
        msg.set_enable(false);
    else
        msg.set_enable(true);

    _gripperpub->Publish(msg);
    _mEE = eepercent;
    return true;
}
