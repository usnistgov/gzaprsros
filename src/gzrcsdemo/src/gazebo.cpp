
#include "aprs_headers/Debug.h"

#include "gzrcsdemo/gazebo.h"
#include "gzrcsdemo/Globals.h"
#include "gzrcsdemo/Shape.h"
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

}
////////////////////////////////////////////////////////////////////////////////
void CGazebo::shutdown()
{
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
   // _sub.reset();
    _sub = nullptr;
//    CGazebo::_node->Fini();
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


