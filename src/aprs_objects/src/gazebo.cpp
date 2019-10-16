

#include "gazebo.h"
#include <chrono>
#include <thread>

// Global helpers
static std::string getexepath()
{
  char result[ PATH_MAX ];
  ssize_t count = readlink( "/proc/self/exe", result, PATH_MAX );
  return std::string( result, (count > 0) ? count : 0 );
}
static std::string getexefolder()
{
    std::string exepath = getexepath();
    return  exepath.substr(0, exepath.find_last_of('/') + 1);
}

// static definitions
gazebo::transport::NodePtr CGazebo::_node;
bool CGazebo::_bInited=false;

// extern global definitions
CGazebo gz;

////////////////////////////////////////////////////////////////////////////////
CGazebo::CGazebo()
{
}

////////////////////////////////////////////////////////////////////////////////
void CGazebo::init()
{
    int argc=1;
    char* argv[1];
    argv[0]=(char *) getexepath().c_str();

#if 0
    // Wait for gazebo server to be running -  gzserver program
    std::cout << "Connecting to gazebo server";
    while( Globals.procFind("gzserver") < 0)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        std::cout << "." << std::flush;
    }
    std::cout << "\n" << std::flush;
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

#endif
    // Load gazebo
#if GAZEBO_MAJOR_VERSION < 6
    gazebo::setupClient(argc, argv);

#else
    gazebo::client::setup(argc, argv);
#endif
    // Create our node for communication
    _node= gazebo::transport::NodePtr(new gazebo::transport::Node());
    _node->Init();

    // Publish to a Gazebo topic
    //pub = node->Advertise<gazebo::msgs::Joint>("~/mtconnect/mtconnect_cmd");

    // Wait for a subscriber to connect
    //pub->WaitForConnection();

    // This works
    // sub = node->Subscribe<gazebo::msgs::Model>("/gazebo/default/ariac/model", cb);
    //sub = node->Subscribe<gazebo::msgs::Model>("/gazebo/default/ariac/model", &ModelReader::OnUpdate,this);

    _bInited=true;

}
void CGazebo::stop()
{
    // Make sure to shut everything down.
#if GAZEBO_MAJOR_VERSION < 6
    gazebo::shutdown();
#else
    gazebo::client::shutdown();
#endif
}

/////////////////////////////////////////////////////////////////////////////
std::string CGazebo::getWorldSdf()
{
    // Get the latest world in SDF.
    boost::shared_ptr<gazebo::msgs::Response> response =
            gazebo::transport::request("default", "world_sdf");

    gazebo::msgs::GzString msg;

    // Make sure the response is correct
    if (response->response() != "error" && response->type() == msg.GetTypeName())
    {
        // Parse the response message
        msg.ParseFromString(response->serialized_data());
    }
    std::string msgData = msg.data();
    //std::cout << msgData << std::flush;
    return msgData;
}

//
//std::vector<std::string> CGazebo::getTopics()
//{
//    transport::ConnectionPtr connection = transport::connectToMaster();
//    if (connection)
//    {
//      std::string topicData;
//      msgs::Packet packet;
//      msgs::Request request;
//      msgs::GzString_V topics;

//      request.set_id(0);
//      request.set_request("get_topics");
//       connection->EnqueueMsg(msgs::Package("request", request), true);

//       try
//       {
//           connection->Read(topicData);

//       }
//       catch(std::exception &_e)
//       {

//           gzerr << "Error during connection read : " << _e.what() << std::endl;
//           return;

//       }

//       packet.ParseFromString(topicData);
//       topics.ParseFromString(packet.serialized_data());
//    }
//}

/////////////////////////////////////////////////////////////////////////////
/// CGzModelReader
/////////////////////////////////////////////////////////////////////////////
CGzModelReader::CGzModelReader()
{

}

/////////////////////////////////////////////////////////////////////////////
void CGzModelReader::start()
{
    _sub = CGazebo::_node->Subscribe<gazebo::msgs::Model>("/gazebo/default/ariac/model", &CGzModelReader::onUpdate,this);
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    std::cout << "Gazebo model reader subscriber started\n";
}
/////////////////////////////////////////////////////////////////////////////
void CGzModelReader::stop()
{
    _sub->Unsubscribe();
    // want to kill gazebo update of model callback
    _sub.reset();
    std::cout << "Gazebo model reader subscriber stopped\n";
}



/////////////////////////////////////////////////////////////////////////////
void CGzModelReader::onUpdate(ConstModelPtr &_msg)
{
    //    instances[_msg->name()]=_msg->pose();
    ignition::math::Pose3d  pose = gazebo::msgs::ConvertIgn(_msg->pose());
    tf::Pose tfpose(tf::Quaternion(pose.Rot().X(),
                                   pose.Rot().Y(),
                                   pose.Rot().Z(),
                                   pose.Rot().W()),
                    tf::Vector3(pose.Pos().X(),
                                pose.Pos().Y(),
                                pose.Pos().Z()));
    _updateCallback(_msg->name(), tfpose);

    //    std::cout << "model update: " << _msg->DebugString();
}
/////////////////////////////////////////////////////////////////////////////
std::vector<double> convert_vector(ignition::math::Pose3d  pose )
{
    std::vector<double> pose_array;
    pose_array.push_back(pose.Pos().X());
    pose_array.push_back(pose.Pos().Y());
    pose_array.push_back(pose.Pos().Z());
    pose_array.push_back(pose.Rot().Roll());
    pose_array.push_back(pose.Rot().Pitch());
    pose_array.push_back(pose.Rot().Yaw());
    return pose_array;
}



void CGzModelReader::AssignUpdateCallback(std::function<void(std::string, tf::Pose)> updateCallback)
{
    _updateCallback=updateCallback;
}
