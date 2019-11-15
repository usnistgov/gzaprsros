#ifndef GAZEBO_H
#define GAZEBO_H

#include <string>

#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>

#if GAZEBO_MAJOR_VERSION < 6
#include <gazebo/gazebo.hh>
#else
#include <gazebo/gazebo_client.hh>
#endif


#if GAZEBO_MAJOR_VERSION < 8
#include <gazebo/math/Pose.hh>
#else

#endif

#include "aprs_headers/Config.h"
#include <sensor_msgs/JointState.h>
#include <unordered_set>

extern gazebo::transport::NodePtr _node;

/**
 * @brief The CGazebo class waits for a gazebo server to be running - i.e., gzserver program.
 * Loads gazebos argc,argv parameters.
 * Creates a node for gazebo protobuf communication used by any gazebo publisher/subscriber.
 */
class CGazebo
{
public:
    /**
     * @brief CGazebo constructor.
     */
    CGazebo();

    /**
     * @brief init waits for a gazebo server to be running - i.e., gzserver program.
     * Loads gazebos argc,argv parameters.
     * Creates a node for gazebo protobuf communication used by any gazebo publisher/subscriber.
     * @param robotname name of robot.
     * @param inifile file path of ini file for all gazebo configuration.
     * @return empty string if no error.
     */
    std::string  init(std::string robotname,
                      std::string inifile);

    /**
     * @brief isInited
     * @return true if gazebo sucessfully inited.
     */
    bool isInited() { return _bInited; }

    /**
     * @brief stop make sure to shut everything down/disconnected in gazebo.
     */
    void stop();

    /**
     * @brief shutdown shuts down gazebo.
     * Hope it averts boost null pointer exception in ignition transport.
     */
    void shutdown();

    /**
     * @brief _node used by to establish gazebo communication by all gazebo clients.
     */
    static std::string _inifile;
    static std::string _robotname;


private:
    static  bool _bInited; /**< inited once to establsh communcation */
};
extern std::shared_ptr<CGazebo> gz;

/**
 * @brief The CGzModelReader class establishes communication subscriber to read all gz models
 * updates. Update only caputres name and centroid pose.
 */
class CGzModelReader
{
public:
    CGzModelReader();

    /**
     * @brief configure read ini file for parameterization. Don't need robot name.
     * @return empty string success, error message otherwise.
     * Will throw and easy to catch if debugging.
     */
    std::string init();

    /**
     * @brief start subscribes to topic /gazebo/default/ariac/model. Fixme: make configurable
     * topic name.
     */
    void start();
    /**
     * @brief stop cleanly detaches gazebo update of model callback.
     */
    void stop();
    /**
     * @brief onUpdate save model instance name and pose. Convert from
     * ignition::math to tf::Pose.
     * Hard coded call to demo  ShapeModel::instances.storeInstance(_msg->name(), tfpose);
     * @param msg model communication message from gazebo.
     */
    void onUpdate(ConstModelPtr & msg);

    static std::map<std::string, long> gzModelName2Id;
    static std::map<std::string, long> gzLinkName2Id;
    static std::map<std::string, ignition::math::Vector3d> gzModelBoundingBox;

private:
    gazebo::transport::SubscriberPtr _sub; /**< subscriber to topic /gazebo/default/ariac/model */
    std::mutex _mymutex;
    std::string _modeltopicname;
};



#endif // GAZEBO_H
