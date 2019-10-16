#ifndef GAZEBO_H
#define GAZEBO_H

#include <string>

//#include <gazebo/math/Pose.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <functional>

#if GAZEBO_MAJOR_VERSION < 6
#include <gazebo/gazebo.hh>
#else
#include <gazebo/gazebo_client.hh>
#endif

#include <sensor_msgs/JointState.h>
#include <functional>
#include <tf/tf.h>

namespace gazebo
{
typedef boost::shared_ptr<gazebo::msgs::Joint const> ConstJointPtr;
}


/**
 * @brief The CGazebo class waits for a gazebo server to be running - i.e., gzserver program.
 * Loads gazebos argc,argv parameters.
 * Creates a node for gazebo protobModelStatesuf communication used by any gazebo publisher/subscriber.
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
     */
    void init();

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
     * @brief getWorldSdf returns the sdf xml from the dfault world.
     * In theory there could be multiple worlds.
     */
    std::string getWorldSdf();


    /**
     * @brief _node used by to establish gazebo communication by all gazebo clients.
     */
    static gazebo::transport::NodePtr _node;


private:
    static  bool _bInited; /**< inited once to establsh communcation */
};
extern CGazebo gz;



/**
 * @brief The CGzModelReader class establishes communication subscriber to read all gz models
 * updates. Update only caputres name and centroid pose.
 */
class CGzModelReader
{
public:
    CGzModelReader();
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
    /**
     * @brief AssignUpdateCallback when model update occurs calls the function
     * assigned by this method.
     */
    void AssignUpdateCallback(std::function<void(std::string, tf::Pose)>);

private:
    std::function<void(std::string, tf::Pose)> _updateCallback;
    gazebo::transport::SubscriberPtr _sub; /**< subscriber to topic /gazebo/default/ariac/model */
};



#endif // GAZEBO_H
