#ifndef GZJNTCMDPLUGIN_H
#define GZJNTCMDPLUGIN_H

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>

#include <string>

#include <boost/bind.hpp>
#include <boost/thread/mutex.hpp>
#include "JointsComm.pb.h"

namespace gazebo
{
typedef boost::shared_ptr<gazebo::msgs::JointCmd const> ConstJointCmdPtr;
typedef boost::shared_ptr<message::JointsComm const> ConstJointsCommPtr;

enum CONTROL_TYPE {
    NO_CONTROL=0,
    POSITION_CONTROL,
    VELOCITY_CONTROL,
    FORCE_TORQUE_CONTROL
};
class  GzJntCmdPlugin : public ModelPlugin
{

public:
    GzJntCmdPlugin();
    ~GzJntCmdPlugin();

    virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
    void setJntPosition(const std::string &name, const double &pos);
    void updateJointStatus();
    void publishJointStates();
    void onUpdate (const common::UpdateInfo & _info);

    /// \brief Handle incoming message
    /// \param[in] _msg allow joint name or pos/vel Pid values
    void onMsg(const ConstJointCmdPtr &msg);
    void onJointsCommMsg(const ConstJointsCommPtr &msg);
    std::vector<std::string> fingers;

private:
    /// \brief A node used for transport
    transport::NodePtr node;

    transport::SubscriberPtr sub;  /// \brief A subscriber to a robots joints topic.
    transport::SubscriberPtr finger_sub;  /// \brief A subscriber to a gripper finger topic.
    transport::SubscriberPtr subJointsComm;  /// \brief A subscriber to a robots joints comm topic.
    gazebo::transport::PublisherPtr pub;  // feedback of joint values

    /// \brief Pointer to the model.
    physics::ModelPtr model;

    /// \brief Pointer to the world.
    physics::WorldPtr world;

    /// \brief Pointer to the joint.
    physics::JointPtr joint;

    std::string topicName;
    std::string robotCmd_topicName;
    std::string robotStatus_topicName;

//    std::string finger_topicName;
    std::vector<CONTROL_TYPE> joint_typeControl;

    std::string model_name;
    std::vector<physics::JointPtr> joints_ptr;

    // dynamic values
    std::vector<double> joint_pos;
    std::vector<double> joint_vel;
    std::vector<double> joint_effort;

    // saved
    std::vector<double> myjoint_postarget;
    std::vector<double> myjoint_veltarget;
    std::vector<double> myjoint_efftarget;


    bool bFlag;

    common::Time updateRate;  ///nano seconds between two updates

    double update_rate_;
    double update_period_;
    common::Time last_update_time_;
    boost::mutex m;
private:
    // Pointer to the update event connection
    event::ConnectionPtr updateConnection;
    bool b_debug;
    std::vector<std::string> joint_names;
    int _nstatus;
};
// Tell Gazebo about this plugin, so that Gazebo can call Load on this plugin.
GZ_REGISTER_MODEL_PLUGIN(GzJntCmdPlugin)

}
#endif // GzJntCmdPlugin_H
