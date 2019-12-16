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
/**
 * @brief The gzJntCmdPlugin class is a gzebo plugin that one would insert
 * into either the world or sdf file. It read/writes Google protobuf custom message
 *
\code
package message;

message JointsComm {
    repeated string name = 1;
    repeated double position = 2;
    repeated double velocity = 3;
    repeated double effort = 4;
}
\endcode

The joint communication custom message acts like the ROS sensor
message joint status.
 */
class  gzJntCmdPlugin : public ModelPlugin
{

public:
    /**
     * @brief gzJntCmdPlugin empty constructor. Sets flags and
     * set timing to 0.005 seconds.
     */
    gzJntCmdPlugin();

    ~gzJntCmdPlugin();

    /**
     * @brief Load overloaded  model plugin method to setup the pluing.
     * Initializes paraemters, saves world, parsed SDF for parameter values,
     * and creates pusblisher and subscriber for joint status and commands.
     * @param[in] _model Pointer to the Model
     * @param[in] _sdf Pointer to the SDF element of the plugin. The sdf contains
     * the parameter settings used by this plugin.
     */
    virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
    /**
     * @brief setJntPosition mutexed update of a joint value that will be written
     * to Gazebo robot model.
     * @param name name of the joint.
     * @param pos position of the robot joint.
     */
    void setJntPosition(const std::string &name, const double &pos);
    void updateJointStatus();
    /**
     * @brief publishJointStates publishes all the joint positions to
     * Gazebo simulation at once. But only if the joints are different.
     */
    void publishJointStates();
    /**
     * @brief onUpdate handles the Gazebo world update event callback.
     * @param _info
     */
    void onUpdate (const common::UpdateInfo & _info);

    /// \brief Handle incoming joint command message
    /// \param[in] _msg allow joint name or pos/vel Pid values
    void onMsg(const ConstJointCmdPtr &msg);
    /**
     * @brief onJointsCommMsg deprecated handling of gazebo one-by-one
     * joint update message.
     * @param msg
     */
    void onJointsCommMsg(const ConstJointsCommPtr &msg);
    /**
     * @brief fingers names of gripper finger joints to ignore when updating robot.
     */
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
GZ_REGISTER_MODEL_PLUGIN(gzJntCmdPlugin)

}
#endif // GzJntCmdPlugin_H
