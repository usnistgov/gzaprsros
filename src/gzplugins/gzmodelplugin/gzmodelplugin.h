#ifndef GZMODELPLUGIN_H
#define GZMODELPLUGIN_H

#include <map>

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/World.hh>

#if GAZEBO_MAJOR_VERSION >= 8
#include <ignition/math/Pose3.hh>
#include <ignition/math/Vector3.hh>
#include <ignition/math/Quaternion.hh>
typedef ignition::math::Pose3d Pose;
typedef ignition::math::Vector3d Vector3d;
typedef ignition::math::Quaterniond Quaterniond;
#endif

namespace gazebo
{

class gzModelPlugin : public WorldPlugin
{

public:
    gzModelPlugin();

    void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf);
    void Connect();
    void onWorldUpdate();
private:
    physics::WorldPtr world;
    event::ConnectionPtr update_connection;
    gazebo::transport::PublisherPtr pub;
    gazebo::transport::NodePtr node;
#if GAZEBO_MAJOR_VERSION >= 8
    std::map<std::string, Pose> last_location;
#else
    std::map<std::string,math::Pose> last_location;
#endif
    common::Time last_update_time_;
    size_t n_readers;
    bool b_debug;
    double d_update_period ;
;

};
GZ_REGISTER_WORLD_PLUGIN(gzModelPlugin)
}
#endif // GZMODELPLUGIN_H
