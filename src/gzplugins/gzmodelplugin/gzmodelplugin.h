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


/**
 * @brief The gzModelPlugin class is a Gazebo World plugin to report
 * on the kitting objects (models that start with "sku" name).
 * Gazebo subscribers to the model message will get the name and pose.
 * Included in the intial message publish is the bounding box of the object
 * is also pusblished to allow listeners to understand the top of the model
 * for simplified grasping.
 */
class gzModelPlugin : public WorldPlugin
{

public:
    gzModelPlugin();

    /// \brief Load function
    ///
    /// Called when a Plugin is first created, and after the World has been
    /// loaded. This function should not be blocking.
    /// \param[in] _world Pointer to the World
    /// \param[in] _sdf Pointer to the SDF element of the plugin.
    void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf);
    /**
     * @brief Connect initialize and start publishing kitting object
     * pose data  if listensers.
     */
    void Connect();

    /**
     * @brief onWorldUpdate cyclic event callbacvk from gazebo. Update model
     * poses. Publish new object information.
     */
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
