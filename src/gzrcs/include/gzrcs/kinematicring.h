#ifndef KINEMATICRING_H
#define KINEMATICRING_H

#include "aprs_headers/Core.h"
#include "aprs_headers/IRcs.h"

namespace RCS {

/**
 * @brief The KinematicRing struct defines a kinematic transform
 * equation where the left hand side defines the goal and the right
 * hand side defines the robot. THe lhs is defined by at least one
 * matrice that defines the desired position/orientation or pose of the robot
 * defined as a transform. The rhs is a series of transforms that
 * at least contains the 0T6 series of transforms from the base joint
 * to the tip joint. However, both the lhs and the rhs may have
 * additional transforms that modify the kinematic equation. For
 * example, the rhs may have a gripper which is deined as another
 * transform.
 */
struct KinematicRing
{
    /**
     * @brief KinematicRing constructor. Sets all preexisting transoforms (base, gripper) to the
     * identity matrix.
     */
    KinematicRing();

    /**
     * @brief setGripperOffset (or end effector) offset, i.e., pose at end of robot arm
     * @param pose an offset pose from end of robot arm to end of gripper.
     */
    void setGripperOffset(tf::Pose pose);

    /**
     * @brief setToolOffset (possibly held by end effector) offset, i.e., pose at end of robot arm
     * @param offset
     */
    void setToolOffset(tf::Pose offset);

    /**
     * @brief setBaseOffset Set base offset from world into robot coordinate system as pose (or homogeneous matrice)
     * @param offset pose describing transform from world origin to robot origin
     */
    void setBaseOffset(tf::Pose transform);

    /**
     * @brief worldCoord transform robot pose into world coordinate system
     * @param robotpose pose given in robot coordinate system
     * @return  new transform in world coordinate system
     */
    tf::Pose worldCoord(tf::Pose robotpose);

    /**
     * @brief robotOnlyCoord  transform world pose into robot coordinate system
     * @param worldpose pose given in world coordinate system
     * @return new transform in robot coordinate system
     */
    tf::Pose robotOnlyCoord(tf::Pose worldpose);

    /**
     * @brief robotAndGripperCoord transform from world into robot, including robot gripper
     * @param worldpose transform describing 6D pose in world coordinate frame
     * @return new robot coordinate transform
     */
    tf::Pose robotAndGripperCoord(tf::Pose worldpose);

    /**
     * @brief robotRemoveGripperCoord given a pose in robot coordinate frame, remove gripper transform
     * @param pose
     * @return pose without gripper transform
     */
    tf::Pose robotRemoveGripperCoord(tf::Pose pose);

    /**
     * @brief robotAddGripperCoord given pose in robot coordinate frame, add in gripper transform.
     * @param robot_pose pose in robot coordinate frame
     * @return pose in robot with gripper coordinate frame
     */
    tf::Pose robotAddGripperCoord(tf::Pose robot_pose);

    /**
     * @brief addBaseTransform  adds base transform to input pose
     * @param pose robot coordinate frame pose
     * @return world coordinate frame pose
     */
    tf::Pose addBaseTransform(tf::Pose pose);

    /**
     * @brief gripperPose transform describing gripper
     * @return tf pose
     */
    tf::Pose  gripperPose();

    /**
     * @brief basePose pose describing transform from world coordinates into robot coordinates
     * @return tf pose
     */
    tf::Pose  basePose();

    /**
     * @brief toolPose tool attached to gripper transform. Default identity matrix.
     * @return tf pose
     */
    tf::Pose  toolPose();
protected:
    std::vector<tf::Pose> _prerobotxform;  /**< prerobot transforms  */
    std::vector<tf::Pose>  _postrobotxform; /**< postsrobot transforms  */
    tf::Pose &_gripperPose; /**< gripper transform  */
    tf::Pose & _basePose;  /**< world to robot base transform  */
    tf::Pose & _toolPose; /**< tool on gripper transform  */

};

}
#endif // KINEMATICRING_H
