#ifndef GMOVE
#define GMOVE

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

#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <sensor_msgs/JointState.h>

namespace gomotion {
typedef sensor_msgs::JointState_<std::allocator<void> > JointState;

/**
 * @brief The GoTrajParams struct defines the maximum speeds (vel, acc, jerk)
 * for the gomotion trajectory generator. Uses ROS default standard length units: meters.
 * Typical definition is 10,100,1000 for vel,acc,jerk.
 */
struct GoTrajParams {
    /**
     * @brief GoTrajParams empty constructor
     */
    GoTrajParams(){}
    /**
     * @brief GoTrajParams
     * @param _vel maximum velocity in meters/second
     * @param _acc maximum acceleration in meters/second/second
     * @param _jerk maximum jerk in meters/second/second/second
     */
    GoTrajParams(double _vel, double _acc, double _jerk) :
        vel(_vel), acc(_acc),jerk(_jerk)
    {
        
    }
    double vel; /*< max vel for each motion */
    double acc; /*< max accel for each motion */
    double jerk; /*< max jerk for each motion */
};

struct go_motion_interface;

/**
     * @brief The GoTraj class is the ROS front-end to the gomotion trajectory generator.
     *
     */
class GoTraj {
public:
    /**
         * @brief GoTraj constructor that creates a go_motion_interface() handler
         * for handling trajectory generation.
         */
    GoTraj();

    /**
         * @brief Init initializees a gomotion trajectory generator.
         * THIS MUST BE CALLED FIRST. You put in the current joint states
         * and the cycle time of each trajectory update.
         * motion into gomotion.
         * @param here where the current joints are located.
         * @param deltat cycle time of updates
         * @return sucess as 0. Throws std::exception otherwise....
         */
    int Init(JointState here, float deltat);

    /**
         * @brief InitPose initializees a pose trajectory generation.
         * @param here current pose position
         * @param there  goal pose position
         * @param tparams translational maximum speeds (vel, acc, jerk)
         * @param rparams rotational maximum speeds (vel, acc, jerk)
         * @return sucess as 0. Throws std::exception otherwise....
         */
    int InitPose(tf::Pose here,
                 tf::Pose there,
                 gomotion::GoTrajParams tparams,
                 gomotion::GoTrajParams rparams);

    /**
         * @brief InitPose initializees a pose trajectory generation.
         * @param here current ROS:tf pose location
         * @param there  goal ROS:tf  pose location
         * @param seconds number of seconds the move is achieved.
         * Must be >= minimum motion duration.
         * @return sucess as 0. Throws std::exception otherwise....
         */
    int InitPose(tf::Pose here,
                 tf::Pose there,
                 double seconds);
    /**
         * @brief InitJoints initializees a joints trajectory generation.
         * @param here current sensor_msgs::JointState
         * @param there goal sensor_msgs::JointState
         * @param params joint speeds (vel, acc, jerk)
         * @param bCoordinated true if all joints  are to reach destination at same time.
         * @return sucess as 0. Throws std::exception otherwise....
         */
    int InitJoints(JointState here,
                   JointState there,
                   std::vector<gomotion::GoTrajParams> params,
                   bool bCoordinated=true);
    /**
         * @brief InitJoints initializees a joints trajectory generation.
         * @param here current sensor_msgs::JointState
         * @param there goal sensor_msgs::JointState
         * @param seconds toatl duration of joint move
         * @param bCoordinated true if all joints  are to reach destination at same time.
         * @return sucess as 0. Throws std::exception otherwise....
         */
    int InitJoints(JointState here, JointState there,
                   double seconds, bool bCoordinated=true);

    /**
         * @brief AppendPose  append another ROS tf goal pose. Note
         * gomotion stops at each goal destination (i.e., no blending).
         */
    void AppendPose(tf::Pose);

    /**
         * @brief AppendJoints  append another sensor_msgs::JointState goal joints. Note
         * gomotion stops at each goal destination.
         */
    void AppendJoints(JointState);

    /**
         * @brief NextPose  get the next cycle pose calculated by the gomotion
         * trajectory geneator.
         * @return ROS tf pose representation.
         */
    tf::Pose NextPose();

    /**
         * @brief NextJoints get the next cycle of joint values calculated by the gomotion
         * trajectory geneator.
         * @return ROS sensor_msgs::JointState of the next joint values (pos,vel,acc).
         */
    JointState NextJoints();

    /**
         * @brief IsDone has gomotion reached its goal position.
         * @return true if done. False if still moving.
         */
    bool IsDone();

    /**
         * @brief InitStop intialize a stopping of the gomotion trajectory
         * so that the motion smoothly stops.
         */
    void InitStop(); // Then use next pose or next joints

private:
    boost::shared_ptr<go_motion_interface> pgm;
    size_t num_joints;
    int InitJoints(JointState here,
                   JointState there,
                   double seconds,
                   std::vector<gomotion::GoTrajParams> jparams,
                   bool bCoordinated);
    int InitPose(tf::Pose here, tf::Pose there, double seconds,
                 gomotion::GoTrajParams tparams,
                 gomotion::GoTrajParams rparams);

};
};


#endif
