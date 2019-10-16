//#ifndef _CRCLAPI_H
//#define _CRCLAPI_H

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

#ifndef _CrclApi_h_
#define _CrclApi_h_

#include <memory>

#include "aprs_headers/Core.h"
#include "aprs_headers/IRcs.h"
#include "aprs_headers/Conversions.h"

#include "gzrcs/Kinematics.h"
#include "gzrcs/Controller.h"

/**
 * \brief CCrclApi provides some inline Crcl commands that are queued to a given CNC motion queue.
 */
class CCrclApi
{
public:

    /**
     * @brief rates specifies the motion rates for joint, linear, angular and gripper for a Crcl motion.
     */
    static RCS::IRate rates;

    /*!
     * \brief Constructor of commands requires reference to Controller object.
     * \param cnc is pointer to CController instance.
     */
    CCrclApi(std::shared_ptr<RCS::CController> cnc)  ;

    /**
     * @brief setVelocity - sets rate of speeds for joint, linear, angular and gripper.
     * Rates of speed for velocity, with 10* for acceleration and 100* for jerk.
     * @param speed baseline velocity
     */
    void setVelocity(double speed);

    /**
     * @brief getSpeeds - gets rate of speeds as ROS crcl profile message.
     * Rates of speed for velocity, with 10* for acceleration and 100* for jerk.
     * Uses current velocity rate to determine acc (10x) and jerk (100x).
     * Fixme: this should incorporate acceleration multiplier for setting acceleration.
     * @param speed baseline velocity
     */
    ::crcl_rosmsgs::CrclMaxProfileMsg getSpeeds();

    /**
     * @brief slow - slow rate of speeds for joint, linear, angular and gripper.
     * e.g., linear speed: .1 m/sec vel, 1 m/sec*sec acc etc.
     */
    void slow();

    /**
     * @brief medium - medium rate of speeds for joint, linear, angular and gripper.
     * e.g., linear speed: 1 m/sec vel, 10 m/sec*sec acc etc.
     */
    void medium();

    /**
     * @brief fast - fast rate of speeds for joint, linear, angular and gripper.
     * e.g., linear speed: 10 m/sec vel, 100 m/sec*sec acc etc.
     */
    void fast();

    /**
     * @brief setDwell- set timing for dwelling (i.e., during grasping)
     * @param d
     */
    void setDwell(double d) ;

    /**
     * @brief setGraspingDwell sets the time period to wait while grasping,
     * so the gazebo grasping kludge plugin can work. It expects a certain time with contact
     * before it initiates a joint association between the grasped object and the gripper "palm".
     * @param d time in seconds
     */
    void setGraspingDwell(double d);

    /**
     * @brief smartCloseGripper uses gazebo contact sensor to close left and right fingers of the gripper
     * until contact is made. Once both fingers are in contact a final squeeze is applied.
     */
    void smartCloseGripper();

    /**
     * @brief setContactGripper sets an absolute position of the fingers of the gripper.
     * @param ee position of gripper fingers. Each finger has an associated multiplier.
     */
    void setContactGripper(double ee);

    /**
     * @brief setVelGripper creates a CanonCmd for type CANON_SET_EE_PARAMETERS,
     * where this uses CRCL set end effector parameters. Parameter names are action/vel/fmax with
     * matching values. Inputs are values for these parameters.
     * @param vel - speed
     * @param fmax - maximum force of motion. Motion stops when reached.
     */
    void setVelGripper(double vel, double fmax);

    /*!
     * \brief setGripper set the robot gripper to the given percentage (from 0..1). .
     * \param ee end effector percentage close(0)..open(1). Note gripper could be closed at 1!
     */
    void setGripper(double ee);

    /*!
     * \brief Return pointer to CNC instance of this robot command object.
     * \return cnc is pointer to CController instance.
     */
    std::shared_ptr<RCS::CController> cnc()
    {
        return _nc;
    }

    /*!
     * \brief Robot picks up an object at pose with given objname.
     * \param pose of the given object to pick up.
     */
    void pick(tf::Pose pose);

    /*!
     * \brief Robot moves to given Cartesian pose, and may move object .
     * \param pose of the given object to move.
     * \param objname name of the object that is being moved.
     */
    void moveTo(tf::Pose pose);

    /*!
     * \brief Robot dwells for given dwell time in seconds. .
     * \param dwelltime time to dwell in seconds.
     * */
    void doDwell(double dwelltime);

    /*!
     * \brief Robot opens gripper. .
     */
    void openGripper();

    /*!
     * \brief Robot closes gripper. .
     */
    void closeGripper();


    /**
     * @brief setAbsPosGripper sets the gripper mode to absolute positioning, and then
     * sends a set gripper command with the knowledge that the commmand is NOT precentage gripper closure, but absolute position.
     * Uses multiplier for opposing prismatic joints to distingh direction of final position (i.e., positive or negative).
     * @param abspos position to move gripper to.
     */
    void setAbsPosGripper(double abspos);

    /*!
     * \brief Robot places up an object at pose with given objname.
     * Retracts to given retraction offset from place pose.
     * \param pose of the given object to pick up.
     */
    void place(tf::Pose pose);


    /*!
     * \brief Robot moves joints as defined joint number vector to positions.
     * Coordinated joint motion "assumed".
     * \param jointnum is a list of joints to move.
     * \param positions is final joint position
     * \param vel is the velocity of the motion fast=10,med=1, slow=.1
     */
    void moveJoints(std::vector<long unsigned int> jointnum,
            std::vector<double> positions,
                    double vel=1.);

    ///////////////////////////////////////////////////
    // These are object dependent
    /*!
     * \brief Robot picks up an object at pose with given objname.
      * \param objname name of the object that is being picked up.
      * this assumes the object is part of the world model and has a pose
      * FIXME additional information such as grasping offset.
     */
    int pickup(std::string objname);

    /**
     * @brief move_to to a given object centroid pose.
     * @param objname - this is based on the gazebo model name of an instance
     * @return  0 sucess.
     */
    int moveTo(std::string objname);

    int retract(std::string objname);
    int retract(double amt=0.01);
    int approach(std::string objname);
    int grasp(std::string objname);

    double _mygraspdwell; /**<  global dwell time after grasp and after release */
    double _mydwell; /**<  global dwell time between motions in seconds */

protected:
    std::shared_ptr<RCS::CController>_nc; /**<  cnc to queue motion commands */
    static int _crclcommandnum; /**<  crcl command number for the robots -i.e., this one */

};

extern  std::shared_ptr<CCrclApi>  crclApi;
#endif
