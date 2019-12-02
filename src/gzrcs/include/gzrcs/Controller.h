// Controller.h

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

#ifndef _CONTROLLER_H
#define _CONTROLLER_H

#include <mutex>

// C++ headers
#include <memory>
#include <list>
#include <condition_variable>
#include <map>

// Nist headers
#include "aprs_headers/Core.h"
#include "aprs_headers/IRcs.h"
#include "aprs_headers/RCSThreadTemplate.h"
#include "aprs_headers/RCSMsgQueue.h"
#include "aprs_headers/Config.h"
#include "aprs_headers/IKinematic.h"

// Nist nc headers
#include "gzrcs/RCSInterpreter.h" // now included in RCS.h
#include "gzrcs/Gripper.h"
#include "gzrcs/kinematicring.h"

#ifdef GAZEBO
#include "gzrcs/gazebo.h"
#endif
#ifdef CRCL_DLL
namespace crcl
{
class crclServer;
}
#endif

#ifdef CRCL_ROS_TOPIC
class CRosCrclRobotHandler;
#endif

namespace RCS {

class CController;

extern Nist::Config robotconfig;
extern std::vector<std::shared_ptr<CController> > ncs;
extern std::mutex cncmutex;


/**
     * \brief The CController provides a collection for all the relevant controller pieces.
     *  The CController is the main controller class to collect all the references/pointers to instances in the project.
     * A global instance of this class, called "Controller", is created and is used throughout the code to reference various instances of
     * control objects (e.g., kinematics, joint writer, joint reader, etc.)
     *
     */
struct CController : public RCS::Thread,
        public KinematicRing
{
    typedef std::list<RCS::CCanonCmd> archive_message_list;
    RCS::CrclMessageQueue crclcmds; /**< queue of commands interpreted from Crcl messages */

    static const int NORMAL = 0;
    static const int EXITING = 1;
    static const int PAUSED = 2;
    static const int ERROR = 3;
    static const int ONCE = 4;
    static const int NOOP = 5;
    static const int AUTO = 6;
    static const int REVERSE=7;
    static const int REPEAT=7;
    int state;

    /**
     * @brief CController constructor that requires a name and cycle time for RCS thread timing.
     * @param name for debugging
     * @param cycletime in seconds.
     */
    CController(std::string name, double cycletime);

    ~CController(void);



    /**
     * @brief isBusy checks controller status to see if controller is processing command.
     * @return true if controller is processing command
     */
    bool isBusy();

    /**
     * @brief Verify verifies that all the pointer references in the controller have been instantiated (i.e., not null).
     * @return true if all have been instantiated
     */
    bool verify();


    /**
     * @brief ActionCyclic loop for the controller. Reads Crcl input mexsage queue, interprets into canon cmds if any, reads canon
     * cmds queue, interprets into robot command messages.
     * @return 0 if successful
     */
    virtual int action();

    /**
     * @brief Cleanup called when thread is terminating.
     */
    virtual void cleanup ( );

    /**
     * @brief setup routine for the controller.
     * initialze ROS, Gazebo, gripper communication.
     * Initialize motion rates for joint, linear, angular, and  gripper.
     * AddPostprocessFcn for updating CRCL status.
     */
    void setup();

    /**
     * @brief publishStatus publish status to CRCL or gazebo or robot interface.
     * Is a postprocess routine so runs inside of controller thread
     */
    void publishStatus();


    /**
      * @brief Creates a comma separated string of current state of robot. (Can use other separator).
      */
    std::string toString(std::string separator = ",");

    /**
     *\brief Creates a header line containing names of comma separated string fields that describes the current state of robot. (Can use other separator).
     */
    std::string dumpHeader(std::string separator = ",");


    /**
     * @brief setKinematics Routine to set the kinematics reference pointer. Uses the interface class IKinematics, but can have any implementation instance.
     * @param k
     */
    void setKinematics(boost::shared_ptr<RCS::IKinematic> k)
    {
        robotKinematics() = k;
    }

    /**
     * @brief update_robot Update position of robot.
     * For now compute cartesian and joint vel, acc, jerk of motion.
     * @return
     */
    bool updateRobot();

    /**
     * @brief where return currenct joint positions for robot AND gripper
     * @return ROS jointstate with names only joint positions.
     */
    sensor_msgs::JointState  where();

    /**
     * @brief currentPose returns the current pose of the robot
     * @return tf pose
     */
    tf::Pose currentPose();

    /**
     * @brief setSpeeds set all joint, linear, gripper and rotational speeds.
     * @param speed new speed in m/sec
     *
     */
    void setSpeeds(double speed);

    /**
     * @brief slowSpeeds 1/2 current speed
     */
    void slowSpeeds();
    /**
     * @brief mediumSpeeds default speeds, 1 m/sec.
     */
    void mediumSpeeds();
    /**
     * @brief fastSpeeds doubles speeds.
     */
    void fastSpeeds();
    /**
     * @brief setRobotJointSpeeds joint speeds for robot.
     * vel=speed.
     * @return old speed.
     */
    double setRobotJointSpeeds(double speed);
    /**
     * @brief setGripperJointSpeeds joint speeds for gripper.
     * vel=speed.
     * @param speed new speed in m/sec
     * @return old speed.
     */
    double setGripperJointSpeeds(double speed);
    /**
     * @brief setLinearSpeeds set linear Cartesian motion rates
     * for velocity, acceleration and jerk. Default, is
     * vel=speed, acc=accelerationMultipler()*speed,
     * jerk=accelerationMultipler()*10*speed.
     * @param speed new speed in m/sec
     * @return old speed
     */
    double setLinearSpeeds(double speed);
    /**
     * @brief setRotationalSpeeds set angular motion rates
     * for velocity, acceleration and jerk. Default, is
     * vel=speed, acc=accelerationMultipler()*speed,
     * jerk=accelerationMultstd::vector<unsigned long> CController::allJointNumbers() ipler()*10*speed.
     *
     * @param speed new speed in m/sec
     * @return old rotational speed
     */
    double setRotationalSpeeds(double speed);
    /**
     * @brief updateJointRates update joint rates for controller based
     * on sensor msg joint state velocities and efforts.
     * @param jointnums list of joint numbers (0..n)
     * @param newjoints new joint values vel/acc copied
     */
    void updateJointRates(std::vector<uint64_t> jointnums,
                          sensor_msgs::JointState newjoints);
    /**
     * @brief DumpRobotNC gives a string summary of controller
     * @param nc pointer to controller to display summary
     */
    static void dumpRobotNC(std::shared_ptr<CController> nc);

    /**
     * @brief allJointNumbers produce a CRCL list of joint numbers
     * corresponding to joints name size
     * @return
     */

    std::vector<unsigned long> allJointNumbers() ;

    ////////////////////////////////////////////////

    // Robot variables
    VAR(std::string, robotName);   /// robot's name
    VAR(std::string, robotPrefix);  // prefix to append to variables e.g., urdf joints/links
    VAR(std::string, robotEelink);  // name of end effector link in robot
    VAR(std::string, robotBaselink);  // name of base link in robot

    // Robot Status variables
    RCS::CCanonWorldModel _status; /**< current status of controller */
    RCS::CCanonWorldModel _laststatus; /**< last status of controller */
    bool bInited;  /**< bool initialization flag, true means initialized */


    VAR(std::shared_ptr<IRCSInterpreter>, robotInterpreter);
    VAR(boost::shared_ptr<RCS::IKinematic>, robotKinematics);
    VAR(bool, bGrasping); /**< robot currently grasping object */


    // Robot Command  variables
    unsigned long last_crcl_command_num;  /// used to detect new commands
    std::list<RCS::CCanonCmd> donecmds; /**< list of commands interpreted from Crcl messages that have completed*/
    NVAR(NextCC, RCS::CCanonCmd, _nextcc);/**<  current new canon command to interpret*/
    NVAR(LastCC, RCS::CCanonCmd, _lastcc); /**<  last canon command interpreted */

    // Saved named robot goal states
    VAR(SINGLE_ARG(std::map<std::string, std::vector<std::string>>), namedCommand);
    VAR(SINGLE_ARG(std::map<std::string, std::vector<double>>), namedJointMove);
    VAR(SINGLE_ARG(std::map<std::string, tf::Pose>), namedPoseMove);

    VAR(tf::Quaternion, QBend); /**< rotation to achieve pose rotation for grasping */
    VAR(tf::Pose, Retract);         /**< pose offset for retract */
    VAR(tf::Pose, RetractInv); /**< inverse pose offset for retract */
    VAR(tf::Pose, Correction);         /**< correction offset for pose */
    VAR(tf::Pose, CorrectionInv);         /**< inverse correction offset for pose */


    VAR(CGzRobotHandler, writer);  /// gazebo robot and gripper joint update publisher (writer).

    // Canonical Robot Command language variables
    VAR(double, crclPublishStatusRate);
    VAR(RCS_Time, crclLastUpdateTime);
    VAR(std::string, crclIp);
    VAR(int, crclPort);
    VAR(std::string, crclGripperAlgorithm);

#ifdef CRCL_DLL
    VAR(boost::shared_ptr<crcl::crclServer>, pCrclServer );
#endif
#ifdef CRCL_ROS_TOPIC
    VAR(std::shared_ptr<CRosCrclRobotHandler>, pRosCrcl)
#endif


    //  Gripper variables
    VAR(GripperInterface, cncGripper);
    VAR(sensor_msgs::JointState, cncGripperJoints);

    VAR(sensor_msgs::JointState, gripper_goal_joints );
    VAR(std::string, gripperName );

    VAR(std::vector<std::string>, fingerNames );

    typedef std::map<std::string, std::vector<std::string> >  TMapStrVector;

    // Motion rate variables
    VAR(std::vector<double>, linearmax);
    VAR(std::vector<double>, rotationmax);
    VAR(std::vector<double>, base_linearmax);
    VAR(std::vector<double>, base_rotationmax);

    VAR(double, currentLinearSpeed);
    VAR(double, currentAngularSpeed);
    VAR(double, currentRobotJointSpeed );
    VAR(double, currentGripperJointSpeed );

    VAR(double, accelerationMultipler);

    // Gotraj rate variables. Unclear how fine a customization required.
    //joint motion rates - vel, acc, jerk
    VAR(std::vector<gomotion::GoTrajParams>, robotJointRateParams);
    VAR(gomotion::GoTrajParams, linearParams);
    VAR(gomotion::GoTrajParams, rotationalParams);
    VAR(std::vector<gomotion::GoTrajParams>, gripperJointRateParams);

    // Demo specific  - Parts and offsets variables
    VAR(std::vector<std::string>, part_list); /// robot parts from set of all parts
    VAR(SINGLE_ARG(std::map<std::string, tf::Pose>), gripperoffset);       /// gripper offset for each part
    VAR(SINGLE_ARG(std::map<std::string, tf::Pose>), slotoffset);          /// gripper offset for container slot
    VAR(SINGLE_ARG(std::map<std::string, double>), graspforce);            /// grasping force for robot gripper for a part

};

//* The RobotCommands is currently a dummy class. The CController thread
#ifdef ROBOTSTATUS

/**
     * \brief  The RobotStatus is a thread that reads the status of the robot and updates the world model.
     * The RobotStatus is a separate thread that reads the robot status using ROS communication mechanisms
     * and updates the controller world model based on these values.
     * Currently, it uses an instance of the class JointReader to read joint values from the controller.
     * It uses a robotKinematics pointer reference to compute the current robot pose
     * using the forward kinematics (FK) routine.
     * It also uses a CrclDelegate pointer reference to update the status reported by CRCL.
     */
class RobotStatus : public RCS::Thread {
public:

    /*!
         * \brief RobotStatus constructor that requires a cycle time for RCS thread timing.
         * \param cycletime  in seconds.
         */
    RobotStatus(double cycletime = DEFAULT_LOOP_CYCLE);

    VAR(JointReader, std::shared_ptr<CJointReader>);
    VAR(robotKinematics, std::shared_ptr<IKinematics>);

    /*!
         * \brief Action is the main loop in the RCS thread timing.
         * Get latest robot joint readings. Use forward kinematics to get current pose.
         * Then, updates the CRCL world model with the latest readings.
         * \fixme Should it keep track of the command id also - in theory only one CRCl command at a time.
         */
    virtual int Action();

    /*!
         * \brief method to determine if the instance is valid, i.e., has all reference pointers.
         * \return boolean to signify whether component is valid.
         */
    bool Verify() {
        //assert(CrclDelegate() != NULL);
        assert(JointReader() != NULL);
        assert(robotKinematics() != NULL);
    }
};



#endif
}

#endif
