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

#ifndef CROS_H
#define CROS_H


#include <ros/ros.h>
#include <ros/package.h>
#include <ros/console.h>

#include <gzrcs/Controller.h>

/**
 * @brief The CRosCrclRobotHandler class handles topic subscription and advertising for
 * CRCL commands and status.
 */
class CRosCrclRobotHandler
{
public:
    /**
     * @brief CRosCrclRobotHandler constructor.
     */
    CRosCrclRobotHandler();

    /**
     * @brief CmdCallback when a CRCL message arrives it uses this method as a callback
     * to post the message to the CRCL message queue for the controller assigned to this class.
     * @param cmdmsg
     */
    void cmdCallback(const crcl_rosmsgs::CrclCommandMsg::ConstPtr& cmdmsg);

    /**
     * @brief Init establish which controller is connected to the ROS CRCL handler.
     * @param cnc  shared pointer to the controller.
     * @param prefix prefix name of the robot
     */
    void init(std::shared_ptr<RCS::CController> cnc, std::string prefix);

    /**
     * @brief Start subscribes to command  topic messages and advertises status topic messages.
     */
    void start();

    /**
     * @brief Stop stops subscriptioin to command  topic messages and advertising status topic messages.
     */
    void stop();

    /**
     * @brief PublishCrclStatus
     * @param statusmsg
     */
    void publishCrclStatus(crcl_rosmsgs::CrclStatusMsg &statusmsg);

    ///////////////////////////////////////////////////////////
    ros::Publisher _crclStatus; /**< ros publisher information used for crcl status updates */
    ros::Subscriber _crclCmd; /**< ros subscriber information used for crcl command updates */
    std::shared_ptr<RCS::CController> _cnc; /**< ros subscriber information used for crcl command updates */
    std::string _prefix;
};

/**
 * @brief The CRos class handles establishing a connection to ROS. It creates a ROS node that
 * is reusable by all. It is important to establish this ROS connection to the master as all of the ROS
 * routines and data structures rely on rosout to log messages. If rosout doesn't exist, problems.
 */
class CRos
{
public:
    /**
     * @brief CRos handles establishing a connection to ROS
     */
    CRos();

    /**
     * @brief Init initializes a ROS session application.
     *  Waits for ROS master to be running -  look for rosout, because rosmaster is python program.
     * Initialize ROS environment ROS_MASTER_URI MUST BE SET or seg fault.
     * Creates universally usable in this application NodeHandlePtr.
     * Creates a ROS asynchronous spinner required for multithreaded ROS communication.
     */
    void init();

    /**
     * @brief Close call ros::shutdown()
     */
    void close();

    /**
     * @brief addRobotController creates a ROS CRCL command and status topics interfaces.
     * appears as if unused.
     * @param cnc - cnc which is using ROS topic communication
     * @param prefix - prefix of robot name for topic naming
     * @return copy of  CRosCrclRobotHandler
     */
    CRosCrclRobotHandler addRobotController(std::shared_ptr<RCS::CController> cnc, std::string prefix);

    static  ros::NodeHandlePtr nh; /**< node handle for ros app calls */

protected:
    std::vector<CRosCrclRobotHandler > _crclRosCncs; /**< vector of robot controllers */
    static ros::AsyncSpinner * _spinner; /**< aynchronous spinning for ROS communication - not sure required */
};
extern CRos Ros;
#endif // CROS_H
