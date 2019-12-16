
#ifndef _CRCL2RCS_H
#define _CRCL2RCS_H

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

// C++ includes
#include <list>

// boost includes
#include <boost/shared_ptr.hpp>
#include "boost/iostreams/stream.hpp"
#include "boost/iostreams/device/null.hpp"

// eigen math library includes
#ifdef EIGEN
#include "Eigen/Core"
#include "Eigen/Geometry"
#endif

#include "aprs_headers/RCSThreadTemplate.h"
#include "aprs_headers/RCSMsgQueueThread.h"
#include "aprs_headers/IRcs.h"

#include "crcl_rosmsgs/CrclCommandMsg.h"
#include "crcl_rosmsgs/CrclStatusMsg.h"


#include "crcllib/crclserver.h"
#include "crcllib/CrclServerInterface.h"
#include "crcllib/CrclClientInterface.h"
#include "crcllib/crcl.h"



namespace RCS
{
extern  CMessageQueue<RCS::CCanonCmd> cmds;
extern CCanonWorldModel wm;  // motion related parameters max vel,acc
// not sure why this cant be part of crclwm

};


/**
 * @brief The CCrcl2RosMsg class  handles the command/status interface to ROS.
 * For commands, CCrcl2RosMsg publishes an  CrclCommandMsg
 */
class CCrcl2RosMsg
#ifdef QTHREAD

        : public CMessageQueue  // CMessageQueueThread
#else
        : public RCS::Thread
#endif
{

public:
#ifndef QTHREAD
    CMessageQueue msgq;
#endif

    /**
     * @brief CCrcl2RosMsg handles translation from crcl representation into ros/tf representation
     * @param xml_string - urdf xml string to parse for joint name information
     * @param base_link -  base link of robot for joint name parsing
     * @param tip_link -  tip link of the robot - not including end effector
     */
    CCrcl2RosMsg(std::string xmlString, std::string baseLink, std::string tipLink);


    /**
    * @brief Cyclic loop for the controller. Reads Crcl input mexsage queue, interprets into canon cmds if any, reads canon
    * cmds queue, interprets into robot command messages.
    * @return  1 sucess,  0 problem
    */
    virtual int action();

    /**
     * @brief Initialization routine for the controller..
     */
    virtual void init();

    /**
     * @brief StatusUpdate accepts rcs/ros status message and translates into CRCL status.
     * @param statusmsg ros message of status
     */
    void statusUpdate(const crcl_rosmsgs::CrclStatusMsg::ConstPtr& statusmsg);

    /**
     * @brief setCmdQueue set CRCL command queue
     * @param crclcmdsq
     */
    void setCmdQueue(RCS::CrclMessageQueue *crclcmdsq)
    {
        boost::mutex::scoped_lock lock(cncmutex);
        this->crclcmdsq=crclcmdsq;
    }



    /**
     * @brief ParseURDF accepts an xml urdf string and parses out the joint information.
     * Really just for joint names since CRCL uses numbers.
     * @param xml_string urdf xml
     * @param base_link  base link of the robot
     * @param tip_link tip link of concern to the robot
     * @return true if successful
     */
    bool parseURDF(std::string xmlString, std::string baseLink, std::string tipLink);

    ////////////////////////////////////////////
    static boost::mutex cncmutex; /**< mutex for thread safe access to RobotProgram commands  */

    std::mutex _crclmutex;

    RCS::CrclMessageQueue *crclcmdsq;

    boost::shared_ptr<Crcl::CrclServerDelegateInterface> crclinterface;
    std::vector<std::string> jointNames;
    std::string xmlString;
    std::string baseLink;
    std::string tipLink;
    // URDF Derived knowledge
    std::vector<std::string> linkNames;
    std::vector< double> jointvalues;
    std::vector< double> jointMin;
    std::vector< double> jointMax;
    std::vector< bool> jointHasLimits;
    std::vector< double> jointEffort;
    std::vector< double> jointVelmax;
    std::vector<tf::Vector3> axis;
    std::vector<tf::Vector3> xyzorigin;
    std::vector<tf::Vector3> rpyorigin;
    std::string  robotName;

};
#endif
