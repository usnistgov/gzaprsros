// CrclInterface.h

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

#pragma once
#include <string>
#include "crcl.h"
#include <iostream>

/**
 * For crcl example xml files visit:
 * https://github.com/ros-industrial/crcl/blob/master/instances
 */

#include <string>
#include <vector>

/**
 * This file relies on numerous conversion routines to translate CRCL representation into ROS representation.
 * It includes translating XML command types into ROS message enumerations.
 * Most all of the different CRCL XML possibilities are grouped into one message union that uses
 * enumerations and intuition (!) to understand what union members are required for which enumerated
 * commands.
 * Included in this file are API to map CRCL commands into ROS message.
 */

namespace Crcl {


    /**
     * \brief CrclClientCmdInterface generates Crcl XML command message from 
     * to send to a Crcl server.
     */
    class CrclClientCmdInterface {
    public:

        CrclClientCmdInterface() {
        }

        void SetCommandNum(unsigned long long n) {
            _commandnum = n;
        }

        // Command interface

        std::string ActuateJoints (ActuateJointsType::ActuateJoint_sequence & joints);
        // std::string ActuateJoint (robotAxes joints, int jointnum);
        std::string CloseToolChanger();
        std::string Dwell(double seconds);
        std::string EndCanon(int reason);
        std::string GetStatus();
        std::string InitCanon();
        std::string Message(std::string message);

        std::string MoveScrew(Crcl::PoseType startPost, VectorType axisPoint, double dAxialDistanceFree, double dAxialDistanceScrew, double dTurn);
        std::string MoveTo(Crcl::PoseType pose, bool bStraight = true);
        std::string MoveThroughTo(Crcl::PoseType *poses,
                int numPoses,
                double * accelerations = NULL,
                double * speeds = NULL,
                Crcl::PoseToleranceType * tolerances = NULL);
        std::string OpenToolChanger();
        std::string RunProgram(std::string programText);
        std::string SetEndEffector(double fractionalSetting);
        std::string SetEndPoseTolerance(Crcl::PoseToleranceType toleranceSetting);
        std::string SetEndEffectorTolerance(Crcl::PoseToleranceType toleranceSetting);
        std::string SetAngleUnits(std::string UnitName); // UnitName is a string that can be only the literals 'angle' or 'radian'.
        std::string SetLengthUnits(std::string UnitName); // UnitName is a string that can be only the literals 'meter',  'millimeter', or 'inch'.
        std::string SetMotionCoordination(bool bCoordinated);
        std::string SetRotAccel(double dAccel);
        std::string SetRotSpeed(double dSpeed);
        std::string StopMotion(int condition); // 0=Immediate, Fast, Normal

        std::string GetStatusReply(CrclStatus *wm);
    private:
        static unsigned long long _commandnum;
    };

    /**
     * \brief CrclStatusMsgInterface parses a Crcl XML status message from 
     * a Crcl server.
      */
    class CrclStatusMsgInterface {
    public:

        CrclStatusMsgInterface() {
        }
        CrclReturn ParseCRCLStatus(std::string filename);
        CrclReturn ParseCRCLStatusString(std::string str);
        CrclStatus status() { return _status; }
    private:
        CrclStatus _status;
    };
};
