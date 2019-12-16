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

#ifndef COMMUNICATION_H
#define COMMUNICATION_H

#include <mutex>

#include <sensor_msgs/JointState.h>
#include <tf/tf.h>


/**
 * \brief  The CJointWriter is a thread to  publish new joint values to a log file.
 * First and header with all joint names, possibly including gripper joints if provided are
 * published as a csv header.
 * Then, when joint updates occur, new values are save as cvs line in the file.
 */
class CJointWriter {
public:
    /*!
     * \brief CJointWriter constructor .
     */
    CJointWriter() {}
    virtual void Init(std::ofstream *logFile);

    virtual void Start(std::vector<std::string> names);
    virtual void Stop();

    /*!
     * \brief Update writes the  joint_state topic values to listener.
     * \param a sensor_msgs::JointState describing robot joints
     * \return boolean if write occurred as expected.
     */
    virtual bool Update(double time, sensor_msgs::JointState joint);
     ////////////////////////////////
    static std::mutex _writer_mutex; /**< for mutexed writing access to joint values */
    bool bRunning;
    sensor_msgs::JointState mJoints;
    std::ofstream *mLogFile;
};


#endif // COMMUNICATION_H
