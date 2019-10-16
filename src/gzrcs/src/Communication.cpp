
#include <sstream>

#include "nist_robotsnc/Communication.h"
#include "nist_robotsnc/Globals.h"

#include "nist/Core.h"
#include "nist/IRcs.h"
#include "nist/Debug.h"

/////////////////////////////////////////////////////////////////////////////
std::mutex CJointWriter::_writer_mutex;

/////////////////////////////////////////////////////////////////////////////
void CJointWriter::Init(std::ofstream *logFile)
{
    mLogFile=logFile;
}
/////////////////////////////////////////////////////////////////////////////
void CJointWriter::Start(std::vector<std::string> names)
{
    bRunning=true;
    if(mLogFile==NULL)
        return;
    *mLogFile << names[0];
    for(size_t i=1; i< names.size(); i++)
    {
        *mLogFile << ",";
        *mLogFile << names[i];
    }
    *mLogFile << "\n" << std::flush;

}
/////////////////////////////////////////////////////////////////////////////
void CJointWriter::Stop() {
    bRunning=false;
}
/////////////////////////////////////////////////////////////////////////////
bool CJointWriter::Update(double time, sensor_msgs::JointState joints)
{
    if(mLogFile==NULL)
        return false;
    if(mJoints.position==joints.position)
        return false;
    mJoints.position=joints.position;

    *mLogFile << time;

    for(size_t i=0; i<  mJoints.position.size(); i++)
    {
        *mLogFile << ",";
        *mLogFile << mJoints.position[i];
    }

    *mLogFile << "\n" << std::flush;
    return true;
}


