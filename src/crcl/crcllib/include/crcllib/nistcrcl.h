#ifndef NISTCRCL_H
#define NISTCRCL_H

// C++ headers
#include <string>
#include <memory>

// crcl message headers
#include "crcl_rosmsgs/CrclCommandMsg.h"
#include "crcl_rosmsgs/CrclStatusMsg.h"


// Nist headers
#include <aprs_headers/IRcs.h>
#include <aprs_headers/RCSThreadTemplate.h>
#include <aprs_headers/RCSMsgQueue.h>

#include <mutex>

class CCrcl2RosMsg;
class CCrclSession;
namespace crcl
{
struct crclServer //: public RCS::Thread
{
    crclServer(std::string crclIp,
                int crclport,
                double d_cycle_time,
                std::string robot_urdf,
                std::string base_link,
                std::string tip_link);

    virtual void statusUpdate(const crcl_rosmsgs::CrclStatusMsg::ConstPtr& statusmsg);
    virtual void setCmdQueue(RCS::CrclMessageQueue *crclcmdsq);
    virtual void start ( );
    virtual void stop ( );
    static std::ofstream *debugstream;
    virtual void setDebugStream(std::ofstream *dstream)
    {
        debugstream=dstream;
    }

    static bool bDebugStatusMsg;
    static bool bDebugCommandMsg;
    static bool bCrclStopIgnore;
    static bool bFlywheel;
    static bool bProcessAllCrclMessages;
    static std::string sRobot;

protected:

    /**
     * @brief crcl2rcs reads new Crcl XML message, interprets them, and translates into RCS.
     * Updates the message queue of the CRCL controller.
     * @return
     */
    std::shared_ptr<CCrcl2RosMsg> crcl2ros;
    std::shared_ptr<CCrclSession> crclCommServer;
    std::string crclIp;
    int crclport;
    std::string urdf_xml;
    std::string base_link;
    std::string tip_link;
    double _cycletime;                                 /**< cycletime of thread in seconds */
};
}
#endif // NISTCRCL_H
