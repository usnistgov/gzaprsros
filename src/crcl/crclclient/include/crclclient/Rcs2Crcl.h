#ifndef RCS2CRCL_H
#define RCS2CRCL_H

#include <tuple>
#include <vector>
#include <map>

#include "aprs_headers/seriallinkrobot.h"
#include "aprs_headers/RCSThreadTemplate.h"
#include "aprs_headers/RCSMsgQueue.h"
#include "aprs_headers/IRcs.h"

#include <boost/thread/mutex.hpp>

#include "crclclient/CrclClientInterface.h"
#include "crclclient/CrclSocketClient.h"


class CRcs2Crcl :
        public RCS::Thread
{
public:
    CRcs2Crcl();

    int addcommand(crcl_rosmsgs::CrclCommandMsg );

    /* @brief Cyclic loop for the controller. Reads Crcl input mexsage queue, interprets into canon cmds if any, reads canon
    * cmds queue, interprets into robot command messages.
    * @return  1 sucess,  0 problem
    */
    virtual int action();

    /**
     * @brief init initialize the conversion and communication process
     * @param crclIp  - crcl server ip
     * @param crclport - crcl server port
     * @param d_cycle_time - my cycle time
     * @param robotkin - kinematics
     */
    virtual void init(std::string crclIp,
                      int crclport,
                      double d_cycle_time,
                      RCS::ISerialLinkRobot robotkin);

    virtual void start ( );
    virtual void stop ( );

    virtual int cmd_status(int cmdnum) ;
    virtual RCS::CCanonWorldModel  status();

    // \brief robot kinematics not sure even used.
    RCS::ISerialLinkRobot _robotkin;

    // \brief socket session & CRCL XML framing handler
    std::shared_ptr<CrclSocketClient> session;

    size_t QSize() { return cmdq.sizeMsgQueue(); }

   int  getStatus(RCS::CCanonWorldModel&);

private:
    std::mutex _crclmutex;

    std::string crclIp;
    int crclport;
    double cycletime;                                 /**< cycletime of thread in seconds */

    // \brief RCS/ROS cmd message queue
    //RCS::CMessageQueue<std::tuple<int,std::string>> cmdq;
    RCS::CMessageQueue<std::tuple<RCS::CCanonCmd,std::string>> cmdq;
    RCS::CCanonCmd lastcmd;
    // \brief RCS/ROS message queue
    std::map<int, int> rcsstatus;

    // \brief client crcl xml string generator
    boost::shared_ptr<Crcl::CrclClientCmdInterface> crclinterface;

    // \brief unit selections should never change as we are the client
    int  _angleUnit ;
    double _angleConversion;
    int _lengthUnit ;
    double _lengthConversion;
    int _commandnum;


    // \brief class to decode crcl status message
    Crcl::CrclStatusMsgInterface _status_handler;

    // \brief world model status used by crcl
    Crcl::CrclStatus _status;

    // \brief wm used by rcs
    RCS::CCanonWorldModel _wm;


};

#endif // RCS2CRCL_H
