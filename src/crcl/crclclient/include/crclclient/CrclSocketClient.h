//
// SocketClient.h
//

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
#include <deque>
#include <mutex>

#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <boost/enable_shared_from_this.hpp>
#include <boost/thread/mutex.hpp>
#include <crclclient/CrclBufferHandler.h>
#include "aprs_headers/RCSThreadTemplate.h"
#include "aprs_headers/RCSMsgQueue.h"

using boost::asio::ip::tcp;
typedef boost::system::error_code error_code;
typedef std::deque<std::string> xml_message_queue;




class CrclSocketClient  : public RCS::Thread
{
    RCS::CMessageQueue<std::string> _msgq;
public:
    void SaveMessage(std::string xmlmessage);

    CrclSocketClient();
    RCS::CMessageQueue<std::string>  & MessageQueue()  { return _msgq; }
    virtual void        start();
    virtual void        stop();
    int 				Connect();
	void				Disconnect();
    void				Init(std::string ipv4, int port);
	bool				IsConnected();	
	void				StartAyncRead();
	void				TimerReset();
	void				SyncWrite(std::string str);
    void				SyncRead(int timeout);
    virtual int         action();
protected:
    CCrclBufferHandler _crclBufferHandler;
    std::mutex _crclmutex;
    struct sockaddr_in serv_addr;
    int _socket;

	std::string _ipv4;
    int _port;
    double _cycleTime;
	bool _bConnected;
};
