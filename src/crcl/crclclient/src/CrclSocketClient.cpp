//
// SocketClient.cpp
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

#include <istream>
#include <boost/regex.hpp>
#include <boost/exception/all.hpp>
#include <crclclient/CrclSocketClient.h>
#include <time.h>
#include <sys/ioctl.h>
#include <fcntl.h>



#include "boost/iostreams/stream.hpp"
#include "boost/iostreams/device/null.hpp"
#include <boost/iostreams/stream.hpp>

#ifndef ONCE
#define ONCE(X)  \
    static std::ostream* os;\
{ boost::iostreams::stream< boost::iostreams::null_sink > nullOstream( ( boost::iostreams::null_sink() ) );\
    static int Y##__LINE__=-1;    if(++Y##__LINE__==0) os=&X;  else os=&nullOstream;}\
    *os
#endif


////////////////////////////////////////////////////////////////////////////////
CrclSocketClient::CrclSocketClient() :
    RCS::Thread(.01, "CrclSocketClient")
  ,_crclBufferHandler(std::bind(&CrclSocketClient::SaveMessage,
                               this, std::placeholders::_1 ))

{

    _cycleTime=0.01; // default thread timing
	_bConnected=false;
    _socket=0;
}

void CrclSocketClient::SaveMessage(std::string xmlmessage)
{
    _msgq.addMsgQueue(xmlmessage);

}
////////////////////////////////////////////////////////////////////////////////
void CrclSocketClient::Init(std::string ipv4, int port)
{
	_ipv4=ipv4;
	_port=port;

}

////////////////////////////////////////////////////////////////////////////////
void CrclSocketClient::start()
{
    this->Connect();
    CrclSocketClient::Thread::start();
}

////////////////////////////////////////////////////////////////////////////////
void CrclSocketClient::stop()
{
    this->Disconnect();
    CrclSocketClient::Thread::stop();
}


////////////////////////////////////////////////////////////////////////////////
int CrclSocketClient::Connect()
{
    if(_socket<=0)
        _socket = socket(AF_INET, SOCK_STREAM, 0);
    if (_socket <= 0)
    {
        std::cerr << "\n Socket creation error \n";
        return -1;
    }

    serv_addr.sin_family = AF_INET;
    serv_addr.sin_port = htons(_port);

    // Convert IPv4 and IPv6 addresses from text to binary form
    if(inet_pton(AF_INET, _ipv4.c_str(), &serv_addr.sin_addr)<=0)
    {
        std::cerr << "\nInvalid address/ Address not supported \n";
        return -1;
    }

    if (connect(_socket, (struct sockaddr *)&serv_addr, sizeof(serv_addr)) < 0)
    {
        ONCE(std::cout) << "Socket Connection Failed:" << _ipv4.c_str()<<":"<< _port<<std::endl;
        this->cycleTime()=5.0; // bump to five seconds
        return -1;
    }
    std::lock_guard<std::mutex> guard(_crclmutex);
    this->cycleTime()=_cycleTime; // default thread timing
    _bConnected=true;
    return 0;
}
////////////////////////////////////////////////////////////////////////////////
void CrclSocketClient::Disconnect()
{
    std::lock_guard<std::mutex> guard(_crclmutex);
    _bConnected=false;
    close(_socket);
}
////////////////////////////////////////////////////////////////////////////////
bool  CrclSocketClient::IsConnected()
{
    std::lock_guard<std::mutex> guard(_crclmutex);
    return _bConnected;
}
////////////////////////////////////////////////////////////////////////////////
static void setSocketRecvTimeout(int sockfd, int ms)
{
    struct timeval t;
    t.tv_sec = 0;
    t.tv_usec = ms;
    setsockopt(
                sockfd,     // Socket descriptor
                SOL_SOCKET, // To manipulate options at the sockets API level
                SO_RCVTIMEO,// Specify the receiving or sending timeouts
                &t, // option values
                sizeof(t)
                );
}
////////////////////////////////////////////////////////////////////////////////
static bool setSocketBlocking(int fd, bool blocking)
{

    if (fd < 0) return false;

#ifdef _WIN32
    unsigned long mode = blocking ? 0 : 1;
    return (ioctlsocket(fd, FIONBIO, &mode) == 0) ? true : false;
#else
    int flags = fcntl(fd, F_GETFL, 0);
    if (flags == -1)
        return false;
    flags = blocking ? (flags & ~O_NONBLOCK) : (flags | O_NONBLOCK);
    return (fcntl(fd, F_SETFL, flags) == 0) ? true : false;
#endif
}
////////////////////////////////////////////////////////////////////////////////
void CrclSocketClient::SyncRead(int timeout)
{
    int n;
    // if not connected connect again.
    if(!_bConnected)
        n=Connect();

    // didn't connect againstop, return
    if(n<0)
        return;

    setSocketBlocking(_socket, false);

    int bytesRead;
    char dataReceived[1024];
    dataReceived[0]=0;
    if((bytesRead = read(_socket, dataReceived, sizeof(dataReceived)-1)) != 0)
    {
        if(bytesRead>0)
        {
            _crclBufferHandler.AppendBuffer(std::string(dataReceived, dataReceived+bytesRead));
        }
        else if (bytesRead < 0)
        {
            if (bytesRead == EINTR)
            {
                return;
            }
            else if (errno == EWOULDBLOCK)
            {
                return;
            }
            else
                Disconnect();
        }
    }

}

////////////////////////////////////////////////////////////////////////////////
int CrclSocketClient::action()
{
    SyncRead(50);
    return 1;
}
void CrclSocketClient::SyncWrite(std::string str)
{
    // Write data to server that contains a delimiter.
    send(_socket, str.c_str(), str.size(), 0);
}

