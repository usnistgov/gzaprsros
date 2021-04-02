#ifndef CRCLSERVER_H
#define CRCLSERVER_H

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


// crclsocketserver.h

// ws header files
#include "aprs_headers/RCSMsgQueue.h"
#include "aprs_headers/RCSMsgQueueThread.h"
#include "aprs_headers/Core.h"
#include <aprs_headers/RCSThreadTemplate.h>

class CCrclSession;
class CCrcl2RosMsg;

typedef boost::tuple<std::string, CCrclSession *> CrclMessage;
typedef RCS::CMessageQueue<CrclMessage> CCrclMessages;
typedef RCS::CMessageQueue<CrclMessage> CMessageQueue;

class CBufferHandler
{
public:

    /**
     * @brief CBufferHandler constructor that takes a pointer to the parent session.
     */
    CBufferHandler(CCrclSession *);

    /**
      * @brief CBufferHandler destructor.
     */
    ~CBufferHandler();

    /**
     * @brief Session return pointer to session parent. not a smart pointer
     * as session should never be destroyed, as there really is only one session.
     * @return  pointer to CCrclSession, not a smart pointer.
     */
    CCrclSession * Session() { return pSession; }
    /*!
     * \brief Appends a socket buffer. Add previous buffer is exists.
     * \param read buffer of characters
     */
    void AppendBuffer(std::string read);

    /*!
     ** \brief Looks for matching end xml tag. If found, saves message into queue,
     * and restarts read process.
     * \param endtag is the ending tag, e.g., </ENDTAG to match against.  Includes backslash.
     */
    void BufferHandler(std::string & endtag);

    /*!
     * \brief Queues message onto message queue.
     * \param xmlmessage to queue onto this session message queue.
     */
    void SaveMessage(std::string xmlmessage);

    /*!
     * \brief Finds the leading XML tag to create matching end tag.
     * If none, return nonsense tag. Uses boost regex.
     * \oaram xml is the text to search for starting tag
     * \return end tag or nonsense tag if none. e.g., </TAG>
     */
    std::string FindLeadingElement(std::string xml);

    /*!
     *  \brief NonsenseTag to be used as dummy ending xml to test against.
     */
    static std::string NonsenseTag() {
        return "<XXXXX>";
    }
    CCrclSession * pSession;
    std::string _current; /**<  current string read from socket */
    std::string _next; /**<  leftover string after pulling out Crcl XML message */
    std::string _endtag; /**<  endtag to designate the end of Crcl XML message, found from beginning */
    static bool _bTrace;

};

class CCrclSession : public RCS::Thread
{
public:

    /**
     * @brief CCrclSession Constructor for each listener on the socket.
     * @param cycletime cycle time of thread in seconds
     * @param name of the thread
     */
    CCrclSession(double cycletime,
                 std::string name,
                 int port,
                 RCS::Thread*);

    std::string RemoteIP (){ return _remoteip; }
    unsigned short RemotePort (){ return _remoteport; }

    /**
     * @brief init handles socket setup.
     * create socket, allow reuse of port
     * ioctl setup for non-blocking socket
     * Create sockaddr_in structure for the server socket.
     * Using INADDR_ANY which means accept a connection on any of this host's IP addresses.
     * bind server socket (serverSock) to server address (serverAddr).
     * Necessary so that server can use a specific port
     * wait for a client by listening on socket.
     * Initialize the timeval struct to 3 milliseconds.
     * If no activity after 3 milliseconds skip
     */
    virtual void init();

    /**
     * @brief action handles checking for new connections, and reading data from all connections.
     * If no new data on connections, returns, otherwise calls BufferHandler to manage new data.
     * Copy the master fd_set over to the working fd_set.
     * Call select() and wait 5 minutes for it to complete.
     * If one or more descriptors are readable then need to determine which ones they are.
     * Check to see if this descriptor is ready
     * Check to see if this is the listening socket.
     * Accept all incoming connections that are queued up on the listening socket before we
     * loop back and call select again.
     * Receive all incoming data on this socket  before we loop back and call select again.
     * Receive data on this connection until the recv fails with EWOULDBLOCK.  If any other
     * failure occurs, we will close the    connection.
     *
     * @return 1 continue processing thread, 0 stop thread.
     */
    virtual int action();
    virtual void stop (bool bWait = false);
    void CloseConnection(int sock);
    void SyncWrite(std::string str);
    virtual void cleanup();

#ifdef QTHREAD
    static void AssignMessageQueueThread(CMessageQueueThread * msgq)
    {
        _inmsgs=msgq;
    }
    static CMessageQueueThread & InMessages()
    {
        assert(_inmsgs!=NULL);
        return *_inmsgs;
    }
    static CMessageQueueThread * _inmsgs; /**<  queue of inbound crcl xml messages from device */
#else

    static void AssignMessageQueue(CMessageQueue * msgq)
    {
        _inmsgs=msgq;
    }
    static CMessageQueue & InMessages()
    {
        assert(_inmsgs!=NULL);
        // should be copied?
        return *_inmsgs;
    }
    static CMessageQueue * _inmsgs; /**<  queue of inbound crcl xml messages from device */
#endif
//    static CCrclMessages _outmsgs; /**<  queue of outbound crcl xml messages to device */

    RCS::Thread * qlistener;
    static int bDebugXML;
protected:
    int nMaxConnections;
    int nConnections;
    int on;
    int serverSock;
    int max_sd;
    std::vector<int> sock_fds;
    std::map<int, std::shared_ptr<CBufferHandler> > sock_buffer;
    char receivedStr[1000];
    struct timeval       timeout;
    fd_set        master_set, working_set;

    std::string _remoteip;
    unsigned short _remoteport;
};


#endif // CRCLSERVER_H
