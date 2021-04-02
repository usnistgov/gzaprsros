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

#include <iostream>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <netinet/in.h>
#include <netdb.h>
#include <unistd.h>
#include <string>
#include <arpa/inet.h>
#include <errno.h>
#include <future>
#include <fcntl.h>

#include <boost/exception/all.hpp>
#include <boost/regex.hpp>

#include "crclclient/CrclSocketServer.h"
#include "crclclient/Crcl2Rcs.h"
#include "crclclient/nistcrcl.h"

bool CBufferHandler::_bTrace;

static unsigned int MyErrorMessage(std::string errmsg) {
    //logFatal(errmsg.c_str());
    return -1;
}

static std::string trim (std::string source, std::string delims = " \t\r\n")
{
    std::string result = source.erase(source.find_last_not_of(delims) + 1);
    return result.erase(0, result.find_first_not_of(delims));
}

////////////////////////////////////////////////////////////////////////////////
///  CBufferHandler
////////////////////////////////////////////////////////////////////////////////

CBufferHandler::CBufferHandler(CCrclSession * pSession)
{
    this->pSession=pSession;
}
CBufferHandler::~CBufferHandler()
{


}

////////////////////////////////////////////////////////////////////////////////
void CBufferHandler::AppendBuffer(std::string read)
{
    if (_next.size() > 0) {
        _current.append(_next);
        _next.clear();
    }
    size_t oldsize = _current.size();
    _current.append(read);
    if (_endtag == NonsenseTag()) {
        _endtag = FindLeadingElement(_current);

        if (_endtag.empty()) {
            _endtag = NonsenseTag();
        }
    }
    BufferHandler(_endtag);
}

////////////////////////////////////////////////////////////////////////////////
void CBufferHandler::SaveMessage(std::string xmlmessage) {
    if (CBufferHandler::_bTrace)
    {
        //        logStatus("===========================================================\n"
        //        "%s:%d: [%s]\n",
        //                RemoteIP().c_str(),
        //                RemotePORT(),
        //                xmlmessage.c_str());
        //        //Globals.AppendFile(Globals.ExeDirectory + "xmltrace.txt", xmlmessage);
    }

    if(CCrclSession::bDebugXML  && xmlmessage.find("GetStatusType") == std::string::npos)
    {
        printf("Raw:%s\n",xmlmessage.c_str());
        (*crcl::crclServer::debugstream) << xmlmessage <<"\n";
    }

    CCrclSession::InMessages().addMsgQueue(boost::make_tuple(xmlmessage, pSession));

    // Theoretically this could cause out of sequence execution...
    //std::async(std::launch::async, &CCrcl2RosMsg::Action,pSession->crcl2ros );

//    std::thread thr2 (std::bind(&CCrcl2RosMsg::Action,pSession->crcl2ros ));
//    _thread.swap(thr2);  // class member _thread is now running the thread created as thr2
    // and thr2 has no thread of execution
    if(pSession->qlistener)
        pSession->qlistener->wake();

}

////////////////////////////////////////////////////////////////////////////////
std::string CBufferHandler::FindLeadingElement(std::string xml)
{
    boost::match_results<std::string::const_iterator> matchResult;
    bool found;
    boost::regex e("<[A-Za-z0-9_]+");
    found = boost::regex_search(xml, matchResult, e);

    if (found) {
        std::string elem(matchResult[0]);
        elem.insert(1, 1, '/');
        elem = trim(elem);
        elem.append(">"); // not space
        return elem;
    }
    return NonsenseTag();
}

////////////////////////////////////////////////////////////////////////////////
void CBufferHandler::BufferHandler(std::string & endtag)
{    int nMaxConnections;
     int nConnections;

      std::size_t found;

       while ((found = _current.find(endtag)) != std::string::npos) {
           found = found + endtag.size();
           _next = _current.substr(found);
           _current = _current.substr(0, found);
           SaveMessage(_current);
           //_inmsgs.AddMsgQueue(boost::make_tuple(_current, this) );
           _current = _next; // MISSING? when messages are pumped out back to back
           _next.clear();
           endtag = FindLeadingElement(_current);
       }
}
////////////////////////////////////////////////////////////////////////////////
///  CCrclSession
////////////////////////////////////////////////////////////////////////////////
int CCrclSession::bDebugXML;

//CMessageQueueThread *  CCrclSession::_inmsgs= NULL;  // queue with thread notify
CMessageQueue *  CCrclSession::_inmsgs= NULL;  // queue with thread notify

////////////////////////////////////////////////////////////////////////////////
CCrclSession::CCrclSession(double cycletime,
                           std::string name,
                           int port,
                           RCS::Thread* qlistener)   :
    RCS::Thread(cycletime, name)
{
    this->qlistener=qlistener;
    on=1;
    nMaxConnections=6;
    nConnections=0;
    sock_fds.clear();
    _remoteport=port;

}



////////////////////////////////////////////////////////////////////////////////
static void set_nonblock(int socket)
{
    int flags;
    flags = fcntl(socket,F_GETFL,0);
    assert(flags != -1);
    fcntl(socket, F_SETFL, flags | O_NONBLOCK);
}
////////////////////////////////////////////////////////////////////////////////
void CCrclSession::init()
{

    /* create socket */
    serverSock=socket(AF_INET, SOCK_STREAM, 0);
    int flags = fcntl(serverSock, F_GETFL, 0);
    fcntl(serverSock, F_SETFL, flags | O_NONBLOCK);

    //allow reuse of port
    int yes=1;
    if (setsockopt(serverSock,SOL_SOCKET,SO_REUSEADDR,&yes,sizeof(int)) == -1) {
        std::cerr << "setsockop failed\n" ;
        close(serverSock);
        return;
    }

    // ioctl setup for non-blocking socket
    if( ioctl(serverSock, FIONBIO, (char *)&on) < 0)
    {
        std::cerr << "ioctl failed\n" ;
        close(serverSock);
        return;
    }

    /* Create sockaddr_in structure for the server socket.
           INADDR_ANY means accept a connection on any of this host's IP addresses*/
    sockaddr_in serverAddr;
    serverAddr.sin_family = AF_INET;
    serverAddr.sin_port = htons(_remoteport); // SERVER_PORT;
    serverAddr.sin_addr.s_addr = INADDR_ANY;

    /* bind (this socket, local address, address length)
           bind server socket (serverSock) to server address (serverAddr).
           Necessary so that server can use a specific port */
    if(bind(serverSock, (struct sockaddr*)&serverAddr, sizeof(struct sockaddr))<0)
    {
        std::cerr << " bind socket failed\n";
        close(serverSock);
        return;
    }

    // wait for a client
    /* listen (this socket, request queue length) */
    if(listen(serverSock, nMaxConnections)<0)
    {
        std::cerr << " listen socket failed\n";
        close(serverSock);
        return;
    }
    // From: https://www.ibm.com/support/knowledgecenter/en/ssw_i5_54/rzab6/xnonblock.htm
    /*************************************************************/
    /* Initialize the master fd_set                              */
    /*************************************************************/
    FD_ZERO(&master_set);
    max_sd = serverSock;
    FD_SET(serverSock, &master_set);

    /*************************************************************/
    /* Initialize the timeval struct to 3 milliseconds.  If no        */
    /* activity after 3 milliseconds skip.                       */
    /*************************************************************/
    timeout.tv_sec  = 0;
    timeout.tv_usec = 3000;

}

////////////////////////////////////////////////////////////////////////////////
int CCrclSession::action()
{

    int new_sd;
    int    desc_ready, end_server = false;
    int    close_conn;
    char   buffer[2048];

    if(sock_fds.size() <nMaxConnections)
    {
        /**********************************************************/
        /* Copy the master fd_set over to the working fd_set.     */
        /**********************************************************/
        memcpy(&working_set, &master_set, sizeof(master_set));

        /**********************************************************/
        /* Call select() and wait 5 minutes for it to complete.   */
        /**********************************************************/
        int rc;
        if( (rc=select(max_sd + 1, &working_set, NULL, NULL, &timeout))<0)
        {
            return 0;
        }

        /**********************************************************/
        /* One or more descriptors are readable.  Need to         */
        /* determine which ones they are.                         */
        /**********************************************************/
        desc_ready = rc;
        for (int i=0; i <= max_sd  &&  desc_ready > 0; ++i)
        {
            /*******************************************************/
            /* Check to see if this descriptor is ready            */
            /*******************************************************/
            if (FD_ISSET(i, &working_set))
            {
                /****************************************************/
                /* A descriptor was found that was readable - one   */
                /* less has to be looked for.  This is being done   */
                /* so that we can stop looking at the working set   */
                /* once we have found all of the descriptors that   */
                /* were ready.                                      */
                /****************************************************/
                desc_ready -= 1;

                /****************************************************/
                /* Check to see if this is the listening socket     */
                /****************************************************/
                if (i == serverSock)
                {
                    printf("  Listening socket is readable\n");
                    /*************************************************/
                    /* Accept all incoming connections that are      */
                    /* queued up on the listening socket before we   */
                    /* loop back and call select again.              */
                    /*************************************************/
                    do
                    {
                        /**********************************************/
                        /* Accept each incoming connection.  If       */
                        /* accept fails with EWOULDBLOCK, then we     */
                        /* have accepted all of them.  Any other      */
                        /* failure on accept will cause us to end the */
                        /* server.                                    */
                        /**********************************************/
                        new_sd = accept(serverSock, NULL, NULL);
                        if (new_sd < 0)
                        {
                            if (errno != EWOULDBLOCK)
                            {
                                perror("  accept() failed");
                                end_server = true;
                            }
                            break;
                        }

                        /**********************************************/
                        /* Add the new incoming connection to the     */
                        /* master read set                            */
                        /**********************************************/
                        printf("  New incoming connection - %d\n", new_sd);
                        FD_SET(new_sd, &master_set);
                        if (new_sd > max_sd)
                            max_sd = new_sd;
                        if(new_sd!=-1)
                        {
                            sock_fds.push_back(new_sd);
                            sock_buffer[new_sd]=
                                    std::shared_ptr<CBufferHandler>(new CBufferHandler(this));
                        }

                        /**********************************************/
                        /* Loop back up and accept another incoming   */
                        /* connection                                 */
                        /**********************************************/
                    } while (new_sd != -1);
                }
            }
        }
        if(sock_fds.size()==0)
            return 1;

        for(size_t j=0; j< sock_fds.size();j++)
        {
            close_conn = false;
            /*************************************************/
            /* Receive all incoming data on this socket      */
            /* before we loop back and call select again.    */
            /*************************************************/
            /**********************************************/
            /* Receive data on this connection until the  */
            /* recv fails with EWOULDBLOCK.  If any other */
            /* failure occurs, we will close the          */
            /* connection.                                */
            /**********************************************/
            rc = recv(sock_fds[j], buffer, sizeof(buffer), 0);
            if (rc < 0)
            {
                if (errno != EWOULDBLOCK)
                {
                    perror("  recv() failed");
                    close_conn = true;
                }
            }

            /**********************************************/
            /* Check to see if the connection has been    */
            /* closed by the client                       */
            /**********************************************/
            if (rc == 0)
            {
                printf("  Connection closed\n");
                close_conn = true;
            }

            /**********************************************/
            /* Data was received                          */
            /**********************************************/
            if(!close_conn)
            {
                int len = rc;
                std::string data = std::string(buffer, buffer + len);
//                if(bDebugXML)
//                {
//                    printf("%s\n",data.c_str());
//                }
                sock_buffer[sock_fds[j]]->AppendBuffer(data);
            }
            else
            {
                CloseConnection(sock_fds[j]);
            }
        }

    }

    if(end_server == true)
        return 0;
    return 1;
}
////////////////////////////////////////////////////////////////////////////////
void CCrclSession::CloseConnection(int i)
{
    printf("Socket closed\n");
    close(i);
    FD_CLR(i, &master_set);
    if (i == max_sd)
    {
        while (FD_ISSET(max_sd, &master_set) == false)
            max_sd -= 1;
    }
    auto it = std::find(sock_fds.begin(), sock_fds.end(), i);
    if(it != sock_fds.end())
        sock_fds.erase(it);
}

void CCrclSession::SyncWrite(std::string str)
{
    /**********************************************/
    /* Echo the data back to the client           */
    /**********************************************/
    for(size_t j=0; j< sock_fds.size();j++)
    {
        int rc = send(sock_fds[j], str.c_str(), str.size(), 0);
        if (rc < 0)
        {
            this->CloseConnection(sock_fds[j]);
        }
    }
}
////////////////////////////////////////////////////////////////////////////////
void CCrclSession::stop (bool bWait)
{
    close(serverSock);
    Thread::stop(bWait);
}

////////////////////////////////////////////////////////////////////////////////
void CCrclSession::cleanup()
{
}
