#ifndef CRCLBUFFERHANDLER_H
#define CRCLBUFFERHANDLER_H

#include "aprs_headers/RCSMsgQueue.h"

typedef std::function<void (std::string)> TCallback;

class CCrclBufferHandler
{
public:

   //RCS::CMessageQueue<std::string> &_msgq;
    TCallback savefcn ;

    /**
     * @brief CBufferHandler constructor that takes a pointer to the parent session.
     */
    //CCrclBufferHandler(RCS::CMessageQueue<std::string> &);
    CCrclBufferHandler(TCallback );
    /**
      * @brief CBufferHandler destructor.
     */
    ~CCrclBufferHandler();

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
    static std::string	NonsenseTag() { return "<XXXXX>"; }

    std::string _current; /**<  current string read from socket */
    std::string _next; /**<  leftover string after pulling out Crcl XML message */
    std::string _endtag; /**<  endtag to designate the end of Crcl XML message, found from beginning */
    static bool _bTrace;

};
#endif // CRCLBUFFERHANDLER_H
