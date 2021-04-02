#ifndef RCSPRIORITYQUEUE_H
#define RCSPRIORITYQUEUE_H

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

#include <queue>
#include <deque>
#include <mutex>

#define SCOPED_LOCK    std::lock_guard<std::mutex> guard(m)

namespace msgq = std;


namespace RCS
{

//struct CompareAge {
//    bool operator()(T const & p1, T const & p2) {
//        // return "true" if "p1" is ordered before "p2", for example:
//        return p1.age < p2.age;
//    }
//};
//Using greater gives the opposite ordering to the default less, meaning that the queue will give you the lowest value rather than the highest.
//shareimprove this answer


template<typename T, typename C>
class CPriorityQueue
{
public:
    std::priority_queue<T, std::deque<T>, C>  message_queue;
    typedef typename std::priority_queue<T, std::deque<T>, C>::iterator   message_queue_iterator;

    CMessageQueue( ) { }

    /// \brief ClearMsgQueue clears all contents in message queue. T
    void ClearMsgQueue ( )
    {
        SCOPED_LOCK;
        message_queue.clear( );
    }
    /// \brief SizeMsgQueue returns number of items in message queue.

    size_t SizeMsgQueue ( )
    {
        SCOPED_LOCK;
        return message_queue.size( );
    }

    /*!
     * \brief PopFrontMsgQueue mutex pop of front item of message queue.
     * \return  T       returns front item from message queue.
     */
    T PopFrontMsgQueue ( )
    {
        SCOPED_LOCK;

        if ( !message_queue.size( ) )
        {
            throw std::runtime_error("Empty queue\n");
        }
        T msg = message_queue.front( );
        message_queue.pop_front( );
        return msg;
    }
    /*!
     * \brief PeekFrontMsgQueue mutex peeks at front item of message queue.
     * \return  T       returns front item from message queue.
     */
    T PeekFrontMsgQueue ( )
    {
        SCOPED_LOCK;

        if ( !message_queue.size( ) )
        {
            throw std::runtime_error("Empty queue\n");
        }
        T msg = message_queue.front( );
        return msg;
    }
    /*!
     * \brief AddMsgQueue mutex push to back an item onto message queue.
     * \param  T       item to place in back of message queue.
     */
    void AddMsgQueue (T t)
    {
        SCOPED_LOCK;
        message_queue.push_back(t);
    }

    /*!
     * \brief AddMsgQueue mutex push to back an item onto message queue.
     * \param  T       item to place in back of message queue.
     */
    void AddBackMsgQueue (T t)
    {
        SCOPED_LOCK;
        message_queue.push_back(t);
    }

    /*!
     * \brief AddMsgQueue mutex push to front an item onto message queue.
     * \param  T       item to place in front of message queue.
     */
    void AddFrontMsgQueue (T t)
    {
        SCOPED_LOCK;
        message_queue.insert(message_queue.begin( ), t);
    }

    /*!
     * \brief InsertFrontMsgQueue mutex push to front an item onto message queue.
     * \param  T       item to place in front of message queue.
     */
    void InsertFrontMsgQueue (T t)
    {
        SCOPED_LOCK;

        message_queue.insert(message_queue.begin( ), t);
    }

    message_queue_iterator end ( ) { return message_queue.end( ); }

    message_queue_iterator begin ( ) { return message_queue.begin( ); }
protected:
    msgq::mutex m;
}
#endif // RCSPRIORITYQUEUE_H
