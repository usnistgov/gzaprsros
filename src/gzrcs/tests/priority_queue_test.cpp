
#include <string>
#include <nist/RCSPriorityQueue.h>
#include <iostream>

/** Expected test output.
Priority Queue
    cmd 1
    cmd 2
    cmd 3
*/

/**
 * @brief The msg struct
 */

struct msg
{
    int cmd_num;
    std::string cmd_name;
    msg(int cmd, std::string name) : cmd_num(cmd), cmd_name(name) {}
};

template<typename T>
struct CompareCmdNum {
    bool operator()(T const & p1, T const & p2) {
        // return "false" if "p1" is ordered before "p2"
        return p1.cmd_num < p2.cmd_num;
    }
};

typedef RCS::CPriorityMessageQueue<msg, CompareCmdNum<msg> >  PQ;


int test_priority_queue()
{
    PQ pq;
    pq.AddMsgQueue(msg(2, "cmd 2"));
    pq.AddMsgQueue(msg(3, "cmd 3"));
    pq.AddMsgQueue(msg(1, "cmd 1"));
    std::cout << "Priority Queue\n";
    // This is how its stored in vector (via push_back)
    for(size_t i=0; i< pq.size(); i++)
    {
        std::cout << "\t" << pq.item(i).cmd_name <<  "\n";

    }
    // THis is in order of the priorities - this is what you want
    for(; pq.size()>0;)
    {
        // This is NO accessor to Nth priority message queue elements
        msg m = pq.peek();
        std::cout << "\t" << m.cmd_name <<  "\n";
        pq.pop();

    }

    return 0;

}


