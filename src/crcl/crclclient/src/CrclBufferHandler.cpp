#include "crclclient/CrclBufferHandler.h"
#include <boost/regex.hpp>

bool CCrclBufferHandler::_bTrace;

static std::string trim (std::string source, std::string delims = " \t\r\n")
{
    std::string result = source.erase(source.find_last_not_of(delims) + 1);
    return result.erase(0, result.find_first_not_of(delims));
}

////////////////////////////////////////////////////////////////////////////////
///  CBufferHandler
////////////////////////////////////////////////////////////////////////////////

//CCrclBufferHandler::CCrclBufferHandler(RCS::CMessageQueue<std::string> &msgq) : _msgq(msgq)

CCrclBufferHandler::CCrclBufferHandler(TCallback fcn) : savefcn(fcn)

{
}
CCrclBufferHandler::~CCrclBufferHandler()
{


}

////////////////////////////////////////////////////////////////////////////////
void CCrclBufferHandler::AppendBuffer(std::string read)
{
    if (_next.size() > 0) {
        _current.append(_next);
        _next.clear();
    }
    size_t oldsize = _current.size();
    _current.append(read);
    if (_endtag == NonsenseTag() || _endtag.empty()) {
        _endtag = FindLeadingElement(_current);

        if (_endtag.empty()) {
            _endtag = NonsenseTag();
        }
    }
    BufferHandler(_endtag);
}

////////////////////////////////////////////////////////////////////////////////
void CCrclBufferHandler::SaveMessage(std::string xmlmessage) {
    if (CCrclBufferHandler::_bTrace)
    {
        //        logStatus("===========================================================\n"
        //        "%s:%d: [%s]\n",
        //                RemoteIP().c_str(),
        //                RemotePORT(),
        //                xmlmessage.c_str());
        //        //Globals.AppendFile(Globals.ExeDirectory + "xmltrace.txt", xmlmessage);
    }

//    if(CCrclSession::bDebugXML  && xmlmessage.find("GetStatusType") == std::string::npos)
//    {
//        printf("Raw:%s\n",xmlmessage.c_str());
//        (*crcl::crclServer::debugstream) << xmlmessage <<"\n";
//    }

   // _msgq.addMsgQueue(xmlmessage);

}

////////////////////////////////////////////////////////////////////////////////
std::string CCrclBufferHandler::FindLeadingElement(std::string xml)
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
void CCrclBufferHandler::BufferHandler(std::string & endtag)
{    int nMaxConnections;
     int nConnections;

      std::size_t found;

       while ((found = _current.find(endtag)) != std::string::npos) {
           found = found + endtag.size();
           _next = _current.substr(found);
           _current = _current.substr(0, found);
           //SaveMessage(_current);
           savefcn(_current);
           //_inmsgs.AddMsgQueue(boost::make_tuple(_current, this) );
           _current = _next; // MISSING? when messages are pumped out back to back
           _next.clear();
           endtag = FindLeadingElement(_current);
       }
}

