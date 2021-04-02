//
// SocketClient.cpp
// 

#include <istream>
#include <boost/regex.hpp>
#include <boost/exception/all.hpp>
#include <crcllib/CrclSocketClient.h>

//http://youku.io/questions/1088470/how-to-read-from-boost-asio-streambuf


////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

boost::asio::io_service  asio_service;

////////////////////////////////////////////////////////////////////////////////
CrclSocketClient::CrclSocketClient() :
    _io_service(asio_service),
    _socket(asio_service),
    _timer(asio_service)	,
    //    _crclBufferHandler(msgq)
    _crclBufferHandler(std::bind(&CrclSocketClient::SaveMessage,
                                 this, std::placeholders::_1 ))

{
	_bConnected=false;
	_nMSServerConnectRetry=2000;
	_bWaitConnect=false;
}

void CrclSocketClient::SaveMessage(std::string xmlmessage)
{
    _msgq.addMsgQueue(xmlmessage);

}
////////////////////////////////////////////////////////////////////////////////
void CrclSocketClient::Init(std::string ipv4, std::string port)
{
	_ipv4=ipv4;
	_port=port;
}

void CrclSocketClient::start()
{
    this->Connect();
}

void CrclSocketClient::stop()
{
    this->StopConnecting();
}


////////////////////////////////////////////////////////////////////////////////
void CrclSocketClient::HandleConnect(const boost::system::error_code& error)
{
	boost::mutex::scoped_lock lock(m);
	_bWaitConnect=false;
	// The async_connect() function automatically opens the socket at the start
  // of the asynchronous operation. If the socket is closed at this time then
  // the timeout handler must have run first.
  if (!_socket.is_open()) 
	  return;

  if (error==boost::asio::error::connection_refused  )
  {
      std::cerr << "HandleConnect connect refused\n";
      ::sleep(_nMSServerConnectRetry* 0.001);
	  Connect();
	  return;
  }
  else if (error==boost::asio::error::already_started  )
  {
      std::cerr <<"HandleConnect connect already_started\n";
      ::sleep(_nMSServerConnectRetry* 0.001);
	  Connect();
	  return;
  }
  // On error, return early.
  if (error) 
  {
	  std::stringstream s;
	  s << "HandleConnect unknown error = (" << error << ") " << error.message()  << std::endl;
      std::cerr <<s.str();
	  return;
  }

  _bConnected=true;
  StartAyncRead();
}

////////////////////////////////////////////////////////////////////////////////
void CrclSocketClient::StopConnecting()
{
	// not sure if this works
	_bConnected=false;
	_socket.cancel();
}
////////////////////////////////////////////////////////////////////////////////
void CrclSocketClient::Connect()
{
	tcp::resolver resolver(_io_service);
	tcp::resolver::query query(_ipv4.c_str(), _port.c_str());
	tcp::resolver::iterator endpoint_iterator = resolver.resolve(query);
	//boost::asio::connect(_socket, endpoint_iterator);
	_bWaitConnect=true;
    _socket.async_connect(*endpoint_iterator, boost::bind(&CrclSocketClient::HandleConnect,
		this,
		boost::asio::placeholders::error ));

	
}
////////////////////////////////////////////////////////////////////////////////
void CrclSocketClient::Disconnect()
{
	_bConnected=false;
	_timer.cancel();
	_socket.close();
}
////////////////////////////////////////////////////////////////////////////////
bool  CrclSocketClient::IsConnected()
{
	// Does not mean that server is listening.
	//return _socket.is_open();  // socket created
	return _bConnected;
}

////////////////////////////////////////////////////////////////////////////////
std::string CrclSocketClient::makeString(boost::asio::streambuf& streambuf, std::size_t n)
{
 return std::string(buffers_begin(streambuf.data()),
         buffers_begin(streambuf.data()) + n);
}


////////////////////////////////////////////////////////////////////////////////
void wait_callback(const boost::system::error_code& error, boost::asio::ip::tcp::socket& _socket)
{
	if (error==boost::asio::error::operation_aborted)
	{
		// Data was read and this timeout was canceled
		return;
	}
	else if (error)
	{
		std::cout  << "read_timeout Error - " << error.message() << std::endl;
		// Data was read and this timeout was canceled
		return;
	}
	_socket.cancel();  // will cause read_callback to fire with an timeout error
}
////////////////////////////////////////////////////////////////////////////////
void CrclSocketClient::TimerReset()
{
	// Setup a deadline time to implement our timeout.
	_timer.expires_from_now(boost::posix_time::milliseconds(2000));
	_timer.async_wait(boost::bind(&wait_callback,
		boost::asio::placeholders::error, boost::ref(_socket)));
}
////////////////////////////////////////////////////////////////////////////////
void CrclSocketClient::StartAyncRead()
{
	try {
		if(!_socket.is_open())
		{
            std::cerr << "Error: StartAyncRead when socket NOT OPENr \n";
            return;
		}
		boost::asio::async_read(_socket,
			boost::asio::buffer(data_, max_length), // buff, //boost::asio::buffer(&readBuffer[0], readBuffer.size()),
            boost::bind(&CrclSocketClient::bytesToRead,
			this,
			boost::asio::placeholders::error, 
			boost::asio::placeholders::bytes_transferred));
		
		TimerReset();
	}
	catch(boost::exception & ex)
	{
		//http://theboostcpplibraries.com/boost.exception
		std::cerr << boost::diagnostic_information(ex);
		_socket.close();
	}	
	catch(...)
	{
		_socket.close();
	}
}

size_t CrclSocketClient::bytesToRead(const error_code& error, size_t bytes_read)
{
    size_t result=bytes_read;
    _timer.cancel();

    // Connection closed cleanly by peer
    // asio errors http://www.boost.org/doc/libs/1_44_0/doc/html/boost_asio/reference/error__basic_errors.html
    if (error == boost::asio::error::eof || boost::asio::error::connection_reset == error)
    {
        Disconnect();
        return  0;
    }
    else if(error == boost::asio::error::operation_aborted  )
    {
        std::cerr << " In bytesToRead Timer expired error \n";
    }
    else if (error)
    {
        // This error stops asynchronous reads
        std::stringstream s;
        s << "unknown bytesToRead error = (" << error << ") " << error.message()  << std::endl;
        std::cerr << s.str();
        return  0;
    }

    if(bytes_read>0)
    {
        _crclBufferHandler.AppendBuffer(std::string(data_, data_+bytes_read));
    }
    StartAyncRead();
    return result;
}


void CrclSocketClient::SyncWrite(std::string str)
{
  // Write data to server that contains a delimiter.
  _socket.send(boost::asio::buffer(str, str.size()));
}

