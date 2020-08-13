#ifndef LOGGING_H
#define LOGGING_H

#include <ostream>
#include <fstream>
#include <string>
#include <algorithm>

/**
 * @brief The Logger struct handles logging to either null, stdout as well as
 * to a file. If a file is given and verbose mode, the diagnostic output is
 * saved to the file as well as output to stdout.
\code{.cpp}
int main(int argc, char *argv[])
{
    std::string exepath(argv[0]);
    exepath = exepath.substr(0,exepath.find_last_of('/')+1);

    Logger log("");
    log.verbose()=1;
    log.color("CYAN");
    log << " spare me the details\n";

    log.open(exepath+"test.log");
    for(size_t i=0; i< 100; i++)
    {
        log.ONCE() << i << " This is a test\n" << std::flush;
        log.ONCE() << i << " This is a second test\n" << std::flush;
        if(i> 50)
            log.verbose(0);
        log.THROTTLE(4) << i << " This is a throttle test every fourth time\n" << std::flush;
        log.DBG() << "Hello world\n";

    }

    return 1;
}
\endcode
 */
struct Logger : public std::ostream
{
    /**
     * @brief The Buffer struct handles mixing in colors and prefixes
     * to diagnostic strings.
     */
    struct Buffer : public std::stringbuf
    {
        Buffer()
        {
            verbose=0;
        }
        std::ostream & Mystream()
        {
            static std::ostream nullout(nullptr);
            switch(verbose)
            {
            case 0:
                return nullout;
            case 1:
                return std::cout;
            default:
                return nullout;
            }
        }

        int sync()
        {
            std::string msg =  this->str() ;

            // don't sync synch unless  end of debug string ...
            if(msg.find("\n")== std::string::npos)
                return 0;

            // No color just print normal
            if(_color.empty())
            {
                Mystream() << this->_prefix << this->str() ;
            }
            // check for linux terminal color scheme
#ifdef __linux__
            else
            {
                Mystream() << "\033[" << this->_color << "m" << this->_prefix << this->str() << "\033[0m";
            }
#else
            //  otherwise just output
            else
            {
                Mystream() << this->_prefix << this->str() ;

            }
#endif

            // Don't want crazy color code chararacters in file only prefix.
            if(ofs && (*ofs).is_open())
            {
                (*ofs) << this->_prefix << this->str();
            }

            // now clear formatting and string
            this->str("");
            _color.clear();
            _prefix.clear();
            return 0; // everything ok
        }

        std::string _color; // linux terminal color scheme for now
        std::string _prefix; // prefix to output before message
        int verbose;  ///  0=no logging, 1=console
        std::ofstream *ofs;  /// pointer to ofstream, null if none and not logging
    };


    Logger() :   std::ostream(buffer=new Buffer()),
        nullOstream(nullptr)
    {
        this->setf(std::ios_base::unitbuf);
    }
    ~Logger()
    {
        if(buffer->ofs)
            (*buffer->ofs).close();
        buffer->ofs=nullptr;
    }

    static std::map<std::string, std::ofstream> &openfds()
    {
        static std::map<std::string, std::ofstream> ofs;
        return ofs;
    }

    int & enable()
    {
        return  buffer->verbose;
    }
    int  isEnabled()
    {
        return  buffer->verbose ||  buffer->ofs ;
    }
    void open(std::string filename)
    {
        std::map<std::string, std::ofstream> & ofs(openfds());
        std::map<std::string, std::ofstream>::iterator it;
        if((it=ofs.find(filename))!= ofs.end())
        {
            // do nothing...
        }
        else
        {
            ofs[filename].open(filename);

        }
        buffer->ofs=&openfds()[filename];

    }
    void close()
    {
        // if not null pointer close
        if(buffer->ofs)
            (*buffer->ofs).close();
        buffer->ofs=nullptr;
    }
    Logger &operator()()
    {
      return (*this);
    }


    static int & debugLevel()
    {
        static int level=0;
        return level;
    }

    std::ostream &logit(std::string filename="", int lineno=0, int level=0)
    {
        if(level > debugLevel())
            return nullOstream;

        if(level==0)
        {
            buffer->_color="0;31";
            buffer->_prefix="[ERR] ";
        }
        if(level ==3)
        {
            buffer->_color="1;34";
            buffer->_prefix="[DBG] ";

        }
        else  if(level ==5)
        {
            buffer->_color="1;33";
            buffer->_prefix="[WRN] ";

        }
        setup(buffer->_prefix, filename,  lineno);
        return (*this);
    }

    void color(std::string name)
    {
#ifdef __linux__
           linuxcolor(name);
#endif

    }

    // fixme this only works in one file
    std::ostream &once(int line)
    {
        buffer->_prefix.clear();
        buffer->_color.clear();
        static     std::map<int,int> seen;
//        static std::ostream nullOstream(nullptr);
        bool bFlag=false;
        if(seen[line])
            bFlag=true;
        if(bFlag)
            return nullOstream;
        seen[line]++;
        return (*this);
    }
    std::ostream &throttle(int line, int counter)
    {
        buffer->_prefix.clear();
        buffer->_color.clear();
        static     std::map<int,int> seen;
//        static std::ostream nullOstream(nullptr);
        bool bFlag=false;
        if((seen[line]%counter) == 0 )
            bFlag=true;
        seen[line]++;

        if(bFlag)
            return (*this);
        return nullOstream;
    }


    template<typename ... Args>
    void logMessage (const char *filename,
                                   int lineno,
                                   int level,
                                   const char *format,
                                   Args ... args)
    {
        if(level > debugLevel())
            return ;

        // this is where level of debugging would go
        size_t size = snprintf( nullptr, 0, format, args ... ) + 1; // Extra space for '\0'
        std::unique_ptr<char[]> buf( new char[ size ] );
        snprintf( buf.get(), size, format, args ... );
        std::string msg( buf.get(), buf.get() + size - 1 ); // We don't want the '\0' inside
        setup(buffer->_prefix, filename,  lineno);
        (*this) << msg;
    }

private:
    Logger &setup(std::string &prefix, std::string filename="", int lineno=0)
    {
        if(filename.size() > 0)
        {
            prefix+="[" ;
            filename=filename.substr(filename.find_last_of('/') + 1);
            prefix+=filename ;
            prefix+=":" ;
            prefix+=std::to_string(lineno) ;
            prefix+="] " ;
        }
        return (*this);
    }

    void linuxcolor(std::string name)
    {
        bool light=false;
        size_t n;
        std::transform(name.begin(), name.end(), name.begin(),
            [](unsigned char c){ return std::tolower(c); });
        if((n=name.find("light"))!=std::string::npos)
        {
            light=true;
            name=name.erase(n,std::string("light").size());
            name.erase(0,name.find_first_not_of(" \n\r\t"));
            name.erase(name.find_last_not_of(" \n\r\t")+1);
        }
        if(name=="white")
            buffer->_color="1;37"; // white
        else if(name=="black")
            buffer->_color="0;30";  // black
        else if(name=="blue")
        {
            buffer->_color = light? "1;" :"0;";
            buffer->_color+="34";  // blue
        }
        else if(name=="green")
        {
            buffer->_color = light? "1;" :"0;";
            buffer->_color+="32";  // green
        }
        else if(name=="cyan")
        {
            buffer->_color = light? "1;" :"0;";
            buffer->_color+="36";  // cyan
        }
        else if(name=="red")
        {
            buffer->_color = light? "1;" :"0;";
            buffer->_color+="31";  // red
        }
        else if(name=="purple")
        {
            buffer->_color = light? "1;" :"0;";
            buffer->_color+="35";  // purple
        }
        else if(name=="brown")

        buffer->_color="[0;33";  // brown
        else if(name=="yellow")
        buffer->_color="[1;33";  // yellow

        else if(name=="gray" ||name=="grey"  )
        {
            if(!light)
                buffer->_color="0;30";  // gray
            else
                buffer->_color="0;37";  // light gray
        }
        else
        buffer->_color="0"; //  No Color

    }
    Buffer * buffer;
    std::ostream nullOstream;

};

#ifndef ONCE
#define ONCE() once(__COUNTER__)
#endif

#ifndef THROTTLE
#define THROTTLE(N) throttle(__COUNTER__,N)
#endif

#ifndef WARN
#define WARN(N) logit(__FILE__,__LINE__,3)
#endif

#ifndef ERR
#define ERR(N) logit(__FILE__,__LINE__,0)
#endif

#ifndef DBG
#define DBG(N) logit(__FILE__,__LINE__,5)
#endif

#ifndef logDEBUG
#define logDEBUG(fmt, ...)     logMessage(__FILE__, __LINE__, 5, fmt,  ## __VA_ARGS__)
#define logDebug(fmt, ...)     logMessage(__FILE__, __LINE__, 5, fmt,  ## __VA_ARGS__)
#endif

#define LOG(fmt, ...)   \
    logMessage(__FILE__, __LINE__, 0, fmt,  ## __VA_ARGS__)

#endif // LOGGING_H
