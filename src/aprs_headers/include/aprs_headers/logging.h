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
        Buffer(std::string filename) : _filename(filename)
        {
            verbose=1;
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
            if(!_color.empty())
            {
                 Mystream() << "\033[" << this->_color << "m" << this->_prefix << this->str() << "\033[0m";
            }
            else
            {
                Mystream() << this->str() ;
            }
            // Don't want crazy color code chararacters. But maybe prefix?
            if(ofs.is_open())
                ofs << this->_prefix << this->str();
            this->str("");
            _color.clear();
            _prefix.clear();
            return 0;
        }

        std::string _color;
        std::string _prefix;
        std::string _filename;
        int verbose;
        std::ofstream ofs;
    };


    Logger(std::string filename) :   std::ostream(buffer=new Buffer(filename))
    {
        this->setf(std::ios_base::unitbuf);
    }
    ~Logger()
    {
         buffer->ofs.close();
    }

    int & verbose()
    {
        return  buffer->verbose;
    }
    void open(std::string filename="")
    {
        if(!filename.empty())
            buffer->ofs.open(filename);
        else if(!buffer->_filename.empty())
            buffer->ofs.open(buffer->_filename);
    }
    void close()
    {
        buffer->ofs.close();
    }
    Logger &operator()()
    {
      return (*this);
    }

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
    Logger &error(std::string filename="", int lineno=0)
    {
        buffer->_color="0;31";
        buffer->_prefix="[ERR] ";
        setup(buffer->_prefix, filename,  lineno);
        return (*this);
    }
    Logger &warning(std::string filename="", int lineno=0)
    {
        buffer->_color="1;33";
        buffer->_prefix="[WRN] ";
        setup(buffer->_prefix, filename,  lineno);
        return (*this);
    }
    Logger &debug(std::string filename="", int lineno=0)
    {
        buffer->_color="1;34";
        buffer->_prefix="[DBG] ";
        setup(buffer->_prefix, filename,  lineno);
        return (*this);
    }

    void color(std::string name)
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

    // fixme this only works in one file
    std::ostream &once(int line)
    {
        static     std::map<int,int> seen;
        static std::ostream nullOstream(nullptr);
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
        static     std::map<int,int> seen;
        static std::ostream nullOstream(nullptr);
        bool bFlag=false;
        if((seen[line]%counter) == 0 )
            bFlag=true;
        seen[line]++;

        if(bFlag)
            return (*this);
        return nullOstream;
    }
    Buffer * buffer;
};

#ifndef ONCE
#define ONCE() once(__COUNTER__)
#endif

#ifndef THROTTLE
#define THROTTLE(N) throttle(__COUNTER__,N)
#endif

#ifndef WARN
#define WARN(N) warning(__FILE__,__LINE__)
#endif

#ifndef ERR
#define ERR(N) error(__FILE__,__LINE__)
#endif

#ifndef DBG
#define DBG(N) debug(__FILE__,__LINE__)
#endif

#endif // LOGGING_H
