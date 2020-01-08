#include <QCoreApplication>

#include "boost/iostreams/device/null.hpp"
#include <boost/iostreams/stream.hpp>
#include <iostream>
#include <sstream>
#include <string>
#include <map>

boost::iostreams::stream< boost::iostreams::null_sink > nullOstream( ( boost::iostreams::null_sink() ) );

static std::ostream* os;
#define ONCE(X)  \
{ static int Y##__LINE__=-1;\
    if(++Y##__LINE__==0) os=&X; \
    else os=&nullOstream;}\
    *os

struct Logger : public std::ostream
{
    struct Buffer : public std::stringbuf
    {
        Buffer(int color, std::string prefix) : _color(color),_prefix(prefix) {}
        int sync()
        {
            std::cout << "\033[1;" << this->_color << "m" << this->str() << "\033[0m";
            this->str("");
            return 0;
        }
        int _color;
        std::string _prefix;
    };

    Logger(int color,std::string prefix) :   std::ostream(new Buffer(color, prefix))
    {
        this->setf(std::ios_base::unitbuf);
        prefix="Hello world";
        //this->rdbuf(std::cout.rdbuf());  // simple std::cout replacement
    }
    Logger &operator()()
    {
      //Console::log << "(" << Time::GetWallTime() << ") ";
      //(*this) << this->prefix;

      return (*this);
    }

    // fixme this only works in one file
    std::ostream &once(int line=__LINE__)
    {
        bool bFlag=false;
        if(seen[line])
            bFlag=true;
        if(bFlag)
            return nullOstream;
        seen[line]++;
        return (*this);
    }
    std::map<int,int> seen;
};

std::string get_env_var( std::string const & key )
{
    char * val;
    val = getenv( key.c_str() );
    std::string retval = "";
    if (val != NULL) {
        retval = val;
    }
    return retval;
}
#include <sys/stat.h>
int main(int argc, char *argv[])
{
    QCoreApplication a(argc, argv);
#if ONCEUPONATIME
    for(size_t i=0; i< 100; i++)
    {
        ONCE(std::cout ) << i << " This is a test\n" << std::flush;

    }
#endif
//    std::cout << std::RED << "What a sham\n";

    std::string ldpath = get_env_var("LD_LIBRARY_PATH");
    std::istringstream f(ldpath);
    std::string s;
    std::vector<std::string> paths;
    while (getline(f, s, ':')) {
        if(!s.empty())
            paths.push_back(s);
    }

    paths.push_back("/home/isd/michalos/src/github/nist/gzaprsros-xenial/lib");

    std::string lib("libgokin_plugin.so");
    for(size_t i=0; i< paths.size(); i++)
    {
        struct stat buffer;
        std::string filepath=paths[i]+"/"+lib;
        if (stat (filepath.c_str(), &buffer) == 0)
        {
            printf("%s/%s\n", paths[i].c_str(), lib.c_str());
        }

    }



    const char* tf = std::tmpnam(nullptr);
    std::cout << "tmpfile: " << tf << '\n';

    Logger log(31,"");  // red
    log << " spare me the details\n";

    for(size_t i=0; i< 100; i++)
    {
        log.once() << i << " This is a test\n" << std::flush;

    }

    return a.exec();
}
