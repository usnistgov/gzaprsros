//
// Config.h
//
// DISCLAIMER:
// This software was developed by U.S. Government employees as part of
// their official duties and is not subject to copyright. No warranty implied
// or intended.

#ifndef __INICONFIG_H__
#define __INICONFIG_H__

#include <fstream>
#include <map>
#include <sstream>
#include <string>
#include <vector>
#include <algorithm>
#include <typeinfo>
#include <iomanip>
//#include <boost/algorithm/string/replace.hpp>
// #define CASE_SENSITIVE

namespace Nist
{
namespace stringy
{
inline bool startsWith(std::string str, std::string substr)
{
    return strncmp(str.c_str(), substr.c_str(), substr.size())==0;
}
inline std::string replaceAll(std::string subject, const std::string& search,
                              const std::string& replace)
{
    size_t pos = 0;
    while ((pos = subject.find(search, pos)) != std::string::npos) {
        subject.replace(pos, search.length(), replace);
        pos += replace.length();
    }
    return subject;
}

inline std::string makeUpper (std::string &str)
{
    std::transform(str.begin( ), str.end( ), str.begin( ), ::toupper);
    return str;
}
/**
                * @brief Trim Utility function to Trim whitespace off the ends of a string
                * @param source input strings
                * @param delims string containing delimiters
                * @return Trimmed string
                */
inline std::string trim (std::string source, std::string delims = " \t\r\n")
{
    std::string result = source.erase(source.find_last_not_of(delims) + 1);
    return result.erase(0, result.find_first_not_of(delims));
}

/**
                * @brief LeftTrim Trims leading spaces
                * @param str std string that is Trimmed
                * @return Trimmed string
                */
inline std::string & leftTrim (std::string & str)
{
    size_t startpos = str.find_first_not_of(" \t\r\n");

    if ( std::string::npos != startpos )
    {
        str = str.substr(startpos);
    }
    return str;
}

/**
                * @brief RightTrim Trims trailing spaces
                * @param str std string that is Trimmed
                * @return Trimmed string
                */
inline std::string & rightTrim (std::string & str, std::string Trim = " \t\r\n")
{
    size_t endpos = str.find_last_not_of(Trim);

    if ( std::string::npos != endpos )
    {
        str = str.substr(0, endpos + 1);
    }
    return str;
}

/**
* @brief Tokenize takes a string and delimiters and parses into vector
* @param str string to tokenize
* @param delimiters string containing delimiters
 * @return  std vector of tokens from parsed string
 */
inline std::vector<std::string> tokenize (const std::string & str,
                                          const std::string & delimiters)
{
    std::vector<std::string> tokens;
    std::string::size_type   delimPos = 0, tokenPos = 0, pos = 0;

    if ( str.length( ) < 1 )
    {
        return tokens;
    }

    while ( 1 )
    {
        delimPos = str.find_first_of(delimiters, pos);
        tokenPos = str.find_first_not_of(delimiters, pos);

        if ( std::string::npos != delimPos )
        {
            if ( std::string::npos != tokenPos )
            {
                if ( tokenPos < delimPos )
                {
                    tokens.push_back(str.substr(pos, delimPos - pos));
                }
                else
                {
                    tokens.push_back("");
                }
            }
            else
            {
                tokens.push_back("");
            }
            pos = delimPos + 1;
        }
        else
        {
            if ( std::string::npos != tokenPos )
            {
                tokens.push_back(str.substr(pos));
            }
            else
            {
                tokens.push_back("");
            }
            break;
        }
    }

    return tokens;
}

/**
                * @brief TrimmedTokenize takes a string and delimiters and parses into
                * vector,
                * but Trims tokens of leading and trailing spaces before saving
                * @param str string to tokenize
                * @param delimiters string containing delimiters
                * @return  std vector of tokens from parsed string Trimmed
                *  tokens of leading and trailing spaces
                */
inline std::vector<std::string> trimmedTokenize (std::string value,
                                                 std::string delimiter)
{
    std::vector<std::string> tokens = tokenize(value, delimiter);

    for ( size_t i = 0; i < tokens.size( ); i++ )
    {
        if ( tokens[i].empty( ) )
        {
            tokens.erase(tokens.begin( ) + i);
            i--;
            continue;
        }
        tokens[i] = trim(tokens[i]);
    }
    return tokens;
}

template<typename T>
T convert (std::string data )
{
    T result;
    try {
        std::istringstream stream(data);

        if ( stream >> result )
        {
            return result;
        }
        //else if ( ( data == "yes" ) || ( data == "true" ) )
        //{
        //	return 1;
        //}
    }
    catch(...)
    {
        // FIXME: should throw on error.
        throw std::runtime_error("Bad conversion");
    }
    return T();
}
template<typename T>
std::string strConvert (T data )
{
    std::ostringstream ss;
    try {
        ss <<  std::fixed << std::setprecision(3)<< data;
    }
    catch(...)
    {
    }
    return ss.str();
}
template<>
inline std::string convert<std::string> (std::string data )
{
    return data;
}
}

template<typename T>
std::vector<T> Convert (std::vector<std::string> data )
{
    std::vector<T> v;
    for(size_t i=0; i< data.size(); i++)
        v.push_back(stringy::convert<T>(data[i]));
    return v;
}

/**
        * @brief The Config class can be used to load simple key/value pairs from a
        *file.
        *
        * @note An example of syntax:
        *	// An example of a comment
        *	username= Bob
        *	gender= male
        *	hair-color= black // inline comments are also allowed
        *	level= 42
        *
        * @note An example of usage:
        *	Config config;
        *	config.load("myFile.txt");
        *
        *	std::string username = config["username"].str();
        *	int level = config["level"].toNumber<int>();
        *
        *	Config config;
        *	config.load(inifile);
        *	OutputDebugString(config.GetSymbolValue("FANUC.INCHES",
        *L"INCHES").c_str());
        *	OutputDebugString(StrFormat("%f\n", config.GetSymbolValue("MAIN.MAXRPM",
        *"9000").toNumber<double>()));
        */

class Config
{
public:
    typedef std::map<std::string, std::vector<std::string> >   SectionMap;
    typedef std::map<std::string, std::string>::iterator     ConfigIt;
private:
    SectionMap _sections;                               //!< map of each section and its string
    std::vector<std::string> _section_order;
    bool _bCaseSensitive;

    std::map<std::string, std::string>
    _inimap;                                            //!< map of full keynames and values
    std::string _filename;
    int _bThrowException;
public:

    /**
                * @brief Config empty constructor. Set case insensitive flag;
                */
    Config( )
    {
        _bCaseSensitive=false;
        _bThrowException=false;
    }

    int & throwExceptions() { return _bThrowException; }
    SectionMap::iterator findSection(std::string sectionname)
    {
        SectionMap::iterator     iter;
#ifdef CASE_SENSITIVE
        iter = sections.find(sectionname);
        if(iter!=_sections.end())
            return iter;
#else
        // Key is now uppercase
        std::transform(sectionname.begin( ), sectionname.end( ), sectionname.begin( ), ::toupper);

        for(iter=_sections.begin(); iter!=_sections.end(); iter++)
        {
            std::string section = (*iter).first;
            std::transform(section.begin( ), section.end( ), section.begin( ), ::toupper);
            if(section==sectionname)
                return iter;
        }
#endif
        return _sections.end();
    }
    /**
                * @brief Clear clears the configuration containers.
                */
    void clear ( )
    {
        _sections.clear( );
        _inimap.clear( );
        _section_order.clear();
    }

    /**
                * @brief GetSections returns the section names the ini file
                * @return std vector of sections as strings. Case is as is in inifile.
                */
    std::vector<std::string> getSections ( )
    {
        std::vector<std::string> s;

        for ( std::map<std::string, std::vector<std::string> >::iterator it
              = _sections.begin( );
              it != _sections.end( ); it++ )
        {
            s.push_back(( *it ).first);
        }
        return s;
    }
    bool isSection(std::string name)
    {
#ifndef CASE_SENSITIVE
        std::transform(name.begin( ), name.end( ), name.begin( ), ::toupper);
#endif
        // Key is now uppercase
        return _sections.find(name)!=_sections.end( );
    }

    /**
                * @brief getkeys returns the keys in a section
                * @param section name of the section
                * @return std vector of keys as strings
                */
    std::vector<std::string> getKeys (std::string section)
    {
        std::vector<std::string> dummy;
        SectionMap::iterator     it = findSection(section); // sections.find(section.c_str( ));

        if ( it != _sections.end( ) )
        {
            return _sections[section.c_str( )];
        }
        return dummy;
    }

    /**
                * @brief getmap returns the keys/value pairs in a section
                * @param section name of the section
                * @return std map with key and values as strings
                */
    std::map<std::string, std::string> getMap (std::string section)
    {
        std::map<std::string, std::string> mapping;

        if ( _sections.end( ) == findSection(section) )
        {
            return mapping;
        }
        std::vector<std::string> keys = _sections[section];

        for ( size_t i = 0; i < keys.size( ); i++ )
        {
            mapping[keys[i]] = getSymbolValue<std::string>(section + "." + keys[i]);
        }
        return mapping;
    }

    bool createSection(std::string section)
    {
        if(isSection(section))
        {
            // fixme: make case insensitive
            _sections[section]= std::vector<std::string>();
            return true;
        }
        return false;
    }

    /**
                * @brief load parses key/value pairs by section from a file
                * @param filename path of file to parse
                * @return  whether or not this operation was successful true if sucessful,
                * false if failed
                */
    bool loadFile (const std::string filename)
    {
        this->_filename=filename;
        std::string section;
        std::string line;
        std::string comment   = "#";
        std::string delimiter = "=";
        size_t ln=0;

        std::ifstream file; //(filename.c_str( ));

        try {
            file.open(filename.c_str( ));
            if ( !file.is_open( ) )
            {
                std::cout <<  "ini file did not open" << filename << " maybe doesn't exist?";
                return false;
            }
            while ( std::getline(file, line)  )
            {
                ln++;
                if(stringy::trim(line).empty())
                {
                    std::stringstream key;
                    key << "__EMPTY_LINE" << ln;
                    _sections[section].push_back(key.str());
                    continue;
                }
                if(stringy::startsWith(stringy::trim(line),";" )||
                        stringy::startsWith(stringy::trim(line),"#") )
                {
                    std::stringstream key;
                    key << "__COMMENT" << ln;
                    _sections[section].push_back(key.str());
                    _inimap[section+"."+key.str()] = line;

                    continue;
                }

                // Remove any comments
                size_t commIdx = line.find(comment);

                if ( commIdx != std::string::npos )
                {
                    line = line.substr(0, commIdx);
                }

                // Remove ; windows comment
                commIdx = line.find(";");

                if ( commIdx != std::string::npos )
                {
                    line = line.substr(0, commIdx);
                }
                // This should only match [section], not a=b[3]

                line=stringy::trim(line);

                if ( line.size( ) < 1 )
                {
                    continue;
                }
                size_t delimIdx = line.find("[");

                // check for trailing ] on same line
                if(delimIdx!= std::string::npos && line.find("]") == std::string::npos)
                {
                    fprintf(stderr, "Ini file %s line %d Missing ] to leading [\n", filename.c_str(), (int) ln);
                    continue;
                }

                if (delimIdx!= std::string::npos )                               // && (line.rfind("]")==(line.size()-1)) ) // << BUG  here
                {
                    line    = stringy::trim(line);
                    line    = line.erase(line.find("]"));
                    line    = line.erase(0, line.find("[") + 1);
                    section = stringy::trim(line);
                    _section_order.push_back(section);
                    continue;
                }

                delimIdx = line.find(delimiter);

                if ( delimIdx == std::string::npos )
                {
                    continue;
                }
                std::string key;
                std::string value;
                bool bConcat=false;

                if(line[delimIdx-1]=='+')
                {
                    bConcat=true;
                    key   = stringy::trim(line.substr(0, delimIdx-1));
                }
                else
                {
                    key   = stringy::trim(line.substr(0, delimIdx));
                }
                value = stringy::trim(line.substr(delimIdx + 1));
                _sections[section].push_back(key);

                if ( !key.empty( ) )
                {
                    if ( !section.empty( ) )
                    {
                        key = section + "." + key;
                    }
                    if(bConcat)
                    {
                        _inimap[key] = _inimap[key] + value;
                    }
                    else
                    {
                        _inimap[key] = value;
                    }
                    // string replacements
                    _inimap[key]=stringy::replaceAll( _inimap[key], "\\n", "\n");
                    _inimap[key]=stringy::replaceAll( _inimap[key], "\\r", "\r");
                    _inimap[key]=stringy::replaceAll( _inimap[key], "\\t", "\t");
                }
            }
        }
        catch (std::ios_base::failure& e) {
            std::cerr << e.what() << '\n';
        }
        file.close( );

        return true;
    }

    /**
                * @brief GetTokens returns a list of tokens associated with a key
                * @param keyName string containing keyname (uses section.keyname).
                * @param delimiter token used to tokenize values into vector
                * @return  vector of values from tokenizing
                */
    template<typename T>
    std::vector<T> getTokens (std::string keyName,
                              std::string delimiter)
    {
        std::string              value  = getSymbolValue<std::string>(keyName, "");
#if 0
        // Trim optional leading and trailing []
        // THis is to imitate the python list file
        value.erase(value.find_last_not_of("]")+1);
        value.erase(value.find_first_not_of("[")+1);
#endif
        std::vector<std::string> tokens = stringy::trimmedTokenize(value, delimiter);

        for ( size_t i = 0; i < tokens.size( ); i++ )
        {
            if ( tokens[i].empty( ) )
            {
                tokens.erase(tokens.begin( ) + i);
                i--;
            }
            tokens[i]=stringy::trim(tokens[i]);
        }
        return Convert<T>(tokens);
    }


    bool exists(std::string   keyName)
    {
        std::map<std::string, std::string>::const_iterator iter;
#ifdef CASE_SENSITIVE
        iter = inimap.find(keyName);
#else
        // Key is now uppercase
        std::transform(keyName.begin( ), keyName.end( ), keyName.begin( ), ::toupper);

        for(iter=_inimap.begin(); iter!=_inimap.end(); iter++)
        {
            std::string key = (*iter).first;
            std::transform(key.begin( ), key.end( ), key.begin( ), ::toupper);
            if(key==keyName)
                break;
        }
#endif
        if ( iter != _inimap.end( ) )
            return true;
        return false;
    }

    /**
                * @brief GetSymbolValue looks up a keys value in a section.
                * Assume tree by separating section and key name by period.
                * @param keyName string containing keyname (uses section.keyname).
                * @param szDefault default value associated with key name, if not found
                * @return  variant containing data
                */
    template<typename T>
    T getSymbolValue (std::string   keyName,
                      std::string szDefault = std::string( ))
    {
        std::map<std::string, std::string>::const_iterator iter;
#ifdef CASE_SENSITIVE
        iter = inimap.find(keyName);
#else
        // Key is now uppercase
        std::transform(keyName.begin( ), keyName.end( ), keyName.begin( ), ::toupper);

        for(iter=_inimap.begin(); iter!=_inimap.end(); iter++)
        {
            std::string key = (*iter).first;
            std::transform(key.begin( ), key.end( ), key.begin( ), ::toupper);
            if(key==keyName)
                break;
        }
#endif
        if ( iter != _inimap.end( ) )
        {
            return stringy::convert<T>(iter->second);
        }
        if(_bThrowException)
        {
            std::cout <<  "ini file parameter" << keyName << " doesn't exist";
            throw  "ini file parameter" + keyName + " doesn't exist";
        }
        return  stringy::convert<T>(szDefault);
    }

    /////////////////////////////////////////////////////////////

    /**
                * @brief StoreKey
                * @param key section.key1 key1is the tree branch to store value
                * @fixme section.key1 should not be a section, but could be.
                * @param value string value
                * @return 0 success, -1 failed
                */
    int setKeyValue(std::string key, std::string value)
    {
        // test if valid key (i.e., section.key)
        // should have period to separate section and key
        if(key.find(".") == std::string::npos)
            return -1;


        // store new key value
        _inimap[key] = value;

        // Create (sub)sections if they don't exist
        std::string fullkeyname(key);
        std::string partialsection;
        while(!fullkeyname.empty())
        {
            partialsection += fullkeyname.substr(0, fullkeyname.find_first_of('.'));
            size_t n = fullkeyname.find_first_of('.');
            if( n != std::string::npos)
                fullkeyname = fullkeyname.substr(fullkeyname.find_first_of('.')+1);
            else
            {
                fullkeyname.clear();
                continue;
            }

            // create new section if not found
            if(_sections.find(partialsection) == _sections.end())
            {
                _sections[partialsection]=std::vector<std::string>();
                this->_section_order.push_back(partialsection);
            }

            partialsection += ".";
        }

        // find section and key -  could be [section.subsection].key2
        std::string section = key.substr(0, key.find_last_of('.'));
        key = key.substr( key.find_last_of('.')+1);
        key=stringy::trim(key);

        // add key if doesn't exist in  list of section keys
        if(std::find(_sections[section].begin(), _sections[section].end(), key) == _sections[section].end())
            _sections[section].push_back(key);

        return 0;
    }

    /**
                * @brief DeleteKey
                * @param key section.key1 key1is the tree branch to store value
                * @fixme section.key1 should not be a section, but could be.
                * @return 0 success, -1 failed
                */
    int deleteKey(std::string key)
    {
        // test if valid key (i.e., section.key)
        // should have period to separate section and key
        if(key.find(".") == std::string::npos)
            return -1;

        // find section and key -  could be [section.subsection].key2
        std::string section = key.substr(0, key.find_last_of('.'));

        // delete section if key exists in section list of keys
        deleteSection(section);

        // delete key value
        ConfigIt it = _inimap.find(key);
        if(it != _inimap.end())
            _inimap.erase(it);

        return 0;
    }


    int deleteSection(std::string section)
    {
        if(!isSection(section))
            return -1;

        // erase all inimap keys
        _sections[section]=std::vector<std::string>();

        // erase section
        _sections.erase(findSection(section));

        return 0;
    }
    int mergeKeys(std::string section, std::map<std::string,std::string> keyvalues)
    {
        // sections are only one deep
        if(_sections.find(section) == _sections.end())
            _sections[section]=std::vector<std::string>();
        for(std::map<std::string,std::string>::iterator it = keyvalues.begin();
            it != keyvalues.end(); it++)
        {
            // really this should never happen, should be an error
            if(std::find(_sections[section].begin(), _sections[section].end(), (*it).first) == _sections[section].end())
                _sections[section].push_back((*it).first);
            std::string key = section +  "." + (*it).first;
            _sections[section].push_back((*it).first);
            _inimap[key] = (*it).second;
        }
        return 0;

    }

    template<typename T>
    void storeKey(std::string key, std::vector< T> values)
    {
        key=stringy::trim(key);
        std::string value;
        std::stringstream ss;
        for(size_t i=0; i< values.size(); i++)
        {
            if(i>0) ss << ", ";
            ss << values[i];
        }
        //value+="["+ss.str()+"]";
        value+=ss.str();
        _inimap[key] = value;

        std::string section = key.substr(0, key.find('.'));
        key = key.substr( key.find('.')+1);
        _sections[section].push_back(key);
    }
    /**
                * @brief ToString generate ini file string
                * @return  string
                */
    std::string toString(std::vector<std::string> sectionorder=std::vector<std::string>())
    {
        std::string tmp;
        std::vector<std::string> allsections;

        if(sectionorder.size()>0)
            allsections=sectionorder;
        else
            allsections= _section_order;

        for(size_t i=0; i< allsections.size(); i++)
        {
            tmp+="["+allsections[i]+"]\n";
            std::vector<std::string> keys = _sections[allsections[i]];
            for(size_t j=0; j< keys.size(); j++)
            {
                if(stringy::startsWith(keys[j], "__EMPTY_LINE"))
                {
                    tmp+="\n";
                    continue;
                }
                if(stringy::startsWith(keys[j], "__COMMENT"))
                {
                    std::string value = getSymbolValue<std::string>(allsections[i] + "." + keys[j]);
                    tmp +=  value + "\n";
                    continue;
                }
                std::string value = getSymbolValue<std::string>(allsections[i] + "." + keys[j]);
                tmp += keys[j] + "=" + value + "\n";
            }
#if 0
            std::map<std::string, std::string> keymap = GetMap (allsections[i]);
            std::map<std::string, std::string>::iterator it;
            for(it=keymap.begin(); it!=keymap.end(); it++)
            {
                tmp+=(*it).first + "=" + (*it).second + "\n";
            }
#endif
            tmp+="\n";
        }
        return tmp;
    }

    void save(std::vector<std::string> sectionorder=std::vector<std::string>())
    {
        if(_filename.empty())
            return;
        std::string contents=toString(sectionorder);
        std::ofstream outFile(_filename.c_str( ) );
        outFile << contents.c_str( );
    }
    void saveAs(std::string myfilename)
    {
        if(myfilename.empty())
            return;
        std::string contents=toString();
        std::ofstream outFile(myfilename.c_str( ) );
        if (!outFile.is_open())
            return;
        outFile << contents.c_str( );
    }
};
}

#endif
