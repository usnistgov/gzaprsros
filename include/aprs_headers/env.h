#ifndef ENV_H
#define ENV_H
#include <vector>
#include <stdlib.h>     /* getenv */

class Env
{
public:
#ifdef WIN32
    constexpr static char  separator = ';';
#else
    constexpr static  char  separator = ':';
#endif

    static std::vector<std::string> searchPath(std::vector<std::string> envs)
    {
        std::vector<std::string> paths;

        for(size_t i=0; i< envs.size(); i++)
        {
            std::string ldpath = get_env(envs[i]);
            std::istringstream f(ldpath);
            std::string s;
            while (getline(f, s, separator)) {
                if(!s.empty())
                    paths.push_back(s);
            }
        }
        paths.push_back("/home/isd/michalos/src/github/nist/gzaprsros-xenial/lib");
        return paths;
    }

    /**
     * @brief findPath
     * @param envs
     * @param filename
     * @return
     *     std::vector<std::string> v = { "LD_LIBRARY_PATH","GZRCS_LIBRARY_PATH" };
     *     std::string path = findPath(v,"libgokin_plugin.so");
     */

    static std::string findPath(std::vector<std::string> envs, std::string filename)
    {

        std::vector<std::string> paths = searchPath(envs);
        for(size_t i=0; i< paths.size(); i++)
        {
            struct stat buffer;
            std::string filepath=paths[i]+"/"+filename;
            if (stat (filepath.c_str(), &buffer) == 0)
            {
                return paths[i];
            }

        }
        return "";
    }

    static std::string get_env(std::string  var )
    {
        const char * val = ::getenv( var.c_str() );
        if ( val == 0 ) {
            return "";
        }
        else {
            return val;
        }
    }

    std::string get_environment()
    {
        std::string str;
        FILE * fp = popen("bash -c env ", "r");
        if (fp == NULL) {
            printf("Failed to run command\n" );
            return "";
        }

        char buffer[1025];
        std::vector<std::string> lines;
        while (fgets(buffer, 1024, fp) != NULL)
        {
            str+=buffer;
            lines.push_back(buffer);
        }
        std::sort(lines.begin(), lines.end());
        std::string a = std::accumulate(lines.begin(), lines.end(), std::string(""));
        //      std::cout << str;
        std::cout << a ;


        pclose(fp);
        return str;
    }

    /////////////////////////////////////////////////////////////////////////////

    /**
    * @brief findEnvFile given a lists of environment variables (e.g., PATH) and a filename
    * search for the file on the path (append each tokenized environment variable path) until the file
    * is found. If found, append to fullpath. Can have multiple fullpath matches.
    * @param envs - list of environment variables to search against.
    * @param path - partial file path in which to append environment variable
    * @param fullpath  a string containing multiple file paths separated by a semi colon.
    * @return true found a fullpath, false no fullpath found.
    */
    bool findEnvFile(std::vector<std::string> envs, std::string path, std::string & fullpath)
    {
        struct stat buffer;
        fullpath.clear();
        std::string file ;
        for(size_t i=0; i< envs.size(); i++)
        {
            const char *v = std::getenv( envs[i].c_str() );

            if( v == NULL )
                continue;
            std::vector<std::string> dirs=trimmedTokenize(std::string(v), std::string(1,separator));
            for(size_t j=0; j< dirs.size(); j++)
            {
                file = dirs[j]+"/"+path;
                if (stat (file.c_str(), &buffer) == 0)
                {
                    if(!fullpath.empty())
                        file+=";";
                    fullpath+=file;
                }
            }
        }
        return !fullpath.empty();
    }
private:
    std::string trim (std::string source, std::string delims = " \t\r\n")
    {
        std::string result = source.erase(source.find_last_not_of(delims) + 1);
        return result.erase(0, result.find_first_not_of(delims));
    }
    /**
     * @brief split tokenize a string based on a character delimiter.
     * @param s string to tokenize
     * @param delim character delimiter
     * @return  vector of tokens
     */
    std::vector<std::string> split(const std::string &s, char delim, bool bTrim=true)
    {
        std::stringstream ss(s);
        std::string item;
        std::vector<std::string> elems;
        while (std::getline(ss, item, delim))
        {
            if(bTrim)
                elems.push_back(trim(item));
            else
                elems.push_back(item);
            // elems.push_back(std::move(item)); // if C++11 (based on comment from @mchiasson)
        }
        return elems;
    }

    /**
    * @brief Tokenize takes a string and delimiters and parses into vector
    * @param str string to tokenize
    * @param delimiters string containing delimiters
     * @return  std vector of tokens from parsed string
     */
    std::vector<std::string> tokenize (const std::string & str,
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
    std::vector<std::string> trimmedTokenize (std::string value,
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

};
#endif // ENV_H
