

/*
DISCLAIMER:
This software was produced by the National Institute of Standards
and Technology (NIST), an agency of the U.S. government, and by statute is
not subject to copyright in the United States.  Recipients of this software
assume all responsibility associated with its operation, modification,
maintenance, and subsequent redistribution.

See NIST Administration Manual 4.09.07 b and Appendix I.
 */

#pragma once

#include <iostream>
#include <exception>
#include <map>
#include <stdarg.h>


//#include <locale>
// IKFAST std::runtime_error("max solutions for joint not initialized")
// IKFAST std::runtime_error("index >= max solutions for joint")
// RCS MSGQ std::runtime_error("Empty queue\n")
// IKFAST std::runtime_error("Error while converting to quaternion! \n")
// IArmConfiguration std::runtime_error("ArmConfiguration not configured\n")
// Scene throw std::runtime_error("Gak UpdateScene!")
// std::runtime_error("Gak DeleteObject!")
// std::runtime_error(std::string("Checkers game could not open file : " + filename + " for reading!").c_str())
// std::runtime_error("RESULT_SINGULAR")
// CController std::runtime_error("Zero joint positions\n");
// math std::runtime_error("Cannot make unit vector out of zero vector")
// std::runtime_error("ERROR: Model Parsing the xml failed")



#define Joint_Limit_Exceeded __COUNTER__
#define Position_Limit_Exceeded __COUNTER__
#define Parsing_Error __COUNTER__
#define Conversion_Error __COUNTER__
#define Bad_Parameter __COUNTER__
#define Robot_IK_Singularity __COUNTER__
#define Not_configured __COUNTER__

class MotionException : public std::exception {
    static std::map<int, std::string> m;
    std::string errmsg;
public:
#if 0

    MotionException(int ex, std::string specifics) {
        // Spanish is "es", English is "en" and French is "fr" and German is "de"
        // http://www.science.co.il/Language/Locale-codes.asp
        // cout << "The language code is " << std::use_facet<boost::locale::info>(some_locale).language() << ndl;
        int n = -1;
        errmsg = m[ex] + specifics;
    };
#endif

    MotionException(int ex, ...) {

        va_list argptr;
        va_start(argptr, ex);
        std::string fmt;
        if (m.find(ex) != m.end())
            fmt = m[ex];
        else
            fmt = "Unknown exception";

        int m;
        int n = (int) strlen(fmt.c_str()) + 1028;
        std::string tmp(n, '0');
        while ((m = vsnprintf(&tmp[0], n - 1, fmt.c_str(), argptr)) < 0) {
            n = n + 1028;
            tmp.resize(n, '0');
        }
        va_end(argptr);
        errmsg = tmp.substr(0, m);
    };
    static void Load();

    virtual const char* what() const throw () {
        return errmsg.c_str();
    }
};




