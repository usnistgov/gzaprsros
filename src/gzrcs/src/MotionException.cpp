

/*
DISCLAIMER:
This software was produced by the National Institute of Standards
and Technology (NIST), an agency of the U.S. government, and by statute is
not subject to copyright in the United States.  Recipients of this software
assume all responsibility associated with its operation, modification,
maintenance, and subsequent redistribution.

See NIST Administration Manual 4.09.07 b and Appendix I.
 */

#include <boost/assign.hpp>
#include <boost/locale.hpp>

#include "nist_robotsnc/MotionException.h"
#include "nist_robotsnc/Globals.h"
#include "nist_robotsnc/NIST/Config.h"

using namespace boost::assign;



std::map<int, std::string> MotionException::m
{
    {Joint_Limit_Exceeded, "Joint Limit exceeded"},
    {Position_Limit_Exceeded, "Position Limit exceeded"},
    {Parsing_Error, "Parsing Error"},
    {Conversion_Error, "Conversion Error"},
    {Bad_Parameter, "Bad Parameter"},
    {Robot_IK_Singularity, "Robot IK Singularity"},
    {Not_configured, "Not configured"}
};

void MotionException::Load()
{
//	std::string path = Globals._appproperties[ROSPACKAGENAME];
//    // Spanish is "es", English is "en" and French is "fr" and German is "de"
//    // http://www.science.co.il/Language/Locale-codes.asp
//    std::locale l("");
//    std::string lang = l.name();
//    if (lang.compare(0, std::string("en").length(), std::string("en")) == 0) {
//        Nist::Config cfg;
//        cfg.load(path + "/config/English.ini");
//        m = cfg.gettemplatemap<int, std::string> ("faults");

//    }
    
}         
