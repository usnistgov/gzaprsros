

/*
DISCLAIMER:
This software was produced by the National Institute of Standards
and Technology (NIST), an agency of the U.S. government, and by statute is
not subject to copyright in the United States.  Recipients of this software
assume all responsibility associated with its operation, modification,
maintenance, and subsequent redistribution.

See NIST Administration Manual 4.09.07 b and Appendix I.
 */

//#include <boost/assign.hpp>
//#include <boost/locale.hpp>

#include "gzrcsdemo/RobotControlException.h"
#include "gzrcsdemo/Globals.h"
#include "aprs_headers/Config.h"

//using namespace boost::assign;



std::map<int, std::string> RobotControlException::error_strings
{
    {Joint_Limit_Exceeded, "Joint Limit exceeded"},
    {Position_Limit_Exceeded, "Position Limit exceeded"},
    {Parsing_Error, "Parsing Error"},
    {Conversion_Error, "Conversion Error"},
    {Bad_Parameter, "Bad Parameter"},
    {Robot_IK_Singularity, "Robot IK Singularity"},
    {Not_configured, "Not configured"},
    {No_URDF_String, "No URDF String to parse robot joint names"},
    {Robot_IK_Problem, "Robot_IK_Problem"},
    {No_valid_KDL_chain_found, "No_valid_KDL_chain_found"},
    {No_valid_KDL_joint_limits_found, "No_valid_KDL_joint_limits_found"},
    {Mismatched_KDL_joint_size_FK_Joint_size, "Mismatched_KDL_joint_size_FK_Joint_size"},
    {File_not_found,"File_not_found"},
    {Interpreter_not_specified,"Interpreter not specified"},
    {Null_Pointer, "Null pointer"},
    {Initialization_Failed, "Initialization Failed"},
    {Ini_File_Error, "Ini File Error"}

};

////////////////////////////////////////////////////////////////////////////////
void RobotControlException::Load()
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
