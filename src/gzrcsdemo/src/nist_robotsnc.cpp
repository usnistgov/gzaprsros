

/*
 * DISCLAIMER:
 * This software was produced by the National Institute of Standards
 * and Technology (NIST), an agency of the U.S. government, and by statute is
 * not subject to copyright in the United States.  Recipients of this software
 * assume all responsibility associated with its operation, modification,
 * maintenance, and subsequent redistribution.
 *
 * See NIST Administration Manual 4.09.07 b and Appendix I.
 */


#include <string>
#include <sys/stat.h>
#include <fcntl.h>
#include <iostream>
#include <sstream>


#include "aprs_headers/Debug.h"
#include "aprs_headers/Config.h"
#define GLOGGER GLogger
#include "aprs_headers/LoggerMacros.h"


#include "gzrcsdemo/Globals.h"
#include "gzrcsdemo/Demo.h"
#include "gzrcsdemo/RobotControlException.h"
#include "gzrcsdemo/Shape.h"
#include "gzrcsdemo/commandlineinterface.h"
// Gazebo and Ros global interfaces
#include "gzrcsdemo/gazebo.h"
#include "gzrcsdemo/cros.h"
#include "gzrcsdemo/urdf.h"

#ifndef MAJOR
#define MAJOR  1
#define MINOR 0
#define BUILD 1
#endif


#include "gzrcsdemo/assimp.h"

using namespace RCS;

// Globals
CGearDemo geardemo;
Logging::CLogger GLogger;

int main(int argc, char** argv)
{
    std::vector<std::string> robots;

    try {
        Globals.catchControlC();  // this must be here or ^C seems to be ignored
        std::string config_file="Config.ini";
        char c;
        while ((c = getopt (argc, argv, "f:r:")) != -1)
            switch (c)
            {
            case 'c':
                config_file = optarg;
                break;
            case 'r':
                robots.push_back(optarg);
                break;
            }



        // Find path of executable
        Globals.appProperties["ExeDirectory"] = getexefolder();
        Globals.appProperties["ConfigFile"] =config_file;
        Globals.appProperties["appName"] = getexepath().substr(getexepath().find_last_of('/') + 1);
        Globals.appProperties["PackageSrcPath"] = getexefolder();
        Globals.appProperties["version"] = std::string("")+std::to_string(MAJOR) +":"+std::to_string(MINOR) +":"+std::to_string(BUILD) ;
        std::cout << Globals.appProperties["appName"] << " Version " << Globals.appProperties["version"] <<" Compiled " << __DATE__ << " " << __TIME__ << "\n" << std::flush;

        //RobotControlException::Load();
        try {

            // Load the ini file in this ptree - THIS MUST BE COPIED OVER DURING INSTALL
            std::string inifile = Globals.appProperties["PackageSrcPath"]
                    + "config/"+ Globals.appProperties["ConfigFile"];
            Globals.appProperties["inifile"] =inifile;

            // Load NIST style - less draconian errors. Maybe worse.
#ifdef DEBUG
            RCS::robotconfig.throwExceptions()=true;
#endif
            if(!RCS::robotconfig.loadFile(inifile))
                throw "ini file  " + inifile + "did not open";

            // Get robot related information
            if(robots.size() == 0)
                robots = RCS::robotconfig.getTokens<std::string>("system.robots", ",");
            if(robots.size() ==0)
                throw "No robots defined";
            Globals.appProperties["robot"] = robots[0] ;

            // ROS configuration
            Globals.bRos=RCS::robotconfig.getSymbolValue<int>("system.ros","0");
            Globals.sRosMasterUrl = RCS::robotconfig.getSymbolValue<std::string>("system.RosMasterUrl","http://localhost:11311");
            Globals.sRosPackageName = RCS::robotconfig.getSymbolValue<std::string>("system.RosPackageName",Globals.appProperties["robot"]+"gzrcs");
            Globals.sRosPackageName = Globals.appProperties["robot"]+Globals.sRosPackageName;
            Globals.sRosPackageName=robots[0]+Globals.sRosPackageName;

            // Gazebo configuratino
            Globals.bGazebo=RCS::robotconfig.getSymbolValue<int>("system.gazebo","0");
            Globals.bCannedDemo=RCS::robotconfig.getSymbolValue<int>("system.CannedDemo","0");
            Globals.bWorldCRCLCoordinates=RCS::robotconfig.getSymbolValue<int>("system.WorldCRCLCoordinates","0");
            Globals.bGzGripperPlugin=RCS::robotconfig.getSymbolValue<int>("system.GzGripperPlugin","0");;

            // Debug Flags for more debugging information:
            Globals.DEBUG_World_Command()=RCS::robotconfig.getSymbolValue<int>("debug.Debug_World_Command","0");
            Globals.DEBUG_Log_Robot_Position()=RCS::robotconfig.getSymbolValue<int>("debug.Log_Robot_Position","0");
            Globals.DEBUG_Log_Robot_Config()=RCS::robotconfig.getSymbolValue<int>("debug.Log_Robot_Config","0");
            Globals.DEBUG_Log_Cyclic_Robot_Position()=RCS::robotconfig.getSymbolValue<int>("debug.Log_Cyclic_Robot_Position","0");
            GLogger.debugLevel()=RCS::robotconfig.getSymbolValue<int>("debug.DebugLevel","0");;
            GLogger.isTimestamping()=RCS::robotconfig.getSymbolValue<int>("debug.Timestamping","1");;
            GLogger.isOutputConsole()=RCS::robotconfig.getSymbolValue<int>("debug.LogConsole","1");;

            Globals.appProperties["robot"] = RCS::robotconfig.getSymbolValue<std::string>(robots[0] + ".robot.longname");


            // adds a breakpoint whenever the variable symbol is accessed - either read or write
            std::vector<std::string> debug_symbols = RCS::robotconfig.getTokens<std::string>("debug.symbols", ",");
            for(size_t k=0; k<debug_symbols.size(); k++)
                add_breakpoint_var(debug_symbols[k]);

            // Debug and logging setup of logging files
            Globals.debugSetup();

            logStatus( "gzrcsdemo: Compiled %s %s\n" , __DATE__ , __TIME__ );
            logStatus( "gzrcsdemo: Build %d\n" , BUILD);
            logStatus( "gzrcsdemo: Started %s\n" , Globals.getTimeStamp().c_str() );


#ifdef ROS
            // Setup up ROS
            if(Globals.bRos)
            {
                Ros.init();
            }
#endif

#ifdef GAZEBO
            // Setup up gazebo
            if(Globals.bGazebo)
            {
                gz.init(Globals.appProperties["robot"],
                        Globals.appProperties["inifile"]);
            }
#endif
            // Configuration options
            Globals.bGearLocations = RCS::robotconfig.getSymbolValue<int>("system.GzGearLocations","0");
            Globals.bFlywheel=RCS::robotconfig.getSymbolValue<double>("CRCL.flywheel", "0");


            for (size_t i = 0; i < robots.size(); i++)
            {

                std::string robotname = RCS::robotconfig.getSymbolValue<std::string>(robots[i] + ".robot.longname");

                Globals.crclIp()=RCS::robotconfig.getSymbolValue<std::string>(robots[i] + ".crcl.Ip", "127.0.0.1");
                Globals.crclPort()=RCS::robotconfig.getSymbolValue<double>(robots[i] + ".crcl.Port", "64444");

                // Part offsets
                std::vector<double> offset;
                offset = RCS::robotconfig.getTokens<double>(robots[i] + ".offset.gripper.smallgear", ",");
                Globals.gripperoffset()["sku_part_small_gear"]=Convert<std::vector<double>, tf::Pose> (offset);
                offset = RCS::robotconfig.getTokens<double>(robots[i] + ".offset.gripper.mediumgear", ",");
                Globals.gripperoffset()["sku_part_medium_gear"]=Convert<std::vector<double>, tf::Pose> (offset);
                offset = RCS::robotconfig.getTokens<double>(robots[i] + ".offset.gripper.largegear", ",");
                Globals.gripperoffset()["sku_part_large_gear"]=Convert<std::vector<double>, tf::Pose> (offset);

                //                ncs[i]->graspforce()["sku_part_large_gear"]= RCS::robotconfig.getSymbolValue<double>(robots[i] + ".graspforce.largegear", "10.");
                //                ncs[i]->graspforce()["sku_part_medium_gear"]= RCS::robotconfig.getSymbolValue<double>(robots[i] + ".graspforce.mediumgear", "10.");
                //                ncs[i]->graspforce()["sku_part_small_gear"]= RCS::robotconfig.getSymbolValue<double>(robots[i] + ".graspforce.smallgear", "10.");


                offset = RCS::robotconfig.getTokens<double>(robots[i] + ".offset.vesselslot", ",");
                Globals.slotoffset()["vessel"]=Convert<std::vector<double>, tf::Pose> (offset);


                Globals.rotationmax() = RCS::robotconfig.getTokens<double>(robots[i] + ".rate.rotationmax", ",");
                Globals.linearmax() = RCS::robotconfig.getTokens<double>(robots[i] + ".rate.linearmax", ",");
                Globals.accelerationMultipler()=RCS::robotconfig.getSymbolValue<double>(robots[i] + ".rate.acceleration_multipler", "10.0");
                Globals.base_rotationmax() = RCS::robotconfig.getTokens<double>(robots[i] + ".rate.rotationmax", ",");
                Globals.base_linearmax() = RCS::robotconfig.getTokens<double>(robots[i] + ".rate.linearmax", ",");


                std::vector<double> dbase = RCS::robotconfig.getTokens<double>(robots[i] + ".xform.base", ",");
                Globals.basePose()=Convert<std::vector<double>, tf::Pose> (dbase);


                // Trasnlate 4 doubles into quaternion
                std::vector<double> dbend = RCS::robotconfig.getTokens<double>(robots[i] + ".xform.qbend",",");
                Globals.QBend() = tf::Quaternion(dbend[0], dbend[1], dbend[2], dbend[3]);


                Globals.Retract() =  Convert<std::vector<double>, tf::Pose>(
                            RCS::robotconfig.getTokens<double>(robots[i] + ".xform.retract", ",")
                            );
                Globals.RetractInv()=Globals.Retract().inverse();

                // Parse and record named joint moves - e.g., home safe
                std::vector<std::string> jointmovenames = RCS::robotconfig.getTokens<std::string>(robots[i] + ".joints.movenames", ",");
                for (size_t j = 0; j < jointmovenames.size(); j++) {
                    std::vector<double> ds = RCS::robotconfig.getTokens<double>(robots[i] + "." + jointmovenames[j], ",");
                    std::transform(jointmovenames[j].begin(), jointmovenames[j].end(), jointmovenames[j].begin(), ::tolower);
                    Globals.namedJointMove()[jointmovenames[j]] = ds;
                }

                // Parse and record named pose moves - e.g., gears
                std::vector<std::string> posemovenames = RCS::robotconfig.getTokens<std::string>(robots[i] + ".pose.movenames", ",");
                for (size_t j = 0; j < posemovenames.size(); j++) {
                    std::vector<double> ds = RCS::robotconfig.getTokens<double>(robots[i] + "." + posemovenames[j], ",");
                    std::transform(posemovenames[j].begin(), posemovenames[j].end(), posemovenames[j].begin(), ::tolower);
                    Globals.namedPoseMove()[posemovenames[j]] = ConvertDblVectorTf(ds);
                }

                // Parse and record macro command sequences - e.g., homing, setup
                std::vector<std::string> macronames = RCS::robotconfig.getTokens<std::string>(robots[i] + ".macros", ",");
                for (size_t j = 0; j < macronames.size(); j++)
                {
                    // Assign robot some preliminary setup commands - FIXME: make names macros?
                    std::string setup_cmds = RCS::robotconfig.getSymbolValue<std::string>(robots[i] + "." + macronames[j], "");
                    std::vector<std::string>  commands = Globals.trimmedTokenize(setup_cmds,",");
                    Globals.namedCommand()[macronames[j]] = commands;
                }


                std::vector<double> jnts(Globals.namedJointMove()["home"]);
                Globals.allJointNumbers().resize(jnts.size());
                std::iota(Globals.allJointNumbers().begin(), Globals.allJointNumbers().end(), 0); // adjusted already to 0..n-1

                std::string baselink = RCS::robotconfig.getSymbolValue<std::string>(robots[i] + ".robot.baselink", "ERROR");
                std::string eelink  = RCS::robotconfig.getSymbolValue<std::string>(robots[i] + ".robot.eelink", "ERROR");

#if 0
                // All this work for the joint names. Not used in kitting demo. Command not based on robot understanding.
                std::string urdffile = Globals.appProperties["PackageSrcPath"] + "config/"+ robotname + ".urdf";
                std::string urdf;
                Globals.readFile(urdffile, urdf);
                Globals.slr.parseURDF(urdf,baselink,eelink);
#endif

                Globals.part_list() = RCS::robotconfig.getTokens<std::string>( robots[i] + ".parts", ",");
                // fixme as to the name
                geardemo.init("fanuc_");


            }
        } catch (std::exception &e) {
            std::cerr << e.what();
            LOG_FATAL << e.what();
            throw;
        }


        if(Globals.bGearLocations)
            geardemo.start();

        // Setup command line interface
        CComandLineInterface cli(geardemo);
        //        for (size_t i = 0; i < ncs.size(); i++)
        //            cli.addController(ncs[i]);


        Globals.sleep(3000);


        // Lame command line interpreter
        std::cout << "> "<< std::flush;

        // Find corresponding macro for robot for homing
        // Execute the homing command strings in the macro
        // homing might actually take robot out of home singularity
        cli.interpretMacro("macro_homing");


        CGlobals::bPaused=false;
        cli.state=RCS::NORMAL;
        int state=0;
        do {

            cli.state = cli.inputLoop();

            if(geardemo.isDone(state))
                state=0;

            if(cli.state==RCS::EXITING)
            {
                CGlobals::bRunning=false;
                break;
            }
            if(cli.state==RCS::PAUSED)
                CGlobals::bPaused=true;

            if(cli.state==RCS::NORMAL)
                CGlobals::bPaused=false;

            if(cli.state==RCS::ONCE)
            {
                CGlobals::bPaused=false;
                Globals.bCannedDemo=true;
            }
            if(cli.state==RCS::AUTO)
            {
                CGlobals::bPaused=false;
                Globals.bCannedDemo=true;
            }

            // If canned demo AND finished last commands
            if(!CGlobals::bPaused && Globals.bCannedDemo && ! geardemo.isBusy(state))
            {
                if(geardemo.issueRobotCommands(state)<0)
                    CGlobals::bRunning=false;  // error - assume unfixable for now
            }

            if(cli.state==RCS::ONCE)
            {
                CGlobals::bPaused=true;
            }

            Globals.sleep(100);


        } while (Globals.ok());

        std::cerr << "Cntrl C pressed  or CLI quit\n" << std::flush;

        // Stopping application
        Globals.bRunning=false;

        // Stop demo (i.e. testing) related activies
        if(Globals.bGearLocations)
            geardemo.stop();

#ifdef FIXME
        // ^C pressed - stop all threads or will hang
        RCS::Thread::stopAll(); // includes thread for Controller, robotstatus
#endif

#ifdef ROS
        // Shutdown ROS
        Ros.close();
#endif
        if(Globals.bGazebo)
        {
            gz.stop();
        }

        // close logging file
        GLogger.close();
    }
    catch (std::string e)
    {
        LOG_FATAL << Globals.strFormat("%s%s", "Abnormal exception end to  CRCL2Robot", e.c_str());
        logFatal( "gzrcsdemo: Abnormal Stop %s\n" , Globals.getTimeStamp() );
    }
    catch (std::exception e)
    {
        LOG_FATAL << Globals.strFormat("%s%s", "Abnormal exception end to  CRCL2Robot", e.what());
        logFatal( "gzrcsdemo: Abnormal Stop %s\n" , Globals.getTimeStamp() );
    }
    catch (...)
    {
        LOG_FATAL << "Abnormal exception end to  CRCL2Robot";
        logFatal( "gzrcsdemo: Abnormal Stop %s\n" , Globals.getTimeStamp() );
    }
    logFatal( "gzrcsdemo: Stopped %s\n" , Globals.getTimeStamp() );
}


