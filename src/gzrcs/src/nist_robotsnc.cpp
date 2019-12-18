

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
#include "aprs_headers/env.h"


#include "gzrcs/Globals.h"
//#include "gzrcs/Kinematics.h"
#include "gzrcs/Controller.h"
#include "gzrcs/RCSInterpreter.h"
#include "gzrcs/Demo.h"
#include "gzrcs/RobotControlException.h"
#include "gzrcs/Shape.h"
#include "gzrcs/commandlineinterface.h"

// Gazebo and Ros global interfaces
#include "gzrcs/gazebo.h"
#include "gzrcs/cros.h"


#ifndef MAJOR
#define MAJOR  1
#define MINOR 0
#define BUILD 1
#endif


//#include "gzrcs/assimp.h"
using namespace RCS;

//#define BOOST_DLL_USE_STD_FS
#define BOOST_NO_CXX11_VARIADIC_TEMPLATES
#define BOOST_NO_CXX11_TRAILING_RESULT_TYPES
#include <boost/dll/import.hpp>
#include <aprs_headers/IKinematic.h>

// Globals
std::shared_ptr<CGearDemo> geardemo;
Logging::CLogger GLogger;
static     std::string get_env(  std::string  var )
{
    const char * val = ::getenv( var.c_str() );
    if ( val == 0 ) {
        return "";
    }
    else {
        return val;
    }
}

// Do not put creator where it can be destroyed or you will
// get segmentation fault. It's here for now.
typedef boost::shared_ptr<RCS::IKinematic> (pluginapi_create_t)();
boost::function<pluginapi_create_t> creator;
bool bSetupDebug=false;
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
            RCS::robotconfig.throwExceptions()=false; // use defaults

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
            Globals.sRosPackageName = RCS::robotconfig.getSymbolValue<std::string>("system.RosPackageName","gzrcs");
            Globals.sRosPackageName=robots[0]+Globals.sRosPackageName;

            // Gazebo configuratino
            Globals.bGazebo=RCS::robotconfig.getSymbolValue<int>("system.gazebo","0");
            Globals.bCannedDemo=RCS::robotconfig.getSymbolValue<int>("system.CannedDemo","0");
            Globals.bWorldCRCLCoordinates=RCS::robotconfig.getSymbolValue<int>("system.WorldCRCLCoordinates","0");
            Globals.bGzGripperPlugin=RCS::robotconfig.getSymbolValue<int>("system.GzGripperPlugin","1");;
            Globals.bDebug=RCS::robotconfig.getSymbolValue<int>("system.debug","1");
            bSetupDebug=RCS::robotconfig.getSymbolValue<int>("system.debug","0");



            // Debug Flags for more debugging information:
            Globals.DEBUG_World_Command()=RCS::robotconfig.getSymbolValue<int>("debug.Debug_World_Command","0");
            Globals.DEBUG_Log_Gripper_Status()=RCS::robotconfig.getSymbolValue<int>("debug.Log_Gripper_Status","0");
            Globals.DEBUG_IKFAST()=RCS::robotconfig.getSymbolValue<int>("debug.Debug_IKFAST","0");
            Globals.DEBUG_Log_Robot_Position()=RCS::robotconfig.getSymbolValue<int>("debug.Log_Robot_Position","0");
            Globals.DEBUG_Log_Robot_Config()=RCS::robotconfig.getSymbolValue<int>("debug.Log_Robot_Config","0");
            Globals.DEBUG_Log_Cyclic_Robot_Position()=RCS::robotconfig.getSymbolValue<int>("debug.Log_Cyclic_Robot_Position","0");
            Globals.DEBUG_LogRobotCrcl()=RCS::robotconfig.getSymbolValue<int>("debug.LogRobotCrcl","0");
            GLogger.debugLevel()=RCS::robotconfig.getSymbolValue<int>("debug.DebugLevel","0");;
            GLogger.isTimestamping()=RCS::robotconfig.getSymbolValue<int>("debug.Timestamping","1");;
            GLogger.isOutputConsole()=RCS::robotconfig.getSymbolValue<int>("debug.LogConsole","1");;

            Globals.appProperties["robot"] = RCS::robotconfig.getSymbolValue<std::string>(robots[0] + ".robot.longname");


            // adds a breakpoint whenever the variable symbol is accessed - either read or write
            std::vector<std::string> debug_symbols = RCS::robotconfig.getTokens<std::string>("debug.symbols", ",");
            for(size_t k=0; k<debug_symbols.size(); k++)
                add_breakpoint_var(debug_symbols[k]);

            // Application setup of debug and logging setup of logging files
            Globals.debugSetup();

            logStatus( "gzrcs: Compiled %s %s\n" , __DATE__ , __TIME__ );
            logStatus( "gzrcs: Build %d\n" , BUILD);
            logStatus( "gzrcs: Started %s\n" , Globals.getTimeStamp().c_str() );


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
                double dCycleTime = RCS::robotconfig.getSymbolValue<double>(robots[i] + ".nc.cycletime", "10.0");
                ncs.push_back(std::shared_ptr<CController>(new RCS::CController(robotname, dCycleTime)));

                ncs[i]->cycleTime() = dCycleTime;
                ncs[i]->robotPrefix() = RCS::robotconfig.getSymbolValue<std::string>(robots[i] + ".robot.prefix", "ERROR");
                ncs[i]->robotTiplink() = RCS::robotconfig.getSymbolValue<std::string>(robots[i] + ".robot.tiplink", "ERROR");
                ncs[i]->robotBaselink() = RCS::robotconfig.getSymbolValue<std::string>(robots[i] + ".robot.baselink", "ERROR");

                ncs[i]->crclPublishStatusRate()=RCS::robotconfig.getSymbolValue<double>(robots[i] + ".crcl.PublishStatusPeriod", "0.05");
                ncs[i]->crclIp()=RCS::robotconfig.getSymbolValue<std::string>(robots[i] + ".crcl.Ip", "127.0.0.1");
                ncs[i]->crclPort()=RCS::robotconfig.getSymbolValue<double>(robots[i] + ".crcl.Port", "64444");

                // Part offsets
                std::vector<double> offset;
                offset = RCS::robotconfig.getTokens<double>(robots[i] + ".offset.gripper.smallgear", ",");
                ncs[i]->gripperoffset()["sku_part_small_gear"]=Convert<std::vector<double>, tf::Pose> (offset);
                offset = RCS::robotconfig.getTokens<double>(robots[i] + ".offset.gripper.mediumgear", ",");
                ncs[i]->gripperoffset()["sku_part_medium_gear"]=Convert<std::vector<double>, tf::Pose> (offset);
                offset = RCS::robotconfig.getTokens<double>(robots[i] + ".offset.gripper.largegear", ",");
                ncs[i]->gripperoffset()["sku_part_large_gear"]=Convert<std::vector<double>, tf::Pose> (offset);

                ncs[i]->graspforce()["sku_part_large_gear"]= RCS::robotconfig.getSymbolValue<double>(robots[i] + ".graspforce.largegear", "10.");
                ncs[i]->graspforce()["sku_part_medium_gear"]= RCS::robotconfig.getSymbolValue<double>(robots[i] + ".graspforce.mediumgear", "10.");
                ncs[i]->graspforce()["sku_part_small_gear"]= RCS::robotconfig.getSymbolValue<double>(robots[i] + ".graspforce.smallgear", "10.");


                offset = RCS::robotconfig.getTokens<double>(robots[i] + ".offset.vesselslot", ",");
                ncs[i]->slotoffset()["vessel"]=Convert<std::vector<double>, tf::Pose> (offset);


                ncs[i]->rotationmax() = RCS::robotconfig.getTokens<double>(robots[i] + ".rate.rotationmax", ",");
                ncs[i]->linearmax() = RCS::robotconfig.getTokens<double>(robots[i] + ".rate.linearmax", ",");
                ncs[i]->accelerationMultipler()=RCS::robotconfig.getSymbolValue<double>(robots[i] + ".rate.acceleration_multipler", "10.0");
                ncs[i]->base_rotationmax() = RCS::robotconfig.getTokens<double>(robots[i] + ".rate.rotationmax", ",");
                ncs[i]->base_linearmax() = RCS::robotconfig.getTokens<double>(robots[i] + ".rate.linearmax", ",");




                // Gripper hacks.
                ncs[i]->gripperName() = RCS::robotconfig.getSymbolValue<std::string>(robots[i] + ".robot.gripper", ",");
                ncs[i]->crclGripperAlgorithm() = RCS::robotconfig.getSymbolValue<std::string>( robots[i] + ".crcl.GripperAlgorithm", "percentage");
                std::vector<double> dtool = RCS::robotconfig.getTokens<double>(ncs[i]->gripperName() + ".tool",",");
                ncs[i]->setGripperOffset(Convert<std::vector<double>, tf::Pose> (dtool));

                std::vector<double> dCorrection = RCS::robotconfig.getTokens<double>(ncs[i]->gripperName() + ".correction",",");
                if(dCorrection.size() ==0)
                    dCorrection={0,0,0,0,0,0,1.};

                ncs[i]->Correction()=ConvertDblVectorTf (dCorrection);
                ncs[i]->CorrectionInv()=ncs[i]->Correction().inverse();

                // Finger gripping contact parameters
                ncs[i]->fingerNames()= RCS::robotconfig.getTokens<std::string>( ncs[i]->gripperName()  + ".fingernames", ",");

                // Parse and record named joint moves - e.g., home safe
                std::vector<std::string> jointmovenames = RCS::robotconfig.getTokens<std::string>(robots[i] + ".joints.movenames", ",");
                for (size_t j = 0; j < jointmovenames.size(); j++) {
                    std::vector<double> ds = RCS::robotconfig.getTokens<double>(robots[i] + "." + jointmovenames[j], ",");
                    std::transform(jointmovenames[j].begin(), jointmovenames[j].end(), jointmovenames[j].begin(), ::tolower);
                    ncs[i]->namedJointMove()[jointmovenames[j]] = ds;
                }

                // Parse and record named pose moves - e.g., gears
                std::vector<std::string> posemovenames = RCS::robotconfig.getTokens<std::string>(robots[i] + ".pose.movenames", ",");
                for (size_t j = 0; j < posemovenames.size(); j++) {
                    std::vector<double> ds = RCS::robotconfig.getTokens<double>(robots[i] + "." + posemovenames[j], ",");
                    std::transform(posemovenames[j].begin(), posemovenames[j].end(), posemovenames[j].begin(), ::tolower);
                    ncs[i]->namedPoseMove()[posemovenames[j]] = ConvertDblVectorTf(ds);
                }

                // Parse and record macro command sequences - e.g., homing, setup
                std::vector<std::string> macronames = RCS::robotconfig.getTokens<std::string>(robots[i] + ".macros", ",");
                for (size_t j = 0; j < macronames.size(); j++)
                {
                    // Assign robot some preliminary setup commands - FIXME: make names macros?
                    std::string setup_cmds = RCS::robotconfig.getSymbolValue<std::string>(robots[i] + "." + macronames[j], "");
                    std::vector<std::string>  commands = Globals.trimmedTokenize(setup_cmds,",");
                    ncs[i]->namedCommand()[macronames[j]] = commands;
                }


                // get kinematic plugin configuration information
                std::string kin_plugin_dll = RCS::robotconfig.getSymbolValue<std::string>(robots[i] + ".nc.kinsolver.plugin", "");

                std::vector<std::string> v = { "LD_LIBRARY_PATH","GZRCS_LIBRARY_PATH" };
                std::string ld_library_path = Env::findPath(v, kin_plugin_dll);

                if(ld_library_path.empty())
                {
                    ld_library_path="/home/isd/michalos/src/github/nist/gzaprsros-xenial/lib";
                }

                // Choose kinematic solver
                boost::filesystem::path lib_path(ld_library_path);

                try {

                    creator = boost::dll::import_alias<pluginapi_create_t>(             // type of imported symbol must be explicitly specified
                         lib_path/kin_plugin_dll,                                           // path to library
                        "create_plugin",                                                // symbol to import
                        boost::dll::load_mode::append_decorations                              // do append extensions and prefixes
                    );

                    ncs[i]->robotKinematics() = creator();
                    std::cout << ncs[i]->robotKinematics() << "\n";
                    std::cout << ncs[i]->robotKinematics()->get("help");

        #if 0
                    // This works for importing single instance of kin solver plugin
                    ncs[i]->robotKinematics() = boost::dll::import<IKinematic> (//using namespace RCS;
                                                                               lib_path/kin_plugin_dll,
                                                                                kin_plugin_name,
                                                                                boost::dll::load_mode::default_mode);
       #endif
                    if(ncs[i]->robotKinematics()==NULL)
                        throw std::runtime_error("Null kinematic plugin");

                    std::vector<std::string> paramnames = RCS::robotconfig.getTokens<std::string>(robots[i] + ".nc.kinsolver.params", ",");
                    for (size_t j = 0; j < paramnames.size(); j++)
                    {
                        std::string value;
                        std::string param= paramnames[j];
                        if(param.find("exepath ",0)==0)
                        {
                            param.erase(0, sizeof("exepath"));
                            value = RCS::robotconfig.getSymbolValue<std::string>(robots[i] + ".nc.kinsolver." + param, "");
                            value=Globals.appProperties["PackageSrcPath"]+ value;
                         }
                        else
                        {
                            value = RCS::robotconfig.getSymbolValue<std::string>(robots[i] + ".nc.kinsolver." + param, "");
                        }

                        ncs[i]->robotKinematics()->set(param,value);
                    }

                    if(ncs[i]->robotKinematics()->init()) //urdf, ncs[i]->robotBaselink(), ncs[i]->robotTiplink())<0)
                    {
                        std::cerr << "Urdf file: " <<  ncs[i]->robotKinematics()->get("urdffile") << std::endl;
                        std::cerr << "errore: " <<  ncs[i]->robotKinematics()->get("error") << std::endl;

                        throw std::runtime_error("robotKinematics() urdf parse failed");

                    }
                    if(bSetupDebug)
                        std::cout << ncs[i]->robotKinematics()->get("HELP") << "\n";

                    std::vector<double> dbase = RCS::robotconfig.getTokens<double>(robots[i] + ".nc.xform.base", ",");
                    std::vector<double> dbend = RCS::robotconfig.getTokens<double>(robots[i] + ".nc.xform.qbend",",");
                    ncs[i]->setBaseOffset(Convert<std::vector<double>, tf::Pose> (dbase));

                    // Translate 4 doubles into quaternion
                    ncs[i]->QBend() = tf::Quaternion(dbend[0], dbend[1], dbend[2], dbend[3]);

                    ncs[i]->Retract() =  Convert<std::vector<double>, tf::Pose>(
                                RCS::robotconfig.getTokens<double>(robots[i] + ".nc.xform.retract", ",")
                                );
                    ncs[i]->RetractInv()=ncs[i]->Retract().inverse();


                }
                catch (const std::exception& e) {
                    throw std::runtime_error(std::string( "boost plugin exception : ")+ e.what());
                }
                catch (boost::exception &e)
                {
                    throw std::runtime_error(std::string( "boost plugin exception : ")+ boost::diagnostic_information(e));
                }
                catch (...) {
                    throw std::runtime_error("boost plugin exception : ");
                }


                // Choose trajectory interpreter
                std::string traj = RCS::robotconfig.getSymbolValue<std::string>(robots[i] + ".nc.traj", "Go");
                if(traj=="Go")
                {
                    ncs[i]->robotInterpreter() = std::shared_ptr<IRCSInterpreter>(new RCS::CGoInterpreter(ncs[i], ncs[i]->robotKinematics()));
                }
                else
                {
                    throw RobotControlException(Interpreter_not_specified, "Not specified in ini file");
                }
                std::cout << "Trajectory generator " <<  ncs[i]->robotInterpreter()->_name << "\n" << std::flush;
                //std::string algorithm = RCS::robotconfig.getSymbolValue<std::string>(robots[i] + ".fingerContactAlgorithm", "force");
                // fixme: should check that enum string exists
                //ncs[i]->robotInterpreter()->fingerContactAlgorithm =  ncs[i]->robotInterpreter()->s2eFingerContactAlgorithm[algorithm];
                //std::cout << "Trajectory finger contact algorithm " <<  ncs[i]->robotInterpreter()->fingerContactAlgorithm << "="<< algorithm << "\n" << std::flush;


                // Setup up robot controller - assign gripper
                ncs[i]->setup();

                // initialize robot interpreter position - assumes move to home
                // should read robot joints, but moving to home ok too
                ncs[i]->robotInterpreter()->init(ncs[i]->namedJointMove()["joints.home"]);

                // Crcl api command interface to robot
                crclApi= std::shared_ptr<CCrclApi>(new CCrclApi(ncs[i]));

                ncs[i]->part_list() = RCS::robotconfig.getTokens<std::string>( robots[i] + ".parts", ",");

//                if(bSetupDebug)
                {
                    ofsRobotURDF.open(Globals.logfolder()+"RobotConfig.log", std::ofstream::app);
                    RCS::CController::dumpRobotNC(ofsRobotURDF, ncs[i]);
                    ofsRobotURDF.close();
                }

            }
        } catch (std::exception &e) {
            std::cerr << e.what();
            LOG_FATAL << e.what();
            throw;
        }

        geardemo=std::shared_ptr<CGearDemo>(new CGearDemo(crclApi));

        if(Globals.bGearLocations)
        {
            geardemo->start();
            geardemo->init(ncs[0]->robotPrefix());
        }

        // Setup command line interface
        CComandLineInterface cli;
        for (size_t i = 0; i < ncs.size(); i++)
            cli.addController(ncs[i]);

        Globals.sleep(3000);


        // start the Controller Session threads
        for (size_t j = 0; j < ncs.size(); j++) {
            ncs[j]->start();
        }

        // Lame command line interpreter
        std::cout << "> "<< std::flush;

        // Find corresponding macro for robot for homing
        // Execute the homing command strings in the macro
        // homing might actually take robot out of home singularity
        cli.interpretMacro("macro_homing");


        CGlobals::bPaused=false;
        cli.state=CController::NORMAL;

        //std::thread t1(&CComandLineInterface::inputLoop, &cli);
        cli.start();

        int demostate=0;
        do {
            for (size_t i = 0; i < ncs.size(); i++)
            {
                // This will hang on auto as it now changes the state back to noop after every empty fetch.
                // Keep last state in cli?
                int clistate= cli.inputState();


//                if(clistate==CController::NOOP)
//                {
//                    Globals.sleep(100);
//                    continue;
//                }


               if(geardemo->isDone(demostate))
                   demostate=0;

               if(clistate==CController::EXITING)
                {
                    CGlobals::bRunning=false;
                    break;
                }
                if(clistate==CController::PAUSED)
                    CGlobals::bPaused=true;

                if(clistate==CController::NORMAL)
                    CGlobals::bPaused=false;

                if(clistate==CController::ONCE)
                {
                    CGlobals::bPaused=false;
                    Globals.bCannedDemo=true;
                }
                if(clistate==CController::AUTO)
                {
                    CGlobals::bPaused=false;
                    Globals.bCannedDemo=true;
                }
                if(clistate==CController::REPEAT)
                {
                    CGlobals::bPaused=false;
                    Globals.bCannedDemo=true;
                    Globals.bRepeatCannedDemo=true;
                }


                // If canned demo AND finished last commands
                if(!CGlobals::bPaused && Globals.bCannedDemo && ! ncs[i]->isBusy())
                {
                    if(geardemo->issueRobotCommands(demostate)<0)
                    {
                        if(Globals.bRepeatCannedDemo)
                        {
                            geardemo->gzInstances.reset();
                            ::sleep(2.0);
                        }
                        else
                            Globals.bCannedDemo=false;
                    }
                }

                if(clistate==CController::ONCE)
                {
                    CGlobals::bPaused=true;
                }

                Globals.sleep(100);
            }

        } while (Globals.ok());

        std::cerr << "Cntrl C pressed  or CLI quit\n" << std::flush;

        // Stopping application
        Globals.bRunning=false;

        // Stop demo (i.e. testing) related activies
        if(Globals.bGearLocations)
            geardemo->stop();

        // ^C pressed - stop all threads or will hang
        RCS::Thread::stopAll(); // includes thread for Controller, robotstatus


#ifdef ROS
        // Shutdown ROS
        Ros.close();
#endif

        // close logging file
        GLogger.close();
    }
    catch (std::string e)
    {
        LOG_FATAL << Globals.strFormat("%s%s", "Abnormal exception end to  CRCL2Robot", e.c_str());
        logFatal( "gzrcs: Abnormal Stop %s\n" , Globals.getTimeStamp() );
    }
    catch (std::exception e)
    {
        LOG_FATAL << Globals.strFormat("%s%s", "Abnormal exception end to  CRCL2Robot", e.what());
        logFatal( "gzrcs: Abnormal Stop %s\n" , Globals.getTimeStamp() );
    }
    catch (...)
    {
        LOG_FATAL << "Abnormal exception end to  CRCL2Robot";
        logFatal( "gzrcs: Abnormal Stop %s\n" , Globals.getTimeStamp() );
    }
    logFatal( "gzrcs: Stopped %s\n" , Globals.getTimeStamp() );
}


