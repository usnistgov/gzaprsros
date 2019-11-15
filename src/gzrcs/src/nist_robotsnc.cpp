

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


#include "gzrcs/Globals.h"
#include "gzrcs/Kinematics.h"
#include "gzrcs/Controller.h"
#include "gzrcs/RCSInterpreter.h"
#include "gzrcs/Demo.h"
#include "gzrcs/RobotControlException.h"
#include "gzrcs/Shape.h"
#include "gzrcs/commandlineinterface.h"
// Gazebo and Ros global interfaces
#include "gzrcs/gazebo.h"
#include "gzrcs/cros.h"

#ifdef GOKIN
#include "gzrcs/Fanuc/fanuc_lrmate200id.h"
#endif

#ifdef Trac_IK
#include <gzrcs/Fanuc/fanuc_lrmate200id_tracik.h>
#endif


#ifndef MAJOR
#define MAJOR  1
#define MINOR 0
#define BUILD 1
#endif


#include "gzrcs/assimp.h"

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
            Globals.bGzGripperPlugin=RCS::robotconfig.getSymbolValue<int>("system.GzGripperPlugin","0");;

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

            // Debug and logging setup of logging files
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
                ncs[i]->robotEelink() = RCS::robotconfig.getSymbolValue<std::string>(robots[i] + ".robot.eelink", "ERROR");
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

                std::vector<double> dbase = RCS::robotconfig.getTokens<double>(robots[i] + ".xform.base", ",");
                std::vector<double> dbend = RCS::robotconfig.getTokens<double>(robots[i] + ".xform.qbend",",");
                ncs[i]->setBaseOffset(Convert<std::vector<double>, tf::Pose> (dbase));

                // Trasnlate 4 doubles into quaternion
                ncs[i]->QBend() = tf::Quaternion(dbend[0], dbend[1], dbend[2], dbend[3]);

#ifdef GOKIN
                KinUtils::SetQuaternionFromRpy( Deg2Rad(dbend[0]), Deg2Rad(dbend[1]), Deg2Rad(dbend[2]), ncs[i]->QBend());
#endif

                ncs[i]->Retract() =  Convert<std::vector<double>, tf::Pose>(
                            RCS::robotconfig.getTokens<double>(robots[i] + ".xform.retract", ",")
                            );
                ncs[i]->RetractInv()=ncs[i]->Retract().inverse();

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


                // the increment to close the gripper - some positive some negative some little some big
//                ncs[i]->fingerIncrement()= RCS::robotconfig.getTokens<double>( ncs[i]->gripperName()  + ".fingerIncrement", ",");

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

                // Choose kinematic solver
                std::string kinsolver = RCS::robotconfig.getSymbolValue<std::string>(robots[i] + ".nc.kinsolver", "");
                std::shared_ptr<IKinematics> kin;
                if (kinsolver == "FanucLRMate200idFastKinematics")
                    kin = std::shared_ptr<IKinematics>(new FanucLRMate200idFastKinematics(ncs[i]));
                else if (kinsolver == "MotomanSia20dFastKinematics") {
                    kin = std::shared_ptr<IKinematics>(new MotomanSia20dFastKinematics(ncs[i]));
                }
                else if (kinsolver == "GoMotoKin") {
                    kin = std::shared_ptr<IKinematics>(new MotomanSia20dGoKin(ncs[i]));
                }

#ifdef Trac_IK
                else if (kinsolver == "FanucTracIk") {
                    kin = std::shared_ptr<IKinematics>(new Fanuc200idTrac_IK(ncs[i]));
                }
#endif
                try {
                    kin->init(std::string("manipulator"),
                              ncs[i]->robotEelink(),
                              ncs[i]->robotBaselink());
                    ncs[i]->robotKinematics() = kin;

                    // FIXME: if ros read param robot_description
                    std::string urdffile = Globals.appProperties["PackageSrcPath"] + "config/"+ robotname + ".urdf";
                    std::string urdf;
                    Globals.readFile(urdffile, urdf);
                    kin->initUrdf(urdf);

                } catch (std::exception & ex) {
                    std::cout << "robotKinematics error: " << ex.what() << "\n";
                }
                std::cout << "robotKinematics " << ncs[i]->name().c_str() << ncs[i]->robotKinematics()->kinName << "\n" << std::flush;

                // Choose trajectory interpreter

                std::string traj = RCS::robotconfig.getSymbolValue<std::string>(robots[i] + ".nc.traj", "Go");
                if(traj=="Go")
                {
                    ncs[i]->robotInterpreter() = std::shared_ptr<IRCSInterpreter>(new RCS::CGoInterpreter(ncs[i], kin));
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
                geardemo.init(ncs[i]->robotPrefix());

                if(Globals.DEBUG_Log_Robot_Config())
                    RCS::CController::dumpRobotNC(ncs[i]);

            }
        } catch (std::exception &e) {
            std::cerr << e.what();
            LOG_FATAL << e.what();
            throw;
        }


        if(Globals.bGearLocations)
            geardemo.start();

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
        int state=0;
        do {
            for (size_t i = 0; i < ncs.size(); i++)
            {
                cli.state = cli.inputLoop();

               if(geardemo.isDone(state,*(crclApi.get())))
                   state=0;

               if(cli.state==CController::EXITING)
                {
                    CGlobals::bRunning=false;
                    break;
                }
                if(cli.state==CController::PAUSED)
                    CGlobals::bPaused=true;

                if(cli.state==CController::NORMAL)
                    CGlobals::bPaused=false;

                if(cli.state==CController::ONCE)
                {
                    CGlobals::bPaused=false;
                    Globals.bCannedDemo=true;
                }
                if(cli.state==CController::AUTO)
                {
                    CGlobals::bPaused=false;
                    Globals.bCannedDemo=true;
                }

                // If canned demo AND finished last commands
                if(!CGlobals::bPaused && Globals.bCannedDemo && ! ncs[i]->isBusy())
                {
                    geardemo.issueRobotCommands(state, *(crclApi.get()));
                }

                if(cli.state==CController::ONCE)
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
            geardemo.stop();

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


