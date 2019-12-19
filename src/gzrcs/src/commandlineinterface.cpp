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

#include <algorithm>
#include <stdlib.h>
#include <termios.h>
#include <cstdio>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <array>
#include <fstream>
#include <readline/readline.h>
#include <readline/history.h>

#include <boost/algorithm/string.hpp>

#include "gzrcs/commandlineinterface.h"
#include "aprs_headers/Conversions.h"
#include "aprs_headers/Debug.h"

// THanks to: http://cc.byexamples.com/2007/04/08/non-blocking-user-input-in-loop-without-ncurses/
#define NB_ENABLE 1
#define NB_DISABLE 2

extern     std::shared_ptr<CGearDemo> geardemo;

////////////////////////////////////////////////////////////////////////////////
static std::string shell(const char* cmd) {
    std::array<char, 128> buffer;
    std::string result;
    std::shared_ptr<FILE> pipe(popen(cmd, "r"), pclose);
    if (!pipe)
    {
        if(Globals.bHandleExceptions)
            throw std::runtime_error("popen() failed!");
        return "popen() failed!";
    }
    while (!feof(pipe.get())) {
        if (fgets(buffer.data(), 128, pipe.get()) != nullptr)
            result += buffer.data();
    }
    return result;
}

////////////////////////////////////////////////////////////////////////////////
int kbhit()
{
    struct timeval tv;
    fd_set fds;
    tv.tv_sec = 0;
    tv.tv_usec = 0;
    FD_ZERO(&fds);
    FD_SET(STDIN_FILENO, &fds); //STDIN_FILENO is 0
    select(STDIN_FILENO+1, &fds, NULL, NULL, &tv);
    return FD_ISSET(STDIN_FILENO, &fds);
}

////////////////////////////////////////////////////////////////////////////////
void nonblock(int state)
{
    struct termios ttystate;

    //get the terminal state
    tcgetattr(STDIN_FILENO, &ttystate);

    if (state==NB_ENABLE)
    {
        //turn off canonical mode
        ttystate.c_lflag &= ~ICANON;
        //minimum of number input read.
        ttystate.c_cc[VMIN] = 1;
    }
    else if (state==NB_DISABLE)
    {
        //turn on canonical mode
        ttystate.c_lflag |= ICANON;
    }
    //set the terminal attributes.
    tcsetattr(STDIN_FILENO, TCSANOW, &ttystate);

}
////////////////////////////////////////////////////////////////////////////////
std::vector<double> ConvertV(
        std::vector<std::string> stringVector) {
    std::vector<double> doubleVector;
    std::transform(stringVector.begin(), stringVector.end(), back_inserter(doubleVector),
                   [](std::string const& val) {
        return stod(val);
    });
    return doubleVector;
}

////////////////////////////////////////////////////////////////////////////////
CComandLineInterface::CComandLineInterface()
    : RCS::Thread(0.01, "CLI")
{
    _bDegrees = false;
    _bFlag = true;
    _ncindex=0;
}

////////////////////////////////////////////////////////////////////////////////
void CComandLineInterface::addController(std::shared_ptr<CController>  nc)
{
    this->_nc=nc;
    //    nccmds.push_back(CrclApi(nc));
    //    ncs.push_back(nc);

    _robotNames.clear();
    for(size_t i=0; i< 1; i++)
        _robotNames.push_back(ncs[i]->name());
}

////////////////////////////////////////////////////////////////////////////////
bool CComandLineInterface::jog(char c,double amt, int &jnt, int &axis)
{
    sensor_msgs::JointState curpos,nextpos;
    curpos.position = ncs[_ncindex]->_status.robotjoints.position;
    double incr=0;
    static std::string jogaxis= "xyzijk";

    switch(c)
    {
    case '!':
    case '\n':
        return false;
    case '+':
        incr=amt;
        break;
    case '-':
        incr=-amt;
        break;
    case 'x':
    case 'y':
    case 'z':
    case 'i':
    case 'j':
    case 'k':
        axis =  jogaxis.find(c);
        jnt = -1;
        break;
    case '0':
    case '1':
    case '2':
    case '3':
    case '4':
    case '5':
    case '6':
        jnt = ((int)c - (int) '0');
        axis=-1;
    }

    if(incr==0)
        return true;
    if(jnt>=0)
    {
        nextpos=curpos;
        nextpos.position[jnt]+=incr;
        crclApi->moveJoints(ncs[_ncindex]->allJointNumbers(), nextpos.position);
    }
    if(axis>=0)
    {
        nextpos=curpos;
        tf::Pose r_curpose;
        ncs[_ncindex]->robotKinematics()->FK(curpos.position, r_curpose);
        tf::Matrix3x3  m3x3, newm3x3;
        //tf::Vector3 v = r_curpose.getOrigin();
        if(axis==0)
            r_curpose.getOrigin().setX(r_curpose.getOrigin().x()+incr);
        if(axis==1)
            r_curpose.getOrigin().setY(r_curpose.getOrigin().y()+incr);
        if(axis==2)
            r_curpose.getOrigin().setZ(r_curpose.getOrigin().z()+incr);
        if(axis==3)
        {
            m3x3=r_curpose.getBasis();
            newm3x3.setRPY(incr,0.0,0.0);
            r_curpose.setBasis(m3x3*newm3x3);
        }
        if(axis==4)
        {
            m3x3=r_curpose.getBasis();
            newm3x3.setRPY(0.0,incr,0.0);
            r_curpose.setBasis(m3x3*newm3x3);
        }
        if(axis==5)
        {
            m3x3=r_curpose.getBasis();
            newm3x3.setRPY(0.0, 0.0, incr);
            r_curpose.setBasis(m3x3*newm3x3);
        }

        sensor_msgs::JointState joints;
        try{
            // seed joints
            joints.position=subset(curpos.position, ncs[_ncindex]->robotKinematics()->numJoints());
            ncs[_ncindex]->robotKinematics()->IK(r_curpose, joints.position);
            crclApi->moveJoints(ncs[_ncindex]->allJointNumbers(), joints.position);
        }
        catch(...)
        {
            std::cout << "error jogging - move robot to initialize position since open loop\n" << std::flush;
        }
    }
    return true;
}

////////////////////////////////////////////////////////////////////////////////
void CComandLineInterface::loopCallback(char* line)
{

    //    Non-blocking polled read of std input
    //    struct pollfd fds;
    //    int ret;
    //    fds.fd = 0; /* this is STDIN */
    //    fds.events = POLLIN;
    //    ret = poll(&fds, 1, 0);
    //    if(ret == 0)
    //    {
    //        return CController::NOOP;
    //    }

#if 1
    if(!line)
    {
        lineq.addMsgQueue("quit");
    }
    else
    {
        add_history(line);
        lineq.addMsgQueue(line);
        free(line);
    }

#else
    std::string line;
    if(!std::getline(std::cin, line))
        return CController::EXITING;
    ret =  interpretLine(line);

    if(ret >=0)
        std::cout << "> " << std::flush;
    return ret;
#endif
}
namespace {
boost::function<void(char *)> callback;
extern "C" void wrapper(char * s) {
    callback(s);
}
}
////////////////////////////////////////////////////////////////////////////////
int CComandLineInterface::action()
{
    rl_callback_read_char();
    return 1;
}
////////////////////////////////////////////////////////////////////////////////
void CComandLineInterface::init ( )
{
    // http://www.mcld.co.uk/blog/2009/simple-gnu-readline-callback-style-example.html
    const char *prompt = "RCS> ";
    // Install the callback handler
    callback=boost::bind(&CComandLineInterface::loopCallback, this,_1);
    rl_callback_handler_install(prompt, (rl_vcpfunc_t*) wrapper);

}
#if 0
////////////////////////////////////////////////////////////////////////////////
int CComandLineInterface::inputLoop()
{
    // http://www.mcld.co.uk/blog/2009/simple-gnu-readline-callback-style-example.html
    const char *prompt = "RCS> ";
    // Install the handler
//    rl_callback_handler_install(prompt, (rl_vcpfunc_t*) std::bind(&CComandLineInterface::loopCallback, this,_1));
    callback=boost::bind(&CComandLineInterface::loopCallback, this,_1);
    rl_callback_handler_install(prompt, (rl_vcpfunc_t*) wrapper);

    // Enter the event loop (simple example, so it doesn't do much except wait)
    bRunning = 1;
    while(bRunning){
        usleep(10000);
        rl_callback_read_char();
    };
    return 0;
}

#endif
////////////////////////////////////////////////////////////////////////////////
int CComandLineInterface::inputState()
{
    if(lineq.sizeMsgQueue()>0)
    {
        std::string line = lineq.popBackMsgQueue();
        state =  interpretLine(line);
        return state;
    }
    if(state==CController::EXITING)
    {
        bRunning=false;
    }
    return CController::NOOP;
}


////////////////////////////////////////////////////////////////////////////////
int CComandLineInterface::interpretMacro(std::string macro_name)
{
    std::map<std::string, std::vector<std::string>>::iterator it;
    if((it=ncs[_ncindex]->namedCommand().find(macro_name))!= ncs[_ncindex]->namedCommand().end())
        for(size_t k=0; k< (*it).second.size(); k++)
        {
            interpretLine((*it).second.at(k));
            // should we wait until done with each command?
        }
    else
        return -1;
    return 0;
}


////////////////////////////////////////////////////////////////////////////////
///
int CComandLineInterface::interpretLine(std::string line)
{
    std::vector<std::string>::iterator sit;

    std::string msg = Globals.trim(line);

    // all input translated to lower case  - so case independent
    std::transform(msg.begin(), msg.end(), msg.begin(), ::tolower);

    if (msg.compare("quit") == 0)
    {
        Globals.bCannedDemo = false;
        Globals.sleep(1000);
        // Fixme: empty motion command queue.
        _bFlag = false;
        return CController::EXITING;
    }
    else if (msg.compare("pause") == 0)
    {
        return CController::PAUSED;
    }
    else if (msg.compare("resume") == 0)
    {
        return CController::NORMAL;
    }
    else if (msg.compare("step") == 0)
    {
        return CController::ONCE;
    }
    else if (msg.compare("degrees") == 0)
    {
        _bDegrees = true;
    }
    else if (msg.compare("manual") == 0)
    {
        Globals.bCannedDemo = false;
        return CController::NOOP;
    }
    else if (msg.compare("auto") == 0)
    {
        interpretLine("macro macro_auto");
        ncs[_ncindex]->setGripperJointSpeeds(ncs[_ncindex]->currentGripperJointSpeed()*.5);
        ncs[_ncindex]->setGripperJointSpeeds(ncs[_ncindex]->currentGripperJointSpeed()*.5);

        Globals.bCannedDemo = true;
        return CController::AUTO;
    }
    else if (msg.compare("repeat") == 0)
    {
        Globals.bRepeatCannedDemo = true;
        return CController::REPEAT;
    }
    else if (msg.compare("radian") == 0)
    {
        _bDegrees = false;
    }

    else if (msg.compare("config") == 0)
    {
        std::string filename = Globals.appProperties["ConfigFile"];
    }
    else if (msg.compare("timing") == 0)
    {
        std::cout << RCS::Thread::cpuLoads();
    }
    else if (msg.compare( 0, strlen("bbox"),"bbox") == 0)
    {
        msg=msg.erase(0,std::string("bbox").size());
        std::map<std::string, ignition::math::Vector3d >::iterator it= CGzModelReader::gzModelBoundingBox.begin();
        for(; it!= CGzModelReader::gzModelBoundingBox.end(); ++it)
        {
            ignition::math::Vector3d bbox = (*it).second;
            std::cout << (*it).first << "=" << bbox.X() << " " << bbox.Y() << " " << bbox.Z() << std::endl << std::flush;
        }
    }
    else if (msg.compare( 0, strlen("instances"),"instances") == 0)
    {
        msg=msg.erase(0,std::string("instances").size());
        bool bWorld;
        std::string world("world");
        auto it = std::search(
                    msg.begin(), msg.end(),
                    world.begin(),   world.end(),
                    [](char ch1, char ch2) { return std::toupper(ch1) == std::toupper(ch2); }
        );
        bWorld= (it != msg.end() );
        bool bRobot;
        std::string robot("robot");
        it = std::search(
                    msg.begin(), msg.end(),
                    robot.begin(),   robot.end(),
                    [](char ch1, char ch2) { return std::toupper(ch1) == std::toupper(ch2); }
        );
        bRobot=(it != msg.end() );

        if(bWorld)
        {
            std::cout << ShapeModel::instances.dumpLocations();
        }
        else if(bRobot)
        {
            std::cout << ShapeModel::instances.dumpLocations(std::bind(&KinematicRing::robotOnlyCoord, ncs[_ncindex], std::placeholders::_1));
        }
        else
            std::cout << ShapeModel::instances.dumpLocations(std::bind(&KinematicRing::robotOnlyCoord, ncs[_ncindex], std::placeholders::_1));

    }
    else if (msg.compare("robots") == 0)
    {
        std::cout << "Robots: \n";
        for(size_t i=0; i< ncs.size(); i++)
            std::cout << "\t" << ncs[i]->robotPrefix() << "\n";
        std::cout << std::flush;
    }
    else if (msg.compare( 0, strlen("shell "), "shell ") == 0)
    {
        msg=msg.erase(0,std::string("shell ").size());
        msg=Globals.trim(msg);
        std::string echo = shell(msg.c_str());
        std::cout << echo << std::flush;
    }


    else if (msg.compare( 0, strlen("robot "), "robot ") == 0)
    {
        msg=msg.erase(0,std::string("robot ").size());
        msg=Globals.trim(msg);

        for(size_t i=0; i< ncs.size(); i++)
        {
            if(ncs[i]->robotPrefix() != msg)
                continue;
            _ncindex=i;
            std::cout << "Commanded Robot Now " << ncs[i]->robotPrefix() << "\n>";
            return 0;
        }
        std::cout << "Robot match not found for " << msg << "\n>"<< std::flush;

    }
    else if (msg.compare("speeds") == 0)
    {
        std::cout << "speed:\n\trobot joint vel=" << ncs[_ncindex]->currentRobotJointSpeed()
                  << "speed:\n\tjgripper    vel=" <<   ncs[_ncindex]->currentGripperJointSpeed()
                  << "\n\tlinear            vel="  <<  ncs[_ncindex]->currentLinearSpeed()
                  << "\n\trotational vel=       "  <<   ncs[_ncindex]->currentAngularSpeed()
                  << "\n" << std::flush;
    }
    else if (msg.compare( 0, strlen("gripper "),"gripper ") == 0)
    {
        msg=Globals.trim(msg.erase(0,std::string("gripper ").size()));
        if(msg == "slower")
        {
            ncs[_ncindex]->setGripperJointSpeeds(ncs[_ncindex]->currentGripperJointSpeed()*.5);
        }
        else if(msg == "faster")
        {
            ncs[_ncindex]->setGripperJointSpeeds(ncs[_ncindex]->currentGripperJointSpeed()*2.);
        }
        else if(msg == "reset")
        {
            ncs[_ncindex]->setGripperJointSpeeds(1.);
        }
    }

    else if (msg.compare("slower") == 0)
    {
        crclApi->slow();
        ncs[_ncindex]->setRobotJointSpeeds(ncs[_ncindex]->currentRobotJointSpeed()*.5);
        ncs[_ncindex]->setLinearSpeeds(ncs[_ncindex]->currentLinearSpeed()*.5);
        ncs[_ncindex]->setRotationalSpeeds(ncs[_ncindex]->currentAngularSpeed()*.5);
        ncs[_ncindex]->setGripperJointSpeeds(ncs[_ncindex]->currentGripperJointSpeed()*.5);
    }
    else if (msg.compare("faster") == 0)
    {
        crclApi->fast();
        ncs[_ncindex]->setRobotJointSpeeds(ncs[_ncindex]->currentRobotJointSpeed()*2.);
        ncs[_ncindex]->setLinearSpeeds(ncs[_ncindex]->currentLinearSpeed()*2.0);
        ncs[_ncindex]->setRotationalSpeeds(ncs[_ncindex]->currentAngularSpeed()*2.0);
    }
    else if (msg.compare("reset") == 0)
    {
        crclApi->medium();
        ncs[_ncindex]->setRobotJointSpeeds(1.);
        ncs[_ncindex]->setGripperJointSpeeds(1.);
        ncs[_ncindex]->setLinearSpeeds(ncs[_ncindex]->base_linearmax()[0]);
        ncs[_ncindex]->setRotationalSpeeds(ncs[_ncindex]->base_rotationmax()[0]);
        geardemo->gzInstances.reset();

    }
    else if (msg.compare( 0, strlen("goto "), "goto ") == 0)
    {
        msg=msg.erase(0,std::string("goto ").size());
        msg=Globals.trim(msg);
        std::transform(msg.begin(), msg.end(), msg.begin(), ::tolower);
        // see if existing shape
        if(msg =="?" || msg.empty())
        {
            // macros
            std::map<std::string, std::vector<std::string>>::iterator it;
            std:: cout << "MACROS" << "\n";
            for(it=ncs[_ncindex]->namedCommand().begin(); it!= ncs[_ncindex]->namedCommand().end(); it++)
            {
                std:: cout << (*it).first << "\n";
            }
            // poses
            std::map<std::string, tf::Pose>::iterator it1;
            std:: cout << "POSES" << "\n";
            for(it1=ncs[_ncindex]->namedPoseMove().begin(); it1!= ncs[_ncindex]->namedPoseMove().end(); it1++)
            {
                std:: cout << (*it1).first << "\n";
            }

            // joint moves
            std::map<std::string, std::vector<double>>::iterator it2;
            std:: cout << "JOINTS" << "\n";
            for(it2=ncs[_ncindex]->namedJointMove().begin(); it2!= ncs[_ncindex]->namedJointMove().end(); it2++)
            {
                std:: cout << (*it2).first << "\n";
            }
        }
        else if(ShapeModel::instances.findInstance(msg)!=NULL)
        {
            crclApi->moveTo(msg);
        }
        else if(ncs[_ncindex]->namedPoseMove().find(msg)!= ncs[_ncindex]->namedPoseMove().end())
        {
            crclApi->moveTo(ncs[_ncindex]->namedPoseMove()[msg]);

        }
        // see if existing joint move name
        else if(ncs[_ncindex]->namedJointMove().find(msg)!= ncs[_ncindex]->namedJointMove().end())
        {
            crclApi->moveJoints(ncs[_ncindex]->allJointNumbers(), ncs[_ncindex]->namedJointMove()[msg]);
        }
        else
        {
            std::cout <<  " Goto error: no joint move or instance named " << msg << " found\n";
            return 0;
        }
    }

    else if (msg.compare("home") == 0)
    {
        crclApi->moveJoints(ncs[_ncindex]->allJointNumbers(), ncs[_ncindex]->namedJointMove()["home"]);
    }
    //    else if (msg.compare("safe") == 0)
    //    {
    //            crclApi->move_joints(ncs[ncindex]->robotKinematics()->allJointNumbers(), ncs[ncindex]->NamedJointMove["safe"]);
    //    }

    else if (msg.compare( 0, strlen("joints "), "joints ") == 0 )
    {
        msg=msg.erase(0,std::string("joints ").size());
        msg=Globals.trim(msg);
        std::vector<std::string> dbls = Globals.split(msg, ',');

        std::vector<double> positions = ConvertV(dbls);
        if (_bDegrees)
            positions = ScaleVector<double>(positions, M_PI / 180.0); //
        crclApi->moveJoints(ncs[_ncindex]->allJointNumbers(), positions);
    }
    else if (msg.compare( 0, strlen("jog "), "jog ") == 0 )
    {
        // jog jnt# incr
        msg=msg.erase(0,std::string("jog ").size());
        msg=Globals.trim(msg);
        char c;
        int jnt=-1,axis=-1;
        // this won't throw
        double amt;

        // Get jog type and amount
        if(sscanf(msg.c_str(), "%c %lf", &c, &amt)!=2)
            return CController::NORMAL;

        // Parse jog type
        jog(c,0.0,jnt,axis);

        // Now loop accepting keyboard input asynchronously
        int doFlag=true;
        //       fflush(stdin);
        nonblock(NB_ENABLE);
        while(doFlag)
        {
            Globals.sleep(25);
            if(kbhit()==0)
            {
                continue;
            }
            c=fgetc(stdin);
            doFlag=jog(c,amt,jnt,axis);

            //crclApi->move_joints(jointnum,positions);
            // wait till curpos != nextpos UNLESS ERROR
        }
        nonblock(NB_DISABLE);
    }
    else if (msg.compare("open") == 0)
    {
        crclApi->openGripper();
    }
    else if (msg.compare("close") == 0)
    {
        crclApi->closeGripper();
    }
    else if (msg.compare("smartclose") == 0)
    {
        crclApi->smartCloseGripper();
    }
    else if (msg.compare(0, strlen("set gripper "),"set gripper ") == 0)
    {
        msg=msg.erase(0,std::string("set gripper ").size());
        msg=Globals.trim(msg);
        double ee = FromStr<double>(msg);
        // End effector should be a percentage, instead use hard position for testing
        //        if(ee<0)
        //            ee=0;
        //        if(ee>1.)
        //            ee=1.;
        // ee is a value 0..1 as a percentage!

        crclApi->setAbsPosGripper(ee);
    }
    else if (msg.compare(0, strlen("force "),"force ") == 0)
    {
        msg=msg.erase(0,std::string("force ").size());
        msg=Globals.trim(msg);
        std::vector<std::string> str_dbls = Globals.split(msg, ',');
        // Fixme: trim the strings.
        if(str_dbls.size() != 2)
        {
            fprintf(stderr,"force needs \"vel,fmax\" comma separaeted pair of values\n");
            return CController::NORMAL;
        }
        for(size_t i=0; i< str_dbls.size(); i++)
        {
            double d;
            if(sscanf(str_dbls[i].c_str(), "%lf", &d)!=1)
            {
                fprintf(stderr,"vel and force must be doubles\n");
                return CController::NORMAL;
            }
        }

        std::vector<double> dbls= ConvertStringVector<double>(str_dbls);
        crclApi->setVelGripper(dbls[0], dbls[1]);
    }
    else if (msg.compare(0, strlen("set dwell "),"set dwell ") == 0)
    {
        // dwell is seconds
        msg=msg.erase(0,std::string("set dwell ").size());
        msg=Globals.trim(msg);
        double d = FromStr<double>(msg);
        crclApi->setDwell(d);
    }
    else if (msg.compare(0, strlen("dwell "),"dwell ") == 0)
    {
        // dwelintl is seconds
        msg=msg.erase(0,std::string("dwell ").size());
        msg=Globals.trim(msg);
        double ee = FromStr<double>(msg);
        crclApi->doDwell(ee);
    }
    else if (msg.compare( 0, strlen("pickup "), "pickup ") == 0 )
    {
        msg=msg.erase(0,std::string("pickup ").size());
        msg=Globals.trim(msg);
        // message should now contain object - can't really detect if exists
        crclApi->pickup(msg);
    }
    else if (msg.compare( 0, strlen("retract "), "retract ") == 0 )
    {
        msg=msg.erase(0,std::string("retract ").size());
        msg=Globals.trim(msg);

        // message should now contain object - FIXME: can't really detect if exists
        crclApi->retract(msg);
    }
    else if (msg.compare( 0, strlen("retract"), "retract") == 0 )
    {
        crclApi->retract();
    }
    else if (msg.compare( 0, strlen("approach "), "approach ") == 0 )
    {
        msg=msg.erase(0,std::string("approach ").size());
        msg=Globals.trim(msg);
        // message should now contain object - can't really detect if exists
        crclApi->approach(msg);
    }
    else if (msg.compare( 0, strlen("grasp "), "grasp ") == 0 )
    {
        msg=msg.erase(0,std::string("grasp ").size());
        msg=Globals.trim(msg);

        // message should now contain object - FIXME: can't really detect if exists
        crclApi->grasp(msg);
    }
    // move x,y,z,r,p,y
    else if (msg.compare( 0, strlen("pmove "), "pmove ") == 0 )
    {
        msg=msg.erase(0,std::string("pmove ").size());
        msg=Globals.trim(msg);
        tf::Pose pose;
        std::vector<std::string> str_dbls = Globals.split(msg, ',');
        std::vector<double> dbls= ConvertStringVector<double>(str_dbls);
        pose = Convert<std::vector<double>, tf::Pose> (dbls);
        crclApi->moveTo(pose);
    }
    else if (msg.compare( 0, strlen("move_to "), "move_to ") == 0 )
    {
        msg=msg.erase(0,std::string("move_to ").size());
        msg=Globals.trim(msg);  // object name
        // message should now contain object - can't really detect if exists
        crclApi->moveTo(msg);
    }

    else if (msg.compare( 0, strlen("ik "), "ik ") == 0 )
    {

        msg=msg.erase(0,std::string("ik ").size());
        msg=Globals.trim(msg);

        // IK of a given object position
        ShapeModel::CShape * shape = ShapeModel::instances.findInstance(msg);
        tf::Pose w_finalpose=shape->_location;
        tf::Pose r_goalpose = ncs[_ncindex]->robotOnlyCoord(w_finalpose);


        JointState joints;
        try{
            // seed
            joints.position=ncs[_ncindex]->namedJointMove()["joints.safe"];
            if(0>ncs[_ncindex]->robotKinematics()->IK(r_goalpose, joints.position))
                throw;
            std::cout << "IK Position             " << RCS::dumpPose(r_goalpose) << "\n";
            std::cout << "IK Joints               " << vectorDump<double>(joints.position).c_str()<< "\n" << std::flush;
        }
        catch (...)
        {
            std::cout << "IK failed to compute" <<  RCS::dumpPoseSimple(r_goalpose)<< "\n";
        }

        std::cout << "\n" << std::flush;
        return CController::NORMAL;
    }

#ifdef GAZEBO
    else if (msg.compare( 0, strlen("where "), "where ") == 0 )
    {
        msg=msg.erase(0,std::string("where ").size());
        msg=Globals.trim(msg);
        ShapeModel::CShape * shape = ShapeModel::instances.findInstance(msg);
        if(shape != NULL)
        {
            std::cout << msg << "=" << RCS::dumpPose(shape->_location) << "\n";
        }
        else
        {
            std::cout << msg << " not found \n" << std::flush;
        }
    }
#endif
    // use robot setup vector variable to command robot via lines of CLI
    else if (msg.compare( 0, strlen("macro "), "macro ") == 0 )
    {
        // Get name of macro
        msg=msg.erase(0,std::string("macro ").size());
        msg=Globals.trim(msg);

        // Find corresponding macro for robot
        std::map<std::string, std::vector<std::string>>::iterator it;
        if((it=ncs[_ncindex]->namedCommand().find(msg))== ncs[_ncindex]->namedCommand().end())
            return CController::NORMAL;

        // Execute the command strings in the macro
        for(size_t k=0; k< (*it).second.size(); k++)
        {
            interpretLine((*it).second.at(k));
        }
    }
    // msg.compare("record") == 0
    else if (msg.compare( 0, strlen("record "), "record ") == 0 )
    {
        msg=msg.erase(0,std::string("record ").size());
        msg=Globals.trim(msg); // now have name of where?
        std::string filename = getexefolder() + "recorded_poses.txt";
        std::fstream recordFile(filename, std::fstream::out | std::fstream::app);
        if (recordFile.is_open())
        {
            recordFile << ncs[_ncindex]->robotPrefix() <<  msg << "\n";
            std::vector<double> joints = ncs[_ncindex]->_status.robotjoints.position;
            recordFile << "\tJoints   =" << vectorDump(joints, ",", "%6.3f") << "\n" ;
            tf::Pose r_curpose;
            ncs[_ncindex]->robotKinematics()->FK(joints, r_curpose);
            recordFile << "\tPose   =" << RCS::dumpPoseSimple(r_curpose) << "\n";
            recordFile.close();
        }
    }
    else if (msg.compare( 0, strlen("IK "), "IK ") == 0 )
    {
        msg=msg.erase(0,std::string("IK ").size());
        msg=Globals.trim(msg); // now have name of where?

        std::vector<std::string> str_dbls = Globals.split(msg, ',');
        std::vector<double> dbls= ConvertStringVector<double>(str_dbls);
        tf::Pose pose = Convert<std::vector<double>, tf::Pose> (dbls);

        std::vector<double> joints;
        ncs[_ncindex]->robotKinematics()->IK(pose, joints);
        std::cout << "IK Joints   =" << vectorDump<double>(joints).c_str()<< "\n" << std::flush;
    }
    else if (msg.compare( 0, strlen("FK "), "FK ") == 0 )
    {
        msg=msg.erase(0,std::string("FK ").size());
        msg=Globals.trim(msg); // now have name of where?

        JointState joints;
        std::vector<std::string> str_dbls = Globals.split(msg, ',');
        joints.position= ConvertStringVector<double>(str_dbls);
        // joint names - assume filled in by kinsolver init

        tf::Pose r_pose;
        ncs[_ncindex]->robotKinematics()->FK(joints.position, r_pose);
        std::cout << "FK Pose   =" << RCS::dumpPoseSimple(r_pose) << "\n";
    }
    else if (msg.compare( 0, strlen("where"), "where") == 0 )
    {
        std::vector<double> joints = ncs[_ncindex]->_status.robotjoints.position;
        std::cout << "Status Joints   =" << vectorDump(joints, ",", "%6.3f") << "\n" ;
        tf::Pose r_curpose;
        ncs[_ncindex]->robotKinematics()->FK(joints, r_curpose);
        std::cout << "FK Robot Pose   =" << RCS::dumpPoseSimple(r_curpose) << "\n";
        tf::Pose w_curpose = ncs[_ncindex]->worldCoord(r_curpose);
        std::cout << "FK World Pose   =" << RCS::dumpPoseSimple(w_curpose) << "\n";
        tf::Pose w_basepose = ncs[_ncindex]->addBaseTransform(r_curpose);
        std::cout << "FK World Base   =" << RCS::dumpPoseSimple(w_basepose) << "\n";

        // seed joints
        joints=subset(ncs[_ncindex]->_status.robotjoints.position, ncs[_ncindex]->robotKinematics()->numJoints());
        ncs[_ncindex]->robotKinematics()->IK(r_curpose, joints);
        std::cout << "IK Robot Joints =" << vectorDump(joints, ",", "%6.3f") << "\n"<< std::flush;

    }
    else if ((sit=std::find(_robotNames.begin(), _robotNames.end(), msg)) != _robotNames.end())
    {
        _ncindex = sit- _robotNames.begin();
    }
    else if (msg.compare("help") == 0)
    {
        std::cout << "> help\t gives this output for joint or Cartesian moves.\n" ;
        std::cout << "> quit\t stops the controller and all its threads.\n" ;
        std::cout << "> slow\t slow rate of motion for joint or Cartesian moves.\n" ;
        std::cout << "> medium\t medium rate of motion for joint or Cartesian moves.\n" ;
        std::cout << "> fast\t fast rate of motion for joint or Cartesian moves.\n" ;
        std::cout << "> reset\t resets the rate of motion to medium.\n" ;
        std::cout << "> instances\t list of gear instances and position.\n" ;
        std::cout << "> slower\t slower rate of motion for joint or Cartesian moves (time 1/2.)\n" ;
        std::cout << "> faster\t faster rate of motion for joint or Cartesian moves (times 2). \n" ;
        std::cout << "> jog [x|y|z|# amt\t jog in xyz or joint # a certain amount. Follow by + or - for direction.\n" ;
        std::cout << "> set gripper amt\t moves the gipper to an absolute position but can't be 1 or 0.\n" ;
        std::cout << "> open\t moves the gipper to the open position as specified in the config.ini file\n" ;
        std::cout << "> close\t moves the gipper to the closed position as specified in the config.ini file\n" ;
        std::cout << "> pickup obj\t given a gear instance name will pickup the gear.\n" ;
        std::cout << "> retract\t moves away in positive z direction from current location.\n" ;
        std::cout << "> approach obj\t moves toward the gear instance given as a full name offset by an approach distance.\n" ;
        std::cout << "> where\t provides the current joint and Cartesian pose of the robot.\n" ;
        std::cout << "> record name\t records the current position to the filw with name header.\n" ;
        std::cout << "> goto name\t name is a stored names in config.ini file describing a set of joint position which the robot will move to..\n" ;
        std::cout << "> move_to obj\t moves to the gear instance given as a full name with a little offset from centroid of object.\n" ;
        std::cout << "> move x,y,z,r,p,y\t moves robot to the given pose given as xyz and roll, pitch, yaw.\n" ;
    }
    else
    {
        if(!msg.empty())
            std::cout << msg  << " :command not found \n" << std::flush;
        //std::cout << "> " << std::flush;
        return CController::NOOP;
    }
    //std::cout << "> " << std::flush;

    return CController::NOOP;
}
