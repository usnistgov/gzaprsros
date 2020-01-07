

#include <gomotion_plugin/gomotion_plugin.h>
#include <inifile.h>
#include <go.h>
#include <genserkins.h>
#include <cmath>
#include <stdlib.h>
#include <sys/stat.h>
#include "gomotion_plugin/Debug.h"


int go_lib_version;

template<typename ... Args>
//static std::string strformat( const std::string& format, Args ... args )
static std::string strformat( const char * format, Args ... args )
{
    size_t size = snprintf( nullptr, 0, format, args ... ) + 1; // Extra space for '\0'
    std::unique_ptr<char[]> buf( new char[ size ] );
    snprintf( buf.get(), size, format, args ... );
    return std::string( buf.get(), buf.get() + size - 1 ); // We don't want the '\0' inside
}


using namespace RCS;

/*!
 * \brief Convert urdf::Vector into an tf vector.
 * \param v is a urdf::Vector3t.
 * \return  tf::Vector3 vector.
 */

static tf::Vector3 Convert (urdf::Vector3 v) {
    return tf::Vector3(v.x, v.y, v.z);
}

static tf::Pose Convert(urdf::Pose pose) {
    // http://answers.ros.org/question/193286/some-precise-definition-or-urdfs-originrpy-attribute/
    tf::Quaternion q(pose.rotation.w, pose.rotation.x, pose.rotation.y, pose.rotation.z);
    return tf::Pose (q, tf::Vector3(pose.position.x, pose.position.y, pose.position.z));
}

static urdf::Vector3 Convert (tf::Vector3 v) {
    return urdf::Vector3(v.getX(), v.getY(), v.getZ());
}

static urdf::Pose Convert(tf::Pose pose) {
    // http://answers.ros.org/question/193286/some-precise-definition-or-urdfs-originrpy-attribute/
    urdf::Pose p;
    p.rotation=urdf::Rotation(pose.getRotation().getX(), pose.getRotation().getY(),pose.getRotation().getZ(),pose.getRotation().getW());
    p.position=urdf::Vector3(pose.getOrigin().getX(),pose.getOrigin().getY(),pose.getOrigin().getZ());
    return p;
}



int
gomotion_plugin::ini_load(char * inifile_name,
                          double * m_per_length_units,
                          double * rad_per_angle_units,
                          go_pose * home,
                          int * link_number,
                          go_link * link_params,
                          go_real * jhome,
                          char * kin_name)
{
    FILE * fp;
    const char * inistring;
    char * servo_string;
    int link;
    double d1, d2, d3, d4, d5, d6, d7, d8, d9;
    go_rpy rpy;

    if (NULL == (fp = fopen(inifile_name, "r"))) return 1;

    inistring = ini_find(fp, "LENGTH_UNITS_PER_M", "GOMOTION");
    if (NULL == inistring) {
        fprintf(stderr, "[GOMOTION] LENGTH_UNITS_PER_M not found, using 1\n");
    } else if (1 != sscanf(inistring, "%lf", &d1)) {
#undef CLOSE_AND_RETURN
#define CLOSE_AND_RETURN(ret)			\
    fclose(fp);					\
    return (ret)
        CLOSE_AND_RETURN(1);
        fprintf(stderr, "bad entry: [GOMOTION] LENGTH_UNITS_PER_M = %s\n", inistring);
        CLOSE_AND_RETURN(1);
    } else if (d1 <= 0.0) {
        fprintf(stderr, "invalid entry: [GOMOTION] LENGTH_UNITS_PER_M = %s must be positive\n", inistring);
        CLOSE_AND_RETURN(1);
    } else {
        *m_per_length_units = 1.0 / d1;
    }

    inistring = ini_find(fp, "ANGLE_UNITS_PER_RAD", "GOMOTION");
    if (NULL == inistring) {
        fprintf(stderr, "[GOMOTION] ANGLE_UNITS_PER_RAD not found, using 1\n");
    } else if (1 != sscanf(inistring, "%lf", &d1)) {
#undef CLOSE_AND_RETURN
#define CLOSE_AND_RETURN(ret)			\
    fclose(fp);					\
    return (ret)
        CLOSE_AND_RETURN(1);
        fprintf(stderr, "bad entry: [GOMOTION] ANGLE_UNITS_PER_RAD = %s\n", inistring);
        CLOSE_AND_RETURN(1);
    } else if (d1 <= 0.0) {
        fprintf(stderr, "invalid entry: [GOMOTION] ANGLE_UNITS_PER_RAD = %s must be positive\n", inistring);
        CLOSE_AND_RETURN(1);
    } else {
        *rad_per_angle_units = 1.0 / d1;
    }

    /* if kin_name is empty, set it from .ini file */
    if (0 == kin_name[0]) {
        inistring = ini_find(fp, "KINEMATICS", "TRAJ");
        if (NULL == inistring) {
            fprintf(stderr, "[TRAJ] KINEMATICS not found\n");
            CLOSE_AND_RETURN(1);
        }
        strncpy(kin_name, inistring, GO_KIN_NAME_LEN);
    } else {
        fprintf(stderr, "overriding ini file: [TRAJ] KINEMATICS = %s\n", kin_name);
    }

    inistring = ini_find(fp, "HOME", "TRAJ");
    if (NULL == inistring) {
        fprintf(stderr, "[TRAJ] HOME not found\n");
        CLOSE_AND_RETURN(1);
    }
    if (6 != sscanf(inistring, "%lf %lf %lf %lf %lf %lf",
                    &d1, &d2, &d3, &d4, &d5, &d6)) {
        fprintf(stderr, "invalid entry: [TRAJ] HOME\n");
        CLOSE_AND_RETURN(1);
    }
    home->tran.x = (go_real) (*m_per_length_units * d1);
    home->tran.y = (go_real) (*m_per_length_units * d2);
    home->tran.z = (go_real) (*m_per_length_units * d3);
    rpy.r = (go_real) (*rad_per_angle_units * d4);
    rpy.p = (go_real) (*rad_per_angle_units * d5);
    rpy.y = (go_real) (*rad_per_angle_units * d6);
    go_rpy_quat_convert(&rpy, &home->rot);

    servo_string = (char *) malloc(sizeof("SERVO_" + DIGITS_IN(link)));
    if (NULL == servo_string) {
        fprintf(stderr, "can't allocate space for SERVO_X section\n");
        CLOSE_AND_RETURN(1);
    }

    for (link = 0; ; link++) {
        sprintf(servo_string, "SERVO_%d", link + 1);

        inistring = ini_find(fp, "QUANTITY", servo_string);
        if (NULL == inistring) {
            /* no "QUANTITY" in this section, or no section, so we're done */
            break;
        } else {
            if (ini_match(inistring, "ANGLE")) {
                link_params[link].quantity = GO_QUANTITY_ANGLE;
            } else if (ini_match(inistring, "LENGTH")) {
                link_params[link].quantity = GO_QUANTITY_LENGTH;
            } else {
                fprintf(stderr, "bad entry: [%s] QUANTITY = %s\n", servo_string, inistring);
                CLOSE_AND_RETURN(1);
            }
        }
        const char * namestring = ini_find(fp, "NAME", servo_string);
        if (NULL != namestring)
            jointNames.push_back(namestring);
        else
            fprintf(stderr, "bad name entry: [%s] NAME = %s\n", servo_string, inistring);


        go_body_init(&link_params[link].body);

        inistring = ini_find(fp, "MASS", servo_string);
        if (NULL != inistring) {
            if (1 == sscanf(inistring, "%lf", &d1)) {
                link_params[link].body.mass = d1;
            } else {
                fprintf(stderr, "bad entry: [%s] MASS = %s\n", servo_string, inistring);
                CLOSE_AND_RETURN(1);
            }
        }

#define TGL(x) (go_real) ((x) * (*m_per_length_units))
        inistring = ini_find(fp, "INERTIA", servo_string);
        if (NULL != inistring) {
            if (9 == sscanf(inistring, "%lf %lf %lf %lf %lf %lf %lf %lf %lf", &d1, &d2, &d3, &d4, &d5, &d6, &d7, &d8, &d9)) {
                link_params[link].body.inertia[0][0] = TGL(TGL(d1));
                link_params[link].body.inertia[0][1] = TGL(TGL(d2));
                link_params[link].body.inertia[0][2] = TGL(TGL(d3));
                link_params[link].body.inertia[1][0] = TGL(TGL(d4));
                link_params[link].body.inertia[1][1] = TGL(TGL(d5));
                link_params[link].body.inertia[1][2] = TGL(TGL(d6));
                link_params[link].body.inertia[2][0] = TGL(TGL(d7));
                link_params[link].body.inertia[2][1] = TGL(TGL(d8));
                link_params[link].body.inertia[2][2] = TGL(TGL(d9));
            } else {
                fprintf(stderr, "bad entry: [%s] INERTIA = %s\n", servo_string, inistring);
                CLOSE_AND_RETURN(1);
            }
        }
#undef TGL

        inistring = ini_find(fp, "HOME", servo_string);
        if (NULL == inistring) {
            /* no "HOME" in this section, or no section, so we're done */
            break;
        } else {
            if (1 == sscanf(inistring, "%lf", &d1)) {
                jhome[link] = link_params[link].quantity == GO_QUANTITY_ANGLE ? (go_real) (*rad_per_angle_units * d1) : (go_real) (*m_per_length_units * d1);
            } else {
                fprintf(stderr, "bad entry: [%s] HOME = %s\n", servo_string, inistring);
                CLOSE_AND_RETURN(1);
            }
        }

        if (NULL != (inistring = ini_find(fp, "DH_PARAMETERS", servo_string))) {
            if (4 == sscanf(inistring, "%lf %lf %lf %lf", &d1, &d2, &d3, &d4)) {
                go_dh dh;
                dh.a = (go_real) (*m_per_length_units * d1);
                dh.alpha = (go_real) (*rad_per_angle_units * d2);
                dh.d = (go_real) (*m_per_length_units * d3);
                dh.theta = (go_real) (*rad_per_angle_units * d4);
                link_params[link].u.dh = dh;
                link_params[link].type = GO_LINK_DH;
            } else {
                fprintf(stderr, "bad entry: [%s] DH = %s\n", servo_string, inistring);
                CLOSE_AND_RETURN(1);
            }
        } else if (NULL != (inistring = ini_find(fp, "PP_PARAMETERS", servo_string))) {
            if (6 == sscanf(inistring, "%lf %lf %lf %lf %lf %lf", &d1, &d2, &d3, &d4, &d5, &d6)) {
                go_rpy rpy;
                link_params[link].u.pp.pose.tran.x = (go_real) (*m_per_length_units * d1);
                link_params[link].u.pp.pose.tran.y = (go_real) (*m_per_length_units * d2);
                link_params[link].u.pp.pose.tran.z = (go_real) (*m_per_length_units * d3);
                rpy.r = (go_real) (*rad_per_angle_units * d4);
                rpy.p = (go_real) (*rad_per_angle_units * d5);
                rpy.y = (go_real) (*rad_per_angle_units * d6);
                go_rpy_quat_convert(&rpy, &link_params[link].u.pp.pose.rot);
                link_params[link].type = GO_LINK_PP;
            } else {
                fprintf(stderr, "bad entry: [%s] PP = %s\n", servo_string, inistring);
                CLOSE_AND_RETURN(1);
            }
        } else if (NULL != (inistring = ini_find(fp, "URDF_PARAMETERS", servo_string))) {
            if (9 == sscanf(inistring, "%lf %lf %lf %lf %lf %lf %lf %lf %lf", &d1, &d2, &d3, &d4, &d5, &d6, &d7, &d8, &d9)) {
                go_rpy rpy;
                go_cart cart;
                link_params[link].u.urdf.pose.tran.x = (go_real) (*m_per_length_units * d1);
                link_params[link].u.urdf.pose.tran.y = (go_real) (*m_per_length_units * d2);
                link_params[link].u.urdf.pose.tran.z = (go_real) (*m_per_length_units * d3);
                rpy.r = (go_real) (*rad_per_angle_units * d4);
                rpy.p = (go_real) (*rad_per_angle_units * d5);
                rpy.y = (go_real) (*rad_per_angle_units * d6);
                go_rpy_quat_convert(&rpy, &link_params[link].u.urdf.pose.rot);
                cart.x = (go_real) (*m_per_length_units * d7);
                cart.y = (go_real) (*m_per_length_units * d8);
                cart.z = (go_real) (*m_per_length_units * d9);
                if (GO_RESULT_OK != go_cart_unit(&cart, &cart)) {
                    fprintf(stderr, "bad entry: [%s] URDF = %s\n", servo_string, inistring);
                    CLOSE_AND_RETURN(1);
                }
                link_params[link].u.urdf.axis = cart;
                link_params[link].type = GO_LINK_URDF;
            } else {
                fprintf(stderr, "bad entry: [%s] PP = %s\n", servo_string, inistring);
                CLOSE_AND_RETURN(1);
            }
        } else if (NULL != (inistring = ini_find(fp, "PK_PARAMETERS", servo_string))) {
            if (6 == sscanf(inistring, "%lf %lf %lf %lf %lf %lf", &d1, &d2, &d3, &d4, &d5, &d6)) {
                link_params[link].u.pk.base.x = (go_real) (*m_per_length_units * d1);
                link_params[link].u.pk.base.y = (go_real) (*m_per_length_units * d2);
                link_params[link].u.pk.base.z = (go_real) (*m_per_length_units * d3);
                link_params[link].u.pk.platform.x = (go_real) (*m_per_length_units * d4);
                link_params[link].u.pk.platform.y = (go_real) (*m_per_length_units * d5);
                link_params[link].u.pk.platform.z = (go_real) (*m_per_length_units * d6);
                link_params[link].type = GO_LINK_PK;
            } else {
                fprintf(stderr, "bad entry: [%s] PK = %s\n", servo_string, inistring);
                CLOSE_AND_RETURN(1);
            }
        } else {
            /* no "DH,PP,PK_PARAMETERS" in this section, so we're done */
            break;
        }
    } /* for (link) */
    *link_number = link;

    CLOSE_AND_RETURN(0);
}


static void print_params(go_link *link_params, int link_number)
{
    go_rpy rpy;
    int t;

    for (t = 0; t < link_number; t++) {
        if (GO_LINK_DH == link_params[t].type) {
            printf("%d: %.3f %.3f %.3f %.3f\n", t+1,
                   link_params[t].u.dh.a,
                   link_params[t].u.dh.alpha,
                   link_params[t].u.dh.d,
                   link_params[t].u.dh.theta);
        } else if (GO_LINK_PP == link_params[t].type) {
            go_quat_rpy_convert(&link_params[t].u.pp.pose.rot, &rpy);
            printf("%d: %.3f %.3f %.3f / %.3f %.3f %.3f\n", t+1,
                   link_params[t].u.urdf.pose.tran.x,
                   link_params[t].u.urdf.pose.tran.y,
                   link_params[t].u.urdf.pose.tran.z,
                   rpy.r, rpy.p, rpy.y);
        } else if (GO_LINK_URDF == link_params[t].type) {
            go_quat_rpy_convert(&link_params[t].u.urdf.pose.rot, &rpy);
            printf("%d: %.3f %.3f %.3f / %.3f %.3f %.3f / %.3f %.3f %.3f\n", t+1,
                   link_params[t].u.urdf.pose.tran.x,
                   link_params[t].u.urdf.pose.tran.y,
                   link_params[t].u.urdf.pose.tran.z,
                   rpy.r, rpy.p, rpy.y,
                   link_params[t].u.urdf.axis.x,
                   link_params[t].u.urdf.axis.y,
                   link_params[t].u.urdf.axis.z);
        } else {
            printf("unknown\n");
        }
    }
}


////////////////////////////////////////////////////////////////////////////////
gomotion_plugin::gomotion_plugin() : CSerialLinkRobot((ISerialLinkRobot*) this)
{
    bDebug=false;
    bHandleExceptions=false;
    debugStream(std::cout);
    robot_name="fanuc lrmate200id";
    _nJoints=6;



}
////////////////////////////////////////////////////////////////////////////////
int gomotion_plugin::debugStream(std::ostream& o)
{
    out.copyfmt(o); //1
    out.clear(o.rdstate()); //2
    out.basic_ios<char>::rdbuf(o.rdbuf());
    return 0;
}
////////////////////////////////////////////////////////////////////////////////
size_t gomotion_plugin::numJoints()
{
    return _nJoints;
}
////////////////////////////////////////////////////////////////////////////////
int gomotion_plugin::debug(bool flag)
{

    bDebug=flag;
    return 0;
}
////////////////////////////////////////////////////////////////////////////////
int gomotion_plugin::init()
{
    errmsg.clear();
    int hr=0;

#if 0
    if (GO_RESULT_OK != go_init()) {
        errmsg= "gomotion_plugin can't init gomotion\n";
        return -1;
    }
#endif
    if (GO_RESULT_OK != genser_kin_init(&kins)) {
        errmsg="gomotion_plugin can't init general serial kinematics\n";
        return -1;
    }

    if(!_inifilename.empty())
    {
        struct stat buffer;
        if( (stat (_inifilename.c_str(), &buffer) != 0))
        {
            errmsg= "gomotion_plugin bad inifile \n";
            hr= Bad_Parameter;
        }
        else
        {
            ini_load((char *) _inifilename.c_str(),
                     &m_per_length_units,
                     &rad_per_angle_units,
                     &home_position,
                     &link_number,
                     link_params,
                     home_joint,
                     kin_name);
            this->_nJoints=this->link_number;

        }


    }
    else
    {
        errmsg= "gomotion_plugin No inifile configured by set\n";
        hr= Bad_Parameter;
    }
    if (GO_RESULT_OK != genser_kin_set_parameters(&kins, link_params, link_number)) {
        errmsg = "gomotion_plugin can't set kinematics parameters\n";
        return -1;
    }

    return hr;

}
////////////////////////////////////////////////////////////////////////////////
int gomotion_plugin::FK(std::vector<double> joints, tf::Pose &pose)
{
    errmsg.clear();
    go_pose gopose;

    //go_result genser_kin_fwd(void * kins, const go_real *joints,  go_pose * pos)
    if (GO_RESULT_OK != genser_kin_fwd(&kins, &joints[0], &gopose)) {
        errmsg= "gomotion_plugin Can't run general serial forward kinematics\n";
        return -1;
    }

    pose=tf::Pose(tf::Quaternion(gopose.rot.x,gopose.rot.y,gopose.rot.z,gopose.rot.s ),
                  tf::Vector3(gopose.tran.x, gopose.tran.y, gopose.tran.z) );


    return Kinematics_Ok;
}


//////////////////////////////////////////////////////////////////////////////
int gomotion_plugin::IK(tf::Pose pose,
                        std::vector<double>& joints)
{
    // Clear error message
    errmsg.clear();

    size_t n=joints.size();
    // Last joints are an estimate for next joints
    //    joints.resize(7,0.0);
    std::vector<double> cpyjoints(joints);
    for(size_t i=n; i< GENSER_MAX_JOINTS; i++)
        cpyjoints.push_back(0.0);

    std::vector<std::string> jointNames;

    go_pose gopose;
    gopose.tran.x=pose.getOrigin().x();
    gopose.tran.y=pose.getOrigin().y();
    gopose.tran.z=pose.getOrigin().z();
    gopose.rot.x=pose.getRotation().x();
    gopose.rot.y=pose.getRotation().y();
    gopose.rot.z=pose.getRotation().z();
    gopose.rot.s=pose.getRotation().w();

    int res =  genser_kin_inv(&kins, &gopose, &cpyjoints[0]);
    if (GO_RESULT_OK != res) {
        std::string ss;
        ss+= strformat( "Can't run general serial inverse kinematics %s\n", go_result_to_string(res));
        ss+= strformat( "tf:\n%s", DumpTfPose(pose).c_str());
        ss+= strformat( "go:\n%s", DumpGoPose(gopose).c_str());
        errmsg=ss;
        return res;
    }
    if(bDebug)
    {
        out<< "IK Joints:\n";
        out<<  strformat("%s", DumpJoints(joints).c_str());
    }
    joints.clear();
    joints.insert(joints.begin(), cpyjoints.begin(), cpyjoints.begin()+n);
    return Kinematics_Ok;
}
////////////////////////////////////////////////////////////////////////////////
std::string gomotion_plugin::get(std::string param)
{
    const char* ws = " \t\n\r";

    param.erase(param.find_last_not_of(ws) + 1);
    param.erase(0, param.find_first_not_of(ws));
    std::transform(param.begin(), param.end(),param.begin(), ::toupper);
    if(param == "ERROR")
    {
        return errmsg;
    }
    else if(param == "HELP")
    {
        std::stringstream ss;
        ss << "gomotion_plugin kinematics solver using genserkins kinematic solver for fanuc 200id\n";
        ss << "Parameters Get:\n";
        ss << "\thelp\n";
        ss << "\terror\n";
        ss << "\turdf\n";
        ss << "\turdffile\n";
        ss << "\tbaselink\n";
        ss << "\ttiplink\n";
        ss << "Parameters Set:\n";
        ss << "\tdebug\n";
        ss << "\tHandleExceptions\n";
        ss << "\turdf\n";
        ss << "\turdffile\n";
        ss << "\tbaselink\n";
        ss << "\ttiplink\n";
        return ss.str();
    }
    else if(param == "URDF")
    {
        return _urdf;
    }
    else if(param == "URDFFILE")
    {
        return _urdffile;
    }
    else if(param == "BASELINK")
    {
        return _baselink;
    }
    else if(param == "TIPLINK")
    {
        return _tiplink;
    }
    else if(param == "INIFILE")
    {
        return _inifilename;
    }
    else if(param == "NUMJOINTS")
    {
        return std::to_string(_nJoints);
    }
    else if(param == "ROBOTNAME")
    {
        return robot_name;
    }
    else if(param == "ALL")
    {
        std::stringstream ss;
        ss << "ROBOTNAME="<< robot_name << std::endl;
        ss << "NUMJOINTS="<< std::to_string(_nJoints)<< std::endl;
        ss << "JOINTS=";
        for(size_t i=0; i< jointNames.size(); i++)
        {
            if(i>0)
                ss<< ",";
            ss << jointNames[i];
        }
        ss<< std::endl;
        ss << "URDFFILE="<< _urdffile << std::endl;
        ss << "INIFILE="<< _inifilename << std::endl;
        ss << "TIPLINK="<< _tiplink << std::endl;
        ss << "BASELINK="<< _baselink << std::endl;
        ss << "ERROR="<< errmsg << std::endl;

        return ss.str();
    }
    return std::string("No get for parameter ") + param;
}


////////////////////////////////////////////////////////////////////////////////
std::string gomotion_plugin::set(std::string param,  std::string value)
{
    const char* ws = " \t\n\r";
    errmsg.clear();

    param.erase(param.find_last_not_of(ws) + 1);
    param.erase(0, param.find_first_not_of(ws));
    std::transform(param.begin(), param.end(),param.begin(), ::toupper);
    if(param == "DEBUG")
    {
        bDebug = std::stoi(value);
    }
    else if(param == "HANDLEEXCEPTIONS")
    {
        bHandleExceptions = std::stoi(value);
    }
    else if(param == "URDF")
    {
        _urdf=value;
    }
    else if(param == "URDFFILE")
    {
        _urdffile=value;

    }
    else if(param == "BASELINK")
    {
        _baselink=value;
    }
    else if(param == "TIPLINK")
    {
        _tiplink=value;
    }
    else if(param == "INIFILE")
    {
        _inifilename=value;
    }
    else if(param == "NUMJOINTS")
    {
        _nJoints=std::stoi(value);
    }
    else
    {
        errmsg=std::string("No match for ") + param;
    }
    return errmsg;
}


////////////////////////////////////////////////////////////////////////////////
std::string gomotion_plugin::set(std::string param,  void * value)
{
    return std::string("No match for ") + param;
}
