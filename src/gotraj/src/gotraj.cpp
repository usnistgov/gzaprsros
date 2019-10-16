

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

#include "gotraj/gotraj.h"
#include "gotraj/gotypes.h"
#include "gotraj/gointerp.h"
#include "gotraj/gotraj_.h"
#include "gotraj/gomotion.h"

#include "gotraj/gomath.h"
#include <stdlib.h>

#define _E(str) #str,__FILE__,__LINE__
#ifndef Deg2Rad
#define Deg2Rad(Ang)    ( (double) ( Ang * M_PI / 180.0 ) )
#define Rad2Deg(Ang)    ( (double) ( Ang * 180.0 / M_PI ) )
#define MM2Meter(d)     ( (double) ( d / 1000.00 ) )
#define Meter2MM(d)     ( (double) ( d * 1000.00 ) )
#endif
namespace gomotion {
    
 

    inline std::string DumpPoseSimple(tf::Pose pose) {
        std::stringstream s;

        s << boost::format("%7.4f") % pose.getOrigin().x() << ":" <<
                boost::format("%7.4f") % pose.getOrigin().y() << ":" <<
                boost::format("%7.4f") % pose.getOrigin().z() << "|";
        tf::Quaternion q = pose.getRotation();
        s << boost::format("%7.4f") % q.x() << ":" <<
                boost::format("%7.4f") % q.y() << ":" <<
                boost::format("%7.4f") % q.z() << ":" <<
                boost::format("%7.4f") % q.w();
        return s.str();
    }
   
    
    // the class that throws, if an HRESULT failed

    class E {
        std::string error_str;
        std::string file_name;
        std::string line_number;

        std::string ExtractFilename(const std::string& path) {
            return path.substr(path.find_last_of('/') + 1);
        }

    public:

        E(char* error, char* file_name, int line_number) {
            char buffer[128];
            error_str = error;
            error_str += "@";
            this->file_name = file_name;
            error_str += ExtractFilename(this->file_name);
            sprintf(buffer, ":%d", line_number);
            this->line_number = buffer;
            error_str += this->line_number;
        }

        void operator=(go_result hr) {
            if (hr != GO_RESULT_OK)
                throw std::runtime_error(error_str.c_str());
        }
    };
#define EF(X) E(#X,__FILE__,__LINE__) = X


    struct go_motion_interface {
        int _type;
        double _deltat;
        go_motion_spec _gms;
        std::vector<go_motion_spec> _space;
        go_motion_queue _gmq;
        go_position _position;
        size_t _queuesize;
        static size_t _id;
    };
    size_t go_motion_interface::_id = 0;

    static go_pose ConvertTfPose(tf::Pose pose) {
        tf::Quaternion q = pose.getRotation();
        return go_pose_this(pose.getOrigin().x(), pose.getOrigin().y(), pose.getOrigin().z(),
                q.w(), q.x(), q.y(), q.z());

    }

    static tf::Pose ConvertGoPose(go_pose p) {
        return tf::Pose(tf::Quaternion(p.rot.x, p.rot.y, p.rot.z, p.rot.s),
                tf::Vector3(p.tran.x, p.tran.y, p.tran.z));
    }

    GoTraj::GoTraj() {
        pgm = boost::shared_ptr<go_motion_interface>(new go_motion_interface());
    }

    int GoTraj::Init(JointState here, float cycletime) {
        try {
            pgm->_gmq.reset();
            pgm->_queuesize = 100;
            pgm->_deltat = cycletime;
            pgm->_space.resize(pgm->_queuesize);
            if (GO_RESULT_OK != pgm->_gmq.init(&pgm->_space[0], pgm->_queuesize, pgm->_deltat)) {
                return 1;
            }
            EF(pgm->_gmq.set_type(GO_MOTION_JOINT));
            go_position position;
            num_joints = here.position.size();
            EF(pgm->_gmq.set_joint_number(num_joints));
            std::copy(here.position.begin(), here.position.end(), position.u.joint);
            EF(pgm->_gmq.set_here(&position));
        } catch (std::runtime_error & e) {
            std::cout << "GoTraj::Init Exception" << e.what() << "\n";
        }
        return GO_RESULT_OK;
    }

    int GoTraj::InitPose(tf::Pose here, tf::Pose there, 
            gomotion::GoTrajParams tparams,
            gomotion::GoTrajParams rparams) {
        return InitPose(here, there, -1,
             tparams,
            rparams);

    }        
    int GoTraj::InitPose(tf::Pose here, tf::Pose there, double seconds,
            gomotion::GoTrajParams tparams,
            gomotion::GoTrajParams rparams) {
        go_position position;
        try {
            //Init(deltat);
            if (pgm->_gmq.get_type() != GO_MOTION_WORLD) {
                EF(pgm->_gmq.reset());
                EF(pgm->_gmq.set_type(GO_MOTION_WORLD));
                position.u.pose = ConvertTfPose(here);
                EF(pgm->_gmq.set_here(&position)); // position copied	

            }
            EF(pgm->_gms.init());
            EF(pgm->_gms.set_type(GO_MOTION_LINEAR));
            EF(pgm->_gms.set_id(1));
            if (seconds >= 0) {
                EF(pgm->_gms.set_time(seconds));
            } else {
                EF(pgm->_gms.set_tpar(tparams.vel, tparams.acc, tparams.jerk)); // set translation parameters
                EF(pgm->_gms.set_rpar(1.0, 10.0, 100.0)); // set rotation parameters
            }
            position.u.pose = ConvertTfPose(there);
            AppendPose(there);

        } catch (std::runtime_error & e) {
            std::cout << "GoTraj::InitPose Exception" << e.what() << "\n";
            throw;
        }
        return GO_RESULT_OK;
    }

    void GoTraj::AppendPose(tf::Pose where) {
        go_position position;
        try {
            if (pgm->_gms.get_type() != GO_MOTION_LINEAR)
                throw std::runtime_error("NextPose when not setup for GO_MOTION_LINEAR");
            // Use pgm->_gms again
            position.u.pose = ConvertTfPose(where);
            EF(pgm->_gms.set_end_position(&position));
            EF(pgm->_gmq.append(&(pgm->_gms)));
        } catch (std::runtime_error & e) {
            std::cout << "GoTraj::InitPose Exception" << e.what() << "\n";
            throw;
        }
    }

    tf::Pose GoTraj::NextPose() {
        go_position position;
        try {
            if (pgm->_gms.get_type() != GO_MOTION_LINEAR)
                throw std::runtime_error("NextPose when not setup for GO_MOTION_LINEAR");
            EF(pgm->_gmq.interp(&position));
        } catch (std::exception &e) {
            std::cout << "GoTraj::NextPose Exception" << e.what() << "\n";
            throw e;
        }
        return ConvertGoPose(position.u.pose);
    }

    int GoTraj::InitJoints(JointState here,
            JointState there,
            std::vector<gomotion::GoTrajParams> jparams,
            bool bCoordinated) {
        return InitJoints(here,
                there,
                -1,
                jparams,
                bCoordinated);

    }
#if 0
    int GoTraj::InitPose(tf::Pose here, tf::Pose there,
            double seconds) {
                return InitPose(here, there, seconds,
             gomotion::GoTrajParams(),
            gomotion::GoTrajParams());

    }
    int GoTraj::InitJoints(JointState here,
            JointState there,
            double seconds,
            bool bCoordinated) {
        return InitJoints(here,
                there,
                seconds,
                std::vector<gomotion::GoTrajParams> (),
                bCoordinated);

    }
#endif
    int GoTraj::InitJoints(JointState here, 
            JointState there, 
            double seconds,  
            std::vector<gomotion::GoTrajParams> jparams,
             bool bCoordinated) {
        JointState nextjoints;
        go_position position;
        try {
            if (pgm->_gmq.get_type() != GO_MOTION_JOINT) {
                // Set type must go BEFORE set_here!
                EF(pgm->_gmq.reset());
                if (bCoordinated) {
                    EF(pgm->_gmq.set_type(GO_MOTION_JOINT));
                } else {
                    EF(pgm->_gms.set_type(GO_MOTION_UJOINT));
                }
                std::copy(here.position.begin(), here.position.end(), position.u.joint);
                EF(pgm->_gmq.set_here(&position)); // position copied
            }
            EF(pgm->_gms.init());
            EF(pgm->_gms.set_type(GO_MOTION_JOINT));
            if (seconds >= 0)
                EF(pgm->_gms.set_time(seconds));
            else {
                if (jparams.size() != num_joints)
                    throw std::runtime_error("InitJoints: not enough joint params");
                    for (size_t i = 0; i < num_joints; i++)
                        EF(pgm->_gms.set_jpar(i, jparams[i].vel, jparams[i].acc, jparams[i].jerk));

            }
            std::copy(there.position.begin(), there.position.end(), position.u.joint);
            EF(pgm->_gms.set_end_position(&position));
            EF(pgm->_gmq.append(&(pgm->_gms)));
            return 0;
        } catch (std::exception &e) {
            std::cout << "GoTraj::InitJoints Exception" << e.what() << "\n";
            throw;
        }
        return -1;
    }
    void GoTraj::AppendJoints(JointState there) {
        go_position position;
        try {
            if (pgm->_gms.get_type() != GO_MOTION_JOINT)
                throw std::runtime_error("AppendJoints when not setup for GO_MOTION_JOINT");
            // Use pgm->_gms again
            std::copy(there.position.begin(), there.position.end(), position.u.joint);
            EF(pgm->_gms.set_end_position(&position));
            EF(pgm->_gmq.append(&(pgm->_gms)));
        } catch (std::runtime_error & e) {
            std::cout << "GoTraj::AppendJoints Exception" << e.what() << "\n";
            throw;
        }
    }
    void GoTraj::InitStop() {
        try {
            EF(pgm->_gmq.stop());
        } catch (std::exception e) {
            std::cout << "GoTraj::Stop Exception" << e.what() << "\n";
            throw;
        }
    }

    bool GoTraj::IsDone() {
        return pgm->_gmq.is_empty();
    }

    JointState GoTraj::NextJoints() {
        JointState nextjoints;
        go_position position;
        try {
            if (pgm->_gms.get_type() != GO_MOTION_JOINT)
                throw std::runtime_error("NextJoints when not setup for GO_MOTION_JOINT");
            EF(pgm->_gmq.interp(&position));
            nextjoints.position = std::vector<double>(&position.u.joint[0], &position.u.joint[ num_joints]);
            nextjoints.velocity.resize(num_joints, 0.0);
        } catch (std::exception &e) {
            std::cout << "GoTraj::NextJoints Exception" << e.what() << "\n";
            throw e;
        }
        return nextjoints;
    }

};


