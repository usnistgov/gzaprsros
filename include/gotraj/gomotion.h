/*
DISCLAIMER:
This software was produced by the National Institute of Standards
and Technology (NIST), an agency of the U.S. government, and by statute is
not subject to copyright in the United States.  Recipients of this software
assume all responsibility associated with its operation, modification,
maintenance, and subsequent redistribution.

See NIST Administration Manual 4.09.07 b and Appendix I. 
 */

/*!
\file gomotion.h

\brief Declarations for motion queue manipulation
 */

#ifndef GOMOTION_H
#define GOMOTION_H

#include "gotraj/gotypes.h"  /* go_result, go_count */
#include "gotraj/gomath.h"  /* go_pose */
#include "gotraj/gotraj_.h"  /* go_traj_cv,a,j_spec */
#include <string>
#include <sstream>   
#include "boost/format.hpp"

#define GO_MOTION_JOINT_NUM 8 /** number of joints supported */
namespace gomotion {

    template<typename T>
    inline std::string VectorDump(std::vector<T> v) {
        std::stringstream s;

        for (size_t i = 0; i < v.size(); i++) {
            s << v[i] << ":";
        }
        return s.str();
    }

    template<typename T>
    inline std::string ArrayDump(T * v, size_t n) {
        std::stringstream s;

        for (size_t i = 0; i < n; i++) {
            s << boost::format("%7.4f") % v[i] << ":";
        }
        return s.str();
    }

    enum {
        /** types of motion queueing done */
        GO_MOTION_NONE,
        GO_MOTION_JOINT,
        GO_MOTION_UJOINT,
        GO_MOTION_WORLD,

        /** these are used to further specify types of world motion */
        GO_MOTION_LINEAR,
        GO_MOTION_CIRCULAR
    };

    /*!
     * \brief Container for a joint position or Cartesian position.
     * Depending upon whether you are doing joint interpolation or
     * world coordinate interpolation (as specified by your call to
     * go_motion_queue_set_type()), fill in joint[] or pose accordingly.
     */
    struct go_position {

        union {
            go_real joint[GO_MOTION_JOINT_NUM];
            go_pose pose;
        } u;

        void zero_joints();
        void zero_pose();
    };

    struct go_motion_params {
        go_real vel; /**< max vel for each motion */
        go_real acc; /**< max accel for each motion */
        go_real jerk; /**< max jerk for each motion */
    };

    /**
    In the following comments, LIN, CIR and ALL refer to linear moves,
    circular moves and both, respectively. If they are not in parens,
    e.g., ALL:, you need to provide them for that type of motion.
    If they are in parens, e.g., (CIR), they are computed for you.
     */

    struct go_motion_linear_params {
        go_cart uvec; /**< (LIN) unit vector along line - computed for you*/
        go_quat uquat; /**< (LIN) unit quaternion along rotation - computed for you*/
    };

    struct go_motion_circular_params {
        go_cart center; /**< The vector to the circle center. */
        go_cart normal; /**< The normal vector that defines the plane of the circle. */
        go_quat qnormal; /**< The normal vector expressed as a unit rotation. */
        go_cart urcsp; /**< The unit vector from center to start, projected onto the normal plane. */
        go_real rstart; /**< The starting radius. */
        go_cart zstart; /**< The vector from the normal plane to the start. */
        go_real thtot; /**< The signed total angular displacement around normal vector */
        go_real rtot; /**< The signed displacement from start to end projected radii */
        go_real ztot; /**< The signed displacement from start to end z off-normals  */

        /*! The inverse of total approximate arc length. If negative, no
        translation motion is taking place. */
        go_real stotinv;

        /*! The number of turns in the circle. 0 means partial CCW, -1 means
        partial CW, otherwise more turns are added in each direction. */
        go_integer turns;
    };

    struct go_motion_spec {
        go_flag type; /**< ALL: GO_MOTION_JOINT,LINEAR,CIRCULAR */
        go_integer id; /**< ALL: id echoed as current move */
        go_real totalt; /**< (ALL) total planned time for the motion - computed for you*/
        go_position start; /**< (ALL)  start pose wrt world; prev end - computed for you*/
        go_position end; /**< ALL: target position, joints or pose */
        go_quat uquat; /**< (ALL) unit rotation from start to end - computed for you*/
        go_motion_params par[GO_MOTION_JOINT_NUM]; /**< For joint moves, each pertains to the indexed joint. For world moves,
        [0] pertains to trans, [1] pertains to rot, and the [2]-[*] are unused. */

        /* specific params for linear and circular moves */
        union {
            go_motion_linear_params lpar; /**< (LIN) linear params */
            go_motion_circular_params cpar; /**< CIR: some circular params */
        } u;

        /* (ALL) times for the various tran phases of CV, CA and CJ profiles */
        /* For world motion [0] is for tran, [1] is for rot, rest unused */
        go_traj_cj_spec cj[GO_MOTION_JOINT_NUM];


        /*!
         * \brief Init the gomotion  specification.
           If 'totalt' is positive, then it is the time for the move.
           Otherwise, it's automatically computed from the vel, acc and jerk,
           the usual case. Here we set 'totalt' to zero to get the usual case.
         * \result go_result success or failure
         */
        go_result init();

        /**
         * Depending upon whether you are doing joint interpolation or
         * world coordinate interpolation (as specified by your call to
         * go_motion_queue_set_type()), fill in joint[] or pose accordingly.
         */

        go_result set_type(go_flag type);
        go_flag get_type();


        go_result set_id(go_integer id);
        go_integer get_id();

        /*!
         * \brief Set a joint parameters
         * \param i joint number.
         * \param vel velocity limit UNITS?
         * \param acc acceleration limit UNITS?
         * \param jerk jerk limit UNITS?
         * \result go_result success or failure
         */
        go_result set_jpar(go_integer i, go_real vel, go_real acc, go_real jerk);

        /*!
         * \brief Set world Cartesian translation parameters
         * \param vel velocity limit UNITS?
         * \param acc acceleration limit UNITS?
         * \param jerk jerk limit UNITS?
         * \result go_result success or failure
         */
        go_result set_tpar(go_real vel, go_real acc, go_real jerk);

        /*!
         * \brief Set world Cartesian rotational parameters
         * \param vel velocity limit UNITS?
         * \param acc acceleration limit UNITS?
         * \param jerk jerk limit UNITS?
         * \result go_result success or failure
         */
        go_result set_rpar(go_real vel, go_real acc, go_real jerk);

        /*!
         * \brief Set circular rotation Cartesian move parameters
         */
        go_result set_cpar(go_cart * center, go_cart * normal, go_integer turns);

        /*!
         * \brief Set time to positive real number.
         * \param time  in seconds.
         * \result go_result success or failure
         */
        go_result set_time(go_real time);

        /**
         * \brief If you are doing joint interpolation (as specified by your call to
         * go_motion_queue_set_type()), et_end_position fills in joint[]  accordingly.
         * \param  end  is the end position of the joints
         * \result go_result success or failure
         */
        go_result set_end_position(go_position * end);

        /**
         * \brief If you are doing world coordinate interpolation  (as specified by your call to
         * go_motion_queue_set_type()), set_end_position fills in  ending pose  accordingly.
         * \param  end  is the end pose of the robot (position and orientation)
         * \result go_result success or failure
         */
        go_result set_end_pose(go_pose * end);
    };

    struct go_scale_spec {
    //protected:
        go_traj_ca_spec scale_spec;
        go_flag scaling; /**< non-zero means we're scaling time */
        go_flag scale_dir; /**< non-zero means add scale to scale_b  */
        go_flag scale_isneg; /**< non-zero means negative scale  */
        go_real scale_b; /**< original base scale factor */
        go_real scale; /**< current scale factor */
        go_real scale_next; /**< pending scale factor */
        go_real scale_v_next; /**< pending d(scale)/dt */
        go_real scale_a_next; /**< pending d^2(scale)/dt^2 */
        go_real scale_t; /**< time into scaling */
    public:
        go_result init(go_real scale);
        go_result set(
                go_real scale,
                go_real scale_v,
                go_real scale_a);
        go_result eval(
                go_real deltat,
                go_real * scale);
    };

    struct go_motion_queue {
    protected:
        go_flag _type; /**< go_MOTION_JOINT for joint, otherwise world */
        go_position _here; /**< initial starting point */
        go_position _there; /**< where the end is */
        go_motion_spec *_startptr; /**< ptr to original start */
        go_motion_spec *_endptr; /**< the original end */
        go_motion_spec *_start; /**< ptr to first entry of the queue */
        go_motion_spec *_end; /**< ptr to last (empty) entry */
        go_integer _size; /**< size of whole queue */
        go_integer _joint_num; /**< number of joints to be interpolated */
        go_integer _number; /**< number of motions on queue */
        go_integer _last_id; /**< id of last motion appended */
        go_real _deltat; /**< cycle time */
        go_real _time; /**< time into the current spec */
        go_scale_spec _timescale; /**< walked-in time scale factor */
    public:
        go_result init(
                go_motion_spec * space,
                go_integer size,
                go_real deltat);

        /**
         * \brief Reset motion queue. Null motion type, reset start and set end to start, 
         * Zero number of motions on queue, last_id of motion on queue, time.
         * Init timescale to 1.0.
         * \result go_result success or failure
         */
        go_result reset();

        /**
         * \brief Set type of motion queueing done:  
         * GO_MOTION_NONE, GO_MOTION_JOINT, GO_MOTION_UJOINT, GO_MOTION_WORLD
         * 
         * these flags are used to further specify types of world motion:
         *  GO_MOTION_LINEAR, GO_MOTION_CIRCULAR
         * \result go_result success or failure
         */
        go_result set_type(go_flag type);

        go_flag get_type();

        /**
         * \brief Set number of joints. Must positive andless than GO_MOTION_JOINT_NUM.
         * \param joints number of joints
         * \result go_result success or failure
         */
        go_result set_joint_number(go_integer joints);
        /**
         * \brief Get number of joints. 
         * \result integer
         */
        go_integer get_joint_number();

        /**
         * \brief Set current position - either joint values or world Cartesian pose.
         * \param here as a pointer to a go_position
         * \result go_result success or failure
         */
        go_result set_here(const go_position * here);

        /**
         * \brief Set deltat,  the  time in seconds per cycle.
         * \param deltat is time in real seconds, so fractional seconds possible.
         * \result go_result success or failure
         */
        go_result set_cycle_time(go_real deltat);

        
        go_result set_scale(
                go_real scale,
                go_real scale_v,
                go_real scale_a);
       /**
         * \brief Append motion_spec onto queue.
         * if type is GO_MOTION_JOINT then motion is done as append_joint
         * if type is GO_MOTION_UJOINT then motion is done as append_ujoint
         * if type is GO_MOTION_WORLD then motion is done as append_world
         * \param motion is a pointer to a motion spec.
         * \result go_result success or failure
         */
        go_result append(const go_motion_spec * motion);

        go_result number(go_integer * number);

        go_result size(go_integer * size);
        /**
         * \brief Get the head of the motion queue.
         * \param motion is a pointer to store the head of the motion queue.
         * \result go_result success or failure
         */
        go_result head(go_motion_spec * motion);
        /**
         * \brief Get the here position and store into   pointer to go_motion_spec.
         * \param position is a pointer to store here motion spec.
         * \result go_result success or failure
         */
        go_result here(go_position * position);

        /**
         * \brief Get the there position and store into the  motion  pointer to go_motion_spec.
         * \param position is a pointer to store there motion spec.
         * \result go_result success or failure
         */
        go_result there(go_position * position);

        /**
         * \brief Do interpret cycle and store result into position pointer.
         * Only if there's something on the queue do we want to increment
         * the queue time. It was set to 0 when the queue was emptied, and
         * will not be set to zero when the next move is appended.
         * \param position is a pointer to store computed joint/world motion.
         * \result go_result success or failure
         */
        go_result interp(go_position * position);


        go_result stop(go_motion_queue * queue);
        /**
         * \brief Set  id of last motion appended.
         * \param integer value for last id of motion queued. 
         * FIXME: why would it ever not be incremental?
         * \result go_result success or failure
         */
        go_result set_id(go_integer id);

        /**
         * \brief Get  id of last motion appended.
         * \result integer value of motion.
         */
        go_integer last_id();

        go_flag is_empty();

        go_result erase();

        go_result drop_pending();
        /**
         * \brief Stop queue interpret cycle now.
         * Don't need to stop an empty or stopping queue store computed joint/world motion.
         * point our attention at the current motion.
         * Call go_traj_cj_stop for each joint- or tran/rot motion, so that
         * each stops as fast as it can. The longest to stop will set the
         * revised stop time, and each will be extended to stop at that
         * longest time.  Note that the specptr values are incremental for
         * the move, as the original distance passed to go_traj_cj_compute
         * was the incremental distance.
         * \result go_result success or failure
         */
        go_result stop();
    protected:
        go_result append_joint(
                const go_motion_spec * motion);

        go_result append_ujoint(
                const go_motion_spec * motion);

        go_result append_world(
                const go_motion_spec * motion);

        go_result interp_joint(go_real * joint);

        go_result interp_world(go_pose * pose);

        go_result
        interp_world(go_motion_spec * motion, go_real time, go_pose * pose);
    };

};

#endif /* GOMOTION_H */
