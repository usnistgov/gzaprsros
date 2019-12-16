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
\file gomotion.c

\brief Motion queue manipulation
 */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <math.h>  /* acos(), fabs() */
#include "gotraj/gotypes.h"
#include "gotraj/gomath.h"
#include "gotraj/gotraj.h"
#include "gotraj/gomotion.h"
namespace gomotion {
    
   

    void go_position::zero_joints() {
        go_integer i;

        for (i = 0; i < GO_MOTION_JOINT_NUM; i++) {
            this->u.joint[i] = 0.0;
        }
    }

    void go_position::zero_pose() {
        this->u.pose.tran.x = 0.0;
        this->u.pose.tran.y = 0.0;
        this->u.pose.tran.z = 0.0;
        this->u.pose.rot.s = 1.0;
        this->u.pose.rot.x = 0.0;
        this->u.pose.rot.y = 0.0;
        this->u.pose.rot.z = 0.0;
    }

    go_result go_motion_spec::init() {
        /*
        If 'totalt' is positive, then it is the time for the move.
        Otherwise, it's automatically computed from the vel, acc and jerk,
        the usual case. Here we set 'totalt' to zero to get the usual case.
         */
        this->totalt = 0.0;

        return GO_RESULT_OK;
    }

    go_result go_motion_spec::set_type(go_flag type) {
        if (type == GO_MOTION_JOINT ||
                type == GO_MOTION_LINEAR ||
                type == GO_MOTION_CIRCULAR) {
            this->type = type;
            return GO_RESULT_OK;
        }

        return GO_RESULT_BAD_ARGS;
    }

    go_flag go_motion_spec::get_type() {
        return this->type;
    }

    go_result go_motion_queue::set_joint_number(
            go_integer joints) {
        if (joints < 1 || joints > GO_MOTION_JOINT_NUM) {
            return GO_RESULT_BAD_ARGS;
        }

        this->_joint_num = joints;

        return GO_RESULT_OK;
    }

    go_integer go_motion_queue::get_joint_number() {
        return this->_joint_num;
    }

    go_result go_motion_spec::set_id(go_integer id) {
        this->id = id;

        return GO_RESULT_OK;
    }

    go_integer go_motion_spec::get_id() {
        return this->id;
    }

    go_result go_motion_spec::set_jpar(go_integer i, go_real vel, go_real acc, go_real jerk) {
        if (i < 0 || i >= GO_MOTION_JOINT_NUM) return GO_RESULT_ERROR;
        if (vel <= GO_REAL_EPSILON ||
                acc <= GO_REAL_EPSILON ||
                jerk <= GO_REAL_EPSILON) return GO_RESULT_BAD_ARGS;

        this->par[i].vel = vel, this->par[i].acc = acc, this->par[i].jerk = jerk;

        return GO_RESULT_OK;
    }

    go_result go_motion_spec::set_tpar(go_real vel, go_real acc, go_real jerk) {
        if (vel <= GO_REAL_EPSILON ||
                acc <= GO_REAL_EPSILON ||
                jerk <= GO_REAL_EPSILON) return GO_RESULT_BAD_ARGS;

        this->par[0].vel = vel, this->par[0].acc = acc, this->par[0].jerk = jerk;

        return GO_RESULT_OK;
    }

    go_result go_motion_spec::set_rpar(go_real vel, go_real acc, go_real jerk) {
        if (vel <= GO_REAL_EPSILON ||
                acc <= GO_REAL_EPSILON ||
                jerk <= GO_REAL_EPSILON) return GO_RESULT_BAD_ARGS;

        this->par[1].vel = vel, this->par[1].acc = acc, this->par[1].jerk = jerk;

        return GO_RESULT_OK;
    }

    go_result go_motion_spec::set_cpar(go_cart * center, go_cart * normal, go_integer turns) {
        go_result retval;

        this->u.cpar.center = *center;
        retval = go_cart_unit(normal, &this->u.cpar.normal);
        if (GO_RESULT_OK != retval) return retval;
        this->u.cpar.turns = turns;

        return GO_RESULT_OK;
    }

    go_result go_motion_spec::set_time(go_real time) {
        if (time > 0.0) {
            this->totalt = time;
            return GO_RESULT_OK;
        }

        return GO_RESULT_BAD_ARGS;
    }

    go_result go_motion_spec::set_end_position(go_position * end) {
        this->end = *end;

        return GO_RESULT_OK;
    }

    go_result go_motion_spec::set_end_pose(go_pose * end) {
        this->end.u.pose = *end;

        return GO_RESULT_OK;
    }

    go_result go_motion_queue::init(
            go_motion_spec * space,
            go_integer size,
            go_real deltat) {
        if (0 == space || size <= 0 || deltat <= 0) {
            return GO_RESULT_BAD_ARGS;
        }

        this->_size = size;
        this->_startptr = space;
        this->_endptr = space + size;
        this->_deltat = deltat;
        this->_joint_num = GO_MOTION_JOINT_NUM;

        return this->reset();
    }

    go_result go_motion_queue::reset() {
        this->_type = GO_MOTION_NONE;
        this->_start = this->_startptr;
        this->_end = this->_start;
        this->_number = 0;
        this->_last_id = 0;
        this->_time = 0.0;

        return this->_timescale.init(1.0);
    }

    go_result go_motion_queue::set_type(go_flag type) {
        if (type == GO_MOTION_NONE ||
                type == GO_MOTION_JOINT ||
                type == GO_MOTION_UJOINT ||
                type == GO_MOTION_WORLD) {
            this->_type = type;
            return GO_RESULT_OK;
        }

        return GO_RESULT_ERROR;
    }

    go_flag go_motion_queue::get_type() {
        return this->_type;
    }

    go_result go_motion_queue::set_here(
            const go_position * here) {
        go_integer i;

        if (this->_number != 0) {
            return GO_RESULT_ERROR;
        }

        if (this->_type == GO_MOTION_JOINT ||
                this->_type == GO_MOTION_UJOINT) {
            for (i = 0; i < this->_joint_num; i++) {
                this->_there.u.joint[i] = this->_here.u.joint[i] = here->u.joint[i];
            }
            return GO_RESULT_OK;
        }

        if (this->_type == GO_MOTION_WORLD) {
            this->_there.u.pose = this->_here.u.pose = here->u.pose;

            return GO_RESULT_OK;
        }

        return GO_RESULT_ERROR;
    }

    go_result go_motion_queue::set_cycle_time(
            go_real deltat) {
        if (deltat <= 0.) {
            return GO_RESULT_BAD_ARGS;
        }

        this->_deltat = deltat;

        return GO_RESULT_OK;
    }

    go_result go_scale_spec::init(go_real scale) {
        this->scaling = 0;
        this->scale = scale;
        this->scale_dir = 0;
        this->scale_b = this->scale;
        this->scale_next = this->scale;
        this->scale_v_next = 1.0;
        this->scale_a_next = 1.0;
        this->scale_t = 0.0;

        return GO_RESULT_OK;
    }

    go_result go_scale_spec::set(
            go_real scale,
            go_real scale_v,
            go_real scale_a) {
        go_real scale_incr;

        if (scale < 0 ||
                scale_v <= 0.0 ||
                scale_a <= 0.0) {
            return GO_RESULT_BAD_ARGS;
        }

        this->scale_next = scale;
        /* need to save these so they can be used when loading scale_next */
        this->scale_v_next = scale_v;
        this->scale_a_next = scale_a;

        if (this->scaling) {
            return GO_RESULT_OK;
        }

        if (scale > this->scale) {
            scale_incr = scale - this->scale;
            this->scale_dir = 1;
        } else {
            scale_incr = this->scale - scale;
            this->scale_dir = 0;
        }
        this->scaling = 1;
        this->scale_b = this->scale;
        this->scale_t = 0.0;

        return go_traj_ca_compute(scale_incr, scale_v, scale_a, &this->scale_spec);
    }

    go_result go_scale_spec::eval(
            go_real deltat,
            go_real * scale) {
        go_traj_interp_spec ti;
        go_real scale_incr;

        if (this->scaling) {
            this->scale_t += deltat;

            go_traj_ca_interp(&this->scale_spec, this->scale_t, &ti);
            scale_incr = ti.d;

            if (this->scale_dir) {
                this->scale = this->scale_b + scale_incr;
            } else {
                this->scale = this->scale_b - scale_incr;
            }
            if (this->scale_t >= this->scale_spec.tend) {
                this->scaling = 0;
                if (!GO_TRAN_CLOSE(this->scale, this->scale_next)) {
                    set(this->scale_next,
                            this->scale_v_next, this->scale_a_next);
                }
            }
        }
        if (NULL != scale) *scale = this->scale;

        return GO_RESULT_OK;
    }

    go_result go_motion_queue::set_scale(
            go_real scale,
            go_real scale_v,
            go_real scale_a) {
        return this->_timescale.set(scale, scale_v, scale_a);
    }

    /*
    Given:

    The current queue, 'queue'.

    The joint motion to append, 'motion', with these set:

    motion->end.u.joint[] for the target joint positions in abs coords;
    motion->profile for the type (CV, CA, CJ);
    motion->par[] for the joint v,a,j.

    Computed:

    The motion spec to be appended, 'spec', copied from *motion and then
    filled in as follows:

    spec.start.u.joint[] for the start positions (from this->there.u.joint[]);
    spec.cv,a,j[] for the phase distances and times for simultaneous arrival;
    spec.totalt for the max time.

    The queue is modified as follows:

    this->there.u.joint[] for the end positions (from spec.end.u.joint[]);
    the queue pointers for an append.
     */

    go_result
    go_motion_queue::append_joint(
            const go_motion_spec * motion) {
        go_motion_spec spec;
        go_real td, maxtd;
        go_integer i;

        if (this->_number >= this->_size) {
            return GO_RESULT_NO_SPACE;
        }

        spec = *motion;
        maxtd = spec.totalt; /* set the max to be the provided time,
									which is 0 normally for speed-based
									moves but will be non-zero for time-based
									moves */

        /* fill in the motion specs (phase times, distances) for each joint,
        keeping track of the maximum time. This maximum time will be used
        to scale all the joint motions so that they arrive at the same time,
        and will be the overall time 'totalt' for the motion. */

        for (i = 0; i < this->_joint_num; i++) {
            /* set the start joint value to be the end of the previous move */
            spec.start.u.joint[i] = this->_there.u.joint[i];

            /* 'td' is the incremental distance traveled by this joint */
            td = fabs(spec.end.u.joint[i] - this->_there.u.joint[i]);

            /* set the end joint values for the queue */
            this->_there.u.joint[i] = spec.end.u.joint[i];
            go_traj_cj_compute(td, spec.par[i].vel, spec.par[i].acc, spec.par[i].jerk, &spec.cj[i]);
            if (spec.cj[i].tend > maxtd) maxtd = spec.cj[i].tend;
        }

        spec.totalt = maxtd; /* set the overall motion time */

        /* now all the joint motion phase times are calculated, and we have
        the maximum time, so scale all the joint moves to arrive at the 
        same time */

        for (i = 0; i < this->_joint_num; i++) {
            go_traj_cj_scale(&spec.cj[i], maxtd, &spec.cj[i]);
        }

        /* copy the spec to the queue, and update the queue pointers */
        *this->_end = spec;
        this->_number++;
        this->_end++;
        if (this->_end >= this->_endptr) {
            this->_end = this->_startptr;
        }
        this->_last_id = spec.id;

        return GO_RESULT_OK;
    }

    go_result
    go_motion_queue::append_ujoint(
            const go_motion_spec * motion) {
        go_motion_spec spec;
        go_real td, maxtd;
        go_integer i;

        if (this->_number >= this->_size) {
            return GO_RESULT_NO_SPACE;
        }

        spec = *motion;
        maxtd = 0.0; /* no timed motion for ujoint moves,
								so ignore spec.totalt */

        /* fill in the motion specs (phase times, distances) for each joint,
        keeping track of the maximum time. This maximum time will be used
        to scale all the joint motions so that they arrive at the same time,
        and will be the overall time 'totalt' for the motion. */

        for (i = 0; i < this->_joint_num; i++) {
            /* set the start joint value to be the end of the previous move */
            spec.start.u.joint[i] = this->_there.u.joint[i];

            /* 'td' is the incremental distance traveled by this joint */
            td = fabs(spec.end.u.joint[i] - this->_there.u.joint[i]);

            /* set the end joint values for the queue */
            this->_there.u.joint[i] = spec.end.u.joint[i];
            go_traj_cj_compute(td, spec.par[i].vel, spec.par[i].acc, spec.par[i].jerk, &spec.cj[i]);
            if (spec.cj[i].tend > maxtd) maxtd = spec.cj[i].tend;
        }

        spec.totalt = maxtd; /* set the overall motion time */

        /* copy the spec to the queue, and update the queue pointers */
        *this->_end = spec;
        this->_number++;
        this->_end++;
        if (this->_end >= this->_endptr) {
            this->_end = this->_startptr;
        }
        this->_last_id = spec.id;

        return GO_RESULT_OK;
    }

    /*
    Circular motion is done in cylindrical coordinates, enabling purely
    circular (theta) motion as well as helical (z) motion and spiral (r)
    motion. Motion is parameterized by curve length in the trajectory
    planner, not in the natural polar angle. The curve length of a
    general helical spiral is closed-form but not invertible, so we
    can't get polar angle as a function of arc length exactly. We can
    approximate arc length as the Cartesian sum of the average azimuthal
    displacment and axial displacement. Given this, we can calculate an
    approximate polar angle and a resulting trajectory point that lies
    exactly on the curve and moves close to the desired speed.

    Given the start pose, end pose, center vector, normal vector and
    number of turns, we compute some quantities that most efficiently
    allow us to interpolate along the general curve with an arc length
    parameterization. 

    First we get the estimated total arc length as

    stot = sqrt(sq(average azimuthal distance) + sq(axial distance))

    where the average azimuthal distance is the distance swept out at
    the average of the starting- and ending radii as the angle moves
    from the starting radial vector to the ending radial vector
    (cylindrical theta coordinate); and the axial distance is the
    distance along the helical axis (cylindrical z coordinate).

    This is a close approximation. For example, for a one-turn spiral
    starting at r = 1 and ending at r = 2, the exact arc length is 9.48
    and the approximate arc length is 9.42.

    Let 'normal' be the unit normal vector, and 'qnormal' be 'normal'
    considered as a rotation vector converted to a quaternion. 'normal'
    will be used to slide along the Z axis. 'qnormal' will be used to
    sweep the radial vector.

    Let 'rcs' be the vector from the center to the start, 'rcsp' be the
    projection of 'rcs' into the normal plane with length 'rstart', and
    'zstart' be the vector from the plane to 'rcs'.

    Likewise, let 'rce' be the vector from the center to the end, 'rcep'
    be the projection of 'rce' into the normal plane with length 'rend',
    and 'zend' be the vector from the plane to 'rce'.

    Let 'rtot' = 'rend' - 'rstart' be the signed difference between the
    ending and starting radii. Let 'zvec' be the vector from 'zstart' to
    'zend', with 'zuvec' and 'zmag' the unit vector and magnitude,
    respectively.

    Let 'urcsp' be the unit vector along 'rcsp'. This is the unit vector
    that initially points toward the projection of the starting pose
    onto the normal plane, and sweeps around as motion proceeds, always
    pointing toward the projection of the trajectory point onto the
    normal plane.

    Let 'thtot' be the total angle of rotation that brings 'rcsp' to
    'rcep', including any extra turns. Let 'srtot' be the azimuthal
    length swept out by the average radial vector through 'thtot'. Let
    'ztot' be the distance from 'zstart' to 'zend'. Let 'stot' be the
    Cartesian sum of 'srtot' and 'ztot'. Let 'stotinv' be its inverse.

    Given a length along the curve 's' calculated by the trajectory
    planner, we multiply by 'stotinv' to get the fraction of the path
    traversed, 'sfrac'. Calculate the fractions of the swept angle,
    radial difference and z vectors as respectively

    thfrac = sfrac * thtot
    rfrac  = sfrac * rtot
    zfrac  = sfrac * ztot

    Scale the unit rotation quaternion 'qnormal' by 'thfrac' and rotate
    the unit radial vector 'urcsp' by this. Scale by 'rstart' + 'rfrac' to
    get the radial vector 'rvec'.

    Scale the unit normal vector 'normal' by 'zfrac' and add to 'zstart'
    to bget the axial vector 'zvec'.

    Calculate the complete vector to the trajectory point as 'center' +
    'rvec' + 'zvec'.

    The rotation from the starting orientation to the ending orientation
    is handled as all motions, in which the "length" is the
    magnitude of the rotation vector from starting orientation to the
    ending orientation, which is scaled from zero to full magnitude
    as the motion proceeds:

    rotend = rotstart * rot -> rot = inv(rotstart) * rotend

    Unitize 'rot' to get 'urot'.

    During interpolation, scale 'urot' by the trajectory planner's
    current rotational distance, and get the current rotation as

    rotcur = rotstart * urot * rotational distance
     */

    go_result
    go_motion_queue::append_world(
            const go_motion_spec * motion) {
        go_motion_spec spec;
        go_quat quat;
        go_cart cart;
        go_cart rcs; /* vector from center to start */
        go_cart rcsp; /* its projection onto normal plane */
        go_cart rce; /* vector from center to end */
        go_cart rcep; /* its projection onto normal plane */
        go_real r;
        go_real dot;
        go_real ravg;
        go_real time;
        go_rvec rvec;
        go_real td;
        go_real rd;
        go_result retval;

        if (this->_number >= this->_size) {
            return GO_RESULT_NO_SPACE;
        }

        spec = *motion;
        time = spec.totalt; /* save requested time and override totalt
								later if applicable */

        switch (spec.type) {
            case GO_MOTION_LINEAR:
                go_cart_cart_sub(&spec.end.u.pose.tran, &this->_there.u.pose.tran, &cart);
                go_cart_mag(&cart, &td);
                if (GO_RESULT_OK != go_cart_unit(&cart, &spec.u.lpar.uvec)) {
                    /* no translational motion, so set uvec to X */
                    spec.u.lpar.uvec.x = 1, spec.u.lpar.uvec.y = spec.u.lpar.uvec.z = 0;
                }
                break;

            case GO_MOTION_CIRCULAR:
                /* unitize the normal vector and express it also as a unit rotation */
                if (GO_RESULT_OK != go_cart_unit(&spec.u.cpar.normal, &spec.u.cpar.normal)) return GO_RESULT_ERROR;
                (void) go_cart_rvec_convert(&spec.u.cpar.normal, &rvec);
                (void) go_rvec_quat_convert(&rvec, &spec.u.cpar.qnormal);

                /* get the vector from the center to the start and project onto
                the normal plane */
                (void) go_cart_cart_sub(&this->_there.u.pose.tran, &spec.u.cpar.center, &rcs);
                (void) go_cart_plane_proj(&rcs, &spec.u.cpar.normal, &rcsp);
                /* save the projected starting radius */
                (void) go_cart_mag(&rcsp, &spec.u.cpar.rstart);
                /* save the vector from the normal plane to the start */
                (void) go_cart_cart_sub(&rcs, &rcsp, &spec.u.cpar.zstart);
                /* save the unit projected start vector for scaling and sweeping later */
                if (GO_RESULT_OK != go_cart_unit(&rcsp, &spec.u.cpar.urcsp)) return GO_RESULT_ERROR;

                /* get the vector from the center to the end and project onto
                the normal plane */
                (void) go_cart_cart_sub(&spec.end.u.pose.tran, &spec.u.cpar.center, &rce);
                (void) go_cart_plane_proj(&rce, &spec.u.cpar.normal, &rcep);
                /* get the projected ending radius */
                (void) go_cart_mag(&rcep, &r);
                /* get the average radius */
                ravg = 0.5 * (spec.u.cpar.rstart + r);
                /* save the signed displacement from start to end radii */
                spec.u.cpar.rtot = r - spec.u.cpar.rstart;
                /* get the vector from the normal plane to the end */
                (void) go_cart_cart_sub(&rce, &rcep, &cart);
                /* and the vector from the start to the end */
                (void) go_cart_cart_sub(&cart, &spec.u.cpar.zstart, &cart);
                /* save the distance between z start and z end */
                go_cart_mag(&cart, &spec.u.cpar.ztot);
                /* we will use the normal vector to move from z start to z end,
                so we need to make the total displacement negative if
                zdiff and normal are antiparallel */
                (void) go_cart_cart_dot(&cart, &spec.u.cpar.normal, &dot);
                if (dot < 0.0) {
                    spec.u.cpar.ztot = -spec.u.cpar.ztot;
                }

                /* compute angle between the projections. This will be the angle
                we will interpolate */
                go_cart_cart_angle(&rcsp, &rcep, &r);
                /* now 'r' is the angle between the start and end vector projections,
                in range [0..PI]. We need to correct this for more than half circles,
                plus number of turns */
                go_cart_cart_cross(&rcsp, &rcep, &cart);
                go_cart_cart_dot(&cart, &spec.u.cpar.normal, &dot);
                if (dot < 0.0) {
                    /* start X end lies in opposite direction of normal, so r should
                    be the larger one */
                    r = GO_2_PI - r;
                }
                /* now 'r' is a non-negative number, so we can add the number of turns */
                r += GO_2_PI * spec.u.cpar.turns;
                /* and save it out */
                spec.u.cpar.thtot = r;

                /* get the average azimuthal distance */
                r = ravg * spec.u.cpar.thtot;
                /* get the Cartesian total distance */
                r = sqrt(go_sq(r) + go_sq(spec.u.cpar.ztot));
                td = r; /* we use td down below */
                if (GO_TRAN_SMALL(r)) {
                    /* no translation, so set stotinv to some negative and thus invalid
                    value to signify this */
                    spec.u.cpar.stotinv = -1.0;
                } else {
                    spec.u.cpar.stotinv = 1.0 / r;
                }
                break;

            default:
                return GO_RESULT_ERROR;
        } /* switch (spec.type) */

        /* do rotation the same way for either linear or circular moves */
        if (GO_RESULT_OK != go_quat_inv(&this->_there.u.pose.rot, &quat)) {
            /* 'there' should always be invertible; if not it wasn't initialized
            to I, so do it here */
            quat = this->_there.u.pose.rot = go_quat_identity();
        }
        retval = go_quat_quat_mult(&quat, &spec.end.u.pose.rot, &quat);
        if (GO_RESULT_OK != retval) {
            /* quat is OK, so spec.u.pose.u.pose.rot must not be */
            return retval;
        }
        go_quat_mag(&quat, &rd);
        if (GO_RESULT_OK != go_quat_unit(&quat, &spec.uquat)) {
            /* no rotational motion, so set uquat I */
            spec.uquat = go_quat_identity();
        }
        

        go_traj_cj_compute(td, spec.par[0].vel, spec.par[0].acc, spec.par[0].jerk,
                &spec.cj[0]);
        go_traj_cj_compute(rd, spec.par[1].vel, spec.par[1].acc, spec.par[1].jerk,
                &spec.cj[1]);
        if (spec.cj[0].tend > spec.cj[1].tend) {
            /* translation takes longer, so scale rotation */
            spec.totalt = spec.cj[0].tend;
            go_traj_cj_scale(&spec.cj[1], spec.totalt, &spec.cj[1]);
        } else {
            /* rotation takes longer, so scale translation */
            spec.totalt = spec.cj[1].tend;
            go_traj_cj_scale(&spec.cj[0], spec.totalt, &spec.cj[0]);
        }
        /* now scale both trans and rot motions to take 'time', if requested */
        if (time > spec.totalt) {
            spec.totalt = time;
            go_traj_cj_scale(&spec.cj[0], spec.totalt, &spec.cj[0]);
            go_traj_cj_scale(&spec.cj[1], spec.totalt, &spec.cj[1]);
        }

        spec.start = this->_there;
        this->_there = spec.end;
#if defined(DEBUG) && defined (GODEBUG)
        std::cout << "    Cart    = " << ArrayDump<double>((double *) &cart, 3) << "\n";
        std::cout << "    MAG Cart= " << td << "\n";
        std::cout << "    CartUnit= " << ArrayDump<double>((double *) &spec.u.lpar.uvec, 3) << "\n";
        std::cout << "    QuatUnit= " << ArrayDump<double>((double *) &spec.uquat, 4) << "\n";
        std::cout << "    QuatMag = " << rd << "\n";
        std::cout << "    spec.start= " << ArrayDump<double>((double *) &spec.start, 7) << "\n";
        std::cout << "    spec.end= " << ArrayDump<double>((double *) & this->_there, 7) << "\n";
#endif
        *this->_end = spec;
        this->_number++;
        this->_end++;
        if (this->_end >= this->_endptr) {
            this->_end = this->_startptr;
        }
        this->_last_id = spec.id;

        return GO_RESULT_OK;
    }

    go_result
    go_motion_queue::append(
            const go_motion_spec * motion) {
        if (this->_type == GO_MOTION_JOINT) {
            return this->append_joint(motion);
        }

        if (this->_type == GO_MOTION_UJOINT) {
            return this->append_ujoint(motion);
        }

        if (this->_type == GO_MOTION_WORLD) {
            return this->append_world(motion);
        }

        return GO_RESULT_ERROR;
    }

    go_result go_motion_queue::number(go_integer * number) {
        *number = this->_number;

        return GO_RESULT_OK;
    }

    go_result go_motion_queue::size(go_integer * size) {
        *size = this->_size;

        return GO_RESULT_OK;
    }

    go_result go_motion_queue::head(go_motion_spec * motion) {
        if (0 == this->_number) {
            return GO_RESULT_EMPTY;
        }

        *motion = *this->_start;

        return GO_RESULT_OK;
    }

    go_result
    go_motion_queue::here(go_position * position) {
        go_integer i;

        if (this->_type == GO_MOTION_JOINT ||
                this->_type == GO_MOTION_UJOINT) {
            for (i = 0; i < this->_joint_num; i++) {
                position->u.joint[i] = this->_here.u.joint[i];
            }
            return GO_RESULT_OK;
        }

        if (this->_type == GO_MOTION_WORLD) {
            position->u.pose = this->_here.u.pose;
            return GO_RESULT_OK;
        }

        return GO_RESULT_ERROR;
    }

    go_result
    go_motion_queue::there(go_position * position) {
        go_integer i;

        if (this->_type == GO_MOTION_JOINT ||
                this->_type == GO_MOTION_UJOINT) {
            for (i = 0; i < this->_joint_num; i++) {
                position->u.joint[i] = this->_there.u.joint[i];
            }
            return GO_RESULT_OK;
        }

        if (this->_type == GO_MOTION_WORLD) {
            position->u.pose = this->_there.u.pose;
            return GO_RESULT_OK;
        }

        return GO_RESULT_ERROR;
    }

    go_result
    go_motion_queue::interp_joint(
            go_real * joint) {
        go_motion_spec spec;
        go_traj_interp_spec tis;
        go_integer i;

        if (0 == this->_number) {
            for (i = 0; i < this->_joint_num; i++) {
                joint[i] = this->_here.u.joint[i];
            }
            return GO_RESULT_OK;
        }

        spec = *this->_start;

        for (i = 0; i < this->_joint_num; i++) {
            go_traj_cj_interp(&spec.cj[i], this->_time, &tis);
            if (spec.end.u.joint[i] > spec.start.u.joint[i]) {
                joint[i] = this->_here.u.joint[i] = spec.start.u.joint[i] + tis.d;
            } else {
                joint[i] = this->_here.u.joint[i] = spec.start.u.joint[i] - tis.d;
            }
        }

        if (this->_time >= spec.totalt) {
            /* we finished this move, so take it off the queue and set
            the time to zero as the basis for the next move */
            this->erase();
            this->_time = 0.;
        }

        return GO_RESULT_OK;
    }

    go_result
    go_motion_queue::interp_world(go_motion_spec * motion, go_real time, go_pose * pose) {
        go_traj_interp_spec tis;
        go_traj_interp_spec ris;
        go_cart cart;
        go_quat quat;
        go_real frac;

        go_traj_cj_interp(&motion->cj[0], time, &tis);
        go_traj_cj_interp(&motion->cj[1], time, &ris);

        switch (motion->type) {
            case GO_MOTION_LINEAR:
                go_cart_scale_mult(&motion->u.lpar.uvec, tis.d, &cart);
                go_cart_cart_add(&motion->start.u.pose.tran, &cart, &pose->tran);
#if defined(DEBUG) && defined (GODEBUG)
                std::cout << "    motion.pre= " << ArrayDump<double>((double *) &motion->start.u.pose.tran, 3) << "\n";
                std::cout << "    motion.pre= " << ArrayDump<double>((double *) &pose->tran, 3) << "\n";
#endif
                break;

            case GO_MOTION_CIRCULAR:
                if (motion->u.cpar.stotinv < 0) {
                    /* no translational motion around the circle, so we're there */
                    pose->tran = motion->end.u.pose.tran;
                    break;
                }
                pose->tran = motion->u.cpar.center; /* start at center */
                frac = tis.d * motion->u.cpar.stotinv; /* get fraction of move */
                /* scale rotation around axis */
                go_quat_scale_mult(&motion->u.cpar.qnormal, frac * motion->u.cpar.thtot, &quat);
                /* grow the radial vector */
                go_cart_scale_mult(&motion->u.cpar.urcsp, motion->u.cpar.rstart + frac * motion->u.cpar.rtot, &cart);
                /* sweep the radial vector */
                go_quat_cart_mult(&quat, &cart, &cart);
                /* add it to our pose vector */
                go_cart_cart_add(&pose->tran, &cart, &pose->tran);
                /* grow the z vector */
                go_cart_scale_mult(&motion->u.cpar.normal, frac * motion->u.cpar.ztot, &cart);
                /* add it to the starting z */
                go_cart_cart_add(&motion->u.cpar.zstart, &cart, &cart);
                /* add this to the pose */
                go_cart_cart_add(&pose->tran, &cart, &pose->tran);
                break;

            default:
                return GO_RESULT_ERROR;
        }

        /* rotation is handled the same way for either type */
        go_quat_scale_mult(&motion->uquat, ris.d, &quat);
        go_quat_quat_mult(&motion->start.u.pose.rot, &quat, &pose->rot);

        return GO_RESULT_OK;
    }

    go_result
    go_motion_queue::interp_world(go_pose * pose) {
        go_motion_spec motion;
        go_real time;
        go_result retval;

        if (0 == this->_number) {
            *pose = this->_there.u.pose;
            return GO_RESULT_OK;
        }

        motion = *this->_start;
        time = this->_time;

        retval = interp_world(&motion, time, pose);
        if (GO_RESULT_OK != retval) return retval;

        if (this->_time >= motion.totalt) {
            /* we finished this move */
            this->erase();
            this->_time = 0.;
        }

        this->_here.u.pose = *pose;

        return GO_RESULT_OK;
    }

    go_result
    go_motion_queue::interp(go_position * position) {
        if (this->_number > 0) {
            /*
            Only if there's something on the queue do we want to increment
            the queue time. It was set to 0 when the queue was emptied, and
            will not be set to zero when the next move is appended.
             */
            this->_timescale.eval(this->_deltat, NULL);
            this->_time += this->_deltat * this->_timescale.scale;
        }

        if (this->_type == GO_MOTION_JOINT ||
                this->_type == GO_MOTION_UJOINT) {
            return this->interp_joint(position->u.joint);
        }

        if (this->_type == GO_MOTION_WORLD) {
            return this->interp_world(&position->u.pose);
        }

        return GO_RESULT_ERROR;
    }
#ifndef maxit
#define maxit(a,m) {if ((a) > (m)) (m) = (a);}
#endif

    go_result go_motion_queue::stop() {
        go_motion_spec * specptr;
        go_real now; /* time to begin stopping motion */
        go_real endtime; /* time motion is really done */
        go_integer i;
        go_flag neg;
        go_result retval;

        /* don't need to stop an empty or stopping queue */
        if (0 == this->_number) {
            return GO_RESULT_OK;
        }

        /* drop all pending motions from the queue */
        (void) this->drop_pending();

        /* point our attention at the current motion */
        specptr = this->_start;

        /* we want to stop now, the current queue time */
        now = this->_time;

        /* 'endtime' will the the revised stop time */
        endtime = 0.0;

        /* Call go_traj_cj_stop for each joint- or tran/rot motion, so that
        each stops as fast as it can. The longest to stop will set the
        revised stop time, and each will be extended to stop at that
        longest time.  Note that the specptr values are incremental for
        the move, as the original distance passed to go_traj_cj_compute
        was the incremental distance. */

        if (this->_type == GO_MOTION_JOINT) {
            for (i = 0; i < this->_joint_num; i++) {
                (void) go_traj_cj_stop(&specptr->cj[i], now, &specptr->cj[i]);
                maxit(specptr->cj[i].tend, endtime);
            }
            /* now 'endtime' is the longest stop time, so extend all the moves
            to that time, and revise the joint end positions and the
            queue end position 'there' */
            for (i = 0; i < this->_joint_num; i++) {
                neg = specptr->end.u.joint[i] < specptr->start.u.joint[i] ? 1 : 0;
                (void) go_traj_cj_extend(&specptr->cj[i], endtime, &specptr->cj[i]);
                if (neg) {
                    specptr->end.u.joint[i] = specptr->start.u.joint[i] - specptr->cj[i].dtend;
                } else {
                    specptr->end.u.joint[i] = specptr->start.u.joint[i] + specptr->cj[i].dtend;
                }
                /* revise the queue's end position */
                this->_there.u.joint[i] = specptr->end.u.joint[i];
            }
        } else if (this->_type == GO_MOTION_UJOINT) {
            for (i = 0; i < this->_joint_num; i++) {
                neg = specptr->end.u.joint[i] < specptr->start.u.joint[i] ? 1 : 0;
                (void) go_traj_cj_stop(&specptr->cj[i], now, &specptr->cj[i]);
                if (neg) {
                    specptr->end.u.joint[i] = specptr->start.u.joint[i] - specptr->cj[i].dtend;
                } else {
                    specptr->end.u.joint[i] = specptr->start.u.joint[i] + specptr->cj[i].dtend;
                }
                maxit(specptr->cj[i].tend, endtime);
                this->_there.u.joint[i] = specptr->end.u.joint[i];
            }
        } else if (this->_type == GO_MOTION_WORLD) {
            (void) go_traj_cj_stop(&specptr->cj[0], now, &specptr->cj[0]);
            maxit(specptr->cj[0].tend, endtime);
            (void) go_traj_cj_stop(&specptr->cj[1], now, &specptr->cj[1]);
            maxit(specptr->cj[1].tend, endtime);
            /* now 'endtime' is the longest stop time, so extend both tran and
            rot to that time */
            (void) go_traj_cj_extend(&specptr->cj[0], endtime, &specptr->cj[0]);
            (void) go_traj_cj_extend(&specptr->cj[1], endtime, &specptr->cj[1]);
            /* now the tran and rot are extended; now we need to recompute
            the new end translation and rotation */
            retval = interp_world(specptr, endtime, &specptr->end.u.pose);
            if (GO_RESULT_OK != retval) return retval;
            /* else revise the queue's end position */
            this->_there.u.pose = specptr->end.u.pose;
        } else {
            /* queue->type isn't JOINT, UJOINT or WORLD */
            return GO_RESULT_BUG;
        }

        specptr->totalt = endtime;

        return GO_RESULT_OK;
    }

    go_result go_motion_queue::set_id(go_integer id) {
        this->_last_id = id;

        return GO_RESULT_OK;
    }

    go_integer go_motion_queue::last_id() {
        return this->_last_id;
    }

    go_flag go_motion_queue::is_empty() {
        return this->_number == 0;
    }

    go_result go_motion_queue::erase() {
        if (0 == this->_number) {
            return GO_RESULT_OK;
        }

        this->_start++;
        if (this->_start >= this->_endptr) {
            this->_start = this->_startptr;
        }
        this->_number--;

        /* This doesn't decrement the cumulative length for any remaining
        length in the motion spec; you need to have done this before
        calling this function if you're deleting an uncompleted motion. */

        if (this->_number == 0) {
            /* clean up any roundoff */
            this->_here = this->_there;
        }

        return GO_RESULT_OK;
    }

    go_result go_motion_queue::drop_pending() {
        if (this->_number < 2) {
            return GO_RESULT_OK;
        }

        this->_end = this->_start + 1;
        if (this->_end >= this->_endptr) {
            this->_end = this->_startptr;
        }
        this->_number = 1;

        return GO_RESULT_OK;
    }
};
