/*
  DISCLAIMER:
  This software was produced by the National Institute of Standards
  and Technology (NIST), an agency of the U.S. government, and by statute is
  not subject to copyright in the United States.  Recipients of this software
  assume all responsibility associated with its operation, modification,
  maintenance, and subsequent redistribution.

  See NIST Administration Manual 4.09.07 b and Appendix I.
*/

/*
  genserkins.c

  These are the forward and inverse kinematic functions for a general
  serial-link manipulator. Thanks to Herman Bruyninckx and John
  Hallam at http://www.roble.info/ for this.
*/

#include <math.h>
#include "gokin_plugin/gotypes.h"		/* go_result, go_integer */
#include "gokin_plugin/gomath.h"		/* go_pose */
#include "gokin_plugin/_gokin.h"		/* go_kin_type */
#include "gokin_plugin/genserkins.h"		/* these decls */

/*
  Set ROTATE_JACOBIANS_BACK if you want the Jacobian matrix to be
  expressed in the {0} frame. This means Cartesian velocities will be
  assumed relative to the {0} frame. If this is not defined, then the
  Jacobian will be relative to the final frame, and Cartesian speeds
  are assumed to be in the final frame.

  Normally, things are expressed in the {0} frame, so rotating
  Jacobians back is consistent with this.
*/
#define ROTATE_JACOBIANS_BACK 1

#if GENSER_MAX_JOINTS < 6
#error GENSER_MAX_JOINTS must be at least 6; fix genserkins.h
#endif

enum {GENSER_DEFAULT_MAX_ITERATIONS = 100};

go_integer genser_kin_size(void)
{
    return (go_integer) sizeof(genser_struct);
}

go_result genser_kin_init(void * kins)
{
    genser_struct * genser = (genser_struct *) kins;
    go_integer t;

    /* clear them all and make them revolute joints */
    for (t = 0; t < GENSER_MAX_JOINTS; t++) {
        genser->links[t].u.dh.a = 0;
        genser->links[t].u.dh.alpha = 0;
        genser->links[t].u.dh.d = 0;
        genser->links[t].u.dh.theta = 0;
        genser->links[t].type = GO_LINK_DH;
        genser->links[t].quantity = GO_QUANTITY_ANGLE;
        go_body_init(&genser->links[t].body);
    }

    /* set a select few to make it PUMA-like */

    genser->links[1].u.dh.alpha = -GO_PI_2;

    genser->links[2].u.dh.a = 0.300;
    genser->links[2].u.dh.d = 0.070;

    genser->links[3].u.dh.a = 0.050;
    genser->links[3].u.dh.alpha = -GO_PI_2;
    genser->links[3].u.dh.a = 0.400;

    genser->links[4].u.dh.alpha = GO_PI_2;

    genser->links[5].u.dh.alpha = -GO_PI_2;

    genser->link_num = 6;
    genser->iterations = 0;
    genser->max_iterations = GENSER_DEFAULT_MAX_ITERATIONS;

    return GO_RESULT_OK;
}

const char * genser_kin_get_name(void)
{
    return "genserkins";
}

go_integer genser_kin_num_joints(void * kins)
{
    genser_struct * genser = (genser_struct *) kins;

    return genser->link_num;
}

go_result genser_kin_set_parameters(void * kins, go_link * params, go_integer num)
{
    genser_struct * genser = (genser_struct *) kins;
    go_integer t;

    if (num > GENSER_MAX_JOINTS) return GO_RESULT_BAD_ARGS;

    for (t = 0; t < num; t++) {
        /* we only handle serial-type link params */
        if (params[t].type != GO_LINK_DH &&
                params[t].type != GO_LINK_PP &&
                params[t].type != GO_LINK_URDF) return GO_RESULT_BAD_ARGS;
        genser->links[t] = params[t];
    }
    genser->link_num = num;

    return GO_RESULT_OK;
}

go_result genser_kin_get_parameters(void * kins, go_link * params, go_integer num)
{
    genser_struct * genser = (genser_struct *) kins;
    go_integer t;

    /* check if they have enough space to hold the params */
    if (num < genser->link_num) return GO_RESULT_BAD_ARGS;

    for (t = 0; t < genser->link_num; t++) {
        params[t] = genser->links[t];
    }

    return GO_RESULT_OK;
}

/* forward decls for these extended functions */

extern go_result genser_kin_compute_jfwd(go_link *link_params, int link_number, go_matrix *Jfwd, go_pose *T_L_0);
extern go_result genser_kin_compute_jinv(go_matrix *Jfwd, go_matrix *Jinv, go_real *weights);

go_result genser_kin_jac_inv(void * kins,
                             const go_pose * pos,
                             const go_vel * vel,
                             const go_real * joints,
                             go_real * jointvels)
{
    genser_struct * genser = (genser_struct *) kins;
    GO_MATRIX_DECLARE(Jfwd, Jfwd_stg, 6, GENSER_MAX_JOINTS);
    GO_MATRIX_DECLARE(Jinv, Jinv_stg, GENSER_MAX_JOINTS, 6);
    go_pose T_L_0;
#ifdef ROTATE_JACOBIANS_BACK
#else
    go_quat Rinv;
#endif
    go_link linkout[GENSER_MAX_JOINTS];
    go_real vw[6];
    go_vector weights[GENSER_MAX_JOINTS];
    go_integer link;
    go_result retval;

    go_matrix_init(Jfwd, Jfwd_stg, 6, genser->link_num);
    go_matrix_init(Jinv, Jinv_stg, GENSER_MAX_JOINTS, 6);

    for (link = 0; link < genser->link_num; link++) {
        go_link_joint_set(&genser->links[link], joints[link], &linkout[link]);
        weights[link] = (GO_QUANTITY_LENGTH == genser->links[link].quantity ? genser->links[link].body.mass : GO_QUANTITY_ANGLE == genser->links[link].quantity ? genser->links[link].body.inertia[2][2] : 1);
    }
    retval = genser_kin_compute_jfwd(linkout, genser->link_num, &Jfwd, &T_L_0);
    if (GO_RESULT_OK != retval) return retval;
    retval = genser_kin_compute_jinv(&Jfwd, &Jinv, weights);
    if (GO_RESULT_OK != retval) return retval;

#ifdef ROTATE_JACOBIANS_BACK
#else
    go_quat_inv(&T_L_0.rot, &Rinv);
    go_quat_cart_mult(&Rinv, &vel->v, &vel->v);
    go_quat_cart_mult(&Rinv, &vel->w, &vel->w);
#endif

    vw[0] = vel->v.x;
    vw[1] = vel->v.y;
    vw[2] = vel->v.z;
    vw[3] = vel->w.x;
    vw[4] = vel->w.y;
    vw[5] = vel->w.z;

    return go_matrix_vector_mult(&Jinv, vw, jointvels);
}

go_result genser_kin_jac_fwd(void * kins,
                             const go_real * joints,
                             const go_real * jointvels,
                             const go_pose * pos,
                             go_vel * vel)
{
    genser_struct * genser = (genser_struct *) kins;
    GO_MATRIX_DECLARE(Jfwd, Jfwd_stg, 6, GENSER_MAX_JOINTS);
    go_pose T_L_0;
#ifdef ROTATE_JACOBIANS_BACK
#else
    go_quat Rinv;
#endif
    go_link linkout[GENSER_MAX_JOINTS];
    go_real vw[6];
    go_integer link;
    go_result retval;

    go_matrix_init(Jfwd, Jfwd_stg, 6, genser->link_num);

    for (link = 0; link < genser->link_num; link++) {
        retval = go_link_joint_set(&genser->links[link], joints[link], &linkout[link]);
        if (GO_RESULT_OK != retval) return retval;
    }

    retval = genser_kin_compute_jfwd(linkout, genser->link_num, &Jfwd, &T_L_0);
    if (GO_RESULT_OK != retval) return retval;

    go_matrix_vector_mult(&Jfwd, jointvels, vw);
    vel->v.x = vw[0];
    vel->v.y = vw[1];
    vel->v.z = vw[2];
    vel->w.x = vw[3];
    vel->w.y = vw[4];
    vel->w.z = vw[5];

#ifdef ROTATE_JACOBIANS_BACK
#else
    go_quat_inv(&T_L_0.rot, &Rinv);
    go_quat_cart_mult(&Rinv, &vel->v, &vel->v);
    go_quat_cart_mult(&Rinv, &vel->w, &vel->w);
#endif

    return GO_RESULT_OK;
}

go_result genser_kin_fwd(void * kins,
                         const go_real *joints,
                         go_pose * pos)
{
    genser_struct * genser = (genser_struct *) kins;
    go_link linkout[GENSER_MAX_JOINTS];
    go_integer link;
    go_result retval;

    for (link = 0; link < genser->link_num; link++) {
        retval = go_link_joint_set(&genser->links[link], joints[link], &linkout[link]);
        if (GO_RESULT_OK != retval) return retval;
    }

    retval = go_link_pose_build(linkout, genser->link_num, pos);
    if (GO_RESULT_OK != retval) return retval;

    return GO_RESULT_OK;
}

go_result genser_kin_inv(void  * kins,
                         const go_pose * pos,
                         go_real * joints)
{
    genser_struct * genser = (genser_struct *) kins;
    GO_MATRIX_DECLARE(Jfwd, Jfwd_stg, 6, GENSER_MAX_JOINTS);
    GO_MATRIX_DECLARE(Jinv, Jinv_stg, GENSER_MAX_JOINTS, 6);
    go_pose T_L_0;
    go_real dvw[6];
    go_real jest[GENSER_MAX_JOINTS];
    go_real dj[GENSER_MAX_JOINTS];
    go_pose pest, pestinv, Tdelta;
    go_rvec rvec;
    go_cart cart;
    go_link linkout[GENSER_MAX_JOINTS];
    go_vector weights[GENSER_MAX_JOINTS];
    go_integer link;
    go_integer smalls;
    go_result retval;

    go_matrix_init(Jfwd, Jfwd_stg, 6, genser->link_num);
    go_matrix_init(Jinv, Jinv_stg, genser->link_num, 6);

    /* jest[] is a copy of joints[], which is the joint estimate */
    for (link = 0; link < genser->link_num; link++) {
        jest[link] = joints[link];
    }

    for (genser->iterations = 0; genser->iterations < genser->max_iterations; genser->iterations++) {
        /* update the Jacobians */
        for (link = 0; link < genser->link_num; link++) {
            go_link_joint_set(&genser->links[link], jest[link], &linkout[link]);
            weights[link] = (GO_QUANTITY_LENGTH == genser->links[link].quantity ? genser->links[link].body.mass : GO_QUANTITY_ANGLE == genser->links[link].quantity ? genser->links[link].body.inertia[2][2] : 1);
        }
        retval = genser_kin_compute_jfwd(linkout, genser->link_num, &Jfwd, &T_L_0);
        if (GO_RESULT_OK != retval)
            return retval;
        retval = genser_kin_compute_jinv(&Jfwd, &Jinv, weights);
        if (GO_RESULT_OK != retval)
            return retval;

        /* pest is the resulting pose estimate given joint estimate */
        genser_kin_fwd(kins, jest, &pest);
        /* pestinv is its inverse */
        go_pose_inv(&pest, &pestinv);
        /*
      Tdelta is the incremental pose from pest to pos, such that

      0        L         0
      . pest *  Tdelta =  pos, or
      L        L         L

      L         L          0
      .Tdelta =  pestinv *  pos
      L         0          L
    */
        go_pose_pose_mult(&pestinv, pos, &Tdelta);

        /*
      We need Tdelta in 0 frame, not pest frame, so rotate it
      back. Since it's effectively a velocity, we just rotate it, and
      don't translate it.
    */

#ifdef ROTATE_JACOBIANS_BACK
        go_quat_cart_mult(&pest.rot, &Tdelta.tran, &cart);
#else
        cart = Tdelta.tran;
#endif
        dvw[0] = cart.x;
        dvw[1] = cart.y;
        dvw[2] = cart.z;

        /* to rotate the rotation differential, convert it to a
       velocity and rotate that */
        go_quat_rvec_convert(&Tdelta.rot, &rvec);
        cart.x = rvec.x;
        cart.y = rvec.y;
        cart.z = rvec.z;
#ifdef ROTATE_JACOBIANS_BACK
        go_quat_cart_mult(&pest.rot, &cart, &cart);
#endif
        dvw[3] = cart.x;
        dvw[4] = cart.y;
        dvw[5] = cart.z;

        /* push the Cartesian velocity vector through the inverse Jacobian */
        go_matrix_vector_mult(&Jinv, dvw, dj);

        /* check for small joint increments, if so we're done */
        for (link = 0, smalls = 0; link < genser->link_num; link++) {
            if (GO_QUANTITY_LENGTH == linkout[link].quantity) {
                if (GO_TRAN_SMALL(dj[link]))
                    smalls++;
            } else {
                if (GO_ROT_SMALL(dj[link]))
                    smalls++;
            }
        }
        if (smalls == genser->link_num) {
            /* converged, copy jest[] out */
            for (link = 0; link < genser->link_num; link++) {
                joints[link] = jest[link];
            }
            return GO_RESULT_OK;
        }

        /* else keep iterating */
        for (link = 0; link < genser->link_num; link++) {
            jest[link] += dj[link];
            if (GO_QUANTITY_ANGLE == linkout[link].quantity) {
                if (jest[link] > GO_PI) jest[link] -= GO_2_PI;
                else if (jest[link] < -GO_PI) jest[link] += GO_2_PI;
            }
        }
    } /* for (iterations) */

    return GO_RESULT_ERROR;
}

go_kin_type genser_kin_get_type(void * kins)
{
    return GO_KIN_BOTH;
}

/*
  Extras, not callable using go_kin_ wrapper but if you know you have
  linked in these kinematics, go ahead and call these for your ad hoc
  purposes.
*/

go_result genser_kin_compute_jfwd(go_link * link_params, int link_number, go_matrix * Jfwd, go_pose * T_L_0)
{
    GO_MATRIX_DECLARE(Jv, Jvstg, 3, GENSER_MAX_JOINTS);
    GO_MATRIX_DECLARE(Jw, Jwstg, 3, GENSER_MAX_JOINTS);
    GO_MATRIX_DECLARE(R_i_ip1, R_i_ip1stg, 3, 3);
    GO_MATRIX_DECLARE(scratch, scratchstg, 3, GENSER_MAX_JOINTS);
    GO_MATRIX_DECLARE(R_inv, R_invstg, 3, 3);
    go_pose pose;
    go_quat quat;
    go_vector P_ip1_i[3];
    int row, col;

    /* init matrices to possibly smaller size */
    go_matrix_init(Jv, Jvstg, 3, link_number);
    go_matrix_init(Jw, Jwstg, 3, link_number);
    go_matrix_init(R_i_ip1, R_i_ip1stg, 3, 3);
    go_matrix_init(scratch, scratchstg, 3, link_number);
    go_matrix_init(R_inv, R_invstg, 3, 3);

    if (GO_LINK_URDF == link_params[0].type) {
        /* move about general axis */
        if (GO_QUANTITY_LENGTH == link_params[0].quantity) {
            Jv.el[0][0] = link_params[0].u.urdf.axis.x;
            Jv.el[1][0] = link_params[0].u.urdf.axis.y;
            Jv.el[2][0] = link_params[0].u.urdf.axis.z;
            Jw.el[0][0] = 0, Jw.el[1][0] = 0, Jw.el[2][0] = 0;
        } else {
            Jw.el[0][0] = link_params[0].u.urdf.axis.x;
            Jw.el[1][0] = link_params[0].u.urdf.axis.y;
            Jw.el[2][0] = link_params[0].u.urdf.axis.z;
            Jv.el[0][0] = 0, Jv.el[1][0] = 0, Jv.el[2][0] = 0;
        }
    } else {
        /* rotation or translate about Z by convention */
        Jv.el[0][0] = 0, Jv.el[1][0] = 0, Jv.el[2][0] = (GO_QUANTITY_LENGTH == link_params[0].quantity ? 1 : 0);
        Jw.el[0][0] = 0, Jw.el[1][0] = 0, Jw.el[2][0] = (GO_QUANTITY_ANGLE == link_params[0].quantity ? 1 : 0);
    }

    /* initialize inverse rotational transform */
    if (GO_LINK_DH == link_params[0].type) {
        go_dh_pose_convert(&link_params[0].u.dh, &pose);
    } else if (GO_LINK_PP == link_params[0].type) {
        pose = link_params[0].u.pp.pose;
    } else if (GO_LINK_URDF == link_params[0].type) {
        pose = link_params[0].u.urdf.pose;
    } else {
        return GO_RESULT_IMPL_ERROR;
    }

    *T_L_0 = pose;

    for (col = 1; col < link_number; col++) {
        /* T_ip1_i */
        if (GO_LINK_DH == link_params[col].type) {
            go_dh_pose_convert(&link_params[col].u.dh, &pose);
        } else if (GO_LINK_PP == link_params[col].type) {
            pose = link_params[col].u.pp.pose;
        } else if (GO_LINK_URDF == link_params[col].type) {
            pose = link_params[col].u.urdf.pose;
        } else {
            return GO_RESULT_IMPL_ERROR;
        }

        go_cart_vector_convert(&pose.tran, P_ip1_i);
        go_quat_inv(&pose.rot, &quat);
        go_quat_matrix_convert(&quat, &R_i_ip1);

        /* Jv */
        go_matrix_vector_cross(&Jw, P_ip1_i, &scratch);
        go_matrix_matrix_add(&Jv, &scratch, &scratch);
        go_matrix_matrix_mult(&R_i_ip1, &scratch, &Jv);

        if (GO_LINK_URDF == link_params[col].type) {
            if (GO_QUANTITY_LENGTH == link_params[col].quantity) {
                Jv.el[0][col] = link_params[col].u.urdf.axis.x;
                Jv.el[1][col] = link_params[col].u.urdf.axis.y;
                Jv.el[2][col] = link_params[col].u.urdf.axis.z;
            } else {
                Jv.el[0][col] = 0, Jv.el[1][col] = 0, Jv.el[2][col] = 0;
            }
        } else {
            Jv.el[0][col] = 0, Jv.el[1][col] = 0, Jv.el[2][col] = (GO_QUANTITY_LENGTH == link_params[col].quantity ? 1 : 0);
        }

        /* Jw */
        go_matrix_matrix_mult(&R_i_ip1, &Jw, &Jw);

        if (GO_LINK_URDF == link_params[col].type) {
            if (GO_QUANTITY_ANGLE == link_params[col].quantity) {
                Jw.el[0][col] = link_params[col].u.urdf.axis.x;
                Jw.el[1][col] = link_params[col].u.urdf.axis.y;
                Jw.el[2][col] = link_params[col].u.urdf.axis.z;
            } else {
                Jw.el[0][col] = 0, Jw.el[1][col] = 0, Jw.el[2][col] = 0;
            }
        } else {
            Jw.el[0][col] = 0, Jw.el[1][col] = 0, Jw.el[2][col] = (GO_QUANTITY_ANGLE == link_params[col].quantity ? 1 : 0);
        }

        if (GO_LINK_DH == link_params[col].type) {
            go_dh_pose_convert(&link_params[col].u.dh, &pose);
        } else if (GO_LINK_PP == link_params[col].type) {
            pose = link_params[col].u.pp.pose;
        } else if (GO_LINK_URDF == link_params[col].type) {
            pose = link_params[col].u.urdf.pose;
        } else {
            return GO_RESULT_IMPL_ERROR;
        }
        go_pose_pose_mult(T_L_0, &pose, T_L_0);
    }

#ifdef ROTATE_JACOBIANS_BACK
    /* rotate back into {0} frame */
    go_quat_matrix_convert(&T_L_0->rot, &R_inv);
    go_matrix_matrix_mult(&R_inv, &Jv, &Jv);
    go_matrix_matrix_mult(&R_inv, &Jw, &Jw);
#endif

    /* put Jv atop Jw in J */
    for (row = 0; row < 6; row++) {
        for (col = 0; col < link_number; col++) {
            if (row < 3) {
                Jfwd->el[row][col] = Jv.el[row][col];
            } else {
                Jfwd->el[row][col] = Jw.el[row - 3][col];
            }
        }
    }

    return GO_RESULT_OK;
}

go_result genser_kin_compute_jinv(go_matrix * Jfwd, go_matrix * Jinv, go_vector *weights)
{
    go_integer row, col;
    go_result retval;

    /* compute inverse, or pseudo-inverse */
    if (Jfwd->rows == Jfwd->cols) {
        retval = go_matrix_inv(Jfwd, Jinv);
        if (GO_RESULT_OK != retval)
            return retval;
    } else if (Jfwd->rows < Jfwd->cols) {
        /* underdetermined, optimize on smallest sum of square of speeds */
        /* JT(JJT)inv */
        GO_MATRIX_DECLARE(JT, JTstg, GENSER_MAX_JOINTS, 6);
        GO_MATRIX_DECLARE(JJT, JJTstg, 6, 6);
        GO_MATRIX_DECLARE(Minv, Minvstg, GENSER_MAX_JOINTS, GENSER_MAX_JOINTS);

        go_matrix_init(JT, JTstg, Jfwd->cols, Jfwd->rows);
        go_matrix_init(JJT, JJTstg, Jfwd->rows, Jfwd->rows);
        go_matrix_init(Minv, Minvstg, Jfwd->cols, Jfwd->cols);

        for (row = 0; row < Jfwd->cols; row++) {
            for (col = 0; col < Jfwd->cols; col++) {
                if (row == col) {
                    Minv.el[row][col] = weights[row] > 0.0 ? 1.0 / weights[row] : 1.0;
                } else {
                    Minv.el[row][col] = 0.0;
                }
            }
        }

        go_matrix_transpose(Jfwd, &JT);
        go_matrix_matrix_mult(&Minv, &JT, &JT);
        go_matrix_matrix_mult(Jfwd, &JT, &JJT);
        retval = go_matrix_inv(&JJT, &JJT);
        if (GO_RESULT_OK != retval) {
            return retval;
        }
        go_matrix_matrix_mult(&JT, &JJT, Jinv);
        go_matrix_matrix_mult(&Minv, Jinv, Jinv);
    } else {
        /* overdetermined, do least-squares best fit */
        /* (JTJ)invJT */
        GO_MATRIX_DECLARE(JT, JTstg, GENSER_MAX_JOINTS, 6);
        GO_MATRIX_DECLARE(JTJ, JTJstg, GENSER_MAX_JOINTS, GENSER_MAX_JOINTS);

        go_matrix_init(JT, JTstg, Jfwd->cols, Jfwd->rows);
        go_matrix_init(JTJ, JTJstg, Jfwd->cols, Jfwd->cols);
        go_matrix_transpose(Jfwd, &JT);
        go_matrix_matrix_mult(&JT, Jfwd, &JTJ);
        retval = go_matrix_inv(&JTJ, &JTJ);
        if (GO_RESULT_OK != retval)
            return retval;
        go_matrix_matrix_mult(&JTJ, &JT, Jinv);
    }

    return GO_RESULT_OK;
}

go_integer genser_kin_inv_iterations(genser_struct * genser)
{
    return genser->iterations;
}

go_result genser_kin_inv_set_max_iterations(genser_struct * genser, go_integer i)
{
    if (i <= 0) return GO_RESULT_ERROR;
    genser->max_iterations = i;
    return GO_RESULT_OK;
}

go_integer genser_kin_inv_get_max_iterations(genser_struct * genser)
{
    return genser->max_iterations;
}
