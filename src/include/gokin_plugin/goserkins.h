#ifndef _GOSERKINS_H
#define _GOSERKINS_H

#include <stdio.h>
#include <stdlib.h>
#include <inifile.h>
#include <go.h>
#include <genserkins.h>

#define NUM_JOINTS 6
#ifndef DIGITS_IN
#define DIGITS_IN(x) (sizeof(x) * 3 + 1)
#endif


inline int
ini_load(char * inifile_name,
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

inline void print_params(go_link *link_params, int link_number)
{
    go_rpy rpy;
    int t;

    for (t = 0; t < link_number; t++) {
        if (GO_LINK_DH == link_params[t].type) {
            fprintf(stdout, "%d: %.3f %.3f %.3f %.3f\n", t+1,
                    link_params[t].u.dh.a,
                    link_params[t].u.dh.alpha,
                    link_params[t].u.dh.d,
                    link_params[t].u.dh.theta);
        } else if (GO_LINK_PP == link_params[t].type) {
            go_quat_rpy_convert(&link_params[t].u.pp.pose.rot, &rpy);
            fprintf(stdout, "%d: %.3f %.3f %.3f / %.3f %.3f %.3f\n", t+1,
                    link_params[t].u.urdf.pose.tran.x,
                    link_params[t].u.urdf.pose.tran.y,
                    link_params[t].u.urdf.pose.tran.z,
                    rpy.r, rpy.p, rpy.y);
        } else if (GO_LINK_URDF == link_params[t].type) {
            go_quat_rpy_convert(&link_params[t].u.urdf.pose.rot, &rpy);
            fprintf(stdout, "%d: %.3f %.3f %.3f / %.3f %.3f %.3f / %.3f %.3f %.3f\n", t+1,
                    link_params[t].u.urdf.pose.tran.x,
                    link_params[t].u.urdf.pose.tran.y,
                    link_params[t].u.urdf.pose.tran.z,
                    rpy.r, rpy.p, rpy.y,
                    link_params[t].u.urdf.axis.x,
                    link_params[t].u.urdf.axis.y,
                    link_params[t].u.urdf.axis.z);
        } else {
            fprintf(stdout, "unknown\n");
        }
    }
}
#ifdef TEST
int main(void)
{
    char inifile_name[] = "fanuc-lrmate-200id.ini";
    double m_per_length_units;
    double rad_per_angle_units;
    int link_number;
    go_link link_params[GENSER_MAX_JOINTS];
    genser_struct kins;
    go_real joints[GENSER_MAX_JOINTS];
    go_pose pose;
    go_rpy rpy;
    int t;

    if (GO_RESULT_OK != go_init()) {
        fprintf(stdout, "can't init gomotion\n");
        return 1;
    }

    if (GO_RESULT_OK != genser_kin_init(&kins)) {
        fprintf(stdout, "can't init general serial kinematics\n");
        return 1;
    }

    if (0 != ini_load(inifile_name,
                      &m_per_length_units,
                      &rad_per_angle_units,
                      &link_number,
                      link_params)) {
        fprintf(stdout, "can't load ini file %s\n", inifile_name);
        return 1;
    }

    print_params(link_params, link_number);

    if (GO_RESULT_OK != genser_kin_set_parameters(&kins, link_params, link_number)) {
        fprintf(stdout, "can't set kinematics parameters\n");
        return 1;
    }

    for (t = 0; t < sizeof(joints)/sizeof(*joints); t++) {
        joints[t] = 0;
    }

    for (t = 0; t < sizeof(joints)/sizeof(*joints); t++) {
        printf("%f ", (double) joints[t]);
    }
    printf("\n");

    if (GO_RESULT_OK != genser_kin_fwd(&kins, joints, &pose)) {
        fprintf(stdout, "Can't run general serial forward kinematics\n");
        return 1;
    }

    go_quat_rpy_convert(&pose.rot, &rpy);

    printf("%f %f %f / %f %f %f\n", (double) pose.tran.x, (double) pose.tran.y, (double) pose.tran.z, (double) rpy.r, (double) rpy.p, (double) rpy.y);

    if (GO_RESULT_OK != genser_kin_inv(&kins, &pose, joints)) {
        fprintf(stdout, "Can't run general serial inverse kinematics\n");
        return 1;
    }

    for (t = 0; t < sizeof(joints)/sizeof(*joints); t++) {
        printf("%f ", (double) joints[t]);
    }
    printf("\n");

    return 0;
}
#endif
#endif
