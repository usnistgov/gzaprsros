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
  genserkins.h
*/

#ifndef GENSERKINS_H
#define GENSERKINS_H

#include "gotypes.h"		/* go_result, go_integer */
#include "gomath.h"		/* go_pose */
#include "_gokin.h"		/* go_kin_type */

#ifdef __cplusplus
extern "C" {
#endif
#if 0
}
#endif

/*! 
  The maximum number of joints supported by the general serial
  kinematics. Make this at least 6; a device can have fewer than these.
*/
#define GENSER_MAX_JOINTS 8

typedef struct {
  go_link links[GENSER_MAX_JOINTS]; /*!< The link description of the device. */
  go_integer link_num;		/*!< How many are actually present. */
  go_integer iterations;	/*!< How many iterations were actually used to compute the inverse kinematics. */
  go_integer max_iterations;	/*!< Number of iterations after which to give up and report an error. */
} genser_struct;

extern go_integer genser_kin_size(void); 

extern go_result genser_kin_init(void *kins); 

extern const char *genser_kin_get_name(void); 

extern go_integer genser_kin_num_joints(void *kins);

extern go_result genser_kin_fwd(void *kins,
				const go_real *joint,
				go_pose *world);

extern go_result genser_kin_inv(void *kins,
				const go_pose *world,
				go_real *joint);

extern go_kin_type genser_kin_get_type(void *kins); 

extern go_result genser_kin_set_parameters(void *kins, go_link *params, go_integer num); 

extern go_result genser_kin_get_parameters(void *kins, go_link *params, go_integer num); 

extern go_result genser_kin_jac_inv(void *kins,
				    const go_pose *pos,
				    const go_vel *vel,
				    const go_real *joints, 
				    go_real *jointvels); 


extern go_result genser_kin_jac_fwd(void *kins,
				    const go_real *joints,
				    const go_real *jointvels,
				    const go_pose *pos, 
				    go_vel *vel); 

/*
  Extras, not callable using go_kin_ wrapper but if you know you have
  linked in these kinematics, go ahead and call these for your ad hoc
  purposes.
*/

extern go_result genser_kin_compute_jfwd(go_link *link_params, int link_number, go_matrix *Jfwd, go_pose *T_L_0);

extern go_result genser_kin_compute_jinv(go_matrix *Jfwd, go_matrix *Jinv, go_vector *weights);

/*! Returns the number of iterations used during the last call to the
  inverse kinematics functions */
extern go_integer genser_kin_inv_iterations(genser_struct *genser);

/*! Sets the maximum number of iterations to use in future calls to
  the inverse kinematics functions, after which an error will be
  reported */
extern go_result genser_kin_inv_set_max_iterations(genser_struct *genser, go_integer i);

/*! Returns the maximum number of iterations that will be used to
 compute inverse kinematics functions */
extern go_integer genser_kin_inv_get_max_iterations(genser_struct *genser);

#if 0
{
#endif
#ifdef __cplusplus
}
#endif

#endif
