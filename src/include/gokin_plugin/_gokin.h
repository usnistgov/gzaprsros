/*
  DISCLAIMER:
  This software was produced by the National Institute of Standards
  and Technology (NIST), an agency of the U.S. government, and by statute is
  not subject to copyright in the United States.  Recipients of this software
  assume all responsibility associated with its operation, modification,
  maintenance, and subsequent redistribution.

  See NIST Administration Manual 4.09.07 b and Appendix I. 
*/

#ifndef GO_KIN_H
#define GO_KIN_H

#include "gotypes.h"		/* go_result, go_real */
#include "gomath.h"		/* go_pose */

#ifdef __cplusplus
extern "C" {
#endif
#if 0
}
#endif

/*
  The type of kinematics used.
  
  GO_KIN_IDENTITY means that the joints and world coordinates are the
  same, as for slideway machines (XYZ milling machines). The EMC will allow
  changing from joint to world mode and vice versa. Also, the EMC will set
  the actual world position to be the actual joint positions (not commanded)
  by calling the forward kinematics each trajectory cycle.

  GO_KIN_FORWARD_ONLY means that only the forward kinematics exist.
  Since the EMC requires at least the inverse kinematics, this should simply
  terminate the EMC.

  GO_KIN_INVERSE_ONLY means that only the inverse kinematics exist.
  The forwards won't be called, and the EMC will only allow changing from
  joint to world mode at the home position.

  GO_KIN_BOTH means that both the forward and inverse kins are defined.
  Like GO_KIN_IDENTITY, the EMC will allow changing between world and
  joint modes. However, the kins are assumed to be somewhat expensive
  computationally, and the forwards won't be called at the trajectory rate
  to compute actual world coordinates from actual joint values.
*/

typedef enum {
  GO_KIN_IDENTITY = 1,	/* forward=inverse, both well-behaved */
  GO_KIN_FORWARD_ONLY,	/* forward but no inverse */
  GO_KIN_INVERSE_ONLY,	/* inverse but no forward */
  GO_KIN_BOTH		/* forward and inverse both */
} go_kin_type;

/* This function gets a descriptive and hopefully unique name so
   that the controller can adjust the meaning of the parameters passed
   to go_kin_set_parameters() */
extern const char * go_kin_get_name(void);
enum {GO_KIN_NAME_LEN = 80};	/* how long a name can be */

/* go_kin_size returns how big the kinematics structure is for
   the particular implementation */
extern go_integer go_kin_size(void);

/* pass the name of the kins you want */
extern go_result go_kin_select(const char * name);

/* go_kin_init() initializes the kinematics pointed to by 'kins' */
extern go_result go_kin_init(void * kins);

/* returns the actual number of joints, possibly less than the max
   supported depending on how many links were present in the call
   to go_kin_set_parameters */
extern go_integer go_kin_num_joints(void * kins);

/* 
   The forward kinematics take joint values and determine world coordinates,
   using any forward kinematics flags to resolve any ambiguities. The inverse
   flags are set to indicate their value appropriate to the joint values
   passed in.
*/
extern go_result go_kin_fwd(void * kins,
			    const go_real *joint,
			    go_pose * world);

/*
  The inverse kinematics take world coordinates and determine joint values,
  given the inverse kinematics flags to resolve any ambiguities. The forward
  flags are set to indicate their value appropriate to the world coordinates
  passed in.
*/
extern go_result go_kin_inv(void * kins,
			    const go_pose * world,
			    go_real *joints);

extern go_kin_type go_kin_get_type(void * kins);

extern go_result go_kin_set_parameters(void * kins, go_link * params, go_integer num);
extern go_result go_kin_get_parameters(void * kins, go_link * params, go_integer num);

extern go_result go_kin_jac_inv(void * kins,
				const go_pose * pos,
				const go_vel * vel,
				const go_real * joints,
				go_real * jointvels);

extern go_result go_kin_jac_fwd(void * kins,
				const go_real * joints,
				const go_real * jointvels,
				const go_pose * pos,
				go_vel * vel);

extern go_result go_kin_set_flags(void *kins,
				  go_flag fflags,
				  go_flag iflags);

extern go_result go_kin_get_flags(void *kins,
				  go_flag *fflags,
				  go_flag *iflags);

#if 0
{
#endif
#ifdef __cplusplus
}
#endif

#endif
