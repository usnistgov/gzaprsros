//

// fanuc_lrmate200id.cpp
//

/*
DISCLAIMER:
This software was produced by the National Institute of Standards
and Technology (NIST), an agency of the U.S. government, and by statute is
not subject to copyright in the United States.  Recipients of this software
assume all responsibility associated with its operation, modification,
maintenance, and subsequent redistribution.

See NIST Administration Manual 4.09.07 b and Appendix I.
*/

#include "gokin_fanuc_plugin/fanuc_lrmate200id.h"
#include <aprs_headers/IKinematic.h>
#define MATH_DEFINES
#include <math.h>

#ifndef HAVE_SINCOS
#define HAVE_SINCOS
void sincos (double x, double *sx, double *cx)
{
  *sx = sin(x);
  *cx = cos(x);
}
#endif

////////////////////////////////////////////////////////////////////////////////
fanuc_lrmate200id::fanuc_lrmate200id(void)
{
  kins.tk.a1     = 0.050;
  kins.tk.a2     = 0.330;
  kins.tk.a3     = 0.035;
  kins.tk.d2     = 0;
  kins.tk.d3     = 0;
  kins.tk.d4     = 0.335;
  kins.tk.iflags = 0;

  kins.t7.position.x = 0;
  kins.t7.position.y = 0;
  kins.t7.position.z = WRIST_OFFSET;
  kins.t7.rotation.w = 1;
  kins.t7.rotation.x = 0;
  kins.t7.rotation.y = 0;
  kins.t7.rotation.z = 0;

  kins.t7_inv = _poseInv(kins.t7);
}
////////////////////////////////////////////////////////////////////////////////
fanuc_lrmate200id::~fanuc_lrmate200id(void)
{ }

////////////////////////////////////////////////////////////////////////////////
int  fanuc_lrmate200id::fwd_kin (std::vector<double>jnts, urdf::Pose &pose)
{
#if 0
    // transform angles from degree to radians
    std::transform(thetas.begin( ), thetas.end( ), thetas.begin( ),
                   std::bind1st(std::multiplies<double>( ), M_PI / 180.0) );
#endif
    if(jnts.size() !=6)
        return RCS::IKinematic::Bad_Joints_Size;

    if(fanuc_lrmate200id_fwd_kin(&jnts[0], pose)!=0)
        return -1;
    return 0;
}



////////////////////////////////////////////////////////////////////////////////
urdf::Pose three21_kin_fwd (three21_kin & kins,
  const double *                          joints)
{
  double    s1, s2, s3, s4, s5, s6;
  double    c1, c2, c3, c4, c5, c6;
  double    s23;
  double    c23;
  double    t1, t2, t3, t4, t5;
  double    sum_sq, k, d23;
  RosMatrix hom;

  sincos(joints[0], &s1, &c1);
  sincos(joints[1], &s2, &c2);
  sincos(joints[2], &s3, &c3);
  sincos(joints[3], &s4, &c4);
  sincos(joints[4], &s5, &c5);
  sincos(joints[5], &s6, &c6);

  /* sin,cos(2+3) */
  s23 = c2 * s3 + s2 * c3;
  c23 = c2 * c3 - s2 * s3;

  /* Calculate terms to be used in definition of... */
  /* first column of rotation matrix.               */
  t1 = c4 * c5 * c6 - s4 * s6;
  t2 = s23 *  s5 * c6;
  t3 = s4 * c5 * c6 + c4 * s6;
  t4 = c23 * t1 - t2;
  t5 = c23 * s5 * c6;

  /* Define first column of rotation matrix */
  hom[4 * 0 + 0] = c1 * t4 + s1 * t3;
  hom[4 * 1 + 0] = s1 * t4 - c1 * t3;
  hom[4 * 2 + 0] = -s23 * t1 - t5;

  /* Calculate terms to be used in definition of...  */
  /* second column of rotation matrix.               */
  t1 = -c4 * c5 * s6 - s4 * c6;
  t2 = s23 * s5 * s6;
  t3 = c4 * c6 - s4 * c5 * s6;
  t4 = c23 * t1 + t2;
  t5 = c23 * s5 * s6;

  /* Define second column of rotation matrix */
  hom[4 * 0 + 1] = c1 * t4 + s1 * t3;
  hom[4 * 1 + 1] = s1 * t4 - c1 * t3;
  hom[4 * 2 + 1] = -s23 * t1 + t5;

  /* Calculate term to be used in definition of... */
  /* third column of rotation matrix.              */
  t1 = c23 * c4 * s5 + s23 * c5;

  /* Define third column of rotation matrix */
  hom[4 * 0 + 2] = -c1 * t1 - s1 * s4 * s5;
  hom[4 * 1 + 2] = -s1 * t1 + c1 * s4 * s5;
  hom[4 * 2 + 2] = s23 * c4 * s5 - c23 * c5;

  /* Calculate term to be used in definition of...  */
  /* position vector.                               */
  t1 = kins.a1 + kins.a2 * c2 + kins.a3 * c23 - kins.d4 * s23;

  /* Define position vector */
  d23            = kins.d2 + kins.d3;
  hom[4 * 0 + 3] = c1 * t1 - d23 * s1;
  hom[4 * 1 + 3] = s1 * t1 + d23 * c1;
  hom[4 * 2 + 3] = -kins.a3 * s23 - kins.a2 * s2 - kins.d4 * c23;

  /* Calculate terms to be used to...   */
  /* determine flags.                   */
  sum_sq = hom[4 * 0 + 3] * hom[4 * 0 + 3]   // hom.tran.x * hom.tran.x
           + hom[4 * 1 + 3] * hom[4 * 1 + 3] // hom[4*1+3] * hom[4*1+3]
           - d23 * d23;
  k = ( sum_sq
        + hom[4 * 2 + 3] * hom[4 * 2 + 3]  // hom[4*2+3] * hom[4*2+3]
        + kins.a1 * kins.a1
        - 2 * kins.a1 * ( c1 * hom[4 * 0 + 3] + s1 * hom[4 * 1 + 3] )
        - kins.a2 * kins.a2
        - kins.a3 * kins.a3
        - kins.d4 * kins.d4 ) / ( 2.0 * kins.a2 );

  /* reset flags */
  kins.iflags = 0;

  /* Set shoulder-up flag if necessary */
  if ( fabs(joints[0] - atan2(hom[4 * 1 + 3], hom[4 * 0 + 3])
         + atan2(d23, -sqrt(sum_sq) ) ) < FLAG_FUZZ )
  {
    kins.iflags |= THREE21_SHOULDER_RIGHT;
  }

  /* Set elbow down flag if necessary */
  if ( fabs(joints[2] - atan2(kins.a3, kins.d4)
         + atan2(k, -sqrt(kins.a3 * kins.a3
             + kins.d4 * kins.d4 - k * k) ) ) < FLAG_FUZZ )
  {
    kins.iflags |= THREE21_ELBOW_DOWN;
  }

  /* set singular flag if necessary */
  t1 = -hom[4 * 2 + 0] * s1 + hom[4 * 2 + 1] * c1;
  t2 = -hom[4 * 2 + 0] * c1 * c23 - hom[4 * 2 + 1] * s1 * c23 + hom[4 * 2 + 2] * s23;

  if ( ( fabs(t1) < SINGULAR_FUZZ ) && ( fabs(t2) < SINGULAR_FUZZ ) )
  {
    kins.iflags |= THREE21_SINGULAR;
  }
  else
  {
    /* if not singular set wrist flip flag if necessary */
    if ( !( fabs(joints[3] - atan2(t1, t2) ) < FLAG_FUZZ ) )
    {
      kins.iflags |= THREE21_WRIST_FLIP;
    }
  }

  return _poseFromMatrix(hom);
}
////////////////////////////////////////////////////////////////////////////////
int fanuc_lrmate200id::fanuc_lrmate200id_fwd_kin (const double *motors, urdf::Pose &pose)
{
  double joints[6];


  /* gearing equations */
  joints[0] = motors[0];
  joints[1] = motors[1] - M_PI_2;
  joints[2] = -( motors[1] + motors[2] );
  joints[3] = -motors[3];
  joints[4] = -motors[4];
  joints[5] = -motors[5];

  pose = three21_kin_fwd(kins.tk, joints);

  // if (GO_RESULT_OK != retval) return retval;

  /* add the end frame */
  pose = _poseMult(pose, kins.t7);
  return 0;
}
////////////////////////////////////////////////////////////////////////////////
int three21_kin_inv (three21_kin *kins,
  const urdf::Pose &              world,
  std::vector<double> &           joints)
{
  RosMatrix hom;

  double t1, t2, t3;
  double k, sum_sq, d23;

  double th1;
  double th3;
  double th23;
  double th2;
  double th4;
  double th5;
  double th6;

  double s1, c1;
  double s3, c3;
  double s23, c23;
  double s4, c4;
  double s5, c5;
  double s6, c6;

  /* convert pose to hom */
  hom = _matrixFromPose(world);

  /* Joint 1 (2 independent solutions) */

  /* save sum of squares for this and subsequent calcs */
  d23    = kins->d2 + kins->d3;
  sum_sq = hom[4 * 0 + 3] * hom[4 * 0 + 3]
           + hom[4 * 1 + 3] * hom[4 * 1 + 3]
           - d23 * d23;

  /* FIXME-- is use of + sqrt shoulder right or left? */
  if ( kins->iflags & THREE21_SHOULDER_RIGHT )
  {
    th1 = atan2(hom[4 * 1 + 3], hom[4 * 0 + 3]) - atan2(d23, -sqrt(sum_sq) );
  }
  else
  {
    th1 = atan2(hom[4 * 1 + 3], hom[4 * 0 + 3]) - atan2(d23, sqrt(sum_sq) );
  }

  /* save sin, cos for later calcs */
  sincos(th1, &s1, &c1);

  /* Joint 3 (2 independent solutions) */

  k = ( sum_sq
        + hom[4 * 2 + 3] * hom[4 * 2 + 3]
        + kins->a1 * kins->a1
        - 2 * kins->a1 * ( c1 * hom[4 * 0 + 3] + s1 * hom[4 * 1 + 3] )
        - kins->a2 * kins->a2
        - kins->a3 * kins->a3
        - kins->d4 * kins->d4 ) / ( 2.0 * kins->a2 );

  /* FIXME-- is use of + sqrt elbow up or down? */
  if ( kins->iflags & THREE21_ELBOW_DOWN )
  {
    th3 = atan2(kins->a3, kins->d4) - atan2(k, -sqrt(kins->a3 * kins->a3 + kins->d4 * kins->d4 - k * k) );
  }
  else
  {
    th3 = atan2(kins->a3, kins->d4) - atan2(k, sqrt(kins->a3 * kins->a3 + kins->d4 * kins->d4 - k * k) );
  }

  /* compute sin, cos for later calcs */
  sincos(th3, &s3, &c3);

  /* Joint 2 */

  t1 = ( -kins->a3 - kins->a2 * c3 ) * hom[4 * 2 + 3] + ( c1 * hom[4 * 0 + 3] + s1 * hom[4 * 1 + 3] - kins->a1 ) * ( kins->a2 * s3 - kins->d4 );
  t2 = ( kins->a2 * s3 - kins->d4 ) * hom[4 * 2 + 3] + ( kins->a3 + kins->a2 * c3 ) * ( c1 * hom[4 * 0 + 3] + s1 * hom[4 * 1 + 3] - kins->a1 );
  t3 = hom[4 * 2 + 3] * hom[4 * 2 + 3] + ( c1 * hom[4 * 0 + 3] + s1 * hom[4 * 1 + 3] - kins->a1 ) * ( c1 * hom[4 * 0 + 3] + s1 * hom[4 * 1 + 3] - kins->a1 );

  th23 = atan2(t1, t2);
  th2  = th23 - th3;

  /* compute sin, cos for later calcs */
  s23 = t1 / t3;
  c23 = t2 / t3;

  /* Joint 4 */

  t1 = -hom[4 * 2 + 0] * s1 + hom[4 * 2 + 1] * c1;
  t2 = -hom[4 * 2 + 0] * c1 * c23 - hom[4 * 2 + 1] * s1 * c23 + hom[4 * 2 + 2] * s23;

  if ( ( fabs(t1) < SINGULAR_FUZZ ) && ( fabs(t2) < SINGULAR_FUZZ ) )
  {
    throw std::runtime_error("RESULT_SINGULAR");
  }

  th4 = atan2(t1, t2);

  /* compute sin, cos for later calcs */
  s4 = sin(th4);
  c4 = cos(th4);

  /* Joint 5 */

  s5 = hom[4 * 2 + 2] * ( s23 * c4 )
       - hom[4 * 2 + 0] * ( c1 * c23 * c4 + s1 * s4 )
       - hom[4 * 2 + 1] * ( s1 * c23 * c4 - c1 * s4 );
  c5 = -hom[4 * 2 + 0] * ( c1 * s23 ) - hom[4 * 2 + 1]
       * ( s1 * s23 ) - hom[4 * 2 + 2] * c23;
  th5 = atan2(s5, c5);

  /* Joint 6 */

  s6 = hom[4 * 0 + 2] * ( s23 * s4 ) - hom[4 * 0 + 0]
       * ( c1 * c23 * s4 - s1 * c4 ) - hom[4 * 0 + 1] * ( s1 * c23 * s4 + c1 * c4 );
  c6 = hom[4 * 0 + 0] * ( ( c1 * c23 * c4 + s1 * s4 )
                          * c5 - c1 * s23 * s5 ) + hom[4 * 0 + 1]
       * ( ( s1 * c23 * c4 - c1 * s4 ) * c5 - s1 * s23 * s5 )
       - hom[4 * 0 + 2] * ( s23 * c4 * c5 + c23 * s5 );
  th6 = atan2(s6, c6);

  /*
  Is wrist flip the normal or offset result? Absent agreement on
  this, we'll just define it ourselves.
  */
  if ( kins->iflags & THREE21_WRIST_FLIP )
  {
    th4 = th4 + M_PI;
    th5 = -th5;
    th6 = th6 + M_PI;
  }

  /* copy out */
  joints    = std::vector<double>(6, 0.0);
  joints[0] = th1;
  joints[1] = th2;
  joints[2] = th3;
  joints[3] = th4;
  joints[4] = th5;
  joints[5] = th6;
  return 0;
}
////////////////////////////////////////////////////////////////////////////////
int fanuc_lrmate200id::inv_kin (const urdf::Pose  pos, std::vector<double>  &motors)
{
  urdf::Pose end_pos;

  std::vector<double> joints;
  int                 t;

  /* take off the end frame */
  end_pos = _poseMult(pos, kins.t7_inv);

  kins.tk.iflags = kins.tk.iflags & THREE21_ELBOW_DOWN;

  try
  {
    long retval = three21_kin_inv(&kins.tk, end_pos, joints);
    if (0 != retval)
    {
        return retval;
    }
  }
  catch ( std::exception e )
  {
    std::cerr << "fanuc_lrmate200id::fanuc_lrmate200id_kin_inv exception" << e.what( );
    throw e;
  }
  catch ( ... )
  {
    std::cerr << "fanuc_lrmate200id::fanuc_lrmate200id_kin_inv exception\n";
    throw std::runtime_error("fanuc_lrmate200id::fanuc_lrmate200id_kin_inv exception\n");
  }

  //

  /* gearing equations */
  motors    = std::vector<double>(joints.size( ), 0.0);
  motors[0] = joints[0];
  motors[1] = joints[1] + M_PI_2;

  // motors[2] = joints[2] - motors[1];
  motors[2] = -( motors[1] + joints[2] );
  motors[3] = -joints[3];
  motors[4] = -joints[4];
  motors[5] = -joints[5];

  /* normalize motor angles to range [-180..180] as Fanuc does */
  for ( t = 0; t < 6; t++ )
  {
    if ( motors[t] <= -M_PI )
    {
      motors[t] += M_PI_2;
    }
    else if ( motors[t] >= M_PI )
    {
      motors[t] -= M_PI_2;
    }
  }

  return 0;
}
