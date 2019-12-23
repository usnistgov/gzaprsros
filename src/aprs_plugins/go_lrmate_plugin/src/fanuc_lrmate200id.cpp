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

#include "go_lrmate_plugin/fanuc_lrmate200id.h"
#include <go_lrmate_plugin/rosmath.h>

#ifndef HAVE_SINCOS
#define HAVE_SINCOS
void sincos (double x, double *sx, double *cx)
{
  *sx = sin(x);
  *cx = cos(x);
}
#endif
using namespace RCS;


#define REAL_EPSILON     ( 1.0e-7 )
#define TRAN_CLOSE(x, y)    ( fabs( ( x ) - ( y ) ) < REAL_EPSILON )
#define SINGULAR_FUZZ    ( 1.0e-6 )
#define FLAG_FUZZ        ( 1.0e-6 )
#if 1
typedef boost::array<double, 12>   RosMatrix;
static tf::Quaternion _tfquatFromMatrix (RosMatrix & mat)
{
	tf::Quaternion rot;
	double         tr = mat[4 * 0 + 0] + mat[4 * 1 + 1] + mat[4 * 2 + 2];

	if ( tr >= 0 )
	{
	rot = tf::Quaternion(
		( mat[4 * 2 + 1] - mat[4 * 1 + 2] ),
		 ( mat[4 * 0 + 2] - mat[4 * 2 + 0] ),
		 ( mat[4 * 1 + 0] - mat[4 * 0 + 1] ),
		tr + 1);
	}
    else
	{
		// find the largest diagonal element and jump to the appropriate case
		if ( mat[4 * 1 + 1] > mat[4 * 0 + 0] )
		{
			if ( mat[4 * 2 + 2] > mat[4 * 1 + 1] )
			{
			rot = tf::Quaternion(
				 ( mat[4 * 2 + 0] + mat[4 * 0 + 2] ),
				 ( mat[4 * 1 + 2] + mat[4 * 2 + 1] ),
				 ( mat[4 * 2 + 2] - ( mat[4 * 0 + 0] + mat[4 * 1 + 1] ) ) + 1,
				 ( mat[4 * 1 + 0] - mat[4 * 0 + 1] ));
            }
			else
			{
			rot = tf::Quaternion(
				( mat[4 * 0 + 1] + mat[4 * 1 + 0] ),
				 ( mat[4 * 1 + 1] - ( mat[4 * 2 + 2] + mat[4 * 0 + 0] ) ) + 1,
                 ( mat[4 * 1 + 2] + mat[4 * 2 + 1] ),
				 ( mat[4 * 0 + 2] - mat[4 * 2 + 0] ));
			}
		}
		else if ( mat[4 * 2 + 2] > mat[4 * 0 + 0] )
		{
			rot = tf::Quaternion(
			 ( mat[4 * 2 + 0] + mat[4 * 0 + 2] ),
			 ( mat[4 * 1 + 2] + mat[4 * 2 + 1] ),
			 ( mat[4 * 2 + 2] - ( mat[4 * 0 + 0] + mat[4 * 1 + 1] ) ) + 1,
			 ( mat[4 * 1 + 0] - mat[4 * 0 + 1] ));
		}
		else
		{
			rot = tf::Quaternion(
			 ( mat[4 * 0 + 0] - ( mat[4 * 1 + 1] + mat[4 * 2 + 2] ) ) + 1,
			 ( mat[4 * 0 + 1] + mat[4 * 1 + 0] ),
			 ( mat[4 * 2 + 0] + mat[4 * 0 + 2] ),
			 ( mat[4 * 2 + 1] - mat[4 * 1 + 2] ));
		}
	}
	double fnorm = std::sqrt(rot.x() * rot.x() + rot.y() * rot.y() + rot.z() * rot.z() + rot.w() * rot.w());

	// don't touch the divides
	rot/=fnorm;
#if 0
	rot.x() /= fnorm;
	rot.y() /= fnorm;
	rot.z() /= fnorm;
	rot.w() /= fnorm;
#endif
	return rot;
}
static tf::Pose _tfposeFromMatrix (const boost::array<double, 12> & m)
{
    tf::Quaternion q  = _tfquatFromMatrix( ( RosMatrix & )m);
	tf::Vector3 t(m[3], m[7], m[11]);
	return tf::Pose(q,t);
}

boost::array<double, 12>  _matrixFromtfPose(tf::Pose pose)
{
	boost::array<double, 12>  m;
	tf::Vector3 t = pose.getOrigin();
	m[3]=t.x();
	m[7]=t.y();
	m[11]=t.z();
	tf::Matrix3x3 b = pose.getBasis () ;
	m[0] = b[0][0];
	m[1] = b[0][1];
	m[2] = b[0][2];

	m[4] = b[1][0];
	m[5] = b[1][1];
	m[6] = b[1][2];

	m[8] = b[2][0];
	m[9] = b[2][1];
	m[10] = b[2][2];
	return m;
	
}
#endif
static bool readFile (std::string filename, std::string & contents)
{
    std::ifstream     in(filename.c_str( ), std::ifstream::in );
    std::stringstream buffer;

    if(!in.is_open())
    {
        return false;
    }
    buffer << in.rdbuf( );
    contents = buffer.str( );
    return true;
}

static void normalizeAngle(double& val, const double& min, const double& max)
{
    if (val > max) {
      //Find actual angle offset
      double diffangle = fmod(val-max,2*M_PI);
      // Add that to upper bound and go back a full rotation
      val = max + diffangle - 2*M_PI;
    }

    if (val < min) {
      //Find actual angle offset
      double diffangle = fmod(min-val,2*M_PI);
      // Add that to upper bound and go back a full rotation
      val = min - diffangle + 2*M_PI;
    }
}

////////////////////////////////////////////////////////////////////////////////
fanuc_lrmate200id::fanuc_lrmate200id(void) : CSerialLinkRobot(this)
{
  kins.tk.a1     = 0.050;
  kins.tk.a2     = 0.330;
  kins.tk.a3     = 0.035;
  kins.tk.d2     = 0;
  kins.tk.d3     = 0;
  kins.tk.d4     = 0.335;
  kins.tk.iflags = 0;

  kins.t7 = tf::Pose(tf::Quaternion(0,0,0,1), tf::Vector3(0,0, WRIST_OFFSET)); 
  kins.t7_inv = kins.t7.inverse(); // _poseInv(kins.t7);
}
////////////////////////////////////////////////////////////////////////////////
fanuc_lrmate200id::~fanuc_lrmate200id(void)
{
}
////////////////////////////////////////////////////////////////////////////////
size_t fanuc_lrmate200id::numJoints()
{
    return this->_nJoints;
}
////////////////////////////////////////////////////////////////////////////////
int fanuc_lrmate200id::init()
{
    int hr=Kinematics_Ok;
    errmsg.clear();

    if(_urdf.empty() && _urdffile.empty())
    {
        errmsg= "fanuc_lrmate200id missing urdf or urdffile parameters\n";
        hr= Bad_Parameter;

    }
    else if(_urdffile.empty() )
    {
        errmsg=std::string("Urdffile parameter not found ")+_urdffile;
        hr= Bad_Parameter;
    }
    else if( !readFile(_urdffile,_urdf))
    {
        errmsg=std::string("Urdf file not found ")+_urdffile;
        hr= Bad_Parameter;
    }

    else if(! parseURDF(_urdf, _baselink, _tiplink))
    {
        errmsg="fanuc_lrmate200id bad URDF string\n";
        hr= -1;
    }
    this->_nJoints=jointNames.size();
    if(!errmsg.empty())
        std::cout << errmsg;
    return hr;
}

////////////////////////////////////////////////////////////////////////////////
int fanuc_lrmate200id::FK(std::vector<double> joints, tf::Pose &pose)
{

    int hr=Kinematics_Ok;
    errmsg.clear();

    try
    {
//        joints= add_fkgearing( joints);
        pose = fanuc_lrmate200id_kin_fwd(&joints[0]);

    }
    catch(...)
    {
        hr=-1;
        errmsg="Failed fanuc_lrmate200id FK computation";
    }
    return hr;
}


////////////////////////////////////////////////////////////////////////////////
int fanuc_lrmate200id::IK(tf::Pose pose, std::vector<double>&  joints)
{
    int hr=Kinematics_Ok;
    errmsg.clear();

    try
    {
        // compute inverse kinematics from robot centric pose
         joints= fanuc_lrmate200id_kin_inv(pose) ;

         // Normalize joint values
         for(size_t i=0; i< this->numJoints(); i++)
            normalizeAngle(joints[i], this->jointMin[i], this->jointMax[i]);

    }
    catch(...)
    {
        hr=-1;
        errmsg="Failed fanuc_lrmate200id IK computation";
    }
    return hr;

}
////////////////////////////////////////////////////////////////////////////////
tf::Pose three21_kin_fwd (three21_kin & kins,
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
  t2 = s23 * s5 * c6;
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
  return _tfposeFromMatrix(hom);
}

////////////////////////////////////////////////////////////////////////////////
tf::Pose fanuc_lrmate200id::fanuc_lrmate200id_kin_fwd(const double *motors) {
    std::vector<double> joints(motors, motors + 6);
    joints=add_fkgearing(joints);
    tf::Pose pose = three21_kin_fwd(kins.tk, &joints[0]);

    /* add the end frame  m+ multiply t7*/
    pose= pose * kins.t7;

    // if (GO_RESULT_OK != retval) return retval;
    // Add .33 to base z
    pose.getOrigin().setZ( pose.getOrigin().getZ() +  .33);
    return pose;
}

////////////////////////////////////////////////////////////////////////////////
int three21_kin_inv (three21_kin *kins,
  tf::Pose &                world,
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


  hom = _matrixFromtfPose(world);

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
std::vector<double> fanuc_lrmate200id::fanuc_lrmate200id_kin_inv (tf::Pose & end_pos)
{
  // assume move 1st z up by 330
    
  std::vector<double> joints;
 
  std::vector<double> motors;
  int                 t;

  // assume this has already been done
  /* take off the end frame */
  //end_pos = _poseMult(pos, kins.t7_inv);


  kins.tk.iflags = kins.tk.iflags & THREE21_ELBOW_DOWN;

  try
  {
      /* convert pose to hom */
      /* "subtract" the end frame t7*/
      end_pos= end_pos * this->kins.t7_inv;
      /* subtract the base offset*/
      end_pos.getOrigin().setZ( end_pos.getOrigin().getZ() -  .33);

    long retval = three21_kin_inv(&kins.tk, end_pos, joints);
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

  // if (GO_RESULT_OK != retval) return -1;

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
    if ( motors[t] < -M_PI )
    {
      motors[t] += M_PI_2;
    }
    else if ( motors[t] > M_PI )
    {
      motors[t] -= M_PI_2;
    }
  }

  return motors;
}


////////////////////////////////////////////////////////////////////////////////
int fanuc_lrmate200id::debugStream(std::ostream& o)
{
    out.copyfmt(o); //1
    out.clear(o.rdstate()); //2
    out.basic_ios<char>::rdbuf(o.rdbuf());
    return Kinematics_Ok;
}

////////////////////////////////////////////////////////////////////////////////
int fanuc_lrmate200id::debug(bool flag)
{
    bDebug=flag;
    return Kinematics_Ok;
}


////////////////////////////////////////////////////////////////////////////////
std::string fanuc_lrmate200id::get(std::string param)
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
        ss << "GoKin kinematics solver using gomotion genserkins\n";
        ss << "Parameters:\n";
        ss << "\tihelp\n";
        ss << "\terror\n";
        ss << "\tini\n";
        ss << "\tinifile\n";
        ss << "\turdf\n";
        ss << "\turdf\n";
        ss << "\tbaselink\n";
        ss << "\ttiplink\n";
        return ss.str();
    }
    else if(param == "INI")
    {
        return ini;
    }
    else if(param == "INIFILE")
    {
        return _inifilename;
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

    return std::string("No get for parameter ") + param;
}


////////////////////////////////////////////////////////////////////////////////
std::string fanuc_lrmate200id::set(std::string param,  std::string value)
{
    errmsg.clear();
    const char* ws = " \t\n\r";

    param.erase(param.find_last_not_of(ws) + 1);
    param.erase(0, param.find_first_not_of(ws));
    std::transform(param.begin(), param.end(),param.begin(), ::toupper);
    if(param == "DEBUG")
    {
        bDebug = std::stoi(value);
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
    else
    {
        errmsg=std::string("No match for ") + param;
    }

    return errmsg;


}
////////////////////////////////////////////////////////////////////////////////
std::string fanuc_lrmate200id::set(std::string param,  void * value)
{
    return std::string("No match for ") + param;
}
