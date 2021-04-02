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
\file gomath.h

\brief Declarations for pose math functions
*/

/*!
\defgroup GOMATH The Go Motion Pose Mathematics Library

\b Go Math Representations

Positions are vectors that indicate where something is. In the
three-dimensional world, three numbers are necessary to indicate
position. Go Motion supports position vectors in several
representations: Cartesian, cylindrical and spherical. 

The Cartesian representation uses three numbers \b x, \b y and \b z to
represent distances from the origin along three perpendicular
axes. The cylindrical representation uses three numbers \b r, \b theta
and \b z to represent radial distance away from the origin, angle
around the origin and distance up and down from the origin
respectively. The spherical representation uses three numbers
\b theta, \b phi and \b r to represent angle down from the zenith,
angle around the origin and radius from the origin respectively. Go
Motion has functions that convert position in one representation
position in another representation, so the choice of which
representation to use can be made for convenience. Cartesian
representations will be assumed unless otherwise specified.

Orientations are vectors that indicate how something is rotated. In
the three-dimensional world, three numbers are necessary to indicate
orientation. Go Motion supports orientation vectors in several
representations: roll, pitch and yaw; Euler angles; quaternions;
rotation vectors and rotation matrices. Some of these
representations use more than three numbers, exploiting redundancy
to make calculations with these representations more efficient. For
example, a quaternion uses four numbers, and a rotation matrix uses
nine numbers. 

Vectors are usually written as a column of numbers enclosed in
vertical bars, like this:

| 1 | <br>
| 2 | - a vector depicted in its column form <br>
| 3 |

This can be unwieldy in text documentation, so vectors may also be
written as a row of numbers enclosed in parentheses, like this:

( 1 2 3 ) - a vector depicted in its row form

The interpretation of a vector depends on the quantity it
represents. The vector shown above could mean a translation of 1, 2
and 3 units in the x, y and z directions if the vector were a
Cartesian position, or a rotation of 1, 2 and 3 units around the x,
y and z directions if the vector were an orientation in roll, pitch
and yaw. 

Both position and orientation are needed to fully describe where
something is and how it is rotated. The combination of position and
orientation is called a 'pose'. Poses can be shown in row
form like this pose representing a Cartesian position of ( 1 2 3 )
and an orientation in roll, pitch and yaw of ( 30 -30 90 ):

( 1 2 3 ; 30 -30 90)

A semicolon is used to separate the position from the orientation.

<b>Go Math Reference Frames</b>

Regardless of the representation chosen, the numbers that indicate
position and orientation of an object depend on the established
origin. Several origins may be established for convenience, for
example one fixed on the world and one that moves with a tool. These
origins may differ from each other in both position and
orientation. The establishment of the position and orientation of an
origin is a 'reference frame'. The term 'coordinate frame' is used
interchangeably with 'reference frame'.

When several reference frames are being used, they are denoted as
identifiers in braces, for example,

{A} - a reference frame called 'A'.

{world} - the world reference frame.

{tool} - the world reference frame.

To convert the representation of a pose in one reference frame to
its representation in another, one needs to know the position and
orientation of one origin with respect to the other. This difference
between the two origins is called a 'transform'.

Poses and transforms are similar things; both include position
and orientation. Whether something is a pose or a transform depends
on how one is using it. Poses are used to indicate the position and
orientation of things with respect to an established reference
frame. Transforms are used to indicate the position and orientation
of reference frames with respect to other reference frames. If a
'thing' happens to be a reference frame, its pose is its transform.

\b Go Math Nomenclature

The letter \b P is used to denote positions and the letter \b R is
used to denote orientations. As usual, trailing subscripts denote
the identity of quantities, for example,

P<small><sub>hand</sub></small> - the position of the hand.

R<small><sub>head</sub></small> - the orientation of the head.

Leading superscripts denote the reference frame in which the
quantity is expressed, for example,

<small><sup>A</sup></small>P<small><sub>hand</sub></small> - the
position of the hand with respect to the {A} reference frame.

<small><sup>B</sup></small>R<small><sub>head</sub></small> - the
orientation of the head with respect to the {B} reference frame.

The figure below shows a rotation of reference frame {B} 
with respect to reference frame {A} by an angle \b u. Note that the
rotation can be viewed as a rotation of {B} with respect to {A},
denoted
<small><sup>A</sup></small>u<small><sub>B</sub></small>,
or as a rotation of {A} with respect to {B}, denoted 
<small><sup>B</sup></small>u<small><sub>A</sub></small>. The heads
of arrows are attached to the 'of the' frames, while the tails of
arrows are based on the 'with respect to' frames. Angles are
taken as positive according the the right hand rule, so in this
figure <small><sup>A</sup></small>u<small><sub>B</sub></small> is a
positive number of about 45 degrees, while
<small><sup>B</sup></small>u<small><sub>A</sub></small> is a
negative number of the same magnitude. 

\image html fig002.png
<center> \e Figure 1. </center>

Transforms from one reference frame to another are denoted with
leading subscripts and superscripts. The leading subscript denotes
the original frame, and the leading superscript denotes the new
frame. If a transform is purely rotation, it is denoted with an
\b R, for example,

<small><sup>B</sup><sub>A</sub></small>R - a rotation from the {A}
frame to the {B} frame.

If a transform includes both a rotation and a translation, it is
denoted with a \b T, for example,

<small><sup>world</sup><sub>tool</sub></small>T - a transform from the {tool}
frame to the {world} frame.

To convert a quantity from one frame to another, pre-multiply the
quantity by the transform. Algebraically, the leading subscript of
the transform must match the leading superscript of the quantity,
for example,

<small><sup>B</sup></small>P = <small><sup>B</sup><sub>A</sub></small>R * <small><sup>A</sup></small>P

This is read, 'the position with respect to the B frame is the
rotation from the A frame to the B frame times the position with
respect to the A frame.' Another example is,

<small><sup>world</sup></small>P = <small><sup>world</sup><sub>tool</sub></small>T * <small><sup>tool</sup></small>P

This is read, 'the position with respect to the world frame is the
transform from the tool frame to the world frame times the position with
respect to the tool frame.'

One can generate a rotational transform by defining an orientation
vector with the appropriate angles. If using a roll, pitch and yaw
representation, the rotational transforms in the figure above (each about
the \b z axis) is a yaw:

| &nbsp; 0 &nbsp; | <br>
| &nbsp; 0 &nbsp; | = <small><sup>A</sup></small>R<small><sub>B</sub></small> <br>
| <small><sup>A</sup></small>u<small><sub>B</sub></small> |

| &nbsp; 0 &nbsp; | <br>
| &nbsp; 0 &nbsp; | = <small><sup>B</sup></small>R<small><sub>A</sub></small> <br>
| <small><sup>B</sup></small>u<small><sub>A</sub></small> |

and negative angles work as well, for

| &nbsp;&nbsp; 0 &nbsp; | <br>
| &nbsp;&nbsp; 0 &nbsp; | = <small><sup>A</sup></small>R<small><sub>B</sub></small> <br>
| -<small><sup>B</sup></small>u<small><sub>A</sub></small> |

\b Go Math Examples

Given a reference frame {B} rotated 30 degrees with respect to the \b
z axis of reference frame {A}, as in Figure 1 above, transform
from points in the {B} frame to points in the {A} frame like this:
\code
go_rpy rot;
go_cart pt_in_b;
go_cart pt_in_a;

// The angle of {B} with respect to {A} is 30 degrees.
// Go uses angles in radians.
rot.r = 0, rot.p = 0, rot.y = GO_TO_RAD(30);

pt_in_b.x = 1, pt_in_b.y = 2, pt_in_b.z = 3;

// Multiply a transform and a point to get a new point.
go_rpy_cart_mult(&rot, &pt_in_b, &pt_in_a);

go_cart_print(&pt_in_a);
\endcode
will print the postion of the point in the {A} frame: 
\code
-0.133975 2.232051 3.000000
\endcode
Here is a more complex example of a full transform, including both
translation and rotation.
\image html fig003.png
<center>\e Figure 2.</center>

The frame {B} is translated and rotated with respect to {A}.
<sup><small>A</small></sup>x<sub><small>B</small></sub> is the
amount of translation of {B} in the \b x direction of {A}.
Likewise, <sup><small>A</small></sup>y<sub><small>B</small></sub> is the
amount of translation of {B} in the \b y direction of {A}.
<sup><small>A</small></sup>u<sub><small>B</small></sub> is the
rotation of the {B} frame about the \b z axis of the {A} frame.

To convert points in the {B} frame to points in the {A} frame, do
this: 
\code
go_pose pose;
go_rpy rot;
go_cart pt_in_b;
go_cart pt_in_a;

// The translation of {B} with respect to {A} is about (2 1 0).
pose.tran.x = 2, pose.tran.y = 1, pose.tran.z = 0;

// The angle of {B} wrt {A} is about 30 degrees, made into radians.
rot.r = 0, rot.p = 0, rot.y = GO_TO_RAD(30);

// A 'go_pose' uses quaternions for rotations, so we have to
// convert a roll-pitch-yaw to a quaternion.
go_rpy_quat_convert(&rot, &pose.rot);

pt_in_b.x = 1, pt_in_b.y = 2, pt_in_b.z = 3;

// Multiply a transform and a point to get a new point.
go_pose_cart_mult(&pose, &pt_in_b, &pt_in_a);

go_cart_print(&pt_in_a);

\endcode
This gives the transformed point in the {A} frame as
\code
1.866025 3.232051 3.000000
\endcode
*/

#ifndef GO_MATH_H
#define GO_MATH_H

#include <stddef.h>		/* sizeof */
#include <math.h>		/* M_PI */
#include <float.h>		/* FLT,DBL_MIN,MAX,EPSILON */
#include "gotraj/gotypes.h"		/* go_integer,real */

namespace gomotion 
{
	/*! Returns the square of \a x. */
#define go_sq(x) ((x)*(x))
	/*! Returns the cube of \a x.  */
#define go_cub(x) ((x)*(x)*(x))
	/*! Returns \a x to the fourth power. */
#define go_qua(x) ((x)*(x)*(x)*(x))

	/*! Returns the sine and cosine of \a x (in radians) in \a s and \a c,
	respectively. Implemented as a single call when supported, to speed
	up the calculation of the two values. */
	void go_sincos(go_real x, go_real *s, go_real *c);

	/*! Returns the cube root of \a x. */
	go_real go_cbrt(go_real x);

#ifdef M_PI
	/*! The value of Pi. */
#define GO_PI M_PI
#else
	/*! The value of Pi. */
#define GO_PI 3.14159265358979323846
#endif
	/*! The value of twice Pi. */
#define GO_2_PI (2.0*GO_PI)

#ifdef M_PI_2
	/*! The value of half of Pi. */
#define GO_PI_2 M_PI_2
#else
	/*! The value of half of Pi. */
#define GO_PI_2 1.57079632679489661923
#endif

#ifdef M_PI_4
	/*! The value of one-fourth of Pi. */
#define GO_PI_4 M_PI_4
#else
	/*! The value of one-fourth of Pi. */
#define GO_PI_4 0.78539816339744830962
#endif

	/*! Returns \a rad in radians as its value in degrees. */
#define GO_TO_DEG(rad) ((rad)*57.295779513082323)
	/*! Returns \a deg in degrees as its value in radians. */
#define GO_TO_RAD(deg) ((deg)*0.0174532925199432952)

	/*! How close translational quantities must be to be equal. */
#define GO_TRAN_CLOSE(x,y) (fabs((x)-(y)) < GO_REAL_EPSILON)
	/*! How small a translational quantity must be to be zero. */
#define GO_TRAN_SMALL(x) (fabs(x) < GO_REAL_EPSILON)

	/*! How close rotational quantities must be to be equal. */
#define GO_ROT_CLOSE(x,y) (fabs((x)-(y)) < GO_REAL_EPSILON)
	/*! How small a rotational quantity must be to be zero. */
#define GO_ROT_SMALL(x) (fabs(x) < GO_REAL_EPSILON)

	/*! How close general quantities must be to be equal. Use this when
	you have something other than translational or rotational quantities,
	otherwise use one of \a GO_TRAN,ROT_CLOSE. */
#define GO_CLOSE(x,y) (fabs((x)-(y)) < GO_REAL_EPSILON)
	/*! How small a general quantity must be to be zero. Use this when
	you have something other than a translational or rotational quantity,
	otherwise use one of \a GO_TRAN,ROT_SMALL. */
#define GO_SMALL(x) (fabs(x) < GO_REAL_EPSILON)

	/*! Double-valued inverse sine, giving both solutions */
	go_result go_asines(go_real s, go_real *asp, go_real *asn);

	/*! Double-valued inverse cosine, giving both solutions */
	go_result go_acoses(go_real c, go_real *acp, go_real *acn);

	/*! Double-valued inverse tangent, giving both solutions */
	go_result go_atans(go_real t, go_real *atp, go_real *atn);

	/*! A point or vector in Cartesian coordinates. */
	typedef struct {
		go_real x;
		go_real y;
		go_real z;
	} go_cart;

	/*! A point or vector in spherical coordinates, with \a phi as 
	the angle down from the zenith, not up from the XY plane. */
	typedef struct {
		go_real theta;
		go_real phi;
		go_real r;
	} go_sph;

	/*! A point or vector in cylindrical coordinates. */
	typedef struct {
		go_real theta;
		go_real r;
		go_real z;
	} go_cyl;

	/*! A rotation vector, whose direction points along the axis of positive
	rotation, and whose magnitude is the amount of rotation around this
	axis, in radians. */
	typedef struct {
		go_real x;
		go_real y;
		go_real z;
	} go_rvec;

	/*            |  m.x.x   m.y.x   m.z.x  | */
	/* go_mat m = |  m.x.y   m.y.y   m.z.y  | */
	/*            |  m.x.z   m.y.z   m.z.z  | */

	/*! A rotation matrix. */
	typedef struct {
		go_cart x;			/*!< X unit vector */
		go_cart y;			/*!< Y unit vector */
		go_cart z;			/*!< Z unit vector */
	} go_mat;

	/*! A quaternion. \a s is the cosine of the half angle of rotation,
	and the \a xyz elements comprise the vector that points in the direction
	of positive rotation and whose magnitude is the sine of the half
	angle of rotation. */
	typedef struct {
		go_real s;
		go_real x;
		go_real y;
		go_real z;
	} go_quat;

	/*! ZYZ Euler angles. \a z is the amount of the first rotation around the
	Z axis. \a y is the amount of the second rotation around the new \a Y
	axis. \a zp is the amount of the third rotation around the new \a Z axis. */
	typedef struct {
		go_real z;
		go_real y;
		go_real zp;
	} go_zyz;

	/*! ZYX Euler angles. \a z is the amount of the first rotation around the
	Z axis. \a y is the amount of the second rotation around the new \a Y
	axis. \a x is the amount of the third rotation around the new \a X axis. */
	typedef struct {
		go_real z;
		go_real y;
		go_real x;
	} go_zyx;

	/*! XYZ Euler angles. \a x is the amount of the first rotation around the
	X axis. \a y is the amount of the second rotation around the new \a Y
	axis. \a z is the amount of the third rotation around the new \a Z axis. */
	typedef struct {
		go_real x;
		go_real y;
		go_real z;
	} go_xyz;

	/*! Roll-pitch-yaw angles. \a r is the amount of the first rotation
	(roll) around the X axis. \a p is the amount of the second rotation
	(pitch) around the original Y axis.  \a y is the amount of the third
	rotation (yaw) around the original Z axis. */
	typedef struct {
		go_real r;
		go_real p;
		go_real y;
	} go_rpy;

	/*! X-Z vector format. \a x is a unit vector in the X direction, and \a z is a unit vector in the Z direction. */
	typedef struct {
		go_cart x;
		go_cart z;
	} go_uxz;

	/*!
	A \a go_pose represents the Cartesian position vector and quaternion
	orientation of a frame.
	*/
	typedef struct {
		go_cart tran;
		go_quat rot;
	} go_pose;

	/*!
	A \a go_vel represents the linear- and angular velocity vectors
	of a frame. \a v is the Cartesian linear velocity vector.
	\a w is the Cartesian angular velocity vector, the instantaneous
	vector about which the frame is rotating.
	*/
	typedef struct {
		go_cart v;
		go_cart w;
	} go_vel;

	/*! Convenience function that returns a \a go_pose given individual
	elements. */
	go_pose
		go_pose_this(go_real x, go_real y, go_real z,
		go_real rs, go_real rx, go_real ry, go_real rz);

	/*! Returns the zero vector. */
	go_cart
		go_cart_zero(void);

	/*! Returns the identity (zero) quaternion, i.e., no rotation. */
	go_quat
		go_quat_identity(void);

	/*! Returns the identity pose, no translation or rotation. */
	go_pose
		go_pose_identity(void);

	typedef struct {
		go_cart tran;
		go_mat rot;
	} go_hom;

	/* lines, planes and related functions */

	/*!
	Lines are represented in point-direction form 
	(point p, direction v) as
	(x - px)/vx = (y - py)/vy = (z - pz)vz
	*/
	typedef struct {
		go_cart point;
		go_cart direction;		/* always a unit vector */
	} go_line;

	/*!
	Given a plane as Ax + By + Cz + D = 0, the normal vector
	\a normal is the Cartesian vector (A,B,C), and the number
	\a d is the value D.

	Planes have a handedness, given by the direction of the normal
	vector, so two planes that appear coincident may be different by the
	direction of their anti-parallel normal vectors.
	*/
	typedef struct {
		go_cart normal;
		go_real d;
	} go_plane;

	/*! Fills in \a line given \a point and \a direction. Returns GO_RESULT_OK
	if \a direction is non-zero, otherwise GO_RESULT_ERROR. */
	go_result go_line_from_point_direction(const go_cart *point, const go_cart *direction, go_line *line);

	/*! Fills in \a line given two points. Returns GO_RESULT_OK if the points
	are different, otherwise GO_RESULT_ERROR. */
	go_result go_line_from_points(const go_cart *point1, const go_cart *point2, go_line *line);

	/*! Fill in \a line with the intersection of the two planes. Returns GO_RESULT_OK if the planes are not parallel, otherwise GO_RESULT_ERROR. */
	go_result go_line_from_planes(const go_plane *plane1, const go_plane *plane2, go_line *line);

	/*! Returns non-zero if the lines are the same, otherwise zero. */
	go_flag go_line_line_compare(const go_line *line1, const go_line *line2);

	/*! Fills in \a point with the point located distance \a d along \a line */
	go_result go_line_evaluate(const go_line *line, go_real d, go_cart *point);

	/*! Fills in \a distance with the distance from \a point to \a line */
	go_result go_point_line_distance(const go_cart *point, const go_line *line, go_real *distance);

	/*! Fills in \a pout with the nearest point on \a line to \a point */
	go_result go_point_line_proj(const go_cart *point, const go_line *line, go_cart *pout);

	/*! Fills in \a proj with the projection of \a point onto \a plane */
	go_result go_point_plane_proj(const go_cart *point, const go_plane *plane, go_cart *proj);

	/*! Fills in \a proj with the projection of \a line onto \a plane */
	go_result go_line_plane_proj(const go_line *line, const go_plane *plane, go_line *proj);

	/*! Fills in \a plane give a \a point on the plane and the normal \a direction. */
	go_result go_plane_from_point_normal(const go_cart *point, const go_cart *normal, go_plane *plane);

	/*! Fills in \a plane given the A, B, C and D values in the canonical
	form Ax + By + Cz + D = 0. Returns GO_RESULT_OK
	if not all of A, B and C are zero, otherwise GO_RESULT_ERROR. */
	go_result go_plane_from_abcd(go_real A, go_real B, go_real C, go_real D, go_plane *plane);

	/*! Fills in \a plane given three points. Returns GO_RESULT_OK
	if the points are distinct, otherwise GO_RESULT_ERROR. */
	go_result go_plane_from_points(const go_cart *point1, const go_cart *point2, const go_cart *point3, go_plane *plane);

	/*! Fills in \a plane given a \a point on the plane and a \a line
	in the plane. Returns GO_RESULT_OK if the point is not on the line,
	GO_RESULT_ERROR otherwise. */
	go_result go_plane_from_point_line(const go_cart *point, const go_line *line, go_plane *plane);

	/*! Returns non-zero if the planes are coincident and have the same
	normal direction, otherwise zero. */
	go_flag go_plane_plane_compare(const go_plane *plane1, const go_plane *plane2);

	/*! Fills in the \a distance from the \a point to the \a plane.  */
	go_result go_point_plane_distance(const go_cart *point, const go_plane *plane, go_real *distance);

	/*! Fills in \a point with the point located distances \a u and \a v along
	some othogonal planar coordinate system in \a plane */
	go_result go_plane_evaluate(const go_plane *plane, go_real u, go_real v, go_cart *point);

	/*! Fills in \a point with the intersection point of
	\a line with \a plane, and \a distance with the distance along the
	line to the intersection point. Returns GO_RESULT_ERROR if the line
	is parallel to the plane and not lying in the plane, otherwise
	GO_RESULT_OK. */
	go_result go_line_plane_intersect(const go_line *line, const go_plane *plane, go_cart *point, go_real *distance);

	/*
	struct arguments are passed to functions as const pointers since the
	speed is at least as fast for all but structs of one or two elements.
	*/

	/* translation rep conversion functions */

	go_result go_cart_sph_convert(const go_cart *, go_sph *);
	go_result go_cart_cyl_convert(const go_cart *, go_cyl *);
	go_result go_sph_cart_convert(const go_sph *, go_cart *);
	go_result go_sph_cyl_convert(const go_sph *, go_cyl *);
	go_result go_cyl_cart_convert(const go_cyl *, go_cart *);
	go_result go_cyl_sph_convert(const go_cyl *, go_sph *);

	/* rotation rep conversion functions */

	go_result go_rvec_quat_convert(const go_rvec *, go_quat *);
	go_result go_rvec_mat_convert(const go_rvec *, go_mat *);
	go_result go_rvec_zyz_convert(const go_rvec *, go_zyz *);
	go_result go_rvec_zyx_convert(const go_rvec *, go_zyx *);
	go_result go_rvec_xyz_convert(const go_rvec *, go_xyz *);
	go_result go_rvec_rpy_convert(const go_rvec *, go_rpy *);

	go_result go_quat_rvec_convert(const go_quat *, go_rvec *);
	go_result go_quat_mat_convert(const go_quat *, go_mat *);
	go_result go_quat_zyz_convert(const go_quat *, go_zyz *);
	go_result go_quat_zyx_convert(const go_quat *, go_zyx *);
	go_result go_quat_xyz_convert(const go_quat *, go_xyz *);
	go_result go_quat_rpy_convert(const go_quat *, go_rpy *);

	go_result go_mat_rvec_convert(const go_mat *, go_rvec *);
	go_result go_mat_quat_convert(const go_mat *, go_quat *);
	go_result go_mat_zyz_convert(const go_mat *, go_zyz *);
	go_result go_mat_zyx_convert(const go_mat *, go_zyx *);
	go_result go_mat_xyz_convert(const go_mat *, go_xyz *);
	go_result go_mat_rpy_convert(const go_mat *, go_rpy *);

	go_result go_zyz_rvec_convert(const go_zyz *, go_rvec *);
	go_result go_zyz_quat_convert(const go_zyz *, go_quat *);
	go_result go_zyz_mat_convert(const go_zyz *, go_mat *);
	go_result go_zyz_zyx_convert(const go_zyz *, go_zyx *);
	go_result go_zyz_xyz_convert(const go_zyz *, go_xyz *);
	go_result go_zyz_rpy_convert(const go_zyz *, go_rpy *);

	go_result go_zyx_rvec_convert(const go_zyx *, go_rvec *);
	go_result go_zyx_quat_convert(const go_zyx *, go_quat *);
	go_result go_zyx_mat_convert(const go_zyx *, go_mat *);
	go_result go_zyx_zyz_convert(const go_zyx *, go_zyz *);
	go_result go_zyx_xyz_convert(const go_zyx *, go_xyz *);
	go_result go_zyx_rpy_convert(const go_zyx *, go_rpy *);

	go_result go_xyz_rvec_convert(const go_xyz *, go_rvec *);
	go_result go_xyz_quat_convert(const go_xyz *, go_quat *);
	go_result go_xyz_mat_convert(const go_xyz *, go_mat *);
	go_result go_xyz_zyz_convert(const go_xyz *, go_zyz *);
	go_result go_xyz_zyx_convert(const go_xyz *, go_zyx *);
	go_result go_xyz_rpy_convert(const go_xyz *, go_rpy *);

	go_result go_rpy_rvec_convert(const go_rpy *, go_rvec *);
	go_result go_rpy_quat_convert(const go_rpy *, go_quat *);
	go_result go_rpy_mat_convert(const go_rpy *, go_mat *);
	go_result go_rpy_zyz_convert(const go_rpy *, go_zyz *);
	go_result go_rpy_zyx_convert(const go_rpy *, go_zyx *);
	go_result go_rpy_xyz_convert(const go_rpy *, go_xyz *);

	go_result go_uxz_mat_convert(const go_uxz *, go_mat *);
	go_result go_mat_uxz_convert(const go_mat *, go_uxz *);

	/* combined rep conversion functions */

	go_result go_pose_hom_convert(const go_pose *, go_hom *);
	go_result go_hom_pose_convert(const go_hom *, go_pose *);

	/* misc conversion functions */
	/*!
	go_cart_rvec_convert and go_rvec_cart_convert convert between
	Cartesian vectors and rotation vectors. The conversion is trivial
	but keeps types distinct.
	*/
	go_result go_cart_rvec_convert(const go_cart *cart, go_rvec *rvec);
	go_result go_rvec_cart_convert(const go_rvec *rvec, go_cart *cart);

	/* translation functions, that work only with the preferred
	go_cart type. Other types must be converted to go_cart
	to use these, e.g., there's no go_sph_cyl_compare() */

	go_flag go_cart_cart_compare(const go_cart *, const go_cart *);
	go_result go_cart_cart_dot(const go_cart *, const go_cart *,
		go_real *);
	go_result go_cart_cart_cross(const go_cart *, const go_cart *,
		go_cart *);
	go_result go_cart_mag(const go_cart *, go_real *);
	go_result go_cart_magsq(const go_cart *, go_real *);
	go_flag go_cart_cart_par(const go_cart *, const go_cart *);
	go_flag go_cart_cart_perp(const go_cart *, const go_cart *);

	/*! Places the Cartesian displacement between two vectors \a v1 and
	\a v2 in \a disp, returning \a GO_RESULT_OK. */
	go_result go_cart_cart_disp(const go_cart *v1,
		const go_cart *v2,
		go_real *disp);

	go_result go_cart_cart_add(const go_cart *, const go_cart *,
		go_cart *);
	go_result go_cart_cart_sub(const go_cart *, const go_cart *,
		go_cart *);
	go_result go_cart_scale_mult(const go_cart *, go_real, go_cart *);
	go_result go_cart_neg(const go_cart *, go_cart *);
	go_result go_cart_unit(const go_cart *, go_cart *);

	/*!
	Given two non-zero vectors \a v1 and \a v2, fill in \a quat with
	the minimum rotation that brings \a v1 to \a v2.
	*/
	go_result go_cart_cart_rot(const go_cart *v1,
		const go_cart *v2,
		go_quat *quat);

	/*!
	Project vector \a v1 onto \a v2, with the resulting vector placed
	into \a vout. Returns GO_RESULT_OK if it can be done, otherwise
	something else.
	*/
	go_result go_cart_cart_proj(const go_cart *v1, const go_cart *v2,
		go_cart *vout);
	go_result go_cart_plane_proj(const go_cart *, const go_cart *,
		go_cart *);
	go_result go_cart_cart_angle(const go_cart *, const go_cart *,
		go_real *);
	/*!
	go_cart_normal finds one of the infinite vectors perpendicular
	to \a v, putting the result in \a vout.
	*/
	go_result go_cart_normal(const go_cart *v, go_cart *vout);
	go_result go_cart_centroid(const go_cart *varray,
		go_integer num,
		go_cart *centroid);
	go_result go_cart_centroidize(const go_cart *vinarray,
		go_integer num,
		go_cart *centroid,
		go_cart *voutarray);

	/*!
	Given an array of Cartesian points measured in one coordinate system,
	and an associated array measured in another coordinate system,
	returns the best-fit transform that takes points in the first coordinate
	system to their representation in the second coordinate system.
	*/
	go_result go_cart_cart_pose
		(
		const go_cart *v1, /*!< An array of Cartesian points in one coordinate system  */
		const go_cart *v2, /*!< An array of the same Cartesian points in another coordinate system  */
		go_cart *v1c, /*!< A scratch array of the same size as \a v1 needed for calculation.  */
		go_cart *v2c, /*!< A scratch array of the same size as \a v2 needed for calculation. */
		go_integer num,		/*!< The number of points in the \a v1 and \a v2 arrays, which must be the same size.  */
		go_pose *pout			/*!< A pointer to the calculated best-fit pose filled in by this function. */
		);

	/*!
	Returns the Cartesian point \a p whose distances from three other points
	\a c1, \a c2 and \a c3 are \a l1, \a l2 and \a l3, respectively. In
	general there are 0, 1 or two points possible. If no point is possible,
	this returns GO_RESULT_ERROR, otherwise the points are returned in \a
	p1 and \a p2, which may be the same point.
	*/

	go_result go_cart_trilaterate(const go_cart *c1, 
		const go_cart *c2,
		const go_cart *c3,
		go_real l1,
		go_real l2,
		go_real l3,
		go_cart *out1,
		go_cart *out2);

	/* quat functions */

	go_flag go_quat_quat_compare(const go_quat *, const go_quat *);
	go_result go_quat_mag(const go_quat *, go_real *);

	/*!
	go_quat_unit takes a quaternion rotation \a q and converts it into
	a unit rotation about the same axis, \a qout.
	*/
	go_result go_quat_unit(const go_quat *q, go_quat *qout);
	go_result go_quat_norm(const go_quat *, go_quat *);
	go_result go_quat_inv(const go_quat *, go_quat *);
	go_flag go_quat_is_norm(const go_quat *);
	go_result go_quat_scale_mult(const go_quat *, go_real, go_quat *);
	go_result go_quat_quat_mult(const go_quat *, const go_quat *,
		go_quat *);
	go_result go_quat_cart_mult(const go_quat *, const go_cart *,
		go_cart *);

	/* rotation vector functions */

	go_flag go_rvec_rvec_compare(const go_rvec *r1, const go_rvec *r2);
	go_result go_rvec_scale_mult(const go_rvec *, go_real, go_rvec *);

	/* rpy functions */

	go_result go_rpy_cart_mult(const go_rpy *rpy, const go_cart *in, go_cart *out);

	/* rotation matrix functions */

	/*        |  m.x.x   m.y.x   m.z.x  |   */
	/*   M =  |  m.x.y   m.y.y   m.z.y  |   */
	/*        |  m.x.z   m.y.z   m.z.z  |   */

	/*!
	Normalizes rotation matrix \a m so that all columns are mutually
	perpendicular unit vectors, placing the result in \a mout.
	*/
	go_result go_mat_norm(const go_mat *m, go_mat *mout);

	go_flag go_mat_is_norm(const go_mat *);
	go_result go_mat_inv(const go_mat *, go_mat *);
	go_result go_mat_cart_mult(const go_mat *, const go_cart *, go_cart *);
	go_result go_mat_mat_mult(const go_mat *, const go_mat *, go_mat *);

	/* pose functions*/

	go_flag go_pose_pose_compare(const go_pose *, const go_pose *);
	go_result go_pose_inv(const go_pose *, go_pose *);
	go_result go_pose_cart_mult(const go_pose *, const go_cart *, go_cart *);
	go_result go_pose_pose_mult(const go_pose *, const go_pose *, go_pose *);
	go_result go_pose_scale_mult(const go_pose *, go_real, go_pose *);

	/*! Given two times \a t1 and \a t2, and associated poses \a p1 and \a
	p2, interpolates (or extrapolates) to find pose \a p3 at time \a
	t3. The result is stored in \a p3. Returns GO_RESULT_OK if \a t1 and
	\a t2 are distinct and \a p1 and \a p2 are valid poses, otherwise it
	can't interpolate and returns a relevant error. */
	go_result
		go_pose_pose_interp(go_real t1,
		const go_pose *p1,
		go_real t2, 
		const go_pose *p2,
		go_real t3, 
		go_pose *p3);

	/* homogeneous transform functions */

	go_result go_hom_inv(const go_hom *, go_hom *);
	go_result go_hom_hom_mult(const go_hom *h1, const go_hom *h2, go_hom *hout);

	/* velocity functions */

	/*! Given \a pose transformation from frame A to B, and a velocity \a vel
	in frame A, transform the velocity into frame B and place in \a out. */
	go_result go_pose_vel_mult(const go_pose *pose, const go_vel *vel, go_vel *out);

	/* declarations for general MxN matrices */

	/*!
	Declare a matrix variable \a m with \a rows rows and \a cols columns.
	Allocates \a rows X \a columns of space in \a mspace.
	*/

	typedef go_real go_vector;

	typedef struct {
		go_integer rows;
		go_integer cols;
		go_real **el;
		go_real **elcpy;
		go_real *v;
		go_integer *index;
	} go_matrix;

#define GO_MATRIX_DECLARE(M,Mspace,_rows,_cols) \
	go_matrix M = {0, 0, 0, 0, 0, 0}; \
	struct { \
	go_real *el[_rows]; \
	go_real *elcpy[_rows]; \
	go_real stg[_rows][_cols]; \
	go_real stgcpy[_rows][_cols]; \
	go_real v[_rows]; \
	go_integer index[_rows]; \
	} Mspace

	/* signed integer sizeof */
#define ISIZEOF(x) ((int) sizeof(x))

#define go_matrix_init(M,Mspace,_rows,_cols) \
	M.el = Mspace.el;				    \
	M.elcpy = Mspace.elcpy;				    \
	for (M.rows = 0; M.rows < (_rows) && M.rows < ISIZEOF(Mspace.el)/ISIZEOF(*(Mspace.el)); M.rows++) { \
	M.el[M.rows] = Mspace.stg[M.rows];	    \
	M.elcpy[M.rows] = Mspace.stgcpy[M.rows];	    \
	} \
	M.cols = (_cols) < ISIZEOF(Mspace.stg[0])/ISIZEOF(*(Mspace.stg[0])) ? (_cols) : ISIZEOF(Mspace.stg[0])/ISIZEOF(*(Mspace.stg[0])); \
	M.v = Mspace.v; \
	M.index = Mspace.index

	go_real
		go_get_singular_epsilon(void);

	go_result
		go_set_singular_epsilon(go_real epsilon);

	go_result
		ludcmp(go_real **a,
		go_real *scratchrow,
		go_integer n,
		go_integer *indx,
		go_real *d);

	go_result
		lubksb(go_real **a,
		go_integer n,
		go_integer *indx,
		go_real *b);

	/* MxN matrix, Mx1 vector functions */

	go_result
		go_cart_vector_convert(const go_cart *c,
		go_vector *v);
	go_result
		go_vector_cart_convert(const go_real *v,
		go_cart *c);

	go_result
		go_quat_matrix_convert(const go_quat *quat,
		go_matrix *matrix);

	go_result
		go_mat_matrix_convert(const go_mat *mat,
		go_matrix *matrix);

	go_result
		go_matrix_matrix_add(const go_matrix *a,
		const go_matrix *b,
		go_matrix *apb);

	go_result
		go_matrix_matrix_copy(const go_matrix *src,
		go_matrix *dst);

	go_result
		go_matrix_matrix_mult(const go_matrix *a,
		const go_matrix *b,
		go_matrix *ab);

	go_result
		go_matrix_vector_mult(const go_matrix *a,
		const go_vector *v,
		go_vector *av);

	/*!
	The matrix-vector cross product is a matrix of the same dimension,
	whose columns are the column-wise cross products of the matrix
	and the vector. The matrices must be 3xN, the vector 3x1.
	*/
	go_result
		go_matrix_vector_cross(const go_matrix *a,
		const go_vector *v,
		go_matrix *axv);

	go_result
		go_matrix_transpose(const go_matrix *a,
		go_matrix *at);

	go_result
		go_matrix_inv(const go_matrix *a,
		go_matrix *ainv);

	/* Square matrix functions, where matN is an NxN matrix, and vecN
	is an Nx1 vector */

	/* Optimized 3x3 functions */
	go_result go_mat3_inv(go_real a[3][3],
		go_real ainv[3][3]);
	go_result go_mat3_mat3_mult(go_real a[3][3],
		go_real b[3][3],
		go_real axb[3][3]);
	go_result go_mat3_vec3_mult(go_real a[3][3],
		go_real v[3],
		go_real axv[3]);

	/* Optimized 4x4 functions */
	go_result go_mat4_inv(go_real a[4][4],
		go_real ainv[4][4]);
	go_result go_mat4_mat4_mult(go_real a[4][4],
		go_real b[4][4],
		go_real axb[4][4]);
	go_result go_mat4_vec4_mult(go_real a[4][4],
		go_real v[4],
		go_real axv[4]);

	/*!
	Given a 6x6 matrix \a a, computes the inverse and returns it in
	\a ainv. Leaves \a a untouched. Returns GO_RESULT_OK if there is an
	inverse, else GO_RESULT_SINGULAR if the matrix is singular.
	*/
	go_result go_mat6_inv(go_real a[6][6],
		go_real ainv[6][6]);

	/*!
	Given a 6x6 matrix \a a, generates the transpose and returns it in
	\a at. Leaves \a a untouched. Returns GO_RESULT_OK.
	*/
	go_result go_mat6_transpose(go_real a[6][6],
		go_real at[6][6]);

	/*!
	Given two 6x6 matrices \a a and \a b, multiplies them and returns
	the result in \a axb. Leaves \a a and \a b untouched.
	Returns GO_RESULT_OK.
	*/
	go_result go_mat6_mat6_mult(go_real a[6][6],
		go_real b[6][6],
		go_real axb[6][6]);

	/*!
	Given a 6x6 matrix \a a and a 6x1 vector \a v, multiplies them
	and returns the result in \a axv. Leaves \a a and \a v untouched.
	Returns GO_RESULT_OK.
	*/
	go_result go_mat6_vec6_mult(go_real a[6][6],
		go_real v[6],
		go_real axv[6]);

	/* Denavit-Hartenberg to pose conversions */

	/*
	The link frams is assumed to be

	| i-1
	|    T
	|   i

	that is, elements of the link frame expressed wrt the
	previous link frame.
	*/

	/*!
	These DH parameters follow the convention in John J. Craig,
	_Introduction to Robotics: Mechanics and Control_.
	*/
	typedef struct {
		go_real a;			/*!< a[i-1] */
		go_real alpha;		/*!< alpha[i-1] */
		/* either d or theta are the free variable, depending on quantity */
		go_real d;			/*!< d[i] */
		go_real theta;		/*!< theta[i] */
	} go_dh;

	/*!
	PK parameters are used for parallel kinematic mechanisms, and
	represent the Cartesian positions of the ends of the link in the
	stationary base frame and the moving platform frame. Currently this
	only supports prismatic links.
	*/
	typedef struct {
		go_cart base;			/*!< position of fixed end in base frame */
		go_cart platform;		/*!< position of moving end in platform frame  */
		go_real d;			/*!< the length of the link */
	} go_pk;

	/*!
	PP parameters represent the pose of the link with respect to the
	previous link. Revolute joints rotate about the Z axis, prismatic
	joints slide along the Z axis.
	*/
	typedef struct {
		go_pose pose;		/*!< the pose of the link wrt to the previous link */
	} go_pp;

	/*!
	URDF parameters represent the pose of the link with respect to the
	previous link. Revolute joints rotate about the axis specified, prismatic
	joints slide along the axis specified.
	*/
	typedef struct {
		go_pose pose;		/*!< the pose of the link wrt to the previous link */
		go_cart axis;		/*!< the axis of rotation or translation */
	} go_urdf;

	/*! Rigid body */
	typedef struct {
		go_real mass;			/*!< total mass of the rigid body */
		/*!
		The \a inertia matrix is the 3x3 matrix of moments of inertia with
		respect to the body's origin.
		*/
		go_real inertia[3][3];
	} go_body;

#define go_body_init(b)				\
	(b)->mass = 1;					\
	(b)->inertia[0][0] = 1;				\
	(b)->inertia[0][1] = 0;				\
	(b)->inertia[0][2] = 0;				\
	(b)->inertia[1][0] = 0;				\
	(b)->inertia[1][1] = 1;				\
	(b)->inertia[1][2] = 0;				\
	(b)->inertia[2][0] = 0;				\
	(b)->inertia[2][1] = 0;				\
	(b)->inertia[2][2] = 1

#define go_body_copy(src,dst)			\
	(dst)->mass = (src)->mass;			\
	(dst)->inertia[0][0] = (src)->inertia[0][0];	\
	(dst)->inertia[0][1] = (src)->inertia[0][1];	\
	(dst)->inertia[0][2] = (src)->inertia[0][2];	\
	(dst)->inertia[1][0] = (src)->inertia[1][0];	\
	(dst)->inertia[1][1] = (src)->inertia[1][1];	\
	(dst)->inertia[1][2] = (src)->inertia[1][2];	\
	(dst)->inertia[2][0] = (src)->inertia[2][0];	\
	(dst)->inertia[2][1] = (src)->inertia[2][1];	\
	(dst)->inertia[2][2] = (src)->inertia[2][2]

	/*! Types of link parameter representations  */
	enum {
		GO_LINK_DH = 1,		/*!< for Denavit-Hartenberg params  */
		GO_LINK_PK,			/*!< for parallel kinematics  */
		GO_LINK_PP,			/*!< for serial kinematics */
		GO_LINK_URDF			/*!< Unified Robot Description Language */
	};

#define go_link_to_string(L)		\
	(L) == GO_LINK_DH ? "DH" :		\
	(L) == GO_LINK_PK ? "PK" :		\
	(L) == GO_LINK_PP ? "PP" :		\
	(L) == GO_LINK_URDF ? "URDF" : "None"

	/*!
	This is the generic link structure for PKM sliding/cable links and 
	serial revolute/prismatic links.
	*/
	typedef struct {
		union {
			go_dh dh; /*!< if you have DH params and don't want to convert to PP */
			go_pk pk; /*!< if you have a parallel machine, e.g., hexapod or robot crane */
			go_pp pp; /*!< if you have a serial machine, e.g., an industrial robot  */
			go_urdf urdf; /*!< if you have URDF parameters for a serial robot  */
		} u;
		go_body body;		       /*!< the link's rigid body parameters */
		go_flag type;		       /*!< one of GO_LINK_DH,PK,PP  */
		go_flag quantity;	       /*!< one of GO_QUANTITY_LENGTH,ANGLE */
	} go_link;

	/*!
	Converts DH parameters in \a dh to their transform equivalent, stored
	in \a hom.
	*/
	go_result go_dh_hom_convert(const go_dh *dh, go_hom *hom);

	/*!
	Converts DH parameters in \a dh to their pose equivalent, stored
	in \a pose.
	*/
	go_result go_dh_pose_convert(const go_dh *dh, go_pose *pose);

	/*!
	Converts \a pose to the equivalent DH parameters, stored in \a dh.
	Warning! Conversion from these DH parameters back to a pose via \a
	go_dh_pose_convert will NOT in general result in the same
	pose. Poses have 6 degrees of freedom, DH parameters have 4, and
	conversion to DH parameters loses some information. The source of
	this information loss is the convention imposed on DH parameters for
	choice of X-Y-Z axis directions. With poses, there is no such
	convention, and poses are thus freer than DH parameters.
	*/
	go_result go_pose_dh_convert(const go_pose *pose, go_dh *dh);

	/*!
	Counterpart to go_pose_dh_convert, for transforms.
	*/
	go_result go_hom_dh_convert(const go_hom *hom, go_dh *dh);

	/*!
	Fixes the link in \a link to its value when the joint
	variable is \a joint, storing the result in \a linkout.
	*/
	go_result go_link_joint_set(const go_link *link, go_real joint, go_link *linkout);

	/*!
	Takes the link description of the device in \a links, and the number
	of these in \a num, and builds the pose of the device and stores in
	\a pose. \a links should have the value of the free link parameter
	filled in with the current joint value, e.g., with a prior call to
	go_link_joint_set.
	*/
	go_result go_link_pose_build(const go_link *links, go_integer num, go_pose *pose);

	typedef struct {
		go_real re;
		go_real im;
	} go_complex;

	go_complex go_complex_add(go_complex z1, go_complex z2);
	go_complex go_complex_sub(go_complex z1, go_complex z2);
	go_complex go_complex_mult(go_complex z1, go_complex z2);
	go_complex go_complex_inv(go_complex z, go_result *result);
	go_complex go_complex_div(go_complex z1, go_complex z2, go_result *result);
	go_complex go_complex_sq(go_complex z);
	go_complex go_complex_scale(go_complex z, go_real scale);
	go_real go_complex_mag(go_complex z);
	go_real go_complex_magsq(go_complex z);
	go_real go_complex_arg(go_complex z);
	void go_complex_sqrt(go_complex z, go_complex *z1, go_complex *z2);
	void go_complex_cbrt(go_complex z, go_complex *z1, go_complex *z2, go_complex *z3);

	typedef struct {
		/* leading coefficient is 1, x^2 + ax + b = 0 */
		go_real a;
		go_real b;
	} go_quadratic;

	typedef struct {
		/* leading coefficient is 1, x^3 + ax^2 + bx + c = 0 */
		go_real a;
		go_real b;
		go_real c;
	} go_cubic;

	typedef struct {
		/* leading coefficient is 1, x^4 + ax^3 + bx^2 + cx + d = 0 */
		go_real a;
		go_real b;
		go_real c;
		go_real d;
	} go_quartic;

	go_result go_quadratic_solve(const go_quadratic *quad,
		go_complex *z1,
		go_complex *z2);

	go_result go_cubic_solve(const go_cubic *cub,
		go_complex *z1,
		go_complex *z2,
		go_complex *z3);

	go_result go_quartic_solve(const go_quartic *quart,
		go_complex *z1,
		go_complex *z2,
		go_complex *z3,
		go_complex *z4);

	go_result go_tridiag_reduce(go_real **a,
		go_integer n,
		go_real *d,
		go_real *e);

	go_result go_tridiag_ql(go_real *d,
		go_real *e,
		go_integer n,
		go_real **z);

	/*! Solves \a cos theta + \b b sin theta = 1 for theta, two solutions */
	go_result go_linear_cos_sin_solve(go_real a, go_real b, go_real *th1, go_real *th2);

};
#endif /* GO_MATH_H */
