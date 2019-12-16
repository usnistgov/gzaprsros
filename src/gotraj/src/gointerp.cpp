/*
  gointerp.c

  Polynomial interpolation of various orders: constant, linear, cubic, 
  quintic. There are two types, point fit and boundary fit. For an
  nth-order point fit, n+1 points are provided and the interpolating
  polynomial fits them exactly. For a boundary fit, (n+1)/2 tuples
  of points and derivatives are provided and the interpolating polynomial
  fits the points and derivatives exactly.

  Time is normalized to t = 0 at start of interval of interpolation,
  t = 1 at end.

  Point-fit interpolation is undesirable for motion control and is
  included here to be used for comparison.

  Boundary interpolation should be used for motion control. If the
  derivatives are not known, they can be estimated by differencing.
  Differencing introduces slight reversal and overshoot errors at the
  beginning and end of step motions, more pronounced the higher the degree.

  For constant- and linear interpolation, point-fit and boundary types
  are the same.
*/

#include "gotraj/gointerp.h"
namespace gomotion 
{

/* some useful multiplicative constants */
static const go_real f_1_2 = 1.0 / 2.0;
static const go_real f_1_6 = 1.0 / 6.0;
static const go_real f_1_120 = 1.0 / 120.0;

go_result go_interp::init()
{
  int t;

  for (t = 0; t < COEFF_MAX; t++) {
    this->a.a[t] = 0.0;
    this->p.p[t] = 0.0;
  }

  return GO_RESULT_OK;
}

go_result go_interp::set_here(go_real here)
{
  int t;

  for (t = 0; t < COEFF_MAX; t++) {
    this->a.a[t] = here;
    this->p.p[t] = here;
  }

  return GO_RESULT_OK;
}

/*
  Constant interpolation, trivial
*/

go_result go_interp::calc_coeff_constant(const points * p,
					coeff * a)
{
  a->a[0] = p->p[0];

  return GO_RESULT_OK;
}

go_result go_interp::add_constant(go_real pt)
{
  /* replace current point */
  this->p.p[0] = pt;
  /* compute new coeff */
  return calc_coeff_constant(&this->p, &this->a);
}

go_real go_interp::eval_constant(go_real t)
{
  return this->a.a[0];
}

/*
  Linear interpolation
*/

go_result go_interp::calc_coeff_linear(const points * p,
				      coeff * a)
{
  a->a[0] = p->p[0];
  a->a[1] = p->p[1] - p->p[0];

  return GO_RESULT_OK;
}

go_result go_interp::add_linear(go_real pt)
{
  /* shift points down */
  this->p.p[0] = this->p.p[1];
  this->p.p[1] = pt;
  /* compute new coeff */
  return calc_coeff_linear(&this->p, &this->a);
}

go_real go_interp::eval_linear(go_real t)
{
  return this->a.a[0] + this->a.a[1] * t;
}

/*
  Boundary-fit cubic (aka C1 continuity)

  Polynomial that fits both position-velocity pairs is

   at^3 +  bt^2 + ct + d = p(t)
  3at^2 + 2bt   + c      = v(t)

  At t = 0,
  
  d = p(0)
  c = v(0)

  At t = 1, we have:

  | 1 1 | . | a | = | p(1) - p(0) - v(0) | = | X |
  | 3 2 |   | b |   | v(1) - v(0)        |   | Y |

  or

  a = -2 X + Y
  b =  3 X - Y
*/

/*
  For boundary-fit cubics, the p[] array should be filled as follows:

  p[0] = p(0)
  p[1] = v(0)
  p[2] = p(1)
  p[3] = v(1)
*/

go_result go_interp::calc_coeff_cubic_bc(const points * p,
					coeff * a)
{
  go_real X;
  go_real Y;

  X = p->p[2] - p->p[0] - p->p[1];
  Y = p->p[3] - p->p[1];

  a->a[0] = p->p[0];		/* d */
  a->a[1] = p->p[1];		/* c */
  a->a[2] = 3 * X - Y;		/* b */
  a->a[3] = -2 * X + Y;		/* a */

  return GO_RESULT_OK;
}

/*
  Point-fit cubic

  Polynomial that fits all 4 position points at t = -1,0,1,2 is

  at^3 + bt^2 + ct + d = p(t).

  At t = 0, d = p(0).

  At t = -1,1,2, we have:

  | -1  1 -1 |   | a |   | p(-1) - p(0) | = | X |
  |  1  1  1 | . | b | = | p(1)  - p(0) | = | Y |
  |  8  4  2 |   | c |   | p(2)  - p(0) | = | Z |

  The leftmost matrix inverts to 

  | -1 -3  1 |
  |  3  3  0 | / 6
  | -2  6 -1 |

  leading to solutions for a,b,c as

  a = (-X - 3Y + Z)/6
  b = (X + Y)/2
  c = (-2X + 6Y - Z)/6
*/

/*
  For point-fit cubics, the p[] array should be filled as follows:

  p[0] = p(-1)
  p[1] = p(0)
  p[2] = p(1)
  p[3] = p(2)

  That is, the smaller the index the earlier the point.
*/

go_result go_interp::calc_coeff_cubic_pf(const points * p,
					coeff * a)
{
  go_real X;
  go_real Y;
  go_real Z;

  X = p->p[0] - p->p[1];
  Y = p->p[2] - p->p[1];
  Z = p->p[3] - p->p[1];

  a->a[0] = p->p[1];		/* d */
  a->a[1] = f_1_6 * (-2 * X + 6 * Y - Z);	/* c */
  a->a[2] = f_1_2 * (X + Y);	/* b */
  a->a[3] = f_1_6 * (-X - 3 * Y + Z);	/* a */

  return GO_RESULT_OK;
}

/*
  Add a pos-vel pair to the boundary cubic interpolator. This shifts the
  last p-v pair to the start, adds the new pair at the end, then computes
  the cubic coefficients over the interval. Calling this twice from scratch
  will fill the interpolator and make it ready to go.
*/
go_result go_interp::add_cubic_pv(go_real pos, go_real vel)
{
  /* shift p-v pair down */
  this->p.p[0] = this->p.p[2];
  this->p.p[1] = this->p.p[3];
  /* add new p-v pair */
  this->p.p[2] = pos;
  this->p.p[3] = vel;
  /* compute new coeff */
  return calc_coeff_cubic_bc(&this->p, &this->a);
}

/*
  Add a point to the boundary cubic interpolator. Since you are not providing
  the necessary velocity partner, it will be estimated by differencing the
  positions. First the point is added by shifting the previous, as with
  the exact-fit interpolator. Then two intermediate p-v pairs are computed
  for the middle of the three intervals, by averaging the differenced 
  velocities at the ends of the middle interval. The boundary cubic
  coefficients are the computed. Calling this four times from scratch will
  fill the interpolator and make it ready to go.

  This gives better behavior than the exact fit cubic for motion control,
  although not as good as if the p-v pairs were computed exactly, from
  the inverse Jacobian for example.
 */
go_result go_interp::add_cubic_pdv( go_real pt)
{
  points p;
  go_real v0, v1;
  go_real wp;
  go_real k = 5.0;
  go_real kp2_inv = 1.0 / (k + 2.0);

  /* estimate boundary values */
  v0 = 0.5 * (this->p.p[2] - this->p.p[0]);
  v1 = 0.5 * (pt - this->p.p[1]);
  /* set up point structure */
  wp = this->p.p[1];		/* original */
  wp = kp2_inv * (this->p.p[0] + k * this->p.p[1] + this->p.p[2]);	/* blended */
  p.p[0] = wp;
  p.p[1] = v0;
  wp = this->p.p[2];		/* original */
  wp = kp2_inv * (this->p.p[1] + k * this->p.p[2] + pt);	/* blended */
  p.p[2] = wp;
  p.p[3] = v1;
  /* shift points down */
  this->p.p[0] = this->p.p[1];
  this->p.p[1] = this->p.p[2];
  /* add new point */
  this->p.p[2] = pt;
  /* calculate coeff from point structure */
  return calc_coeff_cubic_bc(&p, &this->a);
}

/*
  Add a point to the point-fit cubic interpolator. This shifts all the previous
  points down one toward the start, dropping the first (earliest) point
  and adding the new point at the end. It then computes the exact coefficients
  over the middle of the three intervals. Calling this four times from scratch
  will fill the interpolator and make it ready to go.
*/
go_result go_interp::add_cubic_pf( go_real pt)
{
  /* shift points down */
  this->p.p[0] = this->p.p[1];
  this->p.p[1] = this->p.p[2];
  this->p.p[2] = this->p.p[3];
  /* add new point */
  this->p.p[3] = pt;
  /* compute new coeff */
  return calc_coeff_cubic_pf(&this->p, &this->a);
}

/*
  Cubic interpolator, same for both boundary- and point-fit cubics
*/
go_real go_interp::eval_cubic(go_real t)
{
  return ((this->a.a[3] * t + this->a.a[2]) * t + this->a.a[1]) * t + this->a.a[0];
}

/*
  Boundary-fit quintic (aka C2 continuity)

  Polynomial that fits both position-velocity-acceleration triples is

  a   t^5 +   b t^4 +  c t^3 +  d t^2 + e t + f = p(t)
  5a  t^4 +  4b t^3 + 3c t^2 + 2d t + e         = v(t)
  20a t^3 + 12b t^2 + 6c t   + 2d               = a(t)

  At t = 0,

  p(0) = f
  v(0) = e
  a(0) = 2d -> d = a(0)/2

  At t = 1, we have:

  |  1  1  1 |   | a |   | p(1) - p(0) - v(0) - a(0)/2 | = | X |
  |  5  4  3 | . | b | = | v(1) - v(0) - a(0)          | = | Y |
  | 20 12  6 |   | c |   | a(1) - a(0)                 | = | Z |

  or

  a =   6 X - 3 Y + 1/2 Z
  b = -15 X + 7 Y -     Z
  c =  10 X - 4 Y + 1/2 Z
*/

/*
  For boundary-fit quintics, the p[] array should be filled as follows:

  p[0] = p(0)
  p[1] = v(0)
  p[2] = a(0)
  p[3] = p(1)
  p[4] = v(1)
  p[5] = a(1)
*/

go_result go_interp::calc_coeff_quintic_bc(const points * p,
					  coeff * a)
{
  go_real X;
  go_real Y;
  go_real Z;

  X = p->p[3] - p->p[0] - p->p[1] - 0.5 * p->p[2];
  Y = p->p[4] - p->p[1] - p->p[2];
  Z = p->p[5] - p->p[2];

  a->a[0] = p->p[0];		/* f */
  a->a[1] = p->p[1];		/* e */
  a->a[2] = f_1_2 * p->p[2];	/* d */
  a->a[3] = 10 * X - 4 * Y + f_1_2 * Z;	/* c */
  a->a[4] = -15 * X + 7 * Y - Z;	/* b */
  a->a[5] = 6 * X - 3 * Y + f_1_2 * Z;	/* a */

  return GO_RESULT_OK;
}

/*
  Point-fit quintic

  Polynomial that fits all 6 position points at t = -2,-1,0,1,2,3 is

  at^5 + bt^4 + ct^3 + dt^2 + et + f = p(t).

  At t = 0, f = p(0).

  At t = -2,-1,1,2,3 we have:

  | -32 16 -8 4 -2 |   | a |   | p(-2) - p(0) | = | X |
  |  -1  1 -1 1 -1 | . | b | = | p(-1) - p(0) | = | Y |
  |   1  1  1 1  1 |   | c |   | p(1)  - p(0) | = | Z |
  |  32 16  8 4  2 |   | d |   | p(2)  - p(0) | = | U |
  | 243 81 27 9  3 |   | e |   | p(3)  - p(0) | = | V |

  The leftmost matrix inverts to

  | -1   5  10  -5  1 |
  |  5 -20 -20   5  0 |
  | -5  -5 -70  35 -5 | / 120 
  | -5  80  80  -5  0 |
  |  6 -60 120 -30  4 |

  leading to solutions for a,b,c,d,e as

  a = ( -X +  5Y +  10Z -   5U +  V) / 120
  b = ( 5X - 20Y -  20Z +   5U     ) / 120
  c = (-5X -  5Y -  70Z +  35U - 5V) / 120
  d = (-5X + 80Y +  80Z -   5U     ) / 120
  e = ( 6X - 60Y +  120Z - 30U + 4V) / 120
*/

/*
  For point-fit quintics, the p[] array should be filled as follows:

  p[0] = p(-2)
  p[1] = p(-1)
  p[2] = p(0)
  p[3] = p(1)
  p[4] = p(2)
  p[5] = p(3)

  That is, the smaller the index the earlier the point.
*/

go_result go_interp::calc_coeff_quintic_pf(const points * p,
					  coeff * a)
{
  go_real X;
  go_real Y;
  go_real Z;
  go_real U;
  go_real V;

  X = p->p[0] - p->p[2];
  Y = p->p[1] - p->p[2];
  Z = p->p[3] - p->p[2];
  U = p->p[4] - p->p[2];
  V = p->p[5] - p->p[2];

  a->a[0] = p->p[2];		/* f */
  a->a[1] = f_1_120 * (6 * X - 60 * Y + 120 * Z - 30 * U + 4 * V);
  a->a[2] = f_1_120 * (-5 * X + 80 * Y + 80 * Z - 5 * U);
  a->a[3] = f_1_120 * (-5 * X - 5 * Y - 70 * Z + 35 * U - 5 * V);
  a->a[4] = f_1_120 * (5 * X - 20 * Y - 20 * Z + 5 * U);
  a->a[5] = f_1_120 * (-X + 5 * Y + 10 * Z - 5 * U + V);

  return GO_RESULT_OK;
}

/*
  Add a pos-vel-acc triple to the boundary quintic interpolator. This shifts
  the last p-v-a triple to the start, adds the new triple at the end,
  then computes the quintic coefficients over the interval. Calling this
  twice from scratch will fill the interpolator and make it ready to go.
*/
go_result go_interp::add_quintic_pva( go_real pos, go_real vel,
				    go_real acc)
{
  /* shift p-v pair down */
  this->p.p[0] = this->p.p[3];
  this->p.p[1] = this->p.p[4];
  this->p.p[2] = this->p.p[5];
  /* add new p-v pair */
  this->p.p[3] = pos;
  this->p.p[4] = vel;
  this->p.p[5] = acc;
  /* compute new coeff */
  return calc_coeff_quintic_bc(&this->p, &this->a);
}

/*
  Quintic PVDA gets pos-vel pairs, and estimates acc by differencing.

  When called, the points look like this:

   v0  v1 v2 vel
  (p0) p1 p2 pos

  in the order p0 v0 p1 v1 p2 v2, with pos vel as args.

  p0 is not used for the acc calculations and is ignored.

  Boundary coefficients are computed for the interval bounded by pv1 and pv2.
 */
go_result go_interp::add_quintic_pvda( go_real pos, go_real vel)
{
  points p;
  go_real a0, a1;

  /* estimate boundary values */
  a0 = 0.5 * (this->p.p[5] - this->p.p[1]);
  a1 = 0.5 * (vel - this->p.p[3]);
  /* set up p-v structure */
  p.p[0] = this->p.p[2];
  p.p[1] = this->p.p[3];
  p.p[2] = a0;
  p.p[3] = this->p.p[4];
  p.p[4] = this->p.p[5];
  p.p[5] = a1;
  /* shift pairs down, ignoring the unused this->p.p[0] */
  this->p.p[1] = this->p.p[3];
  this->p.p[2] = this->p.p[4];
  this->p.p[3] = this->p.p[5];
  /* add new pair */
  this->p.p[4] = pos;
  this->p.p[5] = vel;
  /* compute new coeff */
  return calc_coeff_quintic_bc(&p, &this->a);
}

/*
  Quintic PDVA gets points, and estimates the vel and acc by differencing.

  When called, the points look like this:

  p0 p1 p2 p3 p4 pos

  where p5 is unused, since we use 'pos' directly.

  Boundary coefficients are computed for the interval bounded by p2 and p3.
 */
go_result go_interp::add_quintic_pdva( go_real pos)
{
  points p;
  go_real v0, v1, a0, a1;
  go_real wp;

  /* estimate boundary values */
  v0 = 0.5 * (this->p.p[3] - this->p.p[1]);
  v1 = 0.5 * (this->p.p[4] - this->p.p[2]);
  a0 = 0.25 * (this->p.p[4] - this->p.p[2] - this->p.p[2] + this->p.p[0]);
  a1 = 0.25 * (pos - this->p.p[3] - this->p.p[3] + this->p.p[1]);
  /* set up p-v-a structure */
  wp =
    (-0.086 * this->p.p[0] + 0.343 * this->p.p[1] + 0.486 * this->p.p[2] +
     0.343 * this->p.p[3] + -0.086 * this->p.p[4]);
  wp = (this->p.p[1] + 2.5 * this->p.p[2] + this->p.p[3]) / 4.5;
  wp = this->p.p[2];		/* original */
  p.p[0] = wp;
  p.p[1] = v0;
  p.p[2] = a0;
  wp =
    (-0.086 * this->p.p[1] + 0.343 * this->p.p[2] + 0.486 * this->p.p[3] +
     0.343 * this->p.p[4] + -0.086 * pos);
  wp = (this->p.p[2] + 2.5 * this->p.p[3] + this->p.p[4]) / 4.5;
  wp = this->p.p[3];		/* original */
  p.p[3] = wp;
  p.p[4] = v1;
  p.p[5] = a1;
  /* shift points down, ignoring unused this->p.p[5] */
  this->p.p[0] = this->p.p[1];
  this->p.p[1] = this->p.p[2];
  this->p.p[2] = this->p.p[3];
  this->p.p[3] = this->p.p[4];
  /* add new pt */
  this->p.p[4] = pos;
  /* compute new coeff */
  return calc_coeff_quintic_bc(&p, &this->a);
}

/*
  Point-fit cubic uses all points directly
*/
go_result go_interp::add_quintic_pf( go_real pos)
{
  /* shift points down */
  this->p.p[0] = this->p.p[1];
  this->p.p[1] = this->p.p[2];
  this->p.p[2] = this->p.p[3];
  this->p.p[3] = this->p.p[4];
  this->p.p[4] = this->p.p[5];
  /* add new point */
  this->p.p[5] = pos;
  /* compute new coeff */
  return calc_coeff_quintic_pf(&this->p, &this->a);
}

/*
  Quintic interpolator, same for both boundary- and point-fit quintics
*/
go_real go_interp::eval_quintic(go_real t)
{
  return ((((this->a.a[5] * t + this->a.a[4]) * t + this->a.a[3]) * t + this->a.a[2]) *
	  t + this->a.a[1]) * t + this->a.a[0];
}
};
