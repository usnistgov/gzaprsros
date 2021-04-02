#ifndef GOINTERP_H
#define GOINTERP_H

#include "gotraj/gotypes.h"		/* go_real */

//typedef int go_result;
//typedef double go_real;
namespace gomotion
{
	/**
	How many coefficients we support, one more than max polynomial degree 
	*/
	enum { COEFF_MAX = 6 };

	/**
	This structure holds the points to be interpolated. See notes below for
	how to fill the p[] array for the particular type of interpolation.
	*/
	struct points {
		go_real p[COEFF_MAX];
	} ;

	/**
	This structure holds the polynomial coefficients. In the derivations
	below, coefficients a,b,c,d,... are used. Here, they correspond to the
	a[] array like this:

	a[0] = 0th order coeff, e.g., 'd' for cubic
	a[1] = 1st order coeff, e.g., 'c' for cubic
	...
	*/
	struct coeff{
		go_real a[COEFF_MAX];
	} ;

	/**
	The interpolator structure

	For constant- and linear interpolation, exact-fit and boundary interpolation
	is the same, so we only have one type for these. Here we pass single points
	in succession, and interpolation is done at/between them. 

	For higher-order interpolation, we have two types, exact-fit and boundary.
	For exact-fit, we pass single points in succession, and interpolation is
	done in the middle interval. For boundary, we pass n-tuples of pos, vel,
	accel, etc. and interpolation is done between each tuple. 

	A variation on boundary interpolation is possible if the derivative values
	are not known but are estimated by differencing. The functions below
	suffixed _est are used for this.
	*/


	class go_interp {
	public:
		coeff a;
		points p;

		go_result init();

		go_result set_here( go_real here);

		go_result calc_coeff_constant(const points * p,
			coeff * a);
		go_result calc_coeff_linear(const points * p,
			coeff * a);
		go_result calc_coeff_cubic_bc(const points * p,
			coeff * a);
		go_result calc_coeff_cubic_pf(const points * p,
			coeff * a);
		go_result calc_coeff_quintic_bc(const points * p,
			coeff * a);
		go_result calc_coeff_quintic_pf(const points * p,
			coeff * a);
		go_result calc_coeff_bc(go_integer order,
			const points * p,
			coeff * a);
		go_result calc_coeff_pf(go_integer order,
			const points * p,
			coeff * a);

		go_real eval_constant( go_real t);
		go_real eval_linear( go_real t);
		go_real eval_cubic(go_real t);
		go_real eval_quintic( go_real t);

		go_result add_constant( go_real pos);
		go_result add_linear(go_real pos);
		go_result add_cubic_pv( go_real pos,
			go_real vel);
		go_result add_cubic_pdv( go_real pos);
		go_result add_cubic_pf( go_real pos);
		go_result add_quintic_pva( go_real pos,
			go_real vel, go_real acc);
		go_result add_quintic_pvda( go_real pos,
			go_real vel);
		go_result add_quintic_pdva( go_real pos);
		go_result add_quintic_pf( go_real pos);

		typedef go_result (*add_func)( go_real pos);
		typedef go_real (*eval_func)(const  go_real t);
	} ;
};
#endif /* GOINTERP_H */
