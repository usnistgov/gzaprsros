#ifndef GOTRAJ_H
#define GOTRAJ_H

#include <math.h>		/* sqrt() */
#include "gotraj/gotypes.h"		/* go_result, go_real */
#include "gotraj/gomath.h"		/* go_cbrt */

#define reciprocate(x) (x) <= 0.0 ? GO_INF : 1.0 / (x)
namespace gomotion {
	struct go_traj_ca_spec {
		go_real at0;			/**< accel for Phase I and III */
		go_real t1;			/**< cumulative time at end of Phase I */
		go_real dt1;			/**< cumulative distance at end of Phase I */
		go_real vt1;			/**< speed at end of Phase I/in Phase II */
		go_real t2;			/**< cumulative time at end of Phase II */
		go_real dt2;			/**< cumulative distance at end of Phase II */
		go_real tend;			/**< total time for motion */
		go_real dtend;		/**< total distance for motion */
		go_real invd;			/**< inverse of total distance */
	} ;

	struct go_traj_cj_spec{
		go_real jt0;			/**< jerk in Phases I, III, V, VII */
		go_real t1;			/**< cumulative time at end of Phase I */
		go_real dt1;			/**< cumulative distance at end of Phase I */
		go_real vt1;			/**< speed at end of Phase I */
		go_real at1;			/**< accel at end of Phase I */
		go_real t2;			/**< cumulative time at end of Phase II */
		go_real dt2;			/**< cumulative distance at end of Phase II */
		go_real vt2;			/**< speed at end of Phase II */
		go_real t3;			/**< cumulative time at end of Phase III */
		go_real dt3;			/**< cumulative distance at end of Phase III */
		go_real vt3;			/**< speed at end of Phase III/in Phase IV */
		go_real t4;			/**< cumulative time at end of Phase IV */
		go_real dt4;			/**< cumulative distance at end of Phase IV */
		go_real t5;			/**< cumulative time at end of Phase V */
		go_real dt5;			/**< cumulative distance at end of Phase V */
		go_real t6;			/**< cumulative time at end of Phase VI */
		go_real dt6;			/**< cumulative distance at end of Phase VI */
		go_real tend;			/**< total time for motion */
		go_real dtend;		/**< total distance for motion */
		go_real invd;			/**< inverse of total distance */
	} ;

	typedef struct {
		go_real s, d, v, a, j;
	} go_traj_interp_spec;

	/**
	go_traj_ca_generate() takes an accel value, and intervals for the accel period
	and cruise period, and fills in the go_traj_ca_spec with the
	interval parameters. This is useful for generating test cases.
	*/

	go_result go_traj_ca_generate(go_real acc, go_real deltacc,
		go_real deltvel, go_traj_ca_spec * pts);

	/**
	go_traj_ca_compute() takes values for distance 'd' to move, max velocity 'v' to
	limit if necessary, and constant accel 'a', and fills in the go_traj_ca_spec
	with the interval parameters.
	*/

	go_result go_traj_ca_compute(go_real d, go_real v, go_real a,
		go_traj_ca_spec * pts);

	/**
	go_traj_ca_scale() takes a time 't' for the desired time of the motion,
	and scales the times and motion params so that the total distance
	remains the same and everything else is in proportion 
	*/

	go_result go_traj_ca_scale(const go_traj_ca_spec * ts, go_real t,
		go_traj_ca_spec * pts);

	/**
	go_traj_ca_stop() takes a time 't' for the desired time to begin
	stopping, and recomputes the times so that the move will stop as
	soon as possible during subsequent interps.
	*/

	go_result go_traj_ca_stop(const go_traj_ca_spec * ts, go_real t,
		go_traj_ca_spec * pts);

	/**
	go_traj_ca_extend() takes a time 't' for the desired time to finish
	the motion, and extends the constant-speed section so that it stops
	then. The time must be shorter than the original time, and longer
	than the fastest stopping time.
	*/

	go_result go_traj_ca_extend(const go_traj_ca_spec * ts, go_real t,
		go_traj_ca_spec * pts);

	/**
	go_traj_ca_interp() takes a go_traj_ca_spec and interpolates the d-v-a
	values for the given time t, storing the d-v-a values in ti.
	*/

	go_result go_traj_ca_interp(const go_traj_ca_spec * ts, go_real t,
		go_traj_interp_spec * ti);

	/**
	go_traj_cj_generate() takes a jerk value, and intervals for the jerk period,
	accel period and cruise period, and fills in the go_traj_cj_spec with the
	interval parameters. This is useful for generating test cases.
	*/

	go_result go_traj_cj_generate(go_real jrk, go_real deltjrk,
		go_real deltacc, go_real deltvel,
		go_traj_cj_spec * pts);

	/**
	go_traj_cj_compute() takes values for distance 'd' to move, max velocity 'v' to
	limit if necessary, max accel 'a' to limit if necessary, and constant
	jerk 'j', and fills in the go_traj_cj_spec with the interval parameters.
	*/

	go_result go_traj_cj_compute(go_real d, go_real v, go_real a,
		go_real j, go_traj_cj_spec * pts);

	/**
	go_traj_cj_scale() takes a time 't' for the desired time of the motion,
	and scales the times and motion params so that the total distance
	remains the same and everything else is in proportion 
	*/

	go_result go_traj_cj_scale(const go_traj_cj_spec * ts, go_real t,
		go_traj_cj_spec * pts);

	/**
	go_traj_cj_stop() takes a time 't' for the desired time to begin
	stopping, and recomputes the times so that the move will stop as
	soon as possible during subsequent interps.
	*/

	go_result go_traj_cj_stop(const go_traj_cj_spec * ts, go_real t,
		go_traj_cj_spec * pts);

	/**
	go_traj_cj_extend() takes a time 't' for the desired time to finish
	the motion, and extends the constant-speed section so that it stops
	then. The time must be shorter than the original time, and longer
	than the fastest stopping time.
	*/

	go_result go_traj_cj_extend(const go_traj_cj_spec * ts, go_real t,
		go_traj_cj_spec * pts);

	/**
	go_traj_cj_interp() takes a go_traj_cj_spec and interpolates the d-v-a-j
	values for the given time t, storing the d-v-a-j values in ti.
	*/

	go_result go_traj_cj_interp(const go_traj_cj_spec * ts, go_real t,
		go_traj_interp_spec * ti);

	/**
	gotraj.c

	Trajectory planning functions

	CV means constant velocity, CA means constant acceleration,
	CJ means constant jerk.
	*/

	/**!
	\defgroup TRAJ Trajectory Planning Algorithms

	The Go Motion trajectory planning algorithms are based on smooth
	velocity profiling with bounded speed, acceleration and jerk, called
	"constant jerk" or "S-curve" velocity profiling. This gives smoother
	control than "trapezoidal" velocity profiling, which transitions
	instantaneously between acceleration and no acceleration and incurs
	spikes in unbounded jerk.

	Constant-jerk (CJ) profiling is shown in Figure 1, a plot of the
	speed versus time. There are 7 phases to the motion. Phase 1 is a
	jerk phase, where the acceleration varies smoothly from 0 at time 0
	to \a a1 at time \a t1 following the jerk (change in acceleration per unit
	time) \a j0. Phase 2 is an acceleration phase, with
	constant acceleration \a a1 throughout. Phase 3 is a jerk phase (or
	de-jerk phase) with constant (negative) jerk slowing down the
	acceleration from \a a1 to 0. Phase 4 is a constant speed phase at
	speed \a v3. Phase 5 is a constant-jerk counterpart to phase 3,
	where the deceleration varies smoothly from 0 to \a -a1. Phase 6 is
	a constant-acceleration counterpart to phase 2. Phase 7 is a
	constant-jerk counterpart to phase 1, where the deceleration varies
	smoothly from \a -a1 to 0 and motion stops.
	\image html fig001.png
	Figure 1. Constant jerk velocity profiling.
	*/


	/** constant acceleration functions */

	inline go_result go_traj_ca_generate(go_real acc, go_real deltacc, go_real deltvel,
		go_traj_ca_spec * pts)
	{
		go_traj_ca_spec ts;

		if (acc <= 0. || deltacc <= 0.) {
			return GO_RESULT_ERROR;
		}

		/** any time intervals less than 0. mean 0. */
		if (deltvel < 0.)
			deltvel = 0.;

		/** compute intermediate values for end of each phase */

		ts.at0 = acc;

		ts.t1 = deltacc;
		ts.dt1 = 1. / 2. * acc * go_sq(deltacc);
		ts.vt1 = acc * deltacc;

		ts.t2 = ts.t1 + deltvel;
		ts.dt2 = ts.dt1 + ts.vt1 * deltvel;

		ts.tend = ts.t2 + deltacc;
		ts.dtend = ts.dt2 + ts.dt1;
		ts.invd = reciprocate(ts.dtend);

		*pts = ts;

		return GO_RESULT_OK;
	}

	inline go_result go_traj_ca_compute(go_real d, go_real v, go_real a,
		go_traj_ca_spec * pts)
	{
		go_traj_ca_spec ts;

		if (d < 0. || v <= 0. || a <= 0.) {
			return GO_RESULT_ERROR;
		}

		ts.at0 = a;

		if (d * a > go_sq(v)) {
			/** cruise phase */

			ts.t1 = v / a;
			ts.dt1 = 1. / 2. * a * go_sq(ts.t1);
			ts.vt1 = v;

			ts.t2 = ts.t1 + (d - ts.dt1 - ts.dt1) / v;
			ts.dt2 = d - ts.dt1;

			ts.tend = ts.t2 + ts.t1;
			ts.dtend = d;
		} else {
			/** no cruise phase */

			ts.t1 = sqrt(d / a);
			ts.dt1 = d / 2.;
			ts.vt1 = a * ts.t1;

			ts.t2 = ts.t1;
			ts.dt2 = ts.dt1;

			ts.tend = ts.t1 + ts.t1;
			ts.dtend = d;
		}

		ts.invd = reciprocate(ts.dtend);

		*pts = ts;

		return GO_RESULT_OK;
	}

	inline go_result go_traj_ca_scale(const go_traj_ca_spec * ts, go_real t,
		go_traj_ca_spec * pts)
	{
		go_traj_ca_spec ots;

		if (ts->tend <= 0. || t <= 0.) {
			ots.t1 = 0.;
			ots.t2 = t;
			ots.tend = t;

			ots.dt1 = ts->dt1;
			ots.dt2 = ts->dt2;
			ots.dtend = ts->dtend;

			ots.vt1 = 0.;
			ots.at0 = 0.;
			ots.invd = ts->invd;
		} else if (t <= ts->tend) {
			/** can't shrink motion */
			ots = *ts;
		} else {
			/** this just adds a cruise phase to stretch the time, and reduces
			the acc value accordingly */
			ots.t1 = ts->t1;
			ots.t2 = t - ts->t1;
			ots.tend = t;

			ots.at0 = ts->dtend / (ts->t1 * (t - ts->t1));
			ots.vt1 = ots.at0 * ots.t1;

			ots.dt1 = 0.5 * ots.t1 * ots.vt1;
			ots.dt2 = ts->dtend - ots.dt1;
			ots.dtend = ts->dtend;
			ots.invd = ts->invd;
		}

		*pts = ots;

		return GO_RESULT_OK;
	}

	inline go_result go_traj_ca_stop(const go_traj_ca_spec * ts, go_real t,
		go_traj_ca_spec * pts)
	{
		go_traj_ca_spec ots;

		if (t <= 0.) {
			ots.at0 = ts->at0;
			ots.t1 = 0.;
			ots.dt1 = 0.;
			ots.vt1 = ts->vt1;
			ots.t2 = 0.;
			ots.dt2 = 0.;
			ots.tend = 0.;
			ots.dtend = 0.;
			ots.invd = GO_INF;
		} else if (t < ts->t1) {
			/** stage I */
			ots.at0 = ts->at0;
			ots.t1 = t;
			ots.dt1 = 0.5 * ts->at0 * go_sq(t);
			ots.vt1 = ts->at0 * t;
			/** null stage II, same as I */
			ots.t2 = t;
			ots.dt2 = ots.dt1;
			/** stage III */
			ots.tend = t + t;
			ots.dtend = ots.dt1 + ots.dt1;
			ots.invd = reciprocate(ots.dtend);
		} else if (t < ts->t2) {
			/** stage I unchanged */
			ots.at0 = ts->at0;
			ots.t1 = ts->t1;
			ots.dt1 = ts->dt1;
			ots.vt1 = ts->vt1;
			/** stage II */
			ots.t2 = t;
			ots.dt2 = ots.dt1 + ts->vt1 * (t - ts->t1);
			/** stage III */
			ots.tend = t + ts->t1;
			ots.dtend = ots.dt2 + ts->dt1;
			ots.invd = reciprocate(ots.dtend);
		} else {
			/** already in stage III, do nothing */
			ots = *ts;
		}

		*pts = ots;

		return GO_RESULT_OK;
	}

	inline go_result go_traj_ca_extend(const go_traj_ca_spec * ts, go_real t,
		go_traj_ca_spec * pts)
	{
		go_real tincr;
		go_real dincr;

		*pts = *ts;

		if (t < ts->tend) {
			return GO_RESULT_OK;
		}

		tincr = t - ts->tend;
		dincr = ts->vt1 * tincr;

		pts->t2 += tincr;
		pts->dt2 += dincr;
		pts->tend = t;
		pts->dtend += dincr;
		pts->invd = reciprocate(pts->dtend);
		return GO_RESULT_OK;
	}

	/**
	run interpolation on each phase for each motion parameter, using

	d = d0 + v0 t + 1/2 a0 t^2
	v =      v0   +     a0 t  
	a =                 a0
	j = 0

	where d0, v0, a0 are the initial values for each phase
	*/

	inline go_result go_traj_ca_interp(const go_traj_ca_spec * ts, go_real t,
		go_traj_interp_spec * ti)
	{
		go_real a, v, d;
		go_real deltat;

		if (t < 0.) {
			a = 0.;
			v = 0.;
			d = 0.;
			t = 0.;
		} else if (t < ts->t1) {
			a = ts->at0;
			v = ts->at0 * t;
			d = 0.5 * ts->at0 * go_sq(t);
		} else if (t < ts->t2) {
			deltat = t - ts->t1;
			a = 0.;
			v = ts->vt1;
			d = ts->dt1 + ts->vt1 * deltat;
		} else if (t < ts->tend) {
			deltat = t - ts->t2;
			a = -ts->at0;
			v = ts->vt1 - ts->at0 * deltat;
			d = ts->dt2 + ts->vt1 * deltat - 1. / 2. * ts->at0 * go_sq(deltat);
		} else {
			a = 0.;
			v = 0.;
			d = ts->dtend;
			t = ts->tend;
		}

		ti->j = 0.;
		ti->a = a;
		ti->v = v;
		ti->d = d;

		if (ts->invd == GO_INF) {
			ti->s = 1.;
		} else {
			ti->s = d * ts->invd;
		}

		return GO_RESULT_OK;
	}

	/** constant jerk functions */

	inline go_result go_traj_cj_generate(go_real jrk, go_real deltjrk, go_real deltacc,
		go_real deltvel, go_traj_cj_spec * pts)
	{
		go_traj_cj_spec ts;

		if (jrk <= 0. || deltjrk <= 0.) {
			return GO_RESULT_ERROR;
		}

		/** any time intervals less than 0. mean 0. */
		if (deltacc < 0.)
			deltacc = 0.;
		if (deltvel < 0.)
			deltvel = 0.;

		/** compute intermediate values for end of each phase */

		ts.jt0 = jrk;

		ts.t1 = deltjrk;
		ts.dt1 = 1. / 6. * jrk * go_cub(deltjrk);
		ts.vt1 = 1. / 2. * jrk * go_sq(deltjrk);
		ts.at1 = jrk * deltjrk;

		ts.t2 = ts.t1 + deltacc;
		ts.dt2 = ts.dt1 + ts.vt1 * deltacc + 1. / 2. * ts.at1 * go_sq(deltacc);
		ts.vt2 = ts.vt1 + ts.at1 * deltacc;

		ts.t3 = ts.t2 + deltjrk;
		ts.dt3 =
			ts.dt2 + ts.vt2 * deltjrk + 1. / 2. * ts.at1 * go_sq(deltjrk) -
			1. / 6. * jrk * go_cub(deltjrk);
		ts.vt3 = ts.vt2 + ts.vt1;

		ts.t4 = ts.t3 + deltvel;
		ts.dt4 = ts.dt3 + ts.vt3 * deltvel;

		ts.t5 = ts.t4 + deltjrk;
		ts.dt5 = ts.dt4 + ts.dt3 - ts.dt2;

		ts.t6 = ts.t5 + deltacc;
		ts.dt6 = ts.dt5 + ts.dt2 - ts.dt1;

		ts.tend = ts.t6 + deltjrk;
		ts.dtend = ts.dt6 + ts.dt1;
		ts.invd = reciprocate(ts.dtend);

		*pts = ts;

		return GO_RESULT_OK;
	}

	/**!
	Expects \a pts attributes \a dtend, \a jt0, \a at1, \a vt3,
	\a t1, \a t2 and \a t3 to be already filled in from previous calls
	to one of the four \a go_traj_cj_compute_tees functions and
	\a go_traj_cj_compute.

	Fills in all the remaining values in \a pts: the times for each of
	the seven phases, the associated cumulative distances, and the
	instantaneous speeds and accelerations.
	*/
	inline go_result go_traj_cj_compute_the_rest(go_traj_cj_spec * pts)
	{
		pts->at1 = pts->jt0 * pts->t1;
		pts->vt1 = 1./2. * pts->at1 * pts->t1;
		pts->dt1 = 1./3. * pts->vt1 * pts->t1;

		pts->vt2 = pts->vt1 + pts->at1 * (pts->t2 - pts->t1);
		pts->vt3 = pts->vt1 + pts->vt2;
		pts->dt2 = pts->dt1 + 1./2. * pts->vt3 * (pts->t2 - pts->t1);
		pts->t3 = pts->t1 + pts->t2;
		pts->dt3 = 1./2. * pts->vt3 * pts->t3;

		pts->dt4 = pts->dt3 + pts->vt3 * (pts->t4 - pts->t3);

		pts->t5 = pts->t4 + pts->t1;
		pts->dt5 = pts->dt4 + pts->dt3 - pts->dt2;

		pts->t6 = pts->t4 + pts->t2;
		pts->dt6 = pts->dt5 + pts->dt2 - pts->dt1;

		pts->tend = pts->t6 + pts->t1;
		pts->dtend = pts->dt6 + pts->dt1;
		pts->invd = reciprocate(pts->dtend);

		return GO_RESULT_OK;
	}

	/**!
	Computes the trajectory defining times \a t1, \a t2 and \a t4
	for a motion with no acceleration or cruise phase, given \a pts
	filled in with \a dtend and \a jt0.

	A subsequent call to \a go_traj_cj_compute_the_rest will fill in the
	remaining parameters which will be used repeatedly by \a
	go_traj_cj_interp.
	*/
	inline go_result go_traj_cj_compute_tees_na_nc(go_traj_cj_spec * pts)
	{
		pts->t1 = go_cbrt(1./2. * pts->dtend / pts->jt0);
		pts->t2 = pts->t1;
		pts->t4 = pts->t1 + pts->t2;

		return GO_RESULT_OK;
	}

	/**!
	Computes the trajectory defining times \a t1, \a t2 and \a t4
	for a motion with acceleration but no cruise phase, given \a pts
	filled in with \a dtend, \a jt0 and \a at1.

	A subsequent call to \a go_traj_cj_compute_the_rest will fill in the
	remaining parameters which will be used repeatedly by \a
	go_traj_cj_interp.
	*/
	inline go_result go_traj_cj_compute_tees_a_nc(go_traj_cj_spec * pts)
	{
		go_real a_j;

		a_j = pts->at1 / pts->jt0;
		pts->t1 = a_j;
		pts->t2 = 1./2. * (sqrt(go_sq(a_j) + 4. * pts->dtend / pts->at1) - a_j);
		pts->t4 = pts->t1 + pts->t2;

		return GO_RESULT_OK;
	}

	/**!
	Computes the trajectory defining times \a t1, \a t2 and \a t4
	for a motion with no acceleration but a cruise phase, given \a pts
	filled in with \a dtend, \a jt0 and \a vt3.

	A subsequent call to \a go_traj_cj_compute_the_rest will fill in the
	remaining parameters which will be used repeatedly by \a
	go_traj_cj_interp.
	*/
	inline go_result go_traj_cj_compute_tees_na_c(go_traj_cj_spec * pts)
	{
		pts->t1 = sqrt(pts->vt3 / pts->jt0);
		pts->t2 = pts->t1;
		pts->t4 = pts->dtend / pts->vt3;

		return GO_RESULT_OK;
	}

	/**!
	Computes the trajectory defining times \a t1, \a t2 and \a t4
	for a motion with no acceleration but a cruise phase, given \a pts
	filled in with \a dtend, \a jt0, at1 and \a vt3.

	A subsequent call to \a go_traj_cj_compute_the_rest will fill in the
	remaining parameters which will be used repeatedly by \a
	go_traj_cj_interp.
	*/
	inline go_result go_traj_cj_compute_tees_a_c(go_traj_cj_spec * pts)
	{
		pts->t1 = pts->at1 / pts->jt0;
		pts->t2 = pts->vt3 / pts->at1;
		pts->t4 = pts->dtend / pts->vt3;

		return GO_RESULT_OK;
	}

	/**!
	Given a total move distance \a d, maximum peak speed \a v, maximum
	peak acceleration \a a and jerk \a j, compute the times for each of
	the seven phases (jerk, accel, dejerk, cruise, dejerk, decel and
	jerk), and the associated cumulative distances, instantaneous speeds
	and accelerations. The resulting structure \a pts will be used by
	the constant-jerk interpolater \a go_traj_cj_interp repeatedly when
	evaluating the motion along this trajectory.
	*/
	inline go_result go_traj_cj_compute(go_real d, go_real v, go_real a, go_real j,
		go_traj_cj_spec * pts)
	{
		go_result retval;

		if (d < 0. || v <= 0. || a <= 0. || j <= 0.) return GO_RESULT_ERROR;

		/** fill the targets in, and they will be recomputed if needed */
		pts->dtend = d;
		pts->vt3 = v;
		pts->at1 = a;
		pts->jt0 = j;

		if (d * go_sq(j) > 2. * go_cub(a) &&
			v * j > go_sq(a)) {
				/** accel phase */
				if (d * a * j > go_sq(v) * j + v * go_sq(a)) {
					/** cruise phase */
					retval = go_traj_cj_compute_tees_a_c(pts);
				} else {
					/** no cruise phase */
					retval = go_traj_cj_compute_tees_a_nc(pts);
				}
		} else {
			/** no accel phase */
			if (go_sq(d) * j > 4. * go_cub(v)) {
				/** cruise phase */
				retval = go_traj_cj_compute_tees_na_c(pts);
			} else {
				/** no cruise phase */
				retval = go_traj_cj_compute_tees_na_nc(pts);
			}
		}

		if (GO_RESULT_OK != retval) return retval;
		return go_traj_cj_compute_the_rest(pts);
	}

	inline go_result go_traj_cj_scale(const go_traj_cj_spec * ts, go_real t,
		go_traj_cj_spec * pts)
	{
		go_traj_cj_spec ots;

		if (ts->tend <= 0. || t <= 0.) {
			ots.t1 = 0.;
			ots.t2 = 0.;
			ots.t3 = 0.;
			ots.t4 = t;
			ots.t5 = t;
			ots.t6 = t;
			ots.tend = t;

			ots.dt1 = ts->dt1;
			ots.dt2 = ts->dt2;
			ots.dt3 = ts->dt3;
			ots.dt4 = ts->dt4;
			ots.dt5 = ts->dt5;
			ots.dt6 = ts->dt6;
			ots.dtend = ts->dtend;

			ots.vt1 = 0.;
			ots.vt2 = 0.;
			ots.vt3 = 0.;
			ots.at1 = 0.;
			ots.jt0 = 0.;

			ots.invd = ts->invd;
		} else if (t <= ts->tend) {
			/** can't shrink shorter */
			ots = *ts;
		} else {
			ots.t1 = ts->t1;
			ots.t2 = ts->t2;
			ots.t3 = ts->t3;
			ots.t4 = t - ts->t3;
			ots.t5 = t - ts->t2;
			ots.t6 = t - ts->t1;
			ots.tend = t;

			ots.vt3 = ts->dtend / ots.t4;
			ots.jt0 = ots.vt3 / (ots.t1 * ots.t2);
			ots.at1 = ots.jt0 * ots.t1;
			ots.vt1 = 0.5 * ots.jt0 * go_sq(ots.t1);
			ots.vt2 = ots.vt3 - ots.vt1;

			ots.dt1 = 1./6. * ots.jt0 * go_cub(ots.t1);
			ots.dt3 = 0.5 * ots.vt3 * ots.t3;
			ots.dt2 = ots.dt1 + 0.5 * ots.vt3 * (ots.t2 - ots.t1);

			ots.dtend = ts->dtend;
			ots.dt4 = ots.dtend - ots.dt3;
			ots.dt5 = ots.dtend - ots.dt2;
			ots.dt6 = ots.dtend - ots.dt1;
			ots.invd = ts->invd;
		}

		*pts = ots;

		return GO_RESULT_OK;
	}

	inline go_result go_traj_cj_stop(const go_traj_cj_spec * ts, go_real t,
		go_traj_cj_spec * pts)
	{
		go_traj_cj_spec ots;
		go_real id2;			/** incremental phase II distance */
		go_real id3;			/** incremental phase III distance */
		go_real it4;			/** incremental phase IV time */
		go_real id4;			/** incremental phase IV distance */

		if (t <= 0.) {
			ots.jt0 = ts->jt0;
			ots.t1 = 0.;
			ots.dt1 = 0.;
			ots.vt1 = ts->vt1;
			ots.at1 = ts->at1;
			ots.t2 = 0.;
			ots.dt2 = 0.;
			ots.vt2 = ts->vt2;
			ots.t3 = 0.;
			ots.dt3 = 0.;
			ots.vt3 = ts->vt3;
			ots.t4 = 0.;
			ots.dt4 = 0.;
			ots.t5 = 0.;
			ots.dt5 = 0.;
			ots.t6 = 0.;
			ots.dt6 = 0.;
			ots.tend = 0.;
			ots.dtend = 0.;
			ots.invd = GO_INF;
		} else if (t < ts->t1) {
			/** stopping in stage I */
			/** adjust stage I */
			ots.jt0 = ts->jt0;
			ots.t1 = t;
			ots.dt1 = 1. / 6. * ts->jt0 * go_cub(t);
			ots.vt1 = 1. / 2. * ts->jt0 * go_sq(t);
			ots.at1 = ts->jt0 * t;
			/** null stage II, same as I */
			ots.t2 = ots.t1;
			ots.dt2 = ots.dt1;
			ots.vt2 = ots.vt1;
			/** stage III */
			ots.t3 = t + t;
			ots.vt3 = ots.vt1 + ots.vt1;
			ots.dt3 = t * ots.vt3;
			/** null stage IV, same as III */
			ots.t4 = ots.t3;
			ots.dt4 = ots.dt3;
			/** stage V */
			ots.t5 = ots.t3 + t;
			ots.dtend = ots.dt3 + ots.dt3; /** do earlier */
			ots.dt5 = ots.dtend - ots.dt1; /** so we can use it here */
			/** null stage VI, same as V */
			ots.t6 = ots.t5;
			ots.dt6 = ots.dt5;
			/** stage VII */
			ots.tend = ots.t5 + t;
			ots.invd = reciprocate(ots.dtend);
		} else if (t < ts->t2) {
			/** stopping in stage II */
			/** stage I unchanged */
			ots.jt0 = ts->jt0;
			ots.t1 = ts->t1;
			ots.dt1 = ts->dt1;
			ots.vt1 = ts->vt1;
			ots.at1 = ts->at1;
			/** adjust stage II */
			ots.t2 = t;
			ots.vt2 = ts->vt1 + ts->at1 * (t - ts->t1);
			id2 = (t - ts->t1) * 0.5 * (ots.vt2 + ts->vt1);
			ots.dt2 = ts->dt1 + id2;
			/** stage III */
			ots.t3 = t + ts->t1;
			ots.vt3 = ots.vt2 + ts->vt1;
			id3 = ts->t1 * ots.vt3 - ts->dt1;
			ots.dt3 = ots.dt2 + id3;
			/** null stage IV, same as III */
			ots.t4 = ots.t3;
			ots.dt4 = ots.dt3;
			/** stage V */
			ots.t5 = ots.t3 + ts->t1;
			ots.dt5 = ots.dt3 + id3; 
			/** stage VI */
			ots.t6 = ots.t5 + (t - ts->t1);
			ots.dt6 = ots.dt5 + id2;
			/** stage VII */
			ots.tend = ots.t6 + ts->t1;
			ots.dtend = ots.dt6 + ts->dt1;
			ots.invd = reciprocate(ots.dtend);
		} else if (t < ts->t3) {
			/** stopping in stage III */
			/** stages I, II and III unchanged */
			ots.jt0 = ts->jt0;
			ots.t1 = ts->t1;
			ots.dt1 = ts->dt1;
			ots.vt1 = ts->vt1;
			ots.at1 = ts->at1;
			ots.t2 = ts->t2;
			ots.dt2 = ts->dt2;
			ots.vt2 = ts->vt2;
			ots.t3 = ts->t3;
			ots.dt3 = ts->dt3;
			ots.vt3 = ts->vt3;
			/** null stage IV */
			ots.t4 = ts->t3;
			ots.dt4 = ts->dt3;
			/** take off original stage IV for remaining stages */
			it4 = ts->t4 - ts->t3;
			id4 = ts->vt3 * it4;
			ots.t5 = ts->t5 - it4;
			ots.dt5 = ts->dt5 - id4;
			ots.t6 = ts->t6 - it4;
			ots.dt6 = ts->dt6 - id4;
			ots.tend = ts->tend - it4;
			ots.dtend = ts->dtend - id4;
			ots.invd = reciprocate(ots.dtend);
		} else if (t < ts->t4) {
			/** stopping in stage IV */
			/** stages I, II and III unchanged */
			ots.jt0 = ts->jt0;
			ots.t1 = ts->t1;
			ots.dt1 = ts->dt1;
			ots.vt1 = ts->vt1;
			ots.at1 = ts->at1;
			ots.t2 = ts->t2;
			ots.dt2 = ts->dt2;
			ots.vt2 = ts->vt2;
			ots.t3 = ts->t3;
			ots.dt3 = ts->dt3;
			ots.vt3 = ts->vt3;
			/** adjusted stage IV */
			it4 = ts->t4 - t;
			id4 = ts->vt3 * it4;
			ots.t4 = t;
			ots.dt4 = ts->dt4 - id4;
			ots.t5 = ts->t5 - it4;
			ots.dt5 = ts->dt5 - id4;
			ots.t6 = ts->t6 - it4;
			ots.dt6 = ts->dt6 - id4;
			ots.tend = ts->tend - it4;
			ots.dtend = ts->dtend - id4;
			ots.invd = reciprocate(ots.dtend);
		} else {
			/** stopping in stage V, VI or VII-- leave alone */
			ots = *ts;
		}

		*pts = ots;

		return GO_RESULT_OK;
	}

	inline go_result go_traj_cj_extend(const go_traj_cj_spec * ts, go_real t,
		go_traj_cj_spec * pts)
	{
		go_real tincr;
		go_real dincr;

		*pts = *ts;

		if (t < ts->tend) {
			return GO_RESULT_OK;
		}

		tincr = t - ts->tend;
		dincr = ts->vt3 * tincr;

		pts->t4 += tincr;
		pts->dt4 += dincr;
		pts->t5 += tincr;
		pts->dt5 += dincr;
		pts->t6 += tincr;
		pts->dt6 += dincr;
		pts->tend = t;
		pts->dtend += dincr;
		pts->invd = reciprocate(pts->dtend);
		return GO_RESULT_OK;
	}

	/**
	run interpolation on each phase for each motion parameter, using

	d = d0 + v0 t + 1/2 a0 t^2 + 1/6 j0 t^3
	v =      v0   +     a0 t   + 1/2 j0 t^2
	a =                 a0     +     j0 t
	j =                              j0

	where d0, v0, a0, j0 are the initial values for each phase
	*/

	inline go_result go_traj_cj_interp(const go_traj_cj_spec * ts, go_real t,
		go_traj_interp_spec * ti)
	{
		go_real j, a, v, d;
		go_real deltat;

		if (t < 0.) {
			j = 0.;
			a = 0.;
			v = 0.;
			d = 0.;
			t = 0.;
		} else if (t < ts->t1) {
			j = ts->jt0;
			a = ts->jt0 * t;
			v = (1. / 2.) * ts->jt0 * go_sq(t);
			d = (1. / 6.) * ts->jt0 * go_cub(t);
		} else if (t < ts->t2) {
			deltat = t - ts->t1;
			j = 0.;
			a = ts->at1;
			v = ts->vt1 + ts->at1 * deltat;
			d = ts->dt1 + ts->vt1 * deltat + (1. / 2.) * ts->at1 * go_sq(deltat);
		} else if (t < ts->t3) {
			deltat = t - ts->t2;
			j = -ts->jt0;
			a = ts->at1 - ts->jt0 * deltat;
			v = ts->vt2 + ts->at1 * deltat - (1. / 2.) * ts->jt0 * go_sq(deltat);
			d =
				ts->dt2 + ts->vt2 * deltat + (1. / 2.) * ts->at1 * go_sq(deltat) -
				(1.0 / 6.0) * ts->jt0 * go_cub(deltat);
		} else if (t < ts->t4) {
			deltat = t - ts->t3;
			j = 0.;
			a = 0.;
			v = ts->vt3;
			d = ts->dt3 + ts->vt3 * deltat;
		} else if (t < ts->t5) {
			deltat = t - ts->t4;
			j = -ts->jt0;
			a = -ts->jt0 * deltat;
			v = ts->vt3 - (1. / 2.) * ts->jt0 * go_sq(deltat);
			d = ts->dt4 + ts->vt3 * deltat - (1. / 6.) * ts->jt0 * go_cub(deltat);
		} else if (t < ts->t6) {
			deltat = t - ts->t5;
			j = 0.;
			a = -ts->at1;
			v = ts->vt2 - ts->at1 * deltat;
			d = ts->dt5 + ts->vt2 * deltat - (1. / 2.) * ts->at1 * go_sq(deltat);
		} else if (t < ts->tend) {
			deltat = t - ts->t6;
			j = ts->jt0;
			a = -ts->at1 + ts->jt0 * deltat;
			v = ts->vt1 - ts->at1 * deltat + (1. / 2.) * ts->jt0 * go_sq(deltat);
			d =
				ts->dt6 + ts->vt1 * deltat - (1. / 2.) * ts->at1 * go_sq(deltat) +
				(1. / 6.) * ts->jt0 * go_cub(deltat);
		} else {
			j = 0.;
			a = 0.;
			v = 0.;
			d = ts->dtend;
			t = ts->tend;
		}

		ti->j = j;
		ti->a = a;
		ti->v = v;
		ti->d = d;

		if (ts->invd == GO_INF) {
			ti->s = 1.;
		} else {
			ti->s = d * ts->invd;
		}

		return GO_RESULT_OK;
	}

};
#endif /* GOTRAJ_H */
