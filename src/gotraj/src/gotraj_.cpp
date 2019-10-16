// gotraj.cpp : Defines the entry point for the console application.
//

#include "gotraj/gotypes.h"
#include "gotraj/gointerp.h"
#include "gotraj/gotraj.h"
#include "gotraj/gomotion.h"

#if defined(GO_RESULT_CHAR)
int go_result_char = 1;

#elif defined(GO_RESULT_SHORT)
int go_result_short = 1;

#else
int go_result_int = 1;
#endif

#if defined(GO_REAL_FLOAT)
int go_real_float = 1;

#elif defined(GO_REAL_LONG_DOUBLE)
int go_real_long_double = 1;

#else
int go_real_double = 1;
#endif

#if defined(GO_INTEGER_SHORT)
int go_integer_short = 1;

#elif defined(GO_INTEGER_LONG)
int go_integer_long = 1;

#elif defined(GO_INTEGER_LONG_LONG)
int go_integer_long_long = 1;

#else
int go_integer_int = 1;
#endif

#if defined(GO_FLAG_USHORT)
int go_flag_ushort = 1;

#elif defined(GO_FLAG_UINT)
int go_flag_uint = 1;

#else
int go_flag_uchar = 1;
#endif

/* the global ad-hoc flag for tracing code */
go_flag gocode = 0;

int go_init() {return 0;}
int go_exit() {return 0;}

#if 0
using namespace gomotion;
int main(int argc, char * argv[])
{
//	enum { QUEUE_SIZE = 1 };	/* < 3 tests for good length handling */
	enum { QUEUE_SIZE = 4 };	/* < 3 tests for good length handling */
	go_real time, deltat = 0.01;
	go_motion_spec gms, space[QUEUE_SIZE];
	go_motion_queue gmq;
	go_position position;
	go_integer i;

	if (0 != go_init() ||
		GO_RESULT_OK != gmq.init(space, QUEUE_SIZE, deltat)) {
			return 1;
	}

	time = 0.0;

		/* no arg means do world motion */
		if (GO_RESULT_OK != gmq.set_type(GO_MOTION_WORLD)) {
			return 1;
		}
		gms.init();
		gms.set_type(GO_MOTION_LINEAR);
		gms.set_id(1);
		position.u.pose = go_pose_this(0,0,0,1,0,0,0);
		gmq.set_here(&position); // jlm added otherwise no starting position
		
		gms.set_end_position(&position);
		gms.set_tpar(1.,1.,1.);
		gms.set_rpar(1.,1.,1.);

		position.u.pose.tran.x = 1.0;
		gms.set_end_position(&position);
		if (GO_RESULT_OK != gmq.append(&gms)) {
			fprintf(stderr, "can't append\n");
		}

		position.u.pose.tran.y = 2.0;
		gms.set_end_position(&position);
		if (GO_RESULT_OK != gmq.append(&gms)) {
			fprintf(stderr, "can't append\n");
		}

		position.u.pose.tran.x = 3.0;
		gms.set_end_position(&position);
		if (GO_RESULT_OK != gmq.append(&gms)) {
			fprintf(stderr, "can't append\n");
		}

		while (!gmq.is_empty()) {
			gmq.interp(&position);
			printf("T=%f X=%f Y=%f\n", (double) time, (double) position.u.pose.tran.x, (double) position.u.pose.tran.y);
			time += deltat;
		}

		position.u.pose.tran.x = -1.0;
		position.u.pose.tran.y = -1.0;
		gms.set_end_position(&position);
		if (GO_RESULT_OK != gmq.append(&gms)) {
			fprintf(stderr, "can't append\n");
		}

		while (!gmq.is_empty()) {
			gmq.interp(&position);
			printf("T=%f X=%f Y=%f\n", (double) time, (double) position.u.pose.tran.x, (double) position.u.pose.tran.y);
			time += deltat;
		}
		///////////////////////////////////////////////////////////////////////////
		/* now do joint motion */
		if (GO_RESULT_OK != gmq.set_type(GO_MOTION_JOINT)) {
			return 1;
		}

		gmq.set_joint_number(7);

		gms.init();
		gms.set_type(GO_MOTION_JOINT);
		gms.set_id(1);
		for (i = 0; i < GO_MOTION_JOINT_NUM; i++) {
			position.u.joint[i] = 0.0;
			gms.set_end_position(&position);
			gms.set_jpar(i, 1,1,1);
		}

		position.u.joint[0] = 1.0;
		gms.set_end_position(&position);
		if (GO_RESULT_OK != gmq.append(&gms)) {
			fprintf(stderr, "can't append\n");
		}

		position.u.joint[1] = 2.0;
		gms.set_end_position(&position);
		if (GO_RESULT_OK != gmq.append(&gms)) {
			fprintf(stderr, "can't append\n");
		}

		position.u.joint[6] = 3.0;
		gms.set_end_position(&position);
		if (GO_RESULT_OK != gmq.append(&gms)) {
			fprintf(stderr, "can't append\n");
		}

		while (!gmq.is_empty()) {
			gmq.interp(&position);
			printf("%f %f %f %f\n", (double) time, (double) position.u.joint[0], (double) position.u.joint[1], (double) position.u.joint[6]);
			time += deltat;
		}

		position.u.joint[0] = -1.0;
		position.u.joint[1] = -1.0;
		gms.set_end_position(&position);
		if (GO_RESULT_OK != gmq.append(&gms)) {
			fprintf(stderr, "can't append\n");
		}

		while (!gmq.is_empty()) {
			gmq.interp(&position);
			printf("%f %f %f %f\n", (double) time, (double) position.u.joint[0], (double) position.u.joint[1], (double) position.u.joint[2]);
			time += deltat;
		}
	return 0;
}
#endif
