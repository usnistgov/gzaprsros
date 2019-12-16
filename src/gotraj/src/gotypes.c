/*
  gotypes.c

  Global variables recording the compile-time values for go_result, etc.
  in the Go library
*/

#include "gotraj/gotypes.h"

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

