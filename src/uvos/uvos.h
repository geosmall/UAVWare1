#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#ifndef NULL
#define NULL  0U
#endif

#ifndef FALSE
#define FALSE 0U
#endif

#ifndef TRUE
#define TRUE  1U
#endif

#define limit( a, b ) \
	( { __typeof__ ( a ) _a = ( a ); \
      __typeof__ ( b ) _b = ( b ); \
      _a > _b ? _b : _a; } )