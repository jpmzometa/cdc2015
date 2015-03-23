#ifndef MC04TYPES_H
#define MC04TYPES_H
#include <stdint.h>
/* MISRA C 2004 compliant numeric typedef */

typedef char char_t;

/* Probably already defined by inttypes.h or types.h*/
# ifndef __int8_t_defined
#  define __int8_t_defined
#if 0
typedef signed char int8_t;
typedef signed short int16_t;
typedef signed int int32_t;
typedef unsigned int uint32_t;
typedef unsigned char uint8_t;
typedef unsigned short uint16_t;
#endif
#endif
typedef float float32_t;
typedef double float64_t;
typedef long double float128_t;

#endif
