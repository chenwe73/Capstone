#ifndef __PRECISION_H_INCLUDED__
#define __PRECISION_H_INCLUDED__


#include <math.h>

typedef float real;

const real PI = (real)3.14159265358;

#define REAL_MAX FLT_MAX

#define real_sqrt sqrtf
#define real_pow powf
#define real_abs fabsf
#define real_sin sinf
#define real_cos cosf
#define real_exp expf
#define real_fmax fmaxf
#define real_fmin fminf



#endif // __PRECISION_H_INCLUDED__