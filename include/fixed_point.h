#pragma once

#include <stdint.h>

// == bit fixed point Q2.14 format ===========
// resolution 2^-14 =  6.1035e-5
// dynamic range is +1.9999/-2.0
typedef int16_t s1x14;
#define muls1x14(a,b) ((s1x14)((((int32_t)(a))*((int32_t)(b)))>>14)) //multiply two fixed s1x14
#define float_to_s1x14(a) ((s1x14)((a)*16384.0)) // 2^14
#define s1x14_to_float(a) ((float)(a)/16384.0)
#define absfix14(a) abs(a)

#define s1x14_FLOAT_MIN -2.0f
#define s1x14_FLOAT_MAX 1.99993896484f

#define s1x14_MIN INT16_MIN
#define s1x14_MAX INT16_MAX


