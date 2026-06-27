#pragma once
#include "numeric.h"

#define K_2PI 6.28318530f
#define K_PI2 1.57079632f

#define FMIN2(a, b) ((a) < (b) ? (a) : (b))
#define FMAX2(a, b) ((a) > (b) ? (a) : (b))
#define FMIN3(a, b, c) FMIN2(a, FMIN2(b, c))
#define FMAX3(a, b, c) FMAX2(a, FMAX2(b, c))
#define CLAMPI(v, lo, hi) ((v) < (lo) ? (lo) : ((v) > (hi) ? (hi) : (v)))

void sincos_f(f32 x, f32 *s, f32 *c);
