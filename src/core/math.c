#include "math.h"

// claude wrote this; i will throw it out at some point
void sincos_f(f32 x, f32 *s, f32 *c) {
    /* reduce to [0, 2π) */
    i32 k = (i32) (x / K_2PI);
    x -= (f32) k * K_2PI;
    if (x < 0.0f) x += K_2PI;

    /* quadrant + remainder in [0, π/2) */
    i32 q = (i32) (x / K_PI2);
    if (q > 3) q = 3;
    f32 r = x - (f32) q * K_PI2;
    f32 r2 = r * r;

    /* sin: 7th-order Taylor     cos: 6th-order Taylor */
    f32 sv = r * (1.0f - r2 / 6.0f * (1.0f - r2 / 20.0f * (1.0f - r2 / 42.0f)));
    f32 cv = 1.0f - r2 / 2.0f * (1.0f - r2 / 12.0f * (1.0f - r2 / 30.0f));

    switch (q) {
    case 0:
        *s = sv;
        *c = cv;
        break;
    case 1:
        *s = cv;
        *c = -sv;
        break;
    case 2:
        *s = -sv;
        *c = -cv;
        break;
    default:
        *s = -cv;
        *c = sv;
        break;
    }
}