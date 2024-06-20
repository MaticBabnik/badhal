/*
ARM pseudo-intrinsics
*/
#include "bad.h"

B_INLINE void a_dmb() { asm volatile("dmb"); }
B_INLINE void a_isb() { asm volatile("isb"); }
B_INLINE void a_dsb() { asm volatile("dsb"); }