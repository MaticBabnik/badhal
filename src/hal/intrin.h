/*
ARM pseudo-intrinsics
*/

static inline void a_dmb() { asm volatile("dmb"); }
static inline void a_isb() { asm volatile("isb"); }
static inline void a_dsb() { asm volatile("dsb"); }