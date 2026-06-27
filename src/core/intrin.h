/*
    ARM pseudo-intrinsics
*/

#pragma once
#include "compiler.h"

INLINE_ALWAYS void a_dmb() {
    asm volatile("dmb sy" ::: "memory");
}

INLINE_ALWAYS void a_isb() {
    asm volatile("isb" ::: "memory");
}

INLINE_ALWAYS void a_dsb() {
    asm volatile("dsb sy" ::: "memory");
}

INLINE_ALWAYS void a_nop() {
    asm volatile("nop");
}

INLINE_ALWAYS void a_wfi() {
    asm volatile("wfi");
}

INLINE_ALWAYS void a_wfe() {
    asm volatile("wfe");
}

INLINE_ALWAYS u32 a_clz(u32 a) {
    u32 r;
    asm("clz %0, %1" : "=r"(r) : "r"(a)); // this one isn't volatile
    return r;
}

INLINE_ALWAYS u32 aa_ffs(u32 a) {
    u32 r;
    asm("rbit %0, %1\n"
        "clz  %0, %0"
        : "=r"(r)
        : "r"(a));
    return a ? r + 1 : 0;
}