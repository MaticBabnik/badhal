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