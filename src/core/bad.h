/*
    HAL core types & utils
*/

#pragma once
#include "numeric.h"
#include "compiler.h"
#include "assert.h"
#include "intrin.h"
#include "stdbad/stddef.h"

#if !defined(__GNUC__) && !defined(__clang__)
#error "Only GCC and Clang compilers are supported"
#endif

INLINE_ALWAYS void mreg(volatile u32 *reg, u32 mask, u32 value) {
    *reg = (*reg & ~mask) | (value & mask);
}

void memset8(void *ptr, u8 value, u32 n);
void memset16(void *ptr, u16 value, u32 n);
void memset32(void *ptr, u32 value, u32 n);