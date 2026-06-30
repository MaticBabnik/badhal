/*
    Core numeric types and utilities
*/

#pragma once
#include "compiler.h"

typedef signed char i8;
typedef unsigned char u8;
typedef short i16;
typedef unsigned short u16;
typedef int i32;
typedef unsigned int u32;
typedef long long i64;
typedef unsigned long long u64;
typedef float f32;
typedef double f64;

#define KILO (1ul << 10)
#define MEGA (1ul << 20)

#define I8_MIN ((i8) 0x80)
#define I8_MAX ((i8) 0x7f)
#define U8_MIN ((u8) 0x00)
#define U8_MAX ((u8) 0xff)
#define I16_MIN ((i16) 0x8000)
#define I16_MAX ((i16) 0x7fff)
#define U16_MIN ((u16) 0x0000)
#define U16_MAX ((u16) 0xffff)
#define I32_MIN ((i32) 0x80000000)
#define I32_MAX ((i32) 0x7fffffff)
#define U32_MIN ((u32) 0x00000000)
#define U32_MAX ((u32) 0xffffffff)

#define DECLARE_MIN_MAX(type)                                                  \
    INLINE_ALWAYS type type##_min(type a, type b) {                            \
        return a < b ? a : b;                                                  \
    }                                                                          \
    INLINE_ALWAYS type type##_max(type a, type b) {                            \
        return a > b ? a : b;                                                  \
    }

DECLARE_MIN_MAX(i32)
DECLARE_MIN_MAX(u32)
DECLARE_MIN_MAX(i64)
DECLARE_MIN_MAX(u64)
DECLARE_MIN_MAX(f32)
DECLARE_MIN_MAX(f64)

#undef DECLARE_MIN_MAX

#define min(a, b)                                                              \
    _Generic((a + b), i32: i32_min, u32: u32_min, f32: f32_min)(a, b)

#define max(a, b)                                                              \
    _Generic((a + b), i32: i32_max, u32: u32_max, f32: f32_max)(a, b)

INLINE_ALWAYS u32 popcnt(u32 x) {
    u32 c = 0;
    while (x) {
        x &= x - 1;
        c++;
    }
    return c;
}