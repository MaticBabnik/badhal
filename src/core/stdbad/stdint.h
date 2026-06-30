#pragma once
#include <core/numeric.h>

typedef i8 int8_t;
typedef i16 int16_t;
typedef i32 int32_t;
typedef i64 int64_t;

typedef u8 uint8_t;
typedef u16 uint16_t;
typedef u32 uint32_t;
typedef u64 uint64_t;

typedef i32 int_fast8_t;
typedef i32 int_fast16_t;
typedef i32 int_fast32_t;
typedef i64 int_fast64_t;

typedef u32 uint_fast8_t;
typedef u32 uint_fast16_t;
typedef u32 uint_fast32_t;
typedef u64 uint_fast64_t;

typedef i8 int_least8_t;
typedef i16 int_least16_t;
typedef i32 int_least32_t;
typedef i64 int_least64_t;

typedef u8 uint_least8_t;
typedef u16 uint_least16_t;
typedef u32 uint_least32_t;
typedef u64 uint_least64_t;

typedef i32 intmax_t;
typedef u64 uintmax_t;

typedef i32 intptr_t;
typedef u32 uintptr_t;

// widths

#define INT8_WIDTH 8
#define INT16_WIDTH 16
#define INT32_WIDTH 32
#define INT64_WIDTH 64

#define UINT8_WIDTH 8
#define UINT16_WIDTH 16
#define UINT32_WIDTH 32
#define UINT64_WIDTH 64

#define INT_LEAST8_WIDTH 8
#define INT_LEAST16_WIDTH 16
#define INT_LEAST32_WIDTH 32
#define INT_LEAST64_WIDTH 64

#define UINT_LEAST8_WIDTH 8
#define UINT_LEAST16_WIDTH 16
#define UINT_LEAST32_WIDTH 32
#define UINT_LEAST64_WIDTH 64

#define INT_FAST8_WIDTH 32
#define INT_FAST16_WIDTH 32
#define INT_FAST32_WIDTH 32
#define INT_FAST64_WIDTH 64

#define UINT_FAST8_WIDTH 32
#define UINT_FAST16_WIDTH 32
#define UINT_FAST32_WIDTH 32
#define UINT_FAST64_WIDTH 64

#define INTPTR_WIDTH 32
#define UINTPTR_WIDTH 32

// min

#define INT8_MIN (-128)
#define INT16_MIN (-32768)
#define INT32_MIN (-2147483648l)
#define INT64_MIN (-9223372036854775808ll)

#define UINT8_MIN 0
#define UINT16_MIN 0
#define UINT32_MIN 0
#define UINT64_MIN 0

#define INT_LEAST8_MIN (-128)
#define INT_LEAST16_MIN (-32768)
#define INT_LEAST32_MIN (-2147483648l)
#define INT_LEAST64_MIN (-9223372036854775808ll)

#define UINT_LEAST8_MIN 0
#define UINT_LEAST16_MIN 0
#define UINT_LEAST32_MIN 0
#define UINT_LEAST64_MIN 0

#define INT_FAST8_MIN (-128)
#define INT_FAST16_MIN (-32768)
#define INT_FAST32_MIN (-2147483648l)
#define INT_FAST64_MIN (-9223372036854775808ll)

#define UINT_FAST8_MIN 0
#define UINT_FAST16_MIN 0
#define UINT_FAST32_MIN 0
#define UINT_FAST64_MIN 0

#define INTPTR_MIN (-2147483648l)
#define UINTPTR_MIN 0

// max

#define INT8_MAX 127
#define INT16_MAX 32767
#define INT32_MAX 2147483647l
#define INT64_MAX 9223372036854775807ll

#define UINT8_MAX 255
#define UINT16_MAX 65535
#define UINT32_MAX 4294967295l
#define UINT64_MAX 18446744073709551615ll

#define INT_LEAST8_MAX 127
#define INT_LEAST16_MAX 32767
#define INT_LEAST32_MAX 2147483647l
#define INT_LEAST64_MAX 9223372036854775807ll

#define UINT_LEAST8_MAX 255
#define UINT_LEAST16_MAX 65535
#define UINT_LEAST32_MAX 4294967295l
#define UINT_LEAST64_MAX 18446744073709551615ll

#define INT_FAST8_MAX 127
#define INT_FAST16_MAX 32767
#define INT_FAST32_MAX 2147483647l
#define INT_FAST64_MAX 9223372036854775807ll

#define UINT_FAST8_MAX 255
#define UINT_FAST16_MAX 65535
#define UINT_FAST32_MAX 4294967295l
#define UINT_FAST64_MAX 18446744073709551615ll

#define INTPTR_MAX (-2147483648l)
#define UINTPTR_MAX 0

#define PTRDIFF_WIDTH 32
#define PTRDIFF_MIN (-2147483648l)
#define PTRDIFF_MAX 2147483647l

#define SIG_ATOMIC_WIDTH 32
#define SIG_ATOMIC_MIN (-2147483648l)
#define SIG_ATOMIC_MAX 2147483647l

#define SIZE_WIDTH 32
#define SIZE_MIN 0
#define SIZE_MAX 4294967295l

#define INT8_C(x) x
#define INT16_C(x) x
#define INT32_C(x) x##l
#define INT64_C(x) x##ll
#define INTMAX_C(x) x##ll

#define UINT8_C(x) x
#define UINT16_C(x) x
#define UINT32_C(x) x##ul
#define UINT64_C(x) x##ull
#define UINTMAX_C(x) x##ull

