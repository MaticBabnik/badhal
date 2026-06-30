#pragma once
#include <core/numeric.h>

typedef i32 ptrdiff_t;
typedef typeof(nullptr) nullptr_t;
typedef i64 max_align_t;
typedef u32 size_t;

#ifndef NULL
#define NULL ((nullptr_t) 0)
#endif

#ifndef offsetof
#define offsetof(type, member) ((size_t) &(((type *) 0)->member))
#endif