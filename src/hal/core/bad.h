/*
    HAL core types & utils
*/

#pragma once
#include <stddef.h> // NULL, size_t, ...
#include "numeric.h"
#include "compiler.h"
#include "assert.h"
#include "intrin.h"

#if !defined(__GNUC__) && !defined(__clang__)
    #error "Only GCC and Clang compilers are supported"
#endif
