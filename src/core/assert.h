/*
    Runtime assertions
*/

#pragma once

#ifdef RTASSERT

extern void sys_trap(const char *msg);

#define ASSERT(expr, msg)                                                      \
    do {                                                                       \
        if (!expr) sys_trap("assertion fail: " msg);                           \
    } while (0)

#define ASSERT_UNREACHABLE(msg) sys_trap("unreachable: " msg)

#define ASSERT_RANGE(val, min, max, msg)                                       \
    do {                                                                       \
        if (val < min || val > max) sys_trap("out of range: " msg);            \
    } while (0)

#else

#define ASSERT(expr, msg)                                                      \
    do { /*assertions disabled*/                                               \
    } while (0)
#define ASSERT_UNREACHABLE(msg)                                                \
    do { /*assertions disabled*/                                               \
    } while (0)
#define ASSERT_RANGE(val, min, max, msg)                                       \
    do { /*assertions disabled*/                                               \
    } while (0)

#endif