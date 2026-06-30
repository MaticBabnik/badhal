#pragma once

#ifdef NDEBUG
#define assert(cond) ((void) 0)
#else
void sys_trap(const char *msg);
#define assert(cond)                                                           \
    do {                                                                       \
        if (!(cond)) sys_trap("assertion fail: " #cond);                       \
    } while (0)
#endif