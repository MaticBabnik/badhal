#pragma once

#include "../bad.h"

struct SysTick_t
{
    R_RW u32 CTRL;
    R_RW u32 LOAD;
    R_RW u32 VAL;
    R_WO u32 CALIB;
};


#define SysTick_CTRL_CLKSOURCE (1UL << 2)
#define SysTick_CTRL_TICKINT (1 << 1)
#define SysTick_CTRL_ENABLE 1
