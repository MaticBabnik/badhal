#pragma once

#include "../bad.h"

struct MPU_t
{
    R_RO u32 TYPE;
    R_RW u32 CTRL;
    R_RW u32 RNR;
    R_RW u32 RBAR;
    R_RW u32 RASR;
    R_RW u32 RBAR_A1;
    R_RW u32 RASR_A1;
    R_RW u32 RBAR_A2;
    R_RW u32 RASR_A2;
    R_RW u32 RBAR_A3;
    R_RW u32 RASR_A3;
};

#define MPU_CTRL_ENABLE 1
#define MPU_PRIVILEGED_DEFAULT 0x4

#define MPU_RASR_ATTRS_Pos 16
#define MPU_RASR_XN_Pos 28
#define MPU_RASR_AP_Pos 24
#define MPU_RASR_TEX_Pos 19
#define MPU_RASR_S_Pos 18
#define MPU_RASR_C_Pos 17
#define MPU_RASR_B_Pos 16
#define MPU_RASR_SRD_Pos 8
#define MPU_RASR_SIZE_Pos 1

#define MPU_RASR_ENABLE 1

#define MPU_REGION_SIZE_32MB 0x18
#define MPU_REGION_FULL_ACCESS 0x3
