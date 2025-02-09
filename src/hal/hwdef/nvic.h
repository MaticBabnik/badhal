#pragma once
#include "../bad.h"

struct NVIC_t
{
    R_RW u32 ISER[8U];
    u32 reserved0[24U];
    R_RW u32 ICER[8U];
    u32 reserved1[24U];
    R_RW u32 ISPR[8U];
    u32 reserved2[24U];
    R_RW u32 ICPR[8U];
    u32 reserved3[24U];
    R_RW u32 IABR[8U];
    u32 reserved4[56U];
    R_RW u8 IP[240U];
    u32 reserved5[644U];
    R_WO u32 STIR;
};