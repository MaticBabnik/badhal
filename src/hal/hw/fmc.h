#pragma once
#include "../bad.h"

struct FMC_Bank1_t
{
    R_RW u32 BTCR[8];
};

struct FMC_Bank5_6_t
{
    R_RW u32 SDCRL;
    R_RW u32 SDCRH;
    R_RW u32 SDTRL;
    R_RW u32 SDTRH;

    R_RW u32 SDCMR;
    R_RW u32 SDRTR;
    R_RW u32 SDSR;
};