#pragma once
#include <core/bad.h>

typedef struct {
    R_RW u32 BTCR[8];
} FMC_Bank1_t;

typedef struct {
    R_RW u32 SDCRL;
    R_RW u32 SDCRH;
    R_RW u32 SDTRL;
    R_RW u32 SDTRH;

    R_RW u32 SDCMR;
    R_RW u32 SDRTR;
    R_RW u32 SDSR;
} FMC_Bank5_6_t;