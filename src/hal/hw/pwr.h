#pragma once

#include "../bad.h"


struct PWR_t
{
    R_RW u32 CR1;
    R_RW u32 CSR1;
    R_RW u32 CR2;
    R_RW u32 CR3;
    R_RW u32 CPUCR;
    u32 RESERVED0;
    R_RW u32 D3CR;
    u32 RESERVED1;
    R_RW u32 WKUPCR;
    R_RW u32 WKUPFR;
    R_RW u32 WKUPEPR;
};


#define PWR_SUPPLY_CONFIG_MASK 0x7UL
#define PWR_SUPPLY_LDO 0x2
#define PWR_CSR1_ACTVOSRDY (1 << 13)
#define PWR_D3CR_VOS (0x3UL << 14)
#define PWR_REGULATOR_VOLTAGE_SCALE1 (0x3UL << 14)
