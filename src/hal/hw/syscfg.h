#pragma once

#include "../bad.h"


struct SYSCFG_t
{
    u32 RESERVED1;
    R_RW u32 PMCR;
    R_RW u32 EXTICR[4];
    R_RW u32 CFGR;
    u32 RESERVED2;
    R_RW u32 CCCSR;
    R_RW u32 CCVR;
    R_RW u32 CCCR;
    R_RW u32 PWRCR;
    u32 RESERVED3[61];
    R_RW u32 PKGR;
    u32 RESERVED4[118];
    R_RW u32 UR0;
    R_RW u32 UR1;
    R_RW u32 UR2;
    R_RW u32 UR3;
    R_RW u32 UR4;
    R_RW u32 UR5;
    R_RW u32 UR6;
    R_RW u32 UR7;
    R_RW u32 UR8;
    R_RW u32 UR9;
    R_RW u32 UR10;
    R_RW u32 UR11;
    R_RW u32 UR12;
    R_RW u32 UR13;
    R_RW u32 UR14;
    R_RW u32 UR15;
    R_RW u32 UR16;
    R_RW u32 UR17;
};

#define SYSCFG_CCCSR_EN 1
#define SYSCFG_PWRCR_ODEN 1ul
