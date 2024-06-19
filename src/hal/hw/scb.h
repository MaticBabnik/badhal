#pragma once

#include "../bad.h"

struct SCB_t
{
    R_RO u32 CPUID;
    R_RW u32 ICSR;
    R_RW u32 VTOR;
    R_RW u32 AIRCR;
    R_RW u32 SCR;
    R_RW u32 CCR;
    R_RW u8 SHPR[12U];
    R_RW u32 SHCSR;
    R_RW u32 CFSR;
    R_RW u32 HFSR;
    R_RW u32 DFSR;
    R_RW u32 MMFAR;
    R_RW u32 BFAR;
    R_RW u32 AFSR;
    R_RO u32 ID_PFR[2U];
    R_RO u32 ID_DFR;
    R_RO u32 ID_AFR;
    R_RO u32 ID_MFR[4U];
    R_RO u32 ID_ISAR[5U];
    u32 reserved0[1U];

    R_RO u32 CLIDR;
    R_RO u32 CTR;
    R_RO u32 CCSIDR;
    R_RW u32 CSSELR;
    R_RW u32 CPACR;
    u32 reserved3[93U];

    R_WO u32 STIR;
    u32 reserved4[15U];

    R_RO u32 MVFR0;
    R_RO u32 MVFR1;
    R_RO u32 MVFR2;
    u32 reserved5[1U];

    R_WO u32 ICIALLU;
    u32 reserved6[1U];

    R_WO u32 ICIMVAU;
    R_WO u32 DCIMVAC;
    R_WO u32 DCISW;
    R_WO u32 DCCMVAU;
    R_WO u32 DCCMVAC;
    R_WO u32 DCCSW;
    R_WO u32 DCCIMVAC;
    R_WO u32 DCCISW;
    u32 reserved7[6U];

    R_RW u32 ITCMCR;
    R_RW u32 DTCMCR;
    R_RW u32 AHBPCR;
    R_RW u32 CACR;
    R_RW u32 AHBSCR;
    u32 reserved8[1U];

    R_RW u32 ABFSR;
};

#define SCB_SHCSR_MEMFAULTENA (1UL << 16)

#define SCB_CCR_DC (1UL << 16)
#define SCB_CCR_IC (1UL << 17)
#define SCB_CCR_BP (1UL << 18)

#define SCB_AIRCR_VECTKEY_Pos 16U
#define SCB_AIRCR_VECTKEY_Msk (0xFFFFUL << SCB_AIRCR_VECTKEY_Pos)

#define SCB_AIRCR_PRIGROUP_Pos 8U
#define SCB_AIRCR_PRIGROUP_Msk (7UL << SCB_AIRCR_PRIGROUP_Pos)
#define SCB_CPARCR_FULL_ACCESS_EVERYTHING 0x0FFFFFFFUL
