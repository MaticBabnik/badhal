#pragma once

#include "../core/bad.h"

struct SCB_t {
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
#define SCB_SHCSR_BUSFAULTENA (1UL << 17)
#define SCB_SHCSR_USGFAULTENA (1UL << 18)

#define SCB_CCR_DC (1UL << 16)
#define SCB_CCR_DC_Pos 16
#define SCB_CCR_IC (1UL << 17)
#define SCB_CCR_IC_Pos 17
#define SCB_CCR_BP (1UL << 18)
#define SCB_CCR_STKALIGN (1UL << 9)
#define SCB_CCR_DIV_0_TRP (1UL << 4)
#define SCB_CCR_UNALIGN_TRP (1UL << 3)

#define SCB_AIRCR_VECTKEY_Pos 16U
#define SCB_AIRCR_VECTKEY_Msk (0xFFFFUL << SCB_AIRCR_VECTKEY_Pos)

#define SCB_AIRCR_PRIGROUP_Pos 8U
#define SCB_AIRCR_PRIGROUP_Msk (7UL << SCB_AIRCR_PRIGROUP_Pos)

#define SCB_CPACR_CP_DENIED (0ul)
#define SCB_CPACR_CP_PRIVILEGED (0x1ul)
#define SCB_CPACR_CP_FULL_ACCESS (0x3ul)
#define SCB_CPACR_CP_MASK (0x3ul)
#define SCB_CPACR_FPU1_Pos 20U
#define SCB_CPACR_FPU2_Pos 22U
#define SCB_CPACR_CP_Pos(n) (2U * (n))

#define SCB_CCSIDR_SETS_Pos 13
#define SCB_CCSIDR_SETS_Msk (0x7FFFUL << SCB_CCSIDR_SETS_Pos)

#define SCB_CCSIDR_WAYS_Pos 3
#define SCB_CCSIDR_WAYS_Msk (0x3FFUL << SCB_CCSIDR_WAYS_Pos)

#define SCB_DCISW_WAY_Pos 30
#define SCB_DCISW_SET_Pos 5
#define SCB_DCCISW_WAY_Pos 30
#define SCB_DCCISW_SET_Pos 5

#define SCB_CSSELR_D_L1 0
#define SCB_CSSELR_I_L1 1

#define SCB_CFSR_MM_IACCVIOL (1u << 0)
#define SCB_CFSR_MM_DACCVIOL (1u << 1)
#define SCB_CFSR_MM_MUNSTKERR (1u << 3)
#define SCB_CFSR_MM_MSTKERR (1u << 4)
#define SCB_CFSR_MM_MLSPERR (1u << 5)
#define SCB_CFSR_MM_MMARVALID (1u << 7)

#define SCB_CFSR_BUS_IBUSERR (1u << 8)
#define SCB_CFSR_BUS_PRECISERR (1u << 9)
#define SCB_CFSR_BUS_IMPRECISERR (1u << 10)
#define SCB_CFSR_BUS_UNSTKERR (1u << 11)
#define SCB_CFSR_BUS_STKERR (1u << 12)
#define SCB_CFSR_BUS_LSPERR (1u << 13)
#define SCB_CFSR_BUS_BFARVALID (1u << 15)

#define SCB_CFSR_USG_UNDEFINSTR (1u << 16)
#define SCB_CFSR_USG_INVSTATE (1u << 17)
#define SCB_CFSR_USG_INVPC (1u << 18)
#define SCB_CFSR_USG_NOCP (1u << 19)
#define SCB_CFSR_USG_UNALIGNED (1u << 24)
#define SCB_CFSR_USG_DIVBYZERO (1u << 25)

#define SCB_HFSR_VECTTBL (1u << 1)
#define SCB_HFSR_FORCED (1u << 30)
#define SCB_HFSR_DEBUGEVT (1u << 31)
