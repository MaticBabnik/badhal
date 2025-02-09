#pragma once
#include "../bad.h"

struct CoreDebug_t
{
    R_RW u32 DHCSR;
    R_WO u32 DCRSR;
    R_RW u32 DCRDR;
    R_RW u32 DEMCR;
};

struct DBGMCU_t
{
    R_RW u32 IDCODE;
    R_RW u32 CR;
    u32 __reserved4[11];
    R_RW u32 APB3FZ1;
    u32 __reserved5;
    R_RW u32 APB1LFZ1;
    u32 __reserved6;
    R_RW u32 APB1HFZ1;
    u32 __reserved7;
    R_RW u32 APB2FZ1;
    u32 __reserved8;
    R_RW u32 APB4FZ1;
};

struct TPI_t
{
    R_RO u32 SSPSR;
    R_RW u32 CSPSR;
    u32 __reserved0[2U];
    R_RW u32 ACPR;
    u32 __reserved1[55U];
    R_RW u32 SPPR;
    u32 __reserved2[131U];
    R_RO u32 FFSR;
    R_RW u32 FFCR;
    R_RO u32 FSCR;
    u32 __reserved3[759U];
    R_RO u32 TRIGGER;
    R_RO u32 FIFO0;
    R_RO u32 ITATBCTR2;
    u32 __reserved4[1U];
    R_RO u32 ITATBCTR0;
    R_RO u32 FIFO1;
    R_RW u32 ITCTRL;
    u32 __reserved5[39U];
    R_RW u32 CLAIMSET;
    R_RW u32 CLAIMCLR;
    u32 __reserved7[8U];
    R_RO u32 DEVID;
    R_RO u32 DEVTYPE;
};

struct ITM_t
{
    R_WO union
    {
        R_WO u8 u8;
        R_WO u16 u16;
        R_WO u32 u32;
    } PORT[32U];
    u32 __reserved0[864U];
    R_RW u32 TER;
    u32 __reserved1[15U];
    R_RW u32 TPR;
    u32 __reserved2[15U];
    R_RW u32 TCR;
    u32 __reserved3[32U];
    u32 __reserved4[43U];
    R_WO u32 LAR;
    R_RO u32 LSR;
    u32 __reserved5[6U];
    R_RO u32 PID4;
    R_RO u32 PID5;
    R_RO u32 PID6;
    R_RO u32 PID7;
    R_RO u32 PID0;
    R_RO u32 PID1;
    R_RO u32 PID2;
    R_RO u32 PID3;
    R_RO u32 CID0;
    R_RO u32 CID1;
    R_RO u32 CID2;
    R_RO u32 CID3;
};

#define DBGMCU_CR_TRACE_IOEN 0x1
#define CoreDebug_DEMCR_TRCENA 0x01000000

#define ITM_TCR_ITMENA (1 << 0)
#define ITM_TCR_TSENA (1 << 1)
