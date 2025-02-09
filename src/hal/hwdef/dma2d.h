#pragma once
#include "../bad.h"

struct DMA2D_t
{
    R_RW u32 CR;
    R_RW u32 ISR;
    R_RW u32 IFCR;
    R_RW u32 FGMAR;
    R_RW u32 FGOR;
    R_RW u32 BGMAR;
    R_RW u32 BGOR;
    R_RW u32 FGPFCCR;
    R_RW u32 FGCOLR;
    R_RW u32 BGPFCCR;
    R_RW u32 BGCOLR;
    R_RW u32 FGCMAR;
    R_RW u32 BGCMAR;
    R_RW u32 OPFCCR;
    R_RW u32 OCOLR;
    R_RW u32 OMAR;
    R_RW u32 OOR;
    R_RW u32 NLR;
    R_RW u32 LWR;
    R_RW u32 AMTCR;
    u32 reserved[236];
    R_RW u32 FGCLUT[256];
    R_RW u32 BGCLUT[256];
};
