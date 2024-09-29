#pragma once
#include "../bad.h"

struct LTDC_t
{
    u32 reserved0[2];
    R_RW u32 SSCR;
    R_RW u32 BPCR;
    R_RW u32 AWCR;
    R_RW u32 TWCR;
    R_RW u32 GCR;
    u32 reserved1[2];
    R_RW u32 SRCR;
    u32 reserved2[1];
    R_RW u32 BCCR;
    u32 reserved3[1];
    R_RW u32 IER;
    R_RW u32 ISR;
    R_RW u32 ICR;
    R_RW u32 LIPCR;
    R_RW u32 CPSR;
    R_RW u32 CDSR;
};

struct LTDC_Layer_t
{
    R_RW u32 CR;
    R_RW u32 WHPCR;
    R_RW u32 WVPCR;
    R_RW u32 CKCR;
    R_RW u32 PFCR;
    R_RW u32 CACR;
    R_RW u32 DCCR;
    R_RW u32 BFCR;
    u32 reserved0[2];
    R_RW u32 CFBAR;
    R_RW u32 CFBLR;
    R_RW u32 CFBLNR;
    u32 reserved1[3];
    R_RW u32 CLUTWR;
};

void ltdc_init();