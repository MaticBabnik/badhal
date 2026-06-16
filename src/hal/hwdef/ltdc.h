#pragma once
#include "../core/bad.h"

struct LTDC_t {
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

struct LTDC_Layer_t {
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


#define LTDC_SSCR_HSW_Pos 16
#define LTDC_SSCR_VSH_Pos 0
#define LTDC_BPCR_HSW_Pos 16
#define LTDC_BPCR_VSH_Pos 0
#define LTDC_AWCR_HSW_Pos 16
#define LTDC_AWCR_VSH_Pos 0
#define LTDC_TWCR_HSW_Pos 16
#define LTDC_TWCR_VSH_Pos 0

#define LTDC_LxCR_LEN (1ul << 0)
#define LTDC_LxCR_COLKEN (1ul << 1)
#define LTDC_LxCR_CLUTEN (1ul << 4)


#define LTDC_LxWHPCR_Stop_Pos 16
#define LTDC_LxWHPCR_Start_Pos 0

#define LTDC_LxWVPCR_Stop_Pos 16
#define LTDC_LxWVPCR_Start_Pos 0

#define LTDC_LxPFCR_ARGB8888 0ul
#define LTDC_LxPFCR_RGB888 1ul
#define LTDC_LxPFCR_RGB565 2ul
#define LTDC_LxPFCR_ARGB1555 3ul
#define LTDC_LxPFCR_ARGB4444 4ul
#define LTDC_LxPFCR_L8 5ul
#define LTDC_LxPFCR_AL44 6ul
#define LTDC_LxPFCR_AL88 7ul

#define LTDC_LxCFBLR_CFBP_Pos 16
#define LTDC_LxCFBLR_CFBLL_Pos 0

#define LTDC_IER_LIE (1ul << 0)
#define LTDC_ICR_CLIF (1ul << 0)