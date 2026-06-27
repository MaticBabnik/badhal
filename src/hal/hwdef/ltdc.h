#pragma once
#include <core/bad.h>

typedef struct {
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
} LTDC_t;

typedef struct {
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
    R_WO u32 CLUTWR;
} LTDC_Layer_t;

#define LTDC_GCR_Pol_High 1ul
#define LTDC_GCR_Pol_Low 0ul

#define LTDC_GCR_HSPOL_Pos 31
#define LTDC_GCR_VSPOL_Pos 30
#define LTDC_GCR_DEPOL_Pos 29
#define LTDC_GCR_PCPOL_Pos 28

#define LTDC_GCR_DEN_Pos 16
#define LTDC_GCR_DEN (1ul << LTDC_GCR_DEN_Pos)
#define LTDC_GCR_DRW_Pos 12
#define LTDC_GCR_DGW_Pos 8
#define LTDC_GCR_DBW_Pos 4
#define LTDC_GCR_DxW_UMask (0x7ul)

#define LTDC_GCR_DRW_Mask (LTDC_GCR_DxW_UMask << LTDC_GCR_DRW_Pos)
#define LTDC_GCR_DGW_Mask (LTDC_GCR_DxW_UMask << LTDC_GCR_DGW_Pos)
#define LTDC_GCR_DBW_Mask (LTDC_GCR_DxW_UMask << LTDC_GCR_DBW_Pos)

#define LTD_GCR_LTDCEN 1ul

#define LTDC_IER_LIE (1ul << 0)
#define LTDC_ICR_CLIF (1ul << 0)

#define LTDC_SRCR_IMR (1ul << 0)
#define LTDC_SRCR_VBR (1ul << 1)

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

#define LTDC_LxBFCR_BF2_Pos 0
#define LTDC_LxBFCR_BF1_Pos 8