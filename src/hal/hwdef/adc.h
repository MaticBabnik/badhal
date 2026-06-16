#pragma once
#include "../core/bad.h"

struct ADC_t {
    R_RW u32 ISR;
    R_RW u32 IER;
    R_RW u32 CR;
    R_RW u32 CFGR;
    R_RW u32 CFGR2;
    R_RW u32 SMPR1;
    R_RW u32 SMPR2;
    R_RW u32 PCSEL;
    R_RW u32 LTR1;
    R_RW u32 HTR1;
    u32 __reserved1;
    u32 __reserved2;
    R_RW u32 SQR1;
    R_RW u32 SQR2;
    R_RW u32 SQR3;
    R_RW u32 SQR4;
    R_RW u32 DR;
    u32 __reserved3;
    u32 __reserved4;
    R_RW u32 JSQR;
    u32 __reserved5[4];
    R_RW u32 OFR1;
    R_RW u32 OFR2;
    R_RW u32 OFR3;
    R_RW u32 OFR4;
    u32 __reserved6[4];
    R_RW u32 JDR1;
    R_RW u32 JDR2;
    R_RW u32 JDR3;
    R_RW u32 JDR4;
    u32 __reserved7[4];
    R_RW u32 AWD2CR;
    R_RW u32 AWD3CR;
    u32 __reserved8;
    u32 __reserved9;
    R_RW u32 LTR2;
    R_RW u32 HTR2;
    R_RW u32 LTR3;
    R_RW u32 HTR3;
    R_RW u32 DIFSEL;
    R_RW u32 CALFACT;
    R_RW u32 CALFACT2;
};

#define ADC_CR_ADEN (1ul << 0ul)
#define ADC_CR_ADSTART (1ul << 2ul)
#define ADC_CR_ADVREGEN (1ul << 28ul)
#define ADC_CR_DEEPPWD (1ul << 29ul)
#define ADC_CR_ADCAL (1ul << 31ul)

#define ADC_CFGR_JQDIS (1ul << 31ul)
#define ADC_CFGR_CONT (1ul << 13ul)
#define ADC_CFGR_OVRMOD (1ul << 12ul)
#define ADC_CFGR_RES_10bit (0x3ul << 2ul)

#define ADC_ISR_ADRDY (0x1ul << 0ul)
