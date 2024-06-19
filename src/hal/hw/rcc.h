#pragma once

#include "../bad.h"

struct RCC_t
{
    R_RW u32 CR;
    R_RW u32 HSISCR;
    R_RW u32 CRRCR;
    R_RW u32 CSICFGR;
    R_RW u32 CFGR;
    u32 __reserved00;
    R_RW u32 D1CFGR;
    R_RW u32 D2CFGR;
    R_RW u32 D3CFGR;
    u32 __reserved1;

    R_RW u32 PLLCKSELR;
    R_RW u32 PLLCFGR;
    R_RW u32 PLL1DIVR;
    R_RW u32 PLL1FRACR;
    R_RW u32 PLL2DIVR;
    R_RW u32 PLL2FRACR;
    R_RW u32 PLL3DIVR;
    R_RW u32 PLL3FRACR;
    u32 __reserved2;

    R_RW u32 D1CCIPR;
    R_RW u32 D2CCIP1R;
    R_RW u32 D2CCIP2R;
    R_RW u32 D3CCIPR;
    u32 __reserved3;

    R_RW u32 CIER;
    R_RW u32 CIFR;
    R_RW u32 CICR;
    u32 __reserved4;

    R_RW u32 BDCR;
    R_RW u32 CSR;
    u32 __reserved5;

    R_RW u32 AHB3RSTR;
    R_RW u32 AHB1RSTR;
    R_RW u32 AHB2RSTR;
    R_RW u32 AHB4RSTR;

    R_RW u32 APB3RSTR;
    R_RW u32 APB1LRSTR;
    R_RW u32 APB1HRSTR;
    R_RW u32 APB2RSTR;
    R_RW u32 APB4RSTR;

    R_RW u32 GCR;
    u32 __reserved6;

    R_RW u32 D3AMR;
    u32 __reserved7[9];

    R_RW u32 RSR;

    R_RW u32 AHB3ENR;
    R_RW u32 AHB1ENR;
    R_RW u32 AHB2ENR;
    R_RW u32 AHB4ENR;

    R_RW u32 APB3ENR;
    R_RW u32 APB1LENR;
    R_RW u32 APB1HENR;
    R_RW u32 APB2ENR;
    R_RW u32 APB4ENR;
    u32 __reserved8;

    R_RW u32 AHB3LPENR;
    R_RW u32 AHB1LPENR;
    R_RW u32 AHB2LPENR;
    R_RW u32 AHB4LPENR;
    R_RW u32 APB3LPENR;
    R_RW u32 APB1LLPENR;
    R_RW u32 APB1HLPENR;
    R_RW u32 APB2LPENR;
    R_RW u32 APB4LPENR;
    u32 __reserved9[5];

    R_RW u32 C1_AHB3ENR;
    R_RW u32 C1_AHB1ENR;
    R_RW u32 C1_AHB2ENR;
    R_RW u32 C1_AHB4ENR;
    R_RW u32 C1_APB3ENR;
    R_RW u32 C1_APB1LENR;
    R_RW u32 C1_APB1HENR;
    R_RW u32 C1_APB2ENR;
    R_RW u32 C1_APB4ENR;
    u32 __reservedA;

    R_RW u32 C1_AHB3LPENR;
    R_RW u32 C1_AHB1LPENR;
    R_RW u32 C1_AHB2LPENR;
    R_RW u32 C1_AHB4LPENR;
    R_RW u32 C1_APB3LPENR;
    R_RW u32 C1_APB1LLPENR;
    R_RW u32 C1_APB1HLPENR;
    R_RW u32 C1_APB2LPENR;
    R_RW u32 C1_APB4LPENR;

    u32 __reservedB[31];
};

#define RCC_AHB2ENR_SRAM1EN (1UL << 29)
#define RCC_AHB2ENR_SRAM2EN (1UL << 30)
#define RCC_AHB2ENR_SRAM3EN (1UL << 31)

#define RCC_AHB3ENR_FMCEN (1UL << 12)
#define RCC_APB4ENR_SYSCFGEN (1UL < 1)
#define RCC_AHB4ENR_GPIOIEN (1UL << 8);

#define RCC_DxCFGR_DxPPRE1_Mask (0x7UL << 4)
#define RCC_DxCFGR_DxPPRE1_DIV2 (1UL << 6)

#define RCC_DxCFGR_DxPPRE2_Mask (0x7UL << 8)
#define RCC_DxCFGR_DxPPRE2_DIV2 (1UL << 10)

#define RCC_D1CFGR_HPRE_Mask (0x7UL)
#define RCC_D1CFGR_HPRE_DIV2 (1UL << 3)

#define RCC_D1CFGR_D1CPRE_Mask (0xFUL << 8)
#define RCC_D1CFGR_D1CPRE_DIV1 0UL

#define RCC_CFGR_SW_Mask 0x7UL
#define RCC_CFGR_SW_PLL1 0x3UL

#define RCC_CFGR_SWS_Mask (0x7UL << 3)
#define RCC_CFGR_SWS_PLL1 (0x3UL << 3)

#define RCC_CR_HSION 1UL
#define RCC_CR_CSION (1UL << 7)
#define RCC_CR_HSEON (1UL << 16)
#define RCC_CR_PLL1ON (1UL << 24)
#define RCC_CR_HSI48ON (1UL << 12)
#define RCC_CSR_LSION 1UL

// RCC_CR_CSION
#define RCC_CR_HSIRDY (1UL << 2)
#define RCC_CR_CSIRDY (1UL << 8)
#define RCC_CR_HSERDY (1UL << 17)
#define RCC_CR_PLL1RDY (1UL << 25)
#define RCC_CR_HSI48RDY (1UL << 13)
#define RCC_CSR_LSIRDY (1UL << 1)

#define RCC_PLLSOURCE_HSE (0x2UL)
#define RCC_PLLSOURCE_Mask (0x3UL)

#define RCC_PLLSOURCE_DIVM1_Pos 4UL
#define RCC_PLLSOURCE_DIVM1_Mask (0x3FUL << 4)

#define RCC_PLL1DIVR_DIVR_Pos 24UL
#define RCC_PLL1DIVR_DIVQ_Pos 16UL
#define RCC_PLL1DIVR_DIVP_Pos 9UL
#define RCC_PLL1DIVR_DIVN_Pos 0UL

#define RCC_PLLCFGR_PLL1FRACEN (1UL)
#define RCC_PLLCFGR_DIVP1EN (1UL << 16)
#define RCC_PLLCFGR_DIVQ1EN (1UL << 17)
#define RCC_PLLCFGR_DIVR1EN (1UL << 18)

#define RCC_PLLCFGR_PLL1VCOSEL_Mask (1UL << 1)
#define RCC_PLLCFGR_PLL1RGE_Pos 2UL
#define RCC_PLLCFGR_PLL1RGE_Mask (0x3UL << 2)

#define RCC_PLL1FRACR_FRACN1_Pos 3UL
#define RCC_PLL1FRACR_FRACN1_Mask (0xfffUL << 3)