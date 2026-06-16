/*
    Clock settings & related RCC bits
*/

#include "clock.h"
#include "../badhal.h"

/*
    These functions should really be inlined, since they can be mostly
   const-evaled
*/

void clk_pll_disable_all() {
    RCC->CR &= ~(RCC_CR_PLL1ON | RCC_CR_PLL2ON | RCC_CR_PLL3ON);
}

void clk_pll_enable_one(u32 pll /* 1-3 */) {
    ASSERT_RANGE(pll, 1, 3, "pll out of range");

    RCC->CR |= (RCC_CR_PLL1ON << ((pll - 1) * 2));
}

void clk_pll_enable_all() {
    RCC->CR |= RCC_CR_PLL1ON | RCC_CR_PLL2ON | RCC_CR_PLL3ON;
}

void clk_wait_ready(u32 rcc_cr_rdymask) {
    while ((RCC->CR & rcc_cr_rdymask) != rcc_cr_rdymask) {
    }
}

void clk_pll_srccfg(
    ClkPllSrc_t src,
    u32 divm1, // 0-63
    u32 divm2, // 0-63
    u32 divm3  // 0-63
) {
    ASSERT_RANGE(src, CK_PLL_SRC_HSI, CK_PLL_SRC_None, "invalid PLL source");
    ASSERT_RANGE(divm1, 0, 63, "divm1 out of range");
    ASSERT_RANGE(divm2, 0, 63, "divm2 out of range");
    ASSERT_RANGE(divm3, 0, 63, "divm3 out of range");

    RCC->PLLCKSELR = (divm3 << RCC_PLLSOURCE_DIVM3_Pos)
                     | (divm2 << RCC_PLLSOURCE_DIVM2_Pos)
                     | (divm1 << RCC_PLLSOURCE_DIVM1_Pos) | src;
}

void clk_pll_cfg(
    u32 pll,        // 1-3
    bool fraction,  // 0 or 1
    bool mediumVco, // 0 for wide, 1 for medium
    ClkPllRange_t range,
    ClkPllDiv_t diven
) {
    ASSERT_RANGE(pll, 1, 3, "pll out of range");
    ASSERT(range <= PLL_Range_8_16MHz, "invalid PLL range");
    ASSERT((diven & ~PLL_Div_All) == 0, "invalid PLL dividers");

    u32 pllIdx = pll - 1;
    u32 loff = pllIdx * 4;
    u32 hoff = 3 * pllIdx + 16;
    u32 clearMask = ~((0xfu << loff) | (0x7u << hoff));
    fraction = fraction ? 1 : 0;
    mediumVco = mediumVco ? 1 : 0;

    RCC->PLLCFGR = (RCC->PLLCFGR & clearMask) | (fraction << loff)
                   | (mediumVco << (loff + 1)) | (range << (loff + 2))
                   | (diven << hoff);
}

void clk_pll_divcfg(
    u32 pll,  // 1-3
    u32 divn, // 4-512
    u32 divp, // even values 4-512
    u32 divq, // 1-128
    u32 divr  // 1-128
) {
    ASSERT_RANGE(pll, 1, 3, "pll out of range");
    ASSERT_RANGE(divn, 4, 512, "divn out of range");
    ASSERT_RANGE(divp, 4, 512, "divp out of range");
    ASSERT((divp & 1) == 0 || divp == 1, "divp can't be odd");
    ASSERT_RANGE(divq, 1, 128, "divq out of range");
    ASSERT_RANGE(divr, 1, 128, "divr out of range");

    // compute the register address for this PLL's dividers
    volatile u32 *pll_divr = &RCC->PLL1DIVR + (pll - 1) * 2;

    *pll_divr =
        (((divn - 1) & RCC_PLLxDIVR_DIVN_UMask) << RCC_PLLxDIVR_DIVN_Pos)
        | (((divp - 1) & RCC_PLLxDIVR_DIVP_UMask) << RCC_PLLxDIVR_DIVP_Pos)
        | (((divq - 1) & RCC_PLLxDIVR_DIVQ_UMask) << RCC_PLLxDIVR_DIVQ_Pos)
        | (((divr - 1) & RCC_PLLxDIVR_DIVR_UMask) << RCC_PLLxDIVR_DIVR_Pos);
}