#pragma once
#include "../core/bad.h"

typedef enum {
    CK_PLL_SRC_HSI = 0,
    CK_PLL_SRC_CSI = 1,
    CK_PLL_SRC_HSE = 2,
    CK_PLL_SRC_None = 3
} ClkPllSrc_t;

typedef enum {
    PLL_DivP = 1,
    PLL_DivQ = 2,
    PLL_DivR = 4,
    PLL_Div_All = 7
} ClkPllDiv_t;

typedef enum {
    PLL_Range_1_2MHz = 0,
    PLL_Range_2_4MHz = 1,
    PLL_Range_4_8MHz = 2,
    PLL_Range_8_16MHz = 3,
} ClkPllRange_t;

void clk_pll_disable_all();
void clk_pll_enable_one(u32 pll);
void clk_pll_enable_all();

void clk_wait_ready(u32 rcc_cr_rdymask);

void clk_pll_srccfg(ClkPllSrc_t src, u32 divm1, u32 divm2, u32 divm3);
void clk_pll_cfg(
    u32 pll,
    bool fraction,
    bool mediumVco,
    ClkPllRange_t range,
    ClkPllDiv_t diven
);
void clk_pll_divcfg(u32 pll, u32 divn, u32 divp, u32 divq, u32 divr);

#define PLL1 1
#define PLL2 2
#define PLL3 3

#define P_NO_FRAC false
#define P_FRAC true
#define P_WIDE_VCO false
#define P_MEDIUM_VCO true

// TODO: D1CPRE, HPRE, CortexDiv?, CortexDiv2?, D1PPRE, D2PPRE1, D2PPRE2, D3PPRE
// TODO: PER mux, USART234578 mux, TRACE mux, RNG mux, I2C 123 MUX, FMC Mux, ...
// TODO: clock security???