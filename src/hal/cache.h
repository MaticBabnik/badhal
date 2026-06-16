#pragma once
#include "badhal.h"

//TODO: this is really driver/scb

INLINE_ALWAYS void sys_icache_enable() {
    a_dsb();
    a_isb();
    SCB->ICIALLU = 0; // invalidate I-cache
    a_dsb();
    a_isb();
    SCB->CCR |= SCB_CCR_IC; // enable I-cache
    a_dsb();
    a_isb();
}

INLINE_ALWAYS void sys_dcache_invalidate() {
    SCB->CSSELR = SCB_CSSELR_D_L1;
    a_dsb();

    u32 dcache = SCB->CCSIDR;
    u32 sets = (dcache & SCB_CCSIDR_SETS_Msk) >> SCB_CCSIDR_SETS_Pos;
    u32 ways = (dcache & SCB_CCSIDR_WAYS_Msk) >> SCB_CCSIDR_WAYS_Pos;

    for (u32 set = 0; set <= sets; set++) {
        for (u32 way = 0; way <= ways; way++) {
            SCB->DCISW =
                (set << SCB_DCISW_SET_Pos) | (way << SCB_DCISW_WAY_Pos);
        }
    }

    a_dsb();
}

INLINE_ALWAYS void sys_dcache_enable() {
    if (SCB->CCR & SCB_CCR_DC) return;

    sys_dcache_invalidate(); // this already selects L1 for us

    SCB->CCR |= SCB_CCR_DC; // enable D-cache

    a_dsb();
    a_isb();
}

INLINE_ALWAYS void sys_dcache_flush() {
    SCB->CSSELR = SCB_CSSELR_D_L1;
    a_dsb();

    // figure out the cache layout
    u32 dcache = SCB->CCSIDR;
    u32 sets = (dcache & SCB_CCSIDR_SETS_Msk) >> SCB_CCSIDR_SETS_Pos;
    u32 ways = (dcache & SCB_CCSIDR_WAYS_Msk) >> SCB_CCSIDR_WAYS_Pos;

    for (u32 set = 0; set <= sets; set++) {
        for (u32 way = 0; way <= ways; way++) {
            SCB->DCCISW =
                (set << SCB_DCCISW_SET_Pos) | (way << SCB_DCCISW_WAY_Pos);
        }
    }

    a_dsb();
    a_isb();
}

INLINE_ALWAYS void sys_dcache_disable() {
    SCB->CSSELR = SCB_CSSELR_D_L1;
    a_dsb();

    SCB->CCR &= ~SCB_CCR_DC; // disable D-cache
    a_dsb();

    sys_dcache_flush();
}

INLINE_ALWAYS bool sys_dcache_enabled() {
    return (SCB->CCR & SCB_CCR_DC) >> SCB_CCR_DC_Pos;
}

INLINE_ALWAYS bool sys_icache_enabled() {
    return (SCB->CCR & SCB_CCR_IC) >> SCB_CCR_IC_Pos;
}