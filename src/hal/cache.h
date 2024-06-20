#pragma once
#include "badhal.h"

B_INLINE void sys_icache_enable()
{
    a_dsb();
    a_isb();
    SCB->ICIALLU = 0; // invalidate I-cache
    a_dsb();
    a_isb();
    SCB->CCR |= SCB_CCR_IC; // enable I-cache
    a_dsb();
    a_isb();
}

B_INLINE void sys_dcache_invalidate()
{
    SCB->CSSELR = SCB_CSSELR_D_L1;
    a_dsb();

    // figure out the cache layout
    u32 dcache = SCB->CCSIDR;
    u32 sets = (dcache & SCB_CCSIDR_SETS_Msk) >> SCB_CCSIDR_SETS_Pos;
    do
    {
        u32 ways = (dcache & SCB_CCSIDR_WAYS_Msk) >> SCB_CCSIDR_WAYS_Pos;
        do
        {
            SCB->DCISW = (sets << SCB_DCISW_SET_Pos) | (ways << SCB_DCISW_WAY_Pos);
        } while (ways-- != 0);
    } while (sets-- != 0);

    a_dsb();
}

B_INLINE void sys_dcache_enable()
{
    if (SCB->CCR & SCB_CCR_DC)
        return;

    sys_dcache_invalidate(); // this already selects L1 for us

    SCB->CCR |= SCB_CCR_DC; // enable D-cache

    a_dsb();
    a_isb();
}

B_INLINE void sys_dcache_flush()
{
    SCB->CSSELR = SCB_CSSELR_D_L1;
    a_dsb();

    // figure out the cache layout
    u32 dcache = SCB->CCSIDR;
    u32 sets = (dcache & SCB_CCSIDR_SETS_Msk) >> SCB_CCSIDR_SETS_Pos;
    do
    {
        u32 ways = (dcache & SCB_CCSIDR_WAYS_Msk) >> SCB_CCSIDR_WAYS_Pos;
        do
        {
            SCB->DCCISW = (sets << SCB_DCCISW_SET_Pos) | (ways << SCB_DCCISW_WAY_Pos);
        } while (ways-- != 0);
    } while (sets-- != 0);

    a_dsb();
    a_isb();
}

B_INLINE void sys_dcache_disable()
{
    SCB->CSSELR = SCB_CSSELR_D_L1;
    a_dsb();
    
    SCB->CCR &= ~SCB_CCR_DC; // disable D-cache
    a_dsb();

    sys_dcache_flush();
}