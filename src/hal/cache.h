#pragma once
#include "badhal.h"

static inline void sys_icache_enable()
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

static inline void sys_dcache_enable()
{
    a_dsb();
    a_isb();
    SCB->CCR |= SCB_CCR_DC; // enable D-cache
    a_dsb();
    a_isb();
}

static inline void sys_dcache_flush()
{
    a_dsb();
    a_isb();
    SCB->DCCISW = 0; // clean and invalidate D-cache
    a_dsb();
    a_isb();
}

static inline void sys_dcache_invalidate()
{
    a_dsb();
    a_isb();
    SCB->DCISW = 0; // invalidate D-cache
    a_dsb();
    a_isb();
}

static inline void sys_dcache_disable()
{
    a_dsb();
    a_isb();
    SCB->CCR &= ~SCB_CCR_DC; // disable D-cache
    a_dsb();
    a_isb();
}