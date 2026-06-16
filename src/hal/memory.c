#include "badhal.h"

// TODO: this is driver/mpu and bsp/extmem

void mem_mpu_enable(u32 ctrl) {
    // Enable MPUs
    MPU->CTRL = ctrl | MPU_CTRL_ENABLE;
    // Enable fault exceptions
    SCB->SHCSR |= SCB_SHCSR_MEMFAULTENA;
    // sync data & flush instruction pipeline
    a_dsb();
    a_isb();
}

void mem_mpu_disable() {
    // finish up all memory accesses
    a_dmb();
    // Disable fault exceptions
    SCB->SHCSR &= ~SCB_SHCSR_MEMFAULTENA;
    // Disable MPU
    MPU->CTRL = 0;
}

void mem_mpu_setup_sdram(CacheType_t c) {
    bool dcache_was_enabled = (SCB->CCR & SCB_CCR_DC) != 0;

    if (dcache_was_enabled) {
        sys_dcache_disable();
    }

    mem_mpu_disable();

    // region 1
    MPU->RNR = 1;
    MPU->RBAR = SDRAM_BASE;
    MPU->RASR = MPU_RASR_ENABLE | (MPU_REGION_SIZE_16MB << MPU_RASR_SIZE_Pos)
                | ((c & 0x3) << MPU_RASR_B_Pos)
                | (MPU_REGION_FULL_ACCESS << MPU_RASR_AP_Pos);

    mem_mpu_enable(MPU_CTRL_PRIVDEFENA);

    if (dcache_was_enabled) {
        sys_dcache_enable();
    }
}
