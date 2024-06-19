#include "badhal.h"

void mem_mpu_enable(u32 ctrl)
{
    // Enable MPUs
    MPU->CTRL = ctrl | MPU_CTRL_ENABLE;
    // Enable fault exceptions
    SCB->SHCSR |= SCB_SHCSR_MEMFAULTENA;
    // sync data & flush instruction pipeline
    a_dsb();
    a_isb();
}

void mem_mpu_disable()
{
    // finish up all memory accesses
    a_dmb();
    // Disable fault exceptions
    SCB->SHCSR &= ~SCB_SHCSR_MEMFAULTENA;
    // Disable MPU
    MPU->CTRL = 0;
}

void mem_mpu_setup_sdram()
{
    mem_mpu_disable();

    // region 1
    MPU->RNR = 1;
    MPU->RBAR = SDRAM_BASE;
    MPU->RASR = MPU_RASR_ENABLE |
                (MPU_REGION_SIZE_32MB << MPU_RASR_SIZE_Pos) | // SDRAM is 32M
                (1 << MPU_RASR_C_Pos) |                       // cacheable
                (MPU_REGION_FULL_ACCESS << MPU_RASR_AP_Pos);  // full access

    mem_mpu_enable(MPU_PRIVILEGED_DEFAULT);
}

