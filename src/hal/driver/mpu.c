#include "mpu.h"
#include "../badhal.h"

bool mpu_enabled() {
    return MPU->CTRL & MPU_CTRL_ENABLE;
}

void mpu_disable() {
    a_dmb();

    MPU->CTRL &= ~MPU_CTRL_ENABLE;
}

void mpu_enable() {
    MPU->CTRL |= MPU_CTRL_ENABLE;
}

void mpu_enable_cfg(u32 flags) {
    MPU->CTRL = flags | MPU_CTRL_ENABLE;
}

void mpu_set_region(u32 region, u32 addr, u32 rasr) {
    bool dc = sys_dcache_enabled();
    bool mpu = mpu_enabled();

    if (dc) sys_icache_disable();
    if (mpu) mpu_disable();

    ASSERT_RANGE(region, 0, 7, "mpu region out of range");

    MPU->RNR = region;
    MPU->RBAR = addr;
    MPU->RASR = rasr;

    if (dc) sys_icache_enabled();
    if (mpu) mpu_enable();
}

void mpu_disable_region(u32 region) {
    mpu_set_region(region, 0, 0);
}
