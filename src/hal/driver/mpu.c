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

    if (dc) sys_dcache_disable();
    if (mpu) mpu_disable();

    ASSERT_RANGE(region, 0, 7, "mpu region out of range");

    MPU->RNR = region;
    MPU->RBAR = addr;
    MPU->RASR = rasr;

    if (dc) sys_dcache_enable();
    if (mpu) mpu_enable();
}

void mpu_set_region_hl(
    u32 region, u32 addr, u32 size, CacheType_t cache, MPUAccess_t access
) {
    ASSERT(
        popcnt(size) == 1 && size >= 32,
        "size must be a power of 2 and atleast 32"
    );

    // re-encode size to the MPU_RASR_SIZE format
    // size in bytes = 2^(size+1)
    u32 size_enc = 30u - a_clz(size);

    mpu_set_region(
        region, addr,
        MPU_RASR_ENABLE | (size_enc << MPU_RASR_SIZE_Pos)
            | ((cache & 0x3) << MPU_RASR_B_Pos) | (access << MPU_RASR_AP_Pos)
    );
}

void mpu_disable_region(u32 region) {
    mpu_set_region(region, 0, 0);
}
