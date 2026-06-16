#include "../core/bad.h"

bool mpu_enabled();

void mpu_disable();

void mpu_enable();

void mpu_enable_cfg(u32 flags);

void mpu_set_region(u32 region, u32 addr, u32 rasr);

void mpu_disable_region(u32 region);