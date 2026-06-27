#include "badhal.h"
#include "driver/clock.h"
#include "driver/nvic.h"

u32 coreFreq = 64000000; // Hz

u32 sys_get_freq() {
    return coreFreq;
}

void sys_set_systick(u32 tick) {
    // update systick
    SysTick->LOAD = tick - 1;
    // sys_nvic_set_priority(-1, 7);
    SysTick->VAL = 0;
    SysTick->CTRL =
        SysTick_CTRL_CLKSOURCE | SysTick_CTRL_TICKINT | SysTick_CTRL_ENABLE;
}

volatile u32 tick = 0;
void SysTick_Handler() {
    tick++;
}

u32 sys_get_tick() {
    return tick;
}

void sys_delay_ms(u32 time) {
    u32 start = tick;

    if (time < 0xffffffff) time++;

    while ((tick - start) < time) {
    }
}

void sys_allfaults() {
    SCB->SHCSR |=
        SCB_SHCSR_BUSFAULTENA
        | SCB_SHCSR_USGFAULTENA; // SCB_SHCSR_MEMFAULTENA (handled in mpu fns)
    SCB->CCR |=
        SCB_CCR_DIV_0_TRP; // | SCB_CCR_UNALIGN_TRP; (randomly dies in crash)
}

INLINE_NEVER void sys_trap(volatile char *msg) {
    for (;;) {
        a_nop();
    }
}

void sys_reboot() {
    SCB->AIRCR = ((0x5FA) << SCB_AIRCR_VECTKEY_Pos) | SCB_AIRCR_SYSRESETREQ;
    a_dsb();
    sys_trap("reboot");
}