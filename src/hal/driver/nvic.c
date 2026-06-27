#include "nvic.h"
#include "../badhal.h"

void nvic_priority_grouping(u32 priority) {
    u32 reg = SCB->AIRCR;

    reg &= ~(SCB_AIRCR_VECTKEY_Msk | SCB_AIRCR_PRIGROUP_Msk); // clear bits
    reg |= (0x5FA) << SCB_AIRCR_VECTKEY_Pos;                  // set key
    reg |= (priority) << SCB_AIRCR_PRIGROUP_Pos;              // set priority

    SCB->AIRCR = reg; // write back
}

void nvic_enable_irq(NVIC_IRQ_t irq) {
    u32 nvic_reg = irq >> 5U;           // divide by 32
    u32 nvic_bit = 1U << (irq & 0x1FU); // modulo 32

    NVIC->ISER[nvic_reg] = nvic_bit;
}

void nvic_disable_irq(NVIC_IRQ_t irq) {
    u32 nvic_reg = irq >> 5U;           // divide by 32
    u32 nvic_bit = 1U << (irq & 0x1FU); // modulo 32

    NVIC->ICER[nvic_reg] = nvic_bit;
}

void nvic_set_priority(NVIC_IRQ_t irq, u32 prio) {
    NVIC->IP[irq] = prio;
}

void nvic_clear_pending(NVIC_IRQ_t irq) {
    u32 nvic_reg = irq >> 5U;           // divide by 32
    u32 nvic_bit = 1U << (irq & 0x1FU); // modulo 32

    NVIC->ICPR[nvic_reg] = nvic_bit;
}
