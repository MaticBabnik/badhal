#pragma once
#include <core/bad.h>
#include "../hwdef/nvic.h"

void nvic_priority_grouping(u32 priority);

void nvic_enable_irq(NVIC_IRQ_t irq);

void nvic_disable_irq(NVIC_IRQ_t irq);

void nvic_set_priority(NVIC_IRQ_t irq, u32 prio);

void nvic_clear_pending(NVIC_IRQ_t irq);
