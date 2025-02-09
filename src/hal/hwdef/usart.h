#pragma once
#include "../bad.h"

struct USART_t
{
    R_RW u32 CR1;
    R_RW u32 CR2;
    R_RW u32 CR3;
    R_RW u32 BRR;
    R_RW u32 GTPR;
    R_RW u32 RTOR;
    R_RW u32 RQR;
    R_RW u32 ISR;
    R_RW u32 ICR;
    R_RW u32 RDR;
    R_RW u32 TDR;
    R_RW u32 PRESC;
};

#define USART_ISR_RXNE (1ul << 5ul)
#define USART_ISR_TXE (1ul << 7ul)