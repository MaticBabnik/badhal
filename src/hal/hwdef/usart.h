#pragma once
#include <core/bad.h>

typedef struct {
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
} USART_t;

#define USART_ISR_RXNE (1ul << 5ul)
#define USART_ISR_TXE (1ul << 7ul)

#define USART_CR1_UE (1UL << 0)
#define USART_CR1_RE (1UL << 2)
#define USART_CR1_TE (1UL << 3)

#define USART_ISR_REACK (1UL << 22)
#define USART_ISR_TEACK (1UL << 21)