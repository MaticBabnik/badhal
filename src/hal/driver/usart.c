#include "usart.h"

u32 usart_freq = 64000000ul;

void usart_use_hsi() {
    RCC->CR |= RCC_CR_HSION;
    while (!(RCC->CR & RCC_CR_HSIRDY)) {
    }

    usart_freq = 64000000ul;
    RCC->D2CCIP2R = (RCC->D2CCIP2R & ~RCC_D2CCIP2R_USART234578SEL_Mask)
                    | RCC_D2CCOP2R_USART234578SEL_HSI;
}

void usart_setup_basic(struct USART_t *usart, u32 baudrate) {
    if (usart == USART3) {
        RCC->APB1LENR |= RCC_APB1LENR_USART3EN;

        // Force a clean peripheral state so startup behavior is identical
        // on power-on reset and debug reflash/reset cycles.
        RCC->APB1LRSTR |= RCC_APB1LRSTR_USART3RST;
        RCC->APB1LRSTR &= ~RCC_APB1LRSTR_USART3RST;
    } else
        sys_trap("Unknown USART");

    usart->CR1 = 0; // disable
    usart->CR2 = 0;
    usart->CR3 = 0;
    usart->BRR = (usart_freq / baudrate); // baud rate
    usart->CR1 = USART_CR1_UE | USART_CR1_TE | USART_CR1_RE;

    while (!(usart->ISR & USART_ISR_TEACK) || !(usart->ISR & USART_ISR_REACK)) {
    }
}

char usart_recv(struct USART_t *usart) {
    // wait for input
    while (!(usart->ISR & USART_ISR_RXNE))
        ;
    return (char) usart->RDR;
}

void usart_send(struct USART_t *usart, char chr) {
    while (!(usart->ISR & USART_ISR_TXE))
        ;

    usart->TDR = (u32) chr;
}

void usart_send_string(struct USART_t *usart, const char *string) {
    while (true) {
        char chr = *(string++);
        if (!chr) return;
        usart_send(usart, chr);
    }
}