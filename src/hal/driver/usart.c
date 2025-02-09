#include "usart.h"

u32 usart_freq = 64000000ul;

void usart_use_hsi()
{
    usart_freq = 64000000ul;
    RCC->D2CCIP2R = (RCC->D2CCIP2R & ~RCC_D2CCIP2R_USART234578SEL_Mask) | RCC_D2CCOP2R_USART234578SEL_HSI;
}

void usart_setup_basic(struct USART_t *usart, u32 baudrate)
{
    if (usart == USART3)
    {
        RCC->APB1LENR |= RCC_APB1LENR_USART3EN;
    }
    else
        sys_trap("Unknown USART");

    usart->CR1 = 0;                       // disable & clear
    usart->BRR = (usart_freq / baudrate); // baud rate
    usart->CR1 = 0xD;                     // enable (TxEn, RxEn, UsartEn)
}

char usart_recv(struct USART_t *usart)
{
    // wait for input
    while (!(usart->ISR & USART_ISR_RXNE))
        ;
    return (char)usart->RDR;
}

void usart_send(struct USART_t *usart, char chr)
{
    while (!(usart->ISR & USART_ISR_TXE))
        ;

    usart->TDR = (u32)chr;
}

void usart_send_string(struct USART_t *usart, const char *string)
{
    while (true)
    {
        char chr = *(string++);
        if (!chr)
            return;
        usart_send(usart, chr);
    }
}