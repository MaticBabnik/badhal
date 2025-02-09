#include "hal/badhal.h"
#include "hal/debug.h"

#include "HD44780.h"

#define SWO_2MHZ 2000000

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

i32 main()
{
    gpio_init_all_ports();
    gpio_init_output(GPIOI, 13, None, Low, PushPull);

    // clock USARTs with the 64MHz HSI
    RCC->D2CCIP2R = (RCC->D2CCIP2R & ~RCC_D2CCIP2R_USART234578SEL_Mask) | RCC_D2CCOP2R_USART234578SEL_HSI;
    // enable USART3
    RCC->APB1LENR |= RCC_APB1LENR_USART3EN;
    // connect USART3 to STLINK
    gpio_init_alt(GPIOB, 10, 7, None, Medium, PushPull);
    gpio_init_alt(GPIOB, 11, 7, None, Medium, PushPull);

    USART3->CR1 = 0;                       // disable & clear USART3 (should be by default)
    USART3->BRR = (64000000ul / 115200ul); // baud rate
    USART3->CR1 = 0xD;                     // TxEn, RxEn, UsartEn

    for (;;)
    {
        usart_send_string(USART3, "Hello, world!\r\n");
        char r = usart_recv(USART3);
        usart_send_string(USART3, "You sent: ");
        usart_send(USART3, r);
        usart_send_string(USART3, "\r\n");
    }
}

void onTick()
{
    static int n = 333;
    n--;
    if (n == 0)
    {
        n = 333;
        gpio_toggle(GPIOI, 13);
    }
}

void entry()
{
    // don't mess with ordering around here too much
    sys_earlyinit();
    sys_icache_enable();
    sys_dcache_enable();
    mem_mpu_setup_sdram(); // mounts external RAM
    sys_lateinit();        // (sets up 64MHz systick)
    sys_go_fast();         // (switches to PLL1@400MHz, fixes systick)
    swo_init(SWO_2MHZ);    // enable SWO logging

    main();
}
