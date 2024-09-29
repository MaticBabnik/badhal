#include "debug.h"

void swo_init(u32 swo_freq)
{
    // Set up Debug Output (SWO) to be used for logging
    DBGMCU->CR |= DBGMCU_CR_TRACE_IOEN;
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA;

    TPI->SPPR = 2; // Select NRZ mode for SWO
    TPI->ACPR = sys_get_freq() / swo_freq - 1;
    
    ITM->LAR = 0xC5ACCE55; // Unlock access to ITM
    ITM->TPR = 0;
    ITM->TCR = ITM_TCR_ITMENA | ITM_TCR_TSENA | (1 << 16 /*trust me*/);
    ITM->TER = 0xffffffff; // Enable all ports
}

void swo_putc(char c)
{
    while (ITM->PORT[0].u32 == 0)
    {
        a_nop();
    }

    ITM->PORT[0U].u8 = c;
}

void swo_write(const char *data, u32 size)
{
    while (size--)
        swo_putc(*data++);
}

void swo_writestr(const char *str)
{
    for (;;)
    {
        char c = *(str++);
        if (!c)
            return;
        swo_putc(c);
    }
}