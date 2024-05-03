#include "hal/roka.h"
#include <stdbool.h>

#define LED_OFF 0x00002000
#define LED_ON 0x20000000

void led_setup()
{
    // Enable GPIOI Peripheral Clock
    RCC->AHB4ENR |= GPIOIEN;
    // Make GPIOI Pin13 as output pin (bits 27:26 in MODER register)
    GPIOI->MODER = (GPIOI->MODER & 0xF3FFFFFF) | 0x04000000;
}

void led_write(bool x)
{
    GPIOI->BSRR = x ? LED_ON : LED_OFF;
}

void glavno()
{   
    sys_earlyinit();
    mem_mpu_setup_sdram();
    mem_enable_icache();
    sys_lateinit();

    led_setup();

    for (;;)
    {
        led_write(1);
        sys_delay_ms(500);
        led_write(0);
        sys_delay_ms(500);
    }
}