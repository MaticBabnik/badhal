#include "hal/badhal.h"
#include <stdbool.h>

#define LED_OFF 0x00002000
#define LED_ON 0x20000000

void led_setup()
{
    // Enable GPIOI Peripheral Clock
    RCC->AHB4ENR |= RCC_AHB4ENR_GPIOIEN;
    // Make GPIOI Pin13 as output pin (bits 27:26 in MODER register)
    GPIOI->MODER = (GPIOI->MODER & 0xF3FFFFFF) | 0x04000000;
}

void led_write(bool x)
{
    GPIOI->BSRR = x ? LED_ON : LED_OFF;
}

void entry()
{
    // don't mess with ordering around here too much
    sys_earlyinit();
    
    sys_icache_enable();
    // this shit hangs
    // fuckt tbnraishik0u-=ee=
    // aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaAAAAAAAAAA
    sys_dcache_invalidate();
    sys_dcache_enable();

    mem_mpu_setup_sdram(); // mounts external RAM
    sys_lateinit(); // (sets up 64MHz systick)
    sys_go_fast();  // (switches to PLL1@400MHz, fixes systick)
    
    led_setup();    


    for (;;)
    {
        led_write(1);
        sys_delay_ms(500);
        led_write(0);
        sys_delay_ms(500);
    }
}