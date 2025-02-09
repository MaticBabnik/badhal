#include "hal/badhal.h"
#include "hal/driver/debug.h"
#include "hal/driver/usart.h"

#define SWO_2MHZ 2000000

i32 main()
{
    // Red LED
    gpio_init_all_ports();
    gpio_init_output(GPIOI, 13, None, Low, PushPull);
    
    // STLINK Serial
    usart_use_hsi();
    usart_setup_basic(USART3, 115200);
    gpio_init_alt(GPIOB, 10, 7, None, Medium, PushPull);
    gpio_init_alt(GPIOB, 11, 7, None, Medium, PushPull);

    for (;;)
    {
        usart_send_string(USART3, "Hello, world!\r\n");
        char r = usart_recv(USART3);
        usart_send_string(USART3, "You sent: ");
        usart_send(USART3, r);
        usart_send_string(USART3, "\r\n");
    }
}


/**
 * SysTick
 */
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

// the very start
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
