#include "hal/badhal.h"
#include "hal/debug.h"

#include "HD44780.h"

#define SWO_2MHZ 2000000

int main()
{
    gpio_init_all_ports();
    gpio_init_output(GPIOI, 13, None, Low, PushPull);
    gpio_init_output(GPIOJ, 2, None, Low, PushPull);
    gpio_init_output(GPIOD, 3, None, Low, PushPull);
    gpio_init_input(GPIOC, 13, None);

    hd44780_init();
    hd44790_puts("Hello, world!");

    for (;;)
    {
        gpio_toggle(GPIOI, 13);
        gpio_toggle(GPIOJ, 2);
        gpio_put(GPIOD, 3, gpio_read(GPIOC, 13));
        swo_writestr("Hello SWO.\n");
        sys_delay_ms(100);
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
