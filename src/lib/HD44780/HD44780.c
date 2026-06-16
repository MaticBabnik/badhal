#include "HD44780.h"

void hd44780_put4(u8 v) {
    gpio_put(HD_D0, v & 1);
    gpio_put(HD_D1, v & 2);
    gpio_put(HD_D2, v & 4);
    gpio_put(HD_D3, v & 8);

    // TODO: more precise timings?

    gpio_put(HD_EN, 0);
    sys_delay_ms(1);
    gpio_put(HD_EN, 1);
    sys_delay_ms(1);
    gpio_put(HD_EN, 0);
    sys_delay_ms(1);
}

void hd44780_send(u8 v, u8 mode) {
    gpio_put(HD_RS, mode);

    hd44780_put4(v >> 4);
    hd44780_put4(v);
}

void hd44780_command(u8 v) {
    hd44780_send(v, 0);
}

void hd44780_write(u8 v) {
    hd44780_send(v, 1);
}

void hd44780_clear() {
    hd44780_command(HD_CLEARDISPLAY);
    sys_delay_ms(2);
}

void hd44790_puts(const char *str) {
    while (*str) {
        hd44780_write(*str);
        str++;
    }
}

void hd44780_init() {
    gpio_init_output(HD_D0, HD_OUTPUT);
    gpio_init_output(HD_D1, HD_OUTPUT);
    gpio_init_output(HD_D2, HD_OUTPUT);
    gpio_init_output(HD_D3, HD_OUTPUT);

    gpio_init_output(HD_RS, HD_OUTPUT);
    gpio_init_output(HD_EN, HD_OUTPUT);
    gpio_init_output(HD_BC, HD_OUTPUT);

    sys_delay_ms(50); // wait for the display to wakeup

    gpio_put(HD_RS, 0);
    gpio_put(HD_EN, 0);
    gpio_put(HD_BC, 1);

    for (u32 i = 0; i < 3; i++) {
        hd44780_put4(3);
        sys_delay_ms(5);
    }

    hd44780_put4(2); // switch to 4b mode
    hd44780_command(HD_FUNCTIONSET | HD_2LINE);
    sys_delay_ms(5);

    hd44780_command(HD_DISPLAYCONTROL | HD_DISPLAYON);
    hd44780_clear();
    hd44780_command(HD_ENTRYMODESET | HD_ENTRYLEFT | HD_ENTRYSHIFTDECREMENT);
}