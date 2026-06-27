#pragma once
#include <core/bad.h>
#include "../hwdef/gpio.h"

#define GPIO_0 (1ul << 0)
#define GPIO_1 (1ul << 1)
#define GPIO_2 (1ul << 2)
#define GPIO_3 (1ul << 3)
#define GPIO_4 (1ul << 4)
#define GPIO_5 (1ul << 5)
#define GPIO_6 (1ul << 6)
#define GPIO_7 (1ul << 7)
#define GPIO_8 (1ul << 8)
#define GPIO_9 (1ul << 9)
#define GPIO_10 (1ul << 10)
#define GPIO_11 (1ul << 11)
#define GPIO_12 (1ul << 12)
#define GPIO_13 (1ul << 13)
#define GPIO_14 (1ul << 14)
#define GPIO_15 (1ul << 15)

#define GPIO_ALL 0xfffful

#define AF0 0ul
#define AF1 1ul
#define AF2 2ul
#define AF3 3ul
#define AF4 4ul
#define AF5 5ul
#define AF6 6ul
#define AF7 7ul
#define AF8 8ul
#define AF9 9ul
#define AF10 10ul
#define AF11 11ul
#define AF12 12ul
#define AF13 13ul
#define AF14 14ul
#define AF15 15ul

#define GPIO_MODE_MASK 0x3UL
#define GPIO_MODE_SIZE 2UL
#define GPIO_MODE_IN 0UL
#define GPIO_MODE_OUT 1UL
#define GPIO_MODE_ALT 2UL
#define GPIO_MODE_ANALOG 3UL

#define GPIO_OTYPE_MASK 1UL
#define GPIO_OTYPE_SIZE 1UL
#define GPIO_OTYPE_PP 0UL
#define GPIO_OTYPE_DRAIN 1UL

#define GPIO_SPEED_MASK 0x3UL
#define GPIO_SPEED_SIZE 2UL
#define GPIO_SPEED_LOW 0UL
#define GPIO_SPEED_MEDIUM 1UL
#define GPIO_SPEED_HIGH 2UL
#define GPIO_SPEED_VERY_HIGH 3UL

#define GPIO_PUPD_MASK 0x3UL
#define GPIO_PUPD_SIZE 2UL
#define GPIO_PUPD_NONE 0UL
#define GPIO_PUPD_PULLUP 1UL
#define GPIO_PUPD_PULLDOWN 2UL

#define GPIO_AF_MASK 0xFUL
#define GPIO_AF_SIZE 4

typedef enum {
    GM_Input = 0,
    GM_Output = 1,
    GM_Alt = 2,
    GM_Analog = 3
} GpioMode_t;

typedef enum { GO_PushPull = 0, GO_OpenDrain = 1 } GpioOutputType_t;

typedef enum {
    GS_Low = 0,
    GS_Medium = 1,
    GS_High = 2,
    GS_VeryHigh = 3
} GpioSpeed_t;

typedef enum { GP_None = 0, GP_PullUp = 1, GP_PullDown = 2 } GpioPull_t;

void gpio_init_all_ports();
void gpio_init_port(GPIO_t *unit);

void gpio_init_output(
    GPIO_t *unit,
    u8 index,
    GpioPull_t pull,
    GpioSpeed_t speed,
    GpioOutputType_t otype
);
void gpio_init_output_mask(
    GPIO_t *unit,
    u16 mask,
    GpioPull_t pull,
    GpioSpeed_t speed,
    GpioOutputType_t otype
);

void gpio_init_input(GPIO_t *unit, u8 index, GpioPull_t pull);
void gpio_init_input_mask(GPIO_t *unit, u16 mask, GpioPull_t pull);

void gpio_init_alt(
    GPIO_t *unit,
    u8 index,
    u8 af,
    GpioPull_t pull,
    GpioSpeed_t speed,
    GpioOutputType_t otype
);
void gpio_init_alt_mask(
    GPIO_t *unit,
    u16 mask,
    u8 af,
    GpioPull_t pull,
    GpioSpeed_t speed,
    GpioOutputType_t otype
);

void gpio_init_analog(GPIO_t *unit, u8 index);
void gpio_init_analog_mask(GPIO_t *unit, u16 mask);

bool gpio_read(GPIO_t *unit, u8 index);
bool gpio_out_state(GPIO_t *unit, u8 index);

void gpio_put(GPIO_t *unit, u8 index, bool value);
void gpio_toggle(GPIO_t *unit, u8 index);