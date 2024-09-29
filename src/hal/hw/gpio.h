#pragma once

#include "../bad.h"

struct GPIO_t
{
    R_RW u32 MODER;   // Mode register
    R_RW u32 OTYPER;  // Output type
    R_RW u32 OSPEEDR; // Output spped
    R_RW u32 PUPDR;   // Pull up/down
    R_RW u32 IDR;     // Input Data
    R_RW u32 ODR;     // Output Data
    R_RW u32 BSRR;    // Bit Set/Reset
    R_RW u32 LCKR;    // Lock
    R_RW u32 AFR[2];  // Alternate function
};

#define GPIO_0 (1 << 0)
#define GPIO_1 (1 << 1)
#define GPIO_2 (1 << 2)
#define GPIO_3 (1 << 3)
#define GPIO_4 (1 << 4)
#define GPIO_5 (1 << 5)
#define GPIO_6 (1 << 6)
#define GPIO_7 (1 << 7)
#define GPIO_8 (1 << 8)
#define GPIO_9 (1 << 9)
#define GPIO_10 (1 << 10)
#define GPIO_11 (1 << 11)
#define GPIO_12 (1 << 12)
#define GPIO_13 (1 << 13)
#define GPIO_14 (1 << 14)
#define GPIO_15 (1 << 15)

#define GPIO_ALL 0xffff

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

enum GpioMode
{
    Input = 0,
    Output = 1,
    Alt = 2,
    Analog = 3
};

enum GpioOutputType
{
    PushPull = 0,
    OpenDrain = 1
};

enum GpioSpeed
{
    Low = 0,
    Medium = 1,
    High = 2,
    VeryHigh = 3
};

enum GpioPull
{
    None = 0,
    PullUp = 1,
    PullDown = 2,
};

void gpio_init_all_ports();
void gpio_init_port(struct GPIO_t *unit);

void gpio_init_output(struct GPIO_t *unit, u8 index, enum GpioPull pull, enum GpioSpeed speed, enum GpioOutputType otype);
void gpio_init_output_mask(struct GPIO_t *unit, u16 mask, enum GpioPull pull, enum GpioSpeed speed, enum GpioOutputType otype);

void gpio_init_input(struct GPIO_t *unit, u8 index, enum GpioPull pull);
void gpio_init_input_mask(struct GPIO_t *unit, u16 mask, enum GpioPull pull);

void gpio_init_alt(struct GPIO_t *unit, u8 index, u8 af, enum GpioPull pull, enum GpioSpeed speed, enum GpioOutputType otype);
void gpio_init_alt_mask(struct GPIO_t *unit, u16 mask, u8 af, enum GpioPull pull, enum GpioSpeed speed, enum GpioOutputType otype);

void gpio_init_analog(struct GPIO_t *unit, u8 index);
void gpio_init_analog_mask(struct GPIO_t *unit, u16 mask);

bool gpio_read(struct GPIO_t *unit, u8 index);
bool gpio_out_state(struct GPIO_t *unit, u8 index);

void gpio_put(struct GPIO_t *unit, u8 index, bool value);
void gpio_toggle(struct GPIO_t *unit, u8 index);