#include "badhal.h"

// TODO: rewrite this, _mask functions get bad efficency; we could do popcnt(mask) times less writing and reading

u32 gpio_ref_to_index(struct GPIO_t *unit)
{
    return ((u32)unit & 0xffff) >> 10;
}

void gpio_init_all_ports()
{
    RCC->AHB4ENR |= RCC_AHB4ENR_GPIOxEN_Mask;
}

void gpio_init_port(struct GPIO_t *unit)
{
    RCC->AHB4ENR |= 1 << gpio_ref_to_index(unit);
}

void gpio_init_output(struct GPIO_t *unit, u8 index, enum GpioPull pull, enum GpioSpeed speed, enum GpioOutputType otype)
{
    unit->MODER = (unit->MODER & (u32) ~(0x3ul << (index * 2ul))) | (Output << (index * 2));
    unit->PUPDR = (unit->PUPDR & (u32) ~(0x3ul << (index * 2))) | (pull << (index * 2));
    unit->OSPEEDR = (unit->OSPEEDR & (u32) ~(0x3ul << (index * 2))) | (speed << (index * 2));
    unit->OTYPER = (unit->OTYPER & (u32) ~(1 << index)) | (otype << index);
}

void gpio_init_output_mask(struct GPIO_t *unit, u16 mask, enum GpioPull pull, enum GpioSpeed speed, enum GpioOutputType otype)
{
    for (u8 i = 0; i < 16; i++)
    {
        if ((1 << i) & mask)
        {
            gpio_init_output(unit, i, pull, speed, otype);
        }
    }
}

void gpio_init_input(struct GPIO_t *unit, u8 index, enum GpioPull pull)
{
    unit->MODER = (unit->MODER & (u32) ~(0x3ul << (index * 2))) | (Input << (index * 2));
    unit->PUPDR = (unit->PUPDR & (u32) ~(0x3ul << (index * 2))) | (pull << (index * 2));
}

void gpio_init_input_mask(struct GPIO_t *unit, u16 mask, enum GpioPull pull)
{
    for (u8 i = 0; i < 16; i++)
    {
        if ((1 << i) & mask)
        {
            gpio_init_input(unit, i, pull);
        }
    }
}

void gpio_init_alt(struct GPIO_t *unit, u8 index, u8 af, enum GpioPull pull, enum GpioSpeed speed, enum GpioOutputType otype)
{
    unit->MODER = (unit->MODER & (u32) ~(0x3ul << (index * 2))) | (Alt << (index * 2));
    unit->PUPDR = (unit->PUPDR & (u32) ~(0x3ul << (index * 2))) | (pull << (index * 2));
    unit->OTYPER = (unit->OTYPER & (u32) ~(1 << index)) | (otype << index);
    unit->OSPEEDR = (unit->OSPEEDR & (u32) ~(0x3ul << (index * 2))) | (speed << (index * 2));

    af &= 0xf;
    u32 afrIdx = (index >> 3) & 1;
    u32 inRegOffset = (index & 0x7) << 2;
    unit->AFR[afrIdx] = (unit->AFR[afrIdx] & (u32) ~(0xf << inRegOffset)) | (af << inRegOffset);
}

void gpio_init_alt_mask(struct GPIO_t *unit, u16 mask, u8 af, enum GpioPull pull, enum GpioSpeed speed, enum GpioOutputType otype)
{
    for (u8 i = 0; i < 16; i++)
    {
        if ((1 << i) & mask)
        {
            gpio_init_alt(unit, i, af, pull, speed, otype);
        }
    }
}

void gpio_init_analog(struct GPIO_t *unit, u8 index)
{
    unit->MODER = (unit->MODER & (u32) ~(0x3ul << (index * 2))) | (Analog << (index * 2));
}

void gpio_init_analog_mask(struct GPIO_t *unit, u16 mask)
{

    for (u8 i = 0; i < 16; i++)
    {
        if ((1 << i) & mask)
        {
            gpio_init_analog(unit, i);
        }
    }
}

bool gpio_read(struct GPIO_t *unit, u8 index)
{
    return unit->IDR & (1 << index);
}

bool gpio_out_state(struct GPIO_t *unit, u8 index)
{
    return unit->ODR & (1 << index);
}

void gpio_put(struct GPIO_t *unit, u8 index, bool value)
{
    if (value)
        unit->BSRR |= 1 << index;
    else
        unit->BSRR |= 1 << (index + 16);
}

void gpio_toggle(struct GPIO_t *unit, u8 index)
{
    gpio_put(unit, index, !gpio_out_state(unit, index));
}