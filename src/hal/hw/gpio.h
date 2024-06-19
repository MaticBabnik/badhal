#pragma once

#include "../bad.h"

struct GPIO_t
{
    R_RW u32 MODER;
    R_RW u32 OTYPER;
    R_RW u32 OSPEEDR;
    R_RW u32 PUPDR;
    R_RW u32 IDR;
    R_RW u32 ODR;
    R_RW u32 BSRR;
    R_RW u32 LCKR;
    R_RW u32 AFRL;
    R_RW u32 AFRH;
};