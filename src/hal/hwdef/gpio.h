#pragma once

#include <core/bad.h>

typedef struct {
    R_RW u32 MODER;   // Mode register
    R_RW u32 OTYPER;  // Output type
    R_RW u32 OSPEEDR; // Output spped
    R_RW u32 PUPDR;   // Pull up/down
    R_RW u32 IDR;     // Input Data
    R_RW u32 ODR;     // Output Data
    R_RW u32 BSRR;    // Bit Set/Reset
    R_RW u32 LCKR;    // Lock
    R_RW u32 AFR[2];  // Alternate function
} GPIO_t;
