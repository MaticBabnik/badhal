/*
    Compiler helpers
*/

#pragma once

#define asm __asm__

#define R_RW volatile       // read-write
#define R_WO volatile       // write-only
#define R_RO volatile const // read-only

#define INLINE_ALWAYS static inline __attribute__((always_inline))
#define INLINE_NEVER __attribute__((noinline))

typedef void (*isr_t)(void);