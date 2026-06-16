#include "hal/core/bad.h"

typedef struct {
    u32 stage;
    u32 *address;
    u32 expected;
    u32 actual;
} MT67_Error_t;

/**
 * Runs a set of memory tests on the region (zeros, ones, own address)...
 * Returns 1 on failure and populates the error struct with details, 0 on
 * success.
 */
u32 memtest67(u32 *base, u32 size, MT67_Error_t *err);