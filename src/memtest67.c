#include "memtest67.h"

u32 memtest67(u32 *base, u32 size, MT67_Error_t *err) {
    u32 *end = (u32 *) ((u32) base + size);

    // write all zeros
    for (u32 *ptr = base; ptr < end; ptr++) {
        *ptr = 0;
    }

    // verify all zeros
    for (u32 *ptr = base; ptr < end; ptr++) {
        if (*ptr != 0) {
            err->stage = 0;
            err->address = ptr;
            err->expected = 0;
            err->actual = *ptr;
            return 1;
        }
    }

    // write all ones
    for (u32 *ptr = base; ptr < end; ptr++) {
        *ptr = 0xFFFFFFFF;
    }

    // verify all ones
    for (u32 *ptr = base; ptr < end; ptr++) {
        if (*ptr != 0xFFFFFFFF) {
            err->stage = 1;
            err->address = ptr;
            err->expected = 0xFFFFFFFF;
            err->actual = *ptr;
            return 1;
        }
    }

    // write address
    for (u32 *ptr = base; ptr < end; ptr++) {
        *ptr = (u32) ptr;
    }

    // check for aliasing
    for (u32 *ptr = base; ptr < end; ptr++) {
        if (*ptr != (u32) ptr) {
            err->stage = 2;
            err->address = ptr;
            err->expected = (u32) ptr;
            err->actual = *ptr;
            return 1;
        }
    }

    return 0;
}