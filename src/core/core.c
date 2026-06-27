#include "bad.h"

void memset8(void *ptr, u8 value, u32 n) {
    u8 *p = (u8 *) ptr;

    for (u32 i = 0; i < n; i++) {
        p[i] = value;
    }
}

void memset16(void *ptr, u16 value, u32 n) {
    u16 *p = (u16 *) ptr;

    for (u32 i = 0; i < n; i++) {
        p[i] = value;
    }
}

void memset32(void *ptr, u32 value, u32 n) {
    u32 *p = (u32 *) ptr;

    for (u32 i = 0; i < n; i++) {
        p[i] = value;
    }
}