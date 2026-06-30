#include "text.h"
#include "font.h"
#include <core/stdbad/stddef.h>

const u8 *get_char(char c) {
    if (c < 33 || c > 127) {
        return NULL;
    }

    return &font[(c - 33) * 8];
}

u8 paint_string(u8 *fb, u32 fb_width, u32 x, u32 y, u8 a, const char *str) {
    while (*str) {
        const u8 *chr = get_char(*str);

        if (chr == NULL) {
            x += 6;
            str++;
            continue;
        }

        for (u32 i = 0; i < 8; i++) {
            for (u32 j = 0; j < 5; j++) {
                if (chr[i] & (1 << (7 - j))) {
                    fb[(y + i) * fb_width + (x + j)] = a;
                }
            }
        }

        x += 6;
        str++;
    }

    return 1;
}

u8 paint_string_by_len(
    u8 *fb, u32 fb_width, u32 x, u32 y, u8 a, const char *str, u32 len
) {
    for (u32 k = 0; k < len; k++) {
        const u8 *chr = get_char(str[k]);

        if (chr == NULL) {
            x += 6;
            continue;
        }

        for (u32 i = 0; i < 8; i++) {
            for (u32 j = 0; j < 5; j++) {
                if (chr[i] & (1 << (7 - j))) {
                    fb[(y + i) * fb_width + (x + j)] = a;
                }
            }
        }

        x += 6;
    }

    return 1;
}