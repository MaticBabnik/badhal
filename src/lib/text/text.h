#pragma once
#include <core/bad.h>

u8 paint_string(u8 *fb, u32 fb_width, u32 x, u32 y, u8 a, const char *str);

u8 paint_string_by_len(
    u8 *fb, u32 fb_width, u32 x, u32 y, u8 a, const char *str, u32 len
);