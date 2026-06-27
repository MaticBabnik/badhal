#pragma once

#include <hal/badhal.h>
#include <core/math.h>

#define SCREEN_W 480
#define SCREEN_H 272

static void fb_clear(u16 *fb, u16 col) {
    u32 n = SCREEN_W * SCREEN_H;
    while (n--)
        *fb++ = col;
}

static u16 pack565(f32 r, f32 g, f32 b) {
    u8 ri = (u8) (r * 255.0f);
    u8 gi = (u8) (g * 255.0f);
    u8 bi = (u8) (b * 255.0f);
    return (u16) ((ri >> 3) << 11 | (gi >> 2) << 5 | (bi >> 3));
}

typedef struct {
    f32 x, y;
} V2;
typedef struct {
    f32 r, g, b;
} Col;
typedef struct {
    V2 p;
    Col c;
} Vtx;

static f32 edge_fn(V2 a, V2 b, V2 p) {
    return (b.x - a.x) * (p.y - a.y) - (b.y - a.y) * (p.x - a.x);
}

static void raster_tri(u16 *fb, Vtx v0, Vtx v1, Vtx v2) {
    /* ensure CCW winding so barycentric weights are +ve inside */
    f32 area = edge_fn(v0.p, v1.p, v2.p);
    if (area < 0.0f) {
        Vtx t = v1;
        v1 = v2;
        v2 = t;
        area = -area;
    }
    if (area < 0.5f) return; /* degenerate */
    f32 inv = 1.0f / area;

    /* integer pixel bounding box, clamped to screen */
    i32 x0 = CLAMPI((i32) FMIN3(v0.p.x, v1.p.x, v2.p.x), 0, SCREEN_W - 1);
    i32 x1 = CLAMPI((i32) FMAX3(v0.p.x, v1.p.x, v2.p.x) + 1, 0, SCREEN_W - 1);
    i32 y0 = CLAMPI((i32) FMIN3(v0.p.y, v1.p.y, v2.p.y), 0, SCREEN_H - 1);
    i32 y1 = CLAMPI((i32) FMAX3(v0.p.y, v1.p.y, v2.p.y) + 1, 0, SCREEN_H - 1);

    for (i32 y = y0; y <= y1; y++) {
        for (i32 x = x0; x <= x1; x++) {
            V2 p = {x + 0.5f, y + 0.5f};
            f32 w0 = edge_fn(v1.p, v2.p, p) * inv;
            f32 w1 = edge_fn(v2.p, v0.p, p) * inv;
            f32 w2 = edge_fn(v0.p, v1.p, p) * inv;
            if (w0 < 0.0f || w1 < 0.0f || w2 < 0.0f) continue;

            fb[y * SCREEN_W + x] = pack565(
                w0 * v0.c.r + w1 * v1.c.r + w2 * v2.c.r,
                w0 * v0.c.g + w1 * v1.c.g + w2 * v2.c.g,
                w0 * v0.c.b + w1 * v1.c.b + w2 * v2.c.b
            );
        }
    }
}

void draw_triangle_frame(u16 *fb, f32 angle) {
    fb_clear(fb, 0x0000);

    const f32 cx = SCREEN_W * 0.5f; /* 240.0 */
    const f32 cy = SCREEN_H * 0.5f; /* 136.0 */
    const f32 R = 110.0f;

    /* three vertices 120° (2π/3) apart */
    f32 s0, c0, s1, c1, s2, c2;
    sincos_f(angle, &s0, &c0);
    sincos_f(angle + 2.09439510f, &s1, &c1); /* + 2π/3 */
    sincos_f(angle + 4.18879020f, &s2, &c2); /* + 4π/3 */

    Vtx tri[3] = {
        {{cx + R * c0, cy + R * s0}, {1.0f, 0.0f, 0.0f}}, /* red   */
        {{cx + R * c1, cy + R * s1}, {0.0f, 1.0f, 0.0f}}, /* green */
        {{cx + R * c2, cy + R * s2}, {0.0f, 0.0f, 1.0f}}, /* blue  */
    };

    raster_tri(fb, tri[0], tri[1], tri[2]);
}