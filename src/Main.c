#include "hal/badhal.h"
#include "hal/driver/gpio.h"
#include "hal/driver/debug.h"
#include "hal/driver/usart.h"
#include "hal/driver/clock.h"

#define STB_SPRINTF_IMPLEMENTATION
#define STB_SPRINTF_NOFLOAT
#include "ext/stb_sprintf.h"

#include "memtest67.h"

/*
 * Rotating RGB triangle — 480×272 RGB565 framebuffer, no stdlib.
 *
 * Usage:
 *   float angle = 0.0f;
 *   while (1) {
 *       draw_triangle_frame(angle);
 *       angle += 0.02f;            // tune to taste / vsync
 *       if (angle > K_2PI) angle -= K_2PI;
 *       // wait_vsync();
 *   }
 */


#define SCREEN_W   480
#define SCREEN_H   272
#define FRAME_BUF  ((u16 *)(SDRAM_BASE + 0x400000UL))

/* ── trig (no math.h) ──────────────────────────────────────── */

#define K_2PI  6.28318530f
#define K_PI2  1.57079632f   /* π/2 */

/* Simultaneous sin+cos, accurate to ~0.03% via quadrant-folded
   Taylor series evaluated only on [0, π/2].               */
static void sincos_f(f32 x, f32 *s, f32 *c) {
    /* reduce to [0, 2π) */
    i32 k = (i32)(x / K_2PI);
    x -= (f32)k * K_2PI;
    if (x < 0.0f) x += K_2PI;

    /* quadrant + remainder in [0, π/2) */
    i32 q = (i32)(x / K_PI2);
    if (q > 3) q = 3;
    f32 r  = x - (f32)q * K_PI2;
    f32 r2 = r * r;

    /* sin: 7th-order Taylor     cos: 6th-order Taylor */
    f32 sv = r * (1.0f - r2/6.0f   * (1.0f - r2/20.0f * (1.0f - r2/42.0f)));
    f32 cv = 1.0f - r2/2.0f * (1.0f - r2/12.0f * (1.0f - r2/30.0f));

    switch (q) {
        case 0: *s =  sv; *c =  cv; break;
        case 1: *s =  cv; *c = -sv; break;
        case 2: *s = -sv; *c = -cv; break;
        default:*s = -cv; *c =  sv; break;
    }
}

/* ── framebuffer helpers ─────────────────────────────────────── */

static void fb_clear(u16 *fb, u16 col) {
    u32 n = SCREEN_W * SCREEN_H;
    while (n--) *fb++ = col;
}

/* r, g, b in [0.0, 1.0] */
static u16 pack565(f32 r, f32 g, f32 b) {
    u8 ri = (u8)(r * 255.0f);
    u8 gi = (u8)(g * 255.0f);
    u8 bi = (u8)(b * 255.0f);
    return (u16)((ri >> 3) << 11 | (gi >> 2) << 5 | (bi >> 3));
}

/* ── rasterizer ──────────────────────────────────────────────── */

typedef struct { f32 x, y;    } V2;
typedef struct { f32 r, g, b; } Col;
typedef struct { V2 p; Col c; } Vtx;

static f32 edge_fn(V2 a, V2 b, V2 p) {
    return (b.x - a.x) * (p.y - a.y) - (b.y - a.y) * (p.x - a.x);
}

#define FMIN2(a, b)      ((a) < (b) ? (a) : (b))
#define FMAX2(a, b)      ((a) > (b) ? (a) : (b))
#define FMIN3(a, b, c)   FMIN2(a, FMIN2(b, c))
#define FMAX3(a, b, c)   FMAX2(a, FMAX2(b, c))
#define CLAMPI(v, lo, hi) ((v) < (lo) ? (lo) : ((v) > (hi) ? (hi) : (v)))

static void raster_tri(u16 *fb, Vtx v0, Vtx v1, Vtx v2) {
    /* ensure CCW winding so barycentric weights are +ve inside */
    f32 area = edge_fn(v0.p, v1.p, v2.p);
    if (area < 0.0f) { Vtx t = v1; v1 = v2; v2 = t; area = -area; }
    if (area < 0.5f) return;                        /* degenerate */
    f32 inv = 1.0f / area;

    /* integer pixel bounding box, clamped to screen */
    i32 x0 = CLAMPI((i32)FMIN3(v0.p.x, v1.p.x, v2.p.x),     0, SCREEN_W - 1);
    i32 x1 = CLAMPI((i32)FMAX3(v0.p.x, v1.p.x, v2.p.x) + 1, 0, SCREEN_W - 1);
    i32 y0 = CLAMPI((i32)FMIN3(v0.p.y, v1.p.y, v2.p.y),     0, SCREEN_H - 1);
    i32 y1 = CLAMPI((i32)FMAX3(v0.p.y, v1.p.y, v2.p.y) + 1, 0, SCREEN_H - 1);

    for (i32 y = y0; y <= y1; y++) {
        for (i32 x = x0; x <= x1; x++) {
            V2  p  = { x + 0.5f, y + 0.5f };
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

/* ── public entry point ──────────────────────────────────────── */

void draw_triangle_frame(f32 angle) {
    u16 *fb = FRAME_BUF;
    fb_clear(fb, 0x0000);

    const f32 cx = SCREEN_W * 0.5f;   /* 240.0 */
    const f32 cy = SCREEN_H * 0.5f;   /* 136.0 */
    const f32 R  = 110.0f;

    /* three vertices 120° (2π/3) apart */
    f32 s0, c0, s1, c1, s2, c2;
    sincos_f(angle,               &s0, &c0);
    sincos_f(angle + 2.09439510f, &s1, &c1);   /* + 2π/3 */
    sincos_f(angle + 4.18879020f, &s2, &c2);   /* + 4π/3 */

    Vtx tri[3] = {
        { { cx + R*c0, cy + R*s0 }, { 1.0f, 0.0f, 0.0f } },   /* red   */
        { { cx + R*c1, cy + R*s1 }, { 0.0f, 1.0f, 0.0f } },   /* green */
        { { cx + R*c2, cy + R*s2 }, { 0.0f, 0.0f, 1.0f } },   /* blue  */
    };

    raster_tri(fb, tri[0], tri[1], tri[2]);
}

// Crash handler impl
void _crash_puts(const char *str);
void _crash_printf(const char *fmt, ...);
#define CRASH_PUTS _crash_puts
#define CRASH_PRINTF _crash_printf
#define CRASH_IMPL
#include "hal/crash.h"

#define SWO_2MHZ 2000000

char printbuf[1024];

volatile u32 _crash_serial_ready = 0;

const u32 hsync = 10;
const u32 vsync = 10;
const u32 hbp = 43;
const u32 vbp = 12;
const u32 hfp = 8;
const u32 vfp = 4;
const u32 hactive = 480;
const u32 vactive = 272;


void ltdc_enable() {
    // misc LCD pins
    gpio_init_alt_mask(GPIOH, 0x0202, 14, GP_None, GS_VeryHigh, GO_PushPull);
    gpio_init_alt_mask(GPIOI, 0xd203, 14, GP_None, GS_VeryHigh, GO_PushPull);
    gpio_init_alt_mask(GPIOJ, 0xfffd, 14, GP_None, GS_VeryHigh, GO_PushPull);
    gpio_init_alt_mask(GPIOK, 0x00fd, 14, GP_None, GS_VeryHigh, GO_PushPull);

    // LCD disp
    gpio_init_output(GPIOD, 7, GP_None, GS_Low, GO_PushPull);
    gpio_put(GPIOD, 7, 1);

    // LCD backlight
    gpio_init_output(GPIOK, 0, GP_None, GS_Low, GO_PushPull);
    gpio_put(GPIOK, 0, 1);

    // enable LTDC
    RCC->APB3ENR |= RCC_APB3ENR_LTDCEN;

    LTDC->GCR = 0;

    LTDC->SSCR =
        ((hsync - 1) << LTDC_SSCR_HSW_Pos) | ((vsync - 1) << LTDC_SSCR_VSH_Pos);

    LTDC->BPCR = ((hsync + hbp - 1) << LTDC_BPCR_HSW_Pos)
                 | ((vsync + vbp - 1) << LTDC_BPCR_VSH_Pos);

    LTDC->AWCR = ((hsync + hbp + hactive - 1) << LTDC_AWCR_HSW_Pos)
                 | ((vsync + vbp + vactive - 1) << LTDC_AWCR_VSH_Pos);

    LTDC->TWCR = ((hsync + hbp + hactive + hfp - 1) << LTDC_TWCR_HSW_Pos)
                 | ((vsync + vbp + vactive + vfp - 1) << LTDC_TWCR_VSH_Pos);

    LTDC->BCCR = 0x00000000;

    LTDC_Layer1->CR = 0;

    LTDC_Layer1->WHPCR = ((hsync + hbp ) << LTDC_LxWHPCR_Start_Pos) 
                       | ((hsync + hbp + hactive - 1) << LTDC_LxWHPCR_Stop_Pos);


    LTDC_Layer1->WVPCR = ((vsync + vbp) << LTDC_LxWVPCR_Start_Pos) 
                       | ((vsync + vbp + vactive - 1) << LTDC_LxWVPCR_Stop_Pos);

    LTDC_Layer1->PFCR = LTDC_LxPFCR_RGB565;

    LTDC_Layer1->DCCR = 0x00000000;
    LTDC_Layer1->CACR = 0xff;
    LTDC_Layer1->BFCR = 0x607;

    LTDC_Layer1->CFBAR = SDRAM_BASE + 0x400'000;

    const u32 bpp = 2;

    LTDC_Layer1->CFBLR = ((hactive * bpp) << LTDC_LxCFBLR_CFBP_Pos) 
                       | ((hactive * bpp + 7) << LTDC_LxCFBLR_CFBLL_Pos);

    LTDC_Layer1->CFBLNR = vactive;

    LTDC_Layer1->CR = 1;

    LTDC->SRCR = 1;

    LTDC->LIPCR = vactive + vbp;
    LTDC->IER |= LTDC_IER_LIE;

    LTDC->GCR = 1;
}

volatile i32 frameSync = 0;

void LTDC_IRQHandler() {
    frameSync = 0;
    LTDC->ICR = LTDC_ICR_CLIF;
}

void printf(const char *fmt, ...) {
    va_list args;
    va_start(args, fmt);
    stbsp_vsnprintf(printbuf, 1024, fmt, args);
    va_end(args);
    usart_send_string(USART3, printbuf);
}

void _crash_printf(const char *fmt, ...) {
    if (!_crash_serial_ready) return;

    va_list args;
    va_start(args, fmt);
    stbsp_vsnprintf(printbuf, 1024, fmt, args);
    va_end(args);
    usart_send_string(USART3, printbuf);
}

void _crash_puts(const char *str) {
    if (_crash_serial_ready) usart_send_string(USART3, str);
}

void adc_init() {
    // clock ADCs from PER_CK
    RCC->D3CCIPR =
        (RCC->D3CCIPR & (~RCC_D3CCIPR_ADCSEL_Msk)) | RCC_D3CCIPR_ADCSEL_per_ck;

    // enable peripheral clocks
    RCC->AHB4ENR |= RCC_AHB4ENR_GPIOAEN;
    RCC->AHB1ENR |= RCC_AHB1ENR_ADC12EN;

    // set pin to analog
    gpio_init_analog(GPIOA, 0);

    // enable ADC1
    ADC1->CR &= ~(ADC_CR_DEEPPWD);
    ADC1->CR |= ADC_CR_ADVREGEN;
    ADC1->CR |= ADC_CR_ADCAL;

    // wait for calibration
    while (ADC1->CR & ADC_CR_ADCAL) {
    }

    // continious mode, allow overwrite, 10bit res and disable injected queue
    ADC1->CFGR =
        ADC_CFGR_JQDIS | ADC_CFGR_CONT | ADC_CFGR_OVRMOD | ADC_CFGR_RES_10bit;

    // setup sequence { (0, 16.5) }
    ADC1->SQR1 = /* sequence length */ 1 |
                 /* 1st channel */ (0ul << 6ul);
    ADC1->SMPR1 = /* sample length 1 */ (0x3ul << 0ul);

    // enable ADC
    ADC1->CR |= ADC_CR_ADEN;

    // wait for it to be ready
    while (!(ADC1->ISR & ADC_ISR_ADRDY)) {
    }
    ADC1->ISR = ADC_ISR_ADRDY;

    // start
    ADC1->CR |= ADC_CR_ADSTART;
}

const char *const cachetype_str[] = {
    "None", "WriteBack_RWAlloc", "WriteThrough_RAlloc", "WriteBack_RAlloc"
};

void run_memtests() {
    const u32 size = 8 * 1024 * 1024;

    MT67_Error_t err;

    printf("Running memtest67\r\n");
    printf("- Testing %u bytes\r\n\r\n", size);

    u32 nPass = 0;

    for (CacheType_t c = CT_None; c <= CT_WriteBack_RAlloc; c++) {
        printf("Testing ext memory w cache %s...", cachetype_str[c]);

        mem_mpu_setup_sdram(c);

        u32 res = memtest67((u32 *) SDRAM_BASE, size, &err);

        if (res) {
            printf(
                "\r\nmemtest67 failed on stage %d at address %p: expected "
                "0x%08X, got 0x%08X\r\n",
                err.stage, (void *) err.address, err.expected, err.actual
            );
        } else {
            printf("OK\r\n");
            nPass++;
        }
    }

    if (nPass == 4) {
        printf("\r\nAll tests passed!\r\n");
    }
}

i32 main() {

    // Red LED
    gpio_init_all_ports();
    gpio_init_output(GPIOI, 13, GP_None, GS_Low, GO_PushPull);

    // STLINK Serial
    usart_use_hsi();
    gpio_init_alt(GPIOB, 10, 7, GP_None, GS_Medium, GO_PushPull);
    gpio_init_alt(GPIOB, 11, 7, GP_None, GS_Medium, GO_PushPull);
    usart_setup_basic(USART3, 115200);
    _crash_serial_ready = 1;

    // usart_send_string(USART3, "Nihao fine shyt\r\n");
    // sys_delay_ms(10);
    // swo_writestr("Test...");
    // run_memtests();
    // swo_writestr("Test done!");

    ltdc_enable();

    u16 *ptr = (u16 *) (SDRAM_BASE + 0x400'000);

    for (u32 y = 0; y < vactive; y++) {
        for (u32 x = 0; x < hactive; x++) {
            *ptr++ = (x & 0x1f) << 11 | (y & 0x3f) << 5;
        }
    }
    
    f32 t = 0.0f;

    // ADC init
    // adc_init();

    usart_send_string(USART3, "Done!\r\n");

    for (;;) {
        // printf("Read %d\r\n", ADC1->DR);
        draw_triangle_frame(t);
        t += 0.01f;

        frameSync = 1;
        while (frameSync) {
            a_wfi();
        }
    }
}

// the very start
void entry() {
    // don't mess with ordering around here too much
    sys_earlyinit();
    sys_init_ext_mem();
    mem_mpu_setup_sdram(CT_WriteThrough_RAlloc); // mounts external RAM
    sys_icache_enable();
    sys_dcache_enable();
    sys_lateinit();     // (sets up 64MHz systick)
    sys_go_fast();      // (switches to PLL1@400MHz, fixes systick)
    swo_init(SWO_2MHZ); // enable SWO logging
    sys_allfaults();    // enable all fault exceptions

    main();
}

void onTick() {}