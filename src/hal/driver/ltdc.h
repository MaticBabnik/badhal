#include <core/bad.h>
#include "../hwdef/ltdc.h"

typedef enum {
    LPOL_ActiveLow = 0,
    LPOL_ActiveHigh = 1,
} LTDC_Polarity_t;

typedef enum {
    LPIXF_ARGB8888 = 0,
    LPIXF_RGB888 = 1,
    LPIXF_RGB565 = 2,
    LPIXF_ARGB1555 = 3,
    LPIXF_ARGB4444 = 4,
    LPIXF_L8 = 5,
    LPIXF_AL44 = 6,
    LPIXF_AL88 = 7,
} LTDC_PixelFormat_t;

typedef enum {
    LBF_Const = 0b100,
    LBF_PixXConst = 0b110,
} LTDC_BlendingFactor_t;

void ltdc_set_polarity(
    LTDC_Polarity_t hsync_pol,
    LTDC_Polarity_t vsync_pol,
    LTDC_Polarity_t de_pol,
    LTDC_Polarity_t pclk_pol
);

void ltdc_set_size(
    u32 hsync,
    u32 vsync,
    u32 hbp,
    u32 vbp,
    u32 hfp,
    u32 vfp,
    u32 hactive,
    u32 vactive
);

void ltdc_set_background_color(u32 color);

void ltdc_set_background_color_components(u8 r, u8 g, u8 b);

void ltdc_enable_dither(u32 wRed, u32 wGreen, u32 wBlue);

void ltdc_disable_dither();

void ltdc_enable();

void ltdc_disable();

void ltdc_reload_vblank();

void ltdc_reload_now();

// TODO: should we expose interrupts through the driver?
// I really don't want to call nvic from ltdc

void ltdc_set_line_interrupt(u32 line);

void ltdc_set_line_interrupt_active_end();

void ltdc_layer_enable(LTDC_Layer_t *layer);

void ltdc_layer_disable(LTDC_Layer_t *layer);

void ltdc_layer_enable_color_keying(LTDC_Layer_t *layer, u32 color);

void ltdc_layer_disable_color_keying(LTDC_Layer_t *layer);

void ltdc_layer_enable_clut(LTDC_Layer_t *layer);

void ltdc_layer_disable_clut(LTDC_Layer_t *layer);

void ltdc_layer_clut_set(LTDC_Layer_t *layer, u8 index, u32 color);

void ltdc_layer_clut_set_components(
    LTDC_Layer_t *layer, u8 index, u8 r, u8 g, u8 b
);

void ltdc_layer_setup_fullscreen(LTDC_Layer_t *layer);

void ltdc_layer_setup_windowed(LTDC_Layer_t *layer, u32 x, u32 y, u32 w, u32 h);

void ltdc_layer_set_pixel_format(
    LTDC_Layer_t *layer, LTDC_PixelFormat_t format
);

void ltdc_layer_setup_buffer_auto(LTDC_Layer_t *layer, u32 address);

// TODO: do we need a manual setup for "aligned" row strides?

void ltdc_layer_swap_buffer(LTDC_Layer_t *layer, u32 address);

void ltdc_layer_set_alpha(LTDC_Layer_t *layer, u8 alpha);

void ltdc_layer_set_default_color(LTDC_Layer_t *layer, u32 color);

void ltdc_layer_set_default_color_components(
    LTDC_Layer_t *layer, u8 a, u8 r, u8 g, u8 b
);

void ltdc_layer_set_blending(
    LTDC_Layer_t *layer, LTDC_BlendingFactor_t bf1, LTDC_BlendingFactor_t bf2
);

#define LTDC_INT_RELOAD (1ul << 3)
#define LTDC_INT_TRANS_ERR (1ul << 2)
#define LTDC_INT_UNDERRUN (1ul << 1)
#define LTDC_INT_LINE (1ul << 0)

void ltdc_enable_interrupts(u32 mask);

void ltdc_disable_interrupts(u32 mask);

u32 ltdc_get_interrupts();

void ltdc_clear_interrupts(u32 mask);