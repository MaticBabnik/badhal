#include "ltdc.h"
#include "../badhal.h"

static u32 _hsync, _vsync, _hbp, _vbp, _hfp, _vfp, _hactive, _vactive;

struct {
    u32 width, height;
    u32 bpp;
} _layers[2];

void ltdc_enable() {
    LTDC->GCR |= LTD_GCR_LTDCEN;
}

void ltdc_disable() {
    LTDC->GCR &= ~LTD_GCR_LTDCEN;
}

void ltdc_set_polarity(
    LTDC_Polarity_t hsync_pol,
    LTDC_Polarity_t vsync_pol,
    LTDC_Polarity_t de_pol,
    LTDC_Polarity_t pclk_pol
) {
    LTDC->GCR =
        (hsync_pol << LTDC_GCR_HSPOL_Pos) | (vsync_pol << LTDC_GCR_VSPOL_Pos)
        | (de_pol << LTDC_GCR_DEPOL_Pos) | (pclk_pol << LTDC_GCR_PCPOL_Pos);
}

void ltdc_set_size(
    u32 hsync,
    u32 vsync,
    u32 hbp,
    u32 vbp,
    u32 hfp,
    u32 vfp,
    u32 hactive,
    u32 vactive
) {
    // store these for layers
    _hsync = hsync;
    _vsync = vsync;
    _hbp = hbp;
    _vbp = vbp;
    _hfp = hfp;
    _vfp = vfp;
    _hactive = hactive;
    _vactive = vactive;

    LTDC->SSCR =
        ((hsync - 1) << LTDC_SSCR_HSW_Pos) | ((vsync - 1) << LTDC_SSCR_VSH_Pos);

    LTDC->BPCR = ((hsync + hbp - 1) << LTDC_BPCR_HSW_Pos)
                 | ((vsync + vbp - 1) << LTDC_BPCR_VSH_Pos);

    LTDC->AWCR = ((hsync + hbp + hactive - 1) << LTDC_AWCR_HSW_Pos)
                 | ((vsync + vbp + vactive - 1) << LTDC_AWCR_VSH_Pos);

    LTDC->TWCR = ((hsync + hbp + hactive + hfp - 1) << LTDC_TWCR_HSW_Pos)
                 | ((vsync + vbp + vactive + vfp - 1) << LTDC_TWCR_VSH_Pos);
}

void ltdc_set_background_color(u32 color) {
    LTDC->BCCR = color;
}

void ltdc_set_background_color_components(u8 r, u8 g, u8 b) {
    LTDC->BCCR = (r << 16) | (g << 8) | b;
}

void ltdc_enable_dither(u32 wRed, u32 wGreen, u32 wBlue) {
    wRed &= LTDC_GCR_DxW_UMask;
    wGreen &= LTDC_GCR_DxW_UMask;
    wBlue &= LTDC_GCR_DxW_UMask;

    // DEN doesn't need to be masked because we're always setting it to 1
    mreg(
        &LTDC->GCR, LTDC_GCR_DRW_Mask | LTDC_GCR_DGW_Mask | LTDC_GCR_DBW_Mask,
        (wRed << LTDC_GCR_DRW_Pos) | (wGreen << LTDC_GCR_DGW_Pos)
            | (wBlue << LTDC_GCR_DBW_Pos) | LTDC_GCR_DEN

    );
}

void ltdc_disable_dither() {
    LTDC->GCR &= ~LTDC_GCR_DEN;
}

void ltdc_reload_vblank() {
    LTDC->SRCR = LTDC_SRCR_VBR;
}

void ltdc_reload_now() {
    LTDC->SRCR = LTDC_SRCR_IMR;
}

void ltdc_set_line_interrupt(u32 line) {
    LTDC->LIPCR = line;
}

void ltdc_set_line_interrupt_active_end() {
    LTDC->LIPCR = _vsync + _vbp + _vactive; // TODO: does it need to be +1?
}

void ltdc_layer_enable(LTDC_Layer_t *layer) {
    layer->CR |= LTDC_LxCR_LEN;
}

void ltdc_layer_disable(LTDC_Layer_t *layer) {
    layer->CR &= ~LTDC_LxCR_LEN;
}

void ltdc_layer_enable_color_keying(LTDC_Layer_t *layer, u32 color) {
    layer->CKCR = color;
    layer->CR |= LTDC_LxCR_COLKEN;
}

void ltdc_layer_disable_color_keying(LTDC_Layer_t *layer) {
    layer->CR &= ~LTDC_LxCR_COLKEN;
}

void ltdc_layer_enable_clut(LTDC_Layer_t *layer) {
    layer->CR |= LTDC_LxCR_CLUTEN;
}

void ltdc_layer_disable_clut(LTDC_Layer_t *layer) {
    layer->CR &= ~LTDC_LxCR_CLUTEN;
}

void ltdc_layer_clut_set(LTDC_Layer_t *layer, u8 index, u32 color) {
    layer->CLUTWR = (index << 24) | (color & 0xFFFFFF);
}

void ltdc_layer_clut_set_components(
    LTDC_Layer_t *layer, u8 index, u8 r, u8 g, u8 b
) {
    layer->CLUTWR = (index << 24) | (r << 16) | (g << 8) | b;
}

void ltdc_layer_setup_fullscreen(LTDC_Layer_t *layer) {
    _layers[layer == LTDC_Layer1 ? 0 : 1].width = _hactive;
    _layers[layer == LTDC_Layer1 ? 0 : 1].height = _vactive;

    layer->WHPCR = ((_hsync + _hbp) << LTDC_LxWHPCR_Start_Pos)
                   | ((_hsync + _hbp + _hactive - 1) << LTDC_LxWHPCR_Stop_Pos);

    layer->WVPCR = ((_vsync + _vbp) << LTDC_LxWVPCR_Start_Pos)
                   | ((_vsync + _vbp + _vactive - 1) << LTDC_LxWVPCR_Stop_Pos);
}

void ltdc_layer_setup_windowed(
    LTDC_Layer_t *layer, u32 x, u32 y, u32 w, u32 h
) {
    _layers[layer == LTDC_Layer1 ? 0 : 1].width = w;
    _layers[layer == LTDC_Layer1 ? 0 : 1].height = h;

    layer->WHPCR = ((_hsync + _hbp + x) << LTDC_LxWHPCR_Start_Pos)
                   | ((_hsync + _hbp + x + w - 1) << LTDC_LxWHPCR_Stop_Pos);

    layer->WVPCR = ((_vsync + _vbp + y) << LTDC_LxWVPCR_Start_Pos)
                   | ((_vsync + _vbp + y + h - 1) << LTDC_LxWVPCR_Stop_Pos);
}

static u32 _pxfmt_to_bpp(LTDC_PixelFormat_t format) {
    switch (format) {
    case LPIXF_ARGB8888:
        return 4;
    case LPIXF_RGB888:
        return 3;
    case LPIXF_RGB565:
    case LPIXF_ARGB1555:
    case LPIXF_ARGB4444:
    case LPIXF_AL88:
        return 2;
    case LPIXF_L8:
    case LPIXF_AL44:
        return 1;
    }

    ASSERT(0, "wtf");

    return 1;
}

void ltdc_layer_set_pixel_format(
    LTDC_Layer_t *layer, LTDC_PixelFormat_t format
) {
    _layers[layer == LTDC_Layer1 ? 0 : 1].bpp = _pxfmt_to_bpp(format);
    layer->PFCR = format;
}

void ltdc_layer_setup_buffer_auto(LTDC_Layer_t *layer, u32 address) {
    u32 _l = layer == LTDC_Layer1 ? 0 : 1;
    u32 bpp = _layers[_l].bpp;
    u32 width = _layers[_l].width;
    u32 height = _layers[_l].height;

    layer->CFBLR = ((bpp * width + 7) << LTDC_LxCFBLR_CFBLL_Pos)
                   | ((bpp * width) << LTDC_LxCFBLR_CFBP_Pos);

    layer->CFBLNR = height;

    layer->CFBAR = address;
}

void ltdc_layer_swap_buffer(LTDC_Layer_t *layer, u32 address) {
    layer->CFBAR = address;
}

void ltdc_layer_set_alpha(LTDC_Layer_t *layer, u8 alpha) {
    layer->CACR = alpha;
}

void ltdc_layer_set_default_color(LTDC_Layer_t *layer, u32 color) {
    layer->DCCR = color;
}

void ltdc_layer_set_default_color_components(
    LTDC_Layer_t *layer, u8 a, u8 r, u8 g, u8 b
) {
    layer->DCCR = (a << 24) | (r << 16) | (g << 8) | b;
}

void ltdc_layer_set_blending(
    LTDC_Layer_t *layer, LTDC_BlendingFactor_t bf1, LTDC_BlendingFactor_t bf2
) {
    layer->BFCR = (bf1 << LTDC_LxBFCR_BF1_Pos) | (bf2 << LTDC_LxBFCR_BF2_Pos);
}

void ltdc_enable_interrupts(u32 mask) {
    LTDC->IER |= mask;
}

void ltdc_disable_interrupts(u32 mask) {
    LTDC->IER &= ~mask;
}

u32 ltdc_get_interrupts() {
    return LTDC->ISR;
}

void ltdc_clear_interrupts(u32 mask) {
    LTDC->ICR = mask;
}