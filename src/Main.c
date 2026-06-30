#include "hal/badhal.h"
#include "hal/driver/nvic.h"
#include "hal/driver/gpio.h"
#include "hal/driver/usart.h"
#include "hal/driver/clock.h"
#include "hal/driver/ltdc.h"
#include "hal/driver/mpu.h"
#include "hal/driver/qspi.h"

#include "bsp/bsp.h"

#define BADFS_STDIO_COMPAT
#include "lib/badfs/badfs.h"
#include "lib/sprintf/sprintf.h"
#include "lib/text/text.h"

static volatile u32 _crash_serial_ready = 0;
static char printbuf[1024];
void _crash_puts(const char *str);
void _crash_printf(const char *fmt, ...);
#define CRASH_PUTS _crash_puts
#define CRASH_PRINTF _crash_printf
#define CRASH_IMPL
#include "bsp/crash.h"

#include "loader/loader.h"

// SDRAM framebuffer base address; uncached
#define SDRAMF_BASE 0xD0000000
// SDRAM cached base address
#define SDRAMC_BASE 0xD0400000
// QSPI flash base address
#define QSPI_BASE 0x90000000

#define FB16_SIZE (2 * 480 * 272)
#define FB8_SIZE (1 * 480 * 272)

static u16 *const l1fb1 = (u16 *) (SDRAMF_BASE);
static u16 *const l1fb2 = (u16 *) (SDRAMF_BASE + FB16_SIZE);
static u8 *const l2fb1 = (u8 *) (SDRAMF_BASE + 2 * FB16_SIZE);
static u16 *volatile l1draw = l1fb1;
static u16 *volatile l1disp = l1fb2;
volatile i32 ltdcSwap = 0;

static void adc_joy_read(u16 *x, u16 *y);

u8 qspi_jedec_ids[2][3];

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

static void paint_clock() {
    stbsp_snprintf(printbuf, 1024, "Time: %10u ticks", sys_get_tick());

    for (u32 y = 10; y < 18; y++) {
        for (u32 x = 340; x < 480; x++) {
            l2fb1[y * 480 + x] = 0;
        }
    }

    paint_string(l2fb1, 480, 346, 10, 0xff, printbuf);
}

static void paint_joy(u16 jx, u16 jy) {
    const u32 start = 96;
    const u32 end = 232;

    for (u32 y = start; y < end; y++) {
        for (u32 x = start; x < end; x++) {
            l2fb1[y * 480 + x] = 0x88;
        }
    }

    // scale values down to [0,127]
    jx >>= 3;
    jy >>= 3;

    u32 cx = 100 + jx;
    u32 cy = 100 + jy;

    u32 sx = cx - 4;
    u32 ex = cx + 4;
    u32 sy = cy - 4;
    u32 ey = cy + 4;

    for (u32 y = sy; y < ey; y++) {
        for (u32 x = sx; x < ex; x++) {
            l2fb1[y * 480 + x] = 0xff;
        }
    }
}

static void qspi_hexdump() {
    // QSPI hexdump
    u8 *addr = (u8 *) 0x90000000;
    for (u32 y = 40; y < 260; y += 10) {
        // print offset (4 digits), 16 bytes hex, 16 bytes ascii

        // lowkey crime
        stbsp_snprintf(
            printbuf, 1024,
            "%04X: %02X %02X %02X %02X %02X %02X %02X %02X "
            "%02X %02X %02X %02X %02X %02X %02X %02X |",
            ((u32) addr) & 0xffff, addr[0], addr[1], addr[2], addr[3], addr[4],
            addr[5], addr[6], addr[7], addr[8], addr[9], addr[10], addr[11],
            addr[12], addr[13], addr[14], addr[15]
        );

        paint_string(l2fb1, 480, 10, y, 0xff, printbuf);
        paint_string_by_len(l2fb1, 480, 340, y, 0xff, addr, 16);
        addr += 16;
    }
}

static void badfs_listing() {
    BadFsFileInfo_t info;
    u32 nfiles = badfs_nfiles();

    for (u32 i = 0; i < nfiles; i++) {
        if (badfs_filedesc(i, &info) != BADFS_ERR_OK) {
            continue;
        }

        stbsp_snprintf(
            printbuf, 1024, "%s (%u KiB)", info.path, info.size >> 10
        );
        paint_string(l2fb1, 480, 10, 40 + i * 10, 0xff, printbuf);
    }
}

static void
draw_bmp(const char *restrict file, u16 *restrict fb, u32 x, u32 y) {
    BadFILE *f = badfs_open(file, "r");
    if (f == NULL) {
        return;
    }

    u8 buf[480 * 3];
    badfs_read(buf, 1, 54, f);

    u32 width = (*(u16 *) &buf[18]) | ((*(u16 *) &buf[20]) << 16);
    u32 height = (*(u16 *) &buf[22]) | ((*(u16 *) &buf[24]) << 16);
    u16 bpp = *(u16 *) &buf[28];
    u32 pixDataOff = (*(u16 *) &buf[10]) | ((*(u16 *) &buf[12]) << 16);

    if (bpp != 24) {
        badfs_close(f);
        return;
    }

    if (width > 480 || height > 272) {
        badfs_close(f);
        return;
    }

    u32 stride = (width * 3 + 3) & ~3; // BMP rows are padded to 4 bytes
    fseek(f, pixDataOff, SEEK_SET);

    for (u32 row = 0; row < height; row++) {
        badfs_read(buf, 1, stride, f);
        for (u32 col = 0; col < width; col++) {
            u8 b = buf[col * 3];
            u8 g = buf[col * 3 + 1];
            u8 r = buf[col * 3 + 2];

            fb[(y + (height - 1 - row)) * 480 + (x + col)] =
                ((r >> 3) << 11) | ((g >> 2) << 5) | (b >> 3);
        }
    }

    badfs_close(f);
}

static void adc_joy_read(u16 *x, u16 *y) {
    ADC1->CR |= ADC_CR_JADSTART;
    while (!(ADC1->ISR & ADC_ISR_JEOS)) {
    }
    ADC1->ISR = ADC_ISR_JEOS;
    *x = 1023 - (u16) ADC1->JDR1;
    *y = (u16) ADC1->JDR2;
}

static i32 main() {
    badfs_mount((void *) QSPI_BASE);

    usart_send_string(USART3, "\x1b[0mBadHAL\r\n-------------\r\n");
    usart_send_string(USART3, "Hello, World!\r\n");

    paint_string(l2fb1, 480, 10, 10, 0xff, "BadHAL");
    paint_string(l2fb1, 480, 10, 20, 0xff, "-------------");
    paint_string(l2fb1, 480, 10, 30, 0xff, "BadFS:");

    badfs_listing();

    // double the buffer; double the painting (~~not double the fun~~ it is
    // double the fun)
    draw_bmp("/fih.bmp", l1fb1, 0, 0);
    draw_bmp("/badhal.bmp", l1fb1, 100, 100);
    draw_bmp("/drool.bmp", l1fb2, 0, 0);

    u16 x = 512, y = 512;

    bool now, prev = false;

    for (;;) {
        adc_joy_read(&x, &y);
        paint_clock();
        // paint_joy(x, y);

        // holy double buffering
        now = gpio_read(GPIOC, 13);
        if (now && !prev) {
            ltdcSwap = 1;
            while (ltdcSwap) {
                // a_wfi();
            }
        }
        prev = now;
    }
}

static void qspi_bootloader() {
    char buf[128];
    paint_string(l2fb1, 480, 10, 10, 0xff, "BadHAL QSPI Bootloader");

    paint_string(l2fb1, 480, 10, 30, 0xff, "Anoterh day naother odla");
    paint_string(l2fb1, 480, 10, 40, 0xff, "r");

    paint_string(l2fb1, 480, 10, 60, 0xff, "JEDEC IDs:");

    stbsp_snprintf(
        buf, 128, "%02X %02X %02X", qspi_jedec_ids[0][0], qspi_jedec_ids[0][1],
        qspi_jedec_ids[0][2]
    );
    paint_string(l2fb1, 480, 20, 70, 0xff, buf);
    stbsp_snprintf(
        buf, 128, "%02X %02X %02X", qspi_jedec_ids[1][0], qspi_jedec_ids[1][1],
        qspi_jedec_ids[1][2]
    );
    paint_string(l2fb1, 480, 20, 80, 0xff, buf);

    loader_main(USART3);
}

static void adc_init() {
    // clock ADCs from PER_CK
    RCC->D3CCIPR =
        (RCC->D3CCIPR & (~RCC_D3CCIPR_ADCSEL_Msk)) | RCC_D3CCIPR_ADCSEL_per_ck;

    // enable peripheral clocks
    RCC->AHB4ENR |= RCC_AHB4ENR_GPIOAEN;
    RCC->AHB1ENR |= RCC_AHB1ENR_ADC12EN;

    // route PA0C and PA1C to ADC1
    SYSCFG->PMCR |= SYSCFG_PMCR_PA0SO | SYSCFG_PMCR_PA1SO;

    // set pins to analog
    gpio_init_analog(GPIOA, 0);
    gpio_init_analog(GPIOA, 1);

    // divide the ADC clock to 32MHz
    mreg(&ADC12_Com->CCR, ADC_COM_CCR_PRESC_Mask, ADC_COM_CCR_PRESC_DIV2);

    // enable ADC1
    ADC1->CR &= ~(ADC_CR_DEEPPWD);
    ADC1->CR |= ADC_CR_ADVREGEN | ADC_CR_BOOST;
    sys_delay_ms(2);
    ADC1->CR |= ADC_CR_ADCAL;

    // wait for calibration
    while (ADC1->CR & ADC_CR_ADCAL) {
    }

    // 10 bit conversions and no auto-injected conversions
    ADC1->CFGR = ADC_CFGR_JQDIS | ADC_CFGR_RES_10bit;

    // sample for a decent time :)
    ADC1->SMPR1 = (0x6ul << 0ul) | (0x6ul << 3ul);

    ADC1->JSQR = (1ul << 0ul) | // JL = 2 conversions
                 (0ul << 9ul) | // JSQ1 = ch0
                 (1ul << 15ul); // JSQ2 = ch1

    // enable ADC1
    ADC1->ISR = ADC_ISR_ADRDY; // clear stale flag
    ADC1->CR |= ADC_CR_ADEN;
    while (!(ADC1->ISR & ADC_ISR_ADRDY)) {
    }
    ADC1->ISR = ADC_ISR_ADRDY;
}

static void setup_ltdc_v2() {
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
    RCC->APB3RSTR = RCC_APB3ENR_LTDCEN;
    RCC->APB3RSTR = 0;
    RCC->APB3ENR |= RCC_APB3ENR_LTDCEN;

    ltdc_disable();
    ltdc_set_polarity(
        LPOL_ActiveLow, LPOL_ActiveLow, LPOL_ActiveLow, LPOL_ActiveLow
    );
    ltdc_set_size(10, 10, 43, 12, 8, 4, 480, 272);
    ltdc_set_background_color(0x00ff0000);

    ltdc_layer_disable(LTDC_Layer1);
    ltdc_layer_disable(LTDC_Layer2);

    // Layer 1: fullscreen RGB565 framebuffer
    ltdc_layer_setup_fullscreen(LTDC_Layer1);
    ltdc_layer_set_pixel_format(LTDC_Layer1, LPIXF_RGB565);
    ltdc_layer_setup_buffer_auto(LTDC_Layer1, (u32) l1fb1);
    ltdc_layer_set_blending(LTDC_Layer1, LBF_PixXConst, LBF_PixXConst);

    // Layer 2: fullscren AL44 text buffer
    ltdc_layer_setup_fullscreen(LTDC_Layer2);
    ltdc_layer_set_pixel_format(LTDC_Layer2, LPIXF_AL44);
    ltdc_layer_setup_buffer_auto(LTDC_Layer2, (u32) l2fb1);
    ltdc_layer_set_blending(LTDC_Layer2, LBF_PixXConst, LBF_PixXConst);

    ltdc_layer_enable(LTDC_Layer1);
    ltdc_layer_enable(LTDC_Layer2);

    // double buffering interrupt for the second layer
    ltdc_set_line_interrupt_active_end();
    ltdc_enable_interrupts(LTDC_INT_LINE);
    nvic_enable_irq(NI_LTDC);

    // just in case :)
    ltdc_reload_now();
    ltdc_enable();
}

void LTDC_IRQHandler() {
    u32 i = ltdc_get_interrupts();

    if (ltdcSwap) {
        // swap buffers
        u16 *tmp = l1disp;
        l1disp = l1draw;
        l1draw = tmp;

        // assign new display buffer and trigger reload
        ltdc_layer_swap_buffer(LTDC_Layer1, (u32) l1disp);
        ltdc_reload_vblank();

        ltdcSwap = 0;
    }

    ltdc_clear_interrupts(i);
}

static void setup_qspi_common() {
    // QSPI_CLK
    gpio_init_alt(GPIOF, 10, AF9, GP_None, GS_VeryHigh, GO_PushPull);
    // QSPI bank 1 (ncs, io0-3)
    gpio_init_alt(GPIOG, 6, AF10, GP_None, GS_VeryHigh, GO_PushPull);
    gpio_init_alt(GPIOD, 11, AF9, GP_None, GS_VeryHigh, GO_PushPull);
    gpio_init_alt(GPIOF, 9, AF10, GP_None, GS_VeryHigh, GO_PushPull);
    gpio_init_alt(GPIOF, 7, AF9, GP_None, GS_VeryHigh, GO_PushPull);
    gpio_init_alt(GPIOF, 6, AF9, GP_None, GS_VeryHigh, GO_PushPull);
    // QSPI bank 2 (ncs, io0-3)
    gpio_init_alt(GPIOC, 11, AF9, GP_None, GS_VeryHigh, GO_PushPull);
    gpio_init_alt(GPIOH, 2, AF9, GP_None, GS_VeryHigh, GO_PushPull);
    gpio_init_alt(GPIOH, 3, AF9, GP_None, GS_VeryHigh, GO_PushPull);
    gpio_init_alt(GPIOG, 9, AF9, GP_None, GS_VeryHigh, GO_PushPull);
    gpio_init_alt(GPIOG, 14, AF9, GP_None, GS_VeryHigh, GO_PushPull);

    // Clock from PLL2R (80MHz)
    mreg(&RCC->D1CCIPR, RCC_D1CCIPR_QSPISEL_Msk, RCC_D1CCIPR_QSPISEL_PLL2R);

    // Enable QSPI clock
    RCC->AHB3ENR |= RCC_AHB3ENR_QSPIEN;

    // Reset QSPI
    RCC->AHB3RSTR = RCC_AHB3RSTR_QSPIRST;
    RCC->AHB3RSTR = 0;

    // /2 prescaler, dual flash, shift
    qspi_cfg(2, true, true);
    // 2*64MB flash, 4 cycle CS high time, mode 0
    qspi_devcfg(128 * MEGA, 4, false);
    qspi_enable();

    // QuadSPI flash reset sequence

    {
        // assume QuadSPI mode
        qspi_cmd(
            QDD_SDR, QM_IWrite, CM_None, 0, CS_8bit, CM_None, CS_8bit, CM_None,
            CM_4Line, FCMD_RSTEN
        );
        qspi_end_transfer();
        qspi_cmd(
            QDD_SDR, QM_IWrite, CM_None, 0, CS_8bit, CM_None, CS_8bit, CM_None,
            CM_4Line, FCMD_RST
        );
        qspi_end_transfer();
    }
    {
        // also try single SPI mode
        qspi_cmd(
            QDD_SDR, QM_IWrite, CM_None, 0, CS_8bit, CM_None, CS_8bit, CM_None,
            CM_1Line, FCMD_RSTEN
        );
        qspi_end_transfer();
        qspi_cmd(
            QDD_SDR, QM_IWrite, CM_None, 0, CS_8bit, CM_None, CS_8bit, CM_None,
            CM_1Line, FCMD_RST
        );
        qspi_end_transfer();
    }

    // wait a bit for the flash to reset
    sys_delay_ms(1);

    u8 tmp[6];

    qspi_setup_read(6);
    qspi_wait_busy();
    QSPI->CCR = (QM_IRead << QSPI_CCR_FMODE_Pos)
                | (CM_1Line << QSPI_CCR_DMODE_Pos)
                | (CM_1Line << QSPI_CCR_IMODE_Pos) | FCMD_READ_ID;

    qspi_read_bytes(tmp, 6);

    qspi_jedec_ids[0][0] = tmp[0];
    qspi_jedec_ids[0][1] = tmp[2];
    qspi_jedec_ids[0][2] = tmp[4];
    qspi_jedec_ids[1][0] = tmp[1];
    qspi_jedec_ids[1][1] = tmp[3];
    qspi_jedec_ids[1][2] = tmp[5];
}

static void setup_qspi_write() {
    setup_qspi_common();

    qspi_wren();
    qspi_cmd(
        QDD_SDR, QM_IWrite, CM_None, 0, CS_8bit, CM_None, CS_8bit, CM_None,
        CM_1Line, FCMD_EN4B
    );
    qspi_end_transfer();
    qspi_wait_busy();
}

static void setup_qspi_mmap() {
    setup_qspi_common();

    qspi_wren(); // enable writes (apparently needed for EN4B)

    // go 4-byte address mode
    qspi_cmd(
        QDD_SDR, QM_IWrite, CM_None, 0, CS_8bit, CM_None, CS_8bit, CM_None,
        CM_1Line, FCMD_EN4B
    );
    qspi_wait_busy();

    // go QUADSPI
    qspi_cmd(
        QDD_SDR, QM_IWrite, CM_None, 0, CS_8bit, CM_None, CS_8bit, CM_None,
        CM_1Line, FCMD_ENTER_QUAD
    );
    qspi_end_transfer();

    // Set up memory-mapped mode
    qspi_cmd(
        QDD_SDR, QM_MMap, CM_4Line, 10, CS_8bit, CM_None, CS_32bit, CM_4Line,
        CM_4Line, FCMD_QUAD_READ_4B
    );
}

void entry() {
    // don't mess with ordering around here too much
    bsp_earlyinit();
    bsp_init_ext_mem();

    // setup MPU to control caching behavior
    mpu_disable();

    mpu_set_region_hl(0, SDRAMF_BASE, 4 * MEGA, CT_None, MAP_FullAccess);
    mpu_set_region_hl(1, SDRAMC_BASE, 4 * MEGA, CT_WT_RAlloc, MAP_FullAccess);

    mpu_enable_cfg(MPU_CTRL_PRIVDEFENA);

    // enable caches
    sys_icache_enable();
    sys_dcache_enable();

    bsp_lateinit();  // (sets up 64MHz systick)
    bsp_go_fast();   // (switches to PLL1@480MHz, fixes systick)
    sys_allfaults(); // enable all fault exceptions

    gpio_init_all_ports();
    // Red LED
    gpio_init_output(GPIOI, 13, GP_None, GS_Low, GO_PushPull);
    // User BTN
    gpio_init_input(GPIOC, 13, GP_None);

    // STLINK Serial
    usart_use_hsi();
    gpio_init_alt(GPIOB, 10, 7, GP_PullUp, GS_Medium, GO_PushPull);
    gpio_init_alt(GPIOB, 11, 7, GP_None, GS_Medium, GO_PushPull);
    usart_setup_basic(USART3, 921600);
    _crash_serial_ready = 1;

    // clear framebuffers
    memset32(l1fb1, 0, 480 * 272 / 2);
    memset32(l1fb2, 0, 480 * 272 / 2);
    memset32(l2fb1, 0, 480 * 272 / 4);

    // setup LTDC for the LCD
    setup_ltdc_v2();

    // USER button pressed at boot; enter QSPI bootloader
    if (gpio_read(GPIOC, 13)) {
        ltdc_layer_disable(LTDC_Layer1);
        ltdc_set_background_color(0x000088);
        ltdc_reload_now();
        setup_qspi_write();
        qspi_bootloader();
        sys_reboot();
        return;
    }

    setup_qspi_mmap();

    // setup ADC for the joystick
    adc_init();

    main();
}

u32 loader_cb_erase8k(u32 addr) {
    qspi_wren();
    qspi_cmd(
        QDD_SDR, QM_IWrite, CM_None, 0, // no data
        CS_8bit, CM_None,               // no alt bytes
        CS_32bit, CM_1Line,             // 4-byte address, 1-line
        CM_1Line, FCMD_SUBSEC_ER_4B
    );
    qspi_addr(addr); // triggers transfer
    qspi_end_transfer();
    qspi_wait_wip();
    return 0;
}

u32 loader_cb_write(u32 addr, const u8 *data, u32 n) {
    qspi_wren();
    // todo: turns out this should also be qspi_setup_read() but that's an
    // unfortunate name
    QSPI->DLR = n - 1;
    qspi_cmd(
        QDD_SDR, QM_IWrite, CM_4Line, 0, // quad data out
        CS_8bit, CM_None,                // no alt bytes
        CS_32bit, CM_1Line,              // 4-byte address, 1-line
        CM_1Line, FCMD_QUAD_PROG_4B
    );
    qspi_addr(addr);
    qspi_write_bytes(data, n);
    qspi_end_transfer();
    qspi_wait_wip();
    return 0;
}

u32 loader_cb_read(u32 addr, u8 *data, u32 n) {
    qspi_setup_read(n); // sets DLR = n-1
    qspi_cmd(
        QDD_SDR, QM_IRead, CM_4Line, 8, // quad data in, 8 dummy
        CS_8bit, CM_None,               // no alt bytes
        CS_32bit, CM_1Line,             // 4-byte address, 1-line
        CM_1Line, FCMD_QUAD_READ_4B
    );
    qspi_addr(addr);
    qspi_read_bytes(data, n);
    return 0;
}


#include <core/stdbad/string.h>