#include "hal/badhal.h"
#include "hal/driver/gpio.h"
#include "hal/driver/debug.h"
#include "hal/driver/usart.h"


#define STB_SPRINTF_IMPLEMENTATION
#define STB_SPRINTF_NOFLOAT
#include "ext/stb_sprintf.h"

#include "memtest67.h"

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

    usart_send_string(USART3, "Nihao fine shyt\r\n");
    sys_delay_ms(10);
    swo_writestr("Test...");
    run_memtests();

    // ADC init
    // adc_init();

    usart_send_string(USART3, "Done!\r\n");

    for (;;) {
        // printf("Read %d\r\n", ADC1->DR);
        sys_delay_ms(100);
    }
}

// the very start
void entry() {
    // don't mess with ordering around here too much
    sys_earlyinit();
    sys_init_ext_mem();
    mem_mpu_setup_sdram(CT_None); // mounts external RAM
    sys_icache_enable();
    sys_dcache_enable();
    sys_lateinit();     // (sets up 64MHz systick)
    sys_go_fast();      // (switches to PLL1@400MHz, fixes systick)
    swo_init(SWO_2MHZ); // enable SWO logging
    sys_allfaults();    // enable all fault exceptions

    main();
}

void onTick() {}