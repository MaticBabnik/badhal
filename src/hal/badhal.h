
/*
   badhal.h - Shitty HAL za STM32H750
*/

#pragma once
#include "core/bad.h"

#include "hwdef/adc.h"
#include "hwdef/flash.h"
#include "hwdef/fmc.h"
#include "hwdef/gpio.h"
#include "hwdef/mpu.h"
#include "hwdef/nvic.h"
#include "hwdef/pwr.h"
#include "hwdef/rcc.h"
#include "hwdef/scb.h"
#include "hwdef/syscfg.h"
#include "hwdef/systick.h"
#include "hwdef/ltdc.h"
#include "hwdef/dma2d.h"
#include "hwdef/debug.h"
#include "hwdef/usart.h"

#define PERIPHERAL_DEF(_type_, _name_, _address_)                              \
    static struct _type_ *const _name_ = (void *) _address_

PERIPHERAL_DEF(USART_t, USART3, 0x40004800);
PERIPHERAL_DEF(ADC_t, ADC1, 0x40022000);
PERIPHERAL_DEF(LTDC_t, LTDC, 0x50001000);
PERIPHERAL_DEF(LTDC_Layer_t, LTDC_Layer1, 0x50001084);
PERIPHERAL_DEF(LTDC_Layer_t, LTDC_Layer2, 0x50001104);
PERIPHERAL_DEF(DMA2D_t, DMA2D, 0x52001000);
PERIPHERAL_DEF(FLASH_t, FLASH, 0x52002000);
PERIPHERAL_DEF(FMC_Bank1_t, FMC_Bank1_R, 0x52004000);
PERIPHERAL_DEF(FMC_Bank5_6_t, FMC_Bank5_6_R, 0x52004140);
PERIPHERAL_DEF(SYSCFG_t, SYSCFG, 0x58000400);
PERIPHERAL_DEF(GPIO_t, GPIOA, 0x58020000);
PERIPHERAL_DEF(GPIO_t, GPIOB, 0x58020400);
PERIPHERAL_DEF(GPIO_t, GPIOC, 0x58020800);
PERIPHERAL_DEF(GPIO_t, GPIOD, 0x58020C00);
PERIPHERAL_DEF(GPIO_t, GPIOE, 0x58021000);
PERIPHERAL_DEF(GPIO_t, GPIOF, 0x58021400);
PERIPHERAL_DEF(GPIO_t, GPIOG, 0x58021800);
PERIPHERAL_DEF(GPIO_t, GPIOH, 0x58021C00);
PERIPHERAL_DEF(GPIO_t, GPIOI, 0x58022000);
PERIPHERAL_DEF(GPIO_t, GPIOJ, 0x58022400);
PERIPHERAL_DEF(GPIO_t, GPIOK, 0x58022800);
PERIPHERAL_DEF(RCC_t, RCC, 0x58024400);
PERIPHERAL_DEF(PWR_t, PWR, 0x58024800);
PERIPHERAL_DEF(DBGMCU_t, DBGMCU, 0x5C001000);
PERIPHERAL_DEF(ITM_t, ITM, 0xE0000000);
PERIPHERAL_DEF(SysTick_t, SysTick, 0xE000E010);
PERIPHERAL_DEF(NVIC_t, NVIC, 0xE000E100);
PERIPHERAL_DEF(SCB_t, SCB, 0xE000ED00);
PERIPHERAL_DEF(MPU_t, MPU, 0xE000ED90);
PERIPHERAL_DEF(CoreDebug_t, CoreDebug, 0xE000EDF0);
PERIPHERAL_DEF(TPI_t, TPI, 0xE0040000);

#include "cache.h"

#define SDRAM_BASE 0xD0000000

typedef enum {
    CT_None = 0b00,
    CT_WriteBack_RWAlloc = 0b01,
    CT_WriteThrough_RAlloc = 0b10,
    CT_WriteBack_RAlloc = 0b11
} CacheType_t;

void sys_earlyinit();
void sys_lateinit();
u32 sys_get_tick();
void sys_delay_ms(u32 time);
void sys_go_fast();
u32 sys_get_freq();
void sys_allfaults();
void sys_init_ext_mem();

void sys_icache_enable();
void sys_dcache_enable();
void sys_dcache_flush();
void sys_dcache_invalidate();
void sys_dcache_disable();

void mem_mpu_setup_sdram(CacheType_t c);
void sys_trap(volatile char *msg);