
/*
   badhal.h - Shitty HAL za STM32H750
*/
#pragma once
#include "bad.h"
#include "intrin.h"

#include "hw/flash.h"
#include "hw/fmc.h"
#include "hw/gpio.h"
#include "hw/mpu.h"
#include "hw/nvic.h"
#include "hw/pwr.h"
#include "hw/rcc.h"
#include "hw/scb.h"
#include "hw/syscfg.h"
#include "hw/systick.h"

// Peripherals
static volatile struct FLASH_t *const FLASH = (void *)0x52002000;
static volatile struct FMC_Bank1_t *const FMC_Bank1_R = (void *)0x52004000;
static volatile struct FMC_Bank5_6_t *const FMC_Bank5_6_R = (void *)0x52004140;
static volatile struct SYSCFG_t *const SYSCFG = (void *)0x58000400;
static volatile struct GPIO_t *const GPIOA = (void *)0x58020000;
static volatile struct GPIO_t *const GPIOB = (void *)0x58020400;
static volatile struct GPIO_t *const GPIOC = (void *)0x58020800;
static volatile struct GPIO_t *const GPIOD = (void *)0x58020C00;
static volatile struct GPIO_t *const GPIOE = (void *)0x58021000;
static volatile struct GPIO_t *const GPIOF = (void *)0x58021400;
static volatile struct GPIO_t *const GPIOG = (void *)0x58021800;
static volatile struct GPIO_t *const GPIOH = (void *)0x58021C00;
static volatile struct GPIO_t *const GPIOI = (void *)0x58022000;
static volatile struct GPIO_t *const GPIOJ = (void *)0x58022400;
static volatile struct GPIO_t *const GPIOK = (void *)0x58022800;
static volatile struct RCC_t *const RCC = (void *)0x58024400;
static volatile struct PWR_t *const PWR = (void *)0x58024800;
static volatile struct SysTick_t *const SysTick = (void *)0xE000E010;
static volatile struct NVIC_t *const NVIC = (void *)0xE000E100;
static volatile struct SCB_t *const SCB = (void *)0xE000ED00;
static volatile struct MPU_t *const MPU = (void *)0xE000ED90;

#define SDRAM_BASE 0xD0000000

void sys_earlyinit();
void sys_lateinit();
u32 sys_get_tick();
void sys_delay_ms();
void sys_go_fast();

void mem_mpu_setup_sdram();
void mem_enable_icache();
