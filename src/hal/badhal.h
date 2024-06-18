
/*
   badhal.h - Shitty HAL za STM32H750
*/
#pragma once
#include <stddef.h>

#define SDRAM_BASE 0xD0000000

#define R_RW volatile       // read-write
#define R_WO volatile       // write-only
#define R_RO volatile const // read-only

typedef signed char i8;
typedef unsigned char u8;
typedef short i16;
typedef unsigned short u16;
typedef int i32;
typedef unsigned int u32;
typedef float f32;

// she G on my P till I O
struct GPIO_t
{
    u32 MODER;
    u32 OTYPER;
    u32 OSPEEDR;
    u32 PUPDR;
    u32 IDR;
    u32 ODR;
    u32 BSRR;
    u32 LCKR;
    u32 AFRL;
    u32 AFRH;
};

// cancer ass struct
struct RCC_t
{
    u32 CR;
    u32 HSISCR;
    u32 CRRCR;
    u32 CSICFGR;
    u32 CFGR;
    u32 __reserved00;
    u32 D1CFGR;
    u32 D2CFGR;
    u32 D3CFGR;
    u32 __reserved1;

    u32 PLLCKSELR;
    u32 PLLCFGR;
    u32 PLL1DIVR;
    u32 PLL1FRACR;
    u32 PLL2DIVR;
    u32 PLL2FRACR;
    u32 PLL3DIVR;
    u32 PLL3FRACR;
    u32 __reserved2;

    u32 D1CCIPR;
    u32 D2CCIP1R;
    u32 D2CCIP2R;
    u32 D3CCIPR;
    u32 __reserved3;

    u32 CIER;
    u32 CIFR;
    u32 CICR;
    u32 __reserved4;

    u32 BDCR;
    u32 CSR;
    u32 __reserved5;

    u32 AHB3RSTR;
    u32 AHB1RSTR;
    u32 AHB2RSTR;
    u32 AHB4RSTR;

    u32 APB3RSTR;
    u32 APB1LRSTR;
    u32 APB1HRSTR;
    u32 APB2RSTR;
    u32 APB4RSTR;

    u32 GCR;
    u32 __reserved6;

    u32 D3AMR;
    u32 __reserved7[9];

    u32 RSR;

    u32 AHB3ENR;
    u32 AHB1ENR;
    u32 AHB2ENR;
    u32 AHB4ENR;

    u32 APB3ENR;
    u32 APB1LENR;
    u32 APB1HENR;
    u32 APB2ENR;
    u32 APB4ENR;
    u32 __reserved8;

    u32 AHB3LPENR;
    u32 AHB1LPENR;
    u32 AHB2LPENR;
    u32 AHB4LPENR;
    u32 APB3LPENR;
    u32 APB1LLPENR;
    u32 APB1HLPENR;
    u32 APB2LPENR;
    u32 APB4LPENR;
    u32 __reserved9[5];

    u32 C1_AHB3ENR;
    u32 C1_AHB1ENR;
    u32 C1_AHB2ENR;
    u32 C1_AHB4ENR;
    u32 C1_APB3ENR;
    u32 C1_APB1LENR;
    u32 C1_APB1HENR;
    u32 C1_APB2ENR;
    u32 C1_APB4ENR;
    u32 __reservedA;

    u32 C1_AHB3LPENR;
    u32 C1_AHB1LPENR;
    u32 C1_AHB2LPENR;
    u32 C1_AHB4LPENR;
    u32 C1_APB3LPENR;
    u32 C1_APB1LLPENR;
    u32 C1_APB1HLPENR;
    u32 C1_APB2LPENR;
    u32 C1_APB4LPENR;

    u32 __reservedB[31];
};

struct SCB_t
{
    R_RO u32 CPUID;
    R_RW u32 ICSR;
    R_RW u32 VTOR;
    R_RW u32 AIRCR;
    R_RW u32 SCR;
    R_RW u32 CCR;
    R_RW u8 SHPR[12U];
    R_RW u32 SHCSR;
    R_RW u32 CFSR;
    R_RW u32 HFSR;
    R_RW u32 DFSR;
    R_RW u32 MMFAR;
    R_RW u32 BFAR;
    R_RW u32 AFSR;
    R_RO u32 ID_PFR[2U];
    R_RO u32 ID_DFR;
    R_RO u32 ID_AFR;
    R_RO u32 ID_MFR[4U];
    R_RO u32 ID_ISAR[5U];
    u32 reserved0[1U];

    R_RO u32 CLIDR;
    R_RO u32 CTR;
    R_RO u32 CCSIDR;
    R_RW u32 CSSELR;
    R_RW u32 CPACR;
    u32 reserved3[93U];

    R_WO u32 STIR;
    u32 reserved4[15U];

    R_RO u32 MVFR0;
    R_RO u32 MVFR1;
    R_RO u32 MVFR2;
    u32 reserved5[1U];

    R_WO u32 ICIALLU;
    u32 reserved6[1U];

    R_WO u32 ICIMVAU;
    R_WO u32 DCIMVAC;
    R_WO u32 DCISW;
    R_WO u32 DCCMVAU;
    R_WO u32 DCCMVAC;
    R_WO u32 DCCSW;
    R_WO u32 DCCIMVAC;
    R_WO u32 DCCISW;
    u32 reserved7[6U];

    R_RW u32 ITCMCR;
    R_RW u32 DTCMCR;
    R_RW u32 AHBPCR;
    R_RW u32 CACR;
    R_RW u32 AHBSCR;
    u32 reserved8[1U];

    R_RW u32 ABFSR;
};

struct MPU_t
{
    R_RO u32 TYPE;
    R_RW u32 CTRL;
    R_RW u32 RNR;
    R_RW u32 RBAR;
    R_RW u32 RASR;
    R_RW u32 RBAR_A1;
    R_RW u32 RASR_A1;
    R_RW u32 RBAR_A2;
    R_RW u32 RASR_A2;
    R_RW u32 RBAR_A3;
    R_RW u32 RASR_A3;
};

struct FLASH_t
{
    R_RW u32 ACR;
    R_RW u32 KEYR1;
    R_RW u32 OPTKEYR;
    R_RW u32 CR1;
    R_RW u32 SR1;
    R_RW u32 CCR1;
    R_RW u32 OPTCR;
    R_RW u32 OPTSR_CUR;
    R_RW u32 OPTSR_PRG;
    R_RW u32 OPTCCR;
    R_RW u32 PRAR_CUR1;
    R_RW u32 PRAR_PRG1;
    R_RW u32 SCAR_CUR1;
    R_RW u32 SCAR_PRG1;
    R_RW u32 WPSN_CUR1;
    R_RW u32 WPSN_PRG1;
    R_RW u32 BOOT_CUR;
    R_RW u32 BOOT_PRG;
    u32 reserved0[2];
    R_RW u32 CRCCR1;
    R_RW u32 CRCSADD1;
    R_RW u32 CRCEADD1;
    R_RW u32 CRCDATA;
    R_RW u32 ECC_FA1;
    u32 reserved;
    R_RW u32 OTPBL_CUR;
    R_RW u32 OTPBL_PRG;
    u32 reserved1[37];
    R_RW u32 KEYR2;
    u32 reserved2;
    R_RW u32 CR2;
    R_RW u32 SR2;
    R_RW u32 CCR2;
    u32 reserved3[4];
    R_RW u32 PRAR_CUR2;
    R_RW u32 PRAR_PRG2;
    R_RW u32 SCAR_CUR2;
    R_RW u32 SCAR_PRG2;
    R_RW u32 WPSN_CUR2;
    R_RW u32 WPSN_PRG2;
    u32 reserved4[4];
    R_RW u32 CRCCR2;
    R_RW u32 CRCSADD2;
    R_RW u32 CRCEADD2;
    R_RW u32 CRCDATA2;
    R_RW u32 ECC_FA2;
};

struct FMC_Bank1_t
{
    R_RW u32 BTCR[8];
};

struct FMC_Bank5_6_t
{
    R_RW u32 SDCRL;
    R_RW u32 SDCRH;
    R_RW u32 SDTRL;
    R_RW u32 SDTRH;

    R_RW u32 SDCMR;
    R_RW u32 SDRTR;
    R_RW u32 SDSR;
};

struct SysTick_t
{
    R_RW u32 CTRL;
    R_RW u32 LOAD;
    R_RW u32 VAL;
    R_WO u32 CALIB;
};

struct NVIC_t
{
    R_RW u32 ISER[8U];
    u32 reserved0[24U];
    R_RW u32 ICER[8U];
    u32 reserved1[24U];
    R_RW u32 ISPR[8U];
    u32 reserved2[24U];
    R_RW u32 ICPR[8U];
    u32 reserved3[24U];
    R_RW u32 IABR[8U];
    u32 reserved4[56U];
    R_RW u8 IP[240U];
    u32 reserved5[644U];
    R_WO u32 STIR;
};

struct PWR_t
{
    R_RW u32 CR1;
    R_RW u32 CSR1;
    R_RW u32 CR2;
    R_RW u32 CR3;
    R_RW u32 CPUCR;
    u32 RESERVED0;
    R_RW u32 D3CR;
    u32 RESERVED1;
    R_RW u32 WKUPCR;
    R_RW u32 WKUPFR;
    R_RW u32 WKUPEPR;
};

struct SYSCFG_t
{
    u32 RESERVED1;
    R_RW u32 PMCR;
    R_RW u32 EXTICR[4];
    R_RW u32 CFGR;
    u32 RESERVED2;
    R_RW u32 CCCSR;
    R_RW u32 CCVR;
    R_RW u32 CCCR;
    R_RW u32 PWRCR;
    u32 RESERVED3[61];
    R_RW u32 PKGR;
    u32 RESERVED4[118];
    R_RW u32 UR0;
    R_RW u32 UR1;
    R_RW u32 UR2;
    R_RW u32 UR3;
    R_RW u32 UR4;
    R_RW u32 UR5;
    R_RW u32 UR6;
    R_RW u32 UR7;
    R_RW u32 UR8;
    R_RW u32 UR9;
    R_RW u32 UR10;
    R_RW u32 UR11;
    R_RW u32 UR12;
    R_RW u32 UR13;
    R_RW u32 UR14;
    R_RW u32 UR15;
    R_RW u32 UR16;
    R_RW u32 UR17;
};

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

#pragma region Defines

#define SCB_SHCSR_MEMFAULTENA (1UL << 16)
#define MPU_CTRL_ENABLE 1
#define MPU_PRIVILEGED_DEFAULT 0x4

#define MPU_RASR_ATTRS_Pos 16
#define MPU_RASR_XN_Pos 28
#define MPU_RASR_AP_Pos 24
#define MPU_RASR_TEX_Pos 19
#define MPU_RASR_S_Pos 18
#define MPU_RASR_C_Pos 17
#define MPU_RASR_B_Pos 16
#define MPU_RASR_SRD_Pos 8
#define MPU_RASR_SIZE_Pos 1

#define MPU_RASR_ENABLE 1

#define MPU_REGION_SIZE_32MB 0x18
#define MPU_REGION_FULL_ACCESS 0x3

#define SCB_CCR_IC (1 << 17)

#define GPIOIEN (1 << 8);

#define SCB_AIRCR_VECTKEY_Pos 16U
#define SCB_AIRCR_VECTKEY_Msk (0xFFFFUL << SCB_AIRCR_VECTKEY_Pos)

#define SCB_AIRCR_PRIGROUP_Pos 8U
#define SCB_AIRCR_PRIGROUP_Msk (7UL << SCB_AIRCR_PRIGROUP_Pos)

#define RCC_DxCFGR_DxPPRE1_Mask (0x7UL << 4)
#define RCC_DxCFGR_DxPPRE1_DIV2 (1UL << 6)

#define RCC_DxCFGR_DxPPRE2_Mask (0x7UL << 8)
#define RCC_DxCFGR_DxPPRE2_DIV2 (1UL << 10)

#define RCC_D1CFGR_HPRE_Mask (0x7UL)
#define RCC_D1CFGR_HPRE_DIV2 (1UL << 3)

#define RCC_D1CFGR_D1CPRE_Mask (0xFUL << 8)
#define RCC_D1CFGR_D1CPRE_DIV1 0UL

#define RCC_CFGR_SW_Mask 0x7UL
#define RCC_CFGR_SW_PLL1 0x3UL

#define RCC_CFGR_SWS_Mask (0x7UL << 3)
#define RCC_CFGR_SWS_PLL1 (0x3UL << 3)

#define RCC_CR_HSION 1UL
#define RCC_CR_CSION (1UL << 7)
#define RCC_CR_HSEON (1UL << 16)
#define RCC_CR_PLL1ON (1UL << 24)
#define RCC_CR_HSI48ON (1UL << 12)
#define RCC_CSR_LSION 1UL

// RCC_CR_CSION
#define RCC_CR_HSIRDY (1UL << 2)
#define RCC_CR_CSIRDY (1UL << 8)
#define RCC_CR_HSERDY (1UL << 17)
#define RCC_CR_PLL1RDY (1UL << 25)
#define RCC_CR_HSI48RDY (1UL << 13)
#define RCC_CSR_LSIRDY (1UL << 1)

#define SCB_CPARCR_FULL_ACCESS_EVERYTHING 0x0FFFFFFFUL

#define RCC_AHB2ENR_SRAM1EN (1UL << 29)
#define RCC_AHB2ENR_SRAM2EN (1UL << 30)
#define RCC_AHB2ENR_SRAM3EN (1UL << 31)

#define RCC_AHB3ENR_FMCEN (1UL << 12)

#define RCC_APB4ENR_SYSCFGEN (1UL < 1)
#define SYSCFG_CCCSR_EN 1

#define SysTick_CTRL_CLKSOURCE (1UL << 2)
#define SysTick_CTRL_TICKINT (1 << 1)
#define SysTick_CTRL_ENABLE 1

#define PWR_SUPPLY_CONFIG_MASK 0x7UL
#define PWR_SUPPLY_LDO 0x2
#define PWR_CSR1_ACTVOSRDY (1 << 13)

#define RCC_PLLSOURCE_HSE (0x2UL)
#define RCC_PLLSOURCE_Mask (0x3UL)

#define RCC_PLLSOURCE_DIVM1_Pos 4UL
#define RCC_PLLSOURCE_DIVM1_Mask (0x3FUL << 4)

#define RCC_PLL1DIVR_DIVR_Pos 24UL
#define RCC_PLL1DIVR_DIVQ_Pos 16UL
#define RCC_PLL1DIVR_DIVP_Pos 9UL
#define RCC_PLL1DIVR_DIVN_Pos 0UL

#define RCC_PLLCFGR_PLL1FRACEN (1UL)
#define RCC_PLLCFGR_DIVP1EN (1UL << 16)
#define RCC_PLLCFGR_DIVQ1EN (1UL << 17)
#define RCC_PLLCFGR_DIVR1EN (1UL << 18)

#define RCC_PLLCFGR_PLL1VCOSEL_Mask (1UL << 1)
#define RCC_PLLCFGR_PLL1RGE_Pos 2UL
#define RCC_PLLCFGR_PLL1RGE_Mask (0x3UL << 2)

#define RCC_PLL1FRACR_FRACN1_Pos 3UL
#define RCC_PLL1FRACR_FRACN1_Mask (0xfffUL << 3)

#define FLASH_ACR_LATENCTY_Mask 0xFUL

#pragma endregion

void sys_earlyinit();
void sys_lateinit();
u32 sys_get_tick();
void sys_delay_ms();
void sys_go_fast();

void mem_mpu_setup_sdram();
void mem_enable_icache();
