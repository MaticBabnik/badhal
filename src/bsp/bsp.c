#include "bsp.h"
#include <hal/badhal.h>
#include <hal/driver/clock.h>
#include <hal/driver/nvic.h>

extern u32 coreFreq;

void bsp_earlyinit() {
    // TODO: check what is actually needed here
    // TODO: add reset and enable functions to driver/rcc

    // default flash latency
    FLASH->ACR = (FLASH->ACR & ~(0xful)) | 7;

    // Enable all coprocessors in all modes
    SCB->CPACR |= 0xfffffffful;

    a_dsb();
    a_isb();

    if (SCB->CPACR == 0) {
        sys_trap("FPU died???");
    }

    // reset a bunch of clock related shit
    RCC->CR |= RCC_CR_HSION;
    RCC->CFGR = 0;
    /* Reset HSEON, CSSON , CSION,RC48ON, CSIKERON PLL1ON, PLL2ON and PLL3ON
     * bits */
    RCC->CR &= 0xEAF6ED7FU;
    RCC->D1CFGR = 0x00000000;
    RCC->D2CFGR = 0x00000000;
    RCC->D3CFGR = 0x00000000;
    RCC->PLLCKSELR = 0x00000000;
    RCC->PLLCFGR = 0x00000000;
    RCC->PLL1DIVR = 0x00000000;
    RCC->PLL1FRACR = 0x00000000;
    RCC->PLL2DIVR = 0x00000000;
    RCC->PLL2FRACR = 0x00000000;
    RCC->PLL3DIVR = 0x00000000;
    RCC->PLL3FRACR = 0x00000000;
    RCC->CR &= 0xFFFBFFFFU;
    RCC->CIER = 0x00000000;

    // Enable the other SRAM banks
    RCC->AHB2ENR |=
        RCC_AHB2ENR_SRAM1EN | RCC_AHB2ENR_SRAM2EN | RCC_AHB2ENR_SRAM3EN;

    // unhinged shit: if stm32h7 revY
    if ((DBGMCU->IDCODE & DBGMCU_IDCODE_REVID_Msk) < 0x20000000U) {
        // Set switch matrix read issuing override of 1 for target 7
        // Target 7 is "AXI-SRAM"
        *((volatile u32 *) 0x51008108) = 0x000000001U;
    }

    RCC->APB4ENR |= 2; // SYSCFG enable ???
    // TODO: #define ^

    // Disable the FMC bank1 (enabled after reset).
    // This, prevents CPU speculation access on this bank which blocks the use
    // of FMC during 24us. During this time the others FMC master (such as LTDC)
    // cannot use it!
    FMC_Bank1_R->BTCR[0] = 0x000030D2;
    // TODO: wtf???
}

void bsp_lateinit() {
    // NVIC Group
    nvic_priority_grouping(3); // what the sigma?

    sys_set_systick(64000);
}

void bsp_power_ldo() {
    volatile u32 tmp;
    PWR->CR3 = (PWR->CR3 & ~(PWR_SUPPLY_CONFIG_MASK)) | PWR_SUPPLY_LDO;
    tmp = PWR->CR3;
    while (!(PWR->CSR1 & PWR_CSR1_ACTVOSRDY)) {
        // wait for voltage level
    }
    // ~~disable~~ enable overdrive
    SYSCFG->PWRCR |= SYSCFG_PWRCR_ODEN;
    tmp = SYSCFG->PWRCR;

    // set VOS to 0b11
    PWR->D3CR |= 0xc000;
    tmp = PWR->D3CR;

    while ((PWR->D3CR & (1u << 13)) == 0) {
        // wait for VOSRDY
    }

    (void) tmp;
}

void bsp_init_osc2() {
    // enable HSE, HSI48 and HSI
    RCC->CR |= RCC_CR_HSEON | RCC_CR_HSI48ON | RCC_CR_HSION;
    clk_wait_ready(RCC_CR_HSERDY | RCC_CR_HSI48RDY | RCC_CR_HSIRDY);

    clk_pll_disable_all();

    clk_pll_srccfg(
        CK_PLL_SRC_HSE, // 25 MHz
        5,              // PLL1 DIVM -> 5MHz       Core
        5,              // PLL2 DIVM -> 5MHz       QSPI
        5               // PLL3 DIVM -> 5MHz       LCD
    );

    clk_pll_cfg(
        PLL1,
        false, // no fraction
        false, // wide VCO
        PLL_Range_8_16MHz, PLL_DivP | PLL_DivQ | PLL_DivR
    );

    clk_pll_cfg(
        PLL2,
        false, // no fraction
        false, // wide VCO
        PLL_Range_8_16MHz, PLL_DivR
    );

    clk_pll_cfg(
        PLL3,
        false, // no fraction
        false, // wide VCO
        PLL_Range_8_16MHz, PLL_DivR
    );

    clk_pll_divcfg(
        PLL1,
        192, // 960MHz internal
        2,   // P: 480MHz - max CPU clock
        8,   // Q: 120MHz
        4    // R: 240MHz - max peripheral clock
    );

    clk_pll_divcfg(
        PLL2,
        80, // N: 5 * 80 = 400 MHz
        4,  // P: (disabled) 100 MHz
        4,  // Q: (disabled) 100 MHz
        5   // R: 80 MHz
    );

    clk_pll_divcfg(
        PLL3,
        40, // N: VCO = 5 * 40 = 200 MHz  (in range)
        80, // P: (disabled) 2.5 MHz
        20, // Q: (disabled) 10 MHz
        20  // R: 10 MHz -> LCD
    );

    clk_pll_enable_one(PLL1);
    clk_pll_enable_one(PLL2);
    clk_pll_enable_one(PLL3);
    clk_wait_ready(RCC_CR_PLL1RDY | RCC_CR_PLL2RDY | RCC_CR_PLL3RDY);
}

void bsp_clk_config() {
    // 7 waits states, 2 for the other thing, idk
    FLASH->ACR = (FLASH->ACR & ~(0xfful)) | 0x27;

    RCC->D1CFGR = (RCC->D1CFGR & ~(0xful)) | (0x8ul); // HCPRE div by 2
    // mess around with domain clocks... i think this is relavant for
    // peripherals
    RCC->D1CFGR =
        (RCC->D1CFGR & ~RCC_DxCFGR_DxPPRE1_Mask) | RCC_DxCFGR_DxPPRE1_DIV2;
    RCC->D1CFGR =
        (RCC->D1CFGR & ~RCC_D1CFGR_D1CPRE_Mask) | RCC_D1CFGR_D1CPRE_DIV1;
    RCC->D2CFGR =
        (RCC->D2CFGR & ~RCC_DxCFGR_DxPPRE1_Mask) | RCC_DxCFGR_DxPPRE1_DIV2;
    RCC->D2CFGR =
        (RCC->D2CFGR & ~RCC_DxCFGR_DxPPRE2_Mask) | RCC_DxCFGR_DxPPRE2_DIV2;
    RCC->D3CFGR =
        (RCC->D3CFGR & ~RCC_DxCFGR_DxPPRE1_Mask) | RCC_DxCFGR_DxPPRE1_DIV2;

    // Switch system clock source (or die trying)
    RCC->CFGR |= RCC_CFGR_SW_PLL1;
    while ((RCC->CFGR & RCC_CFGR_SWS_Mask) != RCC_CFGR_SWS_PLL1) {
    }
}

void bsp_go_fast() {
    volatile u32 tmp;
    bsp_power_ldo();
    bsp_init_osc2();
    bsp_clk_config();

    sys_set_systick(480000); // trust me
    coreFreq = 480000000;

    // activate CSI clock mondatory for I/O Compensation Cell
    RCC->CR |= RCC_CR_CSION;

    // Enable SYSCFG clock mondatory for I/O Compensation Cell
    RCC->APB4ENR |= RCC_APB4ENR_SYSCFGEN;
    // tiny delay
    tmp = RCC->APB4ENR & RCC_APB4ENR_SYSCFGEN;

    // Enables the I/O Compensation Cell
    SYSCFG->CCCSR |= SYSCFG_CCCSR_EN;

    (void) tmp;
}

void bsp_init_ext_mem() {
    volatile u32 tmp;
    register u32 tmpreg = 0xff, timeout = 0xFFFF;
    register volatile u32 index;

    // Enable GPIO D-I interface clock
    RCC->AHB4ENR |= 0x000001F8;
    tmp = (RCC->AHB4ENR);

    /* Connect PDx pins to FMC Alternate function */
    GPIOD->AFR[0] = 0x000000CC;
    GPIOD->AFR[1] = 0xCC000CCC;
    /* Configure PDx pins in Alternate function mode */
    GPIOD->MODER = 0xAFEAFFFA;
    /* Configure PDx pins speed to 100 MHz */
    GPIOD->OSPEEDR = 0xF03F000F;
    /* Configure PDx pins Output type to push-pull */
    GPIOD->OTYPER = 0x00000000;
    /* Configure PDx pins in Pull-up */
    GPIOD->PUPDR = 0x50150005;

    /* Connect PEx pins to FMC Alternate function */
    GPIOE->AFR[0] = 0xC00000CC;
    GPIOE->AFR[1] = 0xCCCCCCCC;
    /* Configure PEx pins in Alternate function mode */
    GPIOE->MODER = 0xAAAABFFA;
    /* Configure PEx pins speed to 100 MHz */
    GPIOE->OSPEEDR = 0xFFFFC00F;
    /* Configure PEx pins Output type to push-pull */
    GPIOE->OTYPER = 0x00000000;
    /* Configure PEx pins in Pull-up */
    GPIOE->PUPDR = 0x55554005;

    /* Connect PFx pins to FMC Alternate function */
    GPIOF->AFR[0] = 0x00CCCCCC;
    GPIOF->AFR[1] = 0xCCCCC000;
    /* Configure PFx pins in Alternate function mode */
    GPIOF->MODER = 0xAABFFAAA;
    /* Configure PFx pins speed to 100 MHz */
    GPIOF->OSPEEDR = 0xFFC00FFF;
    /* Configure PFx pins Output type to push-pull */
    GPIOF->OTYPER = 0x00000000;
    /* Configure PFx pins in Pull-up */
    GPIOF->PUPDR = 0x55400555;

    /* Connect PGx pins to FMC Alternate function */
    GPIOG->AFR[0] = 0x00CC00CC;
    GPIOG->AFR[1] = 0xC000000C;
    /* Configure PGx pins in Alternate function mode */
    GPIOG->MODER = 0xBFFEFAFA;
    /* Configure PGx pins speed to 100 MHz */
    GPIOG->OSPEEDR = 0xC0030F0F;
    /* Configure PGx pins Output type to push-pull */
    GPIOG->OTYPER = 0x00000000;
    /* Configure PGx pins in Pull-up */
    GPIOG->PUPDR = 0x40010505;

    /* Connect PHx pins to FMC Alternate function */
    GPIOH->AFR[0] = 0xCCC00000;
    GPIOH->AFR[1] = 0xCCCCCCCC;
    /* Configure PHx pins in Alternate function mode */
    GPIOH->MODER = 0xAAAAABFF;
    /* Configure PHx pins speed to 100 MHz */
    GPIOH->OSPEEDR = 0xFFFFFC00;
    /* Configure PHx pins Output type to push-pull */
    GPIOH->OTYPER = 0x00000000;
    /* Configure PHx pins in Pull-up */
    GPIOH->PUPDR = 0x55555400;

    /*-- FMC Configuration
     * ------------------------------------------------------*/
    /* Enable the FMC interface clock */
    RCC->AHB3ENR |= (RCC_AHB3ENR_FMCEN);
    /*SDRAM Timing and access interface configuration*/
    /*LoadToActiveDelay  = 2
      ExitSelfRefreshDelay = 6
      SelfRefreshTime      = 4
      RowCycleDelay        = 6
      WriteRecoveryTime    = 2
      RPDelay              = 2
      RCDDelay             = 2
      SDBank             = FMC_SDRAM_BANK2
      ColumnBitsNumber   = FMC_SDRAM_COLUMN_BITS_NUM_8
      RowBitsNumber      = FMC_SDRAM_ROW_BITS_NUM_12
      MemoryDataWidth    = FMC_SDRAM_MEM_BUS_WIDTH_16
      InternalBankNumber = FMC_SDRAM_INTERN_BANKS_NUM_4
      CASLatency         = FMC_SDRAM_CAS_LATENCY_2
      WriteProtection    = FMC_SDRAM_WRITE_PROTECTION_DISABLE
      SDClockPeriod      = FMC_SDRAM_CLOCK_PERIOD_2
      ReadBurst          = FMC_SDRAM_RBURST_ENABLE
      ReadPipeDelay      = FMC_SDRAM_RPIPE_DELAY_0*/

    FMC_Bank5_6_R->SDCRL = 0x00001800;
    FMC_Bank5_6_R->SDCRH = 0x00000154;
    FMC_Bank5_6_R->SDTRL = 0x00105000;
    FMC_Bank5_6_R->SDTRH = 0x01010351;

    /* SDRAM initialization sequence */
    /* Clock enable command */
    FMC_Bank5_6_R->SDCMR = 0x00000009;
    tmpreg = FMC_Bank5_6_R->SDSR & 0x00000020;
    while ((tmpreg != 0) && (timeout-- > 0)) {
        tmpreg = FMC_Bank5_6_R->SDSR & 0x00000020;
    }

    /* Delay */
    for (index = 0; index < 1000; index++)
        ;

    /* PALL command */
    FMC_Bank5_6_R->SDCMR = 0x0000000A;
    timeout = 0xFFFF;
    tmpreg = FMC_Bank5_6_R->SDSR & 0x00000020;
    while ((tmpreg != 0) && (timeout-- > 0)) {
        tmpreg = FMC_Bank5_6_R->SDSR & 0x00000020;
    }

    FMC_Bank5_6_R->SDCMR = 0x000000EB;
    timeout = 0xFFFF;
    tmpreg = FMC_Bank5_6_R->SDSR & 0x00000020;
    while ((tmpreg != 0) && (timeout-- > 0)) {
        tmpreg = FMC_Bank5_6_R->SDSR & 0x00000020;
    }

    FMC_Bank5_6_R->SDCMR = 0x0004400C;
    timeout = 0xFFFF;
    tmpreg = FMC_Bank5_6_R->SDSR & 0x00000020;
    while ((tmpreg != 0) && (timeout-- > 0)) {
        tmpreg = FMC_Bank5_6_R->SDSR & 0x00000020;
    }
    /* Set refresh count */
    tmpreg = FMC_Bank5_6_R->SDRTR;
    FMC_Bank5_6_R->SDRTR = (tmpreg | (0x00000603 << 1));

    /* Disable write protection */
    tmpreg = FMC_Bank5_6_R->SDCRH;
    FMC_Bank5_6_R->SDCRH = (tmpreg & 0xFFFFFDFF);

    /*FMC controller Enable*/
    FMC_Bank1_R->BTCR[0] |= 0x80000000;

    (void) (tmp);
}
