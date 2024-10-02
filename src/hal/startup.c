#include "./bad.h"
#include "./intrin.h"

#pragma GCC push_options
#pragma GCC optimize ("O0")

extern u32 _sidata; // Start of initialization values for .data
extern u32 _sdata;  // Start of .data section in SRAM
extern u32 _edata;  // End of .data section in SRAM
extern u32 _sbss;   // Start of .bss section
extern u32 _ebss;   // End of .bss section
extern u32 _estack; // Stack top
extern void entry() __attribute__((noinline));

#define INTPROTO(__name__) void __name__() __attribute__((weak,noinline,alias("Default_Handler")));

void Reset_Handler()
{
    asm volatile("ldr sp, =_estack");
    u32 *src, *dst;

    src = &_sidata;
    for (dst = &_sdata; dst < &_edata;)
        *(dst++) = *(src++);

    for (dst = &_sbss; dst < &_ebss;)
        *(dst++) = 0;

    entry();
}

// this gets called when we hit an unhandled interrupt
void Default_Handler()
{
    for (;;)
        a_nop();
}

INTPROTO(NMI_Handler)
INTPROTO(HardFault_Handler)
INTPROTO(MemManage_Handler)
INTPROTO(BusFault_Handler)
INTPROTO(UsageFault_Handler)
INTPROTO(SVC_Handler)
INTPROTO(DebugMon_Handler)
INTPROTO(PendSV_Handler)
INTPROTO(SysTick_Handler)
INTPROTO(WWDG_IRQHandler)
INTPROTO(PVD_AVD_IRQHandler)
INTPROTO(TAMP_STAMP_IRQHandler)
INTPROTO(RTC_WKUP_IRQHandler)
INTPROTO(FLASH_IRQHandler)
INTPROTO(RCC_IRQHandler)
INTPROTO(EXTI0_IRQHandler)
INTPROTO(EXTI1_IRQHandler)
INTPROTO(EXTI2_IRQHandler)
INTPROTO(EXTI3_IRQHandler)
INTPROTO(EXTI4_IRQHandler)
INTPROTO(DMA1_Stream0_IRQHandler)
INTPROTO(DMA1_Stream1_IRQHandler)
INTPROTO(DMA1_Stream2_IRQHandler)
INTPROTO(DMA1_Stream3_IRQHandler)
INTPROTO(DMA1_Stream4_IRQHandler)
INTPROTO(DMA1_Stream5_IRQHandler)
INTPROTO(DMA1_Stream6_IRQHandler)
INTPROTO(ADC_IRQHandler)
INTPROTO(FDCAN1_IT0_IRQHandler)
INTPROTO(FDCAN2_IT0_IRQHandler)
INTPROTO(FDCAN1_IT1_IRQHandler)
INTPROTO(FDCAN2_IT1_IRQHandler)
INTPROTO(EXTI9_5_IRQHandler)
INTPROTO(TIM1_BRK_IRQHandler)
INTPROTO(TIM1_UP_IRQHandler)
INTPROTO(TIM1_TRG_COM_IRQHandler)
INTPROTO(TIM1_CC_IRQHandler)
INTPROTO(TIM2_IRQHandler)
INTPROTO(TIM3_IRQHandler)
INTPROTO(TIM4_IRQHandler)
INTPROTO(I2C1_EV_IRQHandler)
INTPROTO(I2C1_ER_IRQHandler)
INTPROTO(I2C2_EV_IRQHandler)
INTPROTO(I2C2_ER_IRQHandler)
INTPROTO(SPI1_IRQHandler)
INTPROTO(SPI2_IRQHandler)
INTPROTO(USART1_IRQHandler)
INTPROTO(USART2_IRQHandler)
INTPROTO(USART3_IRQHandler)
INTPROTO(EXTI15_10_IRQHandler)
INTPROTO(RTC_Alarm_IRQHandler)
INTPROTO(TIM8_BRK_TIM12_IRQHandler)
INTPROTO(TIM8_UP_TIM13_IRQHandler)
INTPROTO(TIM8_TRG_COM_TIM14_IRQHandler)
INTPROTO(TIM8_CC_IRQHandler)
INTPROTO(DMA1_Stream7_IRQHandler)
INTPROTO(FMC_IRQHandler)
INTPROTO(SDMMC1_IRQHandler)
INTPROTO(TIM5_IRQHandler)
INTPROTO(SPI3_IRQHandler)
INTPROTO(UART4_IRQHandler)
INTPROTO(UART5_IRQHandler)
INTPROTO(TIM6_DAC_IRQHandler)
INTPROTO(TIM7_IRQHandler)
INTPROTO(DMA2_Stream0_IRQHandler)
INTPROTO(DMA2_Stream1_IRQHandler)
INTPROTO(DMA2_Stream2_IRQHandler)
INTPROTO(DMA2_Stream3_IRQHandler)
INTPROTO(DMA2_Stream4_IRQHandler)
INTPROTO(ETH_IRQHandler)
INTPROTO(ETH_WKUP_IRQHandler)
INTPROTO(FDCAN_CAL_IRQHandler)
INTPROTO(DMA2_Stream5_IRQHandler)
INTPROTO(DMA2_Stream6_IRQHandler)
INTPROTO(DMA2_Stream7_IRQHandler)
INTPROTO(USART6_IRQHandler)
INTPROTO(I2C3_EV_IRQHandler)
INTPROTO(I2C3_ER_IRQHandler)
INTPROTO(OTG_HS_EP1_OUT_IRQHandler)
INTPROTO(OTG_HS_EP1_IN_IRQHandler)
INTPROTO(OTG_HS_WKUP_IRQHandler)
INTPROTO(OTG_HS_IRQHandler)
INTPROTO(DCMI_IRQHandler)
INTPROTO(CRYP_IRQHandler)
INTPROTO(HASH_RNG_IRQHandler)
INTPROTO(FPU_IRQHandler)
INTPROTO(UART7_IRQHandler)
INTPROTO(UART8_IRQHandler)
INTPROTO(SPI4_IRQHandler)
INTPROTO(SPI5_IRQHandler)
INTPROTO(SPI6_IRQHandler)
INTPROTO(SAI1_IRQHandler)
INTPROTO(LTDC_IRQHandler)
INTPROTO(LTDC_ER_IRQHandler)
INTPROTO(DMA2D_IRQHandler)
INTPROTO(SAI2_IRQHandler)
INTPROTO(QUADSPI_IRQHandler)
INTPROTO(LPTIM1_IRQHandler)
INTPROTO(CEC_IRQHandler)
INTPROTO(I2C4_EV_IRQHandler)
INTPROTO(I2C4_ER_IRQHandler)
INTPROTO(SPDIF_RX_IRQHandler)
INTPROTO(OTG_FS_EP1_OUT_IRQHandler)
INTPROTO(OTG_FS_EP1_IN_IRQHandler)
INTPROTO(OTG_FS_WKUP_IRQHandler)
INTPROTO(OTG_FS_IRQHandler)
INTPROTO(DMAMUX1_OVR_IRQHandler)
INTPROTO(HRTIM1_Master_IRQHandler)
INTPROTO(HRTIM1_TIMA_IRQHandler)
INTPROTO(HRTIM1_TIMB_IRQHandler)
INTPROTO(HRTIM1_TIMC_IRQHandler)
INTPROTO(HRTIM1_TIMD_IRQHandler)
INTPROTO(HRTIM1_TIME_IRQHandler)
INTPROTO(HRTIM1_FLT_IRQHandler)
INTPROTO(DFSDM1_FLT0_IRQHandler)
INTPROTO(DFSDM1_FLT1_IRQHandler)
INTPROTO(DFSDM1_FLT2_IRQHandler)
INTPROTO(DFSDM1_FLT3_IRQHandler)
INTPROTO(SAI3_IRQHandler)
INTPROTO(SWPMI1_IRQHandler)
INTPROTO(TIM15_IRQHandler)
INTPROTO(TIM16_IRQHandler)
INTPROTO(TIM17_IRQHandler)
INTPROTO(MDIOS_WKUP_IRQHandler)
INTPROTO(MDIOS_IRQHandler)
INTPROTO(JPEG_IRQHandler)
INTPROTO(MDMA_IRQHandler)
INTPROTO(SDMMC2_IRQHandler)
INTPROTO(HSEM1_IRQHandler)
INTPROTO(ADC3_IRQHandler)
INTPROTO(DMAMUX2_OVR_IRQHandler)
INTPROTO(BDMA_Channel0_IRQHandler)
INTPROTO(BDMA_Channel1_IRQHandler)
INTPROTO(BDMA_Channel2_IRQHandler)
INTPROTO(BDMA_Channel3_IRQHandler)
INTPROTO(BDMA_Channel4_IRQHandler)
INTPROTO(BDMA_Channel5_IRQHandler)
INTPROTO(BDMA_Channel6_IRQHandler)
INTPROTO(BDMA_Channel7_IRQHandler)
INTPROTO(COMP1_IRQHandler)
INTPROTO(LPTIM2_IRQHandler)
INTPROTO(LPTIM3_IRQHandler)
INTPROTO(LPTIM4_IRQHandler)
INTPROTO(LPTIM5_IRQHandler)
INTPROTO(LPUART1_IRQHandler)
INTPROTO(CRS_IRQHandler)
INTPROTO(ECC_IRQHandler)
INTPROTO(SAI4_IRQHandler)
INTPROTO(WAKEUP_PIN_IRQHandler)

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
__attribute__((section(".isr_vector"), used)) volatile void (*const g_pfnVectors[])() = {
    (void (*)(void))&_estack,
    Reset_Handler,
    NMI_Handler,
    HardFault_Handler,
    MemManage_Handler,
    BusFault_Handler,
    UsageFault_Handler,
    0,
    0,
    0,
    0,
    SVC_Handler,
    DebugMon_Handler,
    0,
    PendSV_Handler,
    SysTick_Handler,
    WWDG_IRQHandler,
    PVD_AVD_IRQHandler,
    TAMP_STAMP_IRQHandler,
    RTC_WKUP_IRQHandler,
    FLASH_IRQHandler,
    RCC_IRQHandler,
    EXTI0_IRQHandler,
    EXTI1_IRQHandler,
    EXTI2_IRQHandler,
    EXTI3_IRQHandler,
    EXTI4_IRQHandler,
    DMA1_Stream0_IRQHandler,
    DMA1_Stream1_IRQHandler,
    DMA1_Stream2_IRQHandler,
    DMA1_Stream3_IRQHandler,
    DMA1_Stream4_IRQHandler,
    DMA1_Stream5_IRQHandler,
    DMA1_Stream6_IRQHandler,
    ADC_IRQHandler,
    FDCAN1_IT0_IRQHandler,
    FDCAN2_IT0_IRQHandler,
    FDCAN1_IT1_IRQHandler,
    FDCAN2_IT1_IRQHandler,
    EXTI9_5_IRQHandler,
    TIM1_BRK_IRQHandler,
    TIM1_UP_IRQHandler,
    TIM1_TRG_COM_IRQHandler,
    TIM1_CC_IRQHandler,
    TIM2_IRQHandler,
    TIM3_IRQHandler,
    TIM4_IRQHandler,
    I2C1_EV_IRQHandler,
    I2C1_ER_IRQHandler,
    I2C2_EV_IRQHandler,
    I2C2_ER_IRQHandler,
    SPI1_IRQHandler,
    SPI2_IRQHandler,
    USART1_IRQHandler,
    USART2_IRQHandler,
    USART3_IRQHandler,
    EXTI15_10_IRQHandler,
    RTC_Alarm_IRQHandler,
    0,
    TIM8_BRK_TIM12_IRQHandler,
    TIM8_UP_TIM13_IRQHandler,
    TIM8_TRG_COM_TIM14_IRQHandler,
    TIM8_CC_IRQHandler,
    DMA1_Stream7_IRQHandler,
    FMC_IRQHandler,
    SDMMC1_IRQHandler,
    TIM5_IRQHandler,
    SPI3_IRQHandler,
    UART4_IRQHandler,
    UART5_IRQHandler,
    TIM6_DAC_IRQHandler,
    TIM7_IRQHandler,
    DMA2_Stream0_IRQHandler,
    DMA2_Stream1_IRQHandler,
    DMA2_Stream2_IRQHandler,
    DMA2_Stream3_IRQHandler,
    DMA2_Stream4_IRQHandler,
    ETH_IRQHandler,
    ETH_WKUP_IRQHandler,
    FDCAN_CAL_IRQHandler,
    0,
    0,
    0,
    0,
    DMA2_Stream5_IRQHandler,
    DMA2_Stream6_IRQHandler,
    DMA2_Stream7_IRQHandler,
    USART6_IRQHandler,
    I2C3_EV_IRQHandler,
    I2C3_ER_IRQHandler,
    OTG_HS_EP1_OUT_IRQHandler,
    OTG_HS_EP1_IN_IRQHandler,
    OTG_HS_WKUP_IRQHandler,
    OTG_HS_IRQHandler,
    DCMI_IRQHandler,
    CRYP_IRQHandler,
    HASH_RNG_IRQHandler,
    FPU_IRQHandler,
    UART7_IRQHandler,
    UART8_IRQHandler,
    SPI4_IRQHandler,
    SPI5_IRQHandler,
    SPI6_IRQHandler,
    SAI1_IRQHandler,
    LTDC_IRQHandler,
    LTDC_ER_IRQHandler,
    DMA2D_IRQHandler,
    SAI2_IRQHandler,
    QUADSPI_IRQHandler,
    LPTIM1_IRQHandler,
    CEC_IRQHandler,
    I2C4_EV_IRQHandler,
    I2C4_ER_IRQHandler,
    SPDIF_RX_IRQHandler,
    OTG_FS_EP1_OUT_IRQHandler,
    OTG_FS_EP1_IN_IRQHandler,
    OTG_FS_WKUP_IRQHandler,
    OTG_FS_IRQHandler,
    DMAMUX1_OVR_IRQHandler,
    HRTIM1_Master_IRQHandler,
    HRTIM1_TIMA_IRQHandler,
    HRTIM1_TIMB_IRQHandler,
    HRTIM1_TIMC_IRQHandler,
    HRTIM1_TIMD_IRQHandler,
    HRTIM1_TIME_IRQHandler,
    HRTIM1_FLT_IRQHandler,
    DFSDM1_FLT0_IRQHandler,
    DFSDM1_FLT1_IRQHandler,
    DFSDM1_FLT2_IRQHandler,
    DFSDM1_FLT3_IRQHandler,
    SAI3_IRQHandler,
    SWPMI1_IRQHandler,
    TIM15_IRQHandler,
    TIM16_IRQHandler,
    TIM17_IRQHandler,
    MDIOS_WKUP_IRQHandler,
    MDIOS_IRQHandler,
    JPEG_IRQHandler,
    MDMA_IRQHandler,
    0,
    SDMMC2_IRQHandler,
    HSEM1_IRQHandler,
    0,
    ADC3_IRQHandler,
    DMAMUX2_OVR_IRQHandler,
    BDMA_Channel0_IRQHandler,
    BDMA_Channel1_IRQHandler,
    BDMA_Channel2_IRQHandler,
    BDMA_Channel3_IRQHandler,
    BDMA_Channel4_IRQHandler,
    BDMA_Channel5_IRQHandler,
    BDMA_Channel6_IRQHandler,
    BDMA_Channel7_IRQHandler,
    COMP1_IRQHandler,
    LPTIM2_IRQHandler,
    LPTIM3_IRQHandler,
    LPTIM4_IRQHandler,
    LPTIM5_IRQHandler,
    LPUART1_IRQHandler,
    0,
    CRS_IRQHandler,
    ECC_IRQHandler,
    SAI4_IRQHandler,
    0,
    0,
    WAKEUP_PIN_IRQHandler,
};
#pragma GCC diagnostic pop

#pragma GCC pop_options