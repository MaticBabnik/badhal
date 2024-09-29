#include "badhal.h"

void ltdc_setup_pll3()
{
}

void ltdc_init()
{
    // enable LTDC, and the required GPIOs
    RCC->APB3ENR |= RCC_APB3ENR_LTDCEN;
    RCC->AHB4ENR |= RCC_AHB4ENR_GPIOIEN | RCC_AHB4ENR_GPIOJEN |
                    RCC_AHB4ENR_GPIOKEN | RCC_AHB4ENR_GPIOHEN |
                    RCC_AHB4ENR_GPIODEN;

}