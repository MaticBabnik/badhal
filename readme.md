# BadHAL

A Hobby-grade HAL for the STM32-H750-disco board.

## Features

- [x] Logging over SWO (`swo_init`, `swo_writestr`)
- [x] Data & Instruction cache
- [x] SysTick based `sys_delay_ms`
- [x] 400 MHz system clock (aka `sys_go_fast`)
- [x] External SDRAM support ( ~~16~~ 8 MiB @ `0x9000_0000`)
- [x] GPIO
- [x] UART (only uart3 for now)
- [ ] QSPI
- [ ] LTDC
- [ ] I2C  
- [ ] Touch

### Other TODOs

- [ ] Cleanup init code
- [ ] Turn the HAL into a library
- [ ] Separate core HAL from BSP

## Building

On archlinux you can grab all deps by doing

```sh
sudo pacman -Syu --needed \
    cmake \
    arm-none-eabi-gcc \
    arm-none-eabi-gdb \
    arm-none-eabi-newlib \
    arm-none-eabi-binutils \
    stlink
```

Otherwise hunt down these deps for your system of choice.
Make sure your compiler supports C23

Then you can setup the project by doing

```sh
cmake -G Ninja -B build -D CMAKE_BUILD_TYPE=Release
```

And then build and flash with

```sh
cd build
ninja burn # has build as a dependency
```
