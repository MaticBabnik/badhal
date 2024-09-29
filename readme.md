# BadHAL

A Hobby-grade HAL for the STM32-H750-disco board.

## Features

- [x] Logging over SWO (`swo_init`, `swo_writestr`)
- [x] Data cache
- [x] Instruction cache
- [x] SysTick based `sys_delay_ms`
- [x] 400 MHz system clock (aka `sys_go_fast`)
- [x] External SDRAM support (32 MiB @ `0x9000_0000`)
- [x] GPIO
- [x] Very bad HD44780 driver (not a part of the HAL)
- [ ] `sys_delay_us`
- [ ] Interupts
- [ ] UART
- [ ] I2C  
- [ ] LTDC
- [ ] Touch
- [ ] Filesystem
- [ ] Audio

### Other TODOs

- [ ] Cleanup init code
- [ ] Switch to C++ (mostly for syntax, not OOP)
- [ ] Make BadHAL a CMake module
