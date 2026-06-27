# BadHAL

A Hobby-grade HAL/BSP for the STM32-H750-disco board.

## TODO

- [ ] Tune memory timings
- [ ] Doom
- [ ] Ethernet + network stack
- [ ] UDP pixelflut
- [ ] Build a pixelflut wall @ DCTF27 or something.

## BSP features

- System clock setup (480MHz + QSPI + LTDC)
- Setup for external SDRAM (8 MiB @ `0x9000_0000`)
- Header only include fault handlers (`crash.h`)

## HAL Drivers

- `usart` (blocking only for now)
- `nvic`
- `clock` (subset of RCC; work in progress)
- `mpu`
- `gpio`
- `debug` (SWO logging)
- `qspi`
- `ltdc`

## Libraries

- `badfs` (a simple read-only filesystem for QSPI flash)
- `text` (a simple text rendering library for LTDC)
- `sprintf` (stb_sprintf)

## Other features

- `SysTick` and `sys_delay`
- FPU setup
- I-Cache, D-Cache and branch predictor setup
- C23 only codebase with a couple intrinsics

## Building

On Arch Linux (btw) you can grab all deps by doing

```sh
sudo pacman -Syu --needed \
    cmake \
    arm-none-eabi-gcc \
    arm-none-eabi-gdb \
    arm-none-eabi-newlib \
    arm-none-eabi-binutils \
    stlink
```

If targeting clang also grab `clang`, `lld`, and `baremetal-compiler-rt`(AUR).

Otherwise hunt down these deps for your system of choice.
Make sure your compiler is new enough for C23

Then you can setup the project by doing

```sh
cmake -G Ninja -B build -DCMAKE_BUILD_TYPE=Release

# you can also set:
# -DGCC_PREFIX="/path/to/your/gcc/arm-none-eabi"
# -- or --
# -DUSE_CLANG=ON
# -DCLANG_PATH="/path/to/folder/containing/clang/"
# -DCLANG_RTLIB_PATH="/dir/libclang_rt.builtins-armv7em.a"
```

And then build and flash with

```sh
cd build
ninja burn # has build as a dependency
```

## Debugging

The project is setup for VSCode debugging with Cortex-Debug.
Pressing F5 should start a debug session.

Make sure you rebuild and flash before debugging.

## Docs / learning

You should check the following docs in order from most to least specific:

- [UM2488 - Discovery kit manual](https://www.st.com/resource/en/user_manual/um2488-discovery-kits-with-stm32h745xi-and-stm32h750xb-mcus-stmicroelectronics.pdf) (pinout, names other components on board)
- [Discovery board schematics](https://www.st.com/resource/en/schematic_pack/mb1381-h750xb-b01-schematic.pdf) (very specific part numbers)
- [STM32H750xB datasheet](https://www.st.com/resource/en/datasheet/stm32h750ib.pdf) (contains GPIO AF table)
- [RM0433 - STM32H7 series manual](https://www.st.com/resource/en/reference_manual/rm0433-stm32h742-stm32h743753-and-stm32h750-value-line-advanced-armbased-32bit-mcus-stmicroelectronics.pdf) (documents most peripherals, memory map)
- [ARM Cortex-M7 manual](https://documentation-service.arm.com/static/5e906b038259fe2368e2a7bb) (MPU, FPU, NVIC...)
- [ARMv7-M Architecture Reference Manual](https://documentation-service.arm.com/static/606dc36485368c4c2b1bf62f) (ISA, SCB, ...)

There are also the specific manuals for the DK components:

- [LCD datasheet](https://www.rocktech.com.hk/wp-content/uploads/tft-lcd-datasheet/RK043FN88H-CT661C.pdf)
- [Flash datasheet](https://jm.pl/gfx-base/s_1/orgs/18/MT25QL512ABB8ESF-0SIT.pdf)

Shoutout:

- [iKramp/stm32-project](https://github.com/iKramp/stm32-project) (Good place to "borrow" some code)

<!-- ## Rants / observations / comments

- ST software is... not great...
- All SVD files FUCKING SUCK.
- FMC, memory buses, MPU, QSPI, and other are confusing and I don't know enough to claim I did it correctly.
- LTDC registers are fucky and require random +1,-1,+3 offsets... They also require you to sum the sync/porch/active values.
- The DMA system seems genuinly cool, but I'm not touching it...
- Building with -O3 by default is a ***Bad*** idea. The compiler does goofy shit all over the place and causes random unaligned accesses. GCC and clang seem to be equaly goofy.
- I really should invest in some CMake fuckery and make multiple projects/entrypoints calling into the same HAL.
- Embeded "people" succesfully gatekept their fuckass code and Claude kinda sucks at it. -->
