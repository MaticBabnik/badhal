/*
 * BadHAL linker script - STM32H750xB, execute from internal FLASH.
 *
 * Memory map (per ST RM0433 / DS12556):
 *   FLASH    0x08000000  128K   code + rodata, .data init image
 *   ITCMRAM  0x00000000   64K   (unused by default)
 *   DTCMRAM  0x20000000  128K   (unused by default)
 *   RAM_D1   0x24000000  512K   .data / .bss / heap / stack
 *   RAM_D2   0x30000000  288K   (unused by default)
 *   RAM_D3   0x38000000   64K   (unused by default)
 *
 */

MEMORY
{
  FLASH    (rx)  : ORIGIN = 0x08000000, LENGTH = 128K
  ITCMRAM  (xrw) : ORIGIN = 0x00000000, LENGTH = 64K
  DTCMRAM  (xrw) : ORIGIN = 0x20000000, LENGTH = 128K
  RAM_D1   (xrw) : ORIGIN = 0x24000000, LENGTH = 512K
  RAM_D2   (xrw) : ORIGIN = 0x30000000, LENGTH = 288K
  RAM_D3   (xrw) : ORIGIN = 0x38000000, LENGTH = 64K
}

ENTRY(Reset_Handler)

/* Top of stack = top of RAM_D1 */
_estack = ORIGIN(RAM_D1) + LENGTH(RAM_D1);

/* Minimum heap/stack the linker should reserve before complaining */
_Min_Heap_Size  = 0x200;
_Min_Stack_Size = 0x400;

SECTIONS
{
  .isr_vector :
  {
    . = ALIGN(4);
    KEEP(*(.isr_vector))
    . = ALIGN(4);
  } > FLASH

  .text :
  {
    . = ALIGN(4);
    *(.text)
    *(.text*)
    KEEP(*(.init))
    KEEP(*(.fini))
    . = ALIGN(4);
    _etext = .;
  } > FLASH

  .rodata :
  {
    . = ALIGN(4);
    *(.rodata)
    *(.rodata*)
    . = ALIGN(4);
  } > FLASH

  _sidata = LOADADDR(.data);

  .data :
  {
    . = ALIGN(4);
    _sdata = .;
    KEEP(*(.data))
    *(.data*)
    . = ALIGN(4);
    _edata = .;
  } > RAM_D1 AT> FLASH

  .bss (NOLOAD) :
  {
    . = ALIGN(4);
    _sbss = .;
    __bss_start__ = _sbss;
    *(.bss)
    *(.bss*)
    *(COMMON)
    . = ALIGN(4);
    _ebss = .;
    __bss_end__ = _ebss;
  } > RAM_D1

  ._user_heap_stack (NOLOAD) :
  {
    . = ALIGN(8);
    PROVIDE(end = .);
    PROVIDE(_end = .);
    . = . + _Min_Heap_Size;
    . = . + _Min_Stack_Size;
    . = ALIGN(8);
  } > RAM_D1

  /DISCARD/ :
  {
    libc.a ( * )
    libm.a ( * )
    libgcc.a ( * )
  }

  .ARM.attributes 0 : { *(.ARM.attributes) }
}
