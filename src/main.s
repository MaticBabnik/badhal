// Constants
.set GPIO_MODER,        0x0
.set GPIO_BSSR,         0x18
.set GPIO_MODE_MASK,    0x3
.set GPIO_MODE_OUT,     0x1
.set GPIOI_BASE,        0x58022000
.set RCC_AHB4ENR,       0x580244E0
.set SysTick,           0xE000E010
.set SysTick_CTRL,      0x0
.set SysTick_LOAD,      0x4
.set SysTick_CTRL_EN,   1
.set SysTick_CTRL_TICKINT,   2
.set SysTick_CTRL_EXT,  4
.set SysTick_CTRL_CF,   (1 << 16)

.set MILISECOND,        63999
.set DELAY_MS,          32000
// Variables (and big constants)
.section .data

lstate: .word 0
cnt: .word 1

// Code
.section .text
.thumb
.global entry
.global SysTick_Handler
.align 4
.type SysTick_Handler, %function
SysTick_Handler:
    push {r0-r4, lr}

    ldr r0, =lstate
    ldr r1, [r0, #4]

    sub r1, #1
    str r1, [r0, #4]
    cmp r1, #0

    bne SysTick_exit
    ldr r1, =500
    str r1, [r0, #4]

    ldr r1, [r0]
    cmp r1, #0
    mvn r1, r1
    str r1, [r0]
    bne SysTick_skip
    bl fn_gpio_on
    b SysTick_exit
SysTick_skip: 
    bl fn_gpio_off

SysTick_exit: 
    pop  {r0-r4, pc}

fn_gpio_init:
    push {r0-r2, lr}

    ldr r0, =RCC_AHB4ENR
    mov r1, #0
    mvn r1, r1 // is there a better way?
    str r1, [r0]

    ldr r0, =GPIOI_BASE
    
    ldr r1, [r0, #GPIO_MODER]
    
    mov r2, #GPIO_MODE_MASK
    lsl r2, r2, #26
    mvn r2, r2
    and r1, r2

    mov r2, #GPIO_MODE_OUT
    lsl r2, r2, #26
    orr r1, r2 
    str r1, [r0, #GPIO_MODER]

    pop {r0-r2, pc}


fn_systick_init:
    push {r0-r2,lr}
    ldr r0, =SysTick
    
    ldr r1, =MILISECOND
    str r1, [r0, #SysTick_LOAD]

    ldr r1, [r0, #SysTick_CTRL]
    mov r2, #SysTick_CTRL_EN
    orr r1, r2
    mov r2, #SysTick_CTRL_EXT
    orr r1, r2
    mov r2, #SysTick_CTRL_TICKINT
    orr r1, r2
    
    str r1, [r0, #SysTick_CTRL]

    pop {r0-r2,pc}
    

fn_gpio_on:
    push {r0-r1, lr}
    ldr r0, =GPIOI_BASE

    mov r1, #1
    lsl r1,r1, #13
    str r1, [r0, #GPIO_BSSR]

    pop {r0-r1, pc}


fn_gpio_off:
    push {r0-r1, lr}
    ldr r0, =GPIOI_BASE

    mov r1, #1
    lsl r1, r1, #29
    str r1, [r0, #GPIO_BSSR]

    pop {r0-r1, pc}

fn_delay:
    push {r1-r2, lr}
    mov r2, #1

delay_loopo:
    ldr r1, =DELAY_MS
delay_loopi:
    sub r1,r1, #1
    cmp r1, #0
    bne delay_loopi

    sub r0,r0, #1
    cmp r0, #0
    bne delay_loopo

    pop {r1,r2, pc}

fn_delay_systick:
    push {r1-r3, lr}
    ldr r1, =SysTick
    ldr r2, =SysTick_CTRL_CF

delays_loop:
    ldr r3, [r1, #SysTick_CTRL]
    
    tst r3, r2
    beq delays_loop
    
    sub r0, #1
    cmp r0, #0
    bne delays_loop

    pop {r1-r3, pc}


entry:
    bl fn_gpio_init
    bl fn_systick_init
loop: 
    nop
    b loop

