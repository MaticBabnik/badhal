// Constants
.set GPIO_MODER,        0x0
.set GPIO_BSSR,         0x18
.set GPIO_MODE_MASK,    0x3
.set GPIO_MODE_OUT,     0x1
.set GPIOI_BASE,        0x58022000
.set RCC_AHB4ENR,       0x580244E0
.set DELAY_MS,          32000
// Variables (and big constants)
.section .data


// Code
.section .text
.thumb
.global entry

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

entry:
    bl fn_gpio_init
loop:
    bl fn_gpio_on
    ldr r0, =#500
    bl fn_delay
    bl fn_gpio_off
    ldr r0, =#500
    bl fn_delay

b loop

