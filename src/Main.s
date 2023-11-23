.syntax unified
.cpu cortex-m7
.thumb

.equ     LEDDELAY,       64000
.equ     RCC_AHB4ENR,    0x580244E0 // RCC AHB4 peripheral clock reg
.equ     GPIOI_BASE,     0x58022000 // GPIOI base address)
.equ     GPIOx_MODER,    0x00 // GPIOx port mode register
.equ     GPIOx_ODR,      0x14 // GPIOx output data register
.equ     GPIOx_BSSR,     0x18 // GPIOx port set/reset register
.equ     LED_OFF,       0x00002000   	// Setting pin to 1 -> LED is off
.equ     LED_ON,   	 0x20000000   	// Setting pin to 0 -> LED is on

.data
.align
// vars?

.text
.type  main, %function
.global main
.align
main:
	    bl setup_led
		mov r5, #0
loop:
		mvn r5, r5 //flip led state
		mov r0, r5
		bl set_led

		ldr r0, =#1000
		bl delay_ms


		b loop          // skok na vrstico loop:


__end: 	b 	__end


setup_led:
	push {r5,r6,lr}
	// Enable GPIOI Peripheral Clock (bit 8 in AHB4ENR register)
	ldr r6, =RCC_AHB4ENR
	ldr r5, [r6]
	orr r5, #0x100
	str r5, [r6]
	// Make GPIOI Pin13 as output pin (bits 27:26 in MODER register)
	ldr r6, =GPIOI_BASE
	ldr r5, [r6,#GPIOx_MODER]
	and r5, #0xF3FFFFFF
	orr r5, #0x04000000
	str r5, [r6]

  	pop {r5, r6, pc}

// r0 - led state
set_led:
	push {lr} 

	ldr r1, =GPIOI_BASE
	cmp r0, #0
	bne set_led__on
//set_led__off: (implicit)
	mov r2, LED_OFF
	str r2, [r1, GPIOx_BSSR]
	pop {pc} 
set_led__on:
	mov r2, LED_ON
	str r2, [r1, GPIOx_BSSR]
	pop {pc}

// r0 - milliseconds
delay_ms:
    push {lr}
delay_ms__1:
    ldr r1, =LEDDELAY
delay_ms__loop:
    subs r1, r1, #1
    bne delay_ms__loop

    subs r0, r0, #1
    bne delay_ms__1
    
	pop {pc}


.equ RCC_BASE,			0x58024400
.equ RCC_CR, 			0
.equ RCC_CR_HSEON, 		0x00010000
.equ RCC_CFGR,			16
.equ RCC_PLLCKSELR,		40
.equ RCC_CFGR_SWS_Mask, 0x38
go_fast:
	push {lr}
	// setup HSE
	ldr r0, =RCC_BASE
	// Enable High-Speed External clock
	ldr r1, [r0 /*, RCC_CR*/] 
	orr r1, RCC_CR_HSEON
	str r1, [r0] 

	mov r0, 100
	bl delay_ms // wait 100ms for clock to stabilize PEPELA
	
	// setup PLL

	pop {pc}