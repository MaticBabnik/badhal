// A file that sets up crash handlers
// TODO: this is bsp/crash.h or something

// #define CRASH_IMPL
#ifdef CRASH_IMPL

#ifndef CRASH_PRINTF
#error "CRASH_PRINTF must be defined to use crash.h"
#endif
#ifndef CRASH_PUTS
#error "CRASH_PUTS must be defined to use crash.h"
#endif

const char *const genericCrashMsg =
    "\x1b[97;44m\x1b[K\r\n"
    "            \r\n"
    "   o   /    \r\n"
    "      |     \r\n"
    "   o   \\    \r\n"
    "            \r\n"
    "   Your STM32H750-disco ran into a problem and needs to restart.\r\n"
    "   \r\n"
    "    For more information about this issue and possible fixes, visit\r\n"
    "    https://ubel.weebify.tv/badcrash.html\r\n"
    "\r\n"
    "\r\n";

static inline unsigned int crash_read_reg(unsigned int addr) {
    return *(volatile unsigned int *) addr;
}

static void crash_dump_common(const char *faultName) {
    u32 cfsr = SCB->CFSR;
    u32 hfsr = SCB->HFSR;
    u32 mmfar = SCB->MMFAR;
    u32 bfar = SCB->BFAR;

    CRASH_PRINTF("%s\r\n", faultName);

    if (cfsr & SCB_CFSR_MM_MMARVALID) {
        CRASH_PRINTF("- while accessing 0x%08X (MMFAR)\r\n", mmfar);
    }

    if (cfsr & SCB_CFSR_BUS_BFARVALID) {
        CRASH_PRINTF("- while accessing 0x%08X (BFAR)\r\n", bfar);
    }

    CRASH_PRINTF("- CFSR=0x%08X HFSR=0x%08X\r\n", cfsr, hfsr);
    CRASH_PRINTF("- AFSR=0x%08X\r\n", SCB->AFSR);
}

static void crash_dump_memmanage() {
    u32 cfsr = SCB->CFSR;

    if (cfsr & SCB_CFSR_MM_IACCVIOL) CRASH_PUTS("- SCB_CFSR_MM_IACCVIOL\r\n");
    if (cfsr & SCB_CFSR_MM_DACCVIOL) CRASH_PUTS("- SCB_CFSR_MM_DACCVIOL\r\n");
    if (cfsr & SCB_CFSR_MM_MUNSTKERR) CRASH_PUTS("- SCB_CFSR_MM_MUNSTKERR\r\n");
    if (cfsr & SCB_CFSR_MM_MSTKERR) CRASH_PUTS("- SCB_CFSR_MM_MSTKERR\r\n");
    if (cfsr & SCB_CFSR_MM_MLSPERR) CRASH_PUTS("- SCB_CFSR_MM_MLSPERR\r\n");
}

static void crash_dump_busfault() {
    u32 cfsr = SCB->CFSR;

    if (cfsr & SCB_CFSR_BUS_IBUSERR) CRASH_PUTS("- SCB_CFSR_BUS_IBUSERR\r\n");
    if (cfsr & SCB_CFSR_BUS_PRECISERR)
        CRASH_PUTS("- SCB_CFSR_BUS_PRECISERR\r\n");
    if (cfsr & SCB_CFSR_BUS_IMPRECISERR)
        CRASH_PUTS("- SCB_CFSR_BUS_IMPRECISERR\r\n");
    if (cfsr & SCB_CFSR_BUS_UNSTKERR) CRASH_PUTS("- SCB_CFSR_BUS_UNSTKERR\r\n");
    if (cfsr & SCB_CFSR_BUS_STKERR) CRASH_PUTS("- SCB_CFSR_BUS_STKERR\r\n");
    if (cfsr & SCB_CFSR_BUS_LSPERR) CRASH_PUTS("- SCB_CFSR_BUS_LSPERR\r\n");
}

static void crash_dump_usagefault() {
    u32 cfsr = SCB->CFSR;

    if (cfsr & SCB_CFSR_USG_UNDEFINSTR)
        CRASH_PUTS("- SCB_CFSR_USG_UNDEFINSTR\r\n");
    if (cfsr & SCB_CFSR_USG_INVSTATE) CRASH_PUTS("- SCB_CFSR_USG_INVSTATE\r\n");
    if (cfsr & SCB_CFSR_USG_INVPC) CRASH_PUTS("- SCB_CFSR_USG_INVPC\r\n");
    if (cfsr & SCB_CFSR_USG_NOCP) CRASH_PUTS("- SCB_CFSR_USG_NOCP\r\n");
    if (cfsr & SCB_CFSR_USG_UNALIGNED)
        CRASH_PUTS("- SCB_CFSR_USG_UNALIGNED\r\n");
    if (cfsr & SCB_CFSR_USG_DIVBYZERO)
        CRASH_PUTS("- SCB_CFSR_USG_DIVBYZERO\r\n");
}

static void crash_halt() {
    CRASH_PUTS("\x1b[0m\r\nSystem halted.\r\n");
    for (;;) {
    }
}

void HardFault_Handler() {
    CRASH_PUTS(genericCrashMsg);
    crash_dump_common("HARD_FAULT");

    if (SCB->HFSR & SCB_HFSR_VECTTBL) CRASH_PUTS("- SCB_HFSR_VECTTBL\r\n");
    if (SCB->HFSR & SCB_HFSR_FORCED) CRASH_PUTS("- SCB_HFSR_FORCED\r\n");
    if (SCB->HFSR & SCB_HFSR_DEBUGEVT) CRASH_PUTS("- SCB_HFSR_DEBUGEVT\r\n");

    crash_halt();
}

void MemManage_Handler() {
    CRASH_PUTS(genericCrashMsg);
    crash_dump_common("MEM_MANAGE");
    crash_dump_memmanage();
    crash_halt();
}

void BusFault_Handler() {
    CRASH_PUTS(genericCrashMsg);
    crash_dump_common("BUS_FAULT");
    crash_dump_busfault();
    crash_halt();
}

void UsageFault_Handler() {
    CRASH_PUTS(genericCrashMsg);
    // crash_dump_common("USAGE_FAULT");
    crash_dump_usagefault();
    crash_halt();
}

#endif