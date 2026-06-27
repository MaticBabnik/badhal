#include <core/bad.h>

typedef struct {
    u32 stage;
    u32 *address;
    u32 expected;
    u32 actual;
} MT67_Error_t;

/**
 * Runs a set of memory tests on the region (zeros, ones, own address)...
 * Returns 1 on failure and populates the error struct with details, 0 on
 * success.
 */
u32 memtest67(u32 *base, u32 size, MT67_Error_t *err);

/*

const char *const cachetype_str[] = {
    "None", "WriteBack_RWAlloc", "WriteThrough_RAlloc", "WriteBack_RAlloc"
};

void run_memtests() {
    const u32 size = 8 * 1024 * 1024;

    MT67_Error_t err;

    printf("Running memtest67\r\n");
    printf("- Testing %u bytes\r\n\r\n", size);

    u32 nPass = 0;

    for (CacheType_t c = CT_None; c <= CT_WriteBack_RAlloc; c++) {
        printf("Testing ext memory w cache %s...", cachetype_str[c]);

        mem_mpu_setup_sdram(c);

        u32 res = memtest67((u32 *) SDRAM_BASE, size, &err);

        if (res) {
            printf(
                "\r\nmemtest67 failed on stage %d at address %p: expected "
                "0x%08X, got 0x%08X\r\n",
                err.stage, (void *) err.address, err.expected, err.actual
            );
        } else {
            printf("OK\r\n");
            nPass++;
        }
    }

    if (nPass == 4) {
        printf("\r\nAll tests passed!\r\n");
    }
}
*/