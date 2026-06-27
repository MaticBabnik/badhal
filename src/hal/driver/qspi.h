#pragma once
#include <core/bad.h>
#include "../hwdef/qspi.h"

#define FCMD_RSTEN 0x66
#define FCMD_RST 0x99
#define FCMD_READ_ID 0x9F
#define FCMD_WREN 0x06
#define FCMD_RDSR 0x05
#define FCMD_RDFSR 0x70
#define FCMD_EN4B 0xB7
#define FCMD_SUBSEC_ER_4B 0x21
#define FCMD_QUAD_PROG_4B 0x34
#define FCMD_QUAD_READ_4B 0x6C
#define FCMD_ENTER_QUAD 0x35

typedef enum {
    QM_IWrite = 0,
    QM_IRead = 1,
    QM_APoll = 2,
    QM_MMap = 3,
} QSPI_Mode_t;

typedef enum {
    QDD_SDR = 0,
    QDD_DDR = 2,
    QDD_DDR_Hold = 3,
} QSPI_DDRMode_t;

typedef enum {
    CM_None = 0,
    CM_1Line = 1,
    CM_2Line = 2,
    CM_4Line = 3,
} QSPI_CMode_t;

typedef enum {
    CS_8bit = 0,
    CS_16bit = 1,
    CS_24bit = 2,
    CS_32bit = 3,
} QSPI_CSize_t;

void qspi_disable();
void qspi_enable();
void qspi_wait_busy();

// QSPI->DCR
void qspi_devcfg(u32 sizeBytes, u32 cshtCycles, bool ckMode);
// QSPI->CR
void qspi_cfg(u32 prescaler, bool dualflash, bool shift);
// QSPI->CCR
void qspi_cmd(
    QSPI_DDRMode_t ddr, // DDRM+DHHC
    QSPI_Mode_t fMd,    // FMODE
    QSPI_CMode_t dMd,   // DMODE
    u8 dummyCycles,     // DCYC
    QSPI_CSize_t abSz,  // ABSIZE
    QSPI_CMode_t abMd,  // ABMODE
    QSPI_CSize_t adSz,  // ADSIZE
    QSPI_CMode_t adMd,  // ADMODE
    QSPI_CMode_t iMd,   // IMODE
    u8 instruction      // INSTRUCTION
);

bool qspi_fifo_nonempty();
void qspi_drain_fifo();
void qspi_end_transfer();
void qspi_cmd_simple(u8 instruction);
void qspi_read_bytes(u8 *dst, u32 n);
void qspi_addr(u32 addr);
void qspi_setup_read(u32 n);
u16 qspi_dual_read_sr();
void qspi_wren();
void qspi_wait_wip();
void qspi_write_bytes(const u8 *data, u32 n);
void qspi_write_bytes(const u8 *data, u32 n);