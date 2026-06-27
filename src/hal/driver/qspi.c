#include "qspi.h"
#include "../badhal.h"

void qspi_wait_busy() {
    while (QSPI->SR & QSPI_SR_BUSY) {
    }
}

void qspi_disable() {
    QSPI->CR &= ~QSPI_CR_EN;
    qspi_wait_busy();
}

void qspi_devcfg(u32 sizeBytes, u32 cshtCycles, bool ckMode) {
    ASSERT_RANGE(cshtCycles, 1, 8, "invalid csht");
    ASSERT_RANGE(sizeBytes, 1, 0x8000'0000, "invalid size");

    u32 size = 30 - a_clz(sizeBytes); // sizeBytes = 2^(fsize+1)

    QSPI->DCR = (size << QSPI_DCR_FSIZE_Pos)
                | ((cshtCycles - 1) << QSPI_DCR_CSHT_Pos)
                | (ckMode ? QSPI_DCR_CKMODE : 0);

    qspi_wait_busy();
}

// TODO: this is more or less hardcoded for our usecase
void qspi_cfg(u32 prescaler, bool dualflash, bool shift) {
    ASSERT_RANGE(prescaler, 1, 256, "invalid prescaler");

    mreg(
        &QSPI->CR, QSPI_CR_PRESCALER_Msk | QSPI_CR_SSHIFT | QSPI_CR_DFM,
        ((prescaler - 1) << QSPI_CR_PRESCALER_Pos)
            | (dualflash ? QSPI_CR_DFM : 0) | (shift ? QSPI_CR_SSHIFT : 0)
    );
}

void qspi_enable() {
    QSPI->CR |= QSPI_CR_EN;
    qspi_wait_busy();
}

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
) {
    ASSERT_RANGE(dummyCycles, 0, 31, "invalid dummy cycles");

    qspi_wait_busy();

    QSPI->CCR = (ddr << QSPI_CCR_DHHC_Pos) | (fMd << QSPI_CCR_FMODE_Pos)
                | (dMd << QSPI_CCR_DMODE_Pos)
                | (dummyCycles << QSPI_CCR_DCYC_Pos)
                | (abSz << QSPI_CCR_ABSIZE_Pos) | (abMd << QSPI_CCR_ABMODE_Pos)
                | (adSz << QSPI_CCR_ADSIZE_Pos) | (adMd << QSPI_CCR_ADMODE_Pos)
                | (iMd << QSPI_CCR_IMODE_Pos)
                | (instruction << QSPI_CCR_INSTRUCTION_Pos);
}

bool qspi_fifo_nonempty() {
    return (QSPI->SR & QSPI_SR_FTF) != 0;
}

void qspi_drain_fifo() {
    volatile u32 tmp;
    while (qspi_fifo_nonempty()) {
        tmp = QSPI->DR.U8;
    }
}

void qspi_end_transfer() {
    while (!(QSPI->SR & QSPI_SR_TCF)) {
    }

    QSPI->FCR = QSPI_FCR_CTCF;
}

void qspi_cmd_simple(u8 instruction) {
    qspi_cmd(
        QDD_SDR, QM_IWrite, CM_None, 0, CS_8bit, CM_None, CS_8bit, CM_None,
        CM_1Line, instruction
    );

    qspi_end_transfer();
}

void qspi_read_bytes(u8 *dst, u32 n) {
    for (u32 i = 0; i < n; i++) {
        while (!qspi_fifo_nonempty()) {
        }
        dst[i] = QSPI->DR.U8;
    }

    qspi_end_transfer();
}

void qspi_addr(u32 addr) {
    QSPI->AR = addr;
}

void qspi_setup_read(u32 n) {
    qspi_wait_busy();
    QSPI->DLR = n - 1;
}

u16 qspi_dual_read_sr() {
    u8 tmp[2];

    qspi_setup_read(2);
    qspi_cmd(
        QDD_SDR, QM_IRead, CM_1Line, 0, CS_8bit, CM_None, CS_8bit, CM_None,
        CM_1Line, 0x05
    );
    qspi_read_bytes(tmp, 2);

    return (tmp[1] << 8) | tmp[0];
}

void qspi_wren() {
    qspi_cmd(
        QDD_SDR, QM_IWrite, CM_None, 0, CS_8bit, CM_None, CS_8bit, CM_None,
        CM_1Line, FCMD_WREN
    );
    qspi_end_transfer();
}

void qspi_wait_wip() {
    u8 sr[2];
    do {
        qspi_setup_read(2);
        qspi_cmd(
            QDD_SDR, QM_IRead, CM_1Line, 0, CS_8bit, CM_None, CS_8bit, CM_None,
            CM_1Line, FCMD_RDSR
        );
        qspi_read_bytes(sr, 2);
    } while ((sr[0] | sr[1]) & 0x01); // WIP = bit0
}

void qspi_write_bytes(const u8 *data, u32 n) {
    volatile u8 *dr = (volatile u8 *) &QSPI->DR; // byte access
    for (u32 i = 0; i < n; i++) {
        // wait until FIFO has room (FTF set when <= FTHRES free)
        while (!(QSPI->SR & QSPI_SR_FTF))
            ;
        *dr = data[i];
    }
    // wait for the transfer to drain/complete
    qspi_wait_busy();
}