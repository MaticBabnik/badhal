#pragma once
#include <core/bad.h>

typedef struct {
    R_RW u32 CR;
    R_RW u32 DCR;
    R_RW u32 SR;
    R_RW u32 FCR;
    R_RW u32 DLR;
    R_RW u32 CCR;
    R_RW u32 AR;
    R_RW u32 ABR;
    R_RW union {
        R_RW u32 U32;
        R_RW u16 U16;
        R_RW u8 U8;
    } DR;
    R_RW u32 PSMKR;
    R_RW u32 PSMAR;
    R_RW u32 PIR;
    R_RW u32 LPTR;
} QSPI_t;

#define QSPI_CR_EN (1u << 0)
#define QSPI_CR_ABORT (1u << 1)
#define QSPI_CR_TCEN (1u << 3)
#define QSPI_CR_SSHIFT (1u << 4)
#define QSPI_CR_DFM (1u << 6)
#define QSPI_CR_FSEL (1u << 7)
#define QSPI_CR_FTRESH_Pos 8
#define QSPI_CR_FTRESH_Msk (0xful << QSPI_CR_FTRESH_Pos)
#define QSPI_CR_TEIE (1u << 16)
#define QSPI_CR_TCIE (1u << 17)
#define QSPI_CR_FTIE (1u << 18)
#define QSPI_CR_SMIE (1u << 19)
#define QSPI_CR_TOIE (1u << 20)
#define QSPI_CR_APMS (1u << 22)
#define QSPI_CR_PMM (1u << 23)
#define QSPI_CR_PRESCALER_Pos 24
#define QSPI_CR_PRESCALER_Msk (0xfful << QSPI_CR_PRESCALER_Pos)

#define QSPI_DCR_CKMODE (1u << 0)
#define QSPI_DCR_CSHT_Pos 8
#define QSPI_DCR_CSHT_Msk (0x7ul << QSPI_DCR_CSHT_Pos)
#define QSPI_DCR_FSIZE_Pos 16
#define QSPI_DCR_FSIZE_Msk (0xful << QSPI_DCR_FSIZE_Pos)

#define QSPI_SR_TEF (1u << 0)
#define QSPI_SR_TCF (1u << 1)
#define QSPI_SR_FTF (1u << 2)
#define QSPI_SR_SMF (1u << 3)
#define QSPI_SR_TOF (1u << 4)
#define QSPI_SR_BUSY (1u << 5)
#define QSPI_SR_FLEVEL_Pos 8
#define QSPI_SR_FLEVEL_Msk (0x3ful << QSPI_SR_FLEVEL_Pos)

#define QSPI_FCR_CTEF (1u << 0)
#define QSPI_FCR_CTCF (1u << 1)
#define QSPI_FCR_CSMF (1u << 3)
#define QSPI_FCR_CTOF (1u << 4)

#define QSPI_CCR_INSTRUCTION_Pos 0
#define QSPI_CCR_INSTRUCTION_Msk (0xfful << QSPI_CCR_INSTRUCTION_Pos)
#define QSPI_CCR_IMODE_Pos 8
#define QSPI_CCR_IMODE_Msk (0x3ul << QSPI_CCR_IMODE_Pos)
#define QSPI_CCR_IMODE_NOINSTR (0x0ul << QSPI_CCR_IMODE_Pos)
#define QSPI_CCR_IMODE_1LINE (0x1ul << QSPI_CCR_IMODE_Pos)
#define QSPI_CCR_IMODE_2LINE (0x2ul << QSPI_CCR_IMODE_Pos)
#define QSPI_CCR_IMODE_4LINE (0x3ul << QSPI_CCR_IMODE_Pos)
#define QSPI_CCR_ADMODE_Pos 10
#define QSPI_CCR_ADMODE_Msk (0x3ul << QSPI_CCR_ADMODE_Pos)
#define QSPI_CCR_ADMODE_NOADDR (0x0ul << QSPI_CCR_ADMODE_Pos)
#define QSPI_CCR_ADMODE_1LINE (0x1ul << QSPI_CCR_ADMODE_Pos)
#define QSPI_CCR_ADMODE_2LINE (0x2ul << QSPI_CCR_ADMODE_Pos)
#define QSPI_CCR_ADMODE_4LINE (0x3ul << QSPI_CCR_ADMODE_Pos)
#define QSPI_CCR_ADSIZE_Pos 12
#define QSPI_CCR_ADSIZE_Msk (0x3ul << QSPI_CCR_ADSIZE_Pos)
#define QSPI_CCR_ADSIZE_8BIT (0x0ul << QSPI_CCR_ADSIZE_Pos)
#define QSPI_CCR_ADSIZE_16BIT (0x1ul << QSPI_CCR_ADSIZE_Pos)
#define QSPI_CCR_ADSIZE_24BIT (0x2ul << QSPI_CCR_ADSIZE_Pos)
#define QSPI_CCR_ADSIZE_32BIT (0x3ul << QSPI_CCR_ADSIZE_Pos)
#define QSPI_CCR_ABMODE_Pos 14
#define QSPI_CCR_ABMODE_Msk (0x3ul << QSPI_CCR_ABMODE_Pos)
#define QSPI_CCR_ABMODE_NOAB (0x0ul << QSPI_CCR_ABMODE_Pos)
#define QSPI_CCR_ABMODE_1LINE (0x1ul << QSPI_CCR_ABMODE_Pos)
#define QSPI_CCR_ABMODE_2LINE (0x2ul << QSPI_CCR_ABMODE_Pos)
#define QSPI_CCR_ABMODE_4LINE (0x3ul << QSPI_CCR_ABMODE_Pos)
#define QSPI_CCR_ABSIZE_Pos 16
#define QSPI_CCR_ABSIZE_Msk (0x3ul << QSPI_CCR_ABSIZE_Pos)
#define QSPI_CCR_ABSIZE_8BIT (0x0ul << QSPI_CCR_ABSIZE_Pos)
#define QSPI_CCR_ABSIZE_16BIT (0x1ul << QSPI_CCR_ABSIZE_Pos)
#define QSPI_CCR_ABSIZE_24BIT (0x2ul << QSPI_CCR_ABSIZE_Pos)
#define QSPI_CCR_ABSIZE_32BIT (0x3ul << QSPI_CCR_ABSIZE_Pos)
#define QSPI_CCR_DCYC_Pos 18
#define QSPI_CCR_DCYC_Msk (0x1ful << QSPI_CCR_DCYC_Pos)
#define QSPI_CCR_DMODE_Pos 24
#define QSPI_CCR_DMODE_Msk (0x3ul << QSPI_CCR_DMODE_Pos)
#define QSPI_CCR_DMODE_NODATA (0x0ul << QSPI_CCR_DMODE_Pos)
#define QSPI_CCR_DMODE_1LINE (0x1ul << QSPI_CCR_DMODE_Pos)
#define QSPI_CCR_DMODE_2LINE (0x2ul << QSPI_CCR_DMODE_Pos)
#define QSPI_CCR_DMODE_4LINE (0x3ul << QSPI_CCR_DMODE_Pos)
#define QSPI_CCR_FMODE_Pos 26
#define QSPI_CCR_FMODE_Msk (0x3ul << QSPI_CCR_FMODE_Pos)
#define QSPI_CCR_FMODE_IWRITE (0x0ul << QSPI_CCR_FMODE_Pos)
#define QSPI_CCR_FMODE_IREAD (0x1ul << QSPI_CCR_FMODE_Pos)
#define QSPI_CCR_FMODE_AUTOPOLL (0x2ul << QSPI_CCR_FMODE_Pos)
#define QSPI_CCR_FMODE_MMAP (0x3ul << QSPI_CCR_FMODE_Pos)
#define QSPI_CCR_SIOO (1u << 28)
#define QSPI_CCR_FRCMD (1u << 29)
#define QSPI_CCR_DHHC_Pos 30
#define QSPI_CCR_DHHC (1u << QSPI_CCR_DHHC_Pos)
#define QSPI_CCR_DDRM_Pos 31
#define QSPI_CCR_DDRM (1u << QSPI_CCR_DDRM_Pos)

#define QUADSPI_PIR_Msk 0xfffful
#define QUADSPI_LPTR_Msk 0xfffful
