#pragma once
#include "../bad.h"

struct FLASH_t
{
    R_RW u32 ACR;
    R_RW u32 KEYR1;
    R_RW u32 OPTKEYR;
    R_RW u32 CR1;
    R_RW u32 SR1;
    R_RW u32 CCR1;
    R_RW u32 OPTCR;
    R_RW u32 OPTSR_CUR;
    R_RW u32 OPTSR_PRG;
    R_RW u32 OPTCCR;
    R_RW u32 PRAR_CUR1;
    R_RW u32 PRAR_PRG1;
    R_RW u32 SCAR_CUR1;
    R_RW u32 SCAR_PRG1;
    R_RW u32 WPSN_CUR1;
    R_RW u32 WPSN_PRG1;
    R_RW u32 BOOT_CUR;
    R_RW u32 BOOT_PRG;
    u32 reserved0[2];
    R_RW u32 CRCCR1;
    R_RW u32 CRCSADD1;
    R_RW u32 CRCEADD1;
    R_RW u32 CRCDATA;
    R_RW u32 ECC_FA1;
    u32 reserved;
    R_RW u32 OTPBL_CUR;
    R_RW u32 OTPBL_PRG;
    u32 reserved1[37];
    R_RW u32 KEYR2;
    u32 reserved2;
    R_RW u32 CR2;
    R_RW u32 SR2;
    R_RW u32 CCR2;
    u32 reserved3[4];
    R_RW u32 PRAR_CUR2;
    R_RW u32 PRAR_PRG2;
    R_RW u32 SCAR_CUR2;
    R_RW u32 SCAR_PRG2;
    R_RW u32 WPSN_CUR2;
    R_RW u32 WPSN_PRG2;
    u32 reserved4[4];
    R_RW u32 CRCCR2;
    R_RW u32 CRCSADD2;
    R_RW u32 CRCEADD2;
    R_RW u32 CRCDATA2;
    R_RW u32 ECC_FA2;
};

#define FLASH_ACR_LATENCTY_Mask 0xFUL
