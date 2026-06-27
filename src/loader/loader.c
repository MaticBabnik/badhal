#include "loader.h"
#include "../hal/driver/usart.h"

typedef enum {
    S_NONE = 0,
    S_HDR = 1,
    S_CMD = 2,
    S_BODY_VAR_HDR = 4,
    S_BODY = 5
} State_t;

typedef enum {
    B_OK = 0,
    B_ERR = 1,
    H_BEGIN = 0x80,
    H_DATA = 0x81,
    H_VERIFY = 0x82,
} Cmd_t;

typedef enum {
    ERR_OK = 0,
    ERR_NO_TRANSACTION = 1,
    ERR_EXISTING_TRANSACTION = 2,
    ERR_UNALIGNED_WRITE = 3,
    ERR_CRC = 4,
    ERR_OOB = 5,
    ERR_INVALID_SIZE = 6,
    ERR_UNFINISHED_TRANSACTION = 7,

    ERR_INVALID = 127,
    ERR_UNKNOWN = 255,
} Err_t;

const u32 max_size = 128 * MEGA;

static bool check_hdr(u32 i, u8 d) {
    if (i == 0 && d != 0x41) return false;
    if (i == 1 && d != 0x80) return false;
    if (i == 2 && d != 0x01) return false;
    if (i == 3 && d != 0xaa) return false;
    return true;
}

#define CRC32_INIT 0xFFFFFFFFul

static void crc32(u32 *curCrc, const u8 *data, u32 len) {
    u32 crc = *curCrc;

    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (int j = 0; j < 8; j++) {
            u32 mask = -(crc & 1);
            crc = (crc >> 1) ^ (0xEDB88320 & mask);
        }
    }

    *curCrc = crc;
}

static bool hasTransaction = false;
static u32 transactionSize = 0;
static u32 transactionProgress = 0;

static void loader_end_transaction() {
    transactionSize = 0;
    transactionProgress = 0;
    hasTransaction = false;
}

static u8 loader_cmd_begin(u32 size) {
    if (hasTransaction) {
        return ERR_EXISTING_TRANSACTION;
    }

    if (size == 0 || size > max_size) {
        return ERR_INVALID_SIZE;
    }

    hasTransaction = true;
    transactionSize = size;
    transactionProgress = 0;

    const u32 eraseAlign = 8192; // 2*4k
    u32 eraseSize = (size + eraseAlign - 1) & ~(eraseAlign - 1);

    for (u32 addr = 0; addr < eraseSize; addr += eraseAlign) {
        u32 err = loader_cb_erase8k(addr);
        if (err) {
            loader_end_transaction();
            return ERR_UNKNOWN;
        }
    }

    return ERR_OK;
}

static u8 loader_cmd_data(u32 n, u8 *data) {
    if (!hasTransaction) {
        return ERR_NO_TRANSACTION;
    }

    const u32 pageSize = 512; // 2*256
    const u32 pageMask = pageSize - 1;

    // figure out if we are crossing a page boundary
    u32 pageStart = transactionProgress & ~pageMask;
    u32 pageEnd = (transactionProgress + n - 1) & ~pageMask;
    if (pageStart != pageEnd) {
        return ERR_UNALIGNED_WRITE;
    }

    // figure out if we are going out of bounds
    if (transactionProgress + n > transactionSize) {
        return ERR_OOB;
    }

    // write the data
    if (loader_cb_write(transactionProgress, data, n)) {
        return ERR_UNKNOWN;
    }
    transactionProgress += n;

    return ERR_OK;
}

static u8 loader_cmd_verify(u32 expected) {
    if (!hasTransaction) {
        return ERR_NO_TRANSACTION;
    }

    if (transactionProgress != transactionSize) {
        return ERR_UNFINISHED_TRANSACTION;
    }

    u32 crc = CRC32_INIT;
    u8 buf[512];

    for (u32 addr = 0; addr < transactionSize; addr += sizeof(buf)) {
        u32 n = sizeof(buf);
        if (addr + n > transactionSize) {
            n = transactionSize - addr;
        }

        if (loader_cb_read(addr, buf, n)) {
            loader_end_transaction();
            return ERR_UNKNOWN;
        }

        crc32(&crc, buf, n);
    }

    crc = ~crc;
    loader_end_transaction();

    if (crc != expected) {
        return ERR_CRC;
    }

    return ERR_OK;
}

void loader_main(USART_t *usart) {
    // QSPI is fine by now

    u8 buf[513];
    State_t state = S_NONE;
    u32 progress = 0;
    u32 size = 0;
    u8 cmd = 0;

    while (1) {
        u8 in = usart_recv(usart);

        switch (state) {
        case S_NONE:
            if (check_hdr(0, in)) {
                state = S_HDR;
                progress++;
            }
            break;

        case S_HDR:
            if (!check_hdr(progress, in)) {
                if (check_hdr(0, in)) {
                    // restart the header
                    progress = 1;
                } else {
                    state = S_NONE;
                    progress = 0;
                }
            } else if (++progress == 4) {
                state = S_CMD;
                progress = 0;
            }
            break;

        case S_CMD:
            cmd = in;
            progress = 0;
            switch (cmd) {
            case H_BEGIN:
            case H_VERIFY:
                state = S_BODY;
                size = 4;
                break;
            case H_DATA:
                state = S_BODY_VAR_HDR;
                size = 2;
                break;
            default:
                state = S_NONE;
                usart_send(usart, B_ERR);
                usart_send(usart, ERR_INVALID);
                break;
            }
            break;

        case S_BODY_VAR_HDR:
            buf[progress++] = in;
            if (progress == size) {
                size = buf[0] | (buf[1] << 8);

                if (size == 0 || size > 512 || size & 1) {
                    state = S_NONE;
                    progress = 0;
                    usart_send(usart, B_ERR);
                    usart_send(usart, ERR_INVALID_SIZE);
                    break;
                }

                state = S_BODY;
                progress = 0;
            }
            break;

        case S_BODY:
            buf[progress++] = in;
            if (progress == size) {
                u8 err = ERR_INVALID;

                switch (cmd) {
                case H_BEGIN:
                    err = loader_cmd_begin(
                        buf[0] | (buf[1] << 8) | (buf[2] << 16) | (buf[3] << 24)
                    );
                    break;
                case H_DATA:
                    err = loader_cmd_data(size, buf);
                    break;
                case H_VERIFY:
                    err = loader_cmd_verify(
                        buf[0] | (buf[1] << 8) | (buf[2] << 16) | (buf[3] << 24)
                    );
                    break;
                }

                usart_send(usart, err ? B_ERR : B_OK);
                if (err) usart_send(usart, err);

                state = S_NONE;
                progress = 0;
            }
            break;
        }
    }
}
