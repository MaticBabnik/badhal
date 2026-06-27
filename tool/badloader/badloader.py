#!/usr/bin/env python3

import sys
import struct
import zlib
import serial

MAGIC = bytes([0x41, 0x80, 0x01, 0xaa])

CMD_BEGIN = 0x80
CMD_DATA = 0x81
CMD_VERIFY = 0x82

RESP_OK = 0x00
RESP_ERR = 0x01

PAGE = 512

ERR_NAMES = {
    1: "NO_TRANSACTION",
    2: "EXISTING_TRANSACTION",
    3: "UNALIGNED_WRITE",
    4: "CRC",
    5: "OOB",
    6: "INVALID_SIZE",
    7: "UNFINISHED_TRANSACTION",
    127: "INVALID",
    255: "UNKNOWN",
}


def err_name(code):
    return ERR_NAMES.get(code, "0x%02x" % code)


def send_packet(ser, cmd, body=b""):
    ser.write(MAGIC + bytes([cmd]) + body)


def recv_resp(ser):
    resp = ser.read(1)
    if len(resp) != 1:
        raise IOError("timeout waiting for response")
    if resp[0] == RESP_OK:
        return
    err = ser.read(1)
    if len(err) != 1:
        raise IOError("timeout waiting for error code")
    raise IOError("board returned error: %s" % err_name(err[0]))


def cmd_begin(ser, size):
    send_packet(ser, CMD_BEGIN, struct.pack("<I", size))
    recv_resp(ser)


def cmd_data(ser, chunk):
    send_packet(ser, CMD_DATA, struct.pack("<H", len(chunk)) + chunk)
    recv_resp(ser)


def cmd_verify(ser, crc):
    send_packet(ser, CMD_VERIFY, struct.pack("<I", crc))
    recv_resp(ser)


def print_progress(sent, total):
    pct = sent * 100 // total
    bar_len = 40
    filled = pct * bar_len // 100
    bar = "#" * filled + "-" * (bar_len - filled)
    print("\r[%s] %3d%% (%d/%d)" % (bar, pct, sent, total), end="", flush=True)


def main():
    if len(sys.argv) != 4:
        print("usage: %s <baudrate> <filename> <serial port>" % sys.argv[0])
        sys.exit(1)

    baudrate = int(sys.argv[1])
    filename = sys.argv[2]
    port = sys.argv[3]

    with open(filename, "rb") as f:
        data = f.read()

    if len(data) == 0:
        print("empty file, nothing to do")
        sys.exit(1)

    if len(data) % 2 != 0:
        data += b"\x00"

    crc = zlib.crc32(data) & 0xFFFFFFFF
    total = len(data)

    print("uploading %s (%d bytes) @ %d baud on %s" % (filename, total, baudrate, port))

    ser = serial.Serial(port, baudrate, timeout=100)

    cmd_begin(ser, total)

    sent = 0
    print_progress(sent, total)
    while sent < total:
        chunk = data[sent:sent + PAGE]
        cmd_data(ser, chunk)
        sent += len(chunk)
        print_progress(sent, total)

    print()
    print("verifying...")
    cmd_verify(ser, crc)
    print("done!")

    ser.close()


if __name__ == "__main__":
    main()
