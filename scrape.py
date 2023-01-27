#!/usr/bin/env python3

import csv
import struct
import sys
import json
import binascii


def size8(size):
    if size <= 0 or size > 0x100:
        return None
    if size == 0x100: size = 0
    return size


def pack24(x):
    assert x < (1 << 24)
    return struct.pack('<HB', x & 0xffff, x >> 16)


class InsufficientData(Exception):
    pass


class MalformedPacket(Exception):
    pass


def tostr(buff):
    if type(buff) is str:
        return buff
    elif type(buff) is bytearray or type(buff) is bytes:
        return ''.join([chr(b) for b in buff])
    else:
        assert 0, type(buff)


def hexdump(data, label=None, indent='', address_width=8, f=sys.stdout):
    def isprint(c):
        return c >= ' ' and c <= '~'

    if label:
        print(label)

    if data is None:
        print("%sNone" % indent)
        return

    bytes_per_half_row = 8
    bytes_per_row = 16
    data = bytearray(data)
    data_len = len(data)

    def hexdump_half_row(start):
        left = max(data_len - start, 0)

        real_data = min(bytes_per_half_row, left)

        f.write(''.join('%02X ' % c for c in data[start:start + real_data]))
        f.write(''.join('   ' * (bytes_per_half_row - real_data)))
        f.write(' ')

        return start + bytes_per_half_row

    pos = 0
    while pos < data_len:
        row_start = pos
        f.write(indent)
        if address_width:
            f.write(('%%0%dX  ' % address_width) % pos)
        pos = hexdump_half_row(pos)
        pos = hexdump_half_row(pos)
        f.write("|")
        # Char view
        left = data_len - row_start
        real_data = min(bytes_per_row, left)

        f.write(''.join([
            c if isprint(c) else '.'
            for c in tostr(data[row_start:row_start + real_data])
        ]))
        f.write((" " * (bytes_per_row - real_data)) + "|\n")


class StructStreamer:
    def __init__(self, buf, verbose=False):
        self.buf = bytearray(buf)
        self.len = len(self.buf)
        self.d = {}
        self.verbose = verbose

    def done(self):
        assert len(self.buf) == 0
        return self.d

    def popped(self):
        """Number of bytes consumed so far"""
        return self.len - len(self.buf)

    def pop_n(self, n):
        self.verbose and hexdump(self.buf, "pop %u" % n)
        assert len(
            self.buf) >= n, "Only %u bytes left, need %u" % (len(self.buf), n)
        v = self.buf[0:n]
        # TypeError: 'bytearray' object doesn't support item deletion
        # del self.buf[0:n]
        self.buf = self.buf[n:]
        return v

    def get(self, k):
        return self.d[k]

    def assert_bytes(self, buf):
        got = self.pop_n(len(buf))
        assert got == buf

    def assert_str(self, want):
        got = self.pop_n(len(want))
        got = tostr(got)
        assert want == got, "Wanted %s got %s" % (want, got)

    def bytes(self, k, n):
        """
        Add n reserved / unknown bytes
        """
        v = self.pop_n(n)
        self.d[k] = v
        return v

    def res(self, n):
        """
        Add n reserved / unknown bytes
        """
        if self.len < 10:
            k = "res%01u" % self.popped()
        elif self.len < 100:
            k = "res%02u" % self.popped()
        else:
            k = "res%03u" % self.popped()
        return bytes(k, n)

    def strn(self, k, n):
        """
        pop string of exactly n characters
        """
        v = tostr(self.pop_n(n))
        self.d[k] = v
        return v

    def strn0(self, k, n):
        """
        pop string of exactly n characters, but truncate at first 0, if any
        """
        buf = self.pop_n(n)
        i = buf.find(0)
        if i >= 0:
            buf = buf[0:i]
        v = tostr(buf)
        self.d[k] = v
        return v

    def u32b(self, k):
        v = struct.unpack('>I', self.pop_n(4))[0]
        self.d[k] = v
        return v

    def u32l(self, k):
        v = struct.unpack('<I', self.pop_n(4))[0]
        self.d[k] = v
        del self.buf[0:4]
        return v

    def u16b(self, k):
        v = struct.unpack('>H', self.pop_n(2))[0]
        self.d[k] = v
        return v

    def u16l(self, k):
        v = struct.unpack('<H', self.pop_n(2))[0]
        self.d[k] = v
        return v

    def u8(self, k):
        v = self.buf[0]
        self.d[k] = v
        del self.buf[0:1]
        return v


def print_silicon_sig(buf):
    """
    RL78/G13
    25.5.5 Description of signature data
    Table 25-9. Signature Data List

    Device code
    3 bytes
    The serial number assigned to the device
    
    Device name
    10 bytes
    Device name (ASCII code)

    Code flash memory area last address
    3 bytes
    Last address of code flash memory area
    (Sent from lower address.
    Example. 00000H to 0FFFFH (64 KB) => FFH, 1FH, 00H)

    Data flash memory area last address
    3 bytes
    Last address of data flash memory area
    (Sent from lower address.
    Example. F1000H to F1FFFH (4 KB) => FFH, 1FH, 0FH)

    Firmware version
    3 bytes
    Version information of firmware for programming
    (Sent from upper address.
    Example. From Ver. 1.23 => 01H, 02H, 03H)
    """
    assert len(buf) == 22
    ss = StructStreamer(buf, verbose=0)
    ss.bytes("device_code", 3)
    ss.strn("device_name", 10)
    # Example. 00000H to 0FFFFH (64 KB) => FFH, 1FH, 00H)
    v = ss.bytes("code_flash_addr_hi_raw", 3)
    ss.d["code_flash_addr_hi"] = (v[2] << 16 | v[1] << 8 | v[0])
    # Example. F1000H to F1FFFH (4 KB) => FFH, 1FH, 0FH)
    v = ss.bytes("data_flash_addr_hi_raw", 3)
    ss.d["data_flash_addr_hi"] = (v[2] << 16 | v[1] << 8 | v[0])
    v = ss.bytes("fw_ver_raw", 3)
    ss.d["fw_ver"] = "%u.%u%u" % (v[0], v[1], v[2])

    if 1:
        print("Device code:", ss.get("device_code"))
        print("Device name:", ss.get("device_name"))
        print("Code flash address hi: 0x%06X" % ss.get("code_flash_addr_hi"))
        print("Data flash address hi: 0x%06X" % ss.get("data_flash_addr_hi"))
        # print("Data flash address hi", ss.get("data_flash_addr_hi_raw"))
        # print("Data flash address hi", ss.get("data_flash_addr_hi_raw"))
        print("FW ver:", ss.get("fw_ver"))

    return ss


def print_security_get(raw_rx):
    """
    FLG: Security flag
    BOT: Boot block cluster block number
    SSL: Flash shield window start block number (Lower)
    SSH: Flash shield window start block number (Higher)
    SEL: Flash shield window end block number (Lower)
    SEH: Flash shield window end block number (Higher)
    RES: Invalid data
    """
    ss = StructStreamer(raw_rx, verbose=0)
    ss.u8("FLG")
    ss.u8("BOT")
    ss.u16l("SS")
    ss.u16l("SE")
    ss.u8("RES1")
    ss.u8("RES2")

    print("FLG 0x%02X" % ss.get("FLG"))
    assert ss.get("FLG") & 0xE8 == 0xE8, "Unexpected bits low"

    flag = int(bool(ss.get("FLG") & 0x10))
    flag_str = {1: "Enables programming", 0: "Disable, programming"}[flag]
    print("  Programming disable: %u (%s)" % (flag, flag_str))

    flag = int(bool(ss.get("FLG") & 0x04))
    flag_str = {1: "Enables block erase", 0: "Disable block eras"}[flag]
    print("  Block erase disable: %u (%s)" % (flag, flag_str))

    flag = int(bool(ss.get("FLG") & 0x02))
    flag_str = {
        1: "Enables boot block cluster rewrite",
        0: "Disable boot block cluster rewrite"
    }[flag]
    print("  Boot block cluster rewrite disable: %u (%s)" % (flag, flag_str))

    flag = int(bool(ss.get("FLG") & 0x01))
    flag_str = {1: "Provided", 0: "None"}[flag]
    print("  Boot area exchange: %u (%s)" % (flag, flag_str))

    # "Boot block cluster block number"
    print("BOT 0x%02X" % ss.get("BOT"))
    # hmm this doesn't appear to be right
    # block_size = 0x400
    # print("  Boot address: 0x%06X" % (ss.get("BOT") * block_size))
    print("SS 0x%04X" % ss.get("SS"))
    print("SE 0x%04X" % ss.get("SE"))
    print("RES1 0x%02X" % ss.get("RES1"))
    print("RES2 0x%02X" % ss.get("RES2"))


def print_block_blank_check(raw_tx, raw_rx, indent=""):
    """
    Figure 3-18. Block Blank Check Command Frame (from Programmer to RL78)
    WARNING: the diagram and the commands disagree
    And don't even look at the other pdf which claims only 3 address bytes...
    I just arranged until output looked right
    """
    def myprint(s):
        print("%s%s" % (indent, s))

    start_address = raw_tx[3] << 16 | raw_tx[2] << 8 | raw_tx[1] << 0
    end_address = raw_tx[6] << 16 | raw_tx[5] << 8 | raw_tx[4] << 0
    d01 = raw_tx[7]
    myprint("Address: 0x%06X : 0x%06X" % (start_address, end_address))
    if d01 == 0:
        myprint("D01: Specified block")
    elif d01 == 1:
        myprint("D01: Specified block and flash option")
    else:
        assert 0

    assert len(raw_rx) == 1
    res = raw_rx[0]
    # 4.9.3 Status at processing completion
    if res == ProtoA.ST_ACK:
        myprint("Blank")
    elif res == ProtoA.ST_BLANK_ERR:
        myprint("Not blank")
    else:
        myprint("FIXME")


def print_baudrate_set(raw_tx, raw_rx):
    print("Baud rate")
    # hexdump(raw_tx, indent="tx     ")
    # hexdump(raw_rx, indent="rx     ")

    br2i = {115200: 0, 250000: 1, 500000: 2, 1000000: 3}
    i2br = {v: k for k, v in br2i.items()}

    print("Baud: %s" % i2br[raw_tx[1]])
    print("Voltage: %u" % raw_tx[2])

    st1 = raw_rx[0]

    if st1 == ProtoA.ST_ACK:
        print("ST1: ACK / ok")
    elif st1 == ProtoA.ST_PROTECT_ERR:
        print("ST1: PROTECT_ERR")
    else:
        print("ST1: 0x%02X" % st1)

    assert len(raw_rx) == 3
    st1, d01, d02 = raw_rx
    print("  ST1: 0x%02X" % st1)
    # OCD: got 0x08
    print("  D01: 0x%02X (%u MHz)" % (d01, d01))
    fstr = {0: "full-speed mode", 1: "wide-voltage mode"}[d02]
    print("  D02: 0x%02X (%s)" % (d02, fstr))


"""
RL78 Microcontroller (RL78 Protocol A) Programmer Edition
R01AN0815EJ0100
"""


class ProtoA:
    """
    header = self.SOH if is_cmd else self.STX
    trailer = self.ETX if last_data else self.ETB
    """

    SOH = 0x01
    STX = 0x02
    ETB = 0x17
    ETX = 0x03

    COM_RESET = 0x00
    COM_19 = 0x19  # undocumented cmd. sets FSSQ=2
    COM_ERASE = 0x22
    COM_PROG = 0x40
    COM_VERIFY = 0x13
    COM_BLANK_CHECK = 0x32
    COM_BAUDRATE_SET = 0x9a
    COM_SILICON_SIG = 0xc0
    COM_SEC_SET = 0xa0
    COM_SEC_GET = 0xa1
    COM_SEC_RLS = 0xa2
    COM_CHECKSUM = 0xb0
    cmd_i2s = {
        COM_RESET: "Reset",
        COM_19: "CMD19",
        COM_ERASE: "Erase",
        COM_PROG: "Program",
        COM_VERIFY: "Verify",
        COM_BLANK_CHECK: "Block Blank Check",
        COM_BAUDRATE_SET: "Baudrate Set",
        COM_SILICON_SIG: "Silicon Signature",
        COM_SEC_SET: "Security Set",
        COM_SEC_GET: "Security Get",
        COM_SEC_RLS: "Security Release",
        COM_CHECKSUM: "Checksum",
    }

    ST_COM_NUM_ERR = 0x04
    ST_PARAM_ERR = 0x05
    ST_ACK = 0x06
    ST_SUM_ERR = 0x07
    ST_VERIFY_ERR = 0x0f
    ST_PROTECT_ERR = 0x10
    ST_NACK = 0x15
    ST_ERASE_ERR = 0x1a
    ST_BLANK_ERR = 0x1b
    ST_WRITE_ERR = 0x1c
    st_i2s = {
        ST_COM_NUM_ERR: "COM_NUM_ERR",
        ST_PARAM_ERR: "PARAM_ERR",
        ST_ACK: "ACK",
        ST_SUM_ERR: "SUM_ERR",
        ST_VERIFY_ERR: "VERIFY_ERR",
        ST_PROTECT_ERR: "PROTECT_ERR",
        ST_NACK: "NACK",
        ST_ERASE_ERR: "ERASE_ERR",
        ST_BLANK_ERR: "BLANK_ERR",
        ST_WRITE_ERR: "WRITE_ERR",
    }

    @staticmethod
    def checksum(data):
        csum = 0
        for d in data:
            csum -= d
            csum &= 0xff
        return csum

    @staticmethod
    def checksum16(data):
        csum = 0
        for d in data:
            csum -= d
            csum &= 0xffff
        return csum

    @staticmethod
    def next_frame(buf, verbose=0):
        def read(n=1):
            ret = buf[0:n]
            del buf[0:n]
            return ret

        def peek(n=1, off=0):
            ret = buf[off:off + n]
            return ret

        def read8():
            return read()[0]

        def peek8():
            return peek()[0]

        # hexdump(buf[0:16], "Next frame")

        while True:
            drops = 0
            while True:
                if len(buf) == 0:
                    raise InsufficientData("Failed to find STX")
                header = read8()
                if header == ProtoA.STX or ProtoA.SOH:
                    break
                drops += 1
            if drops:
                verbose and print("WARNING: dropped %u bytes" % drops)

            try:
                len_b = peek8()
                # data + checksum + ETX
                recv_len = len_b + 2
                data = peek(recv_len, off=1)
                #print('recv %self' % (binascii.hexlify(data)))
                # Also need checksum, ETX
                if len(data) < len_b + 2:
                    raise MalformedPacket(
                        "Insufficient data for complete packet")
                if ProtoA.checksum(bytearray([len_b]) +
                                   data[:len_b]) != data[len_b]:
                    raise MalformedPacket("bad checksum")
                if data[len_b + 1] != ProtoA.ETX:
                    raise MalformedPacket("bad footer")
                header = {ProtoA.STX: "STX", ProtoA.SOH: "SOF"}[header]
            except MalformedPacket as e:
                verbose and print("WARNING: malformed packet:", e)
                read8()
                continue

            read(len_b + 3)
            return (header, data[:len_b])

    @staticmethod
    def transactions(buf, verbose=0):
        while True:
            try:
                print("")
                try:
                    t, frame = ProtoA.next_frame(buf)
                except InsufficientData:
                    break
                if t != "SOF":
                    verbose and print("WARNING: expect SOF, got %s" % t)
                    continue
                hexdump(frame, indent=t + "    ")
                cmd = ProtoA.cmd_i2s[frame[0]]
                print("RX command:", cmd)

                def expect_stx():
                    t, frame = ProtoA.next_frame(buf)
                    hexdump(frame, indent=t + "    ")
                    if t != "STX":
                        raise MalformedPacket("WARNING: expect STX, got %s" %
                                              (t, ))
                    return frame

                def expect_ack():
                    frame = expect_stx()
                    if len(frame) != 1 or frame[0] != ProtoA.ST_ACK:
                        raise MalformedPacket("WARNING: expect ACK, got %s" %
                                              (frame, ))

                if cmd == "Silicon Signature":
                    expect_ack()
                    raw_rx = expect_stx()
                    print_silicon_sig(raw_rx)
                elif cmd == "Block Blank Check":
                    raw_rx = expect_stx()
                    print_block_blank_check(frame, raw_rx)
                elif cmd == "Security Get":
                    expect_ack()
                    raw_rx = expect_stx()
                    print_security_get(raw_rx)
                elif cmd == "Baudrate Set":
                    raw_rx = expect_stx()
                    print_baudrate_set(frame, raw_rx)
                else:
                    print("WARNING: unhandled command")

            except MalformedPacket as e:
                print("WARNING: malformed packet:", e)

    """
    @staticmethod
    def next_device_frame(buf):
        header = self.SOH if is_cmd else self.STX
        trailer = self.ETX if last_data else self.ETB
        LEN = size8(len(data))
        SUM = self._checksum(struct.pack('B', LEN) + data)
        cmd = struct.pack('BB%dBBB' % (len(data)), header, LEN, *data, SUM,
                          trailer)
        #print('send %self' % (binascii.hexlify(cmd)))
        self.port.write(cmd)
        # discard the loopback bytes
        self.read_all(len(cmd))
        return self.recv_frame()
    """


'''
class ProtoOCD:
    """
    NOTE: ProtoA.SET_BAUDRATE (and formatted as ProtoA) is valid in ProtoOCD
    """
    SYNC = 0x00
    PING = 0x90
    UNLOCK = 0x91
    READ = 0x92
    WRITE = 0x93
    EXEC = 0x94
    EXIT_RETI = 0x95
    EXIT_RAM = 0x97
    cmd_i2s = {
        SYNC: "SYNC",
        PING: "PING",
        UNLOCK: "UNLOCK",
        READ: "READ",
        WRITE: "XXX",
        EXEC: "EXEC",
        EXIT_RETI: "EXIT_RETI",
        EXIT_RAM: "EXIT_RAM",
    }



    PONG = bytes([3, 3])

    ST_UNLOCK_ALREADY = 0xf0
    ST_UNLOCK_LOCKED = 0xf1
    ST_UNLOCK_OK = 0xf2
    ST_UNLOCK_SUM = 0xf3
    ST_UNLOCK_NG = 0xf4



    @staticmethod
    def transactions(buf, verbose=0):
        while True:
            try:
                print("")
                try:
                    t, frame = ProtoOCD.next_frame(buf)
                except InsufficientData:
                    break
'''


def load_csv(fn):
    with open(fn, newline='') as csvfile:
        csvfile.readline()
        for row in csv.reader(csvfile):
            yield row


def get_csv_packets(fn):
    buf = bytearray()
    print("Finding bytes...")
    for row in load_csv(fn):
        # ['0.016552800000000', '0x30', '', '']
        b = int(row[1], 0)
        buf += bytearray([b])

    # hexdump(buf)
    print("")
    while buf:
        print("")
        if 0:
            print("")
            hexdump(buf)

        try:
            x = ProtoA.transactions(buf)
            # yield t, frame
        except MalformedPacket:
            print("WARNING: malfored packet")


def get_json_packets(fn):
    """
    j = {
        "silicon_sig": sig,
        "security_get": {
            "raw_rx": sec_raw.hex(),
        },
        "block_blank_checks": bbcs,
        }
    j["silicon_sig"]["raw_rx"] = sig_raw
    """
    j = json.load(open(fn, "r"))

    print("")
    print("Silicon Signature")
    print_silicon_sig(binascii.unhexlify(j["silicon_sig"]["raw_rx"]))
    print("")
    print("Security Get")
    print_security_get(binascii.unhexlify(j["security_get"]["raw_rx"]))
    print("")
    print("Block Blank Checks")
    for addr, bbc in j["block_blank_checks"].items():
        print(addr)
        print_block_blank_check(binascii.unhexlify(bbc["raw_tx"]),
                                binascii.unhexlify(bbc["raw_st1"]),
                                indent="  ")


def main():
    import argparse

    parser = argparse.ArgumentParser(description="Dump packets from CSV")
    parser.add_argument("--verbose", action="store_true")
    parser.add_argument("fn")
    args = parser.parse_args()

    if args.fn.find(".csv") >= 0:
        get_csv_packets(args.fn)
    elif args.fn.find(".json") >= 0:
        get_json_packets(args.fn)
    else:
        raise Exception("need .csv or .json")
    """
    for _x in get_packets(args.csv):
        # print(packet)
        # hexdump(packet, indent=t + "    ")
        pass
    """


if __name__ == "__main__":
    main()
