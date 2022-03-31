#!/usr/bin/env python3
from pyftdi.gpio import GpioController
import serial
import time, struct, binascii, code, os


def delay(amount):
    now = start = time.perf_counter()
    while True:
        now = time.perf_counter()
        if now - start >= amount:
            return


# for C232HM-DDHSL-0 cable
WIRE_ORANGE = 1 << 0
WIRE_YELLOW = 1 << 1
WIRE_GREEN = 1 << 2
WIRE_BROWN = 1 << 3
WIRE_GRAY = 1 << 4
WIRE_PURPLE = 1 << 5
WIRE_WHITE = 1 << 6
WIRE_BLUE = 1 << 7


class Reset:
    def __init__(self, url):
        # init gpio mode with gray (conncted to RESET) and green (TOOL0) as outputs
        self.gpio = GpioController()
        self.gpio.open_from_url(url, direction=WIRE_GRAY | WIRE_GREEN)

    def enter_rom(self):
        self.gpio.set_direction(WIRE_GRAY | WIRE_GREEN)
        # RESET=0, TOOL0=0
        self.gpio.write_port(0)
        delay(.04)
        # RESET=1, TOOL0=0
        self.gpio.write_port(WIRE_GRAY)
        delay(.001)
        # RESET=1, TOOL0=1
        self.gpio.write_port(WIRE_GRAY | WIRE_GREEN)
        delay(.01)
        # stop driving TOOL0 (with this ftdi device - another one takes over)
        self.gpio.set_direction(WIRE_GRAY)


def read_all(port, size):
    data = b''
    while len(data) < size:
        data += port.read(size - len(data))
    assert len(data) == size
    return data


def size8(size):
    if size <= 0 or size > 0x100: return None
    if size == 0x100: size = 0
    return size


def pack24(x):
    assert x < (1 << 24)
    return struct.pack('<HB', x & 0xffff, x >> 16)


class ProtoA:
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

    def __init__(self, port):
        self.port = port

    def read_all(self, size):
        return read_all(self.port, size)

    def _checksum(self, data):
        csum = 0
        for d in data:
            csum -= d
            csum &= 0xff
        return csum

    def _checksum16(self, data):
        csum = 0
        for d in data:
            csum -= d
            csum &= 0xffff
        return csum

    def recv_frame(self):
        while self.port.read() != bytes([self.STX]):
            pass
        len_b = self.port.read()
        LEN = size8(struct.unpack('B', len_b)[0])
        recv_len = LEN + 2
        data = self.read_all(recv_len)
        #print('recv %self' % (binascii.hexlify(data)))
        if self._checksum(len_b + data[:LEN]) != data[LEN]:
            print('bad checksum')
        if data[LEN + 1] != self.ETX:
            print('bad footer')
        return data[:LEN]

    def _send_frame(self, data, is_cmd=True, last_data=True):
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

    def send_frame(self, data, is_cmd=True, last_data=True):
        while True:
            r = self._send_frame(data, is_cmd, last_data)
            if r[0] != self.ST_SUM_ERR:
                return r

    def reset(self):
        return self.send_frame(struct.pack('B', self.COM_RESET))

    def set_baudrate(self, baudrate, voltage):
        return self.send_frame(
            struct.pack('BBB', self.COM_BAUDRATE_SET, baudrate, voltage))

    def silicon_sig(self):
        r = self.send_frame(struct.pack('B', self.COM_SILICON_SIG))
        if r[0] != self.ST_ACK: return None
        return self.recv_frame()

    def security_get(self):
        r = self.send_frame(struct.pack('B', self.COM_SEC_GET))
        if r[0] != self.ST_ACK: return None
        return self.recv_frame()

    def security_set(self, sec):
        r = self.send_frame(struct.pack('B', self.COM_SEC_SET))
        if r[0] != self.ST_ACK: return None
        return self.send_frame(sec, False)[0] == self.ST_ACK

    def verify(self, addr, data):
        assert len(data) > 0
        SA = pack24(addr)
        EA = pack24(addr + len(data) - 1)
        r = self.send_frame(struct.pack('B', self.COM_VERIFY) + SA + EA)
        if r[0] != self.ST_ACK: return False
        for i in range(0, len(data), 0x100):
            last_data = len(data) - i <= 0x100
            r = self.send_frame(data[i:i + 0x100], False, last_data)
        return r[0] == self.ST_ACK and r[1] == self.ST_ACK

    def checksum(self, addr, size):
        assert size > 0
        SA = pack24(addr)
        EA = pack24(addr + size - 1)
        r = self.send_frame(struct.pack('B', self.COM_CHECKSUM) + SA + EA)
        if r[0] != self.ST_ACK: return None
        return struct.unpack('<H', self.recv_frame())[0]

    def blank_check(self, addr, size=0x400):
        assert size > 0
        SA = pack24(addr)
        EA = pack24(addr + size - 1)
        # XXX
        D01 = struct.pack('B', 0)
        r = self.send_frame(
            struct.pack('B', self.COM_BLANK_CHECK) + SA + EA + D01)
        if r[0] not in (self.ST_ACK, self.ST_BLANK_ERR):
            return None
        # True means it is blank
        return r[0] == self.ST_ACK

    def invert_boot_cluster(self):
        # XXX can't be set via protoA :'(
        sec = self.security_get()
        sec = bytes([sec[0] ^ 1, *sec[1:]])
        return self.security_set(sec)

    def cmd19(self):
        # this is standalone "internal verify"
        addr = 0
        size = 0x400
        assert (((addr >> 8) & 0xff) & 3) == 0
        assert ((((addr + size - 1) >> 8) & 0xff) & 3) == 3
        SA = pack24(addr)
        EA = pack24(addr + size - 1)
        return self.send_frame(struct.pack('B', self.COM_19) + SA + EA)

    def erase_block(self, addr):
        return self.send_frame(struct.pack('B', self.COM_ERASE) + pack24(addr))

    def program(self, addr, data):
        SA = pack24(addr)
        EA = pack24(addr + len(data) - 1)
        r = self.send_frame(struct.pack('B', self.COM_PROG) + SA + EA)
        if r[0] != self.ST_ACK: return False
        for i in range(0, len(data), 0x100):
            last_data = len(data) - i <= 0x100
            r = self.send_frame(data[i:i + 0x100], False, last_data)
        if r[0] != self.ST_ACK or r[1] != self.ST_ACK:
            return False
        # iverify status
        return self.recv_frame()

    def write(self, addr, data):
        # erase block = 0x400, everything else can use 0x100
        if addr % 0x400 or len(data) % 0x400:
            return False
        for i in range(0, len(data), 0x400):
            self.erase_block(addr + i)
        # XXX should be able to handle multiple blocks, not sure why it hangs
        #self.program(addr, data)
        for i in range(0, len(data), 0x100):
            self.program(addr + i, data[i:i + 0x100])
        return self.verify(addr, data)


class ProtoOCD:
    SYNC = 0x00
    PING = 0x90
    UNLOCK = 0x91
    READ = 0x92
    WRITE = 0x93
    EXEC = 0x94
    EXIT_RETI = 0x95
    EXIT_RAM = 0x97

    PONG = bytes([3, 3])

    ST_UNLOCK_ALREADY = 0xf0
    ST_UNLOCK_LOCKED = 0xf1
    ST_UNLOCK_OK = 0xf2
    ST_UNLOCK_SUM = 0xf3
    ST_UNLOCK_NG = 0xf4

    def __init__(self, port):
        self.port = port

    def read_all(self, size):
        return read_all(self.port, size)

    def checksum(self, data):
        csum = 0
        for d in data:
            csum += d
            csum &= 0xff
        csum -= 1
        csum &= 0xff
        return csum

    def send_cmd(self, cmd):
        #print('send %self' % (binascii.hexlify(cmd)))
        self.port.write(cmd)
        # discard the loopback bytes
        self.read_all(len(cmd))

    def wait_ack(self):
        while self.read_all(1) != bytes([self.SYNC]):
            pass

    def sync(self):
        self.send_cmd(struct.pack('B', self.SYNC))
        self.wait_ack()

    def ping(self):
        self.send_cmd(struct.pack('B', self.PING))
        return self.read_all(len(self.PONG)) == self.PONG
        #return self.read_all(len(ping_result)) == ping_result
    def unlock(self, ocd_id, corrupt_sum=False):
        self.send_cmd(struct.pack('B', self.UNLOCK))
        status = self.read_all(1)[0]
        # f0: already unlocked
        # f1: need to send
        if status == self.ST_UNLOCK_ALREADY:
            print('already unlocked')
            return True
        if status != self.ST_UNLOCK_LOCKED:
            print('unexpected status')
            return False
        csum = self.checksum(ocd_id)
        if corrupt_sum:
            csum += 1
            csum &= 0xff
        self.send_cmd(struct.pack('10BB', *ocd_id, csum))
        status = self.read_all(1)[0]
        # f2: success
        # f3: checksum mismatch
        # f4: checksum matched but ocd_id did not (could trigger flash erase?)
        if status != self.ST_UNLOCK_OK:
            print('unlock failed: %x' % (status))
        return status == self.ST_UNLOCK_OK

    def read(self, offset, size):
        size8_ = size8(size)
        if size8_ is None: return None
        self.send_cmd(struct.pack('<BHB', self.READ, offset, size8_))
        return self.read_all(size)

    def write(self, addr, data):
        size = size8(len(data))
        if size is None: return None
        self.send_cmd(
            struct.pack('<BHB%dB' (len(data)), self.WRITE, addr, size, *data))
        return self.read_all(1)[0] == self.WRITE

    def call_f07e0(self):
        self.send_cmd(struct.pack('B', self.EXEC))
        return self.read_all(1)[0] == self.EXEC

    def leave(self, to_ram=False):
        cmd = self.EXIT_RAM if to_ram else self.EXIT_RETI
        self.send_cmd(struct.pack('B', cmd))
        return self.read_all(1)[0] == cmd


class RL78:
    MODE_A_1WIRE = b'\x3a'
    MODE_A_2WIRE = b'\x00'
    MODE_OCD = b'\xc5'
    BAUDRATE_INIT = 115200
    BAUDRATE_FAST = 1000000

    def __init__(self, gpio_url, uart_port):
        self.reset_ctl = Reset(gpio_url)
        self.port = serial.Serial(uart_port,
                                  baudrate=self.BAUDRATE_INIT,
                                  timeout=0,
                                  stopbits=2)
        self.a = ProtoA(self.port)
        self.ocd = ProtoOCD(self.port)
        self.mode = None

    def reset(self, mode):
        self.mode = mode
        self.port.baudrate = self.BAUDRATE_INIT
        self.reset_ctl.enter_rom()
        self.port.write(self.mode)
        # we'll see the reset as a null byte. discard it and the init byte
        read_all(self.port, 2)
        # send baudrate cmd (required) & sync
        baudrate = self.BAUDRATE_FAST if self.mode != self.MODE_OCD else self.BAUDRATE_INIT
        rl78_br = {115200: 0, 250000: 1, 500000: 2, 1000000: 3}[baudrate]
        # 21 = 2.1v
        # really just sets internal voltage regulator to output 1.7, 1.8 or 2.1 volts
        # regulator seems to auto-adjust anyways...
        # feeding with 1.7v uses slower mode, 1.8v and 2.1v are same, slightly faster speed
        r = self.a.set_baudrate(rl78_br, 21)
        self.port.baudrate = baudrate
        if r[0] != ProtoA.ST_ACK: return False
        delay(.01)
        if self.mode != self.MODE_OCD:
            r = self.a.reset()
            if r[0] != ProtoA.ST_ACK: return False
        else:
            self.ocd.wait_ack()
            if not self.ocd.ping(): return False
        return True


if __name__ == '__main__':
    rl78 = RL78('ftdi://ftdi:232h/0', 'COM5')
    if not rl78.reset(RL78.MODE_A_1WIRE):
        print('failed to init a')
        exit()
    print('sig', binascii.hexlify(rl78.a.silicon_sig()))
    print('sec', binascii.hexlify(rl78.a.security_get()))
    code.InteractiveConsole(locals=locals()).interact('Entering shell...')
