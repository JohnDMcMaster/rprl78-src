"""
My own E1 capture
Vcc = 3.48 V
Vhigh = 3.46 V
Vlow = 0.00 V

t < 0
    VCC = 0
    TOOL0 = 0
    RESET = 0
t = 0
    VCC = 1
    TOOL0 = 1
    RESET = 0
t + 40 ms
    TOOL0 = 0
t + 2 ms
    RESET = 1
t + 4 ms
    TOOL0 = 1
t + 3 ms
    serial starts
    3A 01 03 9A 02 21 40 03
    period is about 0.4 ms
    8.48 us bit period => 118 kbps

t + ?
    2.08 us bit period => 480 kbps



what happens if I disconnect chip?








>>> import serial
Traceback (most recent call last):
  File "<stdin>", line 1, in <module>
ImportError: no module named 'serial'



https://docs.micropython.org/en/latest/library/machine.UART.html



>>> uart.write("Hello, world!") 
13



pinout
UART0.tx: GPIO0
UART0.rx: GPIO1

screen /dev/ttyUSB0 115200
worked
tx and rx


hmm lets try upgrading
rp2-pico-20220618-v1.19.1.uf2
aha
upgrading to 1.19 gave init function and other stuff missing



>>> import machine
>>> machine.freq() 
125000000
"""

from machine import UART
from machine import Pin
import rp2
import time
import binascii
import struct
import sys


def delay(sec):
    time.sleep_ms(int(sec * 1000))

def hexdump(data, label=None, indent='', address_width=8, f=sys.stdout):
    def isprint(c):
        return c >= ' ' and c <= '~'

    if label:
        print(label)

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
            for c in str(data[row_start:row_start + real_data])
        ]))
        f.write((" " * (bytes_per_row - real_data)) + "|\n")

class Reset:
    def __init__(self, gpio_pwr):
        self.gpio_pwr = gpio_pwr
        self.gpio_reset = None
        self.gpio_tool0 = None

    def enter_rom(self):
        self.gpio_pwr.low()
        self.gpio_reset = Pin(2, Pin.OUT, value=0)
        self.gpio_tool0 = Pin(0, Pin.OUT, value=0)
        time.sleep_ms(100)

        if 1:
            # McMaster E1 observed sequence
            
            # RESET=0, TOOL0=1
            self.gpio_pwr.high()
            self.gpio_tool0.high()
            time.sleep_ms(435)

            # RESET=0, TOOL0=0
            self.gpio_tool0.low()
            time.sleep_ms(2)
            
            # RESET=1, TOOL0=0
            self.gpio_reset.high()
            time.sleep_ms(4)

            # RESET=1, TOOL0=1
            self.gpio_tool0.high()
            # time.sleep_ms(3)
            # serial open takes a while
            time.sleep_ms(2)

        if 0:
            # RESET=0, TOOL0=0
            self.gpio_reset.off()
            self.gpio_tool0.off()
            time.sleep_ms(40)
            # RESET=1, TOOL0=0
            self.gpio_reset.on()
            time.sleep_ms(1)
            # RESET=1, TOOL0=1
            self.gpio_tool0.on()
            time.sleep_ms(10)

        # stop driving TOOL0 as GPIO, serial device will take over
        self.gpio_tool0 = Pin(0, Pin.IN)


def read_all(port, size, timeout=1.0, verbose=False):
    """
    port was serial.Serial
    Now its machine.UART
    https://docs.micropython.org/en/latest/library/machine.UART.html
    """

    tstart = time.time()
    data = b''
    # port.timeout = 0
    while len(data) < size:
        if time.time() - tstart > timeout:
            raise Exception("Timed out")
        new_bytes = port.read(size - len(data))
        if new_bytes:
            verbose and len(new_bytes) and print("rx %u bytes" % len(new_bytes))
            1 and hexdump(new_bytes)
            data += new_bytes
    assert len(data) == size
    return data


def size8(size):
    if size <= 0 or size > 0x100:
        return None
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

    def __init__(self, port, verbose=False):
        self.port = port
        self.verbose = verbose

    def read_all(self, size):
        return read_all(self.port, size, verbose=self.verbose)

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
        if r[0] != self.ST_ACK:
            return None
        return self.recv_frame()

    def security_get(self):
        r = self.send_frame(struct.pack('B', self.COM_SEC_GET))
        if r[0] != self.ST_ACK:
            return None
        return self.recv_frame()

    def security_set(self, sec):
        r = self.send_frame(struct.pack('B', self.COM_SEC_SET))
        if r[0] != self.ST_ACK:
            return None
        return self.send_frame(sec, False)[0] == self.ST_ACK

    def verify(self, addr, data):
        assert len(data) > 0
        SA = pack24(addr)
        EA = pack24(addr + len(data) - 1)
        r = self.send_frame(struct.pack('B', self.COM_VERIFY) + SA + EA)
        if r[0] != self.ST_ACK:
            return False
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

    # SyntaxError: *x must be assignment target
    """
    def invert_boot_cluster_old(self):
        # XXX can't be set via protoA :'(
        sec = self.security_get()
        sec = bytes([sec[0] ^ 1, *sec[1:]])
        return self.security_set(sec)
    """
    
    def invert_boot_cluster(self):
        # XXX can't be set via protoA :'(
        sec = self.security_get()
        sec = bytes([sec[0] ^ 1] + sec[1:])
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
        if size8_ is None:
            return None
        self.send_cmd(struct.pack('<BHB', self.READ, offset, size8_))
        return self.read_all(size)

    def write(self, addr, data):
        size = size8(len(data))
        if size is None:
            return None
        self.send_cmd(
            struct.pack('<BHB%dB' % (len(data)), self.WRITE, addr, size,
                        *data))
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

    def __init__(self, verbose=False):
        self.verbose = verbose
        self.uartn = 0
        print("Opening reset controller...")
        self.gpio_pwr = Pin(3, Pin.OUT, value=0)
        self.reset_ctl = Reset(self.gpio_pwr)
        # input("Press Enter to continue...")
        # print("Opening serial port %u..." % uartn)
        """
        self.port = serial.Serial(uart_port,
                                  baudrate=self.BAUDRATE_INIT,
                                  timeout=0,
                                  stopbits=2)
        """
        self.port = None

        print("Opening ProtoA...")
        self.a = ProtoA(self.port, verbose=self.verbose)
        print("Opening OCD...")
        self.ocd = ProtoOCD(self.port)
        self.mode = None

    def reset(self, mode):
        self.verbose and print("Resetting in mode 0x%02X" % mode[0])
        self.mode = mode
        # self.port.baudrate = self.BAUDRATE_INIT
        self.verbose and print("Sending ROM reset")
        self.reset_ctl.enter_rom()

        self.port = UART(self.uartn, baudrate=self.BAUDRATE_INIT)
        #time.sleep_ms(10)
        self.port.init(baudrate=self.BAUDRATE_INIT, bits=8, parity=None, stop=2)
        # time.sleep_ms(20)

        """
        Nothing connected: expect 1 byte from false serial trigger?
        In original yes, but now UART swaps in after...less sure
        But want 2 bytes (ack)
        
        N/C
        Sometimes I recieve something, sometimes not
        Hmm shrug we'll see
        """
        print("Writing mode 0x%02X" % self.mode[0])
        if 0:
            assert self.port.write(self.mode) == 1
            # "For the rp2, esp8266 and nrf ports the call returns while the last byte is sent.
            # If required, a one character wait time has to be added in the calling script."
            # AttributeError: 'UART' object has no attribute 'flush'
            #self.port.flush()
        if 0:
            print("wrote %s" % self.port.write(b"\x3A\x01\x03\x9A\x02\x21\x40\x03"))
        if 1:
            #for i, b in enumerate(bytearray(b"\x3A\x01\x03\x9A\x02\x21\x40\x03")):
            for i, b in enumerate(bytearray(b"\x3A")):
                assert self.port.write(bytearray([b])) == 1
                if i:
                    time.sleep_us(200)
                else:
                    time.sleep_us(300)

        delay(.1)
        assert 0

        self.verbose and print("Waiting for ack")
        # we'll see the reset as a null byte. discard it and the init byte
        read_all(self.port, 2, verbose=self.verbose)
        self.verbose and print("Got ack")

        # send baudrate cmd (required) & sync
        baudrate = self.BAUDRATE_FAST if self.mode != self.MODE_OCD else self.BAUDRATE_INIT
        rl78_br = {115200: 0, 250000: 1, 500000: 2, 1000000: 3}[baudrate]
        # 21 = 2.1v
        # really just sets internal voltage regulator to output 1.7, 1.8 or 2.1 volts
        # regulator seems to auto-adjust anyways...
        # feeding with 1.7v uses slower mode, 1.8v and 2.1v are same, slightly faster speed
        r = self.a.set_baudrate(rl78_br, 21)
        # self.port.baudrate = baudrate
        self.port.init(baudrate=baudrate, bits=8, parity=None, stop=2)
        if r[0] != ProtoA.ST_ACK:
            return False
        delay(.01)
        if self.mode != self.MODE_OCD:
            r = self.a.reset()
            if r[0] != ProtoA.ST_ACK:
                return False
        else:
            self.ocd.wait_ack()
            if not self.ocd.ping():
                return False
        return True


def main():
    verbose = True
    print("Opening...")
    rl78 = RL78(verbose=verbose)
    print("Resetting...")
    try:
        if not rl78.reset(RL78.MODE_A_1WIRE):
            raise Exception('failed to init a')
        print('sig', binascii.hexlify(rl78.a.silicon_sig()))
        print('sec', binascii.hexlify(rl78.a.security_get()))
        # code.InteractiveConsole(locals=locals()).interact('Entering shell...')
    finally:
        # rl78.gpio_pwr.off()
        pass
