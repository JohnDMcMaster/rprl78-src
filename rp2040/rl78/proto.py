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


OCD
it t looks like everything responds to SET_BAUD
2/3 responded baud w/ ST_ACK, but didn't respond to sync
(verified on logic analyzer)
1/3 responded baud w/ ST_PROTECT_ERR
"""

from machine import UART
from machine import Pin
import rp2
import time
import binascii
import struct
import sys
import json
from .util import hexdump

VERBOSE = 0
gpio_debug1 = Pin(16, Pin.OUT, value=0)


class DebugUART:
    def __init__(self, n, baudrate=115200):
        # Tried to extend class but too special
        self.uart = UART(n, baudrate)
        self.verbose = 0

    def init(self, *args, **kwargs):
        return self.uart.init(*args, **kwargs)

    def write(self, buf):
        if self.verbose:
            hexdump(buf, "write() = %u" % len(buf))
        return self.uart.write(buf)

    def read(self, n=1):
        ret = self.uart.read(n)
        if self.verbose:
            hexdump(ret, "read() = %u" % n)
        return ret


class SerialTimeout(Exception):
    pass


class Reset:
    def __init__(self, gpio_pwr):
        self.gpio_pwr = gpio_pwr
        self.gpio_reset = None
        self.gpio_tool0 = None

    def power_on(self):
        resetz = 0

        self.gpio_tool0 = Pin(0, Pin.IN)
        # self.gpio_tool0 = Pin(0, Pin.OUT, value=1)

        self.gpio_pwr.low()
        if resetz:
            self.gpio_reset = Pin(2, Pin.IN)
        else:
            self.gpio_reset = Pin(2, Pin.OUT, value=0)
        time.sleep_ms(100)
        self.gpio_pwr.high()
        time.sleep_ms(100)
        if not resetz:
            # RESETn => start CPU
            self.gpio_reset.on()

    def power_off(self):
        self.gpio_pwr.low()

    def enter_rom(self):
        """
        25.5.2 Flash memory programming mode

        t SUINIT : The segment shows that it is necessary to finish specifying the initial communication settings within 100
        ms from when the resets end.

        (TOOLRxD, TOOLTxD mode)

        2 wire
        TOOLTxD,
        TOOLRxD
        """

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

        # Original sequence
        # Power is always on
        if 0:
            self.gpio_pwr.high()
            time.sleep_ms(100)

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
            raise SerialTimeout("Timed out")
        new_bytes = port.read(size - len(data))
        if new_bytes:
            verbose and len(new_bytes) and print("rx +%u / %u bytes" %
                                                 (len(new_bytes), size))
            0 and hexdump(new_bytes)
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


class RenesasError(Exception):
    pass


class ProtectError(RenesasError):
    pass


class ProtoAError(RenesasError):
    pass


class ProtoANACK(ProtoAError):
    pass


def st1_exception(buf, msg=""):
    if buf[0] == ProtoA.ST_NACK:
        raise ProtoANACK(msg)
    else:
        s = ProtoA.st_i2s.get(buf[0])
        if msg:
            msg = "0x%02X (%s): %s" % (buf[0], s, str(msg))
        else:
            msg = "0x%02X (%s)" % (buf[0], s)
        raise ProtoAError(msg)


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
        tstart = time.time()
        while True:
            if time.time() - tstart > 0.5:
                raise SerialTimeout("timed out")
            if self.port.read() == bytes([self.STX]):
                break
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
            print("WARNING: SEND_FRAME got %d" % r[0])

    def reset(self):
        return self.send_frame(struct.pack('B', self.COM_RESET))

    def set_baudrate(self, baudrate, voltage):
        return self.send_frame(
            struct.pack('BBB', self.COM_BAUDRATE_SET, baudrate, voltage))

    def silicon_sig(self):
        """
        Reads RL78 information (such as product name and flash
        memory configuration).
        """
        r = self.send_frame(struct.pack('B', self.COM_SILICON_SIG))
        if r[0] != self.ST_ACK:
            raise st1_exception(r)
        return self.recv_frame()

    def security_get(self):
        """
        Reads a security flag, boot block cluster block number, boot area
        exchange flag, and FSW (flash option).
        """
        r = self.send_frame(struct.pack('B', self.COM_SEC_GET))
        if r[0] != self.ST_ACK:
            raise st1_exception(r)
        return self.recv_frame()

    def security_set(self, sec):
        r = self.send_frame(struct.pack('B', self.COM_SEC_SET))
        if r[0] != self.ST_ACK:
            raise st1_exception(r)
        return self.send_frame(sec, False)[0] == self.ST_ACK

    def verify(self, addr, data):
        assert len(data) > 0
        SA = pack24(addr)
        EA = pack24(addr + len(data) - 1)
        r = self.send_frame(struct.pack('B', self.COM_VERIFY) + SA + EA)
        if r[0] != self.ST_ACK:
            raise st1_exception(r)
        for i in range(0, len(data), 0x100):
            last_data = len(data) - i <= 0x100
            r = self.send_frame(data[i:i + 0x100], False, last_data)
        return r[0] == self.ST_ACK and r[1] == self.ST_ACK

    def checksum(self, addr, size):
        """
        addr: if non-0 seems to return "size" instead of an actual checksum
        size must be aligned or returns nack

        can't get a size less than 0x100 to work


        >>> "0x%04X" % rl78.a.checksum(0x000, 0x100)
        '0x2075'
        >>> "0x%04X" % rl78.a.checksum(0x100, 0x100)
        '0x3CD4'
        """
        # assert size > 0
        SA = pack24(addr)
        EA = pack24(addr + size - 1)
        r = self.send_frame(struct.pack('B', self.COM_CHECKSUM) + SA + EA)
        if r[0] != self.ST_ACK:
            raise ProtoANACK()
        return struct.unpack('<H', self.recv_frame())[0]

    def blank_check(self, addr, size=0x400, d01=0, raw=False):
        """
        SOF    00000000  32 00 40 00 FF 43 00 00                           |2.@..C..        |
        Block Blank Check
        Address: 0x004000 : 0x0043FF
        D01: Specified block
        STX    00000000  1B                                                |.               |
        Not blank
        """
        assert size > 0
        SA = pack24(addr)
        EA = pack24(addr + size - 1)
        # 0 => Specified block, 1 => Specified block and flash option
        D01 = struct.pack('B', d01)
        frame = struct.pack('B', self.COM_BLANK_CHECK) + SA + EA + D01
        r = self.send_frame(frame)
        if raw:
            return (frame, r)
        else:
            if r[0] not in (self.ST_ACK, self.ST_BLANK_ERR):
                raise st1_exception(r)
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
        # McMaster: is acked but not actually set
        sec = bytearray(self.security_get())
        sec = bytes([sec[0] ^ 1]) + sec[1:]
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
        r = self.send_frame(struct.pack('B', self.COM_ERASE) + pack24(addr))
        if r[0] != self.ST_ACK:
            raise st1_exception(r)

    def program(self, addr, data):
        SA = pack24(addr)
        EA = pack24(addr + len(data) - 1)
        r = self.send_frame(struct.pack('B', self.COM_PROG) + SA + EA)
        if r[0] != self.ST_ACK:
            raise st1_exception(r)
        for i in range(0, len(data), 0x100):
            last_data = len(data) - i <= 0x100
            r = self.send_frame(data[i:i + 0x100], False, last_data)
        # Status frame ST1
        # ST1 (b): Data reception check result
        if r[0] != self.ST_ACK:
            raise st1_exception(r, "ST1 data frame")
        # Status frame ST2
        # ST2 (b): Write result
        if r[1] != self.ST_ACK:
            raise st1_exception(r[1:], "ST2 status frame")
        # Completion frame
        r = self.recv_frame()
        if r[0] != self.ST_ACK:
            raise st1_exception(r, "completion frame")

    def write(self, addr, data):
        # erase block = 0x400, everything else can use 0x100
        if addr % 0x400 or len(data) % 0x400:
            raise ValueError()
        time.sleep(0.1)
        for i in range(0, len(data), 0x400):
            self.erase_block(addr + i)
        # XXX should be able to handle multiple blocks, not sure why it hangs
        #self.program(addr, data)
        for i in range(0, len(data), 0x100):
            print("offset", i)
            time.sleep(0.1)
            self.program(addr + i, data[i:i + 0x100])
        return self.verify(addr, data)


"""
Handshake
0xC5
0x01 set baudrate 0x03
0x02 set baudrate reply 0x03
0x00
    SYNC = 0x00
0x90 0x03 0x03
    ping
"""


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
        # fail0verflow code had this, but I don't see anything after SYNC in my tests
        self.wait_ack()

    def ping(self):
        self.send_cmd(struct.pack('B', self.PING))
        return self.read_all(len(self.PONG)) == self.PONG
        #return self.read_all(len(ping_result)) == ping_result

    def unlock(self, ocd_id, corrupt_sum=False):
        """
        ocd_id: 10 byte security code
        Seems like all 0's was default on my dev board
        """
        transaction = bytearray()

        def tool0(buf):
            transaction.extend(buf)
            return buf

        self.send_cmd(tool0(struct.pack('B', self.UNLOCK)))
        status = tool0(self.read_all(1))[0]
        # f0: already unlocked
        # f1: need to send
        if status == self.ST_UNLOCK_ALREADY:
            print('already unlocked')
            return True
        if status != self.ST_UNLOCK_LOCKED:
            print('unexpected status')
            return False
        if ocd_id is None:
            raise ProtectError("Aborting on locked + null OCD id")
        assert len(ocd_id) == 10
        csum = self.checksum(ocd_id)
        if corrupt_sum:
            csum += 1
            csum &= 0xff
        self.send_cmd(tool0(struct.pack('10BB', *ocd_id, csum)))
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

    # BAUDRATE_FAST = 1000000
    # BAUDRATE_FAST = 500000
    # BAUDRATE_FAST = 250000
    BAUDRATE_FAST = 115200

    def __init__(self, verbose=None):
        if verbose is None:
            verbose = VERBOSE
        self.verbose = verbose
        self.uartn = 0
        self.verbose and print("Opening reset controller...")
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

        # These are command factories,
        # just creating them doesn't send anything
        self.a = ProtoA(self.port, verbose=self.verbose)
        self.ocd = ProtoOCD(self.port)
        self.mode = None

    def reset(self, mode, baudup=True, flush=True, probe=True):
        gpio_debug1.low()
        self.verbose and print("Resetting in mode 0x%02X" % mode[0])
        self.mode = mode
        # self.port.baudrate = self.BAUDRATE_INIT
        self.verbose and print("Sending ROM reset")
        self.reset_ctl.enter_rom()

        self.port = DebugUART(self.uartn, baudrate=self.BAUDRATE_INIT)
        #time.sleep_ms(10)
        self.port.init(baudrate=self.BAUDRATE_INIT,
                       bits=8,
                       parity=None,
                       stop=1,
                       timeout=500)
        self.a.port = self.port
        self.ocd.port = self.port
        # time.sleep_ms(20)
        """
        Nothing connected: expect 1 byte from false serial trigger?
        In original yes, but now UART swaps in after...less sure
        But want 2 bytes (ack)
        
        N/C
        Sometimes I receive something, sometimes not
        Hmm shrug we'll see
        """
        self.verbose and print("reset: writing mode 0x%02X" % self.mode[0])
        assert self.port.write(self.mode) == 1
        self.verbose and print("wrote mode")
        # works as quick test: send set mode + baudrate write
        # print("wrote %s" % self.port.write(b"\x3A\x01\x03\x9A\x02\x21\x40\x03"))

        # Looks like this might be required?
        # Works in A but not OCD
        # Maybe OCD was just disallowed was the real issue?
        if baudup:
            # send baudrate cmd (required) & sync
            baudrate = self.BAUDRATE_FAST if self.mode != self.MODE_OCD else self.BAUDRATE_INIT
            rl78_br = {115200: 0, 250000: 1, 500000: 2, 1000000: 3}[baudrate]
            # 21 = 2.1v
            # really just sets internal voltage regulator to output 1.7, 1.8 or 2.1 volts
            # regulator seems to auto-adjust anyways...
            # feeding with 1.7v uses slower mode, 1.8v and 2.1v are same, slightly faster speed
            self.verbose and print("setting baudrate")
            # r = self.a.set_baudrate(rl78_br, 0x21)
            # dipping to about 3.1V sometimes...maybe this will make more reliable?
            # RenesasError: Reset failed: baud ACK fail w/ 5
            # r = self.a.set_baudrate(rl78_br, 0x10)
            r = self.a.set_baudrate(rl78_br, 0x21)
            self.verbose and print("checking result")
            if r[0] != ProtoA.ST_ACK:
                if r[0] == ProtoA.ST_PROTECT_ERR:
                    raise ProtectError(
                        "Reset failed: baud ACK fail w/ PROTECT_ERR")
                else:
                    raise RenesasError("Reset failed: baud ACK fail w/ %d" %
                                       (r[0], ))

            assert len(r) == 3
            st1, d01, d02 = r
            self.verbose and print("Baud rate OK")
            self.verbose and print("  ST1: 0x%02X" % st1)
            # OCD: got 0x08
            self.verbose and print("  D01: 0x%02X (%u MHz)" % (d01, d01))
            fstr = {0: "full-speed mode", 1: "wide-voltage mode"}[d02]
            self.verbose and print("  D02: 0x%02X (%s)" % (d02, fstr))

            # there are two acks: one for frame SET_BAUDRATE, a second at the new baudrate
            # however, they are only 2 ms apart and we can't re-initialize the serial port fast enough
            # Delay to intentionally loose the ack at the new badurate
            time.sleep_ms(10)
            self.verbose and print("re-initializing port")
            # From trace, observed to be 8n1, *not* 8n2 as initially claimed
            self.port.init(baudrate=baudrate,
                           bits=8,
                           parity=None,
                           stop=1,
                           timeout=500)

        if flush:
            self.verbose and print("reset: flushing buffer")
            flushed = self.port.read(64) or b""
            self.verbose and print("reset: flushed %u bytes" % len(flushed))

        if probe:
            if self.mode in (self.MODE_A_1WIRE, self.MODE_A_2WIRE):
                self.verbose and print("Sending ProtoA reset")
                gpio_debug1.high()
                r = self.a.reset()
                if r[0] != ProtoA.ST_ACK:
                    raise RenesasError("Reset failed: A NACK")
                self.verbose and print("ProtoA established")
            elif self.mode == self.MODE_OCD:
                self.verbose and print("Waiting for ACK / SYNC....")
                # Should occur 1 ms after baud set
                # We will probably loose it with uart re-initialization and/or flush above
                # self.ocd.wait_ack()
                if not self.ocd.ping():
                    raise RenesasError("Reset failed: OCD failed ping")
            else:
                assert 0, "Bad mode"


def power_on(rl78=None):
    """
    Power on w/o entering debug mode
    """
    if rl78 is None:
        rl78 = RL78(verbose=False)
    rl78.reset_ctl.power_on()
    return rl78


def power_off(rl78=None):
    if rl78 is None:
        rl78 = RL78(verbose=False)
    rl78.reset_ctl.power_off()
    return rl78


def try_a1(rl78=None):
    if not rl78:
        VERBOSE and print("Opening...")
        rl78 = RL78(verbose=False)
    VERBOSE and print("Resetting...")
    rl78.reset(RL78.MODE_A_1WIRE)
    return rl78


def try_ocd(rl78=None):
    if not rl78:
        VERBOSE and print("Opening...")
        rl78 = RL78(verbose=False)
    VERBOSE and print("Resetting...")
    rl78.reset(RL78.MODE_OCD)
    return rl78
