import serial
import platform
import glob
import pexpect
import os
import time
import re
import binascii


def default_port():
    if platform.system() == "Linux":
        acms = glob.glob('/dev/serial/by-id/usb-MicroPython_*')
        if len(acms) == 0:
            return None
        return acms[0]
    else:
        return None


class NoSuchLine(Exception):
    pass


class BadCommand(Exception):
    pass


class Timeout(Exception):
    pass


class SerialExpect(pexpect.spawnbase.SpawnBase):
    '''A pexpect class that works through a serial.Serial instance.
       This is necessary for compatibility with Windows. It is basically
       a pexpect.fdpexpect, except for serial.Serial, not file descriptors.
    '''
    def __init__(self,
                 ser,
                 args=None,
                 timeout=30,
                 maxread=2000,
                 searchwindowsize=None,
                 logfile=None,
                 encoding=None,
                 codec_errors='strict',
                 use_poll=False):
        self.ser = ser
        if not isinstance(ser, serial.Serial):
            raise Exception(
                'The ser argument is not a serial.Serial instance.')
        self.args = None
        self.command = None
        pexpect.spawnbase.SpawnBase.__init__(self,
                                             timeout,
                                             maxread,
                                             searchwindowsize,
                                             logfile,
                                             encoding=encoding,
                                             codec_errors=codec_errors)
        self.child_fd = None
        self.own_fd = False
        self.closed = False
        self.name = ser.name
        self.use_poll = use_poll

    def close(self):
        self.flush()
        self.ser.close()
        self.closed = True

    def flush(self):
        self.ser.flush()

    def isalive(self):
        return not self.closed

    def terminate(self, force=False):
        raise Exception('This method is not valid for serial objects')

    def send(self, s):
        s = self._coerce_send_string(s)
        self._log(s, 'send')
        b = self._encoder.encode(s, final=False)
        self.ser.write(b)

    def sendline(self, s):
        s = self._coerce_send_string(s)
        return self.send(s + self.linesep)

    def write(self, s):
        b = self._encoder.encode(s, final=False)
        self.ser.write(b)

    def writelines(self, sequence):
        for s in sequence:
            self.write(s)

    def read_nonblocking(self, size=1, timeout=None):
        s = self.ser.read(size)
        s = self._decoder.decode(s, final=False)
        self._log(s, 'read')
        return s


class RP2040MP:
    '''ASCII client common'''

    APP = None

    def __init__(self, device=None, verbose=None):
        if device is None:
            device = default_port()
            if device is None:
                raise Exception("Failed to find serial port")
        if verbose is None:
            verbose = os.getenv("VERBOSE", "N") == "Y"
        # verbose = True
        self.verbose = verbose
        self.verbose and print("port: %s" % device)
        self.device = device
        self.ser = None
        self.e = None
        self.timeout = 1.0
        self.init()

    def init(self):
        if self.ser:
            self.ser.close()
        self.ser = None
        self.e = None

        self.ser = serial.Serial(self.device,
                                 timeout=0,
                                 baudrate=115200,
                                 writeTimeout=0)
        self.e = SerialExpect(self.ser, encoding="ascii")

        # send dummy newline to clear any commands in progress
        self.e.write('\n')
        self.e.flush()
        self.flushInput()

    def flushInput(self):
        # Try to get rid of previous command in progress, if any
        tlast = time.time()
        while time.time() - tlast < 0.1:
            buf = self.ser.read(1024)
            if buf:
                tlast = time.time()

        self.ser.flushInput()

    def reset(self):
        """
        MPY: soft reboot
        MicroPython v1.19.1 on 2022-06-18; Raspberry Pi Pico with RP2040
        Type "help()" for more information.
        >>>
        """
        self.cmd("import machine")
        self.cmd("machine.reset()", reply=False)
        tstart = time.time()
        time.sleep(0.1)
        while not os.path.exists(self.device):
            time.sleep(0.05)
        tend = time.time()
        # Reset after 0.7 sec
        print("Reset after %0.1f sec" % (tend - tstart, ))
        # time.sleep(0.1)
        self.init()

    def expect(self, s, timeout=None):
        if timeout is None:
            timeout = self.timeout
        self.e.expect(s, timeout=timeout)
        return self.e.before

    def cmd(self, cmd, reply=True, retry=False, timeout=None):
        '''Send raw command and get string result'''

        attempts = 1
        if retry:
            attempts = 3
        for attempt in range(attempts):
            try:
                tstart = time.time()
                strout = cmd + "\r\n"
                self.verbose and print("cmd out: %s" % strout.strip())
                self.e.ser.flushInput()
                self.e.buffer = ""
                # FIXME: possibly writing above 64 chars is unreliable?
                # hack: slow down and chunk writes
                # self.e.write(cmd)
                for c in strout:
                    self.e.write(c)
                # self.paste(strout)
                self.e.flush()

                if not reply:
                    return None

                ret = self.expect('>>>', timeout=timeout)
                self.verbose and print('cmd ret: chars %u' % (len(ret), ))
                # self.verbose and util.hexdump(ret)
                if "Traceback (most recent call last):" in ret:
                    outterse = ret.strip().replace('\r',
                                                   '').replace('\n', '; ')
                    raise BadCommand("Failed command: %s, got: %s" %
                                     (strout.strip(), outterse))
                tend = time.time()
                self.verbose and print("Took %0.3f sec" % (tend - tstart, ))
                return ret
            except pexpect.TIMEOUT:
                if attempt < attempts - 1:
                    assert 0
                    print("WARNINING: rp2040 cmd retry")
                    continue
                raise
        assert 0

    def match_line(self, a_re, res):
        # print(len(self.e.before), len(self.e.after), len(res))
        lines = res.split('\n')
        for l in lines:
            l = l.strip()
            m = re.match(a_re, l)
            if m:
                self.verbose and print("match: %s" % l)
                return m
        else:
            if self.verbose:
                print("Failed lines %d" % len(lines))
                for l in lines:
                    print("  %s" % l.strip())
            raise NoSuchLine("Failed to match re: %s" % a_re)

    def paste(self, s):
        """
        Send bulk multi-line command
        """
        CTRL_E = "\x05"
        CTRL_D = "\x04"
        # ^C, ^E
        buf = CTRL_E + s
        # was wedging due to waiting for read
        for i in range(0, len(buf), 256):
            self.e.write(buf[i:i + 256])
            self.e.flush()
            while self.e.read_nonblocking(size=256):
                pass
        self.e.write(CTRL_D)
        self.e.flush()
        ret = self.expect('>>>')
        self.verbose and print('paste back: chars %u' % (len(ret), ))
        # FIXME: error check

        self.cmd("")

    def read_str(self, var):
        s = self.cmd("print(%s)" % var).strip()
        s = s.split("\n")[-1]
        return s

    def read_int(self, var):
        s = self.cmd("print(%s)" % var).strip()
        s = s.split("\n")[-1]
        return int(s)

    def read_bool(self, var):
        s = self.cmd("print(%s)" % var).strip()
        s = s.split("\n")[-1]
        return {"False": False, "True": True}[s]

    def to_hex_str(self, buf):
        return '"%s"' % (bytes(buf).hex(), )
