"""
Emulator => interface on rp2040
"""

from .rp2040 import RP2040MP, BadCommand, Timeout
import os
import pexpect

WRITE_SIZE = 256
BLOCK_SIZE = 1024
CLUSTER_SIZE = 4096


class Emulator:
    MODE_A_1WIRE = b'\x3a'
    MODE_A_2WIRE = b'\x00'
    MODE_OCD = b'\xc5'

    def __init__(self, port=None, verbose=False, init=True, mode=MODE_A_1WIRE):
        self.mp = None
        self.loaded = False
        self.mp = RP2040MP()
        self.verbose = verbose or int(os.getenv("RPRL78_VERBOSE", "0"))

        if init:
            self.verbose and print("loading...")
            # Flush half completed command
            try:
                self.mp.cmd("")
            except BadCommand:
                pass
            self.mp.cmd("import binascii")
            self.mp.cmd("from rl78.proto import try_a1, try_ocd")
            self.mp.cmd("from rl78.proto import power_on, power_off")
            self.mp.cmd("from rl78.misc import erase_all, dump_checksum")
            self.mp.cmd("rl = None")

        if mode == self.MODE_A_1WIRE:
            self.try_a1()
        elif mode == self.MODE_OCD:
            self.try_ocd()
        elif mode is not None:
            raise ValueError("Bad mode")

    def try_a1(self):
        # Took 1.069 sec
        self.mp.cmd("rl = try_a1()", timeout=2.0)
        self.mode = self.MODE_A_1WIRE

    def try_ocd(self):
        self.mp.cmd("rl = try_ocd()", timeout=2.0)
        self.mode = self.MODE_OCD

    """
    ProtoA
    """

    def a_erase_block(self, addr):
        """
        Erase specified blocks
        """
        assert addr % BLOCK_SIZE == 0
        self.mp.cmd("rl.a.erase_block(0x%06X)" % addr)

    def a_program(self, addr, data):
        """
        Write specified blocks
        Must already be erased
        """
        # not 100% true but lets assume for now
        assert len(data) % WRITE_SIZE == 0
        self.mp.cmd("rl.a.program(addr=0x%06X, data=%s)" % (
            addr,
            repr(data),
        ))

    def a_verify(self, addr, data):
        # not 100% true but lets assume for now
        assert len(addr) % WRITE_SIZE == 0
        self.mp.cmd("rl.a.verify(addr=0x%06X, data=%s)" % (
            addr,
            repr(data),
        ))

    def a_write(self, addr, data):
        """
        Erase, write, and verify data
        """
        """
        There is a really weird issue where writes time out in an alternating manner
        relative to power cycles
        Workaround: if a timeout occurs, re-establish ProtoA and retry
        """
        def do_cmd():
            # Took 0.563 sec
            # However can wedge the entire chip, where 1.0 sec timeout needs to propagate
            # Erase is fine, its the first write that causes issue
            # Have not yet found a solution to the wedge
            # Considered rebooting, but that doesn't seem to help much
            self.mp.cmd("rl.a.write(addr=0x%06X, data=%s)" %
                        (addr, repr(data)),
                        timeout=1.2)

        try:
            do_cmd()
        except BadCommand as e:
            if "SerialTimeout: timed out" in str(e):
                self.verbose and print("*" * 80)
                self.verbose and print(
                    "WARNING: failed write, attempting recover")
                self.try_a1()
                do_cmd()
            else:
                raise e

    def a_silicon_sig(self):
        print(self.mp.cmd("rl.a.silicon_sig()"))

    def erase_all(self):
        self.mp.cmd("erase_all(rl)", timeout=10.0)

    def print_checksums(self):
        print(self.mp.cmd("dump_checksum(rl)", timeout=3.0))

    def power_on(self):
        self.mp.cmd("power_on(rl)")

    def power_off(self):
        self.mp.cmd("power_off(rl)")
