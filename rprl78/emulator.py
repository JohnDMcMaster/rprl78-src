"""
Emulator => interface on rp2040
"""

from .rp2040 import RP2040MP, BadCommand
import os

WRITE_SIZE = 256
BLOCk_SIZE = 1024
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
            print("loading...")
            self.mp.cmd("import binascii")
            self.mp.cmd("from rl78.proto import try_a1, try_ocd")

        if mode == self.MODE_A_1WIRE:
            self.try_a1()
        elif mode == self.MODE_OCD:
            self.try_ocd()
        elif mode is not None:
            raise ValueError("Bad mode")

    def try_a1(self):
        # Took 1.069 sec
        self.mp.cmd("rl78 = try_a1()", timeout=2.0)
        self.mode = self.MODE_A_1WIRE

    def try_ocd(self):
        self.mp.cmd("rl78 = try_ocd()", timeout=2.0)
        self.mode = self.MODE_OCD

    """
    ProtoA
    """

    def a_erase_block(self, addr):
        """
        Erase specified blocks
        """
        assert addr % BLOCk_SIZE == 0
        self.mp.cmd("rl78.a.erase_block(0x%06X)" % addr)

    def a_program(self, addr, data):
        """
        Write specified blocks
        Must already be erased
        """
        # not 100% true but lets assume for now
        assert len(addr) % WRITE_SIZE == 0
        self.mp.cmd("rl78.a.program(addr=0x%06X, data=%s)" % (
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
            self.mp.cmd("rl78.a.write(addr=0x%06X, data=%s)" %
                        (addr, repr(data)),
                        timeout=1.2)

        # Took 0.563 sec
        # However can stall, where 1.0 sec timeout needs to propagate

        try:
            do_cmd()
        except BadCommand:
            self.verbose and print("WARNING: recovering from wedge")
            self.try_a1()
            do_cmd()

    def a_silicon_sig(self):
        print(self.mp.cmd("rl78.a.silicon_sig()"))
