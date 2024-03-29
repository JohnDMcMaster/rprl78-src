#!/usr/bin/env python3

from rprl78.util import writej, add_bool_arg
import os
import json
import json5
import intelhex
from rprl78 import emulator
from rprl78 import rp2040

# 0x400
BLOCK_SIZE = 1024
BLOCK_MASK = 0xFFFFFF ^ (BLOCK_SIZE - 1)


def get_pages(fwd):
    """
    Return a list of sector addresses given hex file
    """
    pages = set()
    for k in fwd.keys():
        if type(k) is int:
            page = k & BLOCK_MASK
            pages.add(page)
    return sorted(list(pages))


def run(fn, write=True, emu=None):
    if emu is None:
        emu = emulator.Emulator()

    fw = intelhex.IntelHex(fn)
    fwd = fw.todict()
    used_blocks = get_pages(fwd)
    print("Blocks in use (to erase):")
    for address in used_blocks:
        print("  0x%06X" % address)

    print("Writing...")
    for base_address in used_blocks:
        print("  0x%06X + 0x%03X" % (base_address, BLOCK_SIZE))
        buf = bytearray(BLOCK_SIZE)
        for offset in range(BLOCK_SIZE):
            addr = base_address + offset
            buf[offset] = fwd.get(addr, 0x00)
        emu.a_write(base_address, buf)


def main():
    import argparse

    parser = argparse.ArgumentParser(
        description="Write hex file, erasing minimal flash")
    parser.add_argument("--verbose", action="store_true")
    parser.add_argument("fn", help="File to write")
    args = parser.parse_args()

    run(fn=args.fn)


if __name__ == "__main__":
    main()
