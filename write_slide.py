#!/usr/bin/env python3
"""
Write a nop slide firmware
Discard the first erase sector
"""

from rprl78.util import writej, add_bool_arg
import os
import json
import json5
import intelhex
from rprl78 import emulator
from probe import max_code_address

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


def run(fn, max_addr=None, write=True):
    # Run before Emulator() as it will use the serial port
    if max_addr is None:
        max_addr = max_code_address()
    print("Detected max address: 0x%06X" % max_addr)

    emu = emulator.Emulator()

    fw = intelhex.IntelHex(fn)
    fwd = fw.todict()
    used_blocks = get_pages(fwd)
    print("Blocks in original hex:")
    for address in used_blocks:
        print("  0x%06X" % address)
    # The boot sector might have been written as a vector table
    if 0 in used_blocks:
        used_blocks.remove(0)
    print("Blocks after filtering:")
    for address in used_blocks:
        print("  0x%06X" % address)

    print("Writing...")
    for base_address in range(BLOCK_SIZE, max_addr, BLOCK_SIZE):
        # Be really sure...
        if base_address == 0:
            continue
        print("  0x%06X + 0x%03X" % (base_address, BLOCK_SIZE))
        buf = bytearray(BLOCK_SIZE)
        # Blocks with actual firmware are kept
        # Everything else gets NOPs, which happens to be 0x00,
        # the bytearray default value
        if base_address in used_blocks:
            for offset in range(BLOCK_SIZE):
                addr = base_address + offset
                buf[offset] = fwd.get(addr, 0x00)
        if write:
            emu.a_write(base_address, buf)


def main():
    import argparse

    parser = argparse.ArgumentParser(
        description=
        "Write firmware late in the flash, filling with NOPs before it. Direct via RP2040"
    )
    add_bool_arg(parser, "--write", default=True, help="Write after erasing")
    parser.add_argument("--max-addr", default=None)
    parser.add_argument("--verbose", action="store_true")
    parser.add_argument("fn", help="File to write")
    args = parser.parse_args()

    max_addr = None
    if args.max_addr is not None:
        max_addr = int(args.max_addr, 0)
    # max_addr = 0x007FFF
    run(fn=args.fn, max_addr=max_addr, write=args.write)


if __name__ == "__main__":
    main()
