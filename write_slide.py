#!/usr/bin/env python3
"""
Write a nop slide firmware
Discard the first erase sector
"""

import intelhex
from rprl78 import emulator
from probe import max_code_address
from rprl78 import rp2040

# 0x400
BLOCK_SIZE = 1024
BLOCK_MASK = 0xFFFFFF ^ (BLOCK_SIZE - 1)


def get_pages(fw):
    """
    Return a list of sector addresses given hex file
    """
    pages = set()
    for k in fw.addresses():
        if type(k) is int:
            page = k & BLOCK_MASK
            pages.add(page)
    return sorted(list(pages))


def max_addr_from_ihex(fw):
    max_addr = int(max(fw.addresses()))
    # Convert to page min address
    max_addr = max_addr & 0xFFFC00
    # Then move to the end of the page
    max_addr = max_addr + 0x3FF
    print("Detected max address: 0x%06X" % max_addr)
    return max_addr


def run(fn, emu=None, bulk_erase=False):
    fw = intelhex.IntelHex(fn)
    max_addr_ihex = max_addr_from_ihex(fw)
    print("hex max address: 0x%06X" % max_addr_ihex)

    if 0:
        # Run before Emulator() as it will use the serial port
        max_addr_det = max_code_address()
        print("Device max address: 0x%06X" % max_addr_det)

        assert max_addr_ihex <= max_addr_det

    if 0:
        print("Restarting rp2040...")
        rp2040.reset()
        print("Reset")

    if emu is None:
        emu = emulator.Emulator()

    fwd = fw.todict()
    used_blocks = get_pages(fw)
    print("Blocks in original hex:")
    for address in used_blocks:
        print("  0x%06X" % address)
    # The boot sector might have been written as a vector table
    if 0 in used_blocks:
        used_blocks.remove(0)
    print("Blocks after filtering:")
    for address in used_blocks:
        print("  0x%06X" % address)

    if bulk_erase:
        print("Erasing...")
        for base_address in range(BLOCK_SIZE, max_addr_ihex, BLOCK_SIZE):
            # Be really sure...
            if base_address == 0:
                continue
            print("  0x%06X + 0x%03X" % (base_address, BLOCK_SIZE))
            emu.a_erase_block(base_address)

        print("Writing...")
        # WRITE_SIZE = 0x100
        WRITE_SIZE = 0x400
        for base_address in range(WRITE_SIZE, max_addr_ihex, WRITE_SIZE):
            # Be really sure...
            if base_address == 0:
                continue
            print("  0x%06X + 0x%03X" % (base_address, WRITE_SIZE))
            buf = bytearray(BLOCK_SIZE)
            # Blocks with actual firmware are kept
            # Everything else gets NOPs, which happens to be 0x00,
            # the bytearray default value
            if base_address in used_blocks:
                for offset in range(BLOCK_SIZE):
                    addr = base_address + offset
                    buf[offset] = fwd.get(addr, 0x00)
            emu.a_program(base_address, buf)
            emu.a_verify(base_address, buf)
    else:
        if 1:
            print("Writing...")
            for base_address in range(BLOCK_SIZE, max_addr_ihex, BLOCK_SIZE):
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
                emu.a_write(base_address, buf)
        # FIXME: test
        # write everything, include base address
        else:
            print("Writing...")
            for base_address in range(0, max_addr_ihex, BLOCK_SIZE):
                print("  0x%06X + 0x%03X" % (base_address, BLOCK_SIZE))
                buf = bytearray(BLOCK_SIZE)
                # Blocks with actual firmware are kept
                # Everything else gets NOPs, which happens to be 0x00,
                # the bytearray default value
                if base_address in used_blocks:
                    for offset in range(BLOCK_SIZE):
                        addr = base_address + offset
                        buf[offset] = fwd.get(addr, 0x00)
                emu.a_write(base_address, buf)


def main():
    import argparse

    parser = argparse.ArgumentParser(
        description=
        "Write firmware late in the flash, filling with NOPs before it. Direct via RP2040"
    )
    parser.add_argument("--verbose", action="store_true")
    parser.add_argument("fn", help="File to write")
    args = parser.parse_args()

    run(fn=args.fn)


if __name__ == "__main__":
    main()
