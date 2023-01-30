#!/usr/bin/env python3
"""
Write a nop slide firmware
Discard the first erase sector
"""

import intelhex

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


def run(fn_in, fn_out, write=True):
    fw = intelhex.IntelHex(fn_in)
    fwd = fw.todict()
    max_addr = max_addr_from_ihex(fw)
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

    print("Writing...")
    fw_out = intelhex.IntelHex()
    for base_address in range(BLOCK_SIZE, max_addr, BLOCK_SIZE):
        # Be really sure to skip boot sector
        if base_address == 0:
            continue
        print("  0x%06X + 0x%03X" % (base_address, BLOCK_SIZE))
        # Blocks with actual firmware are kept
        # Everything else gets NOPs (0x00)
        for offset in range(BLOCK_SIZE):
            addr = base_address + offset
            fw_out[addr] = fwd.get(addr, 0x00)
    with open(fn_out, "w") as fout:
        fw_out.write_hex_file(fout)


def main():
    assert 0, "Doesn't work: undocumented flash write restrictions?"

    import argparse

    parser = argparse.ArgumentParser(
        description=
        "Write firmware late in the flash, filling with NOPs before it. Write out .hex file for E1 etc"
    )
    parser.add_argument("--verbose", action="store_true")
    parser.add_argument("fn_in", help="Hex file in")
    parser.add_argument("fn_out", help="Hex file out")
    args = parser.parse_args()

    run(fn_in=args.fn_in, fn_out=args.fn_out)


if __name__ == "__main__":
    main()
