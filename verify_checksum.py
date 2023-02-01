#!/usr/bin/env python3

import json


class RL78Memory:
    def __init__(self, metaj, code, data):
        self.metaj = metaj
        self.code = code
        self.data = data
        self.block_size = 0x100
        self.block_mask = 0xFFFC00
        self.code_addr_low = 0x000000
        self.code_addr_high = self.metaj["silicon_sig"]["code_flash_addr_hi"]
        self.data_addr_low = 0x0F1000
        self.data_addr_high = self.metaj["silicon_sig"]["data_flash_addr_hi"]

    def flash_code_size(self):
        return self.metaj["silicon_sig"][
            "code_flash_addr_hi"] - self.code_addr_low + 1

    def flash_data_size(self):
        return self.metaj["silicon_sig"][
            "data_flash_addr_hi"] - self.data_addr_low + 1

    def is_code_addr(self, addr):
        #return addr < 0x0F1100
        return self.code_addr_low <= addr <= self.code_addr_high

    def is_data_addr(self, addr):
        #return addr >= 0x0F1100
        return self.data_addr_low <= addr <= self.data_addr_high

    def map_address(self, address, size):
        if self.is_code_addr(address):
            return self.code[address:address + size]
        else:
            off = address - self.data_addr_low
            return self.data[off:off + size]


def verify_blank(rl, verbose=False):
    print("Verifying blanks...")
    code_matches = 0
    code_entries = 0
    code_blanks = 0
    data_matches = 0
    data_entries = 0
    data_blanks = 0
    """
    "block_blank_checks": {
        "0x000000": {
            "is_blank": false,
            "raw_st1": "1b",
            "raw_tx": "32000000ff030000",
            "start_addr": 0
        },
    """
    BLOCK_SIZE = 1024

    def is_blank(buf):
        return sum(buf) == 0xFF * BLOCK_SIZE

    for blockk, blockv in rl.metaj["block_blank_checks"].items():
        meta_blank = blockv["is_blank"]
        address = blockv["start_addr"]
        # Skip overwritten memory
        if address < 0x400:
            continue
        bin_blank = is_blank(rl.map_address(address, BLOCK_SIZE))
        match = meta_blank == bin_blank

        if rl.is_code_addr(address):
            code_entries += 1
            if meta_blank:
                code_blanks += 1
            if match:
                code_matches += 1
        elif rl.is_data_addr(address):
            data_entries += 1
            if meta_blank:
                data_blanks += 1
            if match:
                data_matches += 1
        else:
            assert 0

        if verbose:
            print("%s: JSON %u, bin %u => %s" %
                  (blockk, meta_blank, bin_blank, match))
    print("Code")
    print("  %u / %u blank status match vs .bin" %
          (code_matches, code_entries))
    code_used = code_entries - code_blanks
    print("  %u / %u used => %0.1f%% used" %
          (code_used, code_entries, code_used / code_entries * 100.0))
    print("Data")
    print("  %u / %u blank status match vs .bin" %
          (data_matches, data_entries))
    data_used = data_entries - data_blanks
    print("  %u / %u used => %0.1f%% used" %
          (data_used, data_entries, data_used / data_entries * 100.0))


def checksum16(data):
    csum = 0
    for d in data:
        csum -= d
        csum &= 0xffff
    return csum


def verify_checksums(rl, checkj, verbose=False):
    print("Verifying checksums...")
    CHECKSUM_BLANK = 0x100
    # address < 0x400
    # omitted from some files, so hard code here?
    # Checksum matches
    code_matches = 0
    # Number of sections not 0x100
    code_used = 0
    code_used_matches = 0
    # Number of checksum entries
    code_entries = 0
    # Assume that nothing matches
    if "0x000000" not in checkj:
        code_matches = 0
        code_entries = 4
        code_used_matches = 0
        code_used = 4

    data_entries = 0
    data_used = 0
    data_used_matches = 0
    data_matches = 0
    # data_issues = 0
    for checkk, checkv in checkj.items():
        checksum_j = checkv["checksum"]
        address = checkv["address"]
        # Skip overwritten memory
        if address < 0x400:
            if verbose:
                print("%s: skipping" % (checkk, ))
            code_entries += 1
            # estimated blank when 0x100
            if checksum_j != CHECKSUM_BLANK:
                code_used += 1
            continue

        size = checkv["size"]
        checksum_bin = checksum16(rl.map_address(address, size))
        match = checksum_j == checksum_bin
        if verbose:
            print("%s: JSON 0x%04X, bin 0x%04X => %s" %
                  (checkk, checksum_j, checksum_bin, match))

        if rl.is_code_addr(address):
            code_entries += 1
            if checksum_j != CHECKSUM_BLANK:
                code_used += 1
                if match:
                    code_used_matches += 1
            if match:
                code_matches += 1
        elif rl.is_data_addr(address):
            data_entries += 1
            if checksum_j != CHECKSUM_BLANK:
                data_used += 1
                if match:
                    data_used_matches += 1
            if match:
                data_matches += 1
        else:
            print("0x%X" % address)
            assert 0

    print("Code")
    print("  Overall: %u / %u matches => %0.1f%% recovered" %
          (code_matches, code_entries, code_matches / code_entries * 100.0))
    print(
        "  Used:    %u / %u matches => %0.1f%% recovered" %
        (code_used_matches, code_used, code_used_matches / code_used * 100.0))
    print("Data")
    print("  Overall: %u / %u matches => %0.1f%% recovered" %
          (data_matches, data_entries, data_matches / data_entries * 100.0))
    print(
        "  Used:    %u / %u matches => %0.1f%% recovered" %
        (data_used_matches, data_used, data_used_matches / data_used * 100.0))


def main():
    import argparse

    parser = argparse.ArgumentParser(description="Dump packets from CSV")
    parser.add_argument("--verbose", action="store_true")
    parser.add_argument("--code-400",
                        help="Flash-code .bin w/ first 0x400 bytes invalid")
    parser.add_argument("--data", help="Flash-data .bin")
    parser.add_argument("--probe",
                        help="Probe directory (meta.json, checksum.json)")
    args = parser.parse_args()

    code400 = open(args.code_400, "rb").read()
    data = open(args.data, "rb").read()
    checkj = json.load(open(args.probe + "/checksum.json", "r"))["checksum"]
    metaj = json.load(open(args.probe + "/meta.json", "r"))
    rl = RL78Memory(metaj, code400, data)

    print("MCU: %s" % metaj["silicon_sig"]["device_name"].strip())
    print("Flash code bytes: 0x%X" % rl.flash_code_size())
    print("Flash data bytes: 0x%X" % rl.flash_data_size())
    print("")
    verify_blank(rl=rl, verbose=args.verbose)
    print("")
    verify_checksums(rl=rl, checkj=checkj, verbose=args.verbose)


if __name__ == "__main__":
    main()
