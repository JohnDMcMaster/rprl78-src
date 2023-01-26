#!/usr/bin/env python3

import csv


def load_csv(fn):
    with open(fn, newline='') as csvfile:
        csvfile.readline()
        for row in csv.reader(csvfile):
            yield row


def get_csv_packets(fn):
    buf = bytearray()
    print("Finding bytes...")
    for row in load_csv(fn):
        # ['0.016552800000000', '0x30', '', '']
        b = int(row[1], 0)
        buf += bytearray([b])
    return buf


def main():
    import argparse

    parser = argparse.ArgumentParser(description="Dump packets from CSV")
    parser.add_argument("--verbose", action="store_true")
    parser.add_argument("--skip", type=str)
    parser.add_argument("--limit", type=str)
    parser.add_argument("fn_in")
    parser.add_argument("fn_out", nargs="?")
    args = parser.parse_args()

    fn_out = args.fn_out
    if not fn_out:
        fn_out = args.fn_in.replace(".csv", ".bin")
        assert fn_out != args.fn_in

    buf = get_csv_packets(args.fn_in)
    print("Loaded 0x%04X bytes" % len(buf))
    skip = 0
    if args.skip:
        skip = int(args.skip, 0)
    if args.skip:
        pre = len(buf)
        buf = buf[skip:]
        print("Skip to 0x%04X bytes (%0.1f%%)" %
              (len(buf), len(buf) / pre * 100.0))
    if args.limit:
        pre = len(buf)
        buf = buf[0:int(args.limit, 0)]
        print("Trimmed to 0x%04X bytes (%0.1f%%)" %
              (len(buf), len(buf) / pre * 100.0))
    print("Final size 0x%04X" % len(buf))
    with open(fn_out, "wb") as f:
        f.write(buf)


if __name__ == "__main__":
    main()
