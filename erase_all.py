#!/usr/bin/env python3

from rprl78 import emulator


def main():
    import argparse

    parser = argparse.ArgumentParser(
        description="Erasre entire flash memory (code + program)")
    parser.add_argument("--im-sure",
                        action="store_true",
                        help="Turn the PAL keys")
    args = parser.parse_args()

    if not args.im_sure:
        raise ValueError("You must set --im-sure to erase the entire device")

    print("Connecting...")
    emu = emulator.Emulator()
    print("Erasing...")
    emu.erase_all()
    print("Device erased")


if __name__ == "__main__":
    main()
