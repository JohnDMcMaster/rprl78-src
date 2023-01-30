#!/usr/bin/env python3

from rprl78 import emulator


def main():
    import argparse

    parser = argparse.ArgumentParser(description="Power on")
    args = parser.parse_args()

    emu = emulator.Emulator(mode=None)
    emu.power_off()


if __name__ == "__main__":
    main()
