#!/usr/bin/env python3

from rp2040 import RP2040MP
"""

"""


def main():
    import argparse

    parser = argparse.ArgumentParser(description="rp2040 tool")
    parser.add_argument('--verbose', action="store_true")
    parser.add_argument('--load', action="store_true")
    parser.add_argument('--run', action="store_true")
    args = parser.parse_args()

    mp = RP2040MP()
    # mp.reset()
    if args.load:
        print("loading...")
        mp.paste(open("main_rp2040.py", "r").read())
    if args.run:
        mp.paste("\n\n\nmain()")


if __name__ == "__main__":
    main()
