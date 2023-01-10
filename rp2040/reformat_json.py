#!/usr/bin/env python3

import json


def main():
    import argparse

    parser = argparse.ArgumentParser(description="Make JSON pretty")
    parser.add_argument("fn")
    args = parser.parse_args()

    print("Reading...")
    with open(args.fn, "r") as f:
        j = json.load(f)

    print("Writing...")
    with open(args.fn, "w") as f:
        json.dump(j, f, sort_keys=True, indent=4, separators=(",", ": "))

if __name__ == "__main__":
    main()
