#!/usr/bin/env python3

"""
Gather info from rp2040
You must already have the library sync.sh'd
"""

import subprocess
from rprl78.util import writej
import os

def uprun(expr):
    return subprocess.check_output(["mpremote", "exec", expr])
    

def uprun_misc(func, **kwargs):
    """
    Tried a few cleaner ways to do this
    ex: mpremote will internally call sys.exit() on many conditions
    """
    argstr = ", ".join(["%s=%s" % (k, v) for k, v in kwargs.items()])
    return uprun("from rl78 import misc; misc.%s(%s)" % (func, argstr))


def probe(aggressive=False, out_dir=None):
    print("Connecting ProtoA (1 wire)...")

    """
    Hmm doesn't persist across runs
    """
    if 1:
        uprun("from rl78.proto import try_a1")
        uprun("rl78 = try_a1()")
    else:
        uprun("from rl78.proto import try_a1; rl78 = try_a1()")

    return

    print("Dumping meta json...")
    metaj = uprun_misc("dump_meta_json", {"rl78": "rl78"})
    if out_dir:
        writej(os.path.join(out_dir, "meta.json"), metaj)
    print("Dumping flash checksums...")
    checksumj = uprun_misc("dump_checksum", {"rl78": "rl78", "printj": True})
    if out_dir:
        writej(os.path.join(out_dir, "checksum.json"), checksumj)
    print("Checking OCD status...")
    # Note: leaves in OCD mode
    ocdj = uprun_misc("probe_ocd", {"rl78": "rl78", "printj": True, "aggressive": aggressive})
    if out_dir:
        writej(os.path.join(out_dir, "ocd.json"), ocdj)
    # Most things expect a1 protocol
    print("Connecting ProtoA (1 wire)...")
    uprun("rl78 = try_a1(rl78=rl78)")
    print("Done!")


def main():
    import argparse

    parser = argparse.ArgumentParser(description="Gather misc info")
    parser.add_argument("--aggressive", action="store_true")
    parser.add_argument("--verbose", action="store_true")
    args = parser.parse_args()

    probe(aggressive=args.aggressive)


if __name__ == "__main__":
    main()
