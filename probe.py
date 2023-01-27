#!/usr/bin/env python3
"""
Gather info from rp2040
You must already have the library sync.sh'd
"""

import subprocess
from rprl78.util import writej
import os
import json
import json5


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
    print("Writing results to %s" % (out_dir, ))
    if not os.path.exists(out_dir):
        os.mkdir(out_dir)
    """
    Hmm doesn't persist across runs
    print("Connecting ProtoA (1 wire)...")
    if 1:
        uprun("from rl78.proto import try_a1")
        uprun("rl78 = try_a1()")
    else:
        uprun("from rl78.proto import try_a1; rl78 = try_a1()")
    """
    def write_jraw(basename, jraw):
        print("  Writing output...")
        if 0:
            with open(os.path.join(out_dir, basename) + ".raw", "wb") as f:
                f.write(jraw)
        j = json5.loads(jraw)
        writej(os.path.join(out_dir, basename), j)

    print("Dumping meta json...")
    jraw = uprun_misc("dump_meta_json")
    if out_dir:
        write_jraw("meta.json", jraw)
    print("Dumping flash checksums...")
    jraw = uprun_misc("dump_checksum", printj=True)
    if out_dir:
        write_jraw("checksum.json", jraw)
    print("Checking OCD status...")
    # Note: leaves in OCD mode
    jraw = uprun_misc("probe_ocd", printj=True, aggressive=aggressive)
    if out_dir:
        write_jraw("ocd.json", jraw)
    print("Done!")


def main():
    import argparse

    parser = argparse.ArgumentParser(description="Gather misc info")
    parser.add_argument("--aggressive", action="store_true")
    parser.add_argument("--verbose", action="store_true")
    parser.add_argument("out_dir", nargs="?", default="probe")
    args = parser.parse_args()

    probe(aggressive=args.aggressive, out_dir=args.out_dir)


if __name__ == "__main__":
    main()
