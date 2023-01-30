#!/usr/bin/env python3

import subprocess
from rprl78.util import tostr


def uprun(expr):
    return subprocess.check_output(["mpremote", "exec", expr])


def uprun_misc(func, **kwargs):
    """
    Tried a few cleaner ways to do this
    ex: mpremote will internally call sys.exit() on many conditions
    """
    argstr = ", ".join(["%s=%s" % (k, v) for k, v in kwargs.items()])
    return uprun("from rl78 import misc; misc.%s(%s)" % (func, argstr))


def uprun_misc_str(func, **kwargs):
    return tostr(uprun_misc(func, **kwargs))


def main():
    import argparse

    parser = argparse.ArgumentParser(description="Gather misc info")
    _args = parser.parse_args()

    print("Dumping flash checksums...")
    print(uprun_misc_str("dump_checksum", printj=False))


if __name__ == "__main__":
    main()
