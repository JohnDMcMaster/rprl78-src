from rl78.proto import try_a1
from rl78.misc import decode_sig
from rl78.util import hexdump
import binascii


def main():
    rl78 = try_a1()
    print("")
    sig = rl78.a.silicon_sig()
    hexdump(sig, "sig")
    print(binascii.hexlify(sig))
    decode_sig(sig)
    print("")


if 0 or __name__ == "__main__":
    main()
