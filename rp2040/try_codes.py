from rl78.misc import dump_meta_json
from rl78.util import hexdump
from rl78.proto import try_ocd

def main():
    # Prefix run with metadata to clearly log if this triggered erase
    dump_meta_json()

    codes = []
    codes.append(b"\x00" * 10)
    codes.append(b"\xFF" * 10)
    codes.append(bytearray([x for x in range(10)]))
    codes.append(bytearray([9 - x for x in range(10)]))
    for x in range(0x100):
        codes.append(bytearray([x] * 10))
    for code in codes:
        print("")
        print("")
        print("")
        rl78 = try_ocd()
        print("unlocking...")
        hexdump(code)
        if rl78.ocd.unlock(code):
            print("Struck gold!")
            break
    else:
        print("")
        print("")
        print("")
        print("no :(")

if 0 or __name__ == "__main__":
    main()
