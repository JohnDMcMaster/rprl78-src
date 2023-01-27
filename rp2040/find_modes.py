from rl78.proto import RL78, SerialTimeout

def main():
    """
    0x3A: ok
    0xC5: ok
    It did not detect 0x00 b/c requires 2 wire
    """
    rl78 = RL78(verbose=False)
    responses = []
    for mode in range(0x100):
        if mode < 0x3A:
            continue
        try:
            rl78.reset(bytearray([mode]), flush=True, probe=False)
            print("0x%02X: ok" % mode)
            responses.append(mode)
        except SerialTimeout:
            print("0x%02X: timeout" % mode)
            pass
    print("")
    for mode in responses:
        print("0x%02X: ok" % mode)


if 0 or __name__ == "__main__":
    main()
