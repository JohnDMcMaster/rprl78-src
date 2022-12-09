#!/usr/bin/env python3

from rp2040 import RP2040MP

mp = RP2040MP()
# mp.reset()
print("loading...")
mp.paste(open("main_py.py", "r").read())
mp.paste("\n\n\nmain()")
#mp.cmd("main()")
