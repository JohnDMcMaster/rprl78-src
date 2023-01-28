Goal: a framework for analyzing RL78/G13 devices.
This is intended to allow more advanced device programming
(ex: set finer grained security permissions) as well as to
allow security research

How:
Use an RP2040 which has internal GPIO-UART muxes to simplify the original test setup
It runs micropython => the code can be ported over relatively quickly


# Hardware setup

RP2040 running micropython (ex: rp2-pico-20220618-v1.19.1.uf2)

TOOL0: 1 wire open drain
* PU resistor required
* RX line directly connected
* TX line connected via diode (such that it can only pull low)

RP2040 pinout
* GPIO0: UART0.tx
* GPIO1: UART0.rx
* GPIO16: DEBUG1 (optional)

Other: I was using Dediprog socket modules
TODO: design PCBs in addition to my protoboard version

# Software setup

Experimenting with different micropython implementations. Currently three interfaces are used:
* ampy
* mpremote
* DIY interface


# Reading memory

You only need TOOL0. However, if you want to match my setup exactly:
* 0: VCC
* 1: TOOL0
* 2: RESET
* 3: DEBUG1

I used a Saleae LogicPro16 at the lowest bandwith it would allow

First do a pass to get the baudrate. Then setup a serial analyzer and export a .csv


# History

Fork of: https://github.com/JohnDMcMaster/rl78-debug
(https://github.com/fail0verflow/rl78-debug)

Note: there are also other fixes incorporated
(ex: stop bits, more proper ack handling)

