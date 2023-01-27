Goal: a framework for analyzing RL78/G13 devices.
This is intended to allow more advanced device programming
(ex: set finer grained security permissions) as well as to
allow security research

# 
Basic idea:
Use an RP2040 which has internal GPIO-UART muxes to simplify the original test setup
It runs micropython => the code can be ported over relatively quickly

Signaling is open drain

pinout
GPIO0: UART0.tx
GPIO1: UART0.rx
GPIO16: DEBUG1





# Reading memory

You only need TOOL0. However, if you want to match my setup exactly:
* 0: VCC
* 1: TOOL0
* 2: RESET
* 3: DEBUG1

I used a Saleae LogicPro16 at the lowest bandwith it would allow




# History

Fork of: https://github.com/JohnDMcMaster/rl78-debug
(https://github.com/fail0verflow/rl78-debug)

Note: there are also other fixes incorporated
(ex: stop bits, more proper ack handling)

