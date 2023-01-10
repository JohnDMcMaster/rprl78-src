Basic idea:
Use an RP2040 which has internal GPIO-UART muxes to simplify the original test setup
It runs micropython => the code can be ported over relatively quickly

Signaling is open drain

pinout
GPIO0: UART0.tx
GPIO1: UART0.rx
GPIO16: DEBUG1




Note: there are also other fixes incorporated
(ex: stop bits, more proper ack handling)





Logic analyzer:
0: VCC
1: TOOL0
2: RESET
3: DEBUG1

Initial communications are 115200
Then to 
