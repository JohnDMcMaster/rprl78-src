#include "ior5f100le.h"
#include "ior5f100le_ext.h"
//#include <stdint.h>

void delay_ms(unsigned int ms) {
  for (int i = 0; i < ms; ++i) {
      for (int j = 0; j <  1325; ++j) {
           asm("NOP");
      }
  }
}

void send_byte(unsigned char byte) {
  //Hack for shitty serial reliable timing
  //use software decoding to set an appropriate baud rate
  //Start bit + 8 data bits + stop bit
  unsigned char bits[12];
  bits[0] = 0;
  bits[9] = 1;
  bits[10] = 1;
  bits[11] = 1;
  P4_bit.no0 = 1;
  for (int i = 0; i < 8; ++i) {
      bits[i + 1] = byte & 1;
      byte = byte >> 1;
  }
  for (int i = 0; i < sizeof(bits); ++i) {
      P7_bit.no7 = bits[i];
      P4_bit.no0 = bits[i];
      //Try to make the timing more consistent
      //131 loops: 99.5 us bit
      //132 loops: 100.5 us bit
      for (int j = 0; j <  132; ++j) {
        //__asm volatile("NOP\n\t");     
           asm("NOP");
      }
  }
}

void  main(void)
{
  /*
  LED appears to be open drain
  */
  //Connect on-chip pull-up resistor to the LED port pin
  //(why?)
  PU7_bit.no7 = 1;
  //Configure as output
  PM7_bit.no7 = 0;

    //TOOL0 as output
  PM4_bit.no0 = 0;
  //NRZ high
  P4_bit.no0 = 1;
  
  /*
DFLCTL
  default: 0
Figure 25-6. Format of Data Flash Control Register (DFLCTL)
  0: Disables data flash access
  1: Enables data flash access
  w/o garbage values come back
  low entropy, mostly 00s
  */
  //  __near __no_init volatile union { unsigned char DFLCTL; __BITS8 DFLCTL_bit; } @ 0xF0090;
  //#define DFLEN             DFLCTL_bit.no0
  DFLEN = 1;
    while (1) {
        unsigned int addr_min = 0x0000;
        unsigned int addr_max = addr_min + 0x200;
      delay_ms(100);
      for (unsigned addr = 0; addr_min < addr_max; ++addr) {
          unsigned char b = *(unsigned char *)addr;
          //unsigned b = *(unsigned char *)0x100;
          send_byte(b);
        }
  }
}
  