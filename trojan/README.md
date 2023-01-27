Notes for building the reference trojan + general development notes

Reference build system:
* Windows 7
* Started with YRPBRL78G13 dev board / starter kit
* Software: IAR IDE 6.1.5 w/ RL78/G13
  * YRPBRL78G13 kit => RL78_G13_RPB.zip from CD / website
  * IAR Embedded Workbench IDE: 6.1.5.1809 (6.1.5.1809)
  * Had to email for a license, was pretty quick / easy
* Renesas E1 programmer


Note: other Renesas build platforms are availible, but it looked like they cost money.
Consider GCC if more development is required



# IAR development

General IAR notes / limitations:
* Debug mode is more picky about generating extra files than release mode

To recreate from scratch (C example):
New workspace
Project => Create New Project
Select Release configuration
Right click => options
    General options => Device: change to your target chip
    Linker => Extra Output: enable and override name to end in .hex. Format: intel-extended
    Linker => Override default: if you want code to start at 0x100 use modified file
    Debugger => Driver: change to E1

Debugging notes:
* Only C builds can be debugged, not assembly
* Seems to set a default OCD password on builds of all 0s
* I had to select target voltage 3.3V
  * 5V (when powered from E1) should have worked? But instead gives error that the ID code doesn't match


# Using flash programmer

I used Renesas_Flash_Programmer_Package_V30902-doc.zip
* Help => About: V3.09.02 (1 Apr 2022)

Example: burning LED blink to dev board using E1
* Select Release configuration
* Build all
* Setup
  * E1 connected to dev board via ribbon cable
  * E1 connected to host via mini USB cable
* Launch Flash Programmer
* File new project
* Select RL78 family
  * Select Release .hex file
* Hit "Run"
* Disconnect E1 ribbon cable
  * It will hold the target in reset
* Connect mini usb cable to dev board
* Plug in mini USB cable to dev board
* You should see LED on but dimmer than power LED


# Disassembling

IAR does not have a disassembler :(
* Workaround 1: when compiling, ask for an annotated listing file that contains assembly
* Workaround 2: use GCC to disassemble

General options:
* https://llvm-gcc-renesas.com/rl78/rl78-download-toolchains/
  * Best option that worked
* IAR
  * Doesn't support
* Renesas CS+
  * Requires license
* https://github.com/mnaberez/k0dasm
  * Not compatible enough
* ghidra
  * Not supported w/o external plugin (ie from fail0verflow)
* ida
  * Didn't try


## LLVM

I used llvm-10.0.0.202212-rl78-elf.exe

Example commands:

# Hex to ELF
rl78-elf-objcopy -I ihex -O elf32-rl78 -B rl78 my.hex my.elf
# Bin to ELF
# Note: I was not able to get anything to add addresses,
# so I hope your code is at address 0
rl78-elf-objcopy -I bin -O elf32-rl78 -B rl78 my.bin my.elf
# Now disassemble
# Couldn't get it to only disassemble the section I want
# So do everything and throw away what you don't want
llvm-objdump --disassemble-all my.elf


## IAR annotated listing

Options => Assembler => List => "Output list file" checked

Sample:

      12            //Configure as output
      13            PM7_bit.no7 = 0;
    \   000005 717B27                CLR1      0xFFF27.7

