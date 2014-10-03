#!/bin/bash

## Call as:
#  ./platform/atmel_sam-flash.sh ${ELF_PATH}.elf

../Resources/TinyG2-OSX-Programmer/DueFromOSX.sh -f "${1}" -p /dev/tty.usbmodem001