#!/bin/sh

# Script to automate programming with DFU bootloaders
# After compiling, flash with:
# ./dfu-flash atmega32u4 target/avr-atmega32u4/{debug or release}/rust-avr-usb.elf

if [ $# -lt 2 ]
then
    echo "usage: mcu binfile"
    exit 1
fi

MCU=$1
BINFILE=$2

set -e
objcopy -O ihex "$BINFILE" "$BINFILE.hex"
sudo dfu-programmer "$MCU" erase
sudo dfu-programmer "$MCU" flash "$BINFILE.hex"
sudo dfu-programmer "$MCU" launch --no-reset