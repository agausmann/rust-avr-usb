#!/bin/sh

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
sudo dfu-programmer "$MCU" reset