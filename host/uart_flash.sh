#!/bin/zsh
# Flash an STM32 through its ROM UART bootloader — no ST-Link needed.
#
#   uart_flash.sh <firmware.bin> [port] [baud]
#
# The bootloader lives on USART1 (PA9 TX, PA10 RX) on both the F103 and the
# F031, and it is the same pair used for the debug console, so one adapter
# covers flashing and logging.
#
# Before running: BOOT0 = 1, then reset the board. After it finishes: BOOT0 = 0,
# reset again to run the firmware.
set -e
BIN=${1:?usage: uart_flash.sh <firmware.bin> [port] [baud]}
PORT=${2:-/dev/cu.usbserial-11230}
BAUD=${3:-115200}

[[ -f $BIN ]] || { echo "khong thay file: $BIN"; exit 1; }
[[ -e $PORT ]] || { echo "khong thay cong: $PORT"; exit 1; }

echo "port=$PORT baud=$BAUD file=$BIN ($(stat -f%z $BIN) byte)"
echo "kiem tra bootloader..."
stm32flash -b $BAUD $PORT

echo
echo "nap va xac minh..."
stm32flash -b $BAUD -w "$BIN" -v -g 0x08000000 $PORT
