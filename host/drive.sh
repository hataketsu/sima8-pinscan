#!/bin/zsh
# drive.sh <pincode-hex> <mode>   mode: 1=HIGH 2=LOW 3=BLINK 0=release
PIN=$1; MODE=$2
CMD=$(( (MODE << 16) | PIN ))
cd /tmp
openocd -f interface/stlink.cfg -c "transport select hla_swd" -f target/stm32f0x.cfg \
  -c "init" \
  -c "mww 0x200000AC $CMD" \
  -c "mem2array a 32 0x200000B0 1" \
  -c "mww 0x200000B0 [expr {\$a(0)+1}]" \
  -c "resume" -c "shutdown" 2>&1 | grep -iE '^Error'
