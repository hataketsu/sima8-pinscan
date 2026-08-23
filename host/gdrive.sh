#!/bin/zsh
# gdrive.sh <maskA> <maskB> <maskF> <mode>   mode: 4=nhom len CAO, 5=nhom xuong THAP, 0=tha het
MA=$1; MB=$2; MF=$3; MODE=$4
CMD=$(( MODE << 16 ))
cd /tmp
openocd -f interface/stlink.cfg -c "transport select hla_swd" -f target/stm32f0x.cfg \
  -c "init" \
  -c "mww 0x200000BC $MA" -c "mww 0x200000C0 $MB" -c "mww 0x200000C4 $MF" \
  -c "mww 0x200000AC $CMD" \
  -c "mem2array a 32 0x200000B0 1" \
  -c "mww 0x200000B0 [expr {\$a(0)+1}]" \
  -c "resume" -c "shutdown" 2>&1 | grep -iE '^Error'
