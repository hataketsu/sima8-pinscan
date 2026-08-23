TC   = $(HOME)/opt/arm-gnu-toolchain-15.2.rel1/bin/arm-none-eabi-
CC   = $(TC)gcc
OC   = $(TC)objcopy
SZ   = $(TC)size

CFLAGS = -mcpu=cortex-m0 -mthumb -Os -g3 -std=c11 -Wall -Wextra \
         -ffunction-sections -fdata-sections -fno-builtin -fno-jump-tables
LDFLAGS = -T stm32f031k6.ld -nostdlib -Wl,--gc-sections -Wl,-Map=pinscan.map

all: pinscan.bin

pinscan.elf: pinscan.c start.c stm32f031k6.ld
	$(CC) $(CFLAGS) $(LDFLAGS) pinscan.c start.c -o $@
	$(SZ) $@

pinscan.bin: pinscan.elf
	$(OC) -O binary $< $@

flash: pinscan.bin
	st-flash --reset write pinscan.bin 0x08000000

clean:
	rm -f pinscan.elf pinscan.bin pinscan.map
