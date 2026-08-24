TC   = $(HOME)/opt/arm-gnu-toolchain-15.2.rel1/bin/arm-none-eabi-
CC   = $(TC)gcc
OC   = $(TC)objcopy
SZ   = $(TC)size

CFLAGS  = -mcpu=cortex-m0 -mthumb -Os -g3 -std=c11 -Wall -Wextra \
          -ffunction-sections -fdata-sections -fno-builtin -fno-jump-tables
LDFLAGS = -T stm32f031k6.ld -nostdlib -Wl,--gc-sections -Wl,-Map=drone.map

all: drone.bin

drone.elf: main.c ../common/start.c ../common/protocol.h stm32f031k6.ld
	$(CC) $(CFLAGS) $(LDFLAGS) main.c ../common/start.c -o $@
	$(SZ) $@

drone.bin: drone.elf
	$(OC) -O binary $< $@

flash: drone.bin
	st-flash --reset write drone.bin 0x08000000

clean:
	rm -f drone.elf drone.bin drone.map

# Flash over the ROM UART bootloader instead of SWD.
# Set BOOT0 = 1 and reset first; set it back to 0 and reset when done.
PORT ?= /dev/cu.usbserial-11230
uart-flash: drone.bin
	../host/uart_flash.sh drone.bin $(PORT)
