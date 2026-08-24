TC   = $(HOME)/opt/arm-gnu-toolchain-15.2.rel1/bin/arm-none-eabi-
CC   = $(TC)gcc
OC   = $(TC)objcopy
SZ   = $(TC)size

CFLAGS  = -mcpu=cortex-m3 -mthumb -Os -g3 -std=c11 -Wall -Wextra \
          -ffunction-sections -fdata-sections -fno-builtin
LDFLAGS = -T stm32f103.ld -nostdlib -Wl,--gc-sections -Wl,-Map=ground.map

all: ground.bin

ground.elf: main.c usb_cdc.c usb_cdc.h ../common/start.c ../common/protocol.h stm32f103.ld
	$(CC) $(CFLAGS) $(LDFLAGS) main.c usb_cdc.c ../common/start.c -o $@
	$(SZ) $@

ground.bin: ground.elf
	$(OC) -O binary $< $@

flash: ground.bin
	st-flash --reset write ground.bin 0x08000000

clean:
	rm -f ground.elf ground.bin ground.map

# Flash over the ROM UART bootloader instead of SWD.
# Set BOOT0 = 1 and reset first; set it back to 0 and reset when done.
PORT ?= /dev/cu.usbserial-11230
uart-flash: ground.bin
	../host/uart_flash.sh ground.bin $(PORT)
