# Bootloader for the ground unit

Two were tried. The second is the one in use.

## Why not the STM32duino DFU bootloader

`stm32duino_boot20_pc13_8k.bin` is here for reference. It flashes and enumerates
correctly — macOS sees `Maple 003` at `1eaf:0003` and `dfu-util -l` lists three
alt settings — but every transfer dies at the same place:

```
dfu-util: Failed to retrieve language identifiers
dfu-util: Cannot set alternate interface: LIBUSB_ERROR_OTHER
```

The bootloader's string-descriptor handling does not satisfy the USB stack on
current macOS, which then refuses `SET_INTERFACE`. It reportedly still works
from Linux and Windows. Rather than depend on another machine, the ground unit
moved to a HID bootloader, which needs no interface claiming at all.

Note that the published `generic_boot20_*.bin` files are combined images: the
bootloader occupies the first 8 KB and a second vector table at offset `0x2000`
carries a bundled application declaring `SP = 0x20005000`. That is the top of
20 KB of RAM and wrong for this part. Only the first 8 KB was ever flashed.

## The HID bootloader in use

`hid_boot_pc13_10kram.bin`, built from
https://github.com/Serasidis/STM32_HID_Bootloader tag 2.2.2 with two changes,
because the stock build targets a 20 KB part:

| Where | Stock | Here |
| --- | --- | --- |
| `Src/main.c` `SRAM_SIZE` | `20 * 1024` | `10 * 1024` |
| `STM32F103C8T6.ld` `_estack` | `0x20005000` | `0x20002800` |
| `STM32F103C8T6.ld` RAM / FLASH | 20K / 64K | 10K / 32K |

`SRAM_SIZE` is the one that matters and it is not taken from the linker script:
`main.c` builds the vector table itself and writes `SRAM_END` into the initial
stack pointer slot. Patching only the linker leaves `SP = 0x20005000` in the
binary, which faults on the first stack access.

Verified after building: `SP = 0x20002800`, `Reset = 0x08000015`, 1984 bytes.

## Layout

| Address | Contents | Size |
| --- | --- | --- |
| `0x08000000` | HID bootloader | 2 KB |
| `0x08000800` | application | 30 KB available |

## Flashing

Bootloader, over the ROM UART bootloader with BOOT0 = 1 and a reset first:

```sh
stm32flash -b 115200 -w bootloader/hid_boot_pc13_10kram.bin -v -S 0x08000000 /dev/cu.usbserial-XXXX
```

Application, over USB once the bootloader is in place:

```sh
host/hid-flash ground/ground.bin <serial-port>
```

`host/hid-flash` is built from the same repo's `cli/` for arm64; the released
macOS binary is x86_64 only.
