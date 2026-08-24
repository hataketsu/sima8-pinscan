# STM32duino DFU bootloader, trimmed for a low-density F103

`stm32duino_boot20_pc13_8k.bin` is the first 8 KB of
`generic_boot20_pc13.bin` from
https://github.com/rogerclarkmelbourne/STM32duino-bootloader

The published 22268-byte file is a combined image, not a bootloader on its own:
the bootloader sits in the first 8 KB and a second vector table begins at offset
`0x2000` carrying a bundled application. That application declares
`SP = 0x20005000`, which is the top of 20 KB of RAM — right for a C8, wrong for
the low-density part on this board, which has 10 KB and ends at `0x20002800`.
Flashing the whole file would have installed an application that faults on its
first instruction and eaten 22 KB of a 32 KB flash.

Only the bootloader half is kept here. It checks out:

| Field | Value |
| --- | --- |
| Initial SP | `0x20002800` — top of the 10 KB actually present |
| Reset_Handler | `0x080000F1` — inside the 8 KB region |
| Real code extent | `0x1C04`, the rest padding |

## Layout

| Address | Contents | Size |
| --- | --- | --- |
| `0x08000000` | bootloader | 8 KB |
| `0x08002000` | application | 24 KB available |

The application links at `0x08002000` and sets `VTOR` to match.

## Flashing it back

Over the ROM UART bootloader, with BOOT0 = 1 and a reset first:

```sh
stm32flash -b 115200 -w bootloader/stm32duino_boot20_pc13_8k.bin -v -S 0x08000000 /dev/cu.usbserial-XXXX
stm32flash -b 115200 -w ground/ground.bin -v -S 0x08002000 /dev/cu.usbserial-XXXX
```

Then BOOT0 = 0 and power-cycle. After that, `dfu-util` flashes the application
over USB and the UART is free.
