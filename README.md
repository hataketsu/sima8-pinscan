# Sima X8 flight board — STM32F031K6 teardown

Pin map for the flight controller of a Sima X8 toy drone, recovered by probing
the running silicon rather than by reading firmware. The stock image was not
available to read: it had already been destroyed by the RDP level 1 -> level 0
mass erase that unlocked the chip.

Every row in the pin map is backed by a measurement. The scanner firmware in
this repo is what produced them.

## Board

| Part | Role | Bus |
| --- | --- | --- |
| STM32F031K6 | MCU, LQFP32, Cortex-M0 | — |
| BK2425 | 2.4 GHz radio, nRF24L01+ workalike | SPI |
| MPU6050 | 3-axis gyro + accelerometer | I2C |
| 4x MOSFET | brushed motor low-side switches | GPIO |

| MCU detail | Value |
| --- | --- |
| DEV_ID | `0x444` |
| Flash | `0x08000000`, 32 KiB, 1 KiB pages |
| SRAM | `0x20000000`, 4 KiB |
| Option bytes | `0x1FFFF800` reads `0x00FF55AA` — RDP level 0 |
| Clock at reset | HSI 8 MHz, no PLL |

## Pin map

| Pin | Function | Peripheral | How it was established |
| --- | --- | --- | --- |
| PA1  | LED 1        | TIM2_CH2  | LED lit when the pin was driven high |
| PA2  | Motor M1     | TIM2_CH3  | subset-encoding rounds A/B |
| PA3  | Motor M2     | TIM2_CH4  | subset-encoding rounds A/B |
| PA4  | BK2425 CSN   | SPI1_NSS  | `RX_ADDR_P0` read back `0xE7` |
| PA5  | BK2425 SCK   | SPI1_SCK  | `RX_ADDR_P0` read back `0xE7` |
| PA6  | BK2425 MISO  | SPI1_MISO | `RX_ADDR_P0` read back `0xE7` |
| PA7  | BK2425 MOSI  | SPI1_MOSI | `RX_ADDR_P0` read back `0xE7` |
| PA8  | Motor M3     | TIM1_CH1  | subset-encoding rounds A/B |
| PA9  | UART TX?     | USART1_TX | unconfirmed — left alone, adapter attached |
| PA10 | UART RX?     | USART1_RX | census reads it externally high |
| PA11 | Motor M4     | TIM1_CH4  | driven alone, only M4 conducted |
| PA13 | SWDIO        | —         | debug link, never driven |
| PA14 | SWCLK        | —         | debug link, never driven |
| PB0  | unknown      | TIM3_CH3  | census reads it low; drives no motor, lights no LED |
| PB6  | MPU6050 SCL  | I2C1_SCL  | `WHO_AM_I` = `0x68` at address `0xD0` |
| PB7  | MPU6050 SDA  | I2C1_SDA  | `WHO_AM_I` = `0x68` at address `0xD0` |
| PF0  | unknown      | OSC_IN    | census reads it externally high |

Reading floating and unclaimed: PA0, PA12, PA15, PB1, PB3, PB4, PB5, PB8, PF1.

Both modules land on the pins of the matching hardware peripheral — I2C1 on
PB6/PB7, SPI1 on PA4-PA7, in NSS/SCK/MISO/MOSI order. New firmware can drive
them with the real peripherals; no bit-banging required.

Alternate-function numbers are from the F031 AF table and are worth checking
against the datasheet before they go into a register write.

## Motor drive polarity — read this before writing firmware

The gates are **active low**. Driving a motor pin low makes that motor's
negative pad conduct to ground; releasing the pin or driving it high stops
conduction. Confirmed with a control test: with every pin released, no pad
conducted, which rules out a path that is grounded regardless of the MCU.

Two consequences:

1. PWM has to be inverted — TIM PWM mode 2, or invert the channel through CCER.
2. At reset the pins are floating inputs, so the gates are undefined and the
   motors can spin. `Reset_Handler` must drive PA2, PA3, PA8 and PA11 **high**
   before it does anything else. Keep the props off until that path is proven.

## Sensor check

Phase 6 streams live MPU6050 samples so a host can tell a working sensor from
one that merely acknowledges its address:

| Read | `mpu_seq` | ax | ay | az | temp | gx | gy | gz |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| 1 | 2   | 1392 | -2880 | 16412 | 28.9 C | -203 | 2  | -110 |
| 2 | 55  | 1296 | -2724 | 16384 | 29.3 C | -155 | 45 | -91  |
| 3 | 108 | 1384 | -2712 | 16324 | 29.2 C | -136 | 76 | -95  |

`az` sits at 16384, exactly 1 g on the +-2 g scale, for a board lying flat.
Temperature is room temperature. The gyro dithers around zero. The sensor is
live.

## Getting SWD to attach

A plain attach finds a dead bus: the resident firmware reconfigures PA13/PA14
away from SWD shortly after boot, so by the time a debugger probes, the pins are
GPIO. NRST was never wired on this board, so connect-under-reset was not an
option either.

The way in is BOOT0. It is a dedicated pin on the LQFP32 package, so it needs no
prior access to the chip:

1. Pull BOOT0 high through 10k and power-cycle. The chip enters the ROM
   bootloader and never runs user code, leaving PA13/PA14 as SWD.
2. Attach and flash.
3. Release BOOT0 and reset to run the new firmware.

## The scanner

`pinscan.c` is bare-metal, no HAL, no libgcc — a Cortex-M0 with `-nostdlib`
needs `-fno-jump-tables` and no integer division, both of which this respects.

| Phase | What it does |
| --- | --- |
| 1 | Census: read every pin with pull-up then pull-down. Same both times means an external driver; differing means floating. |
| 2 | I2C: bit-bang every (SCL, SDA) pair, address `0xD0`/`0xD2`, read `WHO_AM_I`. |
| 3 | SPI: bit-bang every (CSN, SCK, MOSI) triple and sample the entire IDR on each clock edge, so MISO identifies itself instead of multiplying the search. |
| 4 | Optional UART broadcast of the report (parked). |
| 6 | Stream MPU6050 samples. |
| 9 | Host-driven pin exercise: drive one pin or a masked group high, low, or blinking. |

Sampling every port each clock in phase 3 is what makes it tractable: searching
for all four SPI pins is ~280k combinations, while searching for three and
letting the fourth fall out of the sampled data is ~12k.

Results land in a struct at a fixed `0x20000000`, so the host reads them without
needing a symbol table. PA13/PA14 are excluded from the candidate list
everywhere, so the scan can never cut the branch it is sitting on.

The LED on PA1 blinks at 2 Hz from the main loop as a heartbeat.

## Build and run

```sh
make                                    # needs arm-none-eabi-gcc
st-flash --reset write pinscan.bin 0x08000000
```

Reading results, and driving pins, over SWD:

```sh
host/decode.py res.bin                  # after dumping 0x20000000
host/drive.sh  0x01 1                   # PA1 high
host/gdrive.sh 0x990D 0x013B 0x0003 5   # drive a masked group low
```

Pin codes are `port << 4 | pin`, with port 0 = A, 1 = B, 2 = F. Drive modes are
1 = high, 2 = low, 3 = blink, 4 = group high, 5 = group low, 6 = MPU stream.

Note that openocd halts the CPU on `init` and leaves it halted on `shutdown`, so
every read must end with `resume` or the firmware freezes mid-scan.

## Still open

- PB0 and PF0 are unexplained.
- PA9/PA10 are assumed to be the UART but were never confirmed; the bit-banged
  UART never produced readable output at any standard baud rate.
- `fw_flash.bin` is a dump of a test image that was on the chip, not the stock
  Sima firmware. The original is gone.
