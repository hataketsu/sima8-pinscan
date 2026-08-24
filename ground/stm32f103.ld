MEMORY
{
  /* This board reports device ID 0x0412 and a flash-size register of 32 KB,
   * and reads past 0x08008000 fail: it is a low-density F103, not the C8 that
   * "Blue Pill" usually implies. Sizing this at the usual 64K/20K puts the
   * initial stack pointer outside real RAM and the chip faults on the first
   * instruction, which looks exactly like a failed flash. */
  /* The HID bootloader reserves 2 KB and starts the application at 0x08000800
   * (its own BOOTLOADER_SIZE), leaving 30 KB of the 32 KB flash.
   * RAM is 10 KB: this is a low-density part, not the 20 KB C8. */
  FLASH (rx) : ORIGIN = 0x08000800, LENGTH = 30K
  RAM  (rwx) : ORIGIN = 0x20000000, LENGTH = 10K
}
_estack = ORIGIN(RAM) + LENGTH(RAM);
SECTIONS
{
  .isr_vector : { KEEP(*(.isr_vector)) } > FLASH
  .text   : { *(.text*) *(.rodata*) . = ALIGN(4); } > FLASH
  _sidata = LOADADDR(.data);
  .data : { _sdata = .; *(.data*) . = ALIGN(4); _edata = .; } > RAM AT > FLASH
  .bss  : { _sbss = .; *(.bss*) *(COMMON) . = ALIGN(4); _ebss = .; } > RAM
}
