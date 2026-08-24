MEMORY
{
  /* This board reports device ID 0x0412 and a flash-size register of 32 KB,
   * and reads past 0x08008000 fail: it is a low-density F103, not the C8 that
   * "Blue Pill" usually implies. Sizing this at the usual 64K/20K puts the
   * initial stack pointer outside real RAM and the chip faults on the first
   * instruction, which looks exactly like a failed flash. */
  /* The STM32duino DFU bootloader occupies the first 8 KB and hands control to
   * an application at 0x08002000, so the app is linked there and gets the
   * remaining 24 KB. */
  FLASH (rx) : ORIGIN = 0x08002000, LENGTH = 24K
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
