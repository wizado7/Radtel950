/* Derived from upstream linker.ld; no libc/libm/libgcc references.
 * Conservative application range within the first 256 KiB of flash.
 */
MEMORY {
    FLASH (rx) : ORIGIN = 0x08003000, LENGTH = 244K
    SRAM (rwx) : ORIGIN = 0x20000000, LENGTH = 96K
}
ENTRY(Reset_Handler)
_estack = 0x20017BB0;
SECTIONS {
    .text : {
        KEEP(*(.isr_vector))
        . = ORIGIN(FLASH) + 0x3E0;
        KEEP(*(.model_sig))
        . = ORIGIN(FLASH) + 0x7E0;
        KEEP(*(.boot_model_sig))
        . = ALIGN(4);
        *(.text*) *(.rodata*) *(.glue_7*)
        . = ALIGN(4);
    } > FLASH
    .ARM.extab : { *(.ARM.extab*) } > FLASH
    .ARM.exidx : {
        __exidx_start = .; *(.ARM.exidx*) __exidx_end = .;
    } > FLASH
    _sidata = LOADADDR(.data);
    .data : {
        . = ALIGN(4); _sdata = .; *(.data*) *(.ramfunc*) . = ALIGN(4); _edata = .;
    } > SRAM AT> FLASH
    .bss (NOLOAD) : {
        . = ALIGN(4); _sbss = .; *(.bss*) *(COMMON)
        . = ALIGN(4); _ebss = .;
    } > SRAM
    ASSERT(_ebss + 0x1000 <= _estack, "Insufficient stack reserve")
}
