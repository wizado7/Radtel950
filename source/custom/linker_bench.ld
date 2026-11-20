/* Derived from upstream linker.ld; no libc/libm/libgcc references.
 * Conservative application range within the first 256 KiB of flash.
 */
MEMORY {
    HEADER (rx) : ORIGIN = 0x08003000, LENGTH = 4K
    FLASH (rx) : ORIGIN = 0x08014000, LENGTH = 176K
    SRAM (rwx) : ORIGIN = 0x20000000, LENGTH = 96K
}
ENTRY(Reset_Handler)
_estack = 0x20017BB0;
SECTIONS {
    .header : {
        KEEP(*(.isr_vector))
        *(.text.*rf_select*) *(.text.*rf_transmitting*) *(.text.board_tx_budget_ok)
        *(.text.*release_board_pa_bias*) *(.text.*prepare_board_pa_bias*) *(.text.*clear_board_pa_bias*)
        *(.text.board_battery_service_fast) *(.text.board_battery_fast_sample)
        *(.text.board_battery_init) *(.text.board_battery_fast)
        *(.text.board_battery_idle) *(.text.board_battery_deadline)
        . = ORIGIN(HEADER) + 0x3E0;
        KEEP(*(.model_sig))
        /* Place small safety routines in existing header padding. */
        *(.text.board_rf_quiet) *(.text.board_tx_timer) *(.text.board_tx_watch)
        *(.text.fault_hold) *(.text.read_reset_cause) *(.text.board_idle_pins*)
        *(.text.board_battery_percent_from_mv)
        . = ORIGIN(HEADER) + 0x7E0;
        KEEP(*(.boot_model_sig))
        . = ALIGN(4);
    } > HEADER
    .text : {
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
    .noinit (NOLOAD) : {
        *(.noinit.watchdog)
    } > SRAM
    ASSERT(ADDR(.noinit) + SIZEOF(.noinit) + 0x1000 <= _estack, "Insufficient stack reserve")
}

_storage_start = 0x08010000;
_storage_slot_size = 0x2000;
_storage_end = _storage_start + 2 * _storage_slot_size;
ASSERT(ORIGIN(FLASH) == _storage_end, "Bench code must follow both storage slots")
ASSERT(ADDR(.header) + SIZEOF(.header) <= _storage_start, "Header overlaps storage")
