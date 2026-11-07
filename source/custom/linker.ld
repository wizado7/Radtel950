/* Storage reservation for the custom application only.
 * The diagnostic linker and official image are independent. */
INCLUDE ../diagnostic/linker.ld
_storage_start = 0x08010000;
_storage_slot_size = 0x2000;
_storage_end = _storage_start + 2 * _storage_slot_size;
ASSERT(_sidata + SIZEOF(.data) <= _storage_start, "Application overlaps storage reservation")
ASSERT(_storage_start >= ORIGIN(FLASH) && _storage_end <= ORIGIN(FLASH) + LENGTH(FLASH),
       "Storage reservation outside custom application flash")
ASSERT((_storage_start & 0xFFF) == 0 && (_storage_slot_size & 0xFFF) == 0,
       "Storage slot alignment")
